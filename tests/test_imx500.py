"""Tests for the IMX500 (Raspberry Pi AI Camera) detection backend.

Everything here runs on a laptop with no camera, no picamera2 and no sensor: the
parts worth testing are the coordinate maths, the backend choice, and the
staleness contract, and all three are pure logic reachable with fakes. The one
thing genuinely untestable off-hardware — decoding a real output tensor — WAS
isolated in `Decoder.parse()` behind picamera2 imports, and everything AROUND it
was exercised here.

That is no longer true for our own YOLO export. Its layout (four tensors, NMS
already run on the sensor) is decoded without touching picamera2 at all, so the
decode is now covered here too, fakes and all — see the edge-mdt NMS section at
the bottom. That decode is what turns sensor output into the boxes object_align
steers on, so it is worth far more than an eyeball on a rover.

Worth testing rather than eyeballing on a rover:

  * `resolve_backend` must NOT silently switch an existing Edge Impulse rover to
    the AI Camera, and must repoint the camera when it does pick the IMX500 —
    a wrong answer means the rover boots blind, on the wrong model, in a field.
  * Staleness. The IMX500 detector holds the same "don't clear on a miss, let it
    age out" contract that makes a dead detector thread fail safe. Bugs here
    look like "the robot lurched at nothing" or "it kept chasing a target that
    left" — both expensive to reproduce and trivial to assert.

    pytest tests/
"""

from __future__ import annotations

import time

import pytest

from robot.config import RobotConfig
from robot.sensors.imx500 import (IMX500Detector, load_labels, resolve_backend,
                                  select_box, to_detection)


# --- to_detection: pixels -> the controller's normalized units ---------------

def test_centered_box_has_zero_error():
    # 640x480 frame, a 40x80 box centered on both axes.
    d = to_detection((300, 200, 40, 80, "cone", 0.9), 640, 480, 1.0)
    assert d.error_x == pytest.approx(0.0)
    assert d.error_y == pytest.approx(0.0)


def test_error_x_signs_match_the_contract():
    """+1 is the RIGHT edge — get this backwards and the robot steers away."""
    right = to_detection((600, 200, 40, 80, "cone", 0.9), 640, 480, 1.0)
    left = to_detection((0, 200, 40, 80, "cone", 0.9), 640, 480, 1.0)
    assert right.error_x > 0
    assert left.error_x < 0


def test_error_is_clamped_to_unit_range():
    # A box hanging off the right edge still reports at most +1.
    d = to_detection((630, 470, 100, 100, "cone", 0.9), 640, 480, 1.0)
    assert -1.0 <= d.error_x <= 1.0
    assert -1.0 <= d.error_y <= 1.0


def test_size_is_box_height_over_frame_height():
    """Height, not area — the range proxy in control/detection.py."""
    d = to_detection((100, 100, 200, 120, "cone", 0.9), 640, 480, 1.0)
    assert d.size == pytest.approx(120 / 480)


def test_size_is_always_available():
    """Unlike Edge Impulse FOMO, the IMX500 zoo gives real boxes, so
    object_align can always approach and stand off."""
    d = to_detection((10, 10, 20, 30, "cone", 0.5), 640, 480, 1.0)
    assert d.size is not None


# --- select_box: which candidate the controller steers on --------------------

BOXES = [
    (0, 0, 100, 100, "a", 0.7),      # largest
    (300, 200, 20, 20, "b", 0.95),   # most confident, and nearest center
    (600, 400, 40, 40, "c", 0.8),
]


def test_select_largest_picks_by_area():
    assert select_box(BOXES, "largest", 640)[4] == "a"


def test_select_confidence_picks_the_top_score():
    assert select_box(BOXES, "confidence", 640)[4] == "b"


def test_select_centermost_prefers_what_we_already_face():
    assert select_box(BOXES, "centermost", 640)[4] == "b"


def test_select_of_nothing_is_none():
    assert select_box([], "largest", 640) is None


# --- resolve_backend ---------------------------------------------------------

def test_explicit_imx500_repoints_the_camera():
    """Choosing on-sensor detection is choosing the capture backend too — the
    detector is useless against a camera that never loaded the network."""
    cfg = RobotConfig()
    cfg.vision.backend = "imx500"
    cfg.camera.device = "auto"
    assert resolve_backend(cfg.vision, cfg.camera) == "imx500"
    assert cfg.camera.device == "imx500"


def test_explicit_edge_impulse_leaves_the_camera_alone():
    cfg = RobotConfig()
    cfg.vision.backend = "edge_impulse"
    cfg.camera.device = "/dev/video0"
    assert resolve_backend(cfg.vision, cfg.camera) == "edge_impulse"
    assert cfg.camera.device == "/dev/video0"


def test_auto_falls_back_to_edge_impulse_without_an_ai_camera():
    """The fallback every rover in the field is already running. An 'auto' that
    guessed imx500 here would swap the model out from under someone."""
    cfg = RobotConfig()
    cfg.vision.backend = "auto"
    assert resolve_backend(cfg.vision, cfg.camera) == "edge_impulse"
    assert cfg.camera.device == "auto"


def test_auto_needs_the_network_file_too(monkeypatch, tmp_path):
    """An attached camera with no .rpk on disk can't do on-sensor inference."""
    cfg = RobotConfig()
    cfg.vision.backend = "auto"
    monkeypatch.setattr("robot.sensors.imx500.imx500_present", lambda: True)

    cfg.vision.imx500_model = str(tmp_path / "missing.rpk")
    assert resolve_backend(cfg.vision, cfg.camera) == "edge_impulse"

    net = tmp_path / "net.rpk"
    net.write_bytes(b"stub")
    cfg.vision.imx500_model = str(net)
    assert resolve_backend(cfg.vision, cfg.camera) == "imx500"


def test_unknown_backend_falls_back_rather_than_raising():
    """A typo in robot.env must not stop the rover booting."""
    cfg = RobotConfig()
    cfg.vision.backend = "imx-500"  # not a valid name
    assert resolve_backend(cfg.vision, cfg.camera) == "edge_impulse"


# --- load_labels -------------------------------------------------------------

class _Intrinsics:
    def __init__(self, labels, ignore_dash_labels=False):
        self.labels = labels
        self.ignore_dash_labels = ignore_dash_labels


def test_labels_come_from_the_network_by_default():
    assert load_labels(_Intrinsics(["cat", "dog"])) == ["cat", "dog"]


def test_dash_labels_are_dropped_when_the_network_says_so():
    intr = _Intrinsics(["cat", "-", "", "dog"], ignore_dash_labels=True)
    assert load_labels(intr) == ["cat", "dog"]


def test_a_labels_file_overrides_the_network(tmp_path):
    f = tmp_path / "labels.txt"
    f.write_text("cone\nbarrel\n")
    assert load_labels(_Intrinsics(["cat"]), str(f)) == ["cone", "barrel"]


# --- the cache + staleness contract ------------------------------------------

def make_detector(**kw):
    """A detector with its thread never started — we drive _consume() directly,
    which is exactly the seam between 'decode a frame' and 'what the controller
    sees'."""
    cfg = RobotConfig().vision
    for k, v in kw.items():
        setattr(cfg, k, v)
    return IMX500Detector(cfg, camera=None)


def test_detection_is_none_before_anything_is_seen():
    assert make_detector().detection() is None


def test_a_consumed_box_becomes_the_detection():
    d = make_detector()
    d._consume([(300, 200, 40, 80, "cone", 0.9)], 640, 480, time.monotonic())
    got = d.detection()
    assert got is not None and got.label == "cone"


def test_an_empty_frame_does_not_clear_the_cache():
    """The single most load-bearing behaviour in this file: one dropped frame
    must coast on the last detection, not lurch to a stop."""
    d = make_detector()
    d._consume([(300, 200, 40, 80, "cone", 0.9)], 640, 480, time.monotonic())
    d._consume([], 640, 480, time.monotonic())
    assert d.detection() is not None


def test_a_stale_detection_ages_out():
    """And this is why a dead detector thread fails safe: no fresh stamps ->
    the target ages out -> the controller stops."""
    d = make_detector(target_timeout=0.5)
    d._consume([(300, 200, 40, 80, "cone", 0.9)], 640, 480, time.monotonic() - 5.0)
    assert d.detection() is None
    assert d.overlays() == []


def test_overlays_mark_exactly_one_target():
    d = make_detector(select="largest")
    d._consume(BOXES, 640, 480, time.monotonic())
    overlays = d.overlays()
    assert len(overlays) == 3
    assert [o[6] for o in overlays].count(True) == 1
    assert next(o for o in overlays if o[6])[4] == "a"  # the largest


def test_telemetry_stays_small_and_names_the_backend():
    """It rides a shared 57600-baud radio — a summary, never boxes or frames."""
    d = make_detector()
    d._consume([(300, 200, 40, 80, "cone", 0.9)], 640, 480, time.monotonic())
    t = d.telemetry()
    assert t["imx500"] is True
    assert t["label"] == "cone"
    assert set(t) <= {"ok", "fps", "imx500", "label", "conf", "ex", "size", "age"}


def test_start_without_a_camera_is_inert_not_fatal():
    """Vision failing must cost us vision and nothing else — the rover still
    has to arm its ESCs and answer the radio."""
    d = make_detector()
    d.start()
    assert d.detection() is None
    assert d.telemetry()["ok"] is False
    d.stop()


# --- the edge-mdt NMS layout: our own YOLO export ----------------------------
#
# The sensor runs NMS itself and emits four tensors. These tests pin down the
# contract established by reading edge-mdt-cl's multiclass_nms_with_indices and
# Ultralytics' IMX exporter:
#
#   boxes   (max_det, 4)  (x_min, y_min, x_max, y_max) in NETWORK INPUT pixels
#   scores  (max_det,)    descending
#   labels  (max_det,)    class index
#   n_valid (1,)          real detections; everything after is zero padding
#
# Getting any of this wrong is not a crash, it is a rover that steers at boxes
# in the wrong place — which is exactly why it is asserted rather than eyeballed.

import numpy as np

from robot.sensors.imx500 import Decoder, unpack_edgemdt_nms

FRAME_W, FRAME_H = 640, 480
NET = 640  # network input is square 640x640


def _nms_outputs(boxes, scores, labels, n_valid, max_det=8, reverse=False):
    """Build the four sensor tensors, zero-padded to max_det like the real thing."""
    b = np.zeros((max_det, 4), np.float32)
    s = np.zeros((max_det,), np.float32)
    lb = np.zeros((max_det,), np.float32)
    if len(boxes):          # `b[:0] = []` cannot broadcast into shape (0, 4)
        b[:len(boxes)] = boxes
        s[:len(scores)] = scores
        lb[:len(labels)] = labels
    # add_batch=True, so picamera2 hands each tensor back with a leading 1.
    out = [b[None], s[None], lb[None], np.array([[n_valid]], np.float32)]
    return out[::-1] if reverse else out


class _FakeIMX500:
    def __init__(self, outputs):
        self._outputs = outputs

    def get_outputs(self, metadata, add_batch=True):
        return self._outputs

    def get_input_size(self):
        return (NET, NET)

    def convert_inference_coords(self, coords, metadata, picam2):
        """Stand-in for picamera2: normalized (y0, x0, y1, x1) -> frame pixels."""
        y0, x0, y1, x1 = (float(np.asarray(c).reshape(-1)[0]) for c in coords)
        return (x0 * FRAME_W, y0 * FRAME_H,
                (x1 - x0) * FRAME_W, (y1 - y0) * FRAME_H)


class _FakeIntrinsics:
    labels = ["ball", "blue bucket", "orange bucket"]
    ignore_dash_labels = False
    postprocess = ""


class _FakeSource:
    def __init__(self, outputs):
        self.imx500 = _FakeIMX500(outputs)
        self.intrinsics = _FakeIntrinsics()
        self.picam2 = object()


def _decoder(outputs, **cfg_over):
    cfg = RobotConfig().vision
    cfg.min_confidence = cfg_over.pop("min_confidence", 0.5)
    cfg.target_label = cfg_over.pop("target_label", "")
    cfg.imx500_max_detections = cfg_over.pop("max_detections", 10)
    cfg.imx500_labels = ""
    return Decoder(_FakeSource(outputs), cfg)


# --- unpack: shape-based identification --------------------------------------

def test_unpack_reads_the_declared_order():
    out = _nms_outputs([[10, 20, 30, 40]], [0.9], [2], 1)
    boxes, scores, labels, n = unpack_edgemdt_nms(out)
    assert n == 1
    assert boxes.tolist() == [[10, 20, 30, 40]]
    assert scores.tolist() == pytest.approx([0.9])
    assert labels.tolist() == [2]


def test_unpack_reads_the_reversed_order():
    """dnnParams.xml lists these tensors opposite to the ONNX graph, and which
    order picamera2 follows is only observable on a Pi — so both must work."""
    out = _nms_outputs([[10, 20, 30, 40]], [0.9], [2], 1, reverse=True)
    boxes, scores, labels, n = unpack_edgemdt_nms(out)
    assert n == 1
    assert boxes.tolist() == [[10, 20, 30, 40]]
    assert scores.tolist() == pytest.approx([0.9])
    assert labels.tolist() == [2]


def test_unpack_drops_the_zero_padding():
    """Without honouring n_valid, every frame decodes 300 degenerate boxes at
    the origin and 'centermost' selection locks onto the padding forever."""
    out = _nms_outputs([[10, 20, 30, 40], [50, 60, 70, 80]],
                       [0.9, 0.8], [0, 1], n_valid=2, max_det=64)
    boxes, scores, labels, n = unpack_edgemdt_nms(out)
    assert n == 2 and len(boxes) == 2 and len(scores) == 2 and len(labels) == 2


def test_unpack_clamps_a_bogus_count():
    out = _nms_outputs([[1, 2, 3, 4]], [0.9], [0], n_valid=9999, max_det=8)
    _, scores, _, n = unpack_edgemdt_nms(out)
    assert n == 8 and len(scores) == 8


def test_unpack_rejects_other_layouts():
    assert unpack_edgemdt_nms(None) is None
    # model-zoo _pp networks: three tensors, not four
    assert unpack_edgemdt_nms([np.zeros((1, 4, 4)), np.zeros((1, 4)),
                               np.zeros((1, 4))]) is None
    # four tensors but no single-element count -> not this layout
    assert unpack_edgemdt_nms([np.zeros((1, 4, 4))] * 4) is None


# --- parse: sensor tensors -> full-frame pixel boxes -------------------------

def test_parse_maps_boxes_into_frame_pixels():
    """A box spanning the middle half of the network input must land on the
    middle half of the frame — this is the whole coordinate contract."""
    out = _nms_outputs([[160, 120, 480, 360]], [0.9], [1], 1)
    boxes = _decoder(out).parse({}, FRAME_W, FRAME_H)
    assert len(boxes) == 1
    x, y, w, h, label, conf = boxes[0]
    # 160/640 = 0.25 of the width; 120/640 = 0.1875 of the height.
    assert (x, w) == (160, 320)
    assert (y, h) == (90, 180)
    assert label == "blue bucket"
    assert conf == pytest.approx(0.9)


def test_parse_filters_below_confidence():
    out = _nms_outputs([[10, 10, 100, 100], [20, 20, 200, 200]],
                       [0.9, 0.3], [0, 0], 2)
    boxes = _decoder(out, min_confidence=0.5).parse({}, FRAME_W, FRAME_H)
    assert len(boxes) == 1 and boxes[0][5] == pytest.approx(0.9)


def test_parse_filters_by_target_label():
    out = _nms_outputs([[10, 10, 100, 100], [20, 20, 200, 200]],
                       [0.9, 0.8], [0, 2], 2)
    boxes = _decoder(out, target_label="orange bucket").parse({}, FRAME_W, FRAME_H)
    assert len(boxes) == 1 and boxes[0][4] == "orange bucket"


def test_parse_returns_nothing_when_the_sensor_saw_nothing():
    """n_valid == 0 must yield no boxes, NOT 300 zero-area ones."""
    out = _nms_outputs([], [], [], n_valid=0, max_det=32)
    assert _decoder(out).parse({}, FRAME_W, FRAME_H) == []


def test_parse_honours_the_detection_cap():
    n = 12
    out = _nms_outputs([[10, 10, 100, 100]] * n, [0.9] * n, [0] * n, n, max_det=32)
    boxes = _decoder(out, max_detections=5).parse({}, FRAME_W, FRAME_H)
    assert len(boxes) == 5


def test_parse_gives_object_align_a_real_size():
    """The FOMO degradation (no size -> cannot approach) must not apply here:
    a taller box must report a proportionally larger size."""
    small = _nms_outputs([[300, 300, 340, 340]], [0.9], [0], 1)
    big = _nms_outputs([[100, 100, 540, 540]], [0.9], [0], 1)
    ds = to_detection(_decoder(small).parse({}, FRAME_W, FRAME_H)[0],
                      FRAME_W, FRAME_H, 1.0)
    db = to_detection(_decoder(big).parse({}, FRAME_W, FRAME_H)[0],
                      FRAME_W, FRAME_H, 1.0)
    assert ds.size is not None and db.size is not None
    assert db.size > ds.size
