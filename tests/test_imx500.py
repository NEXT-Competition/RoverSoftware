"""Tests for the IMX500 (Raspberry Pi AI Camera) detection backend.

Everything here runs on a laptop with no camera, no picamera2 and no sensor: the
parts worth testing are the coordinate maths, the backend choice, and the
staleness contract, and all three are pure logic reachable with fakes. The one
thing genuinely untestable off-hardware — decoding a real output tensor — is
isolated in `Decoder.parse()` behind picamera2 imports, and everything AROUND it
is exercised here.

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
