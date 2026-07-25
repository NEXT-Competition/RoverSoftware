"""Sony IMX500 (Raspberry Pi AI Camera) object detection — the other set of eyes.

Interchangeable with `sensors/detector.ObjectDetector`: same constructor shape,
same `start()/stop()/detection()/overlays()/telemetry()`, same staleness
semantics. `object_align` cannot tell which one it's wired to, which is the
point — pick a backend in config and the autonomy stack is unchanged.

What IS different is where the work happens. Edge Impulse reads frames out of
the camera and runs a model on the Pi's CPU. The IMX500 runs the network inside
the sensor and ships the RESULT out with each frame, as libcamera metadata:

    Edge Impulse : sensor -> frame -> CPU model (50-200ms, one core) -> boxes
    IMX500       : sensor -> [network on-sensor] -> frame + boxes together

So this class opens nothing and infers nothing. It polls the shared Camera for
(frame, metadata) pairs and decodes the tensor the sensor already produced. The
Pi cost is a tensor decode — well under a millisecond — instead of a core, and
`RS_VISION_FPS` no longer buys back CPU (the sensor infers at its own rate
regardless), it only limits how often we decode.

--- Staleness is still the whole safety story ---
Unchanged from detector.py and load-bearing: on a frame with no target we do NOT
clear the cache, we stop advancing the stamp, and `detection()` ages the sample
out after `target_timeout`. A dead thread therefore fails safe by construction.

--- Coordinates ---
Boxes come out of the sensor in network input space, letterboxed or cropped
relative to what the frame shows. `IMX500.convert_inference_coords()` undoes
that against the same request's metadata, giving full-frame pixels — so unlike
the Edge Impulse path there is no center-crop to compensate for, and
`VisionConfig.hfov_deg` must be the camera's REAL horizontal FOV (~66 deg for
the AI Camera's stock lens), not a post-crop value.
"""

from __future__ import annotations

import os
import threading
import time
from typing import List, Optional, Tuple

from ..control.detection import Detection
from .camera import imx500_present


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def resolve_backend(vision, camera) -> str:
    """Decide which detection backend to run: "imx500" or "edge_impulse".

    Also POINTS THE CAMERA AT IT — an IMX500 detector is useless unless the
    capture backend is the one that loaded the network, so the two decisions are
    really one and are made here, once, before anything opens a device. Callers
    (Robot, the selftest tool) just use the returned name.

    "auto" only picks the AI Camera when one is actually attached AND its
    network file exists, because the fallback — Edge Impulse — is what every
    existing rover in the field is running, and an upgrade must not silently
    change which model is driving the robot.
    """
    want = (vision.backend or "auto").strip().lower()
    if want in ("imx500", "ai-camera", "aicamera", "sony"):
        backend = "imx500"
    elif want in ("edge_impulse", "edge-impulse", "ei", "eim"):
        backend = "edge_impulse"
    elif want == "auto":
        have_net = bool(vision.imx500_model) and os.path.exists(
            os.path.expanduser(vision.imx500_model))
        backend = "imx500" if (have_net and imx500_present()) else "edge_impulse"
    else:
        print(f"[vision] unknown backend {vision.backend!r} — using edge_impulse. "
              "Valid: auto | edge_impulse | imx500")
        backend = "edge_impulse"

    if backend == "imx500":
        if str(camera.device or "").lower() not in ("imx500", "ai-camera", "aicamera"):
            camera.device = "imx500"
        print(f"[vision] backend=imx500 (on-sensor) network="
              f"{os.path.basename(vision.imx500_model)}")
    else:
        print(f"[vision] backend=edge_impulse (on-CPU) model={vision.model_path}")
    return backend


# One detection, already mapped to full-frame pixels:
# (x, y, w, h, label, confidence).
Box = Tuple[int, int, int, int, str, float]


def select_box(boxes: List[Box], mode: str, frame_w: int) -> Optional[Box]:
    """Pick one box out of the candidates — the detector.py policy, in pixels.

    Greedy and per-frame: no tracking, so two similar objects can make 'largest'
    flap between frames. 'centermost' is the mitigation — it prefers the one
    we're already facing.
    """
    if not boxes:
        return None
    if mode == "confidence":
        return max(boxes, key=lambda b: b[5])
    if mode == "centermost":
        return min(boxes, key=lambda b: abs((b[0] + b[2] / 2.0) - frame_w / 2.0))
    return max(boxes, key=lambda b: b[2] * b[3])


def to_detection(box: Box, frame_w: int, frame_h: int, stamp: float) -> Detection:
    """Full-frame pixel box -> the controller's normalized units.

    Normalized against the FRAME, not a model input: convert_inference_coords()
    has already mapped the sensor's letterboxed network space back to frame
    pixels, so the frame is the only space left. `size` is box height over frame
    height, matching Detection's contract (height, not area — see
    control/detection.py).
    """
    x, y, w, h, label, conf = box
    cx = x + w / 2.0
    cy = y + h / 2.0
    return Detection(
        label=label,
        confidence=float(conf),
        error_x=_clamp(2.0 * cx / frame_w - 1.0, -1.0, 1.0) if frame_w else 0.0,
        error_y=_clamp(2.0 * cy / frame_h - 1.0, -1.0, 1.0) if frame_h else 0.0,
        # Always available here: the IMX500 model zoo is real bounding-box
        # detectors, so there is no FOMO-style "no size" degradation.
        size=_clamp(h / frame_h, 0.0, 1.0) if frame_h else None,
        stamp=stamp,
    )


def load_labels(intrinsics, labels_path: str = "") -> List[str]:
    """Class labels for the loaded network.

    Model-zoo `.rpk`s embed their labels; a custom export may not, hence the
    optional file override. Dash entries are placeholder classes the network
    emits but never means (COCO's gaps), and dropping them keeps class indices
    aligned only because the sensor's indices are into the FILTERED list — this
    mirrors picamera2's own demo exactly.
    """
    if labels_path:
        path = os.path.expanduser(labels_path)
        with open(path) as f:
            labels = [ln.strip() for ln in f]
    else:
        labels = list(getattr(intrinsics, "labels", None) or [])
    if getattr(intrinsics, "ignore_dash_labels", False):
        labels = [ln for ln in labels if ln and ln != "-"]
    return labels


class Decoder:
    """One frame's libcamera metadata -> full-frame pixel boxes above threshold.

    Split out of IMX500Detector so the bring-up tool (tools/detector_selftest.py)
    drives the EXACT decode the rover uses, rather than a lookalike that can
    quietly disagree with it about coordinates or labels.

    Built from an open `_IMX500Source`, because everything it needs — the sensor
    handle, the network's intrinsics, the picamera2 object whose crop the
    coordinates are relative to — comes from the camera that loaded the network.
    """

    def __init__(self, source, cfg):
        self.imx500 = source.imx500
        self.intrinsics = source.intrinsics
        self.picam2 = source.picam2
        self.cfg = cfg
        try:
            self.labels = load_labels(self.intrinsics, cfg.imx500_labels)
        except Exception as e:
            print(f"[imx500] could not read labels ({e}) — boxes will be unlabelled")
            self.labels = []

    def describe(self) -> str:
        iw, ih = self.imx500.get_input_size()
        task = getattr(self.intrinsics, "task", "?")
        post = getattr(self.intrinsics, "postprocess", "") or "ssd-style"
        return (f"input={iw}x{ih} task={task} post={post} "
                f"labels={len(self.labels)}")

    def label_for(self, category) -> str:
        try:
            return self.labels[int(category)]
        except Exception:
            return str(int(category)) if self.labels else ""

    def parse(self, metadata: dict, frame_w: int, frame_h: int) -> List[Box]:
        """Sensor output tensor -> full-frame pixel boxes above the threshold.

        Two output layouts, both from picamera2's own IMX500 demo: nanodet needs
        NMS on the Pi, everything else (SSD/YOLO-style `_pp` networks) arrives
        already post-processed as parallel boxes/scores/classes tensors.
        """
        import numpy as np

        outputs = self.imx500.get_outputs(metadata, add_batch=True)
        if outputs is None:
            # No inference attached to this frame — normal right after start.
            return []
        input_w, input_h = self.imx500.get_input_size()
        threshold = self.cfg.min_confidence

        if getattr(self.intrinsics, "postprocess", "") == "nanodet":
            from picamera2.devices.imx500 import postprocess_nanodet_detection
            from picamera2.devices.imx500.postprocess import scale_boxes

            boxes, scores, classes = postprocess_nanodet_detection(
                outputs=outputs[0], conf=threshold,
                iou_thres=self.cfg.imx500_iou,
                max_out_dets=self.cfg.imx500_max_detections)[0]
            boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
        else:
            boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
            if getattr(self.intrinsics, "bbox_normalization", False):
                boxes = boxes / input_h
            if getattr(self.intrinsics, "bbox_order", "") == "xy":
                boxes = boxes[:, [1, 0, 3, 2]]
            # -> an iterable of 4-tuples of 1-element arrays, which is the shape
            # convert_inference_coords() expects (y0, x0, y1, x1).
            boxes = zip(*np.array_split(boxes, 4, axis=1))

        out: List[Box] = []
        for coords, score, category in zip(boxes, scores, classes):
            if score < threshold:
                continue
            label = self.label_for(category)
            if self.cfg.target_label and label != self.cfg.target_label:
                continue
            x, y, w, h = self.imx500.convert_inference_coords(
                coords, metadata, self.picam2)
            # Clip: a box can extend past the frame, since the network sees a
            # letterboxed/cropped field the ISP output doesn't exactly cover.
            x = int(_clamp(x, 0, max(frame_w - 1, 0)))
            y = int(_clamp(y, 0, max(frame_h - 1, 0)))
            w = int(_clamp(w, 0, frame_w - x))
            h = int(_clamp(h, 0, frame_h - y))
            if w <= 0 or h <= 0:
                continue
            out.append((x, y, w, h, label, float(score)))
            if len(out) >= self.cfg.imx500_max_detections:
                break
        return out


class IMX500Detector:
    """On-sensor detection, cached for the control loop. See module docstring."""

    def __init__(self, cfg, camera):
        self.cfg = cfg
        # Shared, and the OWNER of the IMX500 handle — this class never opens a
        # device. Both the FPV streamer and this read from it.
        self.camera = camera
        self._period = 1.0 / cfg.max_fps if cfg.max_fps > 0 else 0.1

        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        # Latest cached state (guarded by _lock).
        self._detection: Optional[Detection] = None
        self._overlays: list = []
        self._overlays_stamp = 0.0
        self._ok = False
        self._fps = 0.0

        # Set once the camera thread has the sensor up.
        self._decoder: Optional[Decoder] = None

    # --- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        """Check the cheap failure modes, then hand off to the thread.

        Nothing expensive happens here for the same reason as detector.py: the
        network upload (tens of seconds on a cold sensor) belongs to the camera
        thread. A camera that never comes up must cost us vision and nothing
        else — the rover still arms its ESCs and answers the radio.
        """
        if not self.cfg.enabled:
            return
        if self.camera is None:
            print("[imx500] camera disabled — object detection disabled "
                  "(object_align will hold still)")
            return
        model = os.path.expanduser(self.cfg.imx500_model or "")
        if not model or not os.path.exists(model):
            print(f"[imx500] no network at {model or '(unset)'} — object detection "
                  "disabled (object_align will hold still). Install the model zoo: "
                  "sudo apt install imx500-all")
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="imx500-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        # The camera owns the sensor handle; closing it here would yank the
        # device out from under the FPV streamer.

    # --- the thread --------------------------------------------------------

    def _await_sensor(self, timeout: float = 120.0) -> bool:
        """Block until the camera thread has the AI Camera open. False = give up.

        The timeout is generous because a cold sensor spends tens of seconds
        accepting the network firmware over CSI, and a rover that boots with a
        slow camera should still end up with working vision.
        """
        deadline = time.monotonic() + timeout
        warned = False
        while self._running and time.monotonic() < deadline:
            source = self.camera.source()
            if source is not None:
                if getattr(source, "imx500", None) is None:
                    print("[imx500] the camera opened a non-IMX500 backend "
                          f"({type(source).__name__}) — on-sensor detection disabled. "
                          "Set RS_CAMERA_DEVICE=imx500 and check the ribbon.")
                    return False
                self._decoder = Decoder(source, self.cfg)
                return True
            if not warned:
                print("[imx500] waiting for the AI Camera (first boot uploads the "
                      "network to the sensor; this can take a while)...")
                warned = True
            time.sleep(0.25)
        if self._running:
            print(f"[imx500] camera did not come up within {timeout:g}s — "
                  "object detection disabled (object_align will hold still)")
        return False

    def _describe_model(self) -> None:
        assert self._decoder is not None
        dec = self._decoder
        net = os.path.basename(self.cfg.imx500_model)
        print(f"[imx500] network '{net}' {dec.describe()}")
        task = getattr(dec.intrinsics, "task", "?")
        if task != "object detection":
            print(f"[imx500] WARNING: this network's task is {task!r}, not object "
                  "detection — it will not produce boxes and object_align will "
                  "hold still. Use a detection .rpk.")
        if self.cfg.target_label and dec.labels and self.cfg.target_label not in dec.labels:
            print(f"[imx500] WARNING: target_label {self.cfg.target_label!r} is not one "
                  f"of this network's labels — nothing will ever match. "
                  f"First few: {dec.labels[:8]}")
        print(f"[imx500] decoding results at up to {self.cfg.max_fps:g} fps "
              "(inference itself runs on the sensor)")

    def _read_loop(self) -> None:
        if not self._await_sensor():
            self._running = False
            return
        self._describe_model()
        with self._lock:
            self._ok = True

        errors = 0
        last_stamp = -1.0
        while self._running:
            t0 = time.monotonic()
            try:
                frame, meta, stamp = self.camera.frame_meta_and_stamp()
                if frame is None or meta is None or stamp == last_stamp:
                    time.sleep(0.01)
                    continue
                last_stamp = stamp
                h, w = frame.shape[0], frame.shape[1]
                boxes = self._decoder.parse(meta, w, h)  # type: ignore[union-attr]
                errors = 0
            except Exception as e:
                # If the sensor has wedged, EVERY iteration raises; an unguarded
                # print would flood the journal at frame rate.
                errors += 1
                if errors == 1 or errors % 50 == 0:
                    print(f"[imx500] decode error (x{errors}): {e}")
                time.sleep(0.5)
                continue

            self._consume(boxes, w, h, time.monotonic())

            elapsed = time.monotonic() - t0
            with self._lock:
                self._fps = (1.0 / elapsed) if elapsed > 0 else 0.0
            sleep_for = self._period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

        with self._lock:
            self._ok = False

    def _consume(self, boxes: List[Box], frame_w: int, frame_h: int, stamp: float) -> None:
        """Fold one frame's boxes into the cached detection + FPV overlays."""
        best = select_box(boxes, self.cfg.select, frame_w)
        if best is None:
            # Deliberately do NOT clear — let the sample age out. See docstring.
            return
        detection = to_detection(best, frame_w, frame_h, stamp)
        overlays = [(*b[:4], b[4], b[5], b is best) for b in boxes]
        with self._lock:
            self._detection = detection
            self._overlays = overlays
            self._overlays_stamp = stamp

    # --- accessors (cheap; safe to call every control tick) ----------------

    def detection(self) -> Optional[Detection]:
        """Latest detection, or None if nothing is currently seen (or it's stale)."""
        with self._lock:
            d = self._detection
            if d is None:
                return None
            if (time.monotonic() - d.stamp) > self.cfg.target_timeout:
                return None
            return d

    def overlays(self) -> list:
        """Detection boxes in full-frame pixels for the FPV overlay, as
        (x, y, w, h, label, conf, is_target). Empty once stale."""
        with self._lock:
            if not self._overlays:
                return []
            if (time.monotonic() - self._overlays_stamp) > self.cfg.target_timeout:
                return []
            return self._overlays

    def telemetry(self) -> dict:
        """Compact status for the base station — a summary, never boxes or frames."""
        with self._lock:
            ok, fps, d = self._ok, self._fps, self._detection
        t = {"ok": ok, "fps": round(fps, 1), "imx500": True}
        if d is not None:
            age = time.monotonic() - d.stamp
            if age <= self.cfg.target_timeout:
                t.update(
                    label=d.label,
                    conf=round(d.confidence, 2),
                    ex=round(d.error_x, 3),
                    size=round(d.size, 3) if d.size is not None else None,
                    age=round(age, 2),
                )
        return t
