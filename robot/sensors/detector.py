"""Edge Impulse object detection — the eyes for object_align mode.

Runs a compiled Edge Impulse `.eim` model over camera frames on a background
thread and caches the best current detection, so the control loop can poll
`detection()` without ever touching the camera or the model (which would stall
the tick — see robot.py's slow-tick watchdog; inference is 50-200ms, and the
tick budget is a few ms).

    det = ObjectDetector(cfg.vision, cfg.camera)
    det.start()
    ...
    d = det.detection()   # -> Detection, or None if nothing is currently seen
    det.stop()

This mirrors the GPS/IMU contract: background thread, lock-guarded cache, cheap
non-blocking accessor, and graceful degradation — if edge_impulse_linux isn't
installed, or there's no model, or no camera, `start()` logs why and the reader
stays inert. `detection()` returns None forever and the rest of the robot runs
unchanged on a dev laptop.

--- Staleness is the whole safety story ---
On a frame with no target we do NOT clear the cache; we just stop advancing the
stamp. `detection()` then ages the sample out after `target_timeout`. One
mechanism, three failure modes:
  * a single dropped/failed frame -> coast on the last detection, no lurch
  * a genuinely lost target       -> ages out cleanly, controller searches
  * the detector THREAD DIES      -> stamps stop advancing -> target ages out ->
                                     the controller stops. No liveness check
                                     needed on the control path; it fails safe
                                     by construction.
Clearing on a miss would break the first case and flap stop/search at frame rate.

--- Model architecture caveat (FOMO) ---
FOMO models emit centroids with fixed, cell-sized boxes — object size is simply
not available, so `Detection.size` is None and object_align degrades to
align-only. Export a YOLO-style model (model_type == 'object_detection') if you
want approach + standoff. We detect this at init and say so once, loudly.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

from ..control.detection import Detection, distance_m

try:  # pragma: no cover - Linux/Pi-only dependency, absent on a dev laptop
    from edge_impulse_linux.image import ImageImpulseRunner
    _IMPORT_ERROR = None
except Exception as _e:
    # Keep WHY it failed. edge_impulse_linux itself is pure Python, but its image
    # module imports cv2 (OpenCV) and numpy at load time, so a missing transitive
    # dependency lands here too and would otherwise read as "not installed" even
    # after the user has installed edge_impulse_linux.
    ImageImpulseRunner = None
    _IMPORT_ERROR = _e

# FOMO. EI reports this model_type for constrained object detection, which
# yields centroids rather than sized boxes.
_FOMO_TYPE = "constrained_object_detection"


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class ObjectDetector:
    def __init__(self, cfg, camera):
        self.cfg = cfg
        # The camera is shared (see sensors/camera.py) — the FPV streamer reads
        # the same device. This class never opens it; it only samples frame().
        self.camera = camera
        self._period = 1.0 / cfg.max_fps if cfg.max_fps > 0 else 0.1

        self._runner = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        # Latest cached state (guarded by _lock).
        self._detection: Optional[Detection] = None
        # Every above-confidence box for the FPV overlay, in FULL-FRAME pixels:
        # (x, y, w, h, label, conf, is_target). Separate from _detection, which is
        # the single normalized target the controller steers on.
        self._overlays: list = []
        self._overlays_stamp = 0.0
        self._ok = False  # model initialized and the loop is alive
        self._fps = 0.0
        self._labels: list = []
        self._fomo = False
        self._model_wh = (0, 0)

    # --- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        """Pre-flight the cheap failure modes, then hand off to the thread.

        Everything checked here is instant (an import, a stat, an access bit).
        The expensive part — runner.init(), which spawns the .eim subprocess and
        does a socket handshake — deliberately happens on the thread instead.
        The IMU opens I2C synchronously in start() because that's fast; doing the
        same with a model that hangs would hang Robot.start(), so the rover would
        never arm its ESCs and never open the radio: dead and uncommandable in
        the field. A broken model must only ever cost us vision.
        """
        if not self.cfg.enabled:
            return
        if ImageImpulseRunner is None:
            missing = getattr(_IMPORT_ERROR, "name", None)
            if isinstance(_IMPORT_ERROR, ModuleNotFoundError) and missing and missing != "edge_impulse_linux":
                # edge_impulse_linux is present, but one of its deps isn't — the
                # usual culprit is OpenCV, which the Pi doesn't have by default.
                hint = ("sudo apt install python3-opencv" if missing == "cv2"
                        else f"pip install {missing}")
                print(f"[detector] edge_impulse_linux is installed, but its dependency "
                      f"'{missing}' is missing — object detection disabled "
                      f"(object_align will hold still). Install it on the Pi: {hint}")
            else:
                print(f"[detector] edge_impulse_linux unavailable ({_IMPORT_ERROR}) — "
                      "object detection disabled (object_align will hold still). "
                      "Install on the Pi: pip install edge_impulse_linux")
            return
        path = os.path.expanduser(self.cfg.model_path)
        if not os.path.exists(path):
            print(f"[detector] no model at {path} — object detection disabled. "
                  "Download one: edge-impulse-linux-runner --download model.eim")
            return
        if not os.access(path, os.X_OK):
            # The .eim is a binary EI execs, not a data file. This is the single
            # most common first-run failure and its native error is opaque.
            print(f"[detector] model {path} is not executable — object detection "
                  f"disabled. Fix: chmod +x {path}")
            return
        if self.camera is None:
            print("[detector] camera disabled — object detection disabled "
                  "(object_align will hold still)")
            return

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="detector-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            # Must outlast one in-flight inference (up to ~200ms) plus a frame
            # read; the 1.0s the GPS/IMU readers use is too tight here.
            self._thread.join(timeout=2.0)
        # The camera is shared and owned elsewhere — don't close it here.
        if self._runner is not None:
            try:
                self._runner.stop()  # reap the .eim subprocess
            except Exception:
                pass

    # --- the thread --------------------------------------------------------

    def _init_runner(self) -> bool:
        """Open the model and the camera. Returns False to kill the thread."""
        path = os.path.expanduser(self.cfg.model_path)
        try:
            self._runner = ImageImpulseRunner(path)
            info = self._runner.init()
        except Exception as e:
            print(f"[detector] could not init model {path}: {e} — object detection disabled")
            return False

        mp = info.get("model_parameters", {}) or {}
        labels = list(mp.get("labels", []) or [])
        mw = int(mp.get("image_input_width", 0) or 0)
        mh = int(mp.get("image_input_height", 0) or 0)
        fomo = mp.get("model_type") == _FOMO_TYPE
        if mw <= 0 or mh <= 0:
            print(f"[detector] model reports no input size ({mw}x{mh}) — disabled")
            return False

        with self._lock:
            self._labels, self._model_wh, self._fomo = labels, (mw, mh), fomo

        owner = info.get("project", {}).get("owner", "?")
        name = info.get("project", {}).get("name", "?")
        print(f"[detector] model '{owner}/{name}' {mw}x{mh} labels={labels}")

        if self.cfg.target_label and self.cfg.target_label not in labels:
            print(f"[detector] WARNING: target_label {self.cfg.target_label!r} is not "
                  f"one of this model's labels {labels} — nothing will ever match")
        if fomo:
            print("[detector] WARNING: this is a FOMO model — it reports centroids, "
                  "not sized boxes, so object size is unavailable. object_align will "
                  "turn to face the target but will NOT approach or stop at standoff. "
                  "Export a YOLO-style (object_detection) model for approach.")

        print(f"[detector] inference capped at {self.cfg.max_fps:g} fps")
        return True

    def _read_loop(self) -> None:
        if not self._init_runner():
            self._running = False
            return
        with self._lock:
            self._ok = True

        errors = 0
        last_stamp = -1.0
        while self._running:
            t0 = time.monotonic()
            try:
                # Frames come from the shared Camera (which the FPV streamer also
                # reads); sample the latest and skip one we've already classified.
                frame, stamp = self.camera.frame_and_stamp()
                if frame is None or stamp == last_stamp:
                    time.sleep(0.01)
                    continue
                last_stamp = stamp
                features, _cropped = self._runner.get_features_from_image(frame)
                res = self._runner.classify(features)
                errors = 0
            except Exception as e:
                # If the .eim subprocess has died, EVERY iteration raises. An
                # unguarded print here would flood the journal at frame rate.
                errors += 1
                if errors == 1 or errors % 50 == 0:
                    print(f"[detector] inference error (x{errors}): {e}")
                time.sleep(0.5)
                continue

            self._consume(res, frame, time.monotonic())

            elapsed = time.monotonic() - t0
            with self._lock:
                self._fps = (1.0 / elapsed) if elapsed > 0 else 0.0
            sleep_for = self._period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

        with self._lock:
            self._ok = False

    def _consume(self, res: dict, frame, stamp: float) -> None:
        """Fold one classification into the cached detection + FPV overlays."""
        boxes = (res.get("result", {}) or {}).get("bounding_boxes", []) or []
        kept = [b for b in boxes
                if b.get("value", 0.0) >= self.cfg.min_confidence
                and (not self.cfg.target_label or b.get("label") == self.cfg.target_label)]
        best = self._select(kept)
        if best is None:
            # Deliberately do NOT clear — let the sample age out. See docstring.
            return

        mw, mh = self._model_wh
        # Box coords are in the model's CROPPED input space, not the capture
        # frame's — get_features_from_image() resized and center-cropped first.
        # Normalize against the model input, never camera_cfg.width/height.
        # (The crop is centered, so error_x == 0 still means centered in the real
        # frame; what the crop costs us is ~25% of the horizontal FOV at 640x480,
        # which is why VisionConfig.hfov_deg must be the post-crop value.)
        cx = best["x"] + best["width"] / 2.0
        cy = best["y"] + best["height"] / 2.0
        detection = Detection(
            label=best.get("label", ""),
            confidence=float(best.get("value", 0.0)),
            # Clamp: boxes can extend past the crop edge.
            error_x=_clamp(2.0 * cx / mw - 1.0, -1.0, 1.0),
            error_y=_clamp(2.0 * cy / mh - 1.0, -1.0, 1.0),
            size=None if self._fomo else _clamp(best["height"] / mh, 0.0, 1.0),
            stamp=stamp,
        )

        # Overlays for the FPV feed: map every kept box from model space back to
        # full-frame pixels (see _to_full_frame). Uses the frame's real shape so
        # it's correct even if the camera ignored the requested resolution.
        h, w = frame.shape[0], frame.shape[1]
        overlays = []
        for b in kept:
            rect = self._to_full_frame(b, w, h)
            overlays.append((*rect, b.get("label", ""), float(b.get("value", 0.0)), b is best))

        with self._lock:
            self._detection = detection
            self._overlays = overlays
            self._overlays_stamp = stamp

    def _to_full_frame(self, box: dict, w: int, h: int):
        """Invert Edge Impulse's resize + center-crop: model-space box -> full-frame
        pixel rect (x, y, bw, bh), clamped to the frame.

        get_features_from_image() scales the frame by the LARGEST factor so it
        covers the model input, then center-crops. So the reverse is: undo the
        crop offset, then undo the scale.
        """
        mw, mh = self._model_wh
        if mw <= 0 or mh <= 0:
            return 0, 0, 0, 0
        scale = max(mw / w, mh / h)          # frame -> model resize factor
        crop_x = (w * scale - mw) / 2.0      # pixels trimmed each side, in model space
        crop_y = (h * scale - mh) / 2.0
        x = int((box.get("x", 0) + crop_x) / scale)
        y = int((box.get("y", 0) + crop_y) / scale)
        bw = int(box.get("width", 0) / scale)
        bh = int(box.get("height", 0) / scale)
        # Clamp to the frame; boxes can spill past the crop edge.
        x = _clamp(x, 0, w - 1)
        y = _clamp(y, 0, h - 1)
        bw = _clamp(bw, 0, w - x)
        bh = _clamp(bh, 0, h - y)
        return x, y, bw, bh

    def _select(self, boxes: list) -> Optional[dict]:
        """Pick one box out of the candidates.

        No tracking or data association here: this is a greedy per-frame choice,
        so two similar objects can make 'largest' flap between frames.
        'centermost' is the mitigation — it prefers the one we're already facing.
        """
        if not boxes:
            return None
        mode = self.cfg.select
        # Every FOMO box is the same size, so 'largest' would be an arbitrary
        # tie-break. Confidence is the only meaningful ranking there.
        if mode == "largest" and self._fomo:
            mode = "confidence"
        if mode == "confidence":
            return max(boxes, key=lambda b: b.get("value", 0.0))
        if mode == "centermost":
            mw = self._model_wh[0] or 1
            return min(boxes, key=lambda b: abs((b["x"] + b["width"] / 2.0) - mw / 2.0))
        return max(boxes, key=lambda b: b.get("width", 0) * b.get("height", 0))

    # --- accessors (cheap; safe to call every control tick) ----------------

    def detection(self) -> Optional[Detection]:
        """Latest detection, or None if nothing is currently seen.

        None once the sample is older than target_timeout — which is also what
        makes a dead detector thread fail safe (no new stamps -> ages out).
        """
        with self._lock:
            d = self._detection
            if d is None:
                return None
            if (time.monotonic() - d.stamp) > self.cfg.target_timeout:
                return None
            return d

    def overlays(self) -> list:
        """Latest detection boxes in full-frame pixels for the FPV overlay, as
        (x, y, w, h, label, conf, is_target) tuples. Empty once stale, so a lost
        target stops drawing boxes rather than freezing the last ones."""
        with self._lock:
            if not self._overlays:
                return []
            if (time.monotonic() - self._overlays_stamp) > self.cfg.target_timeout:
                return []
            return self._overlays

    def telemetry(self) -> dict:
        """Compact status for the base station. Kept small — the radio is 57600
        baud and shared, so this is a summary, never boxes or frames."""
        with self._lock:
            ok, fps, d = self._ok, self._fps, self._detection
        t = {"ok": ok, "fps": round(fps, 1)}
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
                # Metres, only on a calibrated build — the key is absent (not
                # null-padded) so an uncalibrated robot costs zero radio bytes.
                dist = distance_m(d.size, self.cfg.focal_frac,
                                  self.cfg.target_height_m)
                if dist is not None:
                    t["dist"] = round(dist, 2)
        return t


class MockDetector:
    """A synthetic target, so the whole object_align stack runs with no hardware.

    Sweeps a target left/right while it "approaches" (size ramps up), then blanks
    out entirely for a few seconds. That exercises every branch of the controller
    — pivot, approach, arrival latch, target loss, search, and the search
    give-up timeout — against mock ESCs on a laptop. Same interface as
    ObjectDetector; no imports, no camera, no model.

    Enable with RS_MOCK_DETECTOR=1 (cf. RS_MOCK_MOTORS).
    """

    _PERIOD = 15.0  # one full sighting + blackout cycle
    _BLACKOUT = 3.0  # trailing seconds of each cycle with no target

    def __init__(self, label: str = "mock", cycle_s: float = 20.0):
        self.label = label
        self.cycle_s = cycle_s
        self._t0 = 0.0

    def start(self) -> None:
        self._t0 = time.monotonic()
        print("[detector] MOCK detector — synthesizing a sweeping target "
              "(no camera, no model). Unset RS_MOCK_DETECTOR for the real one.")

    def stop(self) -> None:
        pass

    def _phase(self) -> float:
        return (time.monotonic() - self._t0) % self._PERIOD

    def detection(self) -> Optional[Detection]:
        now = time.monotonic()
        phase = self._phase()
        if phase > (self._PERIOD - self._BLACKOUT):
            return None  # blackout: drives loss -> search -> give-up
        t = now - self._t0
        return Detection(
            label=self.label,
            confidence=0.9,
            error_x=0.6 * math.sin(2.0 * math.pi * t / 8.0),
            error_y=0.0,
            # Ramp 0.1 -> 0.6 across cycle_s so it crosses a 0.45 standoff.
            size=0.1 + 0.5 * ((t % self.cycle_s) / self.cycle_s),
            stamp=now,
        )

    def telemetry(self) -> dict:
        d = self.detection()
        t = {"ok": True, "fps": 10.0, "mock": True}
        if d is not None:
            t.update(label=d.label, conf=round(d.confidence, 2),
                     ex=round(d.error_x, 3),
                     size=round(d.size, 3) if d.size is not None else None,
                     age=0.0)
        return t
