"""Camera capture for the vision stack — raw frames, backend auto-detected.

Yields raw BGR numpy frames so the same code runs on different hardware and
degrades gracefully when there's no camera at all:

    picamera2   -> Raspberry Pi Camera (CSI ribbon) on Bookworm
    OpenCV      -> USB webcam (/dev/videoN) or any V4L2 device
    (none)      -> vision disabled, robot runs normally

Install one capture stack on the rover:
    Pi Camera:  sudo apt install python3-picamera2
    USB webcam: pip install opencv-python   (or apt install python3-opencv)

--- Everything here returns BGR. This is not a preference. ---
Edge Impulse's `get_features_from_image()` does its own BGR->RGB conversion
internally, so it must be handed BGR. Feed it RGB and every channel is swapped:
nothing raises, nothing logs, and the model returns confident-looking garbage
that looks exactly like "the model just doesn't work on this camera". Keep the
channel order in this file and nowhere else, and verify it by eye with
`tools/detector_selftest.py --save` on first bring-up rather than trusting
anyone's assumption about what a backend hands back — including this docstring's.

--- One camera, many consumers ---
A V4L2 or CSI device generally can't be opened twice, but both the object
detector and the FPV streamer want frames. So the `Camera` class below owns the
device on a single background thread (the GPS/IMU pattern: threaded reader,
lock-guarded cache, cheap accessor) and hands the latest frame to whoever asks
via `frame()`. The `_Source` classes are the raw device backends it drives.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple


class _Picamera2Source:
    """Pi Camera (CSI) via picamera2."""

    def __init__(self, w: int, h: int, fps: int):
        from picamera2 import Picamera2

        self._picam = Picamera2()
        # picamera2's "RGB888" is a misnomer: the buffer it hands back through
        # capture_array() is BGR-ordered in numpy. Naming here is inverted
        # relative to byte order, which is exactly the trap the module docstring
        # warns about — so this stays "RGB888" *because* we want BGR out.
        cfg = self._picam.create_video_configuration(
            main={"size": (w, h), "format": "RGB888"}
        )
        self._picam.configure(cfg)
        self._picam.set_controls({"FrameRate": fps})
        self._picam.start()

    def read(self):
        """-> HxWx3 BGR ndarray, or None."""
        return self._picam.capture_array()

    def close(self) -> None:
        try:
            self._picam.stop()
        except Exception:
            pass


class _OpenCVSource:
    """USB / V4L2 device via OpenCV. VideoCapture.read() is already BGR."""

    def __init__(self, device, w: int, h: int, fps: int):
        import cv2

        idx = int(device) if str(device).isdigit() else device
        self._cap = cv2.VideoCapture(idx)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {device!r}")

    def read(self):
        """-> HxWx3 BGR ndarray, or None."""
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        try:
            self._cap.release()
        except Exception:
            pass


def open_source(cfg):
    """Pick the best available capture backend, or None if none works."""
    if not cfg.enabled:
        return None
    dev = str(cfg.device or "auto").lower()
    # Pi Camera (auto, or asked for by name).
    if dev in ("auto", "picamera2", "picamera", "csi"):
        try:
            return _Picamera2Source(cfg.width, cfg.height, cfg.fps)
        except Exception as e:
            if dev != "auto":
                print(f"[camera] picamera2 unavailable: {e}")
    # USB / V4L2 (auto falls through to index 0; or an explicit /dev/videoN | index).
    if dev == "auto" or dev.startswith("/dev/") or dev.isdigit():
        try:
            device = 0 if dev == "auto" else cfg.device
            return _OpenCVSource(device, cfg.width, cfg.height, cfg.fps)
        except Exception as e:
            print(f"[camera] OpenCV capture unavailable: {e}")
    return None


def describe(source) -> str:
    """Human-readable backend name, for logs and the selftest."""
    if source is None:
        return "none"
    return {
        _Picamera2Source: "picamera2 (CSI)",
        _OpenCVSource: "opencv (V4L2/USB)",
    }.get(type(source), type(source).__name__)


def draw_boxes(frame, boxes):
    """Return an annotated COPY of a BGR frame with detection boxes drawn.

    `boxes` is an iterable of (x, y, w, h, label, conf, is_target). The copy is
    deliberate: the same frame object is the shared camera's cached frame and the
    detector's input, so drawing in place would corrupt both. If OpenCV isn't
    importable there's nothing to draw with, so the original frame is returned
    unchanged (the feed still works, just without boxes).
    """
    try:
        import cv2
    except Exception:
        return frame
    out = frame.copy()
    for x, y, w, h, label, conf, is_target in boxes:
        # BGR: bright green for the box the controller is tracking, amber for the
        # rest, so at a glance you can tell what object_align is chasing.
        color = (0, 255, 0) if is_target else (0, 190, 255)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2 if is_target else 1)
        tag = f"{label} {conf:.2f}" if label else f"{conf:.2f}"
        cv2.putText(out, tag, (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return out


def encode_jpeg(frame, quality: int) -> Optional[bytes]:
    """Encode a BGR ndarray to JPEG bytes via OpenCV, falling back to Pillow."""
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        return buf.tobytes() if ok else None
    except Exception:
        pass
    try:
        import io
        from PIL import Image
        rgb = frame[:, :, ::-1]  # BGR -> RGB for Pillow
        b = io.BytesIO()
        Image.fromarray(rgb).save(b, format="JPEG", quality=int(quality))
        return b.getvalue()
    except Exception:
        return None


class Camera:
    """Owns the capture device on a background thread; caches the latest frame.

    Shared by every frame consumer — the object detector and the FPV streamer —
    because a V4L2/CSI device generally can't be opened twice. Same contract as
    the GPS/IMU readers: start()/stop(), a cheap locked accessor, and graceful
    degradation when no backend/deps exist (frame() just returns None and the
    consumers idle).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._source = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0  # time.monotonic() of the latest frame
        self._ok = False

    def start(self) -> None:
        if not self.cfg.enabled:
            return
        self._running = True
        # Open the device on the thread, not here: a wedged camera would
        # otherwise hang Robot.start() before the ESCs arm or the radio opens.
        self._thread = threading.Thread(target=self._loop, name="camera-rx", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        self._source = open_source(self.cfg)
        if self._source is None:
            print("[camera] no camera/deps available — capture disabled "
                  "(install python3-picamera2 or python3-opencv)")
            self._running = False
            return
        print(f"[camera] capturing {self.cfg.width}x{self.cfg.height} "
              f"via {describe(self._source)}")
        with self._lock:
            self._ok = True

        errors = 0
        while self._running:
            try:
                frame = self._source.read()  # BGR; blocks at ~device fps
                errors = 0
            except Exception as e:
                errors += 1
                if errors == 1 or errors % 50 == 0:
                    print(f"[camera] read error (x{errors}): {e}")
                time.sleep(0.2)
                continue
            if frame is not None:
                with self._lock:
                    self._frame = frame
                    self._stamp = time.monotonic()

        with self._lock:
            self._ok = False

    def frame(self):
        """Latest BGR frame, or None. Cheap — safe to call every tick."""
        with self._lock:
            return self._frame

    def frame_and_stamp(self) -> Tuple[Optional["object"], float]:
        """Latest frame plus its monotonic capture time, so a consumer can skip
        re-processing a frame it has already seen."""
        with self._lock:
            return self._frame, self._stamp

    def ok(self) -> bool:
        """True once the device is open and the reader loop is running."""
        with self._lock:
            return self._ok

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._source is not None:
            self._source.close()
