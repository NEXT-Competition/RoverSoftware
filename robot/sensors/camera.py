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

Like the GPS and IMU readers, this never blocks the control loop: the caller
(sensors/detector.py) owns the thread; this just opens a device and reads it.
"""

from __future__ import annotations


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
