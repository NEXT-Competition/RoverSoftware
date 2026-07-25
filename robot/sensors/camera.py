"""Camera capture for the vision stack — raw frames, backend auto-detected.

Yields raw BGR numpy frames so the same code runs on different hardware and
degrades gracefully when there's no camera at all:

    IMX500      -> Raspberry Pi AI Camera (Sony IMX500), inference ON the sensor
    picamera2   -> Raspberry Pi Camera (CSI ribbon) on Bookworm
    OpenCV      -> USB webcam (/dev/videoN) or any V4L2 device
    (none)      -> vision disabled, robot runs normally

Install one capture stack on the rover:
    AI Camera:  sudo apt install python3-picamera2 imx500-all
    Pi Camera:  sudo apt install python3-picamera2
    USB webcam: pip install opencv-python   (or apt install python3-opencv)

--- The IMX500 is a camera that also carries a detector ---
Every other backend here is a pure frame source: something else runs the model.
The IMX500 runs the network inside the sensor and ships the result out as
per-frame libcamera METADATA, so the "detections" ride along with the pixels and
must not be separated from them. That's why frames and metadata are cached
together below (`frame_meta_and_stamp()`), and why `sensors/imx500.py` reads
metadata from this class rather than opening anything itself. Everything else in
this file — the shared-device rule, BGR, the reader thread — is unchanged.

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

import os
import sys
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

    def read_with_metadata(self):
        """-> (frame, None). No per-frame metadata worth carrying — see Camera."""
        return self.read(), None

    def close(self) -> None:
        try:
            self._picam.stop()
        except Exception:
            pass


class _IMX500Source:
    """Raspberry Pi AI Camera (Sony IMX500) — picamera2 plus an on-sensor network.

    Same frame contract as `_Picamera2Source` (BGR out), with two additions:

    * `__init__` uploads a `.rpk` network to the sensor. This is SLOW the first
      time (tens of seconds while the firmware crosses the CSI bus), which is
      exactly why `Camera` opens its source on a background thread — a
      synchronous open here would stall the rover's boot before its ESCs arm.
    * `read_with_metadata()` returns the frame AND the libcamera metadata from
      the SAME request. The inference result lives in that metadata, so pairing
      them at the source is the only way a detection can be trusted to describe
      the frame it's drawn on.

    `imx500` and `picam2` are public because `sensors/imx500.py` needs them to
    decode the output tensor and map boxes back into frame pixels.
    """

    def __init__(self, model_path: str, w: int, h: int, fps: int, verbose: bool = False):
        from picamera2 import Picamera2
        from picamera2.devices import IMX500
        from picamera2.devices.imx500 import NetworkIntrinsics

        # Loading the network also tells us which camera slot the AI Camera is
        # in, so IMX500() must come before Picamera2().
        self.imx500 = IMX500(model_path)
        self.intrinsics = self.imx500.network_intrinsics
        if not self.intrinsics:
            # A .rpk packaged without embedded intrinsics (a custom export).
            # Assume object detection; the caller supplies labels via config.
            self.intrinsics = NetworkIntrinsics()
            self.intrinsics.task = "object detection"
        self.intrinsics.update_with_defaults()

        self.picam2 = Picamera2(self.imx500.camera_num)
        try:
            # "RGB888" is BGR in numpy — see _Picamera2Source. buffer_count is
            # the picamera2 IMX500 example's: the sensor needs headroom to keep
            # frames and their inference results in flight together.
            cfg = self.picam2.create_preview_configuration(
                main={"size": (w, h), "format": "RGB888"},
                controls={"FrameRate": float(fps)},
                buffer_count=12,
            )
            if verbose:
                # Progress bar for the firmware upload. Only when someone is
                # watching a terminal — under systemd it would shred the journal.
                self.imx500.show_network_fw_progress_bar()
            self.picam2.start(cfg, show_preview=False)
            if self.intrinsics.preserve_aspect_ratio:
                # Letterbox the network input rather than stretching it, and tell
                # the sensor, so inference coords still map back to the full frame.
                self.imx500.set_auto_aspect_ratio()
        except Exception:
            # Hand the device back before re-raising: open_source() falls back to
            # a plain Pi Camera, and a half-open Picamera2 would hold the sensor
            # and make that fallback fail too — one bad network would then cost
            # us the FPV feed as well as detection.
            try:
                self.picam2.close()
            except Exception:
                pass
            raise

    def read(self):
        """-> HxWx3 BGR ndarray, or None."""
        return self.read_with_metadata()[0]

    def read_with_metadata(self):
        """-> (BGR ndarray, metadata dict) from one request, or (None, None).

        capture_request() (rather than capture_array()) is what guarantees the
        pixels and the inference metadata came from the same exposure.
        """
        request = self.picam2.capture_request()
        try:
            return request.make_array("main"), request.get_metadata()
        finally:
            # A leaked request starves the pipeline of buffers within seconds.
            request.release()

    def close(self) -> None:
        try:
            self.picam2.stop()
            self.picam2.close()
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

    def read_with_metadata(self):
        """-> (frame, None). V4L2 carries no inference metadata — see Camera."""
        return self.read(), None

    def close(self) -> None:
        try:
            self._cap.release()
        except Exception:
            pass


def imx500_present() -> bool:
    """True if a Raspberry Pi AI Camera is attached.

    Cheap: enumerates libcamera devices, uploads nothing. False (not an
    exception) when picamera2 isn't installed, so callers can use it as a plain
    auto-detect on any machine.
    """
    try:
        from picamera2 import Picamera2

        return any("imx500" in str(c.get("Model", "")).lower()
                   for c in Picamera2.global_camera_info())
    except Exception:
        return False


def open_source(cfg, vision=None):
    """Pick the best available capture backend, or None if none works.

    `vision` is the VisionConfig; it's needed only to find the `.rpk` network
    for an IMX500 (the AI Camera's model lives in the sensor, so choosing the
    camera and choosing the model are the same decision). Without it — or
    without a network on disk — an AI Camera still opens as a plain Pi Camera
    and simply doesn't do inference.
    """
    if not cfg.enabled:
        return None
    dev = str(cfg.device or "auto").lower()
    # AI Camera. Only ever by explicit request: "auto" must not silently spend
    # 30s uploading firmware on a rover whose vision is Edge Impulse.
    if dev in ("imx500", "ai-camera", "aicamera"):
        model = os.path.expanduser(getattr(vision, "imx500_model", "") or "") if vision else ""
        if not model or not os.path.exists(model):
            print(f"[camera] no IMX500 network at {model or '(unset)'} — opening the "
                  "AI Camera as a plain Pi Camera (no on-sensor inference). "
                  "Install one: sudo apt install imx500-all")
        else:
            try:
                return _IMX500Source(model, cfg.width, cfg.height, cfg.fps,
                                     verbose=sys.stdout.isatty())
            except Exception as e:
                print(f"[camera] IMX500 unavailable ({e}) — falling back to a plain camera")
        dev = "picamera2"
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
    if isinstance(source, _IMX500Source):
        net = os.path.basename(getattr(source.imx500, "network_file", "") or "network")
        return f"imx500 (AI Camera, on-sensor {net})"
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

    def __init__(self, cfg, vision=None):
        self.cfg = cfg
        # Only used to locate the IMX500 network; see open_source().
        self.vision = vision
        self._source = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._frame = None
        self._meta = None  # per-frame libcamera metadata (IMX500 inference lives here)
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
        self._source = open_source(self.cfg, self.vision)
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
                # Every backend returns the pair; only the IMX500 fills in the
                # metadata, and only because its inference result rides in it.
                frame, meta = self._source.read_with_metadata()  # BGR; blocks at ~device fps
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
                    self._meta = meta
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

    def frame_meta_and_stamp(self):
        """Latest frame, its libcamera metadata, and its capture time.

        The metadata is None on every backend except the IMX500, where it holds
        the on-sensor inference output. Returned together with the frame on
        purpose: a detection paired with the wrong frame draws boxes that lag
        reality, which looks exactly like a mis-tuned controller.
        """
        with self._lock:
            return self._frame, self._meta, self._stamp

    def source(self):
        """The open backend object, or None until the reader thread opens it.

        Only the IMX500 detector needs this (it decodes the sensor's output
        tensor through the same handle that configured it). Frame consumers
        should use frame()/frame_and_stamp() instead.
        """
        return self._source if self.ok() else None

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
