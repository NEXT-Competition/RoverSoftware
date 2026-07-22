"""First-person-view streamer: robot camera -> base station over UDP.

Reads frames from the shared Camera, JPEG-encodes them, and fires them at the
base station via robot.comms.video_udp. Deliberately independent of the object
detector — the live feed works with no model and no Edge Impulse installed; it
needs only a camera and a JPEG encoder (OpenCV or Pillow).

Like the other sensor threads it never blocks the control loop and degrades
gracefully: no camera, or the base unreachable, just means no feed.
"""

from __future__ import annotations

import threading
import time

from .camera import draw_boxes, encode_jpeg


class FPVStreamer:
    def __init__(self, cfg, camera, robot_id: str, overlay_provider=None):
        self.cfg = cfg
        self.camera = camera
        self.robot_id = robot_id
        # overlay_provider() -> list of (x,y,w,h,label,conf,is_target) in
        # full-frame pixels, or None/[] for none. Wired to the detector so the
        # live feed shows what was detected; absent -> a plain feed.
        self.overlay_provider = overlay_provider
        self._sender = None
        self._thread = None
        self._running = False

    def set_overlay_provider(self, provider) -> None:
        self.overlay_provider = provider

    def start(self) -> None:
        if not self.cfg.enabled:
            return
        if self.camera is None:
            print("[fpv] no camera — live view disabled")
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="fpv-tx", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        from ..comms.video_udp import VideoSender

        self._sender = VideoSender(self.cfg.base_host, self.cfg.base_port, self.robot_id)
        print(f"[fpv] streaming to {self.cfg.base_host}:{self.cfg.base_port} "
              f"@ up to {self.cfg.fps}fps q{self.cfg.jpeg_quality}")

        period = 1.0 / max(self.cfg.fps, 1)
        last_stamp = -1.0
        while self._running:
            t0 = time.monotonic()
            frame, stamp = self.camera.frame_and_stamp()
            # Only encode+send a genuinely new frame. Re-sending the same image
            # would burn a core on JPEG encoding and airtime for no visible gain.
            if frame is not None and stamp != last_stamp:
                last_stamp = stamp
                if self.overlay_provider is not None:
                    boxes = self.overlay_provider()
                    if boxes:
                        # Draws on a copy — never mutate the shared camera frame.
                        frame = draw_boxes(frame, boxes)
                jpeg = encode_jpeg(frame, self.cfg.jpeg_quality)
                if jpeg:
                    self._sender.send_frame(jpeg)
            sleep_for = period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sender is not None:
            self._sender.close()
