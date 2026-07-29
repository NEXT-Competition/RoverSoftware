"""First-person-view streamer: robot camera -> base station over UDP.

Reads frames from the shared Camera, JPEG-encodes them, and fires them at the
base station via robot.comms.video_udp. Deliberately independent of the object
detector — the live feed works with no model and no Edge Impulse installed; it
needs only a camera and a JPEG encoder (OpenCV or Pillow).

Like the other sensor threads it never blocks the control loop and degrades
gracefully: no camera, or the base unreachable, just means no feed.

Where it streams TO is settable while it runs (`retarget`), which is what makes
`fpv.base_host` a live parameter in robot/tuning.py. It has to be: the address
is a property of wherever the operator happens to be sitting today, the robot
learns it over the radio, and a rover that needed a service restart to point its
camera at a new laptop is a rover you can't get video from in the pit.
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
        # Where to stream. Held apart from cfg and behind a lock because it is
        # written by the control loop (a config frame off the radio) and read by
        # the sender thread — and read as a PAIR, so a half-applied edit can't
        # aim the feed at a new host on the old port.
        self._lock = threading.Lock()
        self._target = (cfg.base_host, cfg.base_port)

    def set_overlay_provider(self, provider) -> None:
        self.overlay_provider = provider

    def retarget(self, host: str, port: int) -> bool:
        """Point the feed at a different base station. True if it moved.

        Takes effect on the next frame: the sender is rebuilt rather than
        adjusted, because it resolves the hostname once at construction — which
        also means retargeting is how a name that only started resolving later
        (the laptop joined the network after the rover booted) gets picked up.
        """
        with self._lock:
            if (host, port) == self._target:
                return False
            self._target = (host, port)
        print(f"[fpv] now streaming to {host}:{port}")
        return True

    def target(self) -> tuple:
        with self._lock:
            return self._target

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

        aimed_at = self.target()
        sender = VideoSender(aimed_at[0], aimed_at[1], self.robot_id)
        self._sender = sender
        print(f"[fpv] streaming to {aimed_at[0]}:{aimed_at[1]} "
              f"@ up to {self.cfg.fps}fps q{self.cfg.jpeg_quality}")

        last_stamp = -1.0
        while self._running:
            t0 = time.monotonic()
            # Rebuild the sender when the target moves. Done here rather than in
            # retarget() so the socket is only ever touched by this thread.
            target = self.target()
            if target != aimed_at:
                sender.close()
                sender = VideoSender(target[0], target[1], self.robot_id)
                self._sender = sender
                aimed_at = target
            # Read from cfg every pass: fps and quality are live parameters, and
            # a rate captured once would ignore the slider that was moved to get
            # the feed through a congested link.
            period = 1.0 / max(self.cfg.fps, 1)
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
                    sender.send_frame(jpeg)
            sleep_for = period - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sender is not None:
            self._sender.close()
