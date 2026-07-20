"""Low-latency JPEG-over-UDP video link (rover -> base station).

The XBee command/telemetry radio is far too slow for video (57600 baud), so the
live feed rides a separate IP path: the robot fires JPEG frames at the base
station over UDP on the shared WiFi/LAN. That means FPV only works within WiFi
range — the XBee stays the long-range control link — which is the right trade,
since video is inherently short-range and bandwidth-hungry.

Why UDP: for a live feed you want the *freshest* frame, not every frame. UDP
lets us drop lost/late packets instead of stalling to retransmit — a corrupted
or incomplete frame is simply discarded and the next one shown. This is what
keeps glass-to-glass latency low.

Framing: each JPEG frame is split into datagrams that fit in one MTU. Every
packet carries a fixed header so the receiver can reassemble one frame and tell
frames apart:

    magic 'UCV1' | robot_id[16] | frame_seq(u32) | chunk_index(u16) | chunk_count(u16)

The receiver keeps only the newest frame per robot: if any chunk of a frame is
lost, that frame never completes and is dropped as soon as a newer frame_seq
arrives. Shared by both ends (the base station imports VideoReceiver), like
protocol.py and xbee_link.py.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Dict, List, Optional

MAGIC = b"UCV1"
# magic(4) + robot_id(16) + frame_seq(u32) + chunk_index(u16) + chunk_count(u16)
_HEADER = struct.Struct("!4s16sIHH")
DEFAULT_PORT = 5005
DEFAULT_MTU = 1400  # conservative for typical Ethernet/Wi-Fi paths


class VideoSender:
    """Fragment JPEG frames and fire them at the base station over UDP."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, robot_id: str = "rover",
                 mtu: int = DEFAULT_MTU):
        try:  # resolve once; fall back to letting sendto() resolve each call
            ip = socket.gethostbyname(host)
        except OSError:
            ip = host
        self._addr = (ip, port)
        self._rid = robot_id.encode("utf-8")[:16].ljust(16, b"\0")
        self._payload = max(256, mtu - _HEADER.size)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0

    def send_frame(self, jpeg: bytes) -> None:
        n = (len(jpeg) + self._payload - 1) // self._payload
        if n == 0 or n > 0xFFFF:  # empty, or too big to index in u16
            return
        seq = self._seq & 0xFFFFFFFF
        self._seq += 1
        for i in range(n):
            chunk = jpeg[i * self._payload:(i + 1) * self._payload]
            try:
                self._sock.sendto(_HEADER.pack(MAGIC, self._rid, seq, i, n) + chunk, self._addr)
            except OSError:
                break  # base unreachable this frame; try again next one

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass


class VideoReceiver:
    """Reassemble UDP frames on the base station; expose the latest per robot."""

    def __init__(self, bind_host: str = "0.0.0.0", port: int = DEFAULT_PORT,
                 stale_after: float = 3.0):
        self._bind = (bind_host, port)
        self._stale = stale_after
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: Dict[str, tuple] = {}   # rid -> (jpeg, monotonic_ts)
        self._asm: Dict[str, dict] = {}        # rid -> in-progress frame (reader thread only)

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            self._sock.bind(self._bind)
        except OSError as e:
            print(f"[video] could not bind UDP {self._bind[0]}:{self._bind[1]}: {e} "
                  "— live video disabled")
            self._sock.close()
            self._sock = None
            return
        self._sock.settimeout(0.5)
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="video-rx", daemon=True)
        self._thread.start()
        print(f"[video] receiving on udp/{self._bind[1]}")

    def _loop(self) -> None:
        while self._running:
            try:
                pkt, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) < _HEADER.size:
                continue
            magic, rid_raw, seq, idx, count = _HEADER.unpack(pkt[:_HEADER.size])
            if magic != MAGIC:
                continue
            rid = rid_raw.rstrip(b"\0").decode("utf-8", "replace")
            self._on_chunk(rid, seq, idx, count, pkt[_HEADER.size:])

    def _on_chunk(self, rid, seq, idx, count, payload) -> None:
        if count == 0 or idx >= count:
            return
        a = self._asm.get(rid)
        if a is None or seq != a["seq"]:
            # New frame_seq. Ignore stragglers from an *older* frame (u32 wrap-aware);
            # otherwise start a fresh assembly, discarding any incomplete older one.
            if a is not None and ((seq - a["seq"]) & 0xFFFFFFFF) & 0x80000000:
                return
            a = {"seq": seq, "count": count, "chunks": {}}
            self._asm[rid] = a
        a["chunks"][idx] = payload
        if len(a["chunks"]) == count:
            jpeg = b"".join(a["chunks"][i] for i in range(count))
            with self._lock:
                self._latest[rid] = (jpeg, time.monotonic())
            del self._asm[rid]

    def latest(self, robot_id: str) -> Optional[bytes]:
        with self._lock:
            item = self._latest.get(robot_id)
        if not item:
            return None
        jpeg, ts = item
        return jpeg if (time.monotonic() - ts) <= self._stale else None

    def robots(self) -> List[str]:
        now = time.monotonic()
        with self._lock:
            return [r for r, (_, ts) in self._latest.items() if now - ts <= self._stale]

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
