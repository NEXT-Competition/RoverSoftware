"""Bulk-transfer link over WiFi/LAN, so the radio doesn't have to carry it.

The XBee is the CONTROL link: drive frames, telemetry, mode and e-stop. It is
57600 baud shared between every robot on the channel, and a full config snapshot
is ~2.9 KB — roughly half a second of exclusive airtime, during which telemetry
frames are the ones being dropped (see XBeeLink.send's write timeout). Layouts
and routine documents are worse.

None of that traffic is realtime and none of it is safety-critical, so it rides
a TCP socket over the same WiFi the FPV video already uses (see video_udp.py).
Same wire protocol — newline-delimited JSON via protocol.py — so nothing above
the transport changes shape: a `config` frame is the same dict whichever way it
travelled, and FleetManager can't tell the difference.

    robot                                base station
    IPLink("base.local", 5006)  --TCP-->  IPServer(5006)

The robot DIALS OUT, the base station listens. That mirrors the video path (the
robot is already configured with the base's host) and avoids needing to discover
a rover's address on a DHCP field network — outbound also survives the usual
firewall/NAT asymmetry.

--- This link or nothing ---
`send()` returns a BOOL rather than raising: True means it went out, False means
"not connected". False is NOT an instruction to try the radio. Bulk traffic has
no radio path any more — an unreachable base station, a dropped access point or
an unconfigured `base_host` all mean the same thing, which is that this robot
cannot be configured until the link is back. It keeps driving, reporting and
answering the e-stop over the radio the whole time; those are what the range is
for, and spending it on a config dump is what this module exists to stop.

The one thing that still travels the other way is the ADDRESS of this link
(robot/tuning.py::BOOTSTRAP_PATHS) — a rover pointed at no base station has no
WiFi path to be told about one, so that single ~60-byte frame goes by radio.

--- The fallback that remains ---
Between the two ends, though, `False` still means "keep the frame". The callers
above (Robot._drain_outbox, basestation/app.py) only remove a frame once a link
has taken it, because half a document is not a smaller document.

Nothing here retransmits or acknowledges: TCP already does, and a frame handed
to a dead socket is reported False so the caller can decide rather than having
it silently queued into a socket that will never drain.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Dict, Optional

from . import protocol

DEFAULT_PORT = 5006

# How long a blocking socket call may park before the thread re-checks _running,
# which is what makes stop() prompt instead of hanging for a full read timeout.
_SOCKET_TIMEOUT = 0.5
# Reconnect backoff. Starts fast (a base station that just came up should be
# picked up quickly) and eases off so a robot driving out of WiFi range isn't
# spinning on connect() for the rest of the match.
_RETRY_MIN = 1.0
_RETRY_MAX = 15.0
# Ceiling on one accumulated line, so a peer that never sends a newline can't
# grow the buffer without bound. Comfortably above the largest real frame (a
# doc fragment is ~400 chars, a config chunk ~600).
_MAX_LINE = 65536


def _lines(buf: bytearray, chunk: bytes):
    """Feed `chunk` into `buf` and yield each complete newline-terminated line.

    TCP is a byte stream: one recv() can hold half a frame, or three of them.
    Unlike the serial reader this can't lean on a read timeout to mark a message
    boundary, so the newline is the only framing there is.
    """
    buf.extend(chunk)
    while True:
        nl = buf.find(b"\n")
        if nl < 0:
            if len(buf) > _MAX_LINE:
                del buf[:]  # a peer that never terminates a line; resynchronise
            return
        line = bytes(buf[:nl])
        del buf[:nl + 1]
        if line:
            yield line


class IPLink:
    """Robot side: keep a connection to the base station, or say we haven't got one."""

    def __init__(self, host: str, port: int = DEFAULT_PORT,
                 on_message: Optional[Callable[[dict], None]] = None,
                 robot_id: str = "rover"):
        self.host = host
        self.port = port
        self.on_message = on_message
        self.robot_id = robot_id
        self._sock: Optional[socket.socket] = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._connected = False
        self._logged_failure = False

    def start(self) -> None:
        if not self.host:
            return  # not configured; every send() returns False and the radio carries on
        self._running = True
        self._thread = threading.Thread(target=self._run, name="ip-link", daemon=True)
        self._thread.start()
        print(f"[IPLink] bulk transfers -> {self.host}:{self.port} (radio is the fallback)")

    def is_connected(self) -> bool:
        return self._connected

    def send(self, message: dict) -> bool:
        """True if the frame went out over IP; False means the caller must."""
        with self._lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall(protocol.encode(message))
                return True
            except OSError:
                # Don't close here — the reader thread owns the lifecycle and
                # will notice the same break. Just decline the frame.
                self._connected = False
                return False

    def _run(self) -> None:
        delay = _RETRY_MIN
        while self._running:
            if not self._connect():
                time.sleep(delay)
                delay = min(delay * 2, _RETRY_MAX)
                continue
            delay = _RETRY_MIN
            self._read_until_closed()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=_SOCKET_TIMEOUT)
        except OSError as e:
            # One line per outage, not one per retry: a robot parked out of WiFi
            # range would otherwise fill the journal at the backoff rate.
            if not self._logged_failure:
                self._logged_failure = True
                print(f"[IPLink] {self.host}:{self.port} unreachable ({e}); using the radio")
            return False
        sock.settimeout(_SOCKET_TIMEOUT)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._lock:
            self._sock = sock
            self._connected = True
        self._logged_failure = False
        print(f"[IPLink] connected to {self.host}:{self.port}")
        # Announce who we are so the server can address frames back to us before
        # it has seen any telemetry from this socket.
        self.send({"type": "hello", "from": self.robot_id})
        return True

    def _read_until_closed(self) -> None:
        buf = bytearray()
        sock = self._sock
        while self._running and sock is not None:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break  # peer closed
            for line in _lines(buf, chunk):
                msg = protocol.decode(line)
                if msg is None or self.on_message is None:
                    continue
                try:
                    self.on_message(msg)
                except Exception as e:
                    print(f"[IPLink] handler error: {e}")
        self._disconnect()

    def _disconnect(self) -> None:
        with self._lock:
            sock, self._sock, self._connected = self._sock, None, False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            if self._running:
                print("[IPLink] disconnected; using the radio until it comes back")

    def stop(self) -> None:
        self._running = False
        self._disconnect()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class IPServer:
    """Base station side: accept robot connections, address frames back by robot id.

    One socket per robot, keyed by the `from` on anything it sends (and by the
    `hello` it opens with, so the mapping exists before its first telemetry).
    `send()` follows IPLink's contract — False means "that robot isn't on WiFi,
    use the radio" — so a mixed fleet works with no special handling: the rovers
    in WiFi range get their config over IP, the one out at the far end of the
    field gets it over the radio.
    """

    def __init__(self, port: int = DEFAULT_PORT,
                 on_message: Optional[Callable[[dict], None]] = None,
                 host: str = "0.0.0.0"):
        self.host = host
        self.port = port
        self.on_message = on_message
        self._server: Optional[socket.socket] = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._clients: Dict[str, socket.socket] = {}

    def start(self) -> None:
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(8)
            srv.settimeout(_SOCKET_TIMEOUT)
        except OSError as e:
            # A busy port must not stop the dashboard from coming up — without
            # this the fleet still works, just with everything on the radio.
            print(f"[IPServer] could not listen on {self.host}:{self.port}: {e} "
                  f"— bulk transfers stay on the radio")
            return
        self._server = srv
        self._running = True
        self._thread = threading.Thread(target=self._accept_loop, name="ip-server", daemon=True)
        self._thread.start()
        print(f"[IPServer] bulk transfers on tcp/{self.port}")

    def is_connected(self, robot_id: str) -> bool:
        with self._lock:
            return robot_id in self._clients

    def send(self, message: dict) -> bool:
        """True if it went out over IP. Needs a `to` naming one robot."""
        rid = message.get("to")
        if not rid or not isinstance(rid, str):
            return False  # broadcast/unaddressed: the radio is the shared channel
        with self._lock:
            sock = self._clients.get(rid)
            if sock is None:
                return False
            try:
                sock.sendall(protocol.encode(message))
                return True
            except OSError:
                self._clients.pop(rid, None)
                return False

    def _accept_loop(self) -> None:
        srv = self._server
        while self._running and srv is not None:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(_SOCKET_TIMEOUT)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self._client_loop, args=(conn, addr),
                             name="ip-client", daemon=True).start()
        self._close_all()

    def _client_loop(self, conn: socket.socket, addr) -> None:
        rid = None
        buf = bytearray()
        while self._running:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            for line in _lines(buf, chunk):
                msg = protocol.decode(line)
                if msg is None:
                    continue
                sender = msg.get("from")
                if sender and sender != rid:
                    # Learn (or relearn) which robot this socket belongs to. A
                    # reconnect replaces the old entry rather than accumulating
                    # dead sockets under the same id.
                    rid = sender
                    with self._lock:
                        old = self._clients.get(rid)
                        self._clients[rid] = conn
                    if old is not None and old is not conn:
                        try:
                            old.close()
                        except OSError:
                            pass
                    print(f"[IPServer] {rid} connected from {addr[0]}")
                if msg.get("type") == "hello":
                    continue  # transport-level only; nothing above cares
                if self.on_message is not None:
                    try:
                        self.on_message(msg)
                    except Exception as e:
                        print(f"[IPServer] handler error: {e}")
        with self._lock:
            if rid is not None and self._clients.get(rid) is conn:
                del self._clients[rid]
        try:
            conn.close()
        except OSError:
            pass
        if rid is not None and self._running:
            print(f"[IPServer] {rid} disconnected; its transfers fall back to the radio")

    def _close_all(self) -> None:
        with self._lock:
            socks, self._clients = list(self._clients.values()), {}
        for s in socks:
            try:
                s.close()
            except OSError:
                pass

    def stop(self) -> None:
        self._running = False
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
        self._close_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
