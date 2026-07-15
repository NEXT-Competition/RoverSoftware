"""XBee serial link (transparent / AT mode).

The XBee is run in transparent mode: bytes in = bytes out. We read
newline-delimited JSON frames on a background thread and hand each decoded
message to a callback. Sending is available for telemetry back to the base
station.

For API-mode XBee you'd swap the transport internals for digi-xbee; the public
interface (start / stop / send + on_message callback) stays the same.

Threading note: on_message runs on the reader thread. The Robot wires it to a
thread-safe queue and processes messages on the main control loop, so no
controller state is touched from two threads.
"""

from __future__ import annotations

import threading
from typing import Callable

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

from . import protocol


class XBeeLink:
    def __init__(self, port: str, baud: int, on_message: Callable[[dict], None]):
        self.port = port
        self.baud = baud
        self.on_message = on_message
        self._serial = None
        self._thread = None
        self._running = False
        self._write_lock = threading.Lock()

    def start(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed; run: pip install pyserial")
        # write_timeout is essential: without it a stalled write (XBee not
        # draining the buffer — RF congestion, a flaky USB adapter, flow-control
        # stall) blocks forever and freezes the control loop that calls send().
        # With it, a stuck write raises SerialTimeoutException, caught in send().
        self._serial = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.2)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="xbee-rx", daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        buf = bytearray()
        while self._running:
            try:
                chunk = self._serial.readline()  # up to '\n' or the read timeout
            except Exception as e:  # keep the link alive across transient errors
                print(f"[XBeeLink] read error: {e}")
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            if not chunk.endswith(b"\n"):
                continue  # partial line (timeout); keep accumulating
            line = bytes(buf)
            buf.clear()
            msg = protocol.decode(line)
            if msg is None:
                continue
            try:
                self.on_message(msg)
            except Exception as e:
                print(f"[XBeeLink] handler error: {e}")

    def send(self, message: dict) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        try:
            with self._write_lock:
                self._serial.write(protocol.encode(message))
        except serial.SerialTimeoutException:
            # The radio isn't draining the buffer. Telemetry is best-effort, so
            # drop this frame and clear the backlog rather than stalling the
            # control loop — the next frame writes fresh instead of queueing
            # behind a stuck one.
            try:
                self._serial.reset_output_buffer()
            except Exception:
                pass
            print("[XBeeLink] telemetry write timed out; dropped frame")
        except Exception as e:
            print(f"[XBeeLink] write error: {e}")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
