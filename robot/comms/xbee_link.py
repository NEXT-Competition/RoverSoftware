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

Two ways to send, because the traffic is two different things:

    send(msg)       realtime — drive, telemetry, mode, e-stop. Goes now.
                    Best-effort: a frame the radio can't take is dropped,
                    because the next one supersedes it anyway.
    send_bulk(msg)  a fragment of a config snapshot, a layout, a routine.
                    Goes only if the link has the airtime (see airtime.py),
                    and a False means "keep it queued", not "it's gone".

Mixing the two was the bug: bulk frames sent at realtime speed overran the
line, and the drop that kept the control loop responsive silently deleted a
fragment of a document nobody would ever re-request.
"""

from __future__ import annotations

import threading
from typing import Callable

try:
    import serial
except Exception:  # pragma: no cover
    serial = None

from . import protocol
from .airtime import Airtime

# What became of one write. BUSY is the only one worth offering again: the port
# is fine, it just can't take this frame yet.
SENT = "sent"
BUSY = "busy"
DEAD = "dead"


class XBeeLink:
    def __init__(self, port: str, baud: int, on_message: Callable[[dict], None]):
        self.port = port
        self.baud = baud
        self.on_message = on_message
        self._serial = None
        self._thread = None
        self._running = False
        self._write_lock = threading.Lock()
        self.airtime = Airtime(baud)

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

    def send(self, message: dict) -> bool:
        """Put a realtime frame on the radio now. True if it went out.

        Charged against the airtime budget afterwards rather than checked
        against it first: a drive command or an e-stop must never wait behind a
        layout transfer. The charge is what makes the bulk sender back off for
        the interval this frame occupies.
        """
        data = protocol.encode(message)
        self.airtime.debit(len(data))
        return self._write(data, "telemetry") is SENT

    def send_bulk(self, message: dict) -> bool:
        """Offer a non-realtime frame to the radio.

        Returns whether the caller is DONE with this frame — not whether it was
        transmitted. The two differ in the case that matters:

            True   written, or there is no port to write it to and there never
                   will be. Either way, forget it.
            False  the link is busy this instant. Keep the frame at the head of
                   the queue and offer it again next tick.

        The distinction is the whole design. Half a document is not a smaller
        document (see comms/doc_transfer.py), so a fragment dropped because the
        radio was momentarily full is a settings page that never fills — while a
        fragment held forever against a dead port is a queue that never drains.
        """
        data = protocol.encode(message)
        if not self.airtime.take(len(data)):
            return False
        return self._write(data, "bulk") is not BUSY

    def _write(self, data: bytes, what: str) -> str:
        if self._serial is None or not self._serial.is_open:
            return DEAD
        try:
            with self._write_lock:
                self._serial.write(data)
            return SENT
        except serial.SerialTimeoutException:
            # The radio isn't draining the buffer. Clear the backlog rather than
            # stalling the control loop — but the write may have got part of a
            # line out before it timed out, and the next frame would then be
            # read as a continuation of that garbage and lost with it. A bare
            # newline terminates the wreckage so whatever comes next parses.
            try:
                self._serial.reset_output_buffer()
                self._serial.write(b"\n")
            except Exception:
                pass
            print(f"[XBeeLink] {what} write timed out; frame not sent")
            return BUSY
        except Exception as e:
            # Anything else is a fault, not congestion. Retrying it every tick
            # would spin the loop and fill the log without ever succeeding.
            print(f"[XBeeLink] write error: {e}")
            return DEAD

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
