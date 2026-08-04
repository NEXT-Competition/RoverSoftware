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
from collections import deque
import time

# What became of one write. BUSY is the only one worth offering again: the port
# is fine, it just can't take this frame yet.
SENT = "sent"
BUSY = "busy"
DEAD = "dead"


class _Stats:
    def __init__(self, n=1000):
        self.wait = deque(maxlen=n)  # readline block time
        self.decode = deque(maxlen=n)  # protocol.decode
        self.handler = deque(maxlen=n)  # on_message
        self.gap = deque(maxlen=n)  # line-to-line interval (throughput)
        self.partials = 0
        self.drops = 0

    def __str__(self) -> str:
        """Returns a readable string representation of the current stats."""

        # FIX: We use [-1] to look at the last element without removing it.
        wait_val = self.wait[-1] if self.wait else "N/A"
        gap_val = self.gap[-1] if self.gap else "N/A"
        handler_val = self.handler[-1] if self.handler else "N/A"
        decode_val = self.decode[-1] if self.decode else "N/A"

        return (
            f"Wait: {wait_val}, Partials: {self.partials}, Drops: {self.drops}, "
            f"Gap: {gap_val}, Handler: {handler_val}, Decode: {decode_val}"
        )


class XBeeLink:
    def __init__(self, port: str, baud: int, on_message: Callable[[dict], None]):
        self.port = port
        self.baud = baud
        self.on_message = on_message
        self._serial = None
        self._thread = None
        self._running = False
        self._write_lock = threading.Lock()
        self._stats = _Stats()
        self.airtime = Airtime(baud)

    def start(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed; run: pip install pyserial")
        # write_timeout is essential: without it a stalled write (XBee not
        # draining the buffer — RF congestion, a flaky USB adapter, flow-control
        # stall) blocks forever and freezes the control loop that calls send().
        # With it, a stuck write raises SerialTimeoutException, caught in send().
        self._serial = serial.Serial(
            self.port, self.baud, timeout=0.2, write_timeout=0.2
        )
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, name="xbee-rx", daemon=True
        )
        self._dump_stats_thread = threading.Thread(
            target=self._dump_stats, name="dump_stats", daemon=True
        )
        self._thread.start()
        self._dump_stats_thread.start()

    def _dump_stats(self):
        while self._running:
            # flush=True because stdout is block-buffered when it is a journal
            # rather than a tty: without it these lines surface in ~45s batches
            # and a live link reads as a dead one.
            print(str(self._stats), flush=True)
            time.sleep(1)

    def _read_loop(self) -> None:
        buf = bytearray()
        st = self._stats
        last_line = None
        while self._running:
            # Nothing a single frame can do may kill this thread. It is the only
            # path by which a drive command or an e-stop reaches the robot, and
            # the process survives its death: telemetry keeps streaming from the
            # control-loop thread, so the base station still shows the rover
            # healthy while it has in fact stopped listening to anyone. That
            # failure is silent, unrecoverable without a restart, and it takes
            # the e-stop with it. One corrupt byte off the radio is not allowed
            # to cost that, so the whole body is guarded.
            try:
                t0 = time.perf_counter_ns()
                try:
                    chunk = self._serial.readline()
                except Exception as e:
                    print(f"[XBeeLink] read error: {e}")
                    continue
                t1 = time.perf_counter_ns()
                st.wait.append(t1 - t0)
                if not chunk:
                    continue
                buf.extend(chunk)
                if not chunk.endswith(b"\n"):
                    st.partials += 1
                    continue
                line = bytes(buf)
                buf.clear()

                t2 = time.perf_counter_ns()
                msg = protocol.decode(line)
                t3 = time.perf_counter_ns()
                st.decode.append(t3 - t2)
                if msg is None:
                    st.drops += 1
                    continue

                try:
                    self.on_message(msg)
                except Exception as e:
                    print(f"[XBeeLink] handler error: {e}")
                st.handler.append(time.perf_counter_ns() - t3)

                if last_line is not None:
                    st.gap.append(t3 - last_line)
                last_line = t3
            except Exception as e:
                # Never re-raise: see above. Drop the partial frame and carry on.
                print(f"[XBeeLink] rx loop error: {e!r}", flush=True)
                buf.clear()

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
