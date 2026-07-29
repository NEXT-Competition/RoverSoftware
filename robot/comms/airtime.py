"""How many bytes the radio can actually take right now.

A serial link has a hard ceiling: 57600 baud is 5760 bytes a second and no
amount of queueing changes that. Both ends of this system used to pace bulk
transfers in FRAMES per tick, which sounds like pacing but isn't — the robot's
two frames per 50 Hz tick is 100 frames a second, and at ~430 bytes a frame
that is 43 kB/s handed to a link that can carry 5.8. The kernel buffer fills,
`serial.write` hits its timeout, and XBeeLink drops the frame to keep the
control loop alive. What gets dropped is a fragment of a config snapshot or a
layout, nothing retries it, and the settings page stays blank forever.

So bulk senders ask this first. It is a token bucket denominated in bytes,
refilled at the line rate, and a `False` means "not now" rather than "lost" —
the caller keeps the frame at the head of its queue and offers it again next
tick. Realtime frames (drive, telemetry, e-stop) never ask; they go out
immediately and are `debit`ed afterwards, so bulk gives way to them instead of
competing with them.
"""

from __future__ import annotations

import time

# 8N1 framing: every byte on the wire costs a start bit and a stop bit as well
# as its eight data bits.
BITS_PER_BYTE = 10

# Share of the line bulk transfers may claim. The remainder is headroom for the
# traffic that has to be current — drive frames down, telemetry up — which is
# sent without asking and would otherwise queue behind a document. At 57600
# this leaves bulk ~3.4 kB/s: a 2.4 KB config snapshot in about 0.7 s, which is
# the difference between a settings page that fills while you look at it and
# one that never fills at all.
BULK_SHARE = 0.6

# Biggest single write we hand the port. One doc_transfer fragment plus its
# envelope fits comfortably; the point is that the bucket never saves up enough
# credit to burst several frames into a buffer that then can't drain in time.
BURST_BYTES = 512

# Below this, the arithmetic stops describing anything real (nobody runs an
# XBee at 300 baud, and a typo in a config file shouldn't wedge every transfer).
MIN_BAUD = 1200


class Airtime:
    """A byte budget for one link, refilled at the link's own line rate."""

    def __init__(self, baud: int, share: float = BULK_SHARE,
                 burst: int = BURST_BYTES, now: float | None = None):
        self.rate = max(float(baud), MIN_BAUD) / BITS_PER_BYTE * share
        self.burst = burst
        # Starts full: the first fragment of a transfer should go out at once.
        # The bucket only bites once a sender is running ahead of the radio.
        self._tokens = float(burst)
        self._last = time.monotonic() if now is None else now

    def _refill(self, now: float | None) -> None:
        now = time.monotonic() if now is None else now
        # A clock that went backwards (or a caller passing stale times) must not
        # mint credit, so elapsed time is floored at zero.
        elapsed = max(0.0, now - self._last)
        self._last = now
        self._tokens = min(self.burst, self._tokens + elapsed * self.rate)

    def take(self, size: int, now: float | None = None) -> bool:
        """Claim `size` bytes of airtime, or answer False and claim nothing.

        False is not an error. It means the link is already carrying as much as
        it can, and the frame should stay queued.
        """
        self._refill(now)
        if self._tokens < size:
            return False
        self._tokens -= size
        return True

    def debit(self, size: int, now: float | None = None) -> None:
        """Charge for a frame that went out without asking.

        Realtime traffic is never held back, but it does consume the same line,
        so it is charged after the fact. The balance is allowed to go negative:
        that is exactly the interval during which bulk transfers wait, which is
        how a config dump stops being the reason telemetry stutters.
        """
        self._refill(now)
        self._tokens -= size
