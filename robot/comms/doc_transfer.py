"""Sending a JSON document over the radio in pieces, and putting it back.

`tuning.chunks` already splits a config snapshot into radio-sized frames, and
the base station merges the pieces as they land. That works because a snapshot
is a flat map of independent scalars: half of one is a valid smaller one.

A layout is a tree. Half a tree is not a smaller tree — it is a robot with one
drive motor and no second one, or a mechanism whose actuators haven't arrived
yet. Partially applying that is exactly how two motors end up on one channel. So
documents are not merged: the JSON text is sliced into numbered fragments and
nothing is applied until every fragment is in hand.

    split(doc, "layout", "rover1", "L4")  ->  frames with txid/seq/n/part
    Reassembler().feed(frame)             ->  the document, or None

Bounded on purpose. One transfer at a time per receiver, a timeout, and a hard
cap on the reassembled size: this buffer is fed by anything that can put bytes
on a shared radio, and it must not be a way to make a rover run out of memory.
"""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Iterator, Optional

# Fragment size. Frames stay a few hundred bytes including the envelope, which
# is what XBeeLink can reliably write inside its 0.2 s write timeout.
PART_CHARS = 400

# How long a half-finished transfer is kept before it is abandoned. Long enough
# to ride out a congested radio, short enough that a dropped fragment doesn't
# leave the next attempt merging into the wreckage of the last one.
TIMEOUT_S = 5.0

# Ceiling on a reassembled document, independently of what any caller validates.
MAX_CHARS = 32768


def split(doc: Any, mtype: str, sender: Optional[str] = None,
          txid: str = "0", size: int = PART_CHARS, **extra) -> Iterator[dict]:
    """Yield the frames that carry `doc`.

    Every frame names the transfer and its position in it, so a receiver can
    tell a retry from a continuation without any handshake.
    """
    text = json.dumps(doc, separators=(",", ":"))
    parts = [text[i:i + size] for i in range(0, len(text), size)] or [""]
    for seq, part in enumerate(parts):
        frame = {"type": mtype, "txid": txid, "seq": seq, "n": len(parts),
                 "part": part, **extra}
        if sender is not None:
            frame["from"] = sender
        yield frame


class Reassembler:
    """Collects the fragments of one document. Never raises.

    A new txid abandons whatever was in progress. That is the right call for a
    retry — the sender gave up on the old attempt too — and it keeps the whole
    thing to a single buffer.
    """

    def __init__(self, timeout: float = TIMEOUT_S, max_chars: int = MAX_CHARS):
        self.timeout = timeout
        self.max_chars = max_chars
        self._txid: Optional[str] = None
        self._parts: Dict[int, str] = {}
        self._n = 0
        self._started = 0.0
        self.error: Optional[str] = None

    def reset(self) -> None:
        self._txid = None
        self._parts = {}
        self._n = 0
        self._started = 0.0

    def feed(self, msg: dict, now: Optional[float] = None) -> Optional[dict]:
        """Absorb one frame. Returns the document once it is complete."""
        self.error = None
        now = time.monotonic() if now is None else now
        if not isinstance(msg, dict):
            return None

        txid = str(msg.get("txid", "0"))
        try:
            seq = int(msg.get("seq", 0))
            n = int(msg.get("n", 1))
        except (TypeError, ValueError):
            self.error = "malformed fragment header"
            return None
        part = msg.get("part", "")
        if not isinstance(part, str) or n < 1 or seq < 0 or seq >= n:
            self.error = "malformed fragment header"
            return None

        stale = self._txid is not None and (now - self._started) > self.timeout
        if txid != self._txid or stale:
            if stale and txid == self._txid:
                self.error = "previous transfer timed out"
            self._txid = txid
            self._parts = {}
            self._n = n
            self._started = now

        self._parts[seq] = part
        if sum(len(p) for p in self._parts.values()) > self.max_chars:
            self.error = f"document exceeds {self.max_chars} characters"
            self.reset()
            return None

        if len(self._parts) < self._n:
            return None  # still waiting; apply nothing

        text = "".join(self._parts[i] for i in range(self._n))
        self.reset()
        try:
            doc = json.loads(text)
        except Exception as e:
            self.error = f"fragments did not reassemble into JSON: {e}"
            return None
        if not isinstance(doc, dict):
            self.error = "expected an object"
            return None
        return doc

    @property
    def pending(self) -> bool:
        return self._txid is not None
