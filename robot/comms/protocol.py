"""Wire protocol: newline-delimited JSON.

Simple, debuggable, and language-agnostic so the cross-platform base station
(Python on a Pi or a Mac) can speak it trivially. If you later move the XBee to
API mode / a binary framing, swap this module out; the rest of the stack only
depends on encode()/decode().

Message shapes (base station -> robot):
    {"type": "drive", "throttle": 0.5, "steer": -0.2}   # arcade mixing
    {"type": "drive", "left": 0.4, "right": 0.6}         # direct tank
    {"type": "mode", "mode": "teleop" | "object_align" | "shooter_align" | "waypoint"}
    {"type": "estop"}            # latch motors off
    {"type": "clear_estop"}      # release the latch
    {"type": "arm_shooter"}      # shooter_align: permit firing
    {"type": "disarm_shooter"}   # shooter_align: forbid firing, park the servo
    {"type": "fire"}             # shooter_align: manual shot, skips alignment

Arming is deliberately NOT sticky: it is dropped when the mode is left and when
the e-stop latches, so `clear_estop` alone never resumes a pending shot — the
operator must send `arm_shooter` again.

Robot -> base station (telemetry), e.g.:
    {"type": "telemetry", "mode": "teleop", "left": 0.4, "right": 0.6}
"""

from __future__ import annotations

import json
from typing import Optional


def encode(message: dict) -> bytes:
    return (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")


def decode(line: bytes) -> Optional[dict]:
    line = line.strip()
    if not line:
        return None
    try:
        msg = json.loads(line.decode("utf-8", errors="replace"))
    except (ValueError, UnicodeDecodeError):
        return None
    return msg if isinstance(msg, dict) else None
