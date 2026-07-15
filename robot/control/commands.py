"""The normalized drive command every controller produces.

Teleop and all autonomy controllers speak this same little type, so the drive
layer never needs to know who generated a command.
"""

from __future__ import annotations

from dataclasses import dataclass


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


@dataclass(frozen=True)
class DriveCommand:
    """Left/right track speeds, each normalized to [-1.0, 1.0]."""

    left: float = 0.0
    right: float = 0.0

    @classmethod
    def tank(cls, left: float, right: float) -> "DriveCommand":
        return cls(_clamp(left), _clamp(right))

    @classmethod
    def arcade(cls, throttle: float, steer: float) -> "DriveCommand":
        """throttle = forward(+)/back(-), steer = right(+)/left(-)."""
        return cls(_clamp(throttle + steer), _clamp(throttle - steer))

    @classmethod
    def stopped(cls) -> "DriveCommand":
        return cls(0.0, 0.0)
