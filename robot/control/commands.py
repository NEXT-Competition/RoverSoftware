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
        """throttle = forward(+)/back(-), steer = right(+)/left(-).

        Scaled to fit, not clamped per side. Clamping each track on its own
        changes the RATIO the operator asked for, and only once the sum
        saturates: at throttle 0.8 / steer 0.4 the outer track pins at 1.0
        while the inner keeps 0.4, so pushing the stick further stops
        tightening the turn and the rover gains speed mid-corner instead.
        That handover — linear response up to some throttle-dependent point,
        then steering that fades — is what reads as an unsmooth stick.

        Dividing both sides by the overshoot keeps left/right proportional, so
        the commanded turn radius survives at any throttle; only the pair's
        magnitude gives way. Below saturation `m` is 1.0 and this is identical
        to the old clamp, so nothing changes for gentle inputs.
        """
        left, right = throttle + steer, throttle - steer
        m = max(1.0, abs(left), abs(right))
        return cls(left / m, right / m)

    @classmethod
    def stopped(cls) -> "DriveCommand":
        return cls(0.0, 0.0)
