"""Tank drive: takes normalized left/right track speeds and drives the ESCs.

An optional slew-rate limiter smooths abrupt command changes so you don't slam
the drivetrain (and to be gentler on ESCs/batteries).
"""

from __future__ import annotations

import time

from ..config import DriveConfig
from .motor import ESCMotor


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


def _slew(current, target, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class TankDrive:
    def __init__(self, config: DriveConfig):
        self.cfg = config
        self.left_1 = ESCMotor(config.left_1)
        self.left_2 = ESCMotor(config.left_2)
        self.right_1 = ESCMotor(config.right_1)
        self.right_2 = ESCMotor(config.right_2)
        self._cur_left = 0.0
        self._cur_right = 0.0
        self._last_update = None

    def arm(self) -> None:
        """Hold both ESCs at neutral so they arm before we accept commands."""
        self.left_1.stop()
        self.left_2.stop()
        self.right_1.stop()
        self.right_2.stop()
        if self.cfg.arm_seconds > 0:
            time.sleep(self.cfg.arm_seconds)

    def drive(self, left: float, right: float) -> None:
        """Command normalized track speeds in [-1, 1]."""
        left = _clamp(left)
        right = _clamp(right)

        now = time.monotonic()
        if self.cfg.slew_rate > 0 and self._last_update is not None:
            max_step = self.cfg.slew_rate * (now - self._last_update)
            left = _slew(self._cur_left, left, max_step)
            right = _slew(self._cur_right, right, max_step)
        self._last_update = now

        self._cur_left = left
        self._cur_right = right
        self.left_1.set_throttle(left)
        self.left_2.set_throttle(left)
        self.right_1.set_throttle(right)
        self.right_2.set_throttle(right)

    def stop(self) -> None:
        self._cur_left = 0.0
        self._cur_right = 0.0
        self.left_1.stop()
        self.left_2.stop()
        self.right_1.stop()
        self.right_2.stop()
