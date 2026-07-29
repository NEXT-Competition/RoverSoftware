"""Tank drive: normalized left/right track speeds -> the ESCs.

The implementation now lives in `drivetrain.py`, which generalizes it to any
number of motors per side and to the other drivetrain kinds a layout can pick.
`TankDrive` remains as the name the tools, the tests and anything that predates
layouts import — it is the tank drivetrain, constructed from a `DriveConfig`
exactly as before, with the same `arm()` / `drive(left, right)` / `stop()`
surface and the same slew-rate limiting.
"""

from __future__ import annotations

from .drivetrain import TankDrivetrain

TankDrive = TankDrivetrain

__all__ = ["TankDrive"]
