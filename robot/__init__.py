"""RoverSoftware robot package.

A modular tank-drive robot stack:

    hardware (ESC via servo)  <-  drive mixing  <-  control (mode arbitration)
                                                        ^
                                          comms (XBee)  |  teleop / autonomy
                                                        |  controllers

Everything the robot does flows through the ControlManager, which picks one
active Controller (teleop today; object-align and waypoint autonomy later) and
sends its DriveCommand to the tank drive.
"""

from .config import RobotConfig
from .robot import Robot

__all__ = ["Robot", "RobotConfig"]
