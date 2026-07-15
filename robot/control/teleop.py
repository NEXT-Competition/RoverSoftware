"""Teleoperation: drive from commands the base station sends over XBee."""

from __future__ import annotations

import time
from typing import Optional

from .commands import DriveCommand
from .controller import Controller


class TeleopController(Controller):
    name = "teleop"

    def __init__(self, command_timeout: float = 0.5):
        self.command_timeout = command_timeout
        self._command = DriveCommand.stopped()
        self._last_rx = 0.0

    def on_activate(self) -> None:
        self._command = DriveCommand.stopped()
        self._last_rx = 0.0

    def on_message(self, message: dict) -> None:
        if message.get("type") != "drive":
            return
        if "left" in message and "right" in message:
            self._command = DriveCommand.tank(float(message["left"]), float(message["right"]))
        else:
            self._command = DriveCommand.arcade(
                float(message.get("throttle", 0.0)),
                float(message.get("steer", 0.0)),
            )
        self._last_rx = time.monotonic()

    def update(self, dt: float) -> Optional[DriveCommand]:
        # Link failsafe: if commands stop arriving, stop the robot.
        if self._last_rx == 0.0 or (time.monotonic() - self._last_rx) > self.command_timeout:
            return DriveCommand.stopped()
        return self._command
