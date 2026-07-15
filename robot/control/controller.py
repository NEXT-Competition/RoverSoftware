"""Base class for anything that can drive the robot.

Teleop, color-alignment, and waypoint navigation all subclass Controller, so
the ControlManager can switch between them without knowing their internals.
"""

from __future__ import annotations

from typing import Optional

from .commands import DriveCommand


class Controller:
    name = "controller"

    def on_activate(self) -> None:
        """Called when this controller becomes the active mode."""

    def on_deactivate(self) -> None:
        """Called when another controller takes over."""

    def on_message(self, message: dict) -> None:
        """Handle an inbound base-station message routed to the active mode."""

    def update(self, dt: float) -> Optional[DriveCommand]:
        """Return the desired drive command, or None to hold/stop.

        Called every control tick. `dt` is seconds since the last tick.
        """
        raise NotImplementedError
