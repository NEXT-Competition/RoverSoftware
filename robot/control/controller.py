"""Base class for anything that can drive the robot.

Teleop, object alignment, and waypoint navigation all subclass Controller, so
the ControlManager can switch between them without knowing their internals.
"""

from __future__ import annotations

from typing import Dict, Optional

from .commands import DriveCommand


class Controller:
    name = "controller"

    def on_activate(self) -> None:
        """Called when this controller becomes the active mode."""

    def on_deactivate(self) -> None:
        """Called when another controller takes over."""

    def on_message(self, message: dict) -> None:
        """Handle an inbound base-station message routed to the active mode."""

    def on_estop(self) -> None:
        """Called on EVERY controller when the e-stop latches.

        Stopping the motors is the manager's job and needs no cooperation — it
        simply stops calling update(). This hook exists for state that must not
        survive an e-stop, which for a drive-only controller is nothing, but for
        an armed actuator is the whole point: clearing the e-stop must not
        resume a shot that was pending when it was pressed.

        Broadcast to all controllers, not just the active one, so a latch armed
        in one mode can't be inherited by switching modes after the stop.
        """

    def update(self, dt: float) -> Optional[DriveCommand]:
        """Return the desired drive command, or None to hold/stop.

        Called every control tick. `dt` is seconds since the last tick.
        """
        raise NotImplementedError

    def pid_traces(self) -> Dict[str, dict]:
        """Named traces of whatever closed loops this controller is running.

        `{loop_name: PID.trace(...)}`, empty for a controller with no loop —
        which is most of them, and why this is a base-class default rather than
        an interface everything has to implement. Read only for the ACTIVE
        controller, and only to be graphed: a loop nobody is running has nothing
        to say about how it is behaving.
        """
        return {}
