"""Mode arbitration: route base-station messages and pick the active controller.

This is the single place that decides *who* is driving. Mode switches and the
emergency stop are handled here; everything else is forwarded to the active
controller.
"""

from __future__ import annotations

from typing import Dict

from .commands import DriveCommand
from .controller import Controller


class ControlManager:
    def __init__(self, controllers: Dict[str, Controller], start_mode: str):
        if start_mode not in controllers:
            raise ValueError(f"Unknown start mode: {start_mode!r}")
        self.controllers = controllers
        self.mode = start_mode
        self.estop = False
        self.controllers[start_mode].on_activate()

    @property
    def active(self) -> Controller:
        return self.controllers[self.mode]

    def set_mode(self, mode: str) -> None:
        if mode not in self.controllers:
            print(f"[ControlManager] ignoring unknown mode: {mode!r}")
            return
        if mode == self.mode:
            return
        self.active.on_deactivate()
        self.mode = mode
        self.active.on_activate()
        print(f"[ControlManager] mode -> {mode}")

    def handle_message(self, message: dict) -> None:
        mtype = message.get("type")
        if mtype == "estop":
            self.estop = True
            print("[ControlManager] E-STOP engaged")
        elif mtype == "clear_estop":
            self.estop = False
            print("[ControlManager] E-STOP cleared")
        elif mtype == "mode":
            self.set_mode(message.get("mode", self.mode))
        else:
            # drive commands, waypoints, target updates, etc.
            self.active.on_message(message)

    def update(self, dt: float) -> DriveCommand:
        if self.estop:
            return DriveCommand.stopped()
        cmd = self.active.update(dt)
        return cmd if cmd is not None else DriveCommand.stopped()
