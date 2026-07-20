"""PS4 (or any gamepad) input, read on a background thread via pygame.

Emits normalized (throttle, steer) at a fixed rate and fires named actions on
button presses. The app maps these to the *currently selected* robot, so one
controller flies whichever robot you've picked in the UI.

Runs headless (SDL dummy video driver) so it works on a Mac and on a Pi without
a display. Hot-plugging is handled: unplug/replug the controller and it
reconnects. Axis/button indices default to a typical DualShock 4 layout; adjust
the constants if your OS/driver differs.
"""

from __future__ import annotations

import os
import threading
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

try:
    import pygame
except Exception:  # pragma: no cover
    pygame = None

AXIS_THROTTLE = 1   # left stick Y (up is negative; we negate)
AXIS_STEER = 2      # right stick X
BTN_ESTOP = 1       # circle
BTN_CLEAR = 0       # cross
BTN_TELEOP = 4      # L1
BTN_ALIGN = 5       # R1


def _dz(v, dz=0.08):
    return 0.0 if abs(v) < dz else v


class ControllerReader:
    def __init__(self, hz: float = 40.0):
        if pygame is None:
            raise RuntimeError("pygame not installed (pip install pygame)")
        self.hz = hz
        self.on_drive = None    # (throttle, steer) -> None
        self.on_action = None   # (name: str) -> None
        self.connected = False
        self.name = None
        self._js = None
        self._prev = {}
        self._thread = None
        self._running = False

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="controller", daemon=True)
        self._thread.start()

    def _open(self) -> bool:
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.connected, self.name, self._js = False, None, None
            return False
        self._js = pygame.joystick.Joystick(0)
        self._js.init()
        self.connected = True
        self.name = self._js.get_name()
        print(f"[controller] connected: {self.name}")
        return True

    def _edge(self, idx: int) -> bool:
        if self._js is None or idx >= self._js.get_numbuttons():
            return False
        cur = self._js.get_button(idx)
        was = self._prev.get(idx, 0)
        self._prev[idx] = cur
        return bool(cur and not was)

    def _loop(self) -> None:
        pygame.init()
        period = 1.0 / self.hz
        while self._running:
            try:
                pygame.event.pump()
                if self._js is None or pygame.joystick.get_count() == 0:
                    self.connected = False
                    self._js = None
                    if not self._open():
                        time.sleep(1.0)
                        continue
                throttle = -_dz(self._js.get_axis(AXIS_THROTTLE))
                steer = _dz(self._js.get_axis(AXIS_STEER)) if self._js.get_numaxes() > AXIS_STEER else 0.0
                if self.on_drive:
                    self.on_drive(throttle, steer)
                for idx, name in ((BTN_ESTOP, "estop"), (BTN_CLEAR, "clear"),
                                  (BTN_TELEOP, "mode:teleop"), (BTN_ALIGN, "mode:object_align")):
                    if self._edge(idx) and self.on_action:
                        self.on_action(name)
            except Exception as e:
                print(f"[controller] error: {e}")
                self._js = None
                time.sleep(0.5)
            time.sleep(period)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
