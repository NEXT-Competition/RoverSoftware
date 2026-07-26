"""PS4 (or any gamepad) input, read on a background thread via pygame.

Emits normalized (throttle, steer) at a fixed rate and fires named actions on
button presses. The app maps these to the *currently selected* robot, so one
controller flies whichever robot you've picked in the UI.

Runs headless (SDL dummy video driver) so it works on a Mac and on a Pi without
a display. Hot-plugging is handled: unplug/replug the controller and it
reconnects. Axis/button indices default to a typical DualShock 4 layout; adjust
the constants if your OS/driver differs.

Controls: R2 = forward, L2 = reverse, right stick X = steer.
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

AXIS_STEER = 2      # right stick X
AXIS_L2 = 4         # L2 analog trigger -> reverse
AXIS_R2 = 5         # R2 analog trigger -> forward
BTN_ESTOP = 1       # circle
BTN_CLEAR = 0       # cross
BTN_TELEOP = 4      # L1
BTN_ALIGN = 5       # R1

# Value an untouched trigger reports. SDL scales triggers to -1 (released)
# .. +1 (fully pulled); set to 0.0 for drivers that report a plain 0..1.
TRIGGER_REST = -1.0


def _dz(v, dz=0.08):
    return 0.0 if abs(v) < dz else v


class Trigger:
    """Normalize one analog trigger axis to 0..1, safely.

    The mapping itself is trivial -- rescale [TRIGGER_REST, 1] onto [0, 1].
    The reason this is a class is the arming latch: some drivers report a flat
    0.0 for a trigger that has not been moved since the joystick was opened.
    Rescaled naively that resting 0.0 becomes 0.5, i.e. the robot pulls away at
    half throttle the instant a controller is plugged in. So a trigger stays
    disarmed, reporting 0.0, until it has been seen at rest at least once. On a
    well-behaved driver that happens on the very first sample (-1.0); on a
    broken one it happens the first time you pull the trigger and let go, and
    if it never happens the trigger simply stays dead. Fail stopped, not fast.
    """

    def __init__(self):
        self.armed = False

    def reset(self) -> None:
        """Re-arm from scratch (call when a joystick is opened/replaced)."""
        self.armed = False

    def value(self, raw: float) -> float:
        if raw <= TRIGGER_REST + 0.5:
            self.armed = True
        if not self.armed:
            return 0.0
        span = 1.0 - TRIGGER_REST
        return max(0.0, min(1.0, (raw - TRIGGER_REST) / span))


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
        self._l2 = Trigger()
        self._r2 = Trigger()
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
        # A fresh device has fresh triggers: make them prove they're at rest.
        self._l2.reset()
        self._r2.reset()
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
                naxes = self._js.get_numaxes()
                r2 = self._r2.value(self._js.get_axis(AXIS_R2)) if naxes > AXIS_R2 else 0.0
                l2 = self._l2.value(self._js.get_axis(AXIS_L2)) if naxes > AXIS_L2 else 0.0
                throttle = _dz(r2 - l2)  # R2 forward, L2 reverse, both = cancel
                steer = _dz(self._js.get_axis(AXIS_STEER)) if naxes > AXIS_STEER else 0.0
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
