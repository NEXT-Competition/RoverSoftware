"""PS4 (or any gamepad) input, read on a background thread via pygame.

Emits normalized (throttle, steer) at a fixed rate and fires named actions on
button presses. The app maps these to the *currently selected* robot, so one
controller flies whichever robot you've picked in the UI.

Runs headless (SDL dummy video driver) so it works on a Mac and on a Pi without
a display. Hot-plugging is handled: unplug/replug the controller and it
reconnects.

Controls (defaults): R2 = forward, L2 = reverse, right stick X = steer.

--- Why the layout is a setting, not a constant ---
Axis and button indices describe a *driver*, not a controller: the same pad
enumerates differently across macOS, Linux, and Bluetooth vs USB. They live in
a `ControllerMapping` (basestation/settings.py) that the dashboard can edit and
that persists, so re-binding a pad in the field is a tap rather than an ssh
session and a service restart. The module constants below remain the built-in
defaults.

The reader also publishes `state()` — the raw axes and buttons it is seeing
right now — which is what lets the settings page offer "press the button you
want" instead of asking an operator to guess an index.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

from .settings import ControllerMapping

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
    """Dead zone that RESCALES [dz, 1] back onto [0, 1].

    Passing v through unchanged above the threshold makes the output jump
    straight from 0.0 to dz the moment the stick leaves centre — with the
    default dead zone that is 8% of steering differential appearing in a single
    frame, which on a skid-steer chassis is a visible twitch every time you
    start a turn, and again every time you come back to centre.

    Rescaling removes the step without narrowing what is reachable: full stick
    still gives 1.0. It also means `deadzone` can be raised to cover a worn,
    drifting stick without that making the twitch worse, which is the tuning
    dead end the old version created.
    """
    if abs(v) < dz or dz >= 1.0:
        return 0.0
    return (v - dz if v > 0 else v + dz) / (1.0 - dz)


def _expo(v: float, e: float) -> float:
    """Bend an axis toward a cubic response, keeping the endpoints.

    out = (1-e)*v + e*v^3. Odd in v, so the sign is preserved and -1/0/+1 are
    fixed points: the curve only changes how much of the travel it takes to
    reach a given output, never the range that is reachable.
    """
    if e <= 0.0:
        return v
    return (1.0 - e) * v + e * v * v * v


def _clamp1(v):
    return -1.0 if v < -1.0 else 1.0 if v > 1.0 else v


class Trigger:
    """Normalize one analog trigger axis to 0..1, safely.

    The mapping itself is trivial -- rescale [rest, 1] onto [0, 1]. The reason
    this is a class is the arming latch: some drivers report a flat 0.0 for a
    trigger that has not been moved since the joystick was opened. Rescaled
    naively that resting 0.0 becomes 0.5, i.e. the robot pulls away at half
    throttle the instant a controller is plugged in. So a trigger stays
    disarmed, reporting 0.0, until it has been seen at rest at least once. On a
    well-behaved driver that happens on the very first sample (-1.0); on a
    broken one it happens the first time you pull the trigger and let go, and
    if it never happens the trigger simply stays dead. Fail stopped, not fast.

    `rest` is per-instance so the dashboard can retune it for an odd driver;
    it defaults to the module constant, which is what a DualShock reports.
    """

    def __init__(self, rest: float = TRIGGER_REST):
        self.rest = rest
        self.armed = False

    def reset(self) -> None:
        """Re-arm from scratch (call when a joystick is opened/replaced)."""
        self.armed = False

    def set_rest(self, rest: float) -> None:
        """Change the resting value, disarming so the new one must be proven."""
        if rest != self.rest:
            self.rest = rest
            self.armed = False

    def value(self, raw: float) -> float:
        if raw <= self.rest + 0.5:
            self.armed = True
        if not self.armed:
            return 0.0
        span = 1.0 - self.rest
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (raw - self.rest) / span))


class ControllerReader:
    def __init__(self, hz: float = 40.0, mapping: Optional[ControllerMapping] = None):
        if pygame is None:
            raise RuntimeError("pygame not installed (pip install pygame)")
        self.hz = hz
        self.on_drive = None    # (throttle, steer) -> None
        self.on_action = None   # (name: str) -> None
        self.connected = False
        self.name = None
        self._map = mapping or ControllerMapping()
        self._js = None
        self._prev = {}
        self._l2 = Trigger(self._map.trigger_rest)
        self._r2 = Trigger(self._map.trigger_rest)
        self._thread = None
        self._running = False
        # Raw sample published for the settings page's bind-by-pressing flow.
        self._lock = threading.Lock()
        self._axes: list = []
        self._buttons: list = []

    def set_mapping(self, mapping: ControllerMapping) -> None:
        """Swap the layout mid-run. Triggers re-arm if their rest value moved,
        so a mis-set rest can't leave a stale half-throttle latched in."""
        self._map = mapping
        self._l2.set_rest(mapping.trigger_rest)
        self._r2.set_rest(mapping.trigger_rest)

    def state(self) -> dict:
        """What the pad is reporting right now: raw axes and pressed buttons.

        Deliberately raw and unmapped — the point is to show an operator what
        their hardware emits so they can bind it, which pre-mapped values can't.
        """
        with self._lock:
            return {"connected": self.connected, "name": self.name,
                    "axes": list(self._axes), "buttons": list(self._buttons)}

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
        if self._js is None or idx < 0 or idx >= self._js.get_numbuttons():
            return False
        cur = self._js.get_button(idx)
        was = self._prev.get(idx, 0)
        self._prev[idx] = cur
        return bool(cur and not was)

    def _fire_actions(self, m: ControllerMapping) -> None:
        """Run every action whose button was just pressed.

        Edges are sampled ONCE per button, before anything is dispatched, and
        that is the whole point of this being a method. `_edge` *records* the
        press as it tests it, so asking it twice about one button within a tick
        answers False the second time — which meant a button carrying two
        actions only ever ran whichever `actions()` listed first, and the other
        binding was silently dead. Sharing a button is legal and is exactly what
        an operator does by accident when they bind a mechanism preset onto the
        cross that already clears the e-stop; nothing on the settings page could
        have told them the second one would never fire.
        """
        pressed = {idx for idx in {i for i, _ in m.actions()} if self._edge(idx)}
        if not self.on_action:
            return
        for idx, name in m.actions():
            if idx in pressed:
                self.on_action(name)

    def _axis(self, idx: int, naxes: int) -> float:
        return self._js.get_axis(idx) if (0 <= idx < naxes) else 0.0

    def _publish(self, naxes: int) -> None:
        nbtn = self._js.get_numbuttons()
        axes = [round(self._js.get_axis(i), 3) for i in range(naxes)]
        buttons = [bool(self._js.get_button(i)) for i in range(nbtn)]
        with self._lock:
            self._axes, self._buttons = axes, buttons

    def _loop(self) -> None:
        pygame.init()
        period = 1.0 / self.hz
        while self._running:
            try:
                pygame.event.pump()
                if self._js is None or pygame.joystick.get_count() == 0:
                    self.connected = False
                    self._js = None
                    with self._lock:
                        self._axes, self._buttons = [], []
                    if not self._open():
                        time.sleep(1.0)
                        continue
                m = self._map  # one read: the mapping can be swapped mid-tick
                naxes = self._js.get_numaxes()
                self._publish(naxes)
                if m.axis_throttle is not None and m.axis_throttle >= 0:
                    # Arcade drive on one stick. The mixing itself happens on
                    # the robot (DriveCommand.arcade), same as it does for the
                    # trigger path — this only decides where throttle comes
                    # from, so both layouts speak the identical wire protocol.
                    raw = self._axis(m.axis_throttle, naxes)
                    if m.invert_throttle:
                        raw = -raw
                    throttle = _expo(_dz(raw, m.deadzone),
                                     m.throttle_expo) * m.throttle_gain
                else:
                    r2 = self._r2.value(self._axis(m.axis_r2, naxes))
                    l2 = self._l2.value(self._axis(m.axis_l2, naxes))
                    # R2 forward, L2 reverse, both = cancel.
                    throttle = _expo(_dz(r2 - l2, m.deadzone),
                                     m.throttle_expo) * m.throttle_gain
                steer = _expo(_dz(self._axis(m.axis_steer, naxes), m.deadzone),
                              m.steer_expo) * m.steer_gain
                if m.invert_steer:
                    steer = -steer
                if self.on_drive:
                    self.on_drive(_clamp1(throttle), _clamp1(steer))
                self._fire_actions(m)
            except Exception as e:
                print(f"[controller] error: {e}")
                self._js = None
                time.sleep(0.5)
            time.sleep(period)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
