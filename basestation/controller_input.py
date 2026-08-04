"""PS4 (or any gamepad) input, read on a background thread via pygame.

Emits normalized (throttle, steer) at a fixed rate and fires named actions on
button presses. The app maps these to the *currently selected* robot, so one
controller flies whichever robot you've picked in the UI.

Runs headless (SDL dummy video driver) so it works on a Mac and on a Pi without
a display. Hot-plugging is handled: unplug/replug the controller and it
reconnects.

Controls (defaults): left stick Y = throttle, right stick X = steer. Setting
`axis_throttle` to UNBOUND swaps the throttle back onto the triggers (R2 =
forward, L2 = reverse) without touching steering.

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

AXIS_THROTTLE = 1   # left stick Y -> throttle
AXIS_STEER = 2      # right stick X -> steering
AXIS_L2 = 4         # L2 analog trigger -> reverse (fallback throttle)
AXIS_R2 = 5         # R2 analog trigger -> forward (fallback throttle)
BTN_ESTOP = 1       # circle
BTN_CLEAR = 0       # cross
BTN_TELEOP = 4      # L1
BTN_ALIGN = 5       # R1

# Value an untouched trigger reports. SDL scales triggers to -1 (released)
# .. +1 (fully pulled); set to 0.0 for drivers that report a plain 0..1.
TRIGGER_REST = -1.0

# How often a held mechanism control re-announces itself while it stays held.
# Paired with the robot's auto_stop_seconds: the robot stops a held mechanism
# that stops hearing this, so the refresh must be comfortably faster than that
# timeout, and slow enough that it costs the radio almost nothing next to drive
# frames at drive_hz.
HOLD_REFRESH_S = 0.25


def _dz(v, dz=0.08):
    return 0.0 if abs(v) < dz else v


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


def mix(m: ControllerMapping, axes, l2: "Trigger", r2: "Trigger"):
    """Turn one raw sample of the pad into the (throttle, steer) we transmit.

    Two-stick by default: `axis_throttle` names the left stick's Y and
    `axis_steer` the right stick's X, so a hand can hold a speed while the other
    works the steering. Leaving `axis_throttle` UNBOUND puts the throttle back on
    the triggers (R2 forward, L2 reverse, both = cancel) and changes nothing
    else.

    Either way the arcade->tank mixing happens on the ROBOT
    (`DriveCommand.arcade`); this only decides where throttle comes from, so
    every layout speaks the identical wire protocol.

    Pure, and separate from the poll loop, because this is the part worth
    testing and the loop around it needs a joystick to exist.
    """
    def axis(idx: int) -> float:
        return axes[idx] if (0 <= idx < len(axes)) else 0.0

    if m.axis_throttle is not None and m.axis_throttle >= 0:
        raw = axis(m.axis_throttle)
        # Sticks report UP as negative, so a stick throttle is inverted.
        if m.invert_throttle:
            raw = -raw
    else:
        raw = r2.value(axis(m.axis_r2)) - l2.value(axis(m.axis_l2))
    throttle = _expo(_dz(raw, m.deadzone), m.throttle_expo) * m.throttle_gain
    steer = _expo(_dz(axis(m.axis_steer), m.deadzone),
                  m.steer_expo) * m.steer_gain
    if m.invert_steer:
        steer = -steer
    return _clamp1(throttle), _clamp1(steer)


class ControllerReader:
    def __init__(self, hz: float = 40.0, mapping: Optional[ControllerMapping] = None):
        if pygame is None:
            raise RuntimeError("pygame not installed (pip install pygame)")
        self.hz = hz
        self.on_drive = None    # (throttle, steer) -> None
        self.on_action = None   # (name: str) -> None
        self.on_hold = None     # (name: str, on: bool) -> None
        self.connected = False
        self.name = None
        self._map = mapping or ControllerMapping()
        self._js = None
        self._prev = {}
        self._holding = {}        # action name -> currently held
        self._hold_refresh = {}   # action name -> last time we re-announced it
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

    def _held(self, idx: int) -> bool:
        """Current state of a button, without consuming an edge."""
        if self._js is None or idx < 0 or idx >= self._js.get_numbuttons():
            return False
        return bool(self._js.get_button(idx))

    def _hat_held(self, idx: int, direction) -> bool:
        if self._js is None or idx < 0 or idx >= self._js.get_numhats():
            return False
        return tuple(self._js.get_hat(idx)) == tuple(direction)

    def _pump_holds(self, held: dict) -> None:
        """Turn a held/not-held map into on/off callbacks for the robot.

        Sends on the press and on the release, and REPEATS the "on" a few times
        a second while the control stays held. The repeat is the point: the
        robot auto-stops a held mechanism that stops hearing from us, so a
        release frame lost on the radio costs a fraction of a second of extra
        running rather than a feeder that never stops. Rate-limited well below
        the poll rate because this shares airtime with drive frames.
        """
        now = time.monotonic()
        for name, on in held.items():
            was = self._holding.get(name, False)
            if on and not was:
                self._holding[name] = True
                self._hold_refresh[name] = now
                if self.on_hold:
                    self.on_hold(name, True)
            elif on and was:
                if now - self._hold_refresh.get(name, 0.0) >= HOLD_REFRESH_S:
                    self._hold_refresh[name] = now
                    if self.on_hold:
                        self.on_hold(name, True)
            elif was and not on:
                self._holding[name] = False
                if self.on_hold:
                    self.on_hold(name, False)

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
                axes = [self._axis(i, naxes) for i in range(naxes)]
                throttle, steer = mix(m, axes, self._l2, self._r2)
                if self.on_drive:
                    self.on_drive(throttle, steer)
                for idx, name in m.actions():
                    if self._edge(idx) and self.on_action:
                        self.on_action(name)
                # Run-while-held controls, buttons and hat directions alike.
                held = {name: self._held(idx) for idx, name in m.holds()}
                for hat, direction, name in m.hat_holds():
                    held[name] = self._hat_held(hat, direction)
                self._pump_holds(held)
            except Exception as e:
                print(f"[controller] error: {e}")
                self._js = None
                time.sleep(0.5)
            time.sleep(period)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
