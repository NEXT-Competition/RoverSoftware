"""Base-station settings: gamepad mapping and link/UI rates, editable from the UI.

Two things live here, and they're together because they share a lifetime — both
belong to *this* base station, not to any robot, and both must survive a restart
of the service:

    controller.*  which axis is steer, which button is E-STOP, dead zone, ...
    base.*        radio airtime budget, dashboard refresh rate, basemap URL

Robot tuning (PID gains, drive limits, vision) is NOT here: it belongs to the
robot, is stored on the robot, and travels over the radio — see robot/tuning.py.
This module deliberately mirrors that one's shape (a whitelist of flat dotted
paths, clamped rather than refused) so the settings page treats both the same.

Defaults come from the CLI/env at startup, then the saved file is overlaid, so
an operator's saved choice wins over a flag but a flag still sets the baseline
for anything they never touched.
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, Optional, Tuple

from robot.tuning import Param, coerce

# Set by a button binding that is deliberately unassigned. Not None, because
# this crosses JSON and a nullable int in a form field is one more state for
# every consumer to special-case.
UNBOUND = -1

DEFAULT_PATH = "~/.config/roversoftware/basestation.json"


def settings_path() -> str:
    return os.path.expanduser(os.environ.get("RS_BASE_SETTINGS", DEFAULT_PATH))


@dataclass
class ControllerMapping:
    """Where the gamepad's controls are, and how they feel.

    Indices default to a DualShock 4 under SDL/pygame. They are settings rather
    than constants because "axis 2 is the right stick" is a claim about a driver,
    not about a controller: the same pad enumerates differently across macOS,
    Linux and a Bluetooth-vs-USB connection, and the fix used to be editing
    source on the base station.
    """
    # --- axes ---
    axis_steer: int = 2  # right stick X
    axis_l2: int = 4  # L2 analog trigger -> reverse
    axis_r2: int = 5  # R2 analog trigger -> forward
    # Arcade drive on a stick. UNBOUND keeps the R2-forward/L2-reverse trigger
    # behaviour; >= 0 names a stick axis for throttle and takes the drivetrain
    # off the triggers entirely, which frees them to be ordinary buttons.
    axis_throttle: int = UNBOUND
    # Sticks report UP as negative, so a stick throttle is inverted by default.
    # Triggers are not, which is why this is separate from invert_steer.
    invert_throttle: bool = True
    # Value an untouched trigger reports. SDL scales triggers to -1 (released)
    # .. +1 (fully pulled); 0.0 suits drivers that report a plain 0..1.
    trigger_rest: float = -1.0
    deadzone: float = 0.08
    invert_steer: bool = False
    # Authority scaling, applied after the dead zone. Lower steer_gain for a
    # long-wheelbase chassis that darts; lower throttle_gain as a trainer mode.
    steer_gain: float = 1.0
    throttle_gain: float = 1.0
    # Input curve, applied after the dead zone and before the gain. 0 is the
    # linear stick every build had before this existed; 1 is fully cubic.
    #
    # This is the knob for "too sensitive", and it is NOT the same as lowering
    # the gain. Gain scales everything, so it costs top speed to buy fine
    # control. Expo bends the middle of the travel down and leaves the endpoint
    # alone: at 0.6, half stick gives 0.28 instead of 0.5, while full stick
    # still gives 1.0. It also shrinks the step at the edge of the dead zone,
    # which is the other thing that reads as twitchy.
    steer_expo: float = 0.0
    throttle_expo: float = 0.0

    # --- buttons (UNBOUND disables) ---
    btn_estop: int = 1  # circle
    btn_clear: int = 0  # cross
    btn_teleop: int = 4  # L1
    btn_object_align: int = 5  # R1
    btn_shooter_align: int = UNBOUND
    btn_waypoint: int = UNBOUND
    btn_arm_shooter: int = UNBOUND
    btn_fire: int = UNBOUND
    # Manual teleop controls for the mechanisms, distinct from btn_fire: that
    # one is ShooterAlignController's armed/dwelled shot, these work while
    # driving.
    #
    # The flywheel and both intake directions TOGGLE (see actions()): they run
    # for long stretches and holding a button through a match is not a thing
    # anyone wants. The feeder and the agitator RUN WHILE HELD (see holds() and
    # hat_holds()), which is also what lets them carry a dead-man timeout —
    # a toggled mechanism cannot, because nothing is refreshing it.
    btn_shooter_spin: int = UNBOUND
    btn_intake: int = UNBOUND
    btn_intake_spit: int = UNBOUND
    btn_feeder: int = UNBOUND
    # Hat (DPAD) index driving the agitator: UP runs it, DOWN runs it backwards
    # at the same speed. Hats are not buttons — one hat reports a direction pair
    # — so these name the hat and the directions are fixed.
    hat_agitator: int = UNBOUND
    hat_agitator_rev: int = UNBOUND

    def actions(self):
        """(button index, action name) for every bound button.

        Action names are the vocabulary app.py::on_action already speaks, so a
        newly-bound button needs no new plumbing.
        """
        pairs = (
            (self.btn_estop, "estop"),
            (self.btn_clear, "clear"),
            (self.btn_teleop, "mode:teleop"),
            (self.btn_object_align, "mode:object_align"),
            (self.btn_shooter_align, "mode:shooter_align"),
            (self.btn_waypoint, "mode:waypoint"),
            (self.btn_arm_shooter, "arm_shooter"),
            (self.btn_fire, "fire"),
            (self.btn_shooter_spin, "shooter_spin"),
            (self.btn_intake, "intake"),
            (self.btn_intake_spit, "intake_spit"),
        )
        return tuple((idx, name) for idx, name in pairs if idx is not None and idx >= 0)

    def holds(self):
        """(button index, mechanism action) for every RUN-WHILE-HELD button.

        Separate from actions() because these need the release as well as the
        press: app.py answers them with an explicit on/off rather than a toggle,
        so a dropped frame cannot leave the mechanism inverted. The robot also
        auto-stops them if the refresh stops arriving, which is what makes a
        lost release frame safe rather than merely unlikely.
        """
        pairs = (
            (self.btn_feeder, "feeder"),
        )
        return tuple((idx, name) for idx, name in pairs if idx is not None and idx >= 0)

    def hat_holds(self):
        """(hat index, direction, action) for run-while-held hat directions."""
        pairs = ((self.hat_agitator, (0, 1), "agitator"),
                 (self.hat_agitator_rev, (0, -1), "agitator_rev"))
        return tuple((i, d, n) for i, d, n in pairs if i is not None and i >= 0)


@dataclass
class BaseSettings:
    # An airtime budget, not a feel knob — see run_basestation.py --drive-hz.
    drive_hz: float = 15.0
    ui_hz: float = 30.0  # dashboard refresh pushed to the browser
    video_hz: float = 20.0  # max MJPEG frame rate served to browsers
    controller_hz: float = 40.0  # gamepad poll rate (restart to change)
    tiles: str = ""  # basemap URL template the browser loads
    trail_max: int = 400  # breadcrumb points kept per robot


# The whitelist. `live=False` means the value is stored and shown as pending but
# only takes effect on the next start (the gamepad thread's poll rate is fixed
# when the thread starts).
PARAMS: Tuple[Param, ...] = (
    Param("base.drive_hz", "float", lo=1, hi=60),
    Param("base.ui_hz", "float", lo=1, hi=60),
    Param("base.video_hz", "float", lo=1, hi=60),
    Param("base.controller_hz", "float", lo=5, hi=120, live=False),
    Param("base.tiles", "text"),
    Param("base.trail_max", "int", lo=0, hi=5000),

    Param("controller.axis_steer", "int", lo=0, hi=15),
    Param("controller.axis_l2", "int", lo=0, hi=15),
    Param("controller.axis_r2", "int", lo=0, hi=15),
    Param("controller.trigger_rest", "float", lo=-1, hi=1),
    Param("controller.deadzone", "float", lo=0, hi=0.5),
    Param("controller.invert_steer", "bool"),
    # Floored at 0.1 rather than 0: a gain slider that can reach zero is a
    # "why won't it drive" bug report waiting to happen, and 10% is already a
    # slower trainer mode than anyone wants.
    Param("controller.steer_gain", "float", lo=0.1, hi=1),
    Param("controller.throttle_gain", "float", lo=0.1, hi=1),
    Param("controller.btn_estop", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_clear", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_teleop", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_object_align", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_shooter_align", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_waypoint", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_arm_shooter", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_fire", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_shooter_spin", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_intake", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_intake_spit", "int", lo=UNBOUND, hi=31),
    Param("controller.btn_feeder", "int", lo=UNBOUND, hi=31),
    Param("controller.hat_agitator", "int", lo=UNBOUND, hi=3),
    Param("controller.hat_agitator_rev", "int", lo=UNBOUND, hi=3),
    Param("controller.axis_throttle", "int", lo=UNBOUND, hi=15),
    Param("controller.invert_throttle", "bool"),
    Param("controller.steer_expo", "float", lo=0, hi=1),
    Param("controller.throttle_expo", "float", lo=0, hi=1),
)

BY_PATH: Dict[str, Param] = {p.path: p for p in PARAMS}


class SettingsStore:
    """Thread-safe holder for the two dataclasses above, backed by a JSON file.

    Read from the web loop, written from a WebSocket handler, and read by the
    gamepad thread on every poll — hence the lock. `on_change` fires after a
    successful apply so the app can re-push anything it caches.
    """

    def __init__(self, defaults: Optional[Dict[str, Any]] = None,
                 path: Optional[str] = None, load: bool = True):
        self.path = path or settings_path()
        self.base = BaseSettings()
        self.controller = ControllerMapping()
        self._lock = threading.RLock()
        self.on_change: Optional[Callable[[Dict[str, Any]], None]] = None
        if defaults:
            self._write(defaults)  # CLI/env baseline
        if load:
            saved = self._read_file()
            if saved:
                applied = self._write(saved)
                print(f"[base] settings: {len(applied)} saved values from {self.path}")

    # --- internals ----------------------------------------------------------

    def _owner(self, path: str):
        group, name = path.split(".", 1)
        return (self.base if group == "base" else self.controller), name

    def _write(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        """Coerce and store; returns what was actually applied. Caller locks."""
        applied: Dict[str, Any] = {}
        for path, value in updates.items():
            param = BY_PATH.get(path)
            if param is None:
                continue
            try:
                coerced = coerce(param, value)
            except (ValueError, TypeError):
                continue
            obj, name = self._owner(path)
            setattr(obj, name, coerced)
            applied[path] = coerced
        return applied

    def _read_file(self) -> Dict[str, Any]:
        """Saved settings, or {} — never raises. A corrupt file must leave the
        base station launchable on its defaults, not dead before a match."""
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return {}
        except Exception as e:
            print(f"[base] ignoring unreadable {self.path}: {e}")
            return {}
        if not isinstance(data, dict):
            print(f"[base] ignoring {self.path}: expected an object")
            return {}
        return {k: v for k, v in data.items() if k in BY_PATH}

    def _save(self) -> Optional[str]:
        """Write the whole current state atomically. Returns an error string on
        failure — a read-only home directory must not undo a change that has
        already taken effect in memory."""
        tmp = f"{self.path}.tmp"
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(self.snapshot(), fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
            return None
        except Exception as e:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            return f"{e}"

    # --- public -------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        """Every setting, keyed by flat dotted path (the wire/UI shape)."""
        with self._lock:
            flat = {f"base.{k}": v for k, v in asdict(self.base).items()}
            flat.update({f"controller.{k}": v for k, v in asdict(self.controller).items()})
            return {k: v for k, v in flat.items() if k in BY_PATH}

    def mapping(self) -> ControllerMapping:
        """A stable copy for the gamepad thread to read without holding the lock."""
        with self._lock:
            return ControllerMapping(**asdict(self.controller))

    def apply(self, updates: Dict[str, Any], save: bool = True) -> Dict[str, Any]:
        """Apply `updates` and persist. Returns a result dict for the client."""
        if not isinstance(updates, dict):
            return {"applied": {}, "rejected": {"*": "expected an object"}, "save_error": None}
        rejected: Dict[str, str] = {}
        for path, value in updates.items():
            param = BY_PATH.get(path)
            if param is None:
                rejected[str(path)] = "unknown setting"
                continue
            try:
                coerce(param, value)
            except (ValueError, TypeError) as e:
                rejected[path] = str(e) or "invalid value"
        with self._lock:
            applied = self._write({k: v for k, v in updates.items() if k not in rejected})
            error = self._save() if (applied and save) else None
        if error:
            print(f"[base] settings applied but NOT saved: {error}")
        if applied and self.on_change:
            self.on_change(applied)
        restart = sorted(p for p in applied if not BY_PATH[p].live)
        return {"applied": applied, "rejected": rejected,
                "restart": restart, "save_error": error}
