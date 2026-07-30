"""The robot's tunable-parameter surface: what the base station may change.

`RobotConfig` is a plain nest of dataclasses, which is the right shape for code
but the wrong shape for a remote settings page: a browser must not be able to
poke arbitrary attributes on it, and a slider needs to know a field's range and
whether changing it does anything before the next reboot.

So this module is the whitelist. Every parameter the UI can touch is declared
once in `PARAMS` with its type, bounds and whether it applies live, and the only
two entry points are:

    snapshot(cfg)          -> {"drive.slew_rate": 4.0, ...}  flat, dotted paths
    apply(cfg, updates)    -> (applied, rejected)

Anything not in `PARAMS` is rejected, out-of-range numbers are clamped rather
than refused (a slider that silently does nothing is worse than one that stops
at its limit), and a bad type is reported instead of raising — a malformed frame
off the radio must never take the robot down.

--- live vs restart ---
`live=True` means the running robot picks the value up without being restarted.
That is a claim about the *code*, not a wish: it holds either because the
consumer reads `cfg` on every use (the motors, the shooter servo, the detector)
or because `Robot._push_live_config` copies it into the object that cached it
(the controllers, the IMU, the GPS). `live=False` is for anything owned by a
constructor — serial ports, I2C addresses, PWM channels, enable flags — which is
stored and shown as pending, but only takes effect on the next start.

Flat dotted paths, not a nested dict, because the wire is a 57600-baud radio
shared with telemetry: an update carries only the handful of fields that
actually changed, and a full snapshot is ~2.5 KB (~0.4 s of airtime), which is
why the base station asks for one explicitly instead of polling.

The UI mirrors this list in TypeScript for labels/grouping
(basestation-ui/src/settings/schema.ts) exactly as net/types.ts mirrors the /ws
contract. Keep the two in sync; this file is the one that decides.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

from .config import RobotConfig

# Where UI-set values are persisted on the robot so tuning survives a reboot.
# The postinst creates /var/lib/roversoftware owned by the service user; the
# env var lets a dev run point it somewhere writable.
DEFAULT_OVERRIDES_PATH = "/var/lib/roversoftware/tuning.json"


def overrides_path() -> str:
    return os.environ.get("RS_TUNING_FILE", DEFAULT_OVERRIDES_PATH)


@dataclass(frozen=True)
class Param:
    """One tunable field: where it lives, what it accepts, when it applies."""
    path: str  # dotted path into RobotConfig, e.g. "drive.left.deadband"
    kind: str  # float | int | bool | enum | text
    lo: Optional[float] = None
    hi: Optional[float] = None
    choices: Optional[Sequence[str]] = None
    live: bool = True
    # --- presentation, for parameters the UI cannot know about in advance ---
    # The static list below is mirrored by hand in the dashboard's schema.ts,
    # which is where labels and help text live for everything a build always
    # has. Parameters generated from a user's own layout have no such mirror,
    # so the robot describes them itself (see `descriptors`). Trailing and
    # defaulted so `from robot.tuning import Param` in basestation/settings.py
    # keeps constructing these positionally.
    label: str = ""
    unit: str = ""
    step: Optional[float] = None
    help: str = ""
    group: str = ""  # UI grouping key, e.g. "actuator:left" / "mech:intake"


def _f(path, lo, hi, live=True, **extra):
    return Param(path, "float", lo=lo, hi=hi, live=live, **extra)


def _i(path, lo, hi, live=True, **extra):
    return Param(path, "int", lo=lo, hi=hi, live=live, **extra)


def _b(path, live=True, **extra):
    return Param(path, "bool", live=live, **extra)


def _e(path, choices, live=True, **extra):
    return Param(path, "enum", choices=tuple(choices), live=live, **extra)


def _t(path, live=True, **extra):
    return Param(path, "text", live=live, **extra)


# One actuator's calibration, for any dotted prefix. The two stock track motors
# reach this through `_motor` below; a layout's extra motors and every
# mechanism's actuators reach it through `params_for`. One definition, so a
# field added here shows up everywhere an actuator does.
def _actuator_params(prefix: str, group: str = "", label: str = "") -> Tuple[Param, ...]:
    # Endpoints are the Fusion HAT servo range. neutral_angle is deliberately
    # allowed anywhere in it: this rover's ESC stops at +5, not 0.
    def meta(field: str, **kw) -> dict:
        return {"group": group, "label": f"{label} {field}".strip(), **kw} if group else {}

    return (
        _b(f"{prefix}.inverted", **meta("inverted")),
        _f(f"{prefix}.neutral_angle", -90, 90, **meta("neutral angle", unit="deg", step=0.5)),
        _f(f"{prefix}.max_angle", -90, 90, **meta("forward endpoint", unit="deg", step=0.5)),
        _f(f"{prefix}.min_angle", -90, 90, **meta("reverse endpoint", unit="deg", step=0.5)),
        _f(f"{prefix}.deadband", 0, 0.5, **meta("dead band", step=0.005)),
        _f(f"{prefix}.max_forward", 0, 1, **meta("forward cap", step=0.01)),
        _f(f"{prefix}.max_reverse", 0, 1, **meta("reverse cap", step=0.01)),
        _i(f"{prefix}.channel", 0, 15, live=False, **meta("PWM channel")),
    )


def _motor(side: str) -> Tuple[Param, ...]:
    """The two stock track motors. Their paths are `drive.left.*`/`drive.right.*`
    because that is what the default layout names them — the general rule for a
    drive actuator is `drive.<name>.*`, and these are just the default names."""
    return _actuator_params(f"drive.{side}")


# Everything a build always has, regardless of layout. `PARAMS` below appends
# the two stock track motors; `params_for(cfg)` appends whatever THIS robot's
# layout actually declares. Keeping them separate is what lets the default
# layout produce exactly `PARAMS` — see the note there.
_BASE_PARAMS: Tuple[Param, ...] = (
    # --- loop / identity ---
    # loop_hz is live because run() recomputes its period every tick; below
    # ~20 Hz the slew limiter stops interpolating and teleop turns choppy.
    _f("loop_hz", 1, 200),
    _f("telemetry_hz", 0, 20),
    _e("heading_source", ("auto", "gps", "imu")),
    _e("start_mode", ("teleop", "object_align", "shooter_align", "waypoint", "routine"), live=False),
    _t("robot_id", live=False),

    # --- comms ---
    # Below ~0.2 s the failsafe trips between drive frames on a slow link; it
    # must stay above the base station's 0.25 s keepalive.
    _f("comms.command_timeout", 0.05, 5),
    _t("comms.port", live=False),
    _i("comms.baud", 1200, 921600, live=False),
    # WiFi bulk transfers. These two are BOOTSTRAP_PATHS below: the one pair of
    # settings that still travels over the radio, because they are how you point
    # a rover at a base station it cannot reach yet. Live — `Robot` redials the
    # link on the spot (_retarget_ip_link), which is the whole point: a change
    # that needed a service restart to take effect would mean walking out to the
    # rover, and then you may as well have edited robot.env.
    _t("comms.base_host"),
    _i("comms.base_port", 1, 65535),

    # --- drive ---
    _f("drive.slew_rate", 0, 20),
    _f("drive.arm_seconds", 0, 10, live=False),

    # --- object / shooter alignment ---
    _f("align.forward_speed", 0, 1),
    _f("align.pivot_threshold", 0, 1),
    _f("align.aligned_tolerance", 0, 0.5),
    _f("align.search_after", 0, 10),
    _f("align.search_timeout", 0, 120),
    _f("align.pid.kp", 0, 5),
    _f("align.pid.ki", 0, 5),
    _f("align.pid.kd", 0, 5),
    _f("align.pid.out_limit", 0, 1),
    _f("align.pid.i_limit", 0, 5),

    # --- waypoint navigation ---
    _f("nav.arrive_radius_m", 0.2, 50),
    _f("nav.cruise_speed", 0, 1),
    _f("nav.acquire_speed", 0, 1),
    _f("nav.pivot_threshold_deg", 1, 180),
    _f("nav.heading_pid.kp", 0, 5),
    _f("nav.heading_pid.ki", 0, 5),
    _f("nav.heading_pid.kd", 0, 5),
    _f("nav.heading_pid.out_limit", 0, 1),
    _f("nav.heading_pid.i_limit", 0, 180),
    # The slower loop used when heading is the GPS track angle, not the IMU.
    _f("nav.gps_heading_pid.kp", 0, 5),
    _f("nav.gps_heading_pid.ki", 0, 5),
    _f("nav.gps_heading_pid.kd", 0, 5),
    _f("nav.gps_heading_pid.out_limit", 0, 1),
    _f("nav.gps_heading_pid.i_limit", 0, 180),
    # Graph the loops while you tune them. Costs radio airtime on every frame,
    # so it is a switch rather than something that is simply always on.
    _b("nav.pid_trace"),

    # --- vision ---
    _f("vision.min_confidence", 0, 1),
    _f("vision.max_fps", 0.5, 60),
    _f("vision.standoff_size", 0.05, 1),
    _f("vision.hfov_deg", 10, 180),
    _f("vision.search_speed", 0, 1),
    _f("vision.target_timeout", 0.1, 10),
    _t("vision.target_label"),
    _e("vision.select", ("largest", "confidence", "centermost")),
    _b("vision.enabled", live=False),
    _e("vision.backend", ("auto", "edge_impulse", "imx500"), live=False),
    _t("vision.model_path", live=False),
    _t("vision.imx500_model", live=False),
    _f("vision.imx500_iou", 0, 1, live=False),
    _i("vision.imx500_max_detections", 1, 100, live=False),

    # --- camera ---
    _b("camera.enabled", live=False),
    _t("camera.device", live=False),
    _i("camera.width", 64, 4096, live=False),
    _i("camera.height", 64, 4096, live=False),
    _i("camera.fps", 1, 120, live=False),

    # --- shooter ---
    _f("shooter.rest_angle", -90, 90),
    _f("shooter.fire_angle", -90, 90),
    _f("shooter.fire_seconds", 0.05, 5),
    _f("shooter.retract_seconds", 0.05, 5),
    _f("shooter.dwell", 0, 10),
    _f("shooter.cooldown", 0, 60),
    _i("shooter.max_shots", 0, 999),
    _b("shooter.require_arm"),
    _b("shooter.require_arrived"),
    _b("shooter.enabled", live=False),
    _i("shooter.channel", 0, 15, live=False),

    # --- GPS ---
    _f("gps.fix_timeout", 0.5, 60),
    _f("gps.min_move_mps", 0, 5),
    _b("gps.enabled", live=False),
    _t("gps.port", live=False),
    _i("gps.baud", 1200, 921600, live=False),
    _i("gps.update_rate_ms", 100, 10000, live=False),

    # --- IMU ---
    _f("imu.heading_offset_deg", -180, 180),
    _b("imu.invert"),
    _i("imu.min_calib", 0, 3),
    _b("imu.persist_calibration"),
    _b("imu.enabled", live=False),
    _i("imu.i2c_address", 0x08, 0x77, live=False),

    # --- routines (the FSM engine; the documents live in routines.json) ---
    _f("routines.state_timeout_default", 1, 600),
    # Off by default and worth keeping that way: this is the one action a
    # user-authored program can take that makes something physically launch.
    # Even on, arming is only accepted inside a state that drives with
    # shooter_align, and is dropped on every state exit, mode exit and e-stop.
    _b("routines.allow_arm", live=False),

    # --- FPV ---
    _i("fpv.fps", 1, 60),
    _i("fpv.jpeg_quality", 1, 100),
    # All live. The feed is the one thing an operator wants to change from the
    # far side of a field: switch it on to see what the rover is looking at,
    # switch it off when the WiFi is the thing that is struggling, and re-aim it
    # at whichever laptop is running the base station today — an address the
    # robot can only learn over the radio. Needing a service restart for any of
    # that is how you end up with a rover you cannot see out of, at exactly the
    # moment you cannot go and get it. `fpv.enabled` reaches
    # `FPVStreamer.start()`/`stop()`, which opens the camera on demand; the host
    # and port rebuild the socket on the next frame. See sensors/fpv.py.
    _b("fpv.enabled"),
    _t("fpv.base_host"),
    _i("fpv.base_port", 1, 65535),
)

# The stock robot's parameter surface: everything above, plus the two track
# motors the default layout declares. Still a module constant, because it is
# what a build ships with and what the dashboard's schema.ts mirrors.
PARAMS: Tuple[Param, ...] = _BASE_PARAMS + _motor("left") + _motor("right")

BY_PATH: Dict[str, Param] = {p.path: p for p in PARAMS}

# --- the bootstrap exception -------------------------------------------------
#
# Configuration is bulk traffic and rides the WiFi link (robot/comms/ip_link.py),
# not the shared radio. These two paths are the exception, and they have to be:
# they are the address of that link. A rover pointed at the wrong base station —
# or at none at all, which is what a fresh install is — has no WiFi path to be
# told the right one over, so a chicken-and-egg is the alternative to letting
# this one pair through on the radio. It is ~60 bytes, sent by hand, once.
#
# Deliberately narrow. `is_bootstrap` demands that a frame carry NOTHING ELSE,
# so a set_config that slips a PID gain in beside a hostname is not a bootstrap
# frame and does not get a free ride on the radio.
BOOTSTRAP_PATHS: Tuple[str, ...] = ("comms.base_host", "comms.base_port")


def is_bootstrap(values) -> bool:
    """True if `values` is a non-empty set of BOOTSTRAP_PATHS and nothing else."""
    if not isinstance(values, dict) or not values:
        return False
    return all(path in BOOTSTRAP_PATHS for path in values)


# --- this robot's actual parameter surface ----------------------------------
#
# `PARAMS` is what a build ships with. A robot that has been given a layout
# (robot/layout.py) has a different set: its own actuators, named whatever the
# operator named them, plus a parameter block per mechanism. Those paths can't
# be written down in advance, so they are derived from the config instead.
#
# THE COMPATIBILITY CONTRACT, in one line:
#
#     {p.path for p in params_for(RobotConfig())} == {p.path for p in PARAMS}
#
# The default layout names its two actuators `left` and `right`, so the derived
# paths ARE `drive.left.*` and `drive.right.*`. Every deployed tuning.json, the
# dashboard's hand-written schema.ts mirror, and the two tests that assert on
# the snapshot's exact key set all keep working without knowing any of this
# happened. tests/test_tuning_dynamic.py asserts that equality directly.

def _mechanism_params(mech) -> Tuple[Param, ...]:
    """One mechanism's tunables: its own geometry plus each actuator's."""
    prefix = f"mech.{mech.name}"
    group = f"mech:{mech.name}"
    label = mech.label or mech.name.replace("_", " ")
    out: List[Param] = []
    if mech.kind == "pulse":
        out += [
            _f(f"{prefix}.rest_angle", -90, 90, group=group,
               label=f"{label} rest angle", unit="deg", step=0.5),
            _f(f"{prefix}.active_angle", -90, 90, group=group,
               label=f"{label} active angle", unit="deg", step=0.5),
            _f(f"{prefix}.active_seconds", 0.05, 5, group=group,
               label=f"{label} active time", unit="s", step=0.05),
            _f(f"{prefix}.recover_seconds", 0.05, 5, group=group,
               label=f"{label} recover time", unit="s", step=0.05),
            _f(f"{prefix}.cooldown", 0, 60, group=group,
               label=f"{label} cooldown", unit="s", step=0.1),
            _i(f"{prefix}.max_activations", 0, 999, group=group,
               label=f"{label} magazine", help="0 = unlimited"),
        ]
    else:
        out += [
            _f(f"{prefix}.auto_stop_seconds", 0, 300, group=group,
               label=f"{label} auto-stop", unit="s", step=0.1,
               help="Stop this long after being told to run. 0 = run until stopped."),
        ]
    out.append(_b(f"{prefix}.enabled", live=False, group=group,
                  label=f"{label} enabled"))
    for aname, actuator in mech.actuators.items():
        out += list(_actuator_params(
            f"{prefix}.{aname}", group=group,
            label=f"{label} {actuator.label or aname.replace('_', ' ')}"))
    return tuple(out)


def params_for(cfg: RobotConfig) -> Tuple[Param, ...]:
    """Every parameter THIS robot exposes, given the layout it is running."""
    out: List[Param] = list(_BASE_PARAMS)
    for name, actuator in cfg.drive.actuators.items():
        out += list(_actuator_params(
            f"drive.{name}", group=f"actuator:{name}",
            label=actuator.label or name.replace("_", " ")))
    for mech in cfg.mechanisms.values():
        out += list(_mechanism_params(mech))
    return tuple(out)


def by_path_for(cfg: RobotConfig) -> Dict[str, Param]:
    return {p.path: p for p in params_for(cfg)}


def descriptors(cfg: RobotConfig) -> List[dict]:
    """Field metadata for the parameters schema.ts cannot know about.

    The dashboard mirrors `PARAMS` by hand to supply labels, units and help
    text. It cannot mirror an actuator the operator invented ten seconds ago, so
    the robot describes those itself. Only the DERIVED parameters are described
    — the ~90 static ones are already in schema.ts, and restating them would put
    2 KB on a radio for something the browser already has.
    """
    static = {p.path for p in _BASE_PARAMS}
    out = []
    for p in params_for(cfg):
        if p.path in static:
            continue
        out.append({
            "path": p.path, "kind": p.kind, "lo": p.lo, "hi": p.hi,
            "choices": list(p.choices) if p.choices else None,
            "live": p.live, "label": p.label, "unit": p.unit,
            "step": p.step, "help": p.help, "group": p.group,
        })
    return out


# --- reading / writing a RobotConfig by path --------------------------------

def _resolve(cfg: RobotConfig, path: str):
    """Return (owner_object, attribute_name) for a dotted path.

    Plain attribute walking, with one special case: `mech.<name>...` indexes
    RobotConfig.mechanisms, which is a dict rather than a field per mechanism
    (there is no fixed set of them to declare). Drive actuators need no such
    case — `drive.<name>` is served by DriveConfig.__getattr__, which is the
    whole reason `drive.left.deadband` still resolves.
    """
    parts = path.split(".")
    obj: Any = cfg
    if parts[0] == "mech":
        try:
            obj = cfg.mechanisms[parts[1]]
        except (KeyError, IndexError):
            raise AttributeError(f"no mechanism named {path.split('.')[1]!r}") from None
        parts = parts[2:]
        if len(parts) == 2:  # mech.<name>.<actuator>.<field>
            try:
                obj = obj.actuators[parts[0]]
            except KeyError:
                raise AttributeError(f"no actuator named {parts[0]!r}") from None
            parts = parts[1:]
    for part in parts[:-1]:
        obj = getattr(obj, part)
    return obj, parts[-1]


def get(cfg: RobotConfig, path: str) -> Any:
    obj, name = _resolve(cfg, path)
    return getattr(obj, name)


def snapshot(cfg: RobotConfig) -> Dict[str, Any]:
    """Every tunable parameter's current value, keyed by dotted path.

    Derived from THIS robot's layout, so a build with an intake reports its
    intake. On the stock tank layout this is exactly the `PARAMS` key set.
    """
    return {p.path: get(cfg, p.path) for p in params_for(cfg)}


# Parameters per config frame. A full snapshot is ~2.4 KB, which is ~420 ms of
# wire time at 57600 — and XBeeLink writes with a 0.2 s write_timeout and DROPS
# the frame if the radio isn't draining. One big write is therefore exactly the
# shape that vanishes under congestion, leaving the settings page permanently
# blank. Sized so a frame stays a few hundred bytes.
CHUNK_SIZE = 16


def chunks(values: Dict[str, Any], size: int = CHUNK_SIZE):
    """Split a snapshot into radio-sized pieces.

    Costs nothing, because the base station already merges partial payloads —
    that is how it absorbs a set_config acknowledgement. The settings form just
    fills in as the pieces land.
    """
    items = sorted(values.items())
    for start in range(0, len(items), size):
        yield dict(items[start:start + size])


def _clamp(v: float, lo: Optional[float], hi: Optional[float]) -> float:
    if lo is not None and v < lo:
        return lo
    if hi is not None and v > hi:
        return hi
    return v


def coerce(param: Param, value: Any) -> Any:
    """Normalize a wire value to the parameter's type, or raise ValueError.

    Numbers are clamped into range; a slider pinned at its limit is honest,
    whereas silently dropping the update looks like a broken UI.
    """
    if param.kind == "bool":
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        raise ValueError("expected a boolean")
    if param.kind == "int":
        if isinstance(value, bool) or not isinstance(value, (int, float, str)):
            raise ValueError("expected a number")
        return int(round(_clamp(float(value), param.lo, param.hi)))
    if param.kind == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float, str)):
            raise ValueError("expected a number")
        return float(_clamp(float(value), param.lo, param.hi))
    if param.kind == "enum":
        text = str(value).strip()
        if param.choices and text not in param.choices:
            raise ValueError(f"expected one of {', '.join(param.choices)}")
        return text
    return str(value)


def apply(cfg: RobotConfig, updates: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, str]]:
    """Write `updates` (dotted path -> value) into `cfg`.

    Returns (applied, rejected). Applied values are the coerced/clamped ones
    actually stored, so the caller can echo the truth back to the operator
    rather than what they asked for.
    """
    applied: Dict[str, Any] = {}
    rejected: Dict[str, str] = {}
    if not isinstance(updates, dict):
        return applied, {"*": "expected an object of path -> value"}
    # Built per call rather than cached: this runs only when a config frame
    # arrives, never in a control tick, and a stale map after a layout change
    # would silently reject the operator's own actuators.
    known = by_path_for(cfg)
    for path, value in updates.items():
        param = known.get(path)
        if param is None:
            rejected[str(path)] = "unknown parameter"
            continue
        try:
            coerced = coerce(param, value)
        except (ValueError, TypeError) as e:
            rejected[path] = str(e) or "invalid value"
            continue
        obj, name = _resolve(cfg, path)
        setattr(obj, name, coerced)
        applied[path] = coerced
    return applied, rejected


def needs_restart(paths, params: Optional[Dict[str, Param]] = None) -> list:
    """Which of these applied paths won't take effect until the robot restarts.

    `params` defaults to the static surface so a caller that has no config to
    hand still gets a useful answer; `Robot` passes `by_path_for(self.cfg)` so a
    layout's own actuators are covered too.
    """
    params = BY_PATH if params is None else params
    return sorted(p for p in paths if p in params and not params[p].live)


# --- persistence ------------------------------------------------------------

def load_overrides(path: Optional[str] = None,
                   known: Optional[Dict[str, Param]] = None) -> Dict[str, Any]:
    """Read the saved tuning file, or {} if there isn't one / it's unreadable.

    Never raises: a corrupt tuning file must leave the robot bootable on its
    compiled-in defaults, not bricked at the side of a field.

    `known` is the parameter surface to filter against, defaulting to the static
    one. `run_robot.py` passes this robot's layout-derived surface, so a saved
    `drive.left.deadband` still applies on a tank build and is correctly dropped
    on one whose layout no longer has an actuator called `left`.
    """
    path = path or overrides_path()
    known = BY_PATH if known is None else known
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return {}
    except Exception as e:
        print(f"[tuning] ignoring unreadable {path}: {e}")
        return {}
    if not isinstance(data, dict):
        print(f"[tuning] ignoring {path}: expected an object")
        return {}
    return {k: v for k, v in data.items() if k in known}


def save_overrides(values: Dict[str, Any], path: Optional[str] = None) -> Optional[str]:
    """Persist `values`, merged over whatever was already saved.

    Written to a temp file and renamed so a power cut mid-write can't leave a
    half-written config the robot would refuse on next boot. Returns an error
    string on failure (the caller reports it; a read-only filesystem must not
    stop the live tuning that already took effect).
    """
    path = path or overrides_path()
    merged = load_overrides(path)
    merged.update(values)
    tmp = f"{path}.tmp"
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(merged, fh, indent=2, sort_keys=True)
            fh.write("\n")
        os.replace(tmp, path)
        return None
    except Exception as e:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        return f"{e}"
