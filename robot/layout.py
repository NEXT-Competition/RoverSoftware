"""The robot's hardware layout: which actuators exist and what they belong to.

`robot/tuning.py` is the surface for SCALARS — a gain, a limit, a timeout, each
at a fixed dotted path. That model can't express "this build has three intake
motors", because the set of paths itself depends on the answer. So structure
lives here, in its own versioned JSON document:

    layout.json   what actuators exist, the drivetrain kind, the mechanisms
    tuning.json   what their numbers currently are

The split is not tidiness. The two are edited at completely different cadences
(a layout changes when someone reaches for a screwdriver; a gain changes every
few minutes on a field), they have different failure semantics (a bad layout
stops the rover driving, a bad gain makes it drive badly), and they get separate
revision counters so saving one doesn't push the other over the radio.

--- structure never hot-swaps ---
Applying a layout to a RUNNING robot is not supported and is not an oversight.
Actuators are owned by constructors: swapping them mid-loop means destroying and
rebuilding `Servo` objects while the drivetrain is armed, which is how an ESC
ends up holding an undefined pulse width. A layout is stored, validated, and
answered with "restart to apply" — the same contract as today's `live=False`
tuning fields, which include every PWM channel for exactly this reason.

--- nothing here raises ---
Same rule as tuning.py. A layout arrives off a radio or out of a file someone
hand-edited at 2am; it is validated into (doc, errors, warnings) and a bad one
leaves the robot on its previous layout, bootable and drivable.
"""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field, fields as dataclass_fields
from typing import Any, Dict, List, Optional, Tuple

from .config import (
    DriveConfig,
    DriveRoles,
    MechanismConfig,
    MotorConfig,
    RobotConfig,
)
from .tuning import _actuator_params, coerce

DEFAULT_LAYOUT_PATH = "/var/lib/roversoftware/layout.json"

VERSION = 1

# Caps. The radio is the binding constraint, not memory: a layout crosses a
# 57600-baud link shared with telemetry, so a document that can't be sent is
# worse than one that was refused with a message.
MAX_DRIVE_ACTUATORS = 16
MAX_MECHANISMS = 8
MAX_ACTUATORS_PER_MECHANISM = 8
MAX_PRESETS = 8
MAX_DOC_BYTES = 6144
CHANNELS = range(0, 16)  # Fusion HAT PWM channels

DRIVE_KINDS = ("tank", "servo_steer", "single", "none")
MECHANISM_KINDS = ("power", "pulse")
ACTUATOR_KINDS = ("esc", "servo")

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,23}$")

# An actuator called `slew_rate` would be unreachable through
# `DriveConfig.__getattr__`, because the real field wins. Rather than let that
# be a silent mystery, reject the name.
_RESERVED_DRIVE_NAMES = frozenset(f.name for f in dataclass_fields(DriveConfig))
# The built-in launcher keeps its own ShooterConfig and its own `shooter.*`
# tuning paths; a mechanism of the same name would give the FSM two things
# answering to "shooter".
_RESERVED_MECHANISM_NAMES = frozenset({"shooter", "drive"})

# Bounds for an actuator's numeric fields, taken from the single definition in
# tuning.py so a layout and a tuning slider can never disagree about a limit.
_ACTUATOR_PARAMS = {
    p.path.rsplit(".", 1)[-1]: p for p in _actuator_params("x")
}


def layout_path() -> str:
    return os.environ.get("RS_LAYOUT_FILE", DEFAULT_LAYOUT_PATH)


# --- serializing a config out -----------------------------------------------

def _actuator_doc(m: MotorConfig) -> dict:
    return {
        "name": m.name,
        "label": m.label,
        "kind": m.kind,
        "channel": m.channel,
        "inverted": m.inverted,
        "neutral_angle": m.neutral_angle,
        "max_angle": m.max_angle,
        "min_angle": m.min_angle,
        "deadband": m.deadband,
        "max_forward": m.max_forward,
        "max_reverse": m.max_reverse,
        "speed_scale": m.speed_scale,
    }


def _mechanism_doc(mech: MechanismConfig) -> dict:
    doc = {
        "name": mech.name,
        "label": mech.label,
        "kind": mech.kind,
        "enabled": mech.enabled,
        "actuators": [_actuator_doc(a) for a in mech.actuators.values()],
    }
    if mech.kind == "pulse":
        doc.update(
            rest_angle=mech.rest_angle,
            active_angle=mech.active_angle,
            active_seconds=mech.active_seconds,
            recover_seconds=mech.recover_seconds,
            cooldown=mech.cooldown,
            max_activations=mech.max_activations,
        )
    else:
        doc.update(presets=mech.presets, auto_stop_seconds=mech.auto_stop_seconds,
                   slew_rate=mech.slew_rate)
    return doc


def to_doc(cfg: RobotConfig) -> dict:
    """This robot's current layout, as the document the dashboard edits."""
    d = cfg.drive
    return {
        "version": VERSION,
        "drive": {
            "kind": d.kind,
            "actuators": [_actuator_doc(a) for a in d.actuators.values()],
            "roles": {
                "left": list(d.roles.left),
                "right": list(d.roles.right),
                "throttle": list(d.roles.throttle),
                "steer": d.roles.steer,
            },
            "arm_seconds": d.arm_seconds,
            "slew_rate": d.slew_rate,
            "steer_gain": d.steer_gain,
            "min_pivot_throttle": d.min_pivot_throttle,
        },
        "mechanisms": [_mechanism_doc(m) for m in cfg.mechanisms.values()],
    }


def default_doc() -> dict:
    """The stock two-motor tank layout — what a robot with no layout.json runs.

    Built from RobotConfig() rather than written out by hand, so it cannot drift
    away from the compiled-in defaults it is supposed to describe.
    """
    return to_doc(RobotConfig())


# --- validation --------------------------------------------------------------

@dataclass
class Validated:
    """A layout that parsed. `errors` empty means it is safe to apply."""
    drive: DriveConfig
    mechanisms: Dict[str, MechanismConfig] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _check_name(name: Any, what: str, errors: List[str]) -> str:
    text = str(name or "").strip()
    if not _NAME_RE.match(text):
        errors.append(
            f"{what} name {text!r} is invalid: use lower-case letters, digits "
            "and underscores, starting with a letter, up to 24 characters")
        return ""
    return text


def _actuator_from_doc(raw: Any, what: str, errors: List[str]) -> Optional[MotorConfig]:
    if not isinstance(raw, dict):
        errors.append(f"{what}: expected an object")
        return None
    name = _check_name(raw.get("name"), what, errors)
    if not name:
        return None

    kind = str(raw.get("kind", "esc")).strip()
    if kind not in ACTUATOR_KINDS:
        errors.append(f"{what} {name!r}: unknown kind {kind!r} "
                      f"(expected one of {', '.join(ACTUATOR_KINDS)})")
        return None

    try:
        channel = int(raw.get("channel"))
    except (TypeError, ValueError):
        errors.append(f"{what} {name!r}: channel must be a whole number")
        return None
    if channel not in CHANNELS:
        errors.append(f"{what} {name!r}: channel {channel} is outside "
                      f"{CHANNELS.start}-{CHANNELS.stop - 1}")
        return None

    motor = MotorConfig(channel=channel, name=name, kind=kind,
                        label=str(raw.get("label", "") or ""))
    # Numbers are CLAMPED, not refused — same rule as tuning.apply. A field
    # pinned at its limit is honest; dropping the whole layout because one
    # endpoint was typed a degree too far is not.
    for fname, param in _ACTUATOR_PARAMS.items():
        if fname == "channel" or fname not in raw:
            continue
        try:
            setattr(motor, fname, coerce(param, raw[fname]))
        except (ValueError, TypeError) as e:
            errors.append(f"{what} {name!r}: {fname}: {e}")
    return motor


def _actuator_map(raw: Any, what: str, cap: int,
                  errors: List[str]) -> Dict[str, MotorConfig]:
    out: Dict[str, MotorConfig] = {}
    if not isinstance(raw, list):
        errors.append(f"{what}: actuators must be a list")
        return out
    if len(raw) > cap:
        errors.append(f"{what}: {len(raw)} actuators, at most {cap} allowed")
        return out
    for entry in raw:
        motor = _actuator_from_doc(entry, what, errors)
        if motor is None:
            continue
        if motor.name in out:
            errors.append(f"{what}: duplicate actuator name {motor.name!r}")
            continue
        out[motor.name] = motor
    return out


def _validate_roles(kind: str, raw: Any, actuators: Dict[str, MotorConfig],
                    errors: List[str]) -> DriveRoles:
    raw = raw if isinstance(raw, dict) else {}

    def names(key: str) -> List[str]:
        value = raw.get(key, [])
        if isinstance(value, str):
            value = [value]
        if not isinstance(value, list):
            errors.append(f"drive.roles.{key}: expected a list of actuator names")
            return []
        out = []
        for n in value:
            n = str(n)
            if n not in actuators:
                errors.append(f"drive.roles.{key}: no actuator named {n!r}")
            elif n in out:
                errors.append(f"drive.roles.{key}: {n!r} listed twice")
            else:
                out.append(n)
        return out

    roles = DriveRoles(left=names("left"), right=names("right"),
                       throttle=names("throttle"), steer=str(raw.get("steer", "") or ""))

    if roles.steer and roles.steer not in actuators:
        errors.append(f"drive.roles.steer: no actuator named {roles.steer!r}")
        roles.steer = ""

    if kind == "tank":
        if not roles.left or not roles.right:
            errors.append("drive: a tank drivetrain needs at least one actuator "
                          "on each of left and right")
        overlap = set(roles.left) & set(roles.right)
        if overlap:
            errors.append("drive: " + ", ".join(sorted(overlap))
                          + " cannot be on both sides")
    elif kind == "servo_steer":
        if not roles.throttle:
            errors.append("drive: a servo_steer drivetrain needs at least one "
                          "throttle actuator")
        if not roles.steer:
            errors.append("drive: a servo_steer drivetrain needs a steering servo")
        elif roles.steer in roles.throttle:
            errors.append(f"drive: {roles.steer!r} cannot both steer and drive")
        elif actuators[roles.steer].kind != "servo":
            # Not fatal — an ESC-kind actuator still maps a value onto an angle
            # — but it is almost always a mis-set kind, and it changes arming.
            pass
    elif kind == "single":
        if not roles.throttle:
            errors.append("drive: a single drivetrain needs at least one "
                          "throttle actuator")
    return roles


def _validate_mechanism(raw: Any, errors: List[str],
                        warnings: List[str]) -> Optional[MechanismConfig]:
    if not isinstance(raw, dict):
        errors.append("mechanisms: expected an object")
        return None
    name = _check_name(raw.get("name"), "mechanism", errors)
    if not name:
        return None
    if name in _RESERVED_MECHANISM_NAMES:
        errors.append(
            f"mechanism {name!r}: that name is reserved for the built-in "
            "launcher — pick another one")
        return None

    kind = str(raw.get("kind", "power")).strip()
    if kind not in MECHANISM_KINDS:
        errors.append(f"mechanism {name!r}: unknown kind {kind!r} "
                      f"(expected one of {', '.join(MECHANISM_KINDS)})")
        return None

    what = f"mechanism {name!r}"
    actuators = _actuator_map(raw.get("actuators"), what,
                              MAX_ACTUATORS_PER_MECHANISM, errors)
    if not actuators:
        errors.append(f"{what}: needs at least one actuator")
        return None

    mech = MechanismConfig(
        name=name,
        label=str(raw.get("label", "") or ""),
        kind=kind,
        enabled=bool(raw.get("enabled", True)),
        actuators=actuators,
    )

    if kind == "pulse":
        for fname, lo, hi in (("rest_angle", -90, 90), ("active_angle", -90, 90),
                              ("active_seconds", 0.05, 5), ("recover_seconds", 0.05, 5),
                              ("cooldown", 0, 60)):
            if fname in raw:
                try:
                    setattr(mech, fname, _clamp_float(raw[fname], lo, hi))
                except (TypeError, ValueError):
                    errors.append(f"{what}: {fname} must be a number")
        if "max_activations" in raw:
            try:
                mech.max_activations = max(0, min(999, int(raw["max_activations"])))
            except (TypeError, ValueError):
                errors.append(f"{what}: max_activations must be a whole number")
        if len(actuators) > 1:
            warnings.append(f"{what}: a pulse mechanism drives all "
                            f"{len(actuators)} of its actuators together")
    else:
        presets = raw.get("presets", {})
        if not isinstance(presets, dict):
            errors.append(f"{what}: presets must be an object")
        elif len(presets) > MAX_PRESETS:
            errors.append(f"{what}: {len(presets)} presets, at most "
                          f"{MAX_PRESETS} allowed")
        else:
            for pname, values in presets.items():
                pname = _check_name(pname, f"{what} preset", errors)
                if not pname:
                    continue
                if not isinstance(values, dict):
                    errors.append(f"{what}: preset {pname!r} must map actuator "
                                  "names to values")
                    continue
                clean: Dict[str, float] = {}
                for act, value in values.items():
                    if act not in actuators:
                        errors.append(f"{what}: preset {pname!r} names unknown "
                                      f"actuator {act!r}")
                        continue
                    try:
                        clean[act] = _clamp_float(value, -1.0, 1.0)
                    except (TypeError, ValueError):
                        errors.append(f"{what}: preset {pname!r}: {act} must be "
                                      "a number")
                mech.presets[pname] = clean
        if "auto_stop_seconds" in raw:
            try:
                mech.auto_stop_seconds = _clamp_float(raw["auto_stop_seconds"], 0, 300)
            except (TypeError, ValueError):
                errors.append(f"{what}: auto_stop_seconds must be a number")
        if "slew_rate" in raw:
            try:
                mech.slew_rate = _clamp_float(raw["slew_rate"], 0, 20)
            except (TypeError, ValueError):
                errors.append(f"{what}: slew_rate must be a number")

    return mech


def _clamp_float(value: Any, lo: float, hi: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError("expected a number")
    v = float(value)
    return lo if v < lo else hi if v > hi else v


def _resolve_channels(drive: DriveConfig, mechanisms: Dict[str, MechanismConfig],
                      reserved: Dict[int, str], errors: List[str]) -> None:
    """One actuator per PWM channel, resolved in a fixed priority order.

    Until now this was prevented by a comment in config.py and nothing else. It
    has to be code once channels are user-editable: two ESCs on one channel move
    together and neither responds to its own commands, which reads as a wiring
    fault and is not one.

    Priority is deliberate and stated in the message: the drivetrain wins,
    because a robot that can't drive can't be driven away from whatever it is
    about to hit. Then mechanisms in declaration order. A loser is reported and
    its mechanism disabled — the robot still boots and still drives.
    """
    claimed: Dict[int, str] = dict(reserved)

    for name, motor in drive.actuators.items():
        owner = claimed.get(motor.channel)
        if owner is not None:
            errors.append(
                f"channel {motor.channel} is claimed by both {owner} and drive "
                f"actuator {name!r}")
        else:
            claimed[motor.channel] = f"drive actuator {name!r}"

    for mname, mech in mechanisms.items():
        for aname, motor in mech.actuators.items():
            owner = claimed.get(motor.channel)
            if owner is not None:
                errors.append(
                    f"channel {motor.channel} is claimed by both {owner} and "
                    f"{mname}.{aname} — {mname!r} is disabled")
                mech.enabled = False
            else:
                claimed[motor.channel] = f"{mname}.{aname}"


def validate(doc: Any, reserved_channels: Optional[Dict[int, str]] = None) -> Validated:
    """Parse and check a layout document. Never raises.

    `reserved_channels` are channels already spoken for by something outside the
    layout — in practice the built-in launcher's, since it keeps its own config.
    """
    errors: List[str] = []
    warnings: List[str] = []

    if not isinstance(doc, dict):
        return Validated(DriveConfig(), {}, ["layout: expected an object"], [])

    version = doc.get("version", VERSION)
    if version != VERSION:
        return Validated(DriveConfig(), {},
                         [f"layout: unsupported version {version!r} "
                          f"(this robot speaks version {VERSION})"], [])

    encoded = len(json.dumps(doc, separators=(",", ":")).encode("utf-8"))
    if encoded > MAX_DOC_BYTES:
        return Validated(DriveConfig(), {},
                         [f"layout: {encoded} bytes, at most {MAX_DOC_BYTES} "
                          "allowed — the radio is shared with telemetry"], [])

    raw_drive = doc.get("drive")
    if not isinstance(raw_drive, dict):
        return Validated(DriveConfig(), {}, ["layout: missing a 'drive' section"], [])

    kind = str(raw_drive.get("kind", "tank")).strip()
    if kind not in DRIVE_KINDS:
        errors.append(f"drive: unknown kind {kind!r} "
                      f"(expected one of {', '.join(DRIVE_KINDS)})")
        kind = "none"

    actuators = _actuator_map(raw_drive.get("actuators", []), "drive",
                              MAX_DRIVE_ACTUATORS, errors)
    for name in actuators:
        if name in _RESERVED_DRIVE_NAMES:
            errors.append(
                f"drive actuator {name!r}: that name is reserved by the "
                "drivetrain's own settings — pick another one")

    drive = DriveConfig(kind=kind, actuators=actuators)
    drive.roles = _validate_roles(kind, raw_drive.get("roles"), actuators, errors)
    for fname, lo, hi in (("arm_seconds", 0, 10), ("slew_rate", 0, 20),
                          ("steer_gain", 0, 2), ("min_pivot_throttle", 0, 1)):
        if fname in raw_drive:
            try:
                setattr(drive, fname, _clamp_float(raw_drive[fname], lo, hi))
            except (TypeError, ValueError):
                errors.append(f"drive: {fname} must be a number")

    mechanisms: Dict[str, MechanismConfig] = {}
    raw_mechs = doc.get("mechanisms", [])
    if not isinstance(raw_mechs, list):
        errors.append("layout: 'mechanisms' must be a list")
    elif len(raw_mechs) > MAX_MECHANISMS:
        errors.append(f"layout: {len(raw_mechs)} mechanisms, at most "
                      f"{MAX_MECHANISMS} allowed")
    else:
        for raw in raw_mechs:
            mech = _validate_mechanism(raw, errors, warnings)
            if mech is None:
                continue
            if mech.name in mechanisms:
                errors.append(f"layout: duplicate mechanism name {mech.name!r}")
                continue
            mechanisms[mech.name] = mech

    _resolve_channels(drive, mechanisms, reserved_channels or {}, errors)

    return Validated(drive, mechanisms, errors, warnings)


def apply(cfg: RobotConfig, doc: Any) -> Validated:
    """Validate `doc` and, if it is clean, install it on `cfg`.

    Structural, so it REPLACES rather than patches: a layout describes the whole
    robot, and merging a partial one is how you end up with an actuator nobody
    meant to keep. Nothing is written when validation fails, so the caller's
    config is left on its previous layout.
    """
    reserved = {}
    if cfg.shooter.enabled:
        reserved[cfg.shooter.channel] = "the built-in shooter"
    result = validate(doc, reserved)
    if result.ok:
        cfg.drive = result.drive
        cfg.mechanisms = result.mechanisms
    return result


# --- persistence -------------------------------------------------------------

def load(path: Optional[str] = None) -> Optional[dict]:
    """Read the saved layout, or None if there isn't one / it's unreadable.

    Never raises: a corrupt layout must leave the robot bootable on its
    compiled-in tank defaults, not stranded at the side of a field. Same
    contract as tuning.load_overrides.
    """
    path = path or layout_path()
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return None
    except Exception as e:
        print(f"[layout] ignoring unreadable {path}: {e}")
        return None
    if not isinstance(data, dict):
        print(f"[layout] ignoring {path}: expected an object")
        return None
    return data


def save(doc: dict, path: Optional[str] = None) -> Optional[str]:
    """Persist a layout. Returns an error string on failure, never raises.

    Temp file plus rename, so a power cut mid-write can't leave a half-written
    layout the robot would refuse on next boot.
    """
    path = path or layout_path()
    tmp = f"{path}.tmp"
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=2, sort_keys=True)
            fh.write("\n")
        os.replace(tmp, path)
        return None
    except Exception as e:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        return f"{e}"
