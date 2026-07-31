"""Parsing and validating a routine document.

A routine is a finite state machine an operator drew in a browser. It arrives
over a radio, so the rules are the ones the rest of this codebase already
follows: nothing raises, everything is bounded, and a document that doesn't
validate is stored but never armed — the robot keeps running the last one that
did.

The validation that matters most is not about types. It is:

  * every transition target exists, so a routine cannot dead-end into nothing;
  * every state can be left, by a transition or a timeout, so a routine cannot
    become an unattended runaway;
  * `arm` appears only where the alignment controller can enforce its firing
    policy, and only if this robot allows a routine to arm at all.

Compiling happens here too: conditions and actions become closures at load
time, so the 50 Hz loop evaluates them and never parses them.

--- keys this module ignores are still preserved ---
Validation reads the keys it knows and leaves the rest alone, and `Robot` stores
and echoes back the RAW document rather than a re-serialization. That is what
lets the dashboard keep its node positions (`x`/`y` on each state) in the
document itself, so the diagram a teammate opens is the one you drew rather than
whatever an auto-layout produces on their screen. It costs about twenty bytes a
state against a 16 KB cap. Nothing here interprets those keys, so a hand-edited
garbage value cannot reach the engine — the editor coerces defensively and falls
back to laying the graph out itself.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from ..config import RoutineConfig
from .actions import ARMING_ACTIONS, Effect, compile_action, parse_waypoints
from .conditions import Predicate, compile_condition

VERSION = 1

# Caps. The radio is the binding constraint: a routine set that can't be sent is
# worse than one that was refused with a message.
MAX_ROUTINES = 4
MAX_STATES = 32
MAX_TRANSITIONS = 8
MAX_ACTIONS = 8
MAX_DOC_BYTES = 16384

# Where a state's driving comes from. `controller` names are checked against the
# controllers this robot actually has, so a build without vision can't author a
# routine that silently does nothing.
DRIVE_SOURCES = ("stop", "hold", "manual", "controller")

# Controllers that align to something the camera sees, and so accept a `target`
# on their drive spec: "line up on the BUCKET". Without it, a routine can say it
# aligns but not what it aligns to, and the answer comes from whatever the
# detector's config happened to be left on — which is a routine that behaves
# differently depending on what somebody typed in Settings an hour ago.
TARGETING_CONTROLLERS = ("object_align", "shooter_align")

# A detector class name. Bounded like everything else that crosses the radio;
# generous next to any real label ("bucket", "traffic cone").
MAX_TARGET_LEN = 40

# How near an aligning state may be told to get, in metres. The floor is not
# timidity about small numbers — it is that a standoff shorter than the rover is
# an instruction to drive through the thing it is looking at, and the range
# estimate has no accuracy to spare down there anyway. The ceiling is past any
# distance a camera-sized target is still detectable at, so it only catches a
# slipped decimal point.
MIN_STOP_WITHIN_M = 0.1
MAX_STOP_WITHIN_M = 50.0

_ID_RE = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")


@dataclass(frozen=True)
class DriveSpec:
    """One state's `drive` block, parsed. A record rather than a tuple because
    six positional fields is where "what was the fourth one again" starts."""

    source: str = "stop"
    controller: str = ""
    throttle: float = 0.0
    steer: float = 0.0
    target: str = ""
    stop_within_m: float = 0.0


@dataclass
class Transition:
    to: str
    predicate: Predicate
    for_seconds: float = 0.0
    label: str = ""
    # Runtime: when the predicate started holding continuously. Reset by the
    # engine on state entry and whenever the predicate goes false.
    held_since: Optional[float] = None


@dataclass
class State:
    id: str
    drive_source: str = "stop"
    drive_controller: str = ""
    drive_throttle: float = 0.0
    drive_steer: float = 0.0
    # Which detector class the aligning delegate should lock onto while this
    # state is current. "" means whatever the detector is already filtering on,
    # which is also what every routine written before this field existed means.
    # Applied and then RESTORED by RoutineController — a routine borrows the
    # detector's target, it does not redefine it.
    drive_target: str = ""
    # How near the aligning delegate should get, in metres. 0 means "whatever the
    # controller's own standoff is set to", which is what every routine written
    # before this field existed means. Like the target, it is BORROWED by
    # RoutineController and handed back when the state is left.
    drive_stop_within_m: float = 0.0
    # None = inherit RoutineConfig.state_timeout_default, read by the engine on
    # every tick rather than baked in here. That is what makes the default a
    # genuinely live parameter: raising it from the dashboard applies to the
    # routine that is running, not just to the next one someone saves.
    timeout: Optional[float] = None
    terminal: bool = False
    on_enter: List[Effect] = field(default_factory=list)
    on_tick: List[Effect] = field(default_factory=list)
    on_exit: List[Effect] = field(default_factory=list)
    transitions: List[Transition] = field(default_factory=list)


@dataclass
class Routine:
    id: str
    name: str
    start: str
    states: Dict[str, State]
    on_end: str = "stop"      # stop | restart
    on_estop: str = "abort"   # abort | hold
    timeout: float = 0.0      # whole-routine watchdog; 0 = none


@dataclass
class ParseResult:
    routines: Dict[str, Routine] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _num(spec: dict, key: str, default: float) -> float:
    try:
        return float(spec.get(key, default))
    except (TypeError, ValueError):
        return default


def _check_id(value: Any, what: str, errors: List[str]) -> str:
    text = str(value or "").strip()
    if not _ID_RE.match(text):
        errors.append(f"{what} id {text!r} is invalid: lower-case letters, "
                      "digits, underscore and hyphen, starting with a letter")
        return ""
    return text


def _parse_drive(raw: Any, state_id: str, controllers: Tuple[str, ...],
                 errors: List[str]) -> DriveSpec:
    if raw is None:
        return DriveSpec()
    if not isinstance(raw, dict):
        errors.append(f"state {state_id!r}: 'drive' must be an object")
        return DriveSpec()

    mode = str(raw.get("mode", "stop")).strip()
    target = _parse_target(raw, mode, state_id, errors)
    stop_within = _parse_stop_within(raw, mode, state_id, errors)
    # A controller name is accepted directly, because that is how anyone would
    # write it: {"mode": "object_align"} rather than
    # {"source": "controller", "controller": "object_align"}.
    if mode in controllers:
        return DriveSpec(source="controller", controller=mode, target=target,
                         stop_within_m=stop_within)
    if mode in ("stop", "hold"):
        return DriveSpec(source=mode)
    if mode == "manual":
        return DriveSpec(source="manual", throttle=_num(raw, "throttle", 0.0),
                         steer=_num(raw, "steer", 0.0))
    errors.append(
        f"state {state_id!r}: unknown drive mode {mode!r} — expected stop, "
        f"hold, manual, or one of {', '.join(controllers)}")
    return DriveSpec()


def _parse_target(raw: dict, mode: str, state_id: str, errors: List[str]) -> str:
    """The detector class this state aligns to, validated but not resolved.

    Whether the label is one the model can actually report is not knowable here
    — the label set belongs to whatever model is loaded, which may not even be
    running yet. The detector already logs "nothing will ever match" for an
    unknown one (robot/sensors/detector.py), and that is the right place for it:
    refusing the document would make a routine unsaveable on a bench with no
    camera attached.

    A target on a mode that cannot align IS refused, though. Storing it would
    mean the editor shows a target that nothing reads — a routine that looks
    like it aims and doesn't.
    """
    said = raw.get("target")
    if said is None or said == "":
        return ""
    target = str(said).strip()
    if not target:
        return ""
    if mode not in TARGETING_CONTROLLERS:
        errors.append(
            f"state {state_id!r}: 'target' only means something when driving "
            f"with {' or '.join(TARGETING_CONTROLLERS)}, not {mode!r}")
        return ""
    if len(target) > MAX_TARGET_LEN:
        errors.append(f"state {state_id!r}: target {target[:20]!r}… is longer "
                      f"than {MAX_TARGET_LEN} characters")
        return ""
    return target


def _parse_stop_within(raw: dict, mode: str, state_id: str,
                       errors: List[str]) -> float:
    """How near this state drives before it counts as arrived, in metres.

    Refused on a mode that cannot approach anything, for the same reason a
    target is: a number the robot will never read is worse on screen than no
    number, because it reads as a distance that was set.

    Out-of-range is an error rather than a clamp. Clamping 0.02 up to the floor
    would silently turn "stop 2 cm away" into something else and leave the
    document saying 0.02 — and the whole point of putting this in the routine is
    that what you drew is what runs. It is also usually a typo, and a typo you
    are told about beats one that half-works.

    The estimate behind the metres is a bounding-box guess (control/rangefinder.py)
    with no calibration on a fresh build, so this validates the ASK, not the
    accuracy. Nothing here can know how wrong the conversion will be.
    """
    said = raw.get("stop_within_m")
    if said is None or said == "":
        return 0.0
    try:
        metres = float(said)
    except (TypeError, ValueError):
        errors.append(f"state {state_id!r}: stop_within_m {said!r} is not a number")
        return 0.0
    if mode not in TARGETING_CONTROLLERS:
        errors.append(
            f"state {state_id!r}: 'stop_within_m' only means something when "
            f"driving with {' or '.join(TARGETING_CONTROLLERS)}, not {mode!r}")
        return 0.0
    if not (MIN_STOP_WITHIN_M <= metres <= MAX_STOP_WITHIN_M):
        errors.append(
            f"state {state_id!r}: stop_within_m of {metres} m is outside "
            f"{MIN_STOP_WITHIN_M}–{MAX_STOP_WITHIN_M} m")
        return 0.0
    return metres


# Counting is a per-visit act, so it belongs in on_enter. In on_tick a counter
# climbs at the control-loop rate, turning "do this three times" into "do this
# for three ticks" — a bounded loop that ends 50x too early, which reads at the
# bench as the routine skipping states rather than as a slot mistake.
_ONCE_PER_VISIT_ACTIONS = frozenset({"count", "count_set"})


def _parse_actions(raw: Any, slot: str, state_id: str, allow_arm: bool,
                   delegate: str, errors: List[str],
                   warnings: Optional[List[str]] = None) -> List[Effect]:
    out: List[Effect] = []
    if raw is None:
        return out
    if not isinstance(raw, list):
        errors.append(f"state {state_id!r}: {slot} must be a list")
        return out
    if len(raw) > MAX_ACTIONS:
        errors.append(f"state {state_id!r}: {len(raw)} {slot} actions, "
                      f"at most {MAX_ACTIONS} allowed")
        return out
    for spec in raw:
        effect, problems, verb = compile_action(spec)
        errors += [f"state {state_id!r}: {p}" for p in problems]
        if verb in ARMING_ACTIONS:
            # Two gates, both at parse time so a refused routine never runs
            # rather than failing halfway through.
            if not allow_arm:
                errors.append(
                    f"state {state_id!r}: arming from a routine is disabled on "
                    "this robot (RS_ROUTINE_ALLOW_ARM)")
                continue
            if delegate != "shooter_align":
                errors.append(
                    f"state {state_id!r}: 'arm' is only allowed in a state that "
                    "drives with shooter_align, which is what enforces dwell, "
                    "cooldown and the magazine")
                continue
        if verb == "set_route" and not problems and warnings is not None:
            # Legal, and what the starter template ships, but worth saying out
            # loud: an empty route makes `route_done` true on the first tick,
            # so a state that waits for the drive to finish falls straight
            # through and the rover never moves.
            points, _ = parse_waypoints(spec.get("waypoints"))
            if not points:
                warnings.append(
                    f"state {state_id!r}: 'set_route' has no waypoints, so the "
                    "route is finished the moment it loads — pick the places "
                    "this route should visit")
        if (verb == "spin_up" and not problems and warnings is not None
                and not _num(spec, "distance_m", 0.0)
                and delegate not in TARGETING_CONTROLLERS):
            # Legal — the range could come from a state entered moments ago —
            # but almost always the mistake of spinning up in a `stop` state
            # after the camera has been let go of. There is then no detection to
            # measure, so the action declines and the launcher never spins,
            # which at the field reads as a shot that simply didn't happen.
            warnings.append(
                f"state {state_id!r}: 'spin_up' works out the shot from the "
                f"range to the target, but this state drives with "
                f"{delegate or 'no aligning controller'} — keep aiming with "
                f"{' or '.join(TARGETING_CONTROLLERS)} while it spins up, or "
                "give it a fixed distance")
        if (verb in _ONCE_PER_VISIT_ACTIONS and slot == "on_tick"
                and warnings is not None):
            warnings.append(
                f"state {state_id!r}: {verb!r} in on_tick counts once per control "
                f"tick, not once per visit — move it to on_enter unless you "
                f"really mean to count at the loop rate")
        if not problems:
            out.append(effect)
    return out


def _parse_transitions(raw: Any, state_id: str,
                       errors: List[str]) -> List[Transition]:
    out: List[Transition] = []
    if raw is None:
        return out
    if not isinstance(raw, list):
        errors.append(f"state {state_id!r}: 'transitions' must be a list")
        return out
    if len(raw) > MAX_TRANSITIONS:
        errors.append(f"state {state_id!r}: {len(raw)} transitions, "
                      f"at most {MAX_TRANSITIONS} allowed")
        return out
    for spec in raw:
        if not isinstance(spec, dict):
            errors.append(f"state {state_id!r}: a transition must be an object")
            continue
        to = str(spec.get("to", "")).strip()
        if not to:
            errors.append(f"state {state_id!r}: a transition has no 'to'")
            continue
        predicate, problems = compile_condition(spec)
        errors += [f"state {state_id!r}: {p}" for p in problems]
        if problems:
            continue
        out.append(Transition(
            to=to, predicate=predicate,
            for_seconds=max(0.0, _num(spec, "for_seconds", 0.0)),
            label=str(spec.get("when", ""))))
    return out


def _parse_routine(raw: Any, cfg: RoutineConfig, controllers: Tuple[str, ...],
                   errors: List[str], warnings: List[str]) -> Optional[Routine]:
    if not isinstance(raw, dict):
        errors.append("routines: expected an object")
        return None
    rid = _check_id(raw.get("id"), "routine", errors)
    if not rid:
        return None

    raw_states = raw.get("states")
    if not isinstance(raw_states, list) or not raw_states:
        errors.append(f"routine {rid!r}: needs a non-empty 'states' list")
        return None
    if len(raw_states) > MAX_STATES:
        errors.append(f"routine {rid!r}: {len(raw_states)} states, at most "
                      f"{MAX_STATES} allowed")
        return None

    states: Dict[str, State] = {}
    for raw_state in raw_states:
        if not isinstance(raw_state, dict):
            errors.append(f"routine {rid!r}: a state must be an object")
            continue
        sid = _check_id(raw_state.get("id"), "state", errors)
        if not sid:
            continue
        if sid in states:
            errors.append(f"routine {rid!r}: duplicate state id {sid!r}")
            continue

        drive = _parse_drive(raw_state.get("drive"), sid, controllers, errors)
        state = State(
            id=sid, drive_source=drive.source, drive_controller=drive.controller,
            drive_throttle=drive.throttle, drive_steer=drive.steer,
            drive_target=drive.target, drive_stop_within_m=drive.stop_within_m,
            terminal=bool(raw_state.get("terminal", False)),
            timeout=(max(0.0, _num(raw_state, "timeout", 0.0))
                     if "timeout" in raw_state else None),
        )
        for slot, attr in (("on_enter", "on_enter"), ("on_tick", "on_tick"),
                           ("on_exit", "on_exit")):
            setattr(state, attr, _parse_actions(
                raw_state.get(slot), slot, sid, cfg.allow_arm, drive.controller,
                errors, warnings))
        state.transitions = _parse_transitions(
            raw_state.get("transitions"), sid, errors)
        states[sid] = state

    if not states:
        return None

    start = str(raw.get("start", "")).strip()
    if not start:
        start = next(iter(states))
        warnings.append(f"routine {rid!r}: no 'start' given, using {start!r}")
    elif start not in states:
        errors.append(f"routine {rid!r}: start state {start!r} does not exist")

    for state in states.values():
        for transition in state.transitions:
            if transition.to not in states:
                errors.append(f"routine {rid!r}: state {state.id!r} transitions "
                              f"to {transition.to!r}, which does not exist")

    on_end = str(raw.get("on_end", "stop")).strip()
    if on_end not in ("stop", "restart"):
        errors.append(f"routine {rid!r}: on_end must be 'stop' or 'restart'")
        on_end = "stop"
    on_estop = str(raw.get("on_estop", "abort")).strip()
    if on_estop not in ("abort", "hold"):
        errors.append(f"routine {rid!r}: on_estop must be 'abort' or 'hold'")
        on_estop = "abort"

    routine = Routine(id=rid, name=str(raw.get("name", "") or rid), start=start,
                      states=states, on_end=on_end, on_estop=on_estop,
                      timeout=max(0.0, _num(raw, "timeout", 0.0)))

    _check_termination(routine, errors)
    _check_reachability(routine, warnings)
    return routine


def _check_termination(routine: Routine, errors: List[str]) -> None:
    """A routine must be able to end. This is a safety rule, not tidiness.

    A machine of states that all transition forever, with no terminal state and
    no whole-routine timeout, is a robot that runs until someone hits the
    e-stop. Every state already inherits a timeout, so the only way to author
    one is to set them all to zero — which is worth refusing out loud.
    """
    if routine.timeout > 0:
        return
    if any(s.terminal for s in routine.states.values()):
        return
    # `None` means the state inherits RoutineConfig.state_timeout_default, which
    # is bounded above zero — so an inheriting state can always be left.
    if any(s.timeout is None or s.timeout > 0 for s in routine.states.values()):
        return
    errors.append(
        f"routine {routine.id!r}: nothing can stop it — give it a terminal "
        "state, a routine timeout, or leave the per-state timeouts in place")


def _check_reachability(routine: Routine, warnings: List[str]) -> None:
    seen = {routine.start}
    frontier = [routine.start]
    while frontier:
        state = routine.states.get(frontier.pop())
        if state is None:
            continue
        for transition in state.transitions:
            if transition.to not in seen:
                seen.add(transition.to)
                frontier.append(transition.to)
    for sid in routine.states:
        if sid not in seen:
            warnings.append(f"routine {routine.id!r}: state {sid!r} can never "
                            "be reached")


def parse(doc: Any, cfg: Optional[RoutineConfig] = None,
          controllers: Tuple[str, ...] = ()) -> ParseResult:
    """Validate and compile a routine document. Never raises."""
    cfg = cfg or RoutineConfig()
    result = ParseResult()

    if not isinstance(doc, dict):
        result.errors.append("routines: expected an object")
        return result

    version = doc.get("version", VERSION)
    if version != VERSION:
        result.errors.append(f"routines: unsupported version {version!r} "
                             f"(this robot speaks version {VERSION})")
        return result

    encoded = len(json.dumps(doc, separators=(",", ":")).encode("utf-8"))
    if encoded > MAX_DOC_BYTES:
        result.errors.append(
            f"routines: {encoded} bytes, at most {MAX_DOC_BYTES} allowed — "
            "the radio is shared with telemetry")
        return result

    raw_routines = doc.get("routines", [])
    if not isinstance(raw_routines, list):
        result.errors.append("routines: 'routines' must be a list")
        return result
    if len(raw_routines) > MAX_ROUTINES:
        result.errors.append(f"routines: {len(raw_routines)} routines, at most "
                             f"{MAX_ROUTINES} allowed")
        return result

    for raw in raw_routines:
        routine = _parse_routine(raw, cfg, controllers,
                                 result.errors, result.warnings)
        if routine is None:
            continue
        if routine.id in result.routines:
            result.errors.append(f"routines: duplicate routine id {routine.id!r}")
            continue
        result.routines[routine.id] = routine

    return result
