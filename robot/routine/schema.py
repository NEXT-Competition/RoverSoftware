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

_ID_RE = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")


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
                 errors: List[str]) -> Tuple[str, str, float, float]:
    if raw is None:
        return "stop", "", 0.0, 0.0
    if not isinstance(raw, dict):
        errors.append(f"state {state_id!r}: 'drive' must be an object")
        return "stop", "", 0.0, 0.0

    mode = str(raw.get("mode", "stop")).strip()
    # A controller name is accepted directly, because that is how anyone would
    # write it: {"mode": "object_align"} rather than
    # {"source": "controller", "controller": "object_align"}.
    if mode in controllers:
        return "controller", mode, 0.0, 0.0
    if mode in ("stop", "hold"):
        return mode, "", 0.0, 0.0
    if mode == "manual":
        return "manual", "", _num(raw, "throttle", 0.0), _num(raw, "steer", 0.0)
    errors.append(
        f"state {state_id!r}: unknown drive mode {mode!r} — expected stop, "
        f"hold, manual, or one of {', '.join(controllers)}")
    return "stop", "", 0.0, 0.0


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

        source, controller, throttle, steer = _parse_drive(
            raw_state.get("drive"), sid, controllers, errors)
        state = State(
            id=sid, drive_source=source, drive_controller=controller,
            drive_throttle=throttle, drive_steer=steer,
            terminal=bool(raw_state.get("terminal", False)),
            timeout=(max(0.0, _num(raw_state, "timeout", 0.0))
                     if "timeout" in raw_state else None),
        )
        for slot, attr in (("on_enter", "on_enter"), ("on_tick", "on_tick"),
                           ("on_exit", "on_exit")):
            setattr(state, attr, _parse_actions(
                raw_state.get(slot), slot, sid, cfg.allow_arm, controller, errors,
                warnings))
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
