"""Transition conditions: the questions a state machine may ask about the robot.

Every condition is a pure read through an injected `RoutineContext`, which is
the same rule that keeps the controllers testable — nothing here opens a camera
or a GPS, so a routine unit-tests against stubs on a laptop with no hardware.

Most conditions read state the controllers already expose. `aligned` and
`arrived` are ObjectAlignController properties; `shots` comes off the shooter.
That is deliberate: an FSM that recomputed "am I lined up" would be a second,
subtly different answer to a question the alignment controller already answers,
and the two would drift.

Conditions are compiled ONCE, at load time, into closures. The 50 Hz loop
evaluates them; it does not parse them.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..control.waypoint import haversine_m


@dataclass
class RoutineContext:
    """Everything a condition is allowed to look at."""
    controllers: Dict[str, Any] = field(default_factory=dict)
    mechanisms: Dict[str, Any] = field(default_factory=dict)
    pose: Optional[Callable[[], Optional[Tuple[float, float, Optional[float]]]]] = None
    estop: Callable[[], bool] = lambda: False
    # Re-read on every arm attempt rather than captured, so turning the switch
    # off stops the routine that is already running — the only direction a
    # safety gate is allowed to be slow in is ON. See actions._arm.
    allow_arm: Callable[[], bool] = lambda: False
    now: Callable[[], float] = time.monotonic
    # Seconds the machine has been in the current state. Owned by the engine,
    # which is the only thing that knows when the state was entered.
    state_elapsed: float = 0.0
    # Events delivered since the current state was entered, cleared on entry.
    events: set = field(default_factory=set)

    def align(self):
        """Whichever alignment controller this build has, or None.

        shooter_align is an ObjectAlignController subclass, so a routine that
        asks "aligned?" while delegating to either one gets the same answer.
        """
        for name in ("shooter_align", "object_align"):
            c = self.controllers.get(name)
            if c is not None:
                return c
        return None


# A compiled condition: takes the context, answers yes or no.
Predicate = Callable[[RoutineContext], bool]


def _num(spec: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(spec.get(key, default))
    except (TypeError, ValueError):
        return default


def _elapsed(spec) -> Predicate:
    seconds = _num(spec, "seconds", 0.0)
    return lambda ctx: ctx.state_elapsed >= seconds


def _target_visible(spec) -> Predicate:
    def check(ctx):
        align = ctx.align()
        return align is not None and align.last_detection() is not None
    return check


def _aligned(spec) -> Predicate:
    def check(ctx):
        align = ctx.align()
        return align is not None and bool(align.aligned())
    return check


def _arrived(spec) -> Predicate:
    def check(ctx):
        align = ctx.align()
        return align is not None and bool(align.arrived())
    return check


def _route_done(spec) -> Predicate:
    def check(ctx):
        wp = ctx.controllers.get("waypoint")
        return wp is not None and wp.route_done()
    return check


def _shots(spec) -> Predicate:
    at_least = int(_num(spec, "at_least", 1))

    def check(ctx):
        shooter = ctx.mechanisms.get(str(spec.get("mech", "shooter")))
        if shooter is None:
            return False
        count = getattr(shooter, "shots", None)
        if count is None:
            count = getattr(shooter, "activations", 0)
        return count >= at_least
    return check


def _mech_ready(spec) -> Predicate:
    name = str(spec.get("mech", ""))

    def check(ctx):
        mech = ctx.mechanisms.get(name)
        return mech is not None and bool(mech.ready())
    return check


def _heading(spec) -> Predicate:
    of = _num(spec, "of", 0.0)
    within = max(0.1, _num(spec, "within_deg", 10.0))

    def check(ctx):
        if ctx.pose is None:
            return False
        pose = ctx.pose()
        if pose is None or pose[2] is None:
            return False
        error = (float(pose[2]) - of + 180.0) % 360.0 - 180.0
        return abs(error) <= within
    return check


def _distance_m(spec) -> Predicate:
    lat, lon = _num(spec, "lat"), _num(spec, "lon")
    at_most = _num(spec, "at_most", 2.0)

    def check(ctx):
        if ctx.pose is None:
            return False
        pose = ctx.pose()
        if pose is None:
            return False
        return haversine_m(pose[0], pose[1], lat, lon) <= at_most
    return check


def _event(spec) -> Predicate:
    name = str(spec.get("name", ""))
    return lambda ctx: name in ctx.events


def _estopped(spec) -> Predicate:
    return lambda ctx: bool(ctx.estop())


def _always(spec) -> Predicate:
    return lambda ctx: True


def _never(spec) -> Predicate:
    return lambda ctx: False


# Composition. `of` is a list of nested condition specs, compiled the same way.
def _all(spec) -> Predicate:
    inner = [compile_condition(s)[0] for s in spec.get("of", [])]
    return lambda ctx: all(p(ctx) for p in inner)


def _any(spec) -> Predicate:
    inner = [compile_condition(s)[0] for s in spec.get("of", [])]
    return lambda ctx: any(p(ctx) for p in inner)


def _not(spec) -> Predicate:
    inner = [compile_condition(s)[0] for s in spec.get("of", [])]
    return lambda ctx: not all(p(ctx) for p in inner)


# name -> (builder, required numeric/string fields for validation)
BUILDERS: Dict[str, Tuple[Callable[[dict], Predicate], Tuple[str, ...]]] = {
    "always": (_always, ()),
    "never": (_never, ()),
    "elapsed": (_elapsed, ("seconds",)),
    "target_visible": (_target_visible, ()),
    "aligned": (_aligned, ()),
    "arrived": (_arrived, ()),
    "route_done": (_route_done, ()),
    "shots": (_shots, ("at_least",)),
    "mech_ready": (_mech_ready, ("mech",)),
    "heading": (_heading, ("of", "within_deg")),
    "distance_m": (_distance_m, ("lat", "lon", "at_most")),
    "event": (_event, ("name",)),
    "estopped": (_estopped, ()),
    "all": (_all, ("of",)),
    "any": (_any, ("of",)),
    "not": (_not, ("of",)),
}

CONDITIONS = tuple(sorted(BUILDERS))


def compile_condition(spec: Any) -> Tuple[Predicate, List[str]]:
    """Turn one condition spec into a closure plus any problems with it.

    Returns a predicate even when there are errors — a never-true one — so the
    caller can collect every problem in a document rather than stopping at the
    first. A document with errors is refused before it ever runs.
    """
    errors: List[str] = []
    if not isinstance(spec, dict):
        return _never({}), ["condition: expected an object"]
    when = str(spec.get("when", "")).strip()
    entry = BUILDERS.get(when)
    if entry is None:
        return _never({}), [f"condition: unknown 'when' {when!r} "
                            f"(expected one of {', '.join(CONDITIONS)})"]
    builder, required = entry
    for name in required:
        if name not in spec:
            errors.append(f"condition {when!r}: missing {name!r}")
    if when in ("all", "any", "not"):
        of = spec.get("of")
        if not isinstance(of, list) or not of:
            errors.append(f"condition {when!r}: 'of' must be a non-empty list")
            return _never({}), errors
        for nested in of:
            errors += compile_condition(nested)[1]
    if errors:
        return _never({}), errors
    try:
        return builder(spec), []
    except Exception as e:  # a builder should never throw; belt and braces
        return _never({}), [f"condition {when!r}: {e}"]
