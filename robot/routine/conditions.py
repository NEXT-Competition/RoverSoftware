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
    # Distance -> flywheel speed (control/ballistics.py), for the states that
    # work out a shot. Optional: a build with no launcher has none, and the
    # conditions and actions that want it say so rather than failing to load.
    ballistics: Optional[Any] = None
    # Metres to whatever is straight ahead, from the ultrasonic — no idea what
    # it is looking at, but it needs no camera and no calibration. Optional: a
    # build with no ultrasonic never sets it, and the conditions that want it
    # answer False rather than failing to load. See sensors/ultrasonic.py.
    sonar: Optional[Callable[[], Optional[float]]] = None
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
    # Named counters, owned by the engine and cleared when a routine starts.
    # This is what lets a graph loop a BOUNDED number of times: without a count
    # the only way to say "do this three times" is to draw the same three states
    # three times, and a routine you have to redraw to re-tune is one nobody
    # re-tunes. Deliberately per-run, not persisted — a counter that survived a
    # restart would make the same routine behave differently on its second run.
    counters: Dict[str, int] = field(default_factory=dict)

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


def _in_range(spec) -> Predicate:
    """True when the target is close enough to actually be hit.

    Not a distance comparison — it asks the ballistics model whether a shot at
    the measured range has a solution the flywheel can reach. That is a
    different question from "nearer than 5 m", and it is the one that matters:
    too far is out of range because the wheel would have to spin faster than it
    can, and too NEAR can also be out of range, because a fixed launch angle
    cannot throw a short, high arc into a bucket rim above it.

    False whenever the answer isn't known — no launcher, no detection, no range
    calibration, no measured `max_rpm`. A state that gates its shot on this
    therefore holds fire on an unmeasured build rather than firing blind, which
    is the direction this has to fail in.
    """
    def check(ctx):
        if ctx.ballistics is None:
            return False
        align = ctx.align()
        if align is None:
            return False
        return bool(ctx.ballistics.in_range(align.distance_m()))
    return check


def _target_distance(spec) -> Predicate:
    """True while the thing in front is within (or beyond) a named distance.

    The plain distance question, in metres, which `arrived` and `in_range` are
    both NOT:

        arrived    the aligning controller has stopped at ITS OWN standoff. It
                   only means anything while that controller is driving, and the
                   distance it refers to is the one that controller was told.
        in_range   the ballistics model says a shot has a solution — which can
                   be false because the target is too NEAR as well as too far.
        this       a number you named, tested against the measured range,
                   whatever is currently driving. That is what lets a state fire
                   at 2 m on the way in, or hand over to another mode before the
                   approach ever finishes.

    Two things it can measure the distance TO, because the robot has two ways of
    knowing and they answer different questions:

        target ("source": "target", the default)
                   how far away the DETECTED OBJECT is: measured by the
                   ultrasonic when the target is centred in its beam and in
                   range, and inferred from the bounding box otherwise. Needs a
                   detection, so it is unknown when nothing is in view.
                   See control/rangefinder.py.
        ahead  ("source": "ahead")
                   how far away the nearest thing STRAIGHT AHEAD is, from the
                   ultrasonic alone. Needs no model and no calibration, and
                   knows nothing about what it is looking at — which makes it
                   the one to use for "creep forward until something is close",
                   and the wrong one for "until the bucket is close" in a room
                   with a chair in it.

    False whenever the distance is unknown — nothing detected, no calibration,
    no ultrasonic fitted, an echo that never came back. That is the same
    direction `in_range` fails in and for the same reason: a state that waits
    for a distance should wait, not proceed on a number nobody has.
    """
    if "at_most" not in spec and "at_least" not in spec:
        raise ValueError("needs 'at_most', 'at_least', or both (metres)")
    at_most = _num(spec, "at_most") if "at_most" in spec else None
    at_least = _num(spec, "at_least") if "at_least" in spec else None
    source = str(spec.get("source", "target")).strip() or "target"
    if source not in ("target", "ahead"):
        raise ValueError(f"unknown 'source' {source!r} (expected target or ahead)")

    def check(ctx):
        distance = _range_to(ctx, source)
        if distance is None:
            return False
        if at_most is not None and distance > at_most:
            return False
        if at_least is not None and distance < at_least:
            return False
        return True
    return check


def _range_to(ctx: RoutineContext, source: str) -> Optional[float]:
    """Metres to the thing this condition is asking about, or None."""
    if source == "ahead":
        return ctx.sonar() if ctx.sonar is not None else None
    align = ctx.align()
    return align.distance_m() if align is not None else None


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


def _counter(spec) -> Predicate:
    """True once a named counter has reached `at_least`.

    Paired with the `count` action, this is the bounded loop: a state does the
    work and counts, and the transition out fires on the count. An unknown
    counter reads as 0 rather than erroring, so a graph half-drawn still runs.
    """
    name = str(spec.get("name", ""))
    at_least = int(_num(spec, "at_least", 1))
    return lambda ctx: ctx.counters.get(name, 0) >= at_least


# name -> (builder, required numeric/string fields for validation)
BUILDERS: Dict[str, Tuple[Callable[[dict], Predicate], Tuple[str, ...]]] = {
    "always": (_always, ()),
    "never": (_never, ()),
    "elapsed": (_elapsed, ("seconds",)),
    "target_visible": (_target_visible, ()),
    "aligned": (_aligned, ()),
    "arrived": (_arrived, ()),
    "in_range": (_in_range, ()),
    # Neither bound is listed as required: the builder demands at least ONE of
    # them and raises a ValueError naming both, which compile_condition already
    # turns into a document error. A `required` tuple could not express "one of
    # these two", and demanding both would rule out the common case.
    "target_distance": (_target_distance, ()),
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
    "counter": (_counter, ("name", "at_least")),
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
