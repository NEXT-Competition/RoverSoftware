"""Actions: the things a state can do to the robot when it is entered or held.

Actions never touch the drivetrain. What drives is decided by the state's
`drive` source, which either holds a fixed command or delegates to a real
controller — so there is exactly one thing commanding the motors at any moment,
and it is never a list of side effects racing a controller.

An action runs in one of three slots:

    on_enter  once, when the state becomes current
    on_tick   every control tick while the state is current
    on_exit   once, when the state is left — INCLUDING on abort, timeout,
              e-stop and mode exit, which is what makes `on_exit` the right
              place to disarm something

`on_tick` actions must be cheap and idempotent. `PowerMechanism.set_power`
elides unchanged writes precisely so that holding an intake on for a whole state
costs nothing after the first tick.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Tuple

from .conditions import RoutineContext

# A compiled action: does its thing, reports nothing. Failures are logged by the
# action itself, never raised — a routine mid-run must not take the robot down.
Effect = Callable[[RoutineContext], None]


def _mech(ctx: RoutineContext, name: str, verb: str):
    mech = ctx.mechanisms.get(name)
    if mech is None:
        # Validation refuses unknown mechanisms, so reaching here means the
        # layout changed under a stored routine. Say so once per occurrence
        # rather than silently doing nothing.
        print(f"[routine] cannot {verb}: no mechanism named {name!r}")
    return mech


def _mech_power(spec) -> Effect:
    name = str(spec.get("mech", ""))
    actuator = spec.get("actuator")
    actuator = str(actuator) if actuator else None
    try:
        power = float(spec.get("power", 0.0))
    except (TypeError, ValueError):
        power = 0.0

    def run(ctx):
        mech = _mech(ctx, name, "set power")
        if mech is not None and hasattr(mech, "set_power"):
            mech.set_power(power, actuator)
    return run


def _mech_preset(spec) -> Effect:
    name = str(spec.get("mech", ""))
    preset = str(spec.get("preset", ""))

    def run(ctx):
        mech = _mech(ctx, name, f"apply preset {preset!r}")
        if mech is not None and hasattr(mech, "apply_preset"):
            if not mech.apply_preset(preset):
                print(f"[routine] {name!r} has no preset {preset!r}")
    return run


def _mech_stop(spec) -> Effect:
    name = str(spec.get("mech", ""))

    def run(ctx):
        mech = _mech(ctx, name, "stop")
        if mech is not None:
            mech.stop()
    return run


def _pulse(spec) -> Effect:
    name = str(spec.get("mech", ""))

    def run(ctx):
        mech = _mech(ctx, name, "activate")
        if mech is None:
            return
        fire = getattr(mech, "fire", None) or getattr(mech, "activate", None)
        if fire is None:
            print(f"[routine] {name!r} is not a pulse mechanism")
            return
        fire()  # False just means "still cycling"; the mechanism owns that
    return run


def _arm(spec) -> Effect:
    """Permit the alignment controller to fire.

    Only ever reachable from a state that delegates to shooter_align — the
    schema refuses it anywhere else — and dropped again on state exit. The
    controller still enforces every other gate: dwell, cooldown, magazine, and
    the mechanism's own cycle.

    Checked AGAIN here against allow_arm, even though the schema already
    refused the action if it was off. The parse-time check is what gives the
    editor a clear error; this one is what makes turning the switch off take
    effect on the routine that is already running, which is the direction a
    safety gate has to work immediately in.
    """
    def run(ctx):
        if not ctx.allow_arm():
            print("[routine] arm refused: arming from a routine is disabled "
                  "on this robot")
            return
        controller = ctx.controllers.get("shooter_align")
        if controller is None:
            print("[routine] cannot arm: this build has no shooter_align mode")
            return
        controller.on_message({"type": "arm_shooter"})
    return run


def _disarm(spec) -> Effect:
    def run(ctx):
        controller = ctx.controllers.get("shooter_align")
        if controller is not None:
            controller.disarm()
    return run


# A route is bounded like everything else that crosses the radio. Sixty-four
# legs is a far longer run than a battery lasts; the doc size cap in schema.py
# would stop a runaway list anyway, but not with a message about the route.
MAX_WAYPOINTS = 64


def _one_waypoint(item: Any) -> Optional[Tuple[float, float]]:
    """One waypoint out of any of the shapes a document might carry it in.

    Three, because three exist in the wild: the editor writes `[lat, lon]`, a
    hand-written or hand-edited document tends to use `{"lat":…, "lon":…}`, and
    text pasted out of a spreadsheet arrives as `"lat,lon"`. Returns None if it
    isn't a waypoint at all — the caller decides what that means.
    """
    if isinstance(item, str):
        item = item.replace(";", ",").split(",")
    elif isinstance(item, dict):
        item = [item.get("lat"), item.get("lon")]
    try:
        lat, lon = float(item[0]), float(item[1])
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    # NaN fails every comparison, which is how it gets past a range check and
    # into haversine_m as a distance that is never <= arrive_radius_m — a leg
    # the rover can never finish.
    if lat != lat or lon != lon or not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        return None
    return (lat, lon)


def parse_waypoints(raw: Any) -> Tuple[List[Tuple[float, float]], str]:
    """(points, problem). A problem means the value is not a route at all.

    A BAD LEG IS AN ERROR, not something to skip past. Dropping the one
    waypoint with a typo in it leaves a route that still loads, still runs, and
    quietly drives a different shape than the one on the operator's screen —
    which is discovered by watching the rover go the wrong way. Refusing the
    document is discovered in the editor.

    An EMPTY route is not an error, because `[]` is what an honest starter
    template ships (coordinates cannot be guessed for someone else's field).
    The caller warns about it; see schema.py.
    """
    if raw is None:
        return [], ""
    if isinstance(raw, str):
        # "lat,lon" per line, which is what pasting out of a spreadsheet gives.
        raw = [line for line in raw.replace("\r", "\n").split("\n") if line.strip()]
    if not isinstance(raw, (list, tuple)):
        return [], ("'waypoints' must be a list of [lat, lon] pairs "
                    f"(got {type(raw).__name__})")
    if len(raw) > MAX_WAYPOINTS:
        return [], f"{len(raw)} waypoints, at most {MAX_WAYPOINTS} allowed"
    points: List[Tuple[float, float]] = []
    for index, item in enumerate(raw):
        point = _one_waypoint(item)
        if point is None:
            return [], (f"waypoint {index + 1} is not a [lat, lon] pair "
                        f"on the planet: {item!r}")
        points.append(point)
    return points, ""


def _set_route(spec) -> Effect:
    points, problem = parse_waypoints(spec.get("waypoints"))
    if problem:
        # compile_action turns this into a parse error, so the document is
        # refused in the editor rather than arming with a route that silently
        # holds nothing — an empty route reads as `route_done` immediately, so
        # the failure would look like the rover skipping its own drive state.
        raise ValueError(problem)

    def run(ctx):
        wp = ctx.controllers.get("waypoint")
        if wp is None:
            print("[routine] cannot set a route: this build has no waypoint mode")
            return
        wp.on_message({"type": "route", "waypoints": [list(p) for p in points]})
    return run


def _num(spec: dict, key: str, default: float) -> float:
    """A number out of a spec, or the default. Never raises: a routine mid-run
    must not take the robot down over a typo in one field."""
    try:
        return float(spec.get(key, default))
    except (TypeError, ValueError):
        return default


def _spin_up(spec) -> Effect:
    """Work out the shot from how far away the target is, and spin the flywheel.

    This is the one action that COMPUTES something rather than relaying a number
    the operator typed. The alternative — a `mech_power` with a hand-found value
    per distance — is a routine that is correct at exactly one range and quietly
    wrong at every other, which on a field where the bucket moves is most of them.

    The distance comes from the aligning controller's own range estimate, so a
    state that spins up must be a state that is looking at the target (the schema
    warns when it isn't). `distance_m` overrides it with a fixed number, which is
    what you want on a bench with no camera pointed at anything.

    NOTHING SPINS WHEN THE ANSWER ISN'T KNOWN. No launcher config, no range, or a
    shot the wheel cannot reach all leave the mechanism exactly as it was and say
    why. A flywheel that spun at some fallback power would throw a ball a
    distance nobody chose, and "it fired but missed" is a much harder failure to
    read at the field than "it never fired".
    """
    name = str(spec.get("mech", ""))
    actuator = spec.get("actuator")
    actuator = str(actuator) if actuator else None
    # 0 (the default) means "measure it"; an explicit distance is the override.
    fixed = _num(spec, "distance_m", 0.0)

    def run(ctx: RoutineContext) -> None:
        mech = _mech(ctx, name, "spin up")
        if mech is None or not hasattr(mech, "set_power"):
            if mech is not None:
                print(f"[routine] cannot spin up {name!r}: not a power mechanism")
            return
        if ctx.ballistics is None:
            print("[routine] cannot work out a shot: this build has no "
                  "ballistics config")
            return

        distance = fixed
        if distance <= 0.0:
            align = ctx.align()
            distance = (align.distance_m() or 0.0) if align is not None else 0.0
        if distance <= 0.0:
            print("[routine] cannot work out a shot: no range to the target "
                  "(is the rangefinder calibrated, and is the target in view?)")
            return

        shot = ctx.ballistics.shot_for(distance)
        if shot is None:
            print(f"[routine] no shot at {distance:.2f} m: out of the "
                  "launcher's range at this angle")
            return
        rpm, power = shot
        # Always logged, and with all three numbers: this line is how anyone
        # tunes `transfer` — it is the only place the range the robot BELIEVED
        # sits next to the speed it chose and the throttle it sent.
        print(f"[routine] shot at {distance:.2f} m: {rpm:.0f} rpm "
              f"-> {power:.2f} throttle on {name!r}")
        mech.set_power(power, actuator)
    return run


def _int(spec: dict, key: str, default: int) -> int:
    """A whole number out of a spec, or the default. Never raises: a routine
    mid-run must not take the robot down over a typo in one field."""
    try:
        return int(float(spec.get(key, default)))
    except (TypeError, ValueError):
        return default


def _count(spec) -> Effect:
    """Add to a named counter — the other half of the bounded loop.

    Belongs in `on_enter`: a counter incremented `on_tick` climbs at the control
    rate and turns "do this three times" into "do this for three ticks", which
    is the mistake this action exists to make hard to write by accident. The
    schema warns when it lands in the tick slot.
    """
    name = str(spec.get("name", ""))
    by = _int(spec, "by", 1)

    def run(ctx: RoutineContext) -> None:
        ctx.counters[name] = ctx.counters.get(name, 0) + by
    return run


def _count_set(spec) -> Effect:
    """Set a counter outright. `to: 0` is how a loop is re-armed."""
    name = str(spec.get("name", ""))
    to = _int(spec, "to", 0)

    def run(ctx: RoutineContext) -> None:
        ctx.counters[name] = to
    return run


def _log(spec) -> Effect:
    message = str(spec.get("message", ""))
    return lambda ctx: print(f"[routine] {message}")


# name -> (builder, required fields)
BUILDERS: Dict[str, Tuple[Callable[[dict], Effect], Tuple[str, ...]]] = {
    "mech_power": (_mech_power, ("mech", "power")),
    "mech_preset": (_mech_preset, ("mech", "preset")),
    "mech_stop": (_mech_stop, ("mech",)),
    "pulse": (_pulse, ("mech",)),
    "fire": (_pulse, ("mech",)),  # the launcher's word for the same thing
    "arm": (_arm, ()),
    "disarm": (_disarm, ()),
    # No required field: a route with no waypoints yet is the state a freshly
    # dropped node is in, and it is reported as a WARNING (schema.py) rather
    # than an error so the editor can show it inline instead of refusing to
    # save a graph that is still being drawn.
    "set_route": (_set_route, ()),
    # `distance_m` is deliberately NOT required: measuring the range is the
    # normal case and typing one is the bench override, so requiring it would
    # make the common shape the one that needs an extra field.
    "spin_up": (_spin_up, ("mech",)),
    "log": (_log, ("message",)),
    "count": (_count, ("name",)),
    "count_set": (_count_set, ("name", "to")),
}

ACTIONS = tuple(sorted(BUILDERS))

# Actions that make something able to launch. The schema gates these on
# RoutineConfig.allow_arm and on the state's drive delegate.
ARMING_ACTIONS = frozenset({"arm"})


def compile_action(spec: Any) -> Tuple[Effect, List[str], str]:
    """Turn one action spec into a closure, its problems, and its verb."""
    if not isinstance(spec, dict):
        return (lambda ctx: None), ["action: expected an object"], ""
    do = str(spec.get("do", "")).strip()
    entry = BUILDERS.get(do)
    if entry is None:
        return ((lambda ctx: None),
                [f"action: unknown 'do' {do!r} "
                 f"(expected one of {', '.join(ACTIONS)})"], do)
    builder, required = entry
    errors = [f"action {do!r}: missing {name!r}"
              for name in required if name not in spec]
    if errors:
        return (lambda ctx: None), errors, do
    try:
        return builder(spec), [], do
    except Exception as e:
        return (lambda ctx: None), [f"action {do!r}: {e}"], do
