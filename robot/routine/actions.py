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

from typing import Any, Callable, Dict, List, Tuple

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


def _set_route(spec) -> Effect:
    raw = spec.get("waypoints", [])
    points: List[Tuple[float, float]] = []
    for pair in raw if isinstance(raw, list) else []:
        try:
            points.append((float(pair[0]), float(pair[1])))
        except (TypeError, ValueError, IndexError):
            continue

    def run(ctx):
        wp = ctx.controllers.get("waypoint")
        if wp is None:
            print("[routine] cannot set a route: this build has no waypoint mode")
            return
        wp.on_message({"type": "route", "waypoints": [list(p) for p in points]})
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
    "set_route": (_set_route, ("waypoints",)),
    "log": (_log, ("message",)),
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
