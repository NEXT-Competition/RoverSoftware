"""The complete list of things anyone may say, and what each one becomes.

This module is the security boundary. A language model does not send commands to
robots — it *names one of these*, and supplies arguments. `plan()` then rebuilds
the command from scratch out of validated pieces. Nothing the model wrote is
forwarded verbatim; a hallucinated field is dropped on the floor because nothing
reads it.

Every intent lowers to action dicts that `basestation/app.py::handle_action`
already accepted before this package existed. That is deliberate: voice and MCP
get exactly the authority the dashboard has, no more, and the robot's own rules
(arm latch, firing cooldown, e-stop) remain the only place those are enforced.

--- Authority ---
`Authority.ALWAYS`  — runs even if the model is down, and is matched by keyword
                      before the model is consulted at all (fastpath.py). E-stop
                      only.
`Authority.DIRECT`  — executes on recognition. Selecting, mode changes, routes,
                      pulling up a camera: reversible, and an operator who has to
                      confirm every mode change will stop using their voice.
`Authority.CONFIRM` — returns a pending confirmation a human taps. Firing,
                      arming, jogging, raw driving, config writes. The rule is
                      "would a wrong one of these hurt somebody or waste a
                      match", not "is it a write".

Making something *safe* is never gated: disarm and stop-routine are DIRECT even
though their opposites need a confirm.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from .vocabulary import Vocabulary


class Authority(str, Enum):
    ALWAYS = "always"
    DIRECT = "direct"
    CONFIRM = "confirm"


class IntentError(ValueError):
    """A named intent could not be turned into a command.

    Carries an operator-facing sentence, not a stack trace: it is read aloud in
    the command view and returned as an MCP tool error, so it has to say what to
    do differently ("no rover called rover9 — I can see rover1, rover2").
    """


@dataclass(frozen=True)
class Intent:
    name: str
    summary: str
    """One line. Shown in the UI's "what can I say" list, used as the MCP tool
    description, and given to the local model as the menu it chooses from."""
    args: Tuple[str, ...]
    authority: Authority
    examples: Tuple[str, ...] = ()
    """Spoken forms. These do real work — they are the few-shot examples in the
    model prompt, and a 4B model's accuracy on this task is mostly a function of
    how close its input is to one of them."""


# ---------------------------------------------------------------------------
# Argument helpers. Each raises IntentError with something an operator can act
# on, because these messages are the entire error UI.
# ---------------------------------------------------------------------------

def _robot(args: Dict[str, Any], vocab: Vocabulary, *, allow_all: bool = False) -> str:
    """Which rover this order is for, falling back to the selected one.

    The fallback is what makes "align" a usable sentence. It is also why the
    confirm card and the spoken reply always name the rover back: an order that
    silently landed on whoever happened to be selected is how the wrong rover
    moves.
    """
    said = args.get("robot")
    if said is None or said == "":
        if vocab.selected:
            return vocab.selected
        raise IntentError("no rover is selected — say which one, like 'rover1'")
    rid = vocab.resolve_robot(str(said))
    if rid == "*":
        if allow_all:
            return "*"
        raise IntentError("that command has to be given to one rover at a time")
    if rid is None:
        known = ", ".join(vocab.robots) or "none"
        raise IntentError(f"no rover matching '{said}' — I can see: {known}")
    return rid


def _place(args: Dict[str, Any], vocab: Vocabulary, key: str = "place") -> Dict[str, Any]:
    said = args.get(key)
    if not said:
        raise IntentError("say where to — a saved place name")
    place = vocab.resolve_place(str(said))
    if place is None:
        known = ", ".join(p.get("name") or p["id"] for p in vocab.places) or "none saved"
        raise IntentError(f"no place matching '{said}' — saved places: {known}")
    return place


def _number(args: Dict[str, Any], key: str, lo: float, hi: float,
            default: Optional[float] = None) -> float:
    raw = args.get(key, default)
    if raw is None:
        raise IntentError(f"'{key}' is required")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise IntentError(f"'{key}' has to be a number, not {raw!r}")
    if value != value:  # NaN — json.loads produces these from `NaN` literals
        raise IntentError(f"'{key}' has to be a number")
    # Clamped rather than refused. A model that says "full speed" as 1.5 meant
    # 1.0, and the robot's own limits are the ones that matter anyway.
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Planners: (args, vocab) -> (list of action dicts, sentence describing it)
#
# The sentence is not decoration. It is what the confirm card shows, what the
# command view logs, and what an MCP client gets back — the operator's only
# check that what was heard is what will happen.
# ---------------------------------------------------------------------------

def _plan_select(args, vocab):
    rid = _robot(args, vocab)
    # Both halves, and the second one is not optional. The dashboard owns its
    # own selection locally so a tap feels instant (see net/ws.ts), and it only
    # ever adopts the server's on the very first frame. Without the `ui` step, a
    # spoken "rover three" moved the base station's selection — where the
    # gamepad's sticks go — while the screen went on highlighting rover1. An
    # operator driving a rover the screen says they are not driving is the worst
    # outcome this whole feature could produce.
    return ([{"action": "select", "robot_id": rid},
             {"ui": "select", "robot_id": rid}],
            f"select {rid}")


def _plan_set_mode(args, vocab):
    rid = _robot(args, vocab)
    said = args.get("mode")
    mode = vocab.resolve_mode(str(said) if said is not None else None)
    if mode is None:
        raise IntentError(f"'{said}' is not a mode — try: {', '.join(vocab.modes)}")

    actions: List[Dict[str, Any]] = []
    sentence = f"put {rid} in {mode.replace('_', ' ')}"

    # "align to the cone" is two commands, and the operator said one sentence.
    # Set the detector's class filter first so the mode starts already looking
    # for the right thing, rather than locking onto whatever is nearest.
    target = args.get("target")
    if target and mode in ("object_align", "shooter_align"):
        label = vocab.resolve_label(str(target))
        if label:
            actions.append({"action": "set_config", "robot_id": rid,
                            "config": {"vision.target_label": label},
                            # Not persisted: a target chosen out loud for one run
                            # should not still be filtering the detector tomorrow.
                            "save": False})
            sentence += f", tracking '{label}'"
    elif not target and mode in ("object_align", "shooter_align"):
        # No object named: clear any filter a previous order left behind, so
        # "align" means "align to anything" rather than inheriting a stale
        # target the operator has forgotten about.
        actions.append({"action": "set_config", "robot_id": rid,
                        "config": {"vision.target_label": ""}, "save": False})

    actions.append({"action": "mode", "robot_id": rid, "mode": mode})
    return actions, sentence


def _plan_route(args, vocab):
    rid = _robot(args, vocab)
    said = args.get("places") or args.get("place")
    names = [said] if isinstance(said, str) else list(said or [])
    if not names:
        raise IntentError("say where to — one or more saved place names")
    resolved = [_place({"place": n}, vocab) for n in names]
    waypoints = [[p["lat"], p["lon"]] for p in resolved]
    labels = " then ".join(p.get("name") or p["id"] for p in resolved)
    # Route then mode: a waypoint list that arrives after the mode switch leaves
    # the rover briefly in waypoint mode with nowhere to go.
    return (
        [{"action": "route", "robot_id": rid, "waypoints": waypoints},
         {"action": "mode", "robot_id": rid, "mode": "waypoint"}],
        f"send {rid} to {labels}",
    )


def _plan_estop(args, vocab):
    rid = _robot(args, vocab, allow_all=True)
    if rid == "*":
        targets = vocab.robots or ([vocab.selected] if vocab.selected else [])
        if not targets:
            raise IntentError("no rovers to stop")
        return ([{"action": "estop", "robot_id": r} for r in targets],
                f"STOP all {len(targets)} rovers")
    return [{"action": "estop", "robot_id": rid}], f"STOP {rid}"


def _plan_clear_estop(args, vocab):
    rid = _robot(args, vocab)
    return [{"action": "clear_estop", "robot_id": rid}], f"clear {rid}'s e-stop"


def _plan_show_camera(args, vocab):
    # Mostly UI: the camera is already streaming or it isn't, and this points
    # the operator's screen at it — which is why it's DIRECT. The select is
    # real, though, so the same `ui` mirror _plan_select needs applies here.
    rid = _robot(args, vocab)
    return ([{"action": "select", "robot_id": rid},
             {"ui": "select", "robot_id": rid},
             {"ui": "camera", "robot_id": rid}],
            f"show {rid}'s camera")


def _plan_show_view(args, vocab):
    said = str(args.get("view") or "").strip().lower()
    views = {"map": "ops", "ops": "ops", "operations": "ops", "main": "ops",
             "settings": "settings", "config": "settings",
             "command": "command", "voice": "command"}
    view = views.get(said)
    if view is None:
        raise IntentError(f"'{said}' is not a screen — try map, command or settings")
    return [{"ui": "view", "view": view}], f"show the {said} screen"


def _plan_arm(args, vocab):
    rid = _robot(args, vocab)
    return [{"action": "arm_shooter", "robot_id": rid}], f"ARM {rid}'s shooter"


def _plan_disarm(args, vocab):
    rid = _robot(args, vocab)
    return [{"action": "disarm_shooter", "robot_id": rid}], f"disarm {rid}'s shooter"


def _plan_fire(args, vocab):
    rid = _robot(args, vocab)
    return [{"action": "fire", "robot_id": rid}], f"FIRE {rid}"


def _plan_drive(args, vocab):
    rid = _robot(args, vocab)
    throttle = _number(args, "throttle", -1.0, 1.0, 0.0)
    steer = _number(args, "steer", -1.0, 1.0, 0.0)
    return ([{"action": "drive", "robot_id": rid,
              "throttle": throttle, "steer": steer}],
            f"drive {rid} at throttle {throttle:+.2f}, steer {steer:+.2f}")


def _plan_jog(args, vocab):
    rid = _robot(args, vocab)
    mech = args.get("mech")
    if not mech:
        raise IntentError("say which mechanism to jog")
    power = _number(args, "power", -1.0, 1.0, 0.0)
    action = {"action": "jog", "robot_id": rid, "mech": str(mech), "power": power}
    if args.get("actuator"):
        action["actuator"] = str(args["actuator"])
    return [action], f"jog {rid}'s {mech} at {power:+.2f}"


def _plan_start_routine(args, vocab):
    rid = _robot(args, vocab)
    said = args.get("routine")
    routine = vocab.resolve_routine(rid, str(said) if said else None)
    if said and routine is None:
        known = ", ".join(vocab.routines.get(rid, [])) or "none loaded"
        raise IntentError(f"no routine matching '{said}' on {rid} — loaded: {known}")
    actions: List[Dict[str, Any]] = []
    if routine:
        actions.append({"action": "select_routine", "robot_id": rid, "id": routine})
    actions.append({"action": "routine_cmd", "robot_id": rid, "cmd": "start"})
    actions.append({"action": "mode", "robot_id": rid, "mode": "routine"})
    return actions, f"start routine {routine or '(the selected one)'} on {rid}"


def _plan_stop_routine(args, vocab):
    rid = _robot(args, vocab)
    # Back to teleop, not just "stopped": a routine that ended while the robot
    # is still in routine mode leaves the sticks dead, which reads as a crash.
    return ([{"action": "routine_cmd", "robot_id": rid, "cmd": "stop"},
             {"action": "mode", "robot_id": rid, "mode": "teleop"}],
            f"stop {rid}'s routine and return to teleop")


def _plan_set_target(args, vocab):
    rid = _robot(args, vocab)
    said = args.get("target")
    if not said:
        raise IntentError("say what to track")
    label = vocab.resolve_label(str(said))
    return ([{"action": "set_config", "robot_id": rid,
              "config": {"vision.target_label": label}, "save": False}],
            f"track '{label}' on {rid}")


def _plan_status(args, vocab):
    # Answers rather than acts. Present as an intent so "what is rover2 doing"
    # is a first-class thing to say instead of falling through to "I didn't
    # understand" — the executor fills the reply in from live telemetry.
    said = args.get("robot")
    rid = vocab.resolve_robot(str(said)) if said else None
    return [{"ui": "status", "robot_id": rid}], (
        f"status of {rid}" if rid and rid != "*" else "fleet status")


# ---------------------------------------------------------------------------
# The registry. Order matters only for display.
# ---------------------------------------------------------------------------

_PLANNERS: Dict[str, Callable[[Dict[str, Any], Vocabulary], Tuple[List[Dict[str, Any]], str]]] = {}


def _register(intent: Intent, planner) -> Intent:
    _PLANNERS[intent.name] = planner
    return intent


INTENTS: Dict[str, Intent] = {i.name: i for i in [
    _register(Intent(
        "estop", "Stop a rover immediately, or all of them.",
        ("robot",), Authority.ALWAYS,
        ("stop", "stop rover2", "emergency stop", "all stop", "halt everything"),
    ), _plan_estop),

    _register(Intent(
        "select_robot", "Make a rover the one that controls and telemetry refer to.",
        ("robot",), Authority.DIRECT,
        ("rover 2", "switch to rover1", "select rover three", "talk to rover2"),
    ), _plan_select),

    _register(Intent(
        "set_mode",
        "Change what is driving a rover: teleop, object_align, shooter_align, "
        "waypoint or routine. Optionally name the object to align to.",
        ("robot", "mode", "target"), Authority.DIRECT,
        ("go to teleop", "rover2 align to the bucket", "switch to waypoint mode",
         "put rover1 in object align tracking the cone", "back to manual"),
    ), _plan_set_mode),

    _register(Intent(
        "route_to", "Send a rover to one or more saved places, in order.",
        ("robot", "places"), Authority.DIRECT,
        ("send rover2 to bucket A", "rover1 go to start then bucket B"),
    ), _plan_route),

    _register(Intent(
        "clear_estop", "Release a latched e-stop so the rover can drive again.",
        ("robot",), Authority.DIRECT,
        ("clear the e-stop", "release rover2", "reset the stop"),
    ), _plan_clear_estop),

    _register(Intent(
        "show_camera", "Bring up a rover's FPV camera feed.",
        ("robot",), Authority.DIRECT,
        ("show me the camera", "pull up rover2's camera", "fpv on rover1",
         "let me see what rover3 sees"),
    ), _plan_show_camera),

    _register(Intent(
        "show_view", "Switch the dashboard between the map, command and settings screens.",
        ("view",), Authority.DIRECT,
        ("go back to the map", "open settings", "show the command screen"),
    ), _plan_show_view),

    _register(Intent(
        "set_target", "Choose which detected object class object_align tracks.",
        ("robot", "target"), Authority.DIRECT,
        ("track the bucket", "target cones instead", "look for the red ball"),
    ), _plan_set_target),

    _register(Intent(
        "stop_routine", "Stop the running routine and return the rover to teleop.",
        ("robot",), Authority.DIRECT,
        ("stop the routine", "abort the sequence", "cancel rover2's program"),
    ), _plan_stop_routine),

    _register(Intent(
        "disarm_shooter", "Make the shooter safe.",
        ("robot",), Authority.DIRECT,
        ("disarm", "make it safe", "disarm rover2's shooter"),
    ), _plan_disarm),

    _register(Intent(
        "status", "Report what a rover, or the whole fleet, is doing.",
        ("robot",), Authority.DIRECT,
        ("what is rover2 doing", "status", "how is the fleet"),
    ), _plan_status),

    # --- everything below needs a human to tap confirm --------------------
    _register(Intent(
        "start_routine", "Run a routine (autonomous state machine) on a rover.",
        ("robot", "routine"), Authority.CONFIRM,
        ("run the collect routine on rover2", "start the auto sequence"),
    ), _plan_start_routine),

    _register(Intent(
        "arm_shooter", "Permit the shooter to fire.",
        ("robot",), Authority.CONFIRM,
        ("arm the shooter", "arm rover2"),
    ), _plan_arm),

    _register(Intent(
        "fire", "Fire the shooter once.",
        ("robot",), Authority.CONFIRM,
        ("fire", "shoot", "launch one"),
    ), _plan_fire),

    _register(Intent(
        "drive", "Command a raw throttle and steering value, -1 to 1.",
        ("robot", "throttle", "steer"), Authority.CONFIRM,
        ("creep forward", "drive rover2 forward at a quarter power"),
    ), _plan_drive),

    _register(Intent(
        "jog", "Bench-test one mechanism at a given power.",
        ("robot", "mech", "actuator", "power"), Authority.CONFIRM,
        ("jog the intake at half power", "test rover1's arm"),
    ), _plan_jog),
]}


def plan(name: str, args: Any, vocab: Vocabulary
         ) -> Tuple[Intent, List[Dict[str, Any]], str]:
    """Turn a named intent into concrete actions, or raise IntentError.

    The whitelist check is the first line for a reason: this is the function the
    language model's output reaches, and an unknown name here is the expected
    shape of a hallucination, not an exceptional case.
    """
    intent = INTENTS.get((name or "").strip())
    if intent is None:
        raise IntentError(f"'{name}' is not something I can do")
    if not isinstance(args, dict):
        raise IntentError("arguments have to be an object")
    actions, sentence = _PLANNERS[intent.name](args, vocab)
    return intent, actions, sentence


def catalogue() -> List[Dict[str, Any]]:
    """The intent list, for the model prompt, the MCP tool list and the UI."""
    return [{"name": i.name, "summary": i.summary, "args": list(i.args),
             "authority": i.authority.value, "examples": list(i.examples)}
            for i in INTENTS.values()]
