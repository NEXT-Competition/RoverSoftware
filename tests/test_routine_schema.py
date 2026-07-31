"""Validating a state machine somebody drew in a browser.

Most of this is ordinary shape-checking. Three rules are not, and they are the
reason this file is long: a routine must not be able to dead-end into a state
that doesn't exist, must not be able to run forever unattended, and must not be
able to arm the launcher anywhere the firing policy isn't being enforced.
"""

import pytest

from robot.config import RoutineConfig
from robot.routine import schema

CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint", "routine")


def parse(doc, **cfg_kw):
    return schema.parse(doc, RoutineConfig(**cfg_kw), CONTROLLERS)


def routine(states, **kw):
    body = {"id": "r1", "start": states[0]["id"], "states": states}
    body.update(kw)
    return {"version": 1, "routines": [body]}


SIMPLE = routine([
    {"id": "go", "drive": {"mode": "manual", "throttle": 0.5},
     "transitions": [{"when": "elapsed", "seconds": 1, "to": "done"}]},
    {"id": "done", "terminal": True},
])


# --- the happy path ----------------------------------------------------------

def test_a_well_formed_routine_parses():
    result = parse(SIMPLE)
    assert result.ok, result.errors
    r = result.routines["r1"]
    assert r.start == "go"
    assert set(r.states) == {"go", "done"}
    assert r.states["done"].terminal


def test_a_state_may_delegate_to_a_real_controller():
    """The central design point: a state hands driving to the controller that
    already exists, providers and all, instead of the FSM re-expressing it."""
    result = parse(routine([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "aligned", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    seek = result.routines["r1"].states["seek"]
    assert seek.drive_source == "controller"
    assert seek.drive_controller == "object_align"


def test_drive_modes_stop_hold_and_manual_are_understood():
    result = parse(routine([
        {"id": "a", "drive": {"mode": "stop"},
         "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "drive": {"mode": "hold"},
         "transitions": [{"when": "always", "to": "c"}]},
        {"id": "c", "drive": {"mode": "manual", "throttle": 0.3, "steer": -0.2},
         "terminal": True}]))
    assert result.ok, result.errors
    states = result.routines["r1"].states
    assert states["a"].drive_source == "stop"
    assert states["b"].drive_source == "hold"
    assert (states["c"].drive_throttle, states["c"].drive_steer) == (0.3, -0.2)


def test_a_transition_may_require_its_condition_to_hold():
    result = parse(routine([
        {"id": "a", "transitions": [
            {"when": "aligned", "for_seconds": 0.4, "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.routines["r1"].states["a"].transitions[0].for_seconds == 0.4


# --- structural errors -------------------------------------------------------

def test_a_transition_to_a_state_that_does_not_exist_is_rejected():
    result = parse(routine([
        {"id": "a", "transitions": [{"when": "always", "to": "nowhere"}]},
        {"id": "done", "terminal": True}]))
    assert "does not exist" in " ".join(result.errors)


def test_a_start_state_that_does_not_exist_is_rejected():
    doc = routine([{"id": "a", "terminal": True}])
    doc["routines"][0]["start"] = "b"
    assert "start state" in " ".join(parse(doc).errors)


def test_duplicate_state_ids_are_rejected():
    result = parse(routine([
        {"id": "a", "terminal": True}, {"id": "a", "terminal": True}]))
    assert "duplicate state" in " ".join(result.errors)


def test_duplicate_routine_ids_are_rejected():
    doc = routine([{"id": "a", "terminal": True}])
    doc["routines"].append(dict(doc["routines"][0]))
    assert "duplicate routine" in " ".join(parse(doc).errors)


def test_malformed_ids_are_rejected():
    for bad in ("A", "9lives", "has space", "", "x" * 40):
        assert not parse(routine([{"id": bad, "terminal": True}])).ok


def test_an_unknown_drive_mode_is_rejected():
    result = parse(routine([
        {"id": "a", "drive": {"mode": "teleport"}, "terminal": True}]))
    assert "unknown drive mode" in " ".join(result.errors)


def test_a_controller_this_build_does_not_have_is_rejected():
    """Better a message in the editor than a state that silently holds still."""
    result = schema.parse(routine([
        {"id": "a", "drive": {"mode": "object_align"}, "terminal": True}]),
        RoutineConfig(), ("teleop",))
    assert "unknown drive mode" in " ".join(result.errors)


# --- what an aligning state aims at ------------------------------------------

def test_an_aligning_state_may_name_the_object_it_aims_at():
    """The point of the field: a routine that says it lines up must be able to
    say what it lines up ON. Without it the answer is whatever the detector was
    last left filtering on, which makes the same routine behave differently
    depending on what somebody typed in Settings an hour ago."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "aligned", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert result.routines["r1"].states["aim"].drive_target == "bucket"


def test_the_shooter_aims_at_something_too():
    """Shooting IS a routine — align, arm, fire — so the aiming half of it needs
    the same field as object align."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "shooter_align", "target": "goal"},
         "on_enter": [{"do": "arm"}],
         "transitions": [{"when": "shots", "mech": "shooter", "at_least": 1, "to": "done"}]},
        {"id": "done", "terminal": True}]), allow_arm=True)
    assert result.ok, result.errors
    assert result.routines["r1"].states["aim"].drive_target == "goal"


def test_no_target_means_whatever_is_already_selected():
    """Every routine written before this field existed says nothing, and must go
    on meaning exactly what it meant."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align"},
         "transitions": [{"when": "aligned", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert result.routines["r1"].states["aim"].drive_target == ""


def test_a_target_on_a_mode_that_cannot_aim_is_rejected():
    """Storing it would put a target on the editor's screen that nothing reads —
    a routine that looks like it aims and doesn't."""
    result = parse(routine([
        {"id": "a", "drive": {"mode": "waypoint", "target": "bucket"},
         "transitions": [{"when": "route_done", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert not result.ok
    assert "target" in " ".join(result.errors)


def test_an_absurdly_long_target_is_rejected():
    """Bounded like everything else that crosses a 57600-baud radio."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align", "target": "b" * 200},
         "transitions": [{"when": "aligned", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert not result.ok


# --- how near an aligning state gets -----------------------------------------

def test_an_aligning_state_may_say_how_close_to_get():
    """The other half of "align to WHAT": how NEAR. Without it a state stops at
    whatever standoff Settings was last left on, so the same routine closes to a
    different distance between runs nobody edited."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket",
                                "stop_within_m": 1.5},
         "transitions": [{"when": "arrived", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert result.routines["r1"].states["aim"].drive_stop_within_m == 1.5


def test_no_stop_distance_means_the_controllers_own():
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align"},
         "transitions": [{"when": "arrived", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert result.routines["r1"].states["aim"].drive_stop_within_m == 0.0


def test_a_stop_distance_on_a_mode_that_cannot_approach_is_rejected():
    """Waypoint navigation drives to a coordinate, not to something it can see.
    A distance there would sit on the editor's screen unread."""
    result = parse(routine([
        {"id": "a", "drive": {"mode": "waypoint", "stop_within_m": 1.5},
         "transitions": [{"when": "route_done", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert not result.ok
    assert "stop_within_m" in " ".join(result.errors)


@pytest.mark.parametrize("metres", [0.01, 0.0, -2.0, 500.0])
def test_an_out_of_range_stop_distance_is_refused_not_clamped(metres):
    """Clamping would leave the document saying one thing and the robot doing
    another — and it is usually a slipped decimal point, which is worth being
    told about rather than half-honoured."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align", "stop_within_m": metres},
         "transitions": [{"when": "arrived", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert not result.ok


def test_a_non_numeric_stop_distance_is_refused():
    """It crosses a radio as JSON somebody's editor produced."""
    result = parse(routine([
        {"id": "aim", "drive": {"mode": "object_align", "stop_within_m": "near"},
         "transitions": [{"when": "arrived", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert not result.ok


def test_spinning_up_away_from_the_camera_is_a_warning():
    """`spin_up` works the shot out from the range to the target, so a state
    that has let go of the camera has nothing to measure — the action declines
    and the launcher never spins, which at the field reads as a shot that simply
    didn't happen. Legal (the range could come from a state entered moments ago)
    so it is a warning, but it is nearly always the mistake."""
    result = parse(routine([
        {"id": "spin", "drive": {"mode": "stop"},
         "on_enter": [{"do": "spin_up", "mech": "flywheel"}],
         "transitions": [{"when": "elapsed", "seconds": 1, "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert "spin_up" in " ".join(result.warnings)


def test_spinning_up_while_still_aiming_is_silent():
    result = parse(routine([
        {"id": "spin", "drive": {"mode": "shooter_align"},
         "on_enter": [{"do": "spin_up", "mech": "flywheel"}],
         "transitions": [{"when": "elapsed", "seconds": 1, "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert not result.warnings


def test_a_fixed_distance_needs_no_camera_and_so_warns_about_nothing():
    """The bench shape: a known distance typed in, no target in view."""
    result = parse(routine([
        {"id": "spin", "drive": {"mode": "stop"},
         "on_enter": [{"do": "spin_up", "mech": "flywheel", "distance_m": 4}],
         "transitions": [{"when": "elapsed", "seconds": 1, "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors
    assert not result.warnings


def test_spinning_up_is_not_an_arming_action():
    """A flywheel is not a trigger: it spins, and something else feeds it. So
    `spin_up` is allowed on a stock robot exactly as `mech_power` is, and does
    not need RS_ROUTINE_ALLOW_ARM the way `arm` does."""
    result = parse(routine([
        {"id": "spin", "drive": {"mode": "shooter_align"},
         "on_enter": [{"do": "spin_up", "mech": "flywheel"}],
         "transitions": [{"when": "elapsed", "seconds": 1, "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors


def test_an_unreachable_state_is_a_warning_not_an_error():
    result = parse(routine([
        {"id": "a", "terminal": True},
        {"id": "orphan", "terminal": True}]))
    assert result.ok, result.errors
    assert "never be reached" in " ".join(result.warnings)


# --- the termination rule ----------------------------------------------------

def test_a_routine_that_can_never_end_is_rejected():
    """A safety rule, not tidiness: a machine of states that all transition
    forever, with every timeout switched off, is a robot that runs until
    somebody hits the e-stop."""
    result = parse(routine([
        {"id": "a", "timeout": 0, "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "timeout": 0, "transitions": [{"when": "always", "to": "a"}]}]))
    assert "nothing can stop it" in " ".join(result.errors)


def test_a_loop_with_a_routine_timeout_is_allowed():
    result = parse(routine([
        {"id": "a", "timeout": 0, "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "timeout": 0, "transitions": [{"when": "always", "to": "a"}]}],
        timeout=30.0))
    assert result.ok, result.errors


def test_a_loop_that_inherits_the_default_state_timeout_is_allowed():
    """An inheriting state can always be left, because the default is bounded
    above zero — so leaving the timeouts alone is enough."""
    result = parse(routine([
        {"id": "a", "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "transitions": [{"when": "always", "to": "a"}]}]))
    assert result.ok, result.errors


def test_an_unspecified_timeout_is_inherited_not_baked_in():
    """None means "ask the config every tick", which is what makes
    state_timeout_default a live parameter rather than one frozen at save."""
    result = parse(routine([{"id": "a", "terminal": True}]))
    assert result.routines["r1"].states["a"].timeout is None


# --- arming ------------------------------------------------------------------

ARMING = routine([
    {"id": "shoot", "drive": {"mode": "shooter_align"},
     "on_enter": [{"do": "arm"}],
     "transitions": [{"when": "shots", "at_least": 1, "to": "done"}]},
    {"id": "done", "terminal": True}])


def test_arming_is_refused_by_default():
    """The one action a user-authored program can take that makes something
    physically launch, so it is off unless the robot was told otherwise."""
    result = parse(ARMING)
    assert "disabled on this robot" in " ".join(result.errors)


def test_arming_is_allowed_when_the_robot_permits_it():
    result = parse(ARMING, allow_arm=True)
    assert result.ok, result.errors


def test_arming_is_refused_outside_a_shooter_align_state():
    """Anywhere else there is no dwell, no cooldown and no magazine check —
    the firing policy lives in the controller, so arming must too."""
    result = parse(routine([
        {"id": "shoot", "drive": {"mode": "manual", "throttle": 0},
         "on_enter": [{"do": "arm"}]},
        {"id": "done", "terminal": True}]), allow_arm=True)
    assert "only allowed in a state that drives with shooter_align" in \
        " ".join(result.errors)


# --- conditions and actions --------------------------------------------------

def test_an_unknown_condition_is_rejected():
    result = parse(routine([
        {"id": "a", "transitions": [{"when": "vibes", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert "unknown 'when'" in " ".join(result.errors)


def test_a_condition_missing_a_required_field_is_rejected():
    result = parse(routine([
        {"id": "a", "transitions": [{"when": "elapsed", "to": "done"}]},
        {"id": "done", "terminal": True}]))
    assert "missing 'seconds'" in " ".join(result.errors)


def test_an_unknown_action_is_rejected():
    result = parse(routine([
        {"id": "a", "on_enter": [{"do": "explode"}], "terminal": True}]))
    assert "unknown 'do'" in " ".join(result.errors)


def test_an_action_missing_a_required_field_is_rejected():
    result = parse(routine([
        {"id": "a", "on_enter": [{"do": "mech_preset", "mech": "intake"}],
         "terminal": True}]))
    assert "missing 'preset'" in " ".join(result.errors)


def test_conditions_compose():
    result = parse(routine([
        {"id": "a", "transitions": [{"to": "done", "when": "all", "of": [
            {"when": "aligned"}, {"when": "elapsed", "seconds": 1}]}]},
        {"id": "done", "terminal": True}]))
    assert result.ok, result.errors


def test_a_composition_with_an_empty_list_is_rejected():
    result = parse(routine([
        {"id": "a", "transitions": [{"to": "done", "when": "any", "of": []}]},
        {"id": "done", "terminal": True}]))
    assert "non-empty list" in " ".join(result.errors)


def test_a_broken_nested_condition_is_reported():
    result = parse(routine([
        {"id": "a", "transitions": [{"to": "done", "when": "all",
                                     "of": [{"when": "nonsense"}]}]},
        {"id": "done", "terminal": True}]))
    assert "unknown 'when'" in " ".join(result.errors)


# --- caps and junk -----------------------------------------------------------

def test_too_many_states_are_refused():
    states = [{"id": f"s{i}", "terminal": True}
              for i in range(schema.MAX_STATES + 1)]
    assert "at most" in " ".join(parse(routine(states)).errors)


def test_too_many_routines_are_refused():
    doc = routine([{"id": "a", "terminal": True}])
    doc["routines"] *= (schema.MAX_ROUTINES + 1)
    assert "at most" in " ".join(parse(doc).errors)


def test_an_oversized_document_is_refused():
    doc = routine([{"id": "a", "terminal": True}])
    doc["pad"] = "x" * schema.MAX_DOC_BYTES
    assert "at most" in " ".join(parse(doc).errors)


def test_an_unsupported_version_is_refused():
    assert not parse({"version": 99, "routines": []}).ok


def test_junk_never_raises():
    for junk in (None, [], "routine", 7, {"version": 1, "routines": "x"},
                 {"version": 1, "routines": [None]},
                 {"version": 1, "routines": [{"id": "a", "states": "x"}]},
                 {"version": 1, "routines": [{"id": "a", "states": []}]}):
        result = parse(junk)
        assert not result.ok
        assert result.errors


def test_an_empty_document_is_valid_and_holds_no_routines():
    result = parse({"version": 1, "routines": []})
    assert result.ok
    assert result.routines == {}
