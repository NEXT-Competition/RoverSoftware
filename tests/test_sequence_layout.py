"""What the layout validator will and will not accept as a sequence.

The bar is the one the rest of layout.py sets: a mistake that would show up at
the field as odd behaviour is an ERROR here, where it is a red line in the
editor. A mistake that only means "not finished yet" is a warning, because the
editor has to be able to save a mechanism that is still being drawn.
"""

import pytest

from robot.layout import MAX_SEQUENCE_STEPS, to_doc, validate


def layout(**mech):
    """A one-mechanism document on the stock tank drive."""
    base = {
        "name": "launcher", "kind": "sequence",
        "actuators": [
            {"name": "feeder", "channel": 4, "kind": "servo",
             "min_angle": -90, "max_angle": 90},
            {"name": "flywheel", "channel": 5, "kind": "esc",
             "encoder_a": 17, "encoder_b": 27, "encoder_cpr": 20},
            {"name": "belt", "channel": 6, "kind": "esc"},
        ],
    }
    base.update(mech)
    return {"version": 1,
            "drive": {"kind": "tank",
                      "actuators": [{"name": "left", "channel": 0},
                                    {"name": "right", "channel": 1,
                                     "inverted": True}],
                      "roles": {"left": ["left"], "right": ["right"]}},
            "mechanisms": [base]}


def check(**mech):
    return validate(layout(**mech))


def problems(result):
    return " | ".join(result.errors)


THREE_STEPS = [
    {"name": "spin up", "values": {"flywheel": 1.0}, "seconds": 0.2,
     "wait_for": {"kind": "rpm", "actuator": "flywheel", "at_least": 3000}},
    {"name": "feed", "values": {"feeder": 40}, "seconds": 0.35},
    {"name": "advance", "values": {"belt": 0.8}, "seconds": 0.6},
]


# --- the happy path -----------------------------------------------------------

def test_the_motivating_shooter_validates():
    result = check(steps=THREE_STEPS)
    assert result.errors == []
    mech = result.mechanisms["launcher"]
    assert [s.name for s in mech.steps] == ["spin up", "feed", "advance"]
    assert mech.steps[0].wait_for["at_least"] == pytest.approx(3000)


def test_a_sequence_is_an_accepted_kind():
    assert "unknown kind" not in problems(check(steps=THREE_STEPS))


def test_the_steps_survive_a_round_trip_through_the_document():
    """The editor loads what it saved. A field that validates but does not
    round-trip is one that silently reverts the next time anyone opens the
    page."""
    cfg = check(steps=THREE_STEPS)
    from robot.config import RobotConfig
    robot_cfg = RobotConfig()
    robot_cfg.mechanisms = cfg.mechanisms
    doc = to_doc(robot_cfg)
    again = validate({**layout(), "mechanisms": doc["mechanisms"]})
    assert again.errors == []
    assert [s.name for s in again.mechanisms["launcher"].steps] == \
           ["spin up", "feed", "advance"]


def test_the_shared_pulse_fields_are_read():
    result = check(steps=THREE_STEPS, rest_angle=-45, cooldown=2.0,
                   max_activations=5, step_timeout=8.0, loop=True)
    mech = result.mechanisms["launcher"]
    assert result.errors == []
    assert (mech.rest_angle, mech.cooldown) == (-45.0, 2.0)
    assert (mech.max_activations, mech.step_timeout) == (5, 8.0)
    assert mech.loop is True


# --- values are checked against the actuator they name ------------------------

def test_a_step_naming_an_unknown_actuator_is_refused():
    assert "unknown actuator 'hopper'" in problems(
        check(steps=[{"values": {"hopper": 1.0}, "seconds": 0.1}]))


def test_a_servo_value_is_clamped_to_degrees_and_an_esc_to_throttle():
    """Two units in one map, chosen by what the actuator is. The clamp is what
    keeps a typo in the servo column from arriving as a throttle of 40."""
    result = check(steps=[{"values": {"feeder": 400, "flywheel": 9.0},
                           "seconds": 0.1}])
    values = result.mechanisms["launcher"].steps[0].values
    assert values["feeder"] == pytest.approx(90.0)
    assert values["flywheel"] == pytest.approx(1.0)


def test_a_value_that_is_not_a_number_is_refused():
    assert "must be a number" in problems(
        check(steps=[{"values": {"belt": "fast"}, "seconds": 0.1}]))


# --- gates --------------------------------------------------------------------

def test_an_unknown_wait_kind_is_refused():
    assert "unknown wait_for kind" in problems(
        check(steps=[{"values": {}, "wait_for": {"kind": "vibes"}}]))


def test_a_speed_gate_on_an_actuator_with_no_encoder_is_refused():
    """Refused rather than warned. The mechanism holds an unsatisfiable gate
    closed until the step times out, so a build that saved this would find out
    at the field, as a shooter that aborts every single time."""
    assert "has no encoder pins" in problems(
        check(steps=[{"values": {}, "seconds": 0.1,
                      "wait_for": {"kind": "rpm", "actuator": "belt",
                                   "at_least": 100}}]))


def test_a_speed_gate_with_no_bound_is_refused():
    assert "at_least or at_most" in problems(
        check(steps=[{"values": {}, "wait_for": {"kind": "rpm",
                                                 "actuator": "flywheel"}}]))


def test_a_speed_gate_that_nothing_can_satisfy_is_refused():
    assert "which nothing can be" in problems(
        check(steps=[{"values": {}, "wait_for": {
            "kind": "rpm", "actuator": "flywheel",
            "at_least": 4000, "at_most": 1000}}]))


def test_a_speed_gate_naming_an_unknown_actuator_is_refused():
    assert "unknown actuator" in problems(
        check(steps=[{"values": {}, "wait_for": {"kind": "rpm",
                                                 "actuator": "nope",
                                                 "at_least": 10}}]))


def test_waiting_on_a_mechanism_that_does_not_exist_is_refused():
    assert "unknown mechanism 'intake'" in problems(
        check(steps=[{"values": {}, "wait_for": {"kind": "mech_ready",
                                                 "mech": "intake"}}]))


def test_waiting_on_itself_is_refused():
    """It cannot happen: `ready()` is False for exactly as long as the sequence
    is the thing running, so the step would always time out."""
    assert "waits for itself" in problems(
        check(steps=[{"values": {}, "wait_for": {"kind": "mech_ready",
                                                 "mech": "launcher"}}]))


def test_waiting_on_the_built_in_launcher_is_allowed():
    """`shooter` is a reserved name rather than a declared mechanism, and it is
    in the routine registry — so a step may legitimately wait for it."""
    assert "unknown mechanism" not in problems(
        check(steps=[{"values": {}, "seconds": 0.1,
                      "wait_for": {"kind": "mech_ready", "mech": "shooter"}}]))


def test_a_step_may_wait_on_a_mechanism_declared_after_it():
    """Checked in a pass of its own, so the order of an array does not quietly
    carry meaning."""
    doc = layout(steps=[{"values": {}, "seconds": 0.1,
                         "wait_for": {"kind": "mech_ready", "mech": "intake"}}])
    doc["mechanisms"].append({
        "name": "intake", "kind": "power",
        "actuators": [{"name": "roller", "channel": 7}]})
    assert validate(doc).errors == []


def test_an_unknown_on_timeout_is_refused():
    assert "unknown on_timeout" in problems(
        check(steps=[{"values": {}, "seconds": 0.1, "on_timeout": "pray"}]))


def test_on_timeout_defaults_to_abort():
    result = check(steps=THREE_STEPS)
    assert result.mechanisms["launcher"].steps[0].on_timeout == "abort"


# --- bounds -------------------------------------------------------------------

def test_too_many_steps_is_refused():
    many = [{"values": {"belt": 0.1}, "seconds": 0.1}] * (MAX_SEQUENCE_STEPS + 1)
    assert f"at most {MAX_SEQUENCE_STEPS} allowed" in problems(check(steps=many))


def test_steps_must_be_a_list():
    assert "'steps' must be a list" in problems(check(steps={"one": {}}))


def test_a_step_must_be_an_object():
    assert "must be an object" in problems(check(steps=["spin up"]))


# --- what is a warning, and why -----------------------------------------------

def test_an_empty_sequence_is_a_warning_not_an_error():
    """The state a mechanism is in the moment it is added in the editor.
    Refusing it would mean the only way to create one is to type the whole
    thing correctly first time."""
    result = check(steps=[])
    assert result.errors == []
    assert any("no steps yet" in w for w in result.warnings)


def test_a_step_that_does_nothing_is_a_warning():
    result = check(steps=[{"values": {}, "seconds": 0}])
    assert result.errors == []
    assert any("does nothing" in w for w in result.warnings)


def test_a_pulse_with_several_actuators_is_pointed_at_the_sequence_kind():
    """The warning someone hits on the way to needing this feature."""
    result = validate(layout(kind="pulse"))
    assert any("'sequence' is the kind" in w for w in result.warnings)


def test_a_dwell_longer_than_its_own_timeout_is_a_warning():
    """The dwell is served before the gate is ever tested, so this leaves the
    condition one look and no time to come true — almost always a swapped pair
    of numbers."""
    result = check(steps=[{"values": {}, "seconds": 4, "timeout": 2,
                           "wait_for": {"kind": "rpm", "actuator": "flywheel",
                                        "at_least": 3000}}])
    assert result.errors == []
    assert any("no time to come true" in w for w in result.warnings)


def test_a_normal_dwell_and_timeout_pair_is_not_warned_about():
    result = check(steps=[{"values": {}, "seconds": 0.2, "timeout": 4,
                           "wait_for": {"kind": "rpm", "actuator": "flywheel",
                                        "at_least": 3000}}])
    assert not any("no time to come true" in w for w in result.warnings)


def test_looping_one_step_with_no_cooldown_is_a_warning():
    result = check(loop=True, steps=[{"values": {"belt": 1.0}, "seconds": 0}])
    assert any("every tick" in w for w in result.warnings)


# --- ramp ---------------------------------------------------------------------

def test_a_ramp_is_read_off_a_step():
    result = check(steps=[{"name": "spin up", "values": {"flywheel": 1.0},
                           "ramp": 1.2, "seconds": 0.3}])
    assert result.errors == []
    assert result.mechanisms["launcher"].steps[0].ramp == 1.2


def test_a_step_with_no_ramp_defaults_to_none():
    """Unchanged behaviour for every layout written before ramping existed."""
    result = check(steps=THREE_STEPS)
    assert [s.ramp for s in result.mechanisms["launcher"].steps] == [0.0, 0.0, 0.0]


def test_a_ramp_is_clamped_rather_than_refused():
    """Same contract as `seconds` and `timeout`: an out-of-range number is
    pulled into range, not turned into a red line over a typo."""
    result = check(steps=[{"values": {"flywheel": 1.0}, "ramp": 900.0}])
    assert result.errors == []
    assert result.mechanisms["launcher"].steps[0].ramp == 60.0


def test_a_ramp_that_is_not_a_number_is_refused():
    result = check(steps=[{"values": {"flywheel": 1.0}, "ramp": "slowly"}])
    assert "ramp must be a number" in problems(result)


def test_a_ramp_survives_a_round_trip_through_the_document():
    steps = [{"name": "spin up", "values": {"flywheel": 1.0},
              "ramp": 1.2, "seconds": 0.3}]
    cfg = check(steps=steps)
    from robot.config import RobotConfig
    robot_cfg = RobotConfig()
    robot_cfg.mechanisms = cfg.mechanisms
    doc = to_doc(robot_cfg)
    assert doc["mechanisms"][0]["steps"][0]["ramp"] == 1.2
    again = validate({**layout(), "mechanisms": doc["mechanisms"]})
    assert again.errors == []
    assert again.mechanisms["launcher"].steps[0].ramp == 1.2


def test_a_ramp_with_nothing_to_move_is_warned_about():
    result = check(steps=[{"values": {}, "ramp": 1.0, "seconds": 0.5}])
    assert result.errors == []
    assert any("ramp has nothing to travel" in w or "just a wait" in w
               for w in result.warnings)


def test_a_ramp_longer_than_its_gate_timeout_is_warned_about():
    """The ramp counts toward the dwell, so a gate behind a long ramp gets one
    look — the same trap `seconds` already warns about."""
    result = check(steps=[{"values": {"flywheel": 1.0}, "ramp": 3.0,
                           "timeout": 2.0,
                           "wait_for": {"kind": "rpm", "actuator": "flywheel",
                                        "at_least": 3000}}])
    assert any("no time to come true" in w for w in result.warnings)
