"""A sequence mechanism as the rest of the robot sees it.

The unit tests drive `SequenceMechanism` directly. These check the wiring: that
the control loop advances it, that a routine can start one and wait for it, and
above all that the paths which make a robot safe — e-stop, mode exit, shutdown —
reach it. A queue is the one mechanism that can be halfway through something
when any of those happen.
"""

import pytest

from robot.config import MotorConfig, RobotConfig, SequenceStep
from robot.control.commands import DriveCommand
from robot.layout import validate


LAYOUT = {
    "version": 1,
    "drive": {
        "kind": "tank",
        "actuators": [{"name": "left", "channel": 0},
                      {"name": "right", "channel": 1, "inverted": True}],
        "roles": {"left": ["left"], "right": ["right"]},
    },
    "mechanisms": [{
        "name": "launcher",
        "kind": "sequence",
        "rest_angle": -30,
        "step_timeout": 5,
        "actuators": [
            {"name": "feeder", "channel": 4, "kind": "servo",
             "min_angle": -90, "max_angle": 90},
            {"name": "flywheel", "channel": 5, "kind": "esc"},
            {"name": "belt", "channel": 6, "kind": "esc"},
        ],
        "steps": [
            {"name": "spin up", "values": {"flywheel": 1.0}, "seconds": 0.3},
            {"name": "feed", "values": {"feeder": 40}, "seconds": 0.3},
            {"name": "advance", "values": {"belt": 0.8}, "seconds": 0.3},
        ],
    }],
}


@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    result = validate(LAYOUT)
    assert result.errors == []
    cfg.mechanisms = result.mechanisms
    from robot.robot import Robot
    return Robot(cfg)


def launcher(rover):
    return rover.mechanisms["launcher"]


def age(mech, seconds):
    mech._step_at -= seconds


def throttle(mech, name):
    return mech.motors[name].throttle


# --- it is built and reachable ------------------------------------------------

def test_the_layout_builds_a_sequence_mechanism(rover):
    from robot.drive.mechanism import SequenceMechanism
    assert isinstance(launcher(rover), SequenceMechanism)


def test_it_is_in_the_registry_a_routine_addresses(rover):
    assert "launcher" in rover._all_mechanisms()


def test_it_is_bound_to_the_registry_so_a_gate_can_see_the_others(rover):
    """A `mech_ready` step waits on another mechanism, which means the
    mechanism needs a handle on the set — by reference, so it keeps reflecting
    the robot rather than a snapshot taken at boot."""
    assert launcher(rover)._registry is rover._registry


# --- the control loop advances it ---------------------------------------------

def test_a_tick_advances_the_queue(rover):
    m = launcher(rover)
    m.activate()
    assert m.status()["step"] == 1
    age(m, 0.4)
    for mech in rover._all_mechanisms().values():
        mech.update()
    assert m.status()["step"] == 2


def test_the_step_shows_up_in_telemetry(rover):
    m = launcher(rover)
    m.activate()
    t = rover._telemetry(DriveCommand.stopped())
    assert t["mech"]["launcher"]["step_name"] == "spin up"
    assert t["mech"]["launcher"]["state"] == "running"


def test_telemetry_says_it_is_not_ready_while_it_runs(rover):
    m = launcher(rover)
    m.activate()
    assert rover._telemetry(DriveCommand.stopped())["mech"]["launcher"]["ready"] is False


# --- every way of being interrupted ------------------------------------------

def test_the_estop_parks_a_running_sequence(rover):
    """The failure this has to cover: an e-stop mid-queue with the flywheel at
    full throttle and the feeder mid-travel."""
    m = launcher(rover)
    m.activate()
    assert throttle(m, "flywheel") == pytest.approx(1.0)
    # The latch ControlManager owns, which is what `_apply_estop` edge-detects.
    rover.manager.estop = True
    rover._apply_estop()
    assert m.state == "rest"
    assert throttle(m, "flywheel") == pytest.approx(0.0)
    assert m.motors["feeder"].servo._last == pytest.approx(-30.0)


def test_shutdown_parks_it_too(rover):
    m = launcher(rover)
    m.activate()
    for mech in rover._all_mechanisms().values():
        mech.stop()
    assert m.state == "rest"
    assert throttle(m, "flywheel") == pytest.approx(0.0)


def test_starting_the_robot_claims_nothing_when_there_are_no_encoders(rover):
    """`start()` exists for the encoders a sequence can gate on. On a build
    with none it has to be a no-op rather than an error, because every
    mechanism gets it."""
    for mech in rover.mechanisms.values():
        mech.start()
        mech.shutdown()


# --- through a routine --------------------------------------------------------

def test_a_routine_can_start_it_and_wait_for_it_to_finish(rover):
    """The shape a match routine actually uses: fire, then move on once the
    mechanism says it is done. `mech_ready` is false for exactly as long as the
    queue runs, which is what makes the wait correct without a second timer."""
    from robot.routine.actions import compile_action
    from robot.routine.conditions import compile_condition, RoutineContext

    start, errors, _ = compile_action({"do": "sequence", "mech": "launcher"})
    assert errors == []
    done, errors = compile_condition({"when": "mech_ready", "mech": "launcher"})[:2]
    assert errors == []

    ctx = RoutineContext(mechanisms=rover._all_mechanisms())
    m = launcher(rover)

    assert done(ctx) is True          # idle before it starts
    start(ctx)
    assert m.state == "running"
    assert done(ctx) is False         # and not while it runs

    for _ in range(3):
        age(m, 0.4)
        m.update()
    assert done(ctx) is True


def test_the_launcher_verbs_all_reach_it(rover):
    """`fire`, `pulse` and `sequence` are one builder: "start the cycle this
    mechanism owns" is the same instruction whatever the cycle is."""
    from robot.routine.actions import compile_action
    from robot.routine.conditions import RoutineContext

    ctx = RoutineContext(mechanisms=rover._all_mechanisms())
    for verb in ("fire", "pulse", "sequence"):
        m = launcher(rover)
        m.stop()
        run, errors, _ = compile_action({"do": verb, "mech": "launcher"})
        assert errors == []
        run(ctx)
        assert m.state == "running", verb


def test_stopping_it_from_a_routine_parks_it(rover):
    from robot.routine.actions import compile_action
    from robot.routine.conditions import RoutineContext

    m = launcher(rover)
    m.activate()
    run, errors, _ = compile_action({"do": "mech_stop", "mech": "launcher"})
    assert errors == []
    run(RoutineContext(mechanisms=rover._all_mechanisms()))
    assert m.state == "rest"


# --- the layout the robot reports back ---------------------------------------

def test_the_robot_reports_the_sequence_in_its_layout(rover):
    from robot.layout import to_doc
    doc = to_doc(rover.cfg)
    mech = next(m for m in doc["mechanisms"] if m["name"] == "launcher")
    assert mech["kind"] == "sequence"
    assert [s["name"] for s in mech["steps"]] == ["spin up", "feed", "advance"]
