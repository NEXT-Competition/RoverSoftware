"""Mechanisms: the generalized intake/launcher/arm.

Two things here are safety properties rather than features — that an activation
cycle can't be re-entered part way through, and that an e-stop actually stops a
powered intake. The second one is a failure mode mechanisms introduce: the
e-stop latch forces the DRIVETRAIN to zero, but it broadcasts to controllers,
and a mechanism is not a controller.
"""

import time

import pytest

from robot.config import MechanismConfig, MotorConfig, RobotConfig
from robot.drive.mechanism import PowerMechanism, PulseMechanism, build_mechanism
from robot.robot import Robot


@pytest.fixture(autouse=True)
def mock_motors(monkeypatch):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")


def intake_cfg(**kw):
    cfg = MechanismConfig(
        name="intake", kind="power",
        actuators={"roller": MotorConfig(channel=4, name="roller"),
                   "belt": MotorConfig(channel=5, name="belt")},
        presets={"in": {"roller": 1.0, "belt": 0.8},
                 "out": {"roller": -1.0, "belt": -0.8},
                 "roller_only": {"roller": 1.0}},
    )
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def kicker_cfg(**kw):
    cfg = MechanismConfig(
        name="kicker", kind="pulse",
        actuators={"arm": MotorConfig(channel=7, name="arm", kind="servo",
                                      min_angle=-90, max_angle=90)},
        rest_angle=-25.0, active_angle=25.0,
        active_seconds=0.02, recover_seconds=0.02,
    )
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


# --- power -------------------------------------------------------------------

def test_set_power_drives_every_actuator_by_default():
    m = PowerMechanism(intake_cfg())
    m.set_power(0.5)
    assert m.motors["roller"].throttle == 0.5
    assert m.motors["belt"].throttle == 0.5


def test_set_power_can_name_one_actuator():
    m = PowerMechanism(intake_cfg())
    m.set_power(0.5, "roller")
    assert m.motors["roller"].throttle == 0.5
    assert m.motors["belt"].throttle == 0.0


def test_a_preset_drives_the_whole_mechanism_at_once():
    m = PowerMechanism(intake_cfg())
    assert m.apply_preset("in")
    assert m.motors["roller"].throttle == 1.0
    assert m.motors["belt"].throttle == pytest.approx(0.8)


def test_a_preset_zeroes_the_actuators_it_does_not_mention():
    """A preset describes the whole mechanism's state. A belt still running
    because the PREVIOUS preset named it is a surprise near someone's hands."""
    m = PowerMechanism(intake_cfg())
    m.apply_preset("in")
    m.apply_preset("roller_only")
    assert m.motors["roller"].throttle == 1.0
    assert m.motors["belt"].throttle == 0.0


def test_an_unknown_preset_is_reported_not_applied():
    m = PowerMechanism(intake_cfg())
    assert m.apply_preset("nope") is False
    assert m.motors["roller"].throttle == 0.0


def test_an_unchanged_value_is_not_written_to_the_hardware():
    """A routine can hold an action every tick. Writing an unchanged value would
    cost one I2C transaction per actuator per tick — 300 a second on a six
    actuator rover, inside a 100 ms tick budget."""
    m = PowerMechanism(intake_cfg())
    m.set_power(0.5)
    m.motors["roller"].servo._last = None  # a write would set this again
    m.set_power(0.5)
    assert m.motors["roller"].servo._last is None


def test_a_changed_value_is_written():
    m = PowerMechanism(intake_cfg())
    m.set_power(0.5)
    m.motors["roller"].servo._last = None
    m.set_power(0.6)
    assert m.motors["roller"].servo._last is not None


def test_auto_stop_releases_the_mechanism_on_its_own():
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.02))
    m.set_power(1.0)
    time.sleep(0.03)
    m.update()
    assert m.motors["roller"].throttle == 0.0


def test_auto_stop_does_not_fire_while_the_mechanism_is_idle():
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.01))
    time.sleep(0.02)
    m.update()  # must not raise or do anything odd
    assert m.motors["roller"].throttle == 0.0


def test_status_reports_what_each_actuator_is_doing():
    m = PowerMechanism(intake_cfg())
    m.apply_preset("in")
    status = m.status()
    assert status["kind"] == "power"
    assert status["values"]["belt"] == pytest.approx(0.8)


# --- pulse -------------------------------------------------------------------

def test_activate_swings_to_the_active_angle_and_returns_immediately():
    """Non-blocking for the same reason the launcher is: a sleep would stall the
    50 Hz loop and freeze the drive outputs at whatever they last were."""
    m = PulseMechanism(kicker_cfg())
    started = time.monotonic()
    assert m.activate() is True
    assert time.monotonic() - started < 0.01
    assert m.motors["arm"].servo._last == 25.0


def test_the_cycle_advances_through_rest_active_recovering():
    m = PulseMechanism(kicker_cfg())
    m.activate()
    assert m.state == "active"
    time.sleep(0.03)
    m.update()
    assert m.state == "recovering"
    assert m.motors["arm"].servo._last == -25.0
    time.sleep(0.03)
    m.update()
    assert m.state == "rest"


def test_activating_mid_cycle_does_nothing():
    """The mechanism is the authority on its own cycle, so something asking
    every tick gets one activation per cycle rather than needing a timer."""
    m = PulseMechanism(kicker_cfg())
    assert m.activate() is True
    assert m.activate() is False
    assert m.activations == 1


def test_cooldown_holds_off_the_next_activation():
    m = PulseMechanism(kicker_cfg(cooldown=5.0))
    m.activate()
    for _ in range(10):
        time.sleep(0.005)
        m.update()
    assert m.state == "rest"
    assert m.ready() is False
    assert m.activate() is False


def test_a_magazine_limit_is_respected():
    m = PulseMechanism(kicker_cfg(max_activations=1))
    assert m.activate() is True
    time.sleep(0.05)
    m.update()
    m.update()
    assert m.activate() is False


def test_stop_parks_at_rest_from_any_point_in_the_cycle():
    """A mode switch or e-stop mid-pulse must retract, not leave the servo
    stalled against its stop with the mechanism cocked."""
    m = PulseMechanism(kicker_cfg())
    m.activate()
    m.stop()
    assert m.state == "rest"
    assert m.motors["arm"].servo._last == -25.0


def test_a_pulse_mechanism_can_drive_shooter_align():
    """ShooterLike is a structural fire()/stop(), so a user-declared launcher
    drops into the existing controller without touching it. Asserted by handing
    one over and firing it, not by isinstance — the protocol is deliberately not
    runtime-checkable, and what matters is that the controller can use it."""
    from robot.config import ShooterConfig
    from robot.control.shooter_align import ShooterAlignController

    m = PulseMechanism(kicker_cfg())
    controller = ShooterAlignController(shooter=m, config=ShooterConfig())
    controller.on_message({"type": "arm_shooter"})
    controller.on_message({"type": "fire"})
    assert m.activations == 1


def test_the_built_in_shooter_still_satisfies_the_same_interface():
    from robot.config import ShooterConfig
    from robot.drive.shooter import Shooter
    s = Shooter(ShooterConfig())
    for method in ("fire", "stop", "update", "ready", "status"):
        assert callable(getattr(s, method))
    assert s.status()["kind"] == "pulse"


# --- the factory -------------------------------------------------------------

def test_the_factory_builds_the_kind_the_layout_asked_for():
    assert isinstance(build_mechanism(intake_cfg()), PowerMechanism)
    assert isinstance(build_mechanism(kicker_cfg()), PulseMechanism)


def test_an_unknown_kind_becomes_something_that_can_be_stopped():
    assert isinstance(build_mechanism(intake_cfg(kind="warp_drive")),
                      PowerMechanism)


# --- integration with the robot ---------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.mechanisms = {"intake": intake_cfg()}
    bot = Robot(cfg)
    bot.link.send = lambda msg: None
    return bot


def test_the_robot_builds_the_mechanisms_its_layout_declares(rover):
    assert "intake" in rover.mechanisms
    assert isinstance(rover.mechanisms["intake"], PowerMechanism)


def test_an_estop_stops_a_running_mechanism(rover):
    """The failure mode mechanisms introduce. ControlManager forces the
    DRIVETRAIN to stopped(), and broadcasts on_estop() to controllers — but a
    mechanism is not a controller, so an intake at full power would keep
    spinning through the one button that exists to prevent exactly that."""
    rover.mechanisms["intake"].apply_preset("in")
    assert rover.mechanisms["intake"].motors["roller"].throttle == 1.0

    rover._inbox.put({"type": "estop"})
    rover._drain_inbox()
    rover._apply_estop()

    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_clearing_the_estop_does_not_restart_anything(rover):
    rover.mechanisms["intake"].apply_preset("in")
    rover._inbox.put({"type": "estop"})
    rover._drain_inbox()
    rover._apply_estop()
    rover._inbox.put({"type": "clear_estop"})
    rover._drain_inbox()
    rover._apply_estop()
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_the_estop_stop_is_edge_triggered_not_continuous(rover):
    """Held down rather than edge-detected, this would make bring-up impossible:
    jogging a mechanism while the robot is safely stopped is what bring-up is."""
    rover._inbox.put({"type": "estop"})
    rover._drain_inbox()
    rover._apply_estop()
    rover.mechanisms["intake"].set_power(0.3, "roller")
    rover._apply_estop()  # a later tick, still latched
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.3


def test_shutdown_parks_every_mechanism(rover):
    rover.mechanisms["intake"].apply_preset("in")
    rover.shutdown()
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_mechanism_state_reaches_telemetry(rover):
    from robot.control.commands import DriveCommand
    rover.mechanisms["intake"].apply_preset("in")
    telemetry = rover._telemetry(DriveCommand.stopped())
    assert telemetry["mech"]["intake"]["values"]["roller"] == 1.0
