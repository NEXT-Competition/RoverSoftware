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


def test_auto_stop_releases_a_held_mechanism_on_its_own():
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.02))
    m.set_power(1.0, hold=True)
    time.sleep(0.03)
    m.update()
    assert m.motors["roller"].throttle == 0.0


def test_auto_stop_does_not_fire_while_the_mechanism_is_idle():
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.01))
    time.sleep(0.02)
    m.update()  # must not raise or do anything odd
    assert m.motors["roller"].throttle == 0.0


def test_a_latched_command_is_not_subject_to_the_dead_man():
    """A press-once toggle means "run until told to stop". Nothing refreshes it,
    so applying the held control's timeout to it would stop it a moment later."""
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.02))
    m.apply_preset("in")  # hold defaults to False
    time.sleep(0.03)
    m.update()
    assert m.motors["roller"].throttle == 1.0


def test_a_held_preset_stops_when_it_stops_being_refreshed():
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.02))
    m.apply_preset("out", hold=True)
    assert m.motors["roller"].throttle == -1.0
    time.sleep(0.03)
    m.update()
    assert m.motors["roller"].throttle == 0.0


def test_refreshing_a_held_preset_keeps_it_running():
    """What the gamepad's repeat does: re-announcing a held control several
    times a second is what keeps pushing the dead-man out."""
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.05))
    for _ in range(4):
        m.apply_preset("out", hold=True)
        time.sleep(0.02)
        m.update()
    assert m.motors["roller"].throttle == -1.0


def test_latching_clears_a_dead_man_armed_by_a_held_command():
    """Holding spit and then toggling the intake on must not leave the toggle
    carrying the spit's timeout — it would stop on its own a moment later."""
    m = PowerMechanism(intake_cfg(auto_stop_seconds=0.02))
    m.apply_preset("out", hold=True)
    m.apply_preset("in")
    time.sleep(0.03)
    m.update()
    assert m.motors["roller"].throttle == 1.0


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
    rover.mechanisms["intake"].apply_preset("in")
    telemetry = telemetry_with(rover, "mech")
    assert telemetry["mech"]["intake"]["values"]["roller"] == 1.0


# --- spin-up ramp ------------------------------------------------------------
#
# A flywheel commanded from neutral to full in one PWM step asks its ESC for a
# current it cannot deliver, and the ESC's own protection cuts out: the wheel
# spins up hard and then dies while the software still believes it is running.
# The drivetrain has never driven its ESCs with a step for the same reason.

def _flywheel(rate=10.0):
    """One actuator, ramping at `rate` per second. 10/s = full in 0.1 s."""
    cfg = MechanismConfig(
        name="flywheel", kind="power",
        actuators={"motor": MotorConfig(channel=2, name="motor")},
        presets={"run": {"motor": 1.0}},
        slew_rate=rate,
    )
    return PowerMechanism(cfg)


def test_a_ramped_mechanism_does_not_jump_on_the_command():
    """The whole point: the first thing the ESC sees must not be full power."""
    m = _flywheel()
    m.apply_preset("run")
    assert m.motors["motor"].throttle == 0.0


def test_a_ramp_climbs_to_the_commanded_value():
    m = _flywheel()
    m.apply_preset("run")
    end = time.monotonic() + 2.0
    while m.motors["motor"].throttle < 1.0 and time.monotonic() < end:
        m.update()
        time.sleep(0.005)
    assert m.motors["motor"].throttle == pytest.approx(1.0)


def test_a_ramp_climbs_gradually_rather_than_in_one_step():
    m = _flywheel(rate=2.0)  # full in 0.5 s
    m.apply_preset("run")
    time.sleep(0.05)
    m.update()
    partial = m.motors["motor"].throttle
    assert 0.0 < partial < 1.0, f"expected a partial value, got {partial}"


def test_stopping_is_never_ramped():
    """An e-stop that eased a flywheel to a halt would be a bug."""
    m = _flywheel(rate=0.5)  # slow enough that a ramped stop would be visible
    m.apply_preset("run")
    for _ in range(10):
        m.update()
        time.sleep(0.01)
    assert m.motors["motor"].throttle > 0.0
    m.stop()
    assert m.motors["motor"].throttle == 0.0


def test_a_ramping_mechanism_still_reports_what_it_was_told_to_do():
    """Robot._set_mechanism decides whether a bare toggle means start or stop by
    comparing status() against the preset. Reporting the part-way output would
    never match, so pressing the button mid-spin-up would restart the wheel
    instead of stopping it."""
    m = _flywheel(rate=1.0)
    m.apply_preset("run")
    m.update()
    assert m.status()["values"]["motor"] == pytest.approx(1.0)
    assert m.status()["output"]["motor"] < 1.0


def test_an_unramped_mechanism_is_unchanged():
    """slew_rate 0 is what every mechanism did before this existed, and what
    the intake, feeder and agitator still do."""
    m = PowerMechanism(intake_cfg())
    m.apply_preset("in")
    assert m.motors["roller"].throttle == pytest.approx(1.0)
    assert "output" not in m.status()


def test_a_ramp_restarts_from_where_the_output_actually_is():
    """A mechanism that sat at its target must not be handed all the elapsed
    time as one step the next time it is commanded."""
    m = _flywheel(rate=2.0)
    m.apply_preset("run")
    end = time.monotonic() + 2.0
    while m.motors["motor"].throttle < 1.0 and time.monotonic() < end:
        m.update()
        time.sleep(0.005)
    m.stop()
    time.sleep(0.3)          # idle, accumulating wall-clock
    m.apply_preset("run")
    assert m.motors["motor"].throttle == 0.0, "the idle gap became a free step"


def telemetry_with(rover, block, cmd=None):
    """A telemetry frame carrying `block`.

    The robot rotates its bulky blocks one per frame so the core stays inside a
    single XBee RF packet (see Robot._telemetry), so any ONE frame probably
    isn't the one carrying the block you asked about. This spins the rotation
    until it comes round — which is also the assertion that it comes round at
    all, rather than having been dropped from the rotation entirely.
    """
    from robot.control.commands import DriveCommand
    from robot.robot import _TELEM_BLOCKS
    cmd = cmd if cmd is not None else DriveCommand.stopped()
    for _ in range(len(_TELEM_BLOCKS) * 3):
        rover._telem_block_at = 0.0      # this frame may carry a block
        frame = rover._telemetry(cmd)
        if block in frame:
            return frame
    raise AssertionError(f"{block!r} never appeared over three rotations")
