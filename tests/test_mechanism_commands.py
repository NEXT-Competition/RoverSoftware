"""The two gamepad-bound mechanism commands: {"type":"intake"} and
{"type":"shooter_spin"}.

Both exist because `fire` and `jog` cannot serve this case. `fire` is answered
only by ShooterAlignController, so it does nothing in teleop; `jog` is gated to
teleop with its own expiry because it is a bench tool. An operator holding a
gamepad mid-match needs neither of those shapes, so these are separate messages
with their own (much smaller) rules: the e-stop, and nothing else.
"""

import json
import os

import pytest

os.environ.setdefault("RS_MOCK_MOTORS", "1")

from robot import layout
from robot.config import RobotConfig
from robot.drive import shooter as shooter_mod
from robot.drive.shooter import Shooter
from robot.robot import Robot


INTAKE_CHANNEL = 3
# neutral 5 with min -30 / max 40 gives a symmetric throw of
# min(40 - 5, 5 - -30) = 35, so a preset of -1.0 lands exactly on -30 degrees.
INTAKE_ANGLE = -30.0


def _layout_doc():
    cfg = RobotConfig()
    doc = layout.to_doc(cfg)
    doc["mechanisms"].append({
        "name": "intake",
        "label": "Intake",
        "kind": "power",
        "enabled": True,
        "actuators": [{
            "name": "roller", "label": "Roller", "kind": "esc",
            "channel": INTAKE_CHANNEL, "inverted": False,
            "neutral_angle": 5.0, "max_angle": 40.0, "min_angle": INTAKE_ANGLE,
            "deadband": 0.03, "max_forward": 1.0, "max_reverse": 1.0,
        }],
        "presets": {"in": {"roller": -1.0}},
        "auto_stop_seconds": 0.0,
    })
    return doc


def _robot(target_rpm=0.0):
    cfg = RobotConfig()
    cfg.shooter.enabled = True
    cfg.shooter.channel = 2
    cfg.shooter.target_rpm = target_rpm
    for sub in ("gps", "imu", "vision", "camera", "fpv"):
        getattr(cfg, sub).enabled = False
    result = layout.apply(cfg, _layout_doc())
    assert result.ok, result.errors
    return Robot(cfg)


def _send(robot, msg):
    robot._inbox.put(msg)
    robot._drain_inbox()


def _roller(robot):
    return robot.mechanisms["intake"].motors["roller"].servo


# --- the layout itself -------------------------------------------------------

def test_intake_layout_validates_against_the_reserved_shooter_channel():
    result = layout.validate(_layout_doc(), {2: "the built-in shooter"})
    assert result.errors == []
    assert result.mechanisms["intake"].actuators["roller"].channel == INTAKE_CHANNEL


def test_the_preset_commands_exactly_minus_thirty_degrees():
    """The requested speed is an ANGLE, and the symmetric-throw mapping is what
    decides whether it is reachable: with the stock +-25/35 endpoints full
    reverse stops at -25, so the endpoints are widened rather than the preset
    pushed past -1.0 (which would simply clamp)."""
    robot = _robot()
    _send(robot, {"type": "intake", "on": True})
    assert _roller(robot)._last == INTAKE_ANGLE


# --- intake ------------------------------------------------------------------

def test_a_bare_message_toggles():
    robot = _robot()
    _send(robot, {"type": "intake"})
    assert _roller(robot)._last == INTAKE_ANGLE
    _send(robot, {"type": "intake"})
    assert _roller(robot)._last == 5.0  # neutral: the ESC stays armed


def test_an_explicit_state_is_idempotent():
    """A dropped frame must not invert the mechanism. `on` is absolute."""
    robot = _robot()
    for _ in range(3):
        _send(robot, {"type": "intake", "on": True})
        assert _roller(robot)._last == INTAKE_ANGLE
    for _ in range(3):
        _send(robot, {"type": "intake", "on": False})
        assert _roller(robot)._last == 5.0


def test_it_works_outside_teleop_unlike_jog():
    """The whole reason this is not `jog`: an operator runs the intake while
    the robot is doing something else."""
    robot = _robot()
    robot.manager.mode = "waypoint"
    _send(robot, {"type": "intake", "on": True})
    assert _roller(robot)._last == INTAKE_ANGLE


def test_estop_stops_it_and_then_refuses_to_start_it():
    robot = _robot()
    _send(robot, {"type": "intake", "on": True})
    robot.manager.estop = True
    robot._apply_estop()
    assert _roller(robot)._last == 5.0
    _send(robot, {"type": "intake", "on": True})
    assert _roller(robot)._last == 5.0


def test_an_unknown_mechanism_is_refused_rather_than_raising():
    robot = _robot()
    _send(robot, {"type": "intake", "mech": "nope", "on": True})
    assert _roller(robot)._last == 5.0


# --- shooter -----------------------------------------------------------------

def test_with_no_target_rpm_it_pulses_the_servo_launcher():
    robot = _robot(target_rpm=0.0)
    assert robot.shooter.shots == 0
    _send(robot, {"type": "shooter_spin"})
    assert robot.shooter.shots == 1
    assert not robot.shooter.spinning


def test_with_a_target_rpm_it_toggles_the_flywheel():
    robot = _robot(target_rpm=1500.0)
    _send(robot, {"type": "shooter_spin"})
    assert robot.shooter.spinning
    _send(robot, {"type": "shooter_spin"})
    assert not robot.shooter.spinning
    assert robot.shooter.shots == 0  # a flywheel does not use the pulse cycle


@pytest.mark.parametrize("target", [1500.0, 3350.0])
def test_the_modelled_rpm_settles_on_the_feed_forward_throttle(monkeypatch, target):
    """With no tachometer the loop is fed its own model, so the trim term must
    converge to zero and leave the pure feed-forward throttle. This test is the
    standing proof of that caveat: if it ever fails, something has started
    claiming to measure a speed it cannot see.

    Ticked at the real 50 Hz loop rate against a fake clock, so the controller
    honours its own 0.1 s interval. That detail is the test: driving the
    controller on every tick instead makes the one-tick-delayed model estimate
    ring in a limit cycle, which is a property of the harness and not of the
    robot — Robot.run() calls update() at loop rate and lets it self-pace.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(shooter_mod.time, "monotonic", lambda: clock["t"])

    robot = _robot(target_rpm=target)
    shooter = robot.shooter
    _send(robot, {"type": "shooter_spin"})
    for _ in range(150):  # 3 simulated seconds
        shooter.update()
        clock["t"] += 1.0 / 50.0

    expected = shooter._THROTTLE_DEADBAND + target / shooter._RPM_PER_THROTTLE
    assert shooter.pid_throttle == pytest.approx(expected, abs=0.05)
    assert shooter._pid_measured_rpm == pytest.approx(target, abs=25.0)
    assert shooter._pid_trim == 0.0


@pytest.mark.parametrize("target", [1500.0, 3350.0])
def test_the_open_loop_throttle_does_not_oscillate(monkeypatch, target):
    """Every control tick must produce the SAME throttle, not an average that
    happens to look right.

    This is a regression test for a real bug: an earlier _estimated_rpm()
    synthesised a speed by inverting the commanded throttle, which made the
    model track the command one tick late. The derivative term then reacted to
    the deadband-zeroed error and drove trim between 0 and its 0.8 clamp
    forever — a 5 Hz, +-272 rpm limit cycle. Sampling the throttle once at the
    end passed or failed purely on the parity of the tick count, so the bug
    survived a bench run on real hardware. Collect every tick instead.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(shooter_mod.time, "monotonic", lambda: clock["t"])

    robot = _robot(target_rpm=target)
    shooter = robot.shooter
    _send(robot, {"type": "shooter_spin"})

    samples = []
    for i in range(400):
        shooter.update()
        clock["t"] += 1.0 / 50.0
        if i % 5 == 4:  # once per 0.1 s control interval
            samples.append(round(shooter.pid_throttle, 4))

    settled = set(samples[4:])  # drop the spin-up transient
    assert len(settled) == 1, f"throttle oscillates between {sorted(settled)}"


def test_the_configured_target_is_not_silently_clamped():
    """The controller caps throttle at 1.25x the rpm that puts this wheel's rim
    at 12 m/s. A target above that ceiling would be quietly capped and the
    operator would never know the wheel was slower than they asked for."""
    ceiling = (Shooter._THROTTLE_DEADBAND
               + (Shooter._MAX_LEGAL_RPM / Shooter._RPM_PER_THROTTLE) * 1.25)
    feed_forward = (Shooter._THROTTLE_DEADBAND
                    + 3350.0 / Shooter._RPM_PER_THROTTLE)
    assert feed_forward < ceiling


def test_a_real_reading_permanently_displaces_the_model():
    robot = _robot(target_rpm=1500.0)
    shooter = robot.shooter
    _send(robot, {"type": "shooter_spin"})
    shooter.set_measured_rpm(900.0)
    shooter.update()
    assert shooter._pid_measured_rpm == 900.0  # not overwritten by the estimate


def test_a_flywheel_idles_at_neutral_so_its_esc_arms():
    """Regression: a flywheel parked at rest_angle never arms.

    Only the drivetrain is armed explicitly at start-up, so whatever the shooter
    idles at IS its arming signal. Parked at rest_angle (-30, a servo geometry
    value) the ESC on that channel never sees neutral, ignores every later
    command, and the robot logs a healthy "flywheel -> N rpm" while the wheel
    never turns — indistinguishable from a dead button.
    """
    robot = _robot(target_rpm=3350.0)
    assert robot.shooter.servo._last == Shooter._NEUTRAL_ANGLE

    _send(robot, {"type": "shooter_spin", "on": True})
    robot.shooter.update()
    _send(robot, {"type": "shooter_spin", "on": False})
    assert robot.shooter.servo._last == Shooter._NEUTRAL_ANGLE  # still armed


def test_a_servo_launcher_still_parks_at_its_rest_position():
    """The other half of the same rule: a positional launcher must NOT be moved
    to neutral, which is not a position its geometry knows about."""
    robot = _robot(target_rpm=0.0)
    assert robot.shooter.servo._last == robot.cfg.shooter.rest_angle


def test_estop_refuses_to_spin_it_up():
    robot = _robot(target_rpm=1500.0)
    robot.manager.estop = True
    robot._apply_estop()
    _send(robot, {"type": "shooter_spin"})
    assert not robot.shooter.spinning


def test_a_robot_with_no_shooter_refuses_rather_than_raising():
    cfg = RobotConfig()
    cfg.shooter.enabled = False
    for sub in ("gps", "imu", "vision", "camera", "fpv"):
        getattr(cfg, sub).enabled = False
    robot = Robot(cfg)
    _send(robot, {"type": "shooter_spin"})  # must not raise
    assert robot.shooter is None
