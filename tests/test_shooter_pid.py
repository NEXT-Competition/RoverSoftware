"""The flywheel path through Shooter: a launcher held at a speed, not pulsed.

`shooter.target_rpm` is what decides which of the two shapes a build is, so
these pin that switch as well as the loop itself — a servo launcher must be
completely unaffected by any of this.
"""

import pytest

from robot.config import ShooterConfig
from robot.drive.shooter import Shooter


@pytest.fixture(autouse=True)
def _mock_servos(monkeypatch):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")


def test_pid_control_can_be_started_and_applies_throttle():
    shooter = Shooter(ShooterConfig())
    shooter.set_target_rpm(1000.0)
    shooter.set_measured_rpm(0.0)
    shooter.update()

    assert shooter.pid_active
    assert shooter.pid_throttle > 0.0


def test_a_flywheel_parks_at_neutral_so_its_esc_arms():
    """rest_angle is servo geometry; an ESC that never sees neutral stays deaf.

    Getting this wrong is silent: the robot logs a healthy "flywheel -> N rpm"
    and the wheel never moves.
    """
    servo = Shooter(ShooterConfig(target_rpm=0.0))
    flywheel = Shooter(ShooterConfig(target_rpm=3000.0))

    assert servo.servo._last == servo.cfg.rest_angle
    assert flywheel.servo._last == Shooter._NEUTRAL_ANGLE


def test_spin_toggles_between_the_configured_speed_and_stopped():
    shooter = Shooter(ShooterConfig(target_rpm=3000.0))
    assert not shooter.spinning

    shooter.spin(True)
    assert shooter.spinning
    shooter.update()
    assert shooter.pid_throttle > 0.0

    shooter.spin(False)
    assert not shooter.spinning
    assert shooter.servo._last == Shooter._NEUTRAL_ANGLE


def test_stop_drops_the_wheel_and_the_loop_state():
    """e-stop reaches the shooter through this; it must leave nothing latched."""
    shooter = Shooter(ShooterConfig(target_rpm=3000.0))
    shooter.spin(True)
    shooter.update()

    shooter.stop()
    assert not shooter.pid_active
    assert shooter.pid_throttle == 0.0
    # And a tick afterwards must not resurrect it.
    shooter.update()
    assert not shooter.pid_active


def test_throttle_stays_under_the_ceiling_when_the_wheel_reads_slow():
    """A wheel that never reaches speed must not wind the ESC wide open.

    A ball in the wheel, or a lying encoder, holds the error large forever. The
    trim clamp plus the headroom ceiling is what keeps that from becoming full
    throttle held indefinitely.
    """
    shooter = Shooter(ShooterConfig(target_rpm=3000.0))
    shooter.spin(True)
    for tick in range(200):
        shooter.set_measured_rpm(0.0)  # never gets there
        shooter._pid_last_control = 0.0  # let every tick run the loop
        shooter.update()

    ceiling = Shooter._THROTTLE_DEADBAND + (
        3000.0 / Shooter._RPM_PER_THROTTLE
    ) * Shooter._THROTTLE_HEADROOM
    assert shooter.pid_throttle <= min(ceiling, Shooter._MAX_THROTTLE) + 1e-9


def test_a_servo_launcher_still_pulses_and_never_runs_the_loop():
    shooter = Shooter(ShooterConfig())  # target_rpm defaults to 0
    assert shooter.fire()
    assert shooter.state == "firing"
    assert not shooter.pid_active
    assert shooter.shots == 1
