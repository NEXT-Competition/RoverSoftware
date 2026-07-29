from robot.config import ShooterConfig
from robot.drive.shooter import Shooter


def test_pid_control_can_be_started_and_applies_throttle(monkeypatch):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")

    shooter = Shooter(ShooterConfig())
    shooter.set_target_rpm(1000.0)
    shooter.set_measured_rpm(0.0)
    shooter.update()

    assert shooter.pid_active
    assert shooter.pid_throttle > 0.0
