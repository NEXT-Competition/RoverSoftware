"""A heading is only an answer while it is CURRENT.

The failure these pin down is a quiet one, and it is quiet by design. The IMU
reader deliberately survives I2C errors — it logs, backs off and retries — so a
sensor that has stopped answering looks, from outside the object, exactly like
one reporting a heading that happens not to be changing. `PoseEstimator` prefers
any non-None IMU heading to the GPS course, so without a clock on the cache a
rover whose IMU died mid-run keeps navigating on the last bearing it ever read:
a confident straight line in the wrong direction.

The GPS has had `fix_timeout` for this since it was written. This is the same
contract for the IMU, plus the two consequences that make it visible to a human:
the log line when the handover happens, and the calibration pips going away.

No hardware — the sample cache is fed directly, which is what the reader thread
does with whatever the driver hands it.
"""

import pytest

from robot.config import RobotConfig
from robot.control.commands import DriveCommand
from robot.sensors.bno085 import IMU
from robot.sensors.pose import PoseEstimator

# A quaternion whose yaw is 0, and a gyro at rest. The maths is tested in
# test_imu_start.py; here only the clock matters.
FLAT = (0.0, 0.0, 0.0, 1.0)
STILL = (0.0, 0.0, 0.0)


def reading(timeout=2.0, calib=3):
    """An IMU that has just taken one good sample."""
    imu = IMU(min_calib=1, sample_timeout=timeout)
    imu._consume(FLAT, STILL, calib)
    return imu


def age(imu, seconds):
    """Backdate the cached sample, rather than sleeping through a timeout."""
    imu._heading_at -= seconds
    imu._rate_at -= seconds


# --- the gate ---------------------------------------------------------------

def test_a_fresh_reading_answers():
    imu = reading()
    assert imu.heading() == pytest.approx(0.0)
    assert imu.yaw_rate() == pytest.approx(0.0)
    assert imu.has_heading() and imu.fresh()


def test_a_stale_reading_is_not_a_heading():
    imu = reading(timeout=2.0)
    age(imu, 2.5)
    assert imu.heading() is None
    assert imu.has_heading() is False
    assert imu.fresh() is False


def test_a_stale_yaw_rate_is_not_a_rate():
    """Sharper than the heading, if anything: a frozen rate is fed to a D term
    as though it were happening, so the loop keeps damping a rotation that
    stopped seconds ago."""
    imu = reading()
    age(imu, 3.0)
    assert imu.yaw_rate() is None


def test_the_two_stamps_are_independent():
    """The quaternion and the gyro are separate reports. A read that returns one
    but not the other must not refresh both."""
    imu = reading()
    age(imu, 3.0)
    imu._consume(FLAT, None, None)           # heading only
    assert imu.heading() is not None
    assert imu.yaw_rate() is None


def test_a_new_sample_makes_it_current_again():
    imu = reading()
    age(imu, 3.0)
    assert imu.heading() is None
    imu._consume(FLAT, STILL, 3)
    assert imu.heading() is not None


def test_zero_disables_the_check():
    """The escape hatch, for a build where flapping is worse than staleness. It
    restores exactly the old behaviour: the last reading, forever."""
    imu = reading(timeout=0.0)
    age(imu, 3600.0)
    assert imu.heading() is not None


def test_a_sensor_that_never_read_anything_answers_nothing():
    imu = IMU(min_calib=0, sample_timeout=2.0)
    assert imu.heading() is None
    assert imu.fresh() is False


def test_calibration_still_meets_min_calib_before_answering():
    """The freshness gate is on top of the old rule, not instead of it."""
    imu = IMU(min_calib=2, sample_timeout=2.0)
    imu._consume(FLAT, STILL, 1)
    assert imu.heading() is None              # current, but not yet absolute


def test_the_raw_calibration_level_is_not_gated():
    """It is the diagnostic: a caller asking what the chip last said should get
    what the chip last said. Reporting it to a HUMAN is what needs pairing with
    fresh() — see the telemetry test below."""
    imu = reading(calib=3)
    age(imu, 10.0)
    assert imu.calibration() == 3
    assert imu.fresh() is False


# --- what it costs the navigation stack --------------------------------------

def test_a_stale_imu_hands_heading_back_to_the_gps_course():
    """The whole reason the gate exists. `auto` prefers the IMU precisely
    because it is valid at a standstill — but only while it is talking."""
    class FakeGPS:
        def pose(self):
            return (37.0, -122.0, 90.0)      # course over ground, from motion

    imu = reading()
    pose = PoseEstimator(FakeGPS(), imu, "auto")
    assert pose.pose()[2] == pytest.approx(0.0)     # the IMU's absolute heading
    assert pose.heading_is_absolute() is True

    age(imu, 5.0)
    assert pose.pose()[2] == pytest.approx(90.0)    # the GPS course instead
    assert pose.heading_is_absolute() is False


def test_imu_only_reports_no_heading_rather_than_a_stale_one():
    """With heading_source='imu' there is nothing to fall back TO, and that is
    still better than a bearing nobody has confirmed for a minute."""
    class FakeGPS:
        def pose(self):
            return (37.0, -122.0, 90.0)

    imu = reading()
    pose = PoseEstimator(FakeGPS(), imu, "imu")
    age(imu, 5.0)
    assert pose.pose()[2] is None


# --- saying so ---------------------------------------------------------------

def test_repeated_read_errors_are_logged_once_per_interval(capsys):
    """A corrupted I2C stream produces the same error five times a second for as
    long as the cause lasts, and a journal full of one repeated line is a
    journal nobody reads."""
    imu = reading()
    for _ in range(20):
        imu._note_read_error(KeyError(123))
    out = capsys.readouterr().out
    assert out.count("[IMU] read error") == 1
    assert "(1 since start)" in out


def test_the_first_error_explains_what_a_bare_number_means(capsys):
    """`KeyError(123)` prints as '123' and nothing else, which is exactly the
    message that sends someone hunting for a missing sensor when the sensor is
    fine and the byte stream is not."""
    imu = reading()
    imu._note_read_error(KeyError(123))
    out = capsys.readouterr().out
    assert "SHTP report id" in out
    assert "not a missing sensor" in out


def test_the_handover_to_the_gps_is_announced_once(capsys):
    imu = reading()
    age(imu, 5.0)
    for _ in range(5):
        imu._note_read_error(OSError("bus"))
    out = capsys.readouterr().out
    assert out.count("falls back to the GPS course") == 1


def test_recovery_is_announced_too(capsys):
    """An operator told the heading has gone needs telling when it came back, or
    the next thing they do is go looking for a fault that has fixed itself."""
    imu = reading()
    age(imu, 5.0)
    imu._note_read_error(OSError("bus"))
    capsys.readouterr()
    imu._consume(FLAT, STILL, 3)
    assert "the heading is back" in capsys.readouterr().out


def test_a_corrupted_packet_costs_one_sample_not_a_backoff():
    """The observed real-world failure is a single flipped bit in an otherwise
    valid packet, so the NEXT packet is very probably fine. Backing off 200 ms
    for that turns one bad packet into eight lost samples."""
    from robot.sensors import bno085
    assert not isinstance(KeyError(123), bno085._TRANSPORT_ERRORS)
    assert isinstance(OSError("bus"), bno085._TRANSPORT_ERRORS)


def test_a_healthy_reader_says_nothing_about_staleness(capsys):
    imu = reading()
    imu._note_read_error(OSError("one glitch"))
    assert "falls back to the GPS course" not in capsys.readouterr().out


# --- through the robot -------------------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    from robot.robot import Robot
    return Robot(cfg)


def test_the_calibration_pips_go_away_with_the_heading(rover):
    """Three pips beside a bearing that stopped updating is the dashboard being
    reassuring about a sensor that is not answering — the exact failure this
    whole gate exists to catch."""
    rover.imu._consume(FLAT, STILL, 3)
    assert rover._telemetry(DriveCommand.stopped())["imu_calib"] == 3
    age(rover.imu, 10.0)
    assert rover._telemetry(DriveCommand.stopped())["imu_calib"] is None


def test_the_timeout_is_live(rover):
    rover._set_config({"config": {"imu.sample_timeout": 0.5}, "save": False})
    assert rover.imu.sample_timeout == pytest.approx(0.5)


def test_the_shipped_default_is_a_couple_of_seconds():
    """Not tighter, because heading source is not a free switch — the waypoint
    controller runs different gains on an absolute heading than on a GPS course,
    so flapping on every bus hiccup would be its own bug."""
    assert RobotConfig().imu.sample_timeout == pytest.approx(2.0)
