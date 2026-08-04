"""The BNO085 read as UART-RVC frames: what it gives, and what it gives up.

Two halves. The first is the frame itself, which is the reason to be on this
transport at all — every frame carries a checksum, so corruption is DETECTED and
dropped rather than becoming a plausible-looking heading. That is exactly what
SHTP over I2C could not do: a single flipped bit there arrived as an
unrecognised report id (see tests/test_imu_packet_audit.py) or, worse, as a
heading nobody could tell was wrong.

The second half is the price, and these tests exist so it stays visible rather
than becoming folklore: no calibration level, so `min_calib` cannot be enforced;
no gyro, so the yaw rate is differentiated rather than measured; and no channel
back to the chip, so calibration cannot be saved from here.

No hardware — a fake serial port serves canned frames.
"""

import time

import pytest

from robot.config import IMUConfig, RobotConfig
from robot.sensors import bno085_rvc as rvc_mod
from robot.sensors.bno085_rvc import (FRAME_LEN, HEADER, RVCIMU, parse_frame,
                                      wrap_delta)
from robot.sensors.imu import build_imu
from robot.sensors.bno085 import IMU


def frame(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, sequence=1, accel=(0, 0, 0),
          checksum=None):
    """A well-formed 19-byte RVC frame, or a corrupted one if you pass one."""
    body = bytes([sequence])
    for value in (yaw_deg, pitch_deg, roll_deg):
        body += int(round(value * 100)).to_bytes(2, "little", signed=True)
    for value in accel:
        body += int(value).to_bytes(2, "little", signed=True)
    body += bytes([0x00, 0x00, 0x00])          # MI, MR, reserved
    check = (sum(body) & 0xFF) if checksum is None else checksum
    return HEADER + body + bytes([check])


class FakeSerial:
    """A serial port that hands back a scripted byte stream, then blocks."""

    def __init__(self, stream=b""):
        self.stream = bytearray(stream)
        self.closed = False

    def read(self, count):
        taken, self.stream = self.stream[:count], self.stream[count:]
        return bytes(taken)

    def close(self):
        self.closed = True


# --- the frame ---------------------------------------------------------------

def test_a_good_frame_decodes():
    decoded = parse_frame(frame(yaw_deg=123.45, pitch_deg=-1.5, sequence=7))
    assert decoded is not None
    assert decoded.sequence == 7
    assert decoded.yaw == pytest.approx(123.45)
    assert decoded.pitch == pytest.approx(-1.5)


def test_a_frame_is_nineteen_bytes():
    assert len(frame()) == FRAME_LEN


def test_a_bad_checksum_is_refused():
    """The entire reason to be on this transport. On I2C the same corruption
    arrived as an unrecognised report id — or as a heading nobody could tell was
    wrong."""
    assert parse_frame(frame(checksum=0x00)) is None


def test_a_single_flipped_bit_is_caught():
    """The exact failure that started all this, on the other wire."""
    good = bytearray(frame(yaw_deg=90.0))
    good[3] ^= 0x80
    assert parse_frame(bytes(good)) is None


def test_a_frame_without_the_header_is_refused():
    assert parse_frame(b"\x00\x00" + frame()[2:]) is None


def test_a_short_frame_is_refused():
    assert parse_frame(frame()[:-1]) is None


def test_negative_angles_survive_the_round_trip():
    assert parse_frame(frame(yaw_deg=-179.99)).yaw == pytest.approx(-179.99)


def test_crossing_north_is_a_small_step_not_a_huge_one():
    """A 359 -> 1 move is +2 degrees. Getting it wrong spikes the derived yaw
    rate to thousands of deg/s once per revolution, and the heading PID's D term
    answers a spike with a lurch."""
    assert wrap_delta(1.0 - 359.0) == pytest.approx(2.0)
    assert wrap_delta(359.0 - 1.0) == pytest.approx(-2.0)


# --- reading the stream ------------------------------------------------------

def reader(stream=b"", **kw):
    imu = RVCIMU(**kw)
    imu._serial = FakeSerial(stream)
    imu._running = True
    return imu


def test_frames_become_a_compass_heading():
    """RVC yaw is counter-clockwise-positive; the project convention is a
    compass heading. Same negation the quaternion path applies."""
    imu = reader(frame(yaw_deg=90.0))
    imu._consume(parse_frame(frame(yaw_deg=90.0)))
    assert imu.heading() == pytest.approx(270.0)


def test_the_offset_and_inversion_apply_as_they_do_on_i2c():
    imu = reader(heading_offset_deg=90.0)
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert imu.heading() == pytest.approx(90.0)

    mirrored = reader(invert=True)
    mirrored._consume(parse_frame(frame(yaw_deg=90.0)))
    assert mirrored.heading() == pytest.approx(90.0)


def test_the_reader_resynchronises_on_the_header():
    """A dropped byte must cost one frame, not every frame after it."""
    imu = reader(b"\x13\x37" + frame(yaw_deg=45.0))
    assert imu._read_frame() is None            # the stray 0x13
    assert imu._read_frame() is None            # the stray 0x37
    decoded = imu._read_frame()
    assert decoded is not None and decoded.yaw == pytest.approx(45.0)


def test_a_corrupted_frame_is_counted_and_dropped():
    imu = reader(frame(checksum=0x00) + frame(yaw_deg=10.0))
    assert imu._read_frame() is None
    assert imu._read_frame() is not None
    assert imu.frame_counts() == (2, 1)         # two seen, one rejected


def test_a_truncated_stream_does_not_hang_or_lie():
    imu = reader(HEADER + b"\x01\x02")          # a header and then nothing
    assert imu._read_frame() is None


# --- the derived yaw rate ----------------------------------------------------

def test_the_yaw_rate_is_differentiated_from_successive_frames():
    """RVC has no gyro, so this is a difference of two angles. It is the D term
    of the heading PID, which is why it is worth pinning the sign down."""
    imu = reader()
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert imu.yaw_rate() is None               # one sample is not a rate
    time.sleep(0.02)
    imu._consume(parse_frame(frame(yaw_deg=-1.0)))
    # Yaw fell 1 degree CCW, i.e. the robot turned 1 degree CLOCKWISE, so the
    # rate is positive in the project's convention.
    assert imu.yaw_rate() > 0


def test_a_gap_in_the_frames_produces_no_rate_rather_than_an_average():
    """Differentiating across a gap would report an average over a period we
    did not observe, as though it were happening now."""
    imu = reader()
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    imu._last_yaw_at -= 5.0                     # as if frames had stopped
    imu._consume(parse_frame(frame(yaw_deg=90.0)))
    assert imu.yaw_rate() is None


def test_the_heading_still_arrives_across_that_gap():
    """Only the RATE is unknowable across a gap. The orientation is not — the
    frame says where the robot is pointing right now."""
    imu = reader()
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    imu._last_yaw_at -= 5.0
    imu._consume(parse_frame(frame(yaw_deg=-90.0)))
    assert imu.heading() == pytest.approx(90.0)


# --- what this transport gives up --------------------------------------------

def test_there_is_no_calibration_level():
    """RVC frames carry no accuracy field. Reporting 0 would block the heading
    forever behind min_calib; reporting 3 would be a lie. None is the truth, and
    the telemetry frame already renders it as 'no pips'."""
    imu = reader()
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert imu.calibration() is None
    assert imu.heading() is not None            # and it is not blocked by that


def test_min_calib_cannot_gate_a_transport_that_reports_no_accuracy():
    imu = reader()
    imu.min_calib = 3
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert imu.heading() is not None


def test_the_loosened_gate_is_announced_at_every_boot(capsys):
    """A warning that only appears in a docstring is a warning nobody reads."""
    imu = RVCIMU(port="/dev/null-not-a-port")
    imu.start()
    imu.stop()
    out = capsys.readouterr().out
    assert "no calibration accuracy" in out
    assert "min_calib cannot be enforced" in out


def test_calibration_cannot_be_saved_from_here():
    """Output only: there is no command channel to ask over."""
    assert RVCIMU().save_calibration() is False


# --- the shared contract -----------------------------------------------------

def test_staleness_works_the_same_on_this_transport():
    """The rule lives in HeadingSource, so it cannot drift between the two."""
    imu = reader(sample_timeout=2.0)
    imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert imu.heading() is not None
    imu._heading_at -= 5.0
    imu._rate_at -= 5.0
    assert imu.heading() is None
    assert imu.fresh() is False


def test_both_readers_answer_the_same_four_questions():
    """PoseEstimator, the waypoint controller and the telemetry frame are wired
    to this contract and to nothing else."""
    for method in ("heading", "yaw_rate", "calibration", "fresh", "has_heading",
                   "start", "stop", "save_calibration"):
        assert callable(getattr(RVCIMU(), method))
        assert callable(getattr(IMU(), method))


def test_a_missing_pyserial_is_inert_not_fatal(monkeypatch, capsys):
    monkeypatch.setattr(rvc_mod, "serial", None)
    imu = RVCIMU()
    imu.start()
    assert imu.is_running() is False
    assert imu.heading() is None
    assert "pyserial not installed" in capsys.readouterr().out


def test_an_unopenable_port_says_which_uart_is_already_taken(capsys):
    imu = RVCIMU(port="/dev/definitely-not-here")
    imu.start()
    for _ in range(50):                        # the open happens on the thread
        if not imu.is_running():
            break
        time.sleep(0.02)
    imu.stop()
    out = capsys.readouterr().out
    assert "could not open /dev/definitely-not-here" in out
    assert "GPS owns /dev/ttyAMA0" in out      # the actual cause, most of the time


# --- choosing the transport --------------------------------------------------

def test_the_config_picks_the_reader():
    assert isinstance(build_imu(IMUConfig(mode="uart_rvc")), RVCIMU)
    assert isinstance(build_imu(IMUConfig(mode="i2c")), IMU)


def test_a_disabled_imu_builds_nothing():
    assert build_imu(IMUConfig(enabled=False)) is None


def test_an_unknown_mode_falls_back_rather_than_refusing_to_boot(capsys):
    """A typo in an env var must cost a heading source, not a rover."""
    imu = build_imu(IMUConfig(mode="uart-rvc"))   # a hyphen, not an underscore
    assert isinstance(imu, IMU)
    assert "unknown mode" in capsys.readouterr().out


def test_the_shipped_default_is_the_checksummed_transport():
    assert RobotConfig().imu.mode == "uart_rvc"


def test_the_rvc_port_is_not_the_gps_port():
    """They are two different UARTs and always will be — the GPS holds
    /dev/ttyAMA0 on every build of this rover."""
    cfg = RobotConfig()
    assert cfg.imu.port != cfg.gps.port


# --- through the robot -------------------------------------------------------

def test_the_robot_builds_the_rvc_reader_and_reports_no_pips(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    from robot.control.commands import DriveCommand
    from robot.robot import Robot

    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    bot = Robot(cfg)
    assert isinstance(bot.imu, RVCIMU)
    bot.imu._consume(parse_frame(frame(yaw_deg=0.0)))
    assert bot.imu.heading() == pytest.approx(0.0)
    telemetry = bot._telemetry(DriveCommand.stopped())
    assert telemetry["imu_calib"] is None       # nothing to show, so nothing shown


def test_persist_calibration_is_not_pushed_at_a_reader_that_cannot_use_it(
        monkeypatch, tmp_path):
    """Setting it anyway would create the attribute and leave a dashboard
    control that reads back as applied while doing nothing."""
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    from robot.robot import Robot

    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    bot = Robot(cfg)
    bot._set_config({"config": {"imu.persist_calibration": False}, "save": False})
    assert not hasattr(bot.imu, "persist_calibration")
