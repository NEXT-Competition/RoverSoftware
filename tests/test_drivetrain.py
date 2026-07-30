"""Drivetrain kinds: turning one DriveCommand into whatever this build has.

The load-bearing test here is the first one. `DriveCommand(left, right)` is the
only command type in the system, and every controller was written against a
two-motor tank — so the proof that generalizing the drivetrain changed nothing
is that the tank kind still produces byte-identical servo angles.
"""

import pytest

from robot import layout
from robot.config import DriveConfig, DriveRoles, MotorConfig, RobotConfig
from robot.drive.drivetrain import (
    NullDrivetrain,
    SingleDrivetrain,
    SteeredDrivetrain,
    TankDrivetrain,
    build_drivetrain,
)
from robot.drive.tank_drive import TankDrive


@pytest.fixture(autouse=True)
def mock_motors(monkeypatch):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")


def angles(dt):
    return {name: m.servo._last for name, m in dt.motors.items()}


# --- tank: the regression guard ---------------------------------------------

SWEEP = [(0, 0), (1, 1), (-1, -1), (0.5, -0.5), (0.02, 0.02), (2, -2), (0.3, 0.7)]


def test_tank_reproduces_the_original_two_motor_behaviour():
    """The whole compatibility argument, measured at the servo.

    ESCMotor's mapping is unchanged, so what this really pins down is that the
    drivetrain still clamps, still slews, and still sends each side's value to
    the same motor it always did.
    """
    cfg = DriveConfig()
    cfg.slew_rate = 0  # measure the command, not the ramp
    dt = TankDrivetrain(cfg)
    for left, right in SWEEP:
        dt.drive(left, right)
        want_left = cfg.actuators["left"]
        clamped = max(-1.0, min(1.0, left))
        throw = min(want_left.max_angle - want_left.neutral_angle,
                    want_left.neutral_angle - want_left.min_angle)
        expected = (want_left.neutral_angle if abs(clamped) < want_left.deadband
                    else want_left.neutral_angle + clamped * throw)
        assert dt.left.servo._last == pytest.approx(expected)


def test_tank_drive_is_still_importable_and_constructs_from_a_driveconfig():
    """tools/ and the tests import this name; it must keep working."""
    dt = TankDrive(DriveConfig())
    assert isinstance(dt, TankDrivetrain)
    assert dt.left is not dt.right


def test_tank_fans_each_side_out_to_every_motor_on_it():
    cfg = DriveConfig(
        actuators={
            "l1": MotorConfig(channel=0, name="l1"),
            "l2": MotorConfig(channel=1, name="l2"),
            "r1": MotorConfig(channel=2, name="r1"),
            "r2": MotorConfig(channel=3, name="r2"),
        },
        roles=DriveRoles(left=["l1", "l2"], right=["r1", "r2"]),
    )
    cfg.slew_rate = 0
    dt = TankDrivetrain(cfg)
    dt.drive(0.6, -0.6)
    got = angles(dt)
    assert got["l1"] == got["l2"]
    assert got["r1"] == got["r2"]
    assert got["l1"] != got["r1"]


def test_the_slew_limiter_still_bounds_how_fast_a_side_may_change():
    cfg = DriveConfig()
    cfg.slew_rate = 1.0
    dt = TankDrivetrain(cfg)
    dt.drive(0.0, 0.0)   # establishes the clock
    dt.drive(1.0, 1.0)   # microseconds later, so the step must be tiny
    assert abs(dt.left.throttle) < 0.1


def test_stop_zeroes_everything_and_forgets_the_ramp():
    cfg = DriveConfig()
    dt = TankDrivetrain(cfg)
    dt.drive(1.0, 1.0)
    dt.stop()
    assert dt.left.throttle == 0.0
    assert dt.right.throttle == 0.0


# --- servo_steer -------------------------------------------------------------

def steered(**kw):
    cfg = DriveConfig(
        kind="servo_steer",
        actuators={
            "rear": MotorConfig(channel=0, name="rear", kind="esc"),
            "steering": MotorConfig(channel=3, name="steering", kind="servo"),
        },
        roles=DriveRoles(throttle=["rear"], steer="steering"),
    )
    cfg.slew_rate = 0
    cfg.min_pivot_throttle = 0.0  # off unless a test asks for it
    for k, v in kw.items():
        setattr(cfg, k, v)
    return SteeredDrivetrain(cfg)


def test_servo_steer_inverts_arcade_exactly():
    """arcade() produced these left/right values; recovering throttle and steer
    from them is the exact inverse, which is what lets object_align and waypoint
    drive a steered chassis with no changes at all."""
    dt = steered()
    dt.drive(0.8, 0.2)  # arcade(throttle=0.5, steer=0.3)
    assert dt.motors["rear"].throttle == pytest.approx(0.5)
    assert dt.motors["steering"].throttle == pytest.approx(0.3)


def test_servo_steer_drives_straight_with_no_steer():
    dt = steered()
    dt.drive(0.4, 0.4)
    assert dt.motors["rear"].throttle == pytest.approx(0.4)
    assert dt.motors["steering"].throttle == pytest.approx(0.0)


def test_steer_gain_scales_the_servos_share_of_its_throw():
    dt = steered(steer_gain=0.5)
    dt.drive(0.5, -0.5)  # steer = 0.5
    assert dt.motors["steering"].throttle == pytest.approx(0.25)


def test_min_pivot_throttle_creeps_so_the_steering_has_authority():
    """A steered chassis cannot pivot, but the autonomy controllers ask it to —
    they express 'point, then go' as arcade(0, steer). Without the creep the
    robot sits still with its wheels turned until the search timer expires."""
    dt = steered(min_pivot_throttle=0.15)
    dt.drive(0.4, -0.4)  # throttle 0, steer 0.4
    assert dt.motors["rear"].throttle == pytest.approx(0.15)
    assert dt.motors["steering"].throttle == pytest.approx(0.4)


def test_min_pivot_throttle_does_not_touch_a_real_throttle():
    dt = steered(min_pivot_throttle=0.15)
    dt.drive(0.9, 0.3)  # throttle 0.6, steer 0.3
    assert dt.motors["rear"].throttle == pytest.approx(0.6)


def test_min_pivot_throttle_stays_out_of_the_way_when_going_straight():
    dt = steered(min_pivot_throttle=0.15)
    dt.drive(0.0, 0.0)
    assert dt.motors["rear"].throttle == pytest.approx(0.0)


def test_zero_disables_the_creep_entirely():
    dt = steered(min_pivot_throttle=0.0)
    dt.drive(0.4, -0.4)
    assert dt.motors["rear"].throttle == pytest.approx(0.0)


# --- single and none ---------------------------------------------------------

def test_single_averages_the_two_sides_and_discards_steer():
    cfg = DriveConfig(
        kind="single",
        actuators={"main": MotorConfig(channel=0, name="main")},
        roles=DriveRoles(left=[], right=[], throttle=["main"]),
    )
    cfg.slew_rate = 0
    dt = SingleDrivetrain(cfg)
    dt.drive(0.8, 0.2)
    assert dt.motors["main"].throttle == pytest.approx(0.5)


def test_none_accepts_commands_and_moves_nothing():
    cfg = DriveConfig(kind="none", actuators={}, roles=DriveRoles(left=[], right=[]))
    dt = NullDrivetrain(cfg)
    dt.drive(1.0, -1.0)  # must not raise
    dt.stop()
    assert dt.motors == {}


# --- arming ------------------------------------------------------------------

def test_arm_holds_neutral_and_waits_once_for_the_escs():
    cfg = DriveConfig()
    cfg.arm_seconds = 0.02
    dt = TankDrivetrain(cfg)
    dt.arm()
    assert dt.left.servo._last == cfg.actuators["left"].neutral_angle


def test_a_servo_only_drivetrain_does_not_wait_to_arm():
    """There is no ESC to recognise a neutral pulse, so the wait is dead time."""
    import time
    cfg = DriveConfig(
        kind="single",
        actuators={"s": MotorConfig(channel=0, name="s", kind="servo")},
        roles=DriveRoles(left=[], right=[], throttle=["s"]),
    )
    cfg.arm_seconds = 2.0
    dt = SingleDrivetrain(cfg)
    started = time.monotonic()
    dt.arm()
    assert time.monotonic() - started < 0.5


# --- the factory -------------------------------------------------------------

@pytest.mark.parametrize("kind,cls", [
    ("tank", TankDrivetrain), ("servo_steer", SteeredDrivetrain),
    ("single", SingleDrivetrain), ("none", NullDrivetrain),
])
def test_the_factory_builds_the_kind_the_layout_asked_for(kind, cls):
    cfg = DriveConfig(kind=kind)
    assert isinstance(build_drivetrain(cfg), cls)


def test_an_unknown_kind_falls_back_to_something_that_can_be_stopped():
    """Validation is what refuses bad kinds. By the time we are here the robot
    is booting and must end up with a drivetrain regardless."""
    assert isinstance(build_drivetrain(DriveConfig(kind="hovercraft")),
                      TankDrivetrain)


def test_a_layout_reaches_all_the_way_to_the_hardware():
    cfg = RobotConfig()
    layout.apply(cfg, {"version": 1, "drive": {
        "kind": "servo_steer",
        "actuators": [{"name": "rear", "kind": "esc", "channel": 0},
                      {"name": "steering", "kind": "servo", "channel": 3}],
        "roles": {"throttle": ["rear"], "steer": "steering"},
        "slew_rate": 0}})
    dt = build_drivetrain(cfg.drive)
    dt.drive(0.8, 0.2)
    assert isinstance(dt, SteeredDrivetrain)
    assert dt.motors["rear"].throttle == pytest.approx(0.5)


# --- wheel encoders and the speed loop ---------------------------------------
#
# The decoder and the loop have their own files (test_encoder.py,
# test_rpm_trim.py). What is checked here is only the wiring between them and
# the drivetrain — including the case that matters most on a real robot, which
# is a build with no encoders behaving exactly as it did before any of this
# existed.


class FakeEncoder:
    """Stands in for a wired encoder, reporting whatever the test says."""

    def __init__(self, rpm=0.0):
        self._rpm = rpm
        self.samples = 0
        self.started = self.stopped = False

    def start(self):
        self.started = True
        return True

    def stop(self):
        self.stopped = True

    def sample(self, now=None):
        self.samples += 1

    def rpm(self):
        return self._rpm

    def telemetry(self):
        return round(self._rpm, 1)


def tank_with_encoders(left_rpm, right_rpm, **trim):
    cfg = DriveConfig()
    cfg.slew_rate = 0
    for key, value in trim.items():
        setattr(cfg.trim, key, value)
    dt = TankDrivetrain(cfg)
    dt.encoders = {"left": FakeEncoder(left_rpm), "right": FakeEncoder(right_rpm)}
    return dt


def test_a_build_with_no_encoders_reports_nothing_and_drives_as_before():
    """The compatibility claim, stated as a test: encoders are opt-in hardware
    and a robot without them must not pay for the feature in any way."""
    dt = TankDrivetrain(DriveConfig())
    assert dt.encoders == {}
    assert dt.status() is None
    dt.drive(0.5, 0.5)
    assert dt.left.throttle == pytest.approx(0.5)


def test_the_trim_is_inert_until_a_mode_is_chosen():
    """Off by default, however well the encoders are working: this is the one
    thing here that can add throttle nobody asked for."""
    dt = tank_with_encoders(90.0, 110.0)
    dt.drive(0.5, 0.5)
    assert dt.left.throttle == pytest.approx(0.5)
    assert dt.right.throttle == pytest.approx(0.5)


def test_match_mode_corrects_the_side_that_is_running_slow():
    dt = tank_with_encoders(90.0, 110.0, mode="match")
    for _ in range(50):
        dt.drive(0.5, 0.5)
    assert dt.left.throttle > 0.5
    assert dt.right.throttle < 0.5


def test_encoders_are_sampled_every_tick_even_with_the_trim_off():
    """Measuring is useful on its own — it is what you read to set max_rpm in
    the first place — so the readout does not depend on the loop being on."""
    dt = tank_with_encoders(100.0, 100.0)
    dt.drive(0.2, 0.2)
    dt.drive(0.2, 0.2)
    assert dt.encoders["left"].samples == 2


def test_side_rpm_averages_the_encoders_on_a_multi_motor_side():
    """A six-wheel side has three motors and the useful number is how fast the
    TRACK is going."""
    cfg = DriveConfig(
        actuators={"l1": MotorConfig(channel=0, name="l1"),
                   "l2": MotorConfig(channel=1, name="l2"),
                   "r1": MotorConfig(channel=2, name="r1")},
        roles=DriveRoles(left=["l1", "l2"], right=["r1"]))
    dt = TankDrivetrain(cfg)
    dt.encoders = {"l1": FakeEncoder(100.0), "l2": FakeEncoder(80.0),
                   "r1": FakeEncoder(90.0)}
    assert dt.side_rpm(["l1", "l2"]) == pytest.approx(90.0)


def test_a_side_with_no_encoder_reports_no_measurement_rather_than_zero():
    """None is what makes the loop fail open. Zero would make it chase a
    standstill that isn't there."""
    dt = tank_with_encoders(100.0, 100.0, mode="match")
    del dt.encoders["right"]
    assert dt.side_rpm(["right"]) is None
    for _ in range(50):
        dt.drive(0.5, 0.5)
    assert dt.left.throttle == pytest.approx(0.5)


def test_stopping_releases_the_loop_including_a_latched_fault():
    dt = tank_with_encoders(0.0, 100.0, mode="match", stall_seconds=0.0)
    dt.trim._fault = "left"
    dt.stop()
    assert dt.trim.fault == ""


def test_arming_starts_the_encoders_and_shutdown_stops_them():
    """Claimed while the ESCs hold neutral, so the first speed sample is taken
    against a known standstill."""
    dt = tank_with_encoders(0.0, 0.0)
    dt.cfg.arm_seconds = 0
    dt.arm()
    assert dt.encoders["left"].started
    dt.shutdown()
    assert dt.encoders["left"].stopped


def test_status_reports_rpm_per_actuator_and_what_the_trim_did():
    dt = tank_with_encoders(90.0, 110.0, mode="match")
    dt.drive(0.5, 0.5)
    status = dt.status()
    assert status["rpm"] == {"left": 90.0, "right": 110.0}
    assert status["mode"] == "match"
