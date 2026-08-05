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


# --- per-motor speed trim ----------------------------------------------------

def _trimmed(scale, throttle=1.0, reverse=None):
    """One motor at `throttle`, trimmed by `scale`. Returns its servo angle.

    `reverse` trims the backwards direction when it differs; by default both
    directions get `scale`, which is the symmetric case the old single trim
    covered.
    """
    from robot.drive.motor import ESCMotor
    m = ESCMotor(MotorConfig(channel=0, name="m", neutral_angle=5.0,
                             max_angle=40.0, min_angle=-30.0,
                             speed_scale_forward=scale,
                             speed_scale_reverse=scale if reverse is None else reverse))
    m.set_throttle(throttle)
    return m.servo._last


def test_the_speed_trim_scales_the_angle_away_from_neutral():
    """Full throttle on a 35 degree throw lands on 40; at 0.9 it lands 10% of
    the THROW short of it, not 10% short of the raw angle."""
    assert _trimmed(1.0) == pytest.approx(40.0)
    assert _trimmed(0.9) == pytest.approx(5.0 + 0.9 * 35.0)


def test_the_speed_trim_applies_in_both_directions():
    """Set both to the same number and it corrects the mismatch either way — a
    side trimmed only going forward would still pull in reverse."""
    assert _trimmed(0.9, -1.0) == pytest.approx(5.0 - 0.9 * 35.0)


def test_each_direction_carries_its_own_trim():
    """The mismatch is rarely the same forwards as backwards: an ESC's reverse
    gain differs from its forward gain, and one compromise number is wrong in
    both directions."""
    assert _trimmed(0.75, 1.0, reverse=0.8) == pytest.approx(5.0 + 0.75 * 35.0)
    assert _trimmed(0.75, -1.0, reverse=0.8) == pytest.approx(5.0 - 0.8 * 35.0)


def test_a_reverse_trim_does_not_touch_forward():
    """Trimming one direction must not spend the other — that is the whole
    reason these are two fields and not one."""
    assert _trimmed(1.0, 1.0, reverse=0.5) == pytest.approx(5.0 + 35.0)


def test_the_speed_trim_does_not_move_the_dead_band():
    """Trimming must not change WHERE a motor starts moving, or the two sides
    would break away at different stick positions — the very thing it fixes."""
    dead = 0.02  # below the 0.03 default dead band
    assert _trimmed(1.0, dead) == pytest.approx(5.0)
    assert _trimmed(0.5, dead) == pytest.approx(5.0)


def test_the_default_trim_is_inert():
    """1.0 must map exactly as it did before the field existed, or every
    deployed tuning.json and layout.json would quietly shift when it landed."""
    for throttle in (-1.0, -0.4, 0.4, 1.0):
        assert _trimmed(1.0, throttle) == pytest.approx(5.0 + throttle * 35.0)


def test_trimming_the_faster_side_evens_a_tank_drive_up():
    """The actual use: same command, two motors, matched output."""
    cfg = DriveConfig(actuators={
        "left": MotorConfig(channel=0, name="left"),
        "right": MotorConfig(channel=1, name="right", speed_scale_forward=0.9,
                             speed_scale_reverse=0.9),
    }, roles=DriveRoles(left=["left"], right=["right"]), slew_rate=0)
    dt = TankDrivetrain(cfg)
    dt.drive(1.0, 1.0)
    left, right = dt.motors["left"], dt.motors["right"]
    # Both were COMMANDED the same; only what reaches the hardware differs.
    assert left.throttle == right.throttle == pytest.approx(1.0)
    assert right.servo._last < left.servo._last
    assert right.servo._last == pytest.approx(
        cfg.actuators["right"].neutral_angle + 0.9 * 30.0)


def test_the_trim_survives_a_round_trip_through_a_layout():
    """It is a layout field, so a saved document has to carry it — a trim that
    silently reset to 1.0 on reboot would read as the robot drifting again."""
    cfg = RobotConfig()
    cfg.drive.actuators["right"].speed_scale_forward = 0.75
    cfg.drive.actuators["right"].speed_scale_reverse = 0.8
    result = layout.apply(RobotConfig(), layout.to_doc(cfg))
    assert result.ok, result.errors
    right = result.drive.actuators["right"]
    assert right.speed_scale_forward == pytest.approx(0.75)
    assert right.speed_scale_reverse == pytest.approx(0.8)


def test_the_document_still_carries_the_retired_key_for_older_robots():
    """A layout is pushed independently of the rover's code, so a document from
    this build lands on older ones — which skip keys they don't know and end up
    UNTRIMMED, i.e. worse than before the trim was ever touched."""
    cfg = RobotConfig()
    cfg.drive.actuators["right"].speed_scale_forward = 0.75
    cfg.drive.actuators["right"].speed_scale_reverse = 0.8
    doc = layout.to_doc(cfg)
    right = next(a for a in doc["drive"]["actuators"] if a["name"] == "right")
    assert right["speed_scale"] == pytest.approx(0.75)  # the driven direction


def test_the_retired_key_never_wins_on_read_back():
    """It is written for old robots and ignored by this one — a doc carrying
    both must not have the compatibility value undo the real one."""
    cfg = RobotConfig()
    cfg.drive.actuators["right"].speed_scale_reverse = 0.8
    doc = layout.to_doc(cfg)
    result = layout.apply(RobotConfig(), doc)
    assert result.ok, result.errors
    assert result.drive.actuators["right"].speed_scale_reverse == pytest.approx(0.8)


def test_a_pre_split_layout_still_carries_its_trim():
    """`speed_scale` was one number for both directions. A document written
    before the split — a rover's saved layout, a hand-edited file — must not
    come back un-calibrated, pulling to one side with nothing in the log."""
    doc = layout.to_doc(RobotConfig())
    for actuator in doc["drive"]["actuators"]:
        actuator.pop("speed_scale_forward")
        actuator.pop("speed_scale_reverse")
        if actuator["name"] == "right":
            actuator["speed_scale"] = 0.9
    result = layout.apply(RobotConfig(), doc)
    assert result.ok, result.errors
    right = result.drive.actuators["right"]
    assert right.speed_scale_forward == pytest.approx(0.9)
    assert right.speed_scale_reverse == pytest.approx(0.9)
