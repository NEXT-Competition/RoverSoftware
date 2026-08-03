"""The collision guard: what it clamps, what it deliberately doesn't, and when.

Two arguments are being defended here, and both are about NOT over-reaching.

The first is that only forward motion is limited. A rover stopped a hand's width
from a wall must still be able to pivot away and reverse out, and an operator who
has just been overruled needs the controls to still do something — so the command
is split into a forward part and a turn part, and only the forward part is
touched.

The second is that it fails OPEN. An ultrasonic hears nothing when the path is
clear and when it is broken, and the silence is identical; a guard that clamped
on silence would strand a rover in an empty field with no way to say why. So a
missing reading is never an obstacle, and the tests below say so out loud
because it is the kind of decision someone will one day be tempted to "fix".
"""

import pytest

from robot.config import RobotConfig, UltrasonicConfig
from robot.control.collision import CollisionGuard
from robot.control.commands import DriveCommand

FORWARD = DriveCommand.tank(1.0, 1.0)
REVERSE = DriveCommand.tank(-1.0, -1.0)
PIVOT = DriveCommand.tank(1.0, -1.0)


class Sonar:
    """A distance the test sets, and a count of how often it was asked."""

    def __init__(self, distance=None):
        self.distance = distance
        self.asks = 0

    def __call__(self):
        self.asks += 1
        return self.distance


@pytest.fixture
def guard():
    cfg = UltrasonicConfig(enabled=True, avoid=True, stop_m=0.3, slow_m=0.9,
                           release_m=0.1)
    g = CollisionGuard(cfg, Sonar())
    return g


def sonar(guard) -> Sonar:
    return guard._distance


def forward_of(cmd: DriveCommand) -> float:
    return (cmd.left + cmd.right) / 2.0


# --- the zones ---------------------------------------------------------------

def test_far_away_is_not_the_guards_business(guard):
    sonar(guard).distance = 2.0
    assert guard.apply(FORWARD) == FORWARD
    assert guard.state == "clear"


def test_the_slow_zone_scales_forward_linearly(guard):
    """0.6 m is exactly halfway between the 0.3 m stop and the 0.9 m slow-down,
    so half the commanded throttle survives. The run-in matters more than the
    stop: braking from cruise in one tick is a lurch the slew limiter then has
    to absorb, and on a light chassis it is how you tip a rover onto its nose."""
    sonar(guard).distance = 0.6
    assert forward_of(guard.apply(FORWARD)) == pytest.approx(0.5)
    assert guard.state == "slow"


def test_inside_the_stop_distance_forward_is_refused(guard):
    sonar(guard).distance = 0.2
    assert forward_of(guard.apply(FORWARD)) == pytest.approx(0.0)
    assert guard.blocked and guard.state == "stop"


def test_a_slow_distance_at_or_below_the_stop_is_a_hard_stop():
    """A build that asked for no run-in gets none: full throttle right up to the
    threshold, nothing past it."""
    cfg = UltrasonicConfig(enabled=True, stop_m=0.3, slow_m=0.3)
    g = CollisionGuard(cfg, Sonar(0.31))
    assert g.apply(FORWARD) == FORWARD


# --- what it must never touch ------------------------------------------------

def test_reverse_is_never_clamped(guard):
    """Backing away from the wall is the way out of the situation. A guard that
    took that too would be a trap."""
    sonar(guard).distance = 0.05
    assert guard.apply(REVERSE) == REVERSE


def test_a_pivot_in_place_is_never_clamped(guard):
    """left = -right is zero forward motion — the rover turns to face somewhere
    else, which is the other way out."""
    sonar(guard).distance = 0.05
    assert guard.apply(PIVOT) == PIVOT


def test_steering_keeps_full_authority_while_forward_is_scaled(guard):
    """The whole reason the command is decomposed. Scaling left and right
    together would shrink the turn along with the throttle, so the rover would
    steer away from the obstacle more and more slowly the closer it got."""
    sonar(guard).distance = 0.6                  # scale 0.5
    out = guard.apply(DriveCommand.tank(1.0, 0.5))  # forward 0.75, turn 0.25
    assert forward_of(out) == pytest.approx(0.375)
    assert (out.left - out.right) / 2.0 == pytest.approx(0.25)


def test_a_stopped_rover_can_still_turn_on_the_spot_out_of_a_block(guard):
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    assert guard.blocked
    assert guard.apply(PIVOT) == PIVOT


# --- hysteresis ---------------------------------------------------------------

def test_clearing_the_stop_distance_is_not_enough_to_release(guard):
    """0.35 m is past the 0.3 m stop and inside the 0.1 m release margin. Without
    the margin, a rover parked on the threshold with a jittering reading answers
    with a throttle that switches on and off every 20 ms."""
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    sonar(guard).distance = 0.35
    assert forward_of(guard.apply(FORWARD)) == pytest.approx(0.0)
    assert guard.blocked


def test_real_clearance_releases_the_latch(guard):
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    sonar(guard).distance = 0.45
    assert forward_of(guard.apply(FORWARD)) == pytest.approx(0.25)
    assert not guard.blocked


def test_backing_away_releases_it_too(guard):
    """The latch tracks the MEASUREMENT, not the command, so reversing out of a
    block and then driving forward acts on what the sensor says now."""
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    sonar(guard).distance = 1.5
    guard.apply(REVERSE)                 # not clamped, but still measured
    assert not guard.blocked
    assert guard.apply(FORWARD) == FORWARD


# --- failing open -------------------------------------------------------------

def test_no_reading_means_no_intervention(guard):
    """An open field and an unplugged sensor produce the same silence. Clamping
    on it would stop the rover in the middle of nowhere and tell nobody why."""
    sonar(guard).distance = None
    assert guard.apply(FORWARD) == FORWARD


def test_silence_does_not_release_a_latched_block(guard):
    """The other half of that: silence is not evidence the wall went away, so a
    blocked rover has to SEE clearance before it drives forward again."""
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    sonar(guard).distance = None
    assert guard.apply(FORWARD) == FORWARD   # fails open, as it must
    assert guard.blocked                     # but has not forgotten
    sonar(guard).distance = 0.35
    assert forward_of(guard.apply(FORWARD)) == pytest.approx(0.0)


def test_a_build_with_no_sensor_at_all_is_untouched():
    """Every build that existed before this module did. The guard is
    constructed unconditionally, so this is the common case, not a corner one."""
    g = CollisionGuard(UltrasonicConfig(), None)
    assert g.apply(FORWARD) == FORWARD
    assert g.state == "off"


# --- the switches -------------------------------------------------------------

def test_avoid_off_measures_without_intervening(guard):
    """The escape hatch, and it is live: the moment you want it is the moment
    the sensor is the thing misbehaving, in a field, with the rover refusing to
    go forward."""
    guard.cfg.avoid = False
    sonar(guard).distance = 0.05
    assert guard.apply(FORWARD) == FORWARD
    assert guard.state == "off"
    assert sonar(guard).asks == 1        # still reading, still on telemetry


def test_a_disabled_sensor_is_not_even_asked(guard):
    guard.cfg.enabled = False
    sonar(guard).distance = 0.05
    assert guard.apply(FORWARD) == FORWARD
    assert sonar(guard).asks == 0


def test_it_says_it_is_holding_once_not_every_tick(guard, capsys):
    sonar(guard).distance = 0.1
    for _ in range(10):
        guard.apply(FORWARD)
    assert capsys.readouterr().out.count("[Collision] holding") == 1


# --- telemetry ----------------------------------------------------------------

def test_status_carries_the_sensors_summary_and_the_verdict(guard):
    """One `sonar` object on the frame, not two nearly-identical ones. `state`
    is the half that cannot be inferred from the distance — an operator whose
    rover has stopped responding to forward needs to see WHY on the same frame."""
    sonar(guard).distance = 0.1
    guard.apply(FORWARD)
    assert guard.status({"d": 0.1}) == {"d": 0.1, "state": "stop"}


def test_status_works_without_a_sensor_summary(guard):
    assert guard.status() == {"state": "clear"}


# --- fitting it from robot.env ------------------------------------------------

def test_setting_the_pins_fits_the_sensor(monkeypatch):
    import run_robot
    monkeypatch.setenv("RS_ULTRASONIC_PINS", "27,22")
    cfg = RobotConfig()
    run_robot._apply_ultrasonic_env(cfg)
    assert cfg.ultrasonic.enabled
    assert (cfg.ultrasonic.trig_pin, cfg.ultrasonic.echo_pin) == (27, 22)


def test_a_typo_in_the_pins_leaves_the_rover_exactly_as_it_was(monkeypatch, capsys):
    """This runs at boot from a file somebody edited over SSH. A broken guard
    must behave like no guard, never like a guard that clamps on nothing."""
    import run_robot
    monkeypatch.setenv("RS_ULTRASONIC_PINS", "twenty-seven")
    cfg = RobotConfig()
    run_robot._apply_ultrasonic_env(cfg)
    assert not cfg.ultrasonic.enabled
    assert "ultrasonic disabled" in capsys.readouterr().out


def test_pins_shared_with_an_encoder_are_refused(monkeypatch, capsys):
    """Two claimants on one line is not an error either library reports — it is
    an encoder that counts nothing, or a sensor that hears nothing, found days
    later. The encoder was configured first and feeds a control loop, so the
    ultrasonic is the one that loses, out loud."""
    import run_robot
    monkeypatch.setenv("RS_ULTRASONIC_PINS", "17,22")
    cfg = RobotConfig()
    cfg.drive.actuators["left"].encoder_a = 17
    cfg.drive.actuators["left"].encoder_b = 27
    run_robot._apply_ultrasonic_env(cfg)
    assert not cfg.ultrasonic.enabled
    assert "already belongs to the 'left' encoder" in capsys.readouterr().out


# --- through the robot --------------------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.ultrasonic.enabled = True
    from robot.robot import Robot
    bot = Robot(cfg)
    # The sensor never starts (no fusion_hat off the Pi), so stand in for its
    # cached lookup — which is exactly the seam the control loop uses.
    bot.collision.set_distance_provider(Sonar(None))
    return bot


def test_the_guard_sits_between_the_controller_and_the_drivetrain(rover):
    """The wiring, checked where it matters: a full-forward teleop command with
    a wall in front of the rover reaches the motors as a stop.

    This is the same call `Robot.run` makes on every tick, in the same order —
    controller, then guard, then drive."""
    rover.manager.handle_message({"type": "drive", "throttle": 1.0, "steer": 0.0})
    rover.collision._distance.distance = 0.1
    cmd = rover.collision.apply(rover.manager.update(0.02))
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_the_same_command_drives_when_the_path_is_clear(rover):
    rover.manager.handle_message({"type": "drive", "throttle": 1.0, "steer": 0.0})
    rover.collision._distance.distance = 3.0
    cmd = rover.collision.apply(rover.manager.update(0.02))
    assert cmd.left > 0 and cmd.right > 0


def test_the_distance_and_the_verdict_reach_telemetry(rover):
    rover.collision._distance.distance = 0.1
    rover.collision.apply(DriveCommand.tank(1.0, 1.0))
    sonar_frame = rover._telemetry(DriveCommand.stopped())["sonar"]
    assert sonar_frame["state"] == "stop"


def test_a_build_without_an_ultrasonic_puts_nothing_on_the_radio(monkeypatch, tmp_path):
    """It costs those builds nothing, which is the same bet the encoders make."""
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    from robot.robot import Robot
    bot = Robot(cfg)
    assert bot.ultrasonic is None
    assert "sonar" not in bot._telemetry(DriveCommand.stopped())


def test_thresholds_are_tunable_from_the_dashboard_without_a_restart(rover):
    """You find them by driving at a wall and nudging the number. A restart per
    attempt is a loop nobody completes."""
    rover.collision._distance.distance = 1.0     # past the shipped slow_m
    assert rover.collision.apply(DriveCommand.tank(1.0, 1.0)) == DriveCommand.tank(1.0, 1.0)
    rover._set_config({"config": {"ultrasonic.stop_m": 1.2}, "save": False})
    assert forward_of(rover.collision.apply(DriveCommand.tank(1.0, 1.0))) == 0.0


def test_switching_avoidance_off_over_the_radio_frees_the_drivetrain(rover):
    rover.collision._distance.distance = 0.1
    rover.collision.apply(DriveCommand.tank(1.0, 1.0))
    assert rover.collision.blocked
    rover._set_config({"config": {"ultrasonic.avoid": False}, "save": False})
    assert rover.collision.apply(DriveCommand.tank(1.0, 1.0)) == DriveCommand.tank(1.0, 1.0)
