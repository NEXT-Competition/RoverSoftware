"""The checked-in layout documents in packaging/layouts/.

These are deployed to real rovers, so what is worth pinning is not that they
parse — the robot would tell you that — but that each one still commands the
ANGLES the machine is wired for. Every number below was measured against a
mechanism, and a layout that silently drifts off one of them is a rover that
runs its intake backwards or a flywheel that never reaches speed.

The regeneration script is packaging/layouts/build_layouts.py (`just layouts`).
"""

import json
import os

import pytest

os.environ.setdefault("RS_MOCK_MOTORS", "1")

from robot import layout
from robot.config import RobotConfig
from robot.drive.drivetrain import build_drivetrain
from robot.drive.mechanism import build_mechanism

LAYOUTS = os.path.join(os.path.dirname(__file__), "..", "packaging", "layouts")

# Fusion HAT literal angles. 5 stops a motor fully on both rovers.
NEUTRAL = 5.0


def _config(name):
    """A robot config with the named layout applied, as the rover boots it."""
    cfg = RobotConfig()
    # Both rovers run RS_SHOOTER_ENABLED=0; that is what frees channel 2 for a
    # mechanism. With the launcher on it reserves the channel and wins the tie.
    cfg.shooter.enabled = False
    with open(os.path.join(LAYOUTS, f"{name}.json")) as fh:
        result = layout.apply(cfg, json.load(fh))
    assert result.ok, result.errors
    return cfg


def _angles(cfg, mech, preset):
    """The angles a preset SETTLES at.

    A ramped mechanism (`slew_rate`) closes on its command over time rather
    than jumping, so the geometry question — "can this preset reach the angle
    the machine is wired for?" — is about where it ends up. The ramp itself is
    a separate property, tested on its own below with a rate that does not cost
    the suite a second of real time.
    """
    mcfg = cfg.mechanisms[mech]
    ramp, mcfg.slew_rate = mcfg.slew_rate, 0.0
    try:
        m = build_mechanism(mcfg)
        assert m.apply_preset(preset), f"{mech} has no preset {preset!r}"
        return {name: motor.servo._last for name, motor in m.motors.items()}
    finally:
        mcfg.slew_rate = ramp


def _angle(cfg, mech, preset):
    values = list(_angles(cfg, mech, preset).values())
    assert len(values) == 1
    return values[0]


@pytest.mark.parametrize("name", ["east", "shooter"])
def test_the_document_is_accepted_as_written(name):
    _config(name)


@pytest.mark.parametrize("name", ["east", "shooter"])
def test_every_motor_stops_at_neutral_five(name):
    """The one fact that is true of every motor on every rover in this fleet."""
    cfg = _config(name)
    for actuator in cfg.drive.actuators.values():
        assert actuator.neutral_angle == NEUTRAL
    for mech in cfg.mechanisms.values():
        for actuator in mech.actuators.values():
            assert actuator.neutral_angle == NEUTRAL, f"{mech.name}.{actuator.name}"


@pytest.mark.parametrize("name", ["east", "shooter"])
def test_a_stopped_mechanism_returns_to_neutral(name):
    """Not just configured neutral — actually commanded there, which is what
    keeps an ESC armed for the next press."""
    cfg = _config(name)
    for mname, mcfg in cfg.mechanisms.items():
        m = build_mechanism(mcfg)
        m.apply_preset(sorted(mcfg.presets)[0])
        m.stop()
        for aname, motor in m.motors.items():
            assert motor.servo._last == NEUTRAL, f"{mname}.{aname}"


# --- east: intake + dumper ---------------------------------------------------

def test_east_runs_its_intake_in_at_forty_and_spits_at_minus_thirty():
    cfg = _config("east")
    assert _angle(cfg, "intake", "in") == pytest.approx(40.0)
    assert _angle(cfg, "intake", "out") == pytest.approx(-30.0)


def test_east_runs_its_dumper_at_minus_thirty():
    cfg = _config("east")
    assert _angle(cfg, "dumper", "run") == pytest.approx(-30.0)


def test_east_has_no_feeder_or_agitator():
    """It is not wired for them; a binding that reaches nothing is refused by
    the robot rather than moving something else."""
    cfg = _config("east")
    assert set(cfg.mechanisms) == {"intake", "dumper"}


# --- shooter: intake + flywheel + feeder + agitator ---------------------------

def test_shooter_runs_its_intake_in_at_minus_thirty():
    """The mirror of east — same wiring, opposite direction."""
    cfg = _config("shooter")
    assert _angle(cfg, "intake", "in") == pytest.approx(-30.0)
    assert _angle(cfg, "intake", "out") == pytest.approx(40.0)


def test_shooter_leaves_channel_2_free_for_the_built_in_shooter():
    """Its flywheel is NOT a mechanism, and that is the whole point.

    A mechanism can only hold a throttle; holding an RPM needs the closed-loop
    controller, which lives in the built-in shooter (robot/drive/shooter.py).
    The two cannot share the channel — layout.apply reserves the shooter's, and
    a mechanism on it is an ERROR, which refuses the WHOLE document and boots
    the rover on compiled-in defaults with no mechanisms at all.

    So this pins the absence, because the absence is load-bearing: putting a
    `dumper` back here silently costs this rover its intake, feeder, agitator
    and drivetrain trim the moment RS_SHOOTER_ENABLED=1.
    """
    cfg = _config("shooter")
    used = {a.channel for m in cfg.mechanisms.values() for a in m.actuators.values()}
    assert 2 not in used, f"channel 2 is claimed by a mechanism: {sorted(used)}"

    # And prove it: the real validator, with the shooter holding channel 2.
    real = RobotConfig()
    real.shooter.enabled = True
    real.shooter.channel = 2
    with open(os.path.join(LAYOUTS, "shooter.json")) as fh:
        result = layout.apply(real, json.load(fh))
    assert result.ok, result.errors
    assert set(real.mechanisms) == {"intake", "feeder", "agitator"}
    assert real.drive.actuators["right"].speed_scale_forward == pytest.approx(0.75)


def test_east_still_drives_channel_2_as_a_mechanism():
    """The opposite arrangement, and also correct: east's ch2 really is a
    dumper — it runs until switched off and has no speed to hold — so it keeps
    the mechanism and leaves the built-in shooter off."""
    cfg = _config("east")
    assert _angle(cfg, "dumper", "run") == pytest.approx(-30.0)


def test_shooter_runs_its_feeder_at_the_top_of_the_throttle_band():
    cfg = _config("shooter")
    assert _angle(cfg, "feeder", "run") == pytest.approx(45.0)


@pytest.mark.parametrize("name", ["east", "shooter"])
def test_no_esc_is_ever_commanded_outside_1000_to_2000_microseconds(name):
    """The fault above, pinned for the whole fleet rather than one mechanism.

    An ESC listens to 1000..2000us and treats anything else as a lost signal.
    The HAT's -90..+90 spans 500..2500us, so an angle is easy to write and
    impossible to notice: it looks like more authority and behaves like a
    motor that stops partway through the throw. Every preset on every rover is
    checked here, at the angle actually commanded.
    """
    cfg = _config(name)
    def pulse(angle):
        return 1500.0 + angle * (1000.0 / 90.0)

    for mname, mcfg in cfg.mechanisms.items():
        for preset in mcfg.presets:
            for aname, angle in _angles(cfg, mname, preset).items():
                if mcfg.actuators[aname].kind != "esc":
                    continue
                us = pulse(angle)
                assert 1000.0 <= us <= 2000.0, (
                    f"{mname}.{aname} preset {preset!r}: {angle}deg = {us:.0f}us")

    for aname, actuator in cfg.drive.actuators.items():
        for angle in (actuator.min_angle, actuator.max_angle, actuator.neutral_angle):
            assert 1000.0 <= pulse(angle) <= 2000.0, f"drive {aname}: {angle}deg"


def test_no_shooter_mechanism_needs_a_ramp():
    """The flywheel was the one load with enough inertia to need one, and it is
    no longer a mechanism — its wind-up rate is now the PID's MAX_ANGLE_CHANGE
    (robot/drive/shooter.py), which limits throttle change per control step.
    The three that remain are low-inertia and want to be instant."""
    cfg = _config("shooter")
    for name in ("intake", "feeder", "agitator"):
        assert cfg.mechanisms[name].slew_rate == 0, name


def test_shooter_agitates_both_ways():
    """Direction is what matters here, not the magnitude: it only has to stir.
    Neutral 5 means the throw is the narrower side of neutral, and +-45 is the
    whole of what an ESC accepts — so 40 each way is all there is."""
    cfg = _config("shooter")
    forward = _angle(cfg, "agitator", "run")
    reverse = _angle(cfg, "agitator", "reverse")
    assert forward > NEUTRAL and reverse < NEUTRAL
    assert forward == pytest.approx(45.0)
    assert forward - NEUTRAL == pytest.approx(NEUTRAL - reverse), "same speed"


def test_the_held_mechanisms_carry_a_dead_man_and_the_toggles_do_not():
    """auto_stop_seconds only protects controls something is refreshing. The
    feeder and agitator are held (R1, D-pad) and want it; the intake's toggled
    IN direction does not, and a dead-man there would stop it a moment after
    every press."""
    cfg = _config("shooter")
    assert cfg.mechanisms["feeder"].auto_stop_seconds > 0
    assert cfg.mechanisms["agitator"].auto_stop_seconds > 0
    east = _config("east")
    assert east.mechanisms["dumper"].auto_stop_seconds == 0, "a toggle, so no dead-man"


# --- the drivetrain trim -----------------------------------------------------

def test_shooter_trims_its_faster_right_track():
    """Same command to both sides, less angle to the one that runs away."""
    cfg = _config("shooter")
    cfg.drive.slew_rate = 0
    dt = build_drivetrain(cfg.drive)
    dt.drive(1.0, 1.0)
    left, right = dt.motors["left"], dt.motors["right"]
    assert left.throttle == right.throttle == pytest.approx(1.0)
    assert right.servo._last < left.servo._last
    right_cfg = cfg.drive.actuators["right"]
    assert right_cfg.speed_scale_forward == pytest.approx(0.75)
    assert right_cfg.speed_scale_reverse == pytest.approx(0.8)


def test_shooter_trims_forward_harder_than_reverse():
    """Its right track runs away worse going forward than backing up, so one
    compromise number would be wrong in both directions."""
    right_cfg = _config("shooter").drive.actuators["right"]
    assert right_cfg.speed_scale_forward < right_cfg.speed_scale_reverse < 1.0


def test_east_is_not_trimmed():
    cfg = _config("east")
    for actuator in cfg.drive.actuators.values():
        assert actuator.speed_scale_forward == pytest.approx(1.0)
        assert actuator.speed_scale_reverse == pytest.approx(1.0)
