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


def test_shooter_spins_its_flywheel_to_minus_fifty():
    """Direction matters as much as magnitude: this wheel launches the wrong
    way at +50."""
    cfg = _config("shooter")
    assert _angle(cfg, "dumper", "run") == pytest.approx(-50.0)


def test_shooter_runs_its_feeder_at_plus_fifty():
    cfg = _config("shooter")
    assert _angle(cfg, "feeder", "run") == pytest.approx(50.0)


def test_the_flywheel_winds_up_on_a_ramp_and_nothing_else_does():
    """A step from neutral to full is what trips this ESC's protection. The
    other three are low-inertia and want to be instant."""
    cfg = _config("shooter")
    assert cfg.mechanisms["dumper"].slew_rate > 0
    for name in ("intake", "feeder", "agitator"):
        assert cfg.mechanisms[name].slew_rate == 0, name


def test_shooter_agitates_both_ways():
    """Direction is what matters here, not the magnitude: it only has to stir.
    Neutral 5 means +-90 is not reachable both ways, since the throw is the
    narrower side of neutral."""
    cfg = _config("shooter")
    forward = _angle(cfg, "agitator", "run")
    reverse = _angle(cfg, "agitator", "reverse")
    assert forward > NEUTRAL and reverse < NEUTRAL
    assert forward == pytest.approx(90.0)
    assert forward - NEUTRAL == pytest.approx(NEUTRAL - reverse), "same speed"


def test_the_held_mechanisms_carry_a_dead_man_and_the_toggles_do_not():
    """auto_stop_seconds only protects controls something is refreshing. The
    feeder and agitator are held (R1, D-pad); the flywheel is a toggle, and a
    dead-man there would stop it a moment after every press."""
    cfg = _config("shooter")
    assert cfg.mechanisms["feeder"].auto_stop_seconds > 0
    assert cfg.mechanisms["agitator"].auto_stop_seconds > 0
    assert cfg.mechanisms["dumper"].auto_stop_seconds == 0


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
    assert cfg.drive.actuators["right"].speed_scale == pytest.approx(0.9)


def test_east_is_not_trimmed():
    cfg = _config("east")
    for actuator in cfg.drive.actuators.values():
        assert actuator.speed_scale == pytest.approx(1.0)
