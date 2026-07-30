"""The robot's tunable-parameter whitelist.

These tests exist because this module is the only thing standing between a
browser and the rover's drivetrain. The interesting cases are all about what
happens to input that *isn't* well-formed: an unknown path, a value past its
limit, a string where a float belongs, a tuning file someone hand-edited badly.
None of those may crash the robot or move a value somewhere unsafe.
"""

import json

import pytest

from robot import tuning
from robot.config import RobotConfig


def test_snapshot_covers_every_declared_parameter():
    snap = tuning.snapshot(RobotConfig())
    assert set(snap) == {p.path for p in tuning.PARAMS}


def test_snapshot_reads_the_real_config():
    cfg = RobotConfig()
    cfg.align.pid.kp = 0.77
    cfg.drive.left.deadband = 0.11
    snap = tuning.snapshot(cfg)
    assert snap["align.pid.kp"] == 0.77
    assert snap["drive.left.deadband"] == 0.11


def test_every_path_resolves():
    """A typo in PARAMS must fail here, not at 3 a.m. in a field."""
    cfg = RobotConfig()
    for param in tuning.PARAMS:
        tuning.get(cfg, param.path)  # raises AttributeError if the path is wrong


def test_apply_writes_through_to_nested_dataclasses():
    cfg = RobotConfig()
    applied, rejected = tuning.apply(cfg, {"nav.heading_pid.kd": 0.25})
    assert applied == {"nav.heading_pid.kd": 0.25}
    assert rejected == {}
    assert cfg.nav.heading_pid.kd == 0.25


def test_unknown_paths_are_rejected_not_written():
    cfg = RobotConfig()
    applied, rejected = tuning.apply(cfg, {"__class__": "x", "drive.nope": 1})
    assert applied == {}
    assert set(rejected) == {"__class__", "drive.nope"}


def test_numbers_are_clamped_rather_than_refused():
    """A slider pinned at its limit is honest; silently dropping the update
    looks like a broken UI."""
    cfg = RobotConfig()
    applied, _ = tuning.apply(cfg, {"align.pid.kp": 999, "vision.min_confidence": -3})
    assert applied["align.pid.kp"] == 5.0  # PARAMS caps kp at 5
    assert applied["vision.min_confidence"] == 0.0
    assert cfg.align.pid.kp == 5.0


def test_ints_stay_ints():
    cfg = RobotConfig()
    applied, _ = tuning.apply(cfg, {"fpv.jpeg_quality": 61.7})
    assert applied["fpv.jpeg_quality"] == 62
    assert isinstance(cfg.fpv.jpeg_quality, int)


def test_numeric_strings_are_accepted():
    """JSON from a form field arrives as a string more often than not."""
    cfg = RobotConfig()
    applied, rejected = tuning.apply(cfg, {"drive.slew_rate": "6.5"})
    assert rejected == {}
    assert cfg.drive.slew_rate == 6.5


def test_non_numeric_value_is_reported_not_raised():
    cfg = RobotConfig()
    applied, rejected = tuning.apply(cfg, {"drive.slew_rate": "fast"})
    assert applied == {}
    assert "drive.slew_rate" in rejected
    assert cfg.drive.slew_rate == 4.0  # untouched


def test_bad_frame_shape_does_not_raise():
    """Anything can arrive off a radio; none of it may take the robot down."""
    cfg = RobotConfig()
    applied, rejected = tuning.apply(cfg, ["not", "a", "dict"])
    assert applied == {}
    assert rejected


def test_enum_only_accepts_its_choices():
    cfg = RobotConfig()
    _, rejected = tuning.apply(cfg, {"heading_source": "compass"})
    assert "heading_source" in rejected
    assert cfg.heading_source == "auto"
    applied, _ = tuning.apply(cfg, {"heading_source": "imu"})
    assert cfg.heading_source == "imu"


def test_bools_accept_the_shapes_json_and_forms_produce():
    cfg = RobotConfig()
    tuning.apply(cfg, {"imu.invert": "true"})
    assert cfg.imu.invert is True
    tuning.apply(cfg, {"imu.invert": 0})
    assert cfg.imu.invert is False


def test_needs_restart_flags_only_constructor_owned_fields():
    restart = tuning.needs_restart(["comms.baud", "align.pid.kp", "drive.left.channel"])
    assert restart == ["comms.baud", "drive.left.channel"]


def test_live_parameters_are_reachable_from_push_live_config():
    """Every live=True path is either re-read from cfg or pushed by the robot.

    Guards the claim tuning.py's docstring makes. A parameter marked live that
    neither its owner re-reads nor `_push_live_config` copies across is a
    slider that quietly does nothing — the failure this whole feature is most
    likely to ship with, and the one an operator can't diagnose.

    This is a static net (it reads the source of the push and the PID helper it
    delegates to). test_robot_config.py backs it with the behavioural check
    that the values genuinely reach the running controllers.
    """
    import inspect

    from robot import robot as robot_module

    pushed = (inspect.getsource(robot_module.Robot._push_live_config)
              + inspect.getsource(robot_module._retune))
    # Prefixes whose owner reads its config dataclass on every use, so there is
    # nothing to push: the motors, the shooter servo, the detector, the FPV
    # streamer, and the loop rates the run loop re-reads each tick.
    #
    # `mech.` and `routines.` join them for the same reason, not as an
    # exemption. A Mechanism reads its MechanismConfig on every command exactly
    # as the shooter servo does, and RoutineConfig is held by reference by both
    # the engine (which re-reads state_timeout_default on every tick a state is
    # inheriting it) and the arm action (which re-reads allow_arm on every
    # attempt, so turning the gate off stops a routine already running).
    reads_config_directly = (
        "drive.", "shooter.", "vision.", "fpv.", "loop_hz", "telemetry_hz",
        "mech.", "routines.",
    )
    # Same reason, named exactly rather than by prefix. `nav.pid_trace` is read
    # off cfg by _telemetry on every frame it builds, but the rest of `nav.` is
    # gains that DO have to be pushed onto a live PID — exempting the prefix
    # would stop this test checking the parameters it exists for.
    reads_config_directly_exactly = ("nav.pid_trace",)
    for param in tuning.PARAMS:
        if (not param.live or param.path.startswith(reads_config_directly)
                or param.path in reads_config_directly_exactly):
            continue
        leaf = param.path.rsplit(".", 1)[-1]
        assert leaf in pushed, f"{param.path} is marked live but never pushed"


# --- persistence ------------------------------------------------------------

def test_save_and_load_round_trip(tmp_path):
    path = str(tmp_path / "tuning.json")
    assert tuning.save_overrides({"align.pid.kp": 1.5}, path) is None
    assert tuning.load_overrides(path) == {"align.pid.kp": 1.5}


def test_save_merges_rather_than_replaces():
    """Each edit sends only what changed, so a save must not drop the rest."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = f"{d}/tuning.json"
        tuning.save_overrides({"align.pid.kp": 1.5}, path)
        tuning.save_overrides({"nav.cruise_speed": 0.2}, path)
        assert tuning.load_overrides(path) == {
            "align.pid.kp": 1.5, "nav.cruise_speed": 0.2,
        }


def test_missing_file_is_not_an_error(tmp_path):
    assert tuning.load_overrides(str(tmp_path / "nope.json")) == {}


def test_corrupt_file_is_ignored_not_fatal(tmp_path):
    """A half-written config must leave the robot bootable on its defaults."""
    path = tmp_path / "tuning.json"
    path.write_text("{ this is not json")
    assert tuning.load_overrides(str(path)) == {}


def test_saved_unknown_keys_are_dropped_on_load(tmp_path):
    path = tmp_path / "tuning.json"
    path.write_text(json.dumps({"align.pid.kp": 1.0, "retired.param": 2}))
    assert tuning.load_overrides(str(path)) == {"align.pid.kp": 1.0}


def test_save_failure_is_reported_not_raised(tmp_path):
    """A read-only filesystem must not undo tuning that already took effect."""
    blocked = tmp_path / "afile"
    blocked.write_text("")
    err = tuning.save_overrides({"align.pid.kp": 1.0}, str(blocked / "sub" / "t.json"))
    assert err  # a message, not an exception


def test_overrides_path_honours_the_env_var(monkeypatch):
    monkeypatch.setenv("RS_TUNING_FILE", "/tmp/custom.json")
    assert tuning.overrides_path() == "/tmp/custom.json"
