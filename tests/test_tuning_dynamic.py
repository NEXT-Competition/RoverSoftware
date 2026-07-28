"""The tunable surface when the robot's layout is not the stock one.

test_tuning.py covers the static whitelist. This covers the part that makes
user-declared actuators possible: a parameter set derived from the layout rather
than written down in advance, and the compatibility identity that keeps every
deployed rover working while that happens.
"""

import pytest

from robot import layout, tuning
from robot.config import RobotConfig


CUSTOM = {
    "version": 1,
    "drive": {
        "kind": "servo_steer",
        "actuators": [
            {"name": "rear", "kind": "esc", "channel": 0},
            {"name": "steering", "kind": "servo", "channel": 3},
        ],
        "roles": {"throttle": ["rear"], "steer": "steering"},
    },
    "mechanisms": [{
        "name": "intake",
        "kind": "power",
        "actuators": [{"name": "roller", "channel": 4}],
        "presets": {"in": {"roller": 1.0}},
    }],
}


def custom_cfg():
    cfg = RobotConfig()
    result = layout.apply(cfg, CUSTOM)
    assert result.ok, result.errors
    return cfg


# --- the compatibility contract ---------------------------------------------

def test_the_default_layout_derives_exactly_the_static_params():
    """THE compatibility contract, and the reason nothing else had to change.

    `PARAMS` is what a build ships with and what the dashboard's schema.ts
    mirrors by hand; `params_for` is what this robot actually exposes. They are
    equal on a stock robot because the default layout names its two actuators
    `left` and `right`, so the derived paths ARE `drive.left.*`/`drive.right.*`.

    Every deployed tuning.json, the hand-written TS mirror, and the two
    snapshot-key assertions in test_tuning.py and test_robot_config.py rest on
    this one identity. Break it and they all break together.
    """
    derived = {p.path for p in tuning.params_for(RobotConfig())}
    static = {p.path for p in tuning.PARAMS}
    assert derived == static


def test_every_derived_path_resolves_on_a_custom_layout():
    """A typo in the generated paths must fail here, not in a field."""
    cfg = custom_cfg()
    for param in tuning.params_for(cfg):
        tuning.get(cfg, param.path)


# --- what a layout adds and removes -----------------------------------------

def test_a_layout_adds_its_own_actuators_and_mechanisms():
    snap = tuning.snapshot(custom_cfg())
    assert "drive.rear.deadband" in snap
    assert "drive.steering.max_angle" in snap
    assert "mech.intake.roller.max_forward" in snap
    assert "mech.intake.auto_stop_seconds" in snap


def test_a_layout_removes_the_actuators_it_no_longer_has():
    """The counterpart nobody thinks about: a stale path must stop being real,
    or a saved value silently writes to an actuator that isn't there."""
    snap = tuning.snapshot(custom_cfg())
    assert "drive.left.deadband" not in snap
    assert "drive.right.deadband" not in snap


def test_writes_reach_a_custom_layouts_actuators():
    cfg = custom_cfg()
    applied, rejected = tuning.apply(cfg, {"mech.intake.roller.max_forward": 0.4})
    assert rejected == {}
    assert applied == {"mech.intake.roller.max_forward": 0.4}
    assert cfg.mechanisms["intake"].actuators["roller"].max_forward == 0.4


def test_paths_from_another_layout_are_rejected_not_written():
    cfg = custom_cfg()
    applied, rejected = tuning.apply(cfg, {"drive.left.deadband": 0.2})
    assert applied == {}
    assert "drive.left.deadband" in rejected


# --- the __getattr__ shim ----------------------------------------------------

def test_drive_actuators_are_reachable_by_name_through_the_config():
    """DriveConfig.__getattr__ is load-bearing: tuning._resolve walks a dotted
    path with plain getattr/setattr, so this is the only reason
    `drive.left.deadband` still resolves now that `left` is a dict key."""
    cfg = RobotConfig()
    assert cfg.drive.left is cfg.drive.actuators["left"]
    assert cfg.drive.right is cfg.drive.actuators["right"]


def test_writes_through_the_shim_land_on_the_stored_actuator():
    """Reading through the shim would be useless if writes went elsewhere."""
    cfg = RobotConfig()
    tuning.apply(cfg, {"drive.left.deadband": 0.19})
    assert cfg.drive.actuators["left"].deadband == 0.19


def test_a_real_field_always_wins_over_an_actuator_of_the_same_name():
    cfg = RobotConfig()
    assert cfg.drive.slew_rate == 4.0  # the field, not an actuator lookup


def test_an_unknown_name_raises_attributeerror_not_recursion():
    cfg = RobotConfig()
    with pytest.raises(AttributeError):
        cfg.drive.definitely_not_an_actuator


# --- signatures that keep the old callers working ---------------------------

def test_needs_restart_defaults_to_the_static_surface():
    assert tuning.needs_restart(["comms.baud"]) == ["comms.baud"]


def test_needs_restart_covers_a_layouts_own_channels():
    cfg = custom_cfg()
    restart = tuning.needs_restart(
        ["drive.rear.channel", "mech.intake.roller.deadband"],
        tuning.by_path_for(cfg))
    assert restart == ["drive.rear.channel"]


def test_load_overrides_filters_against_the_surface_it_is_given(tmp_path):
    path = tmp_path / "tuning.json"
    tuning.save_overrides({"drive.left.deadband": 0.2, "align.pid.kp": 1.0}, str(path))
    keep = tuning.load_overrides(str(path), known=tuning.by_path_for(custom_cfg()))
    assert keep == {"align.pid.kp": 1.0}


# --- descriptors -------------------------------------------------------------

def test_descriptors_cover_only_what_the_ui_cannot_know_in_advance():
    """The dashboard already has the ~90 static fields in schema.ts. Restating
    them would put 2 KB on a shared 57600-baud radio for nothing."""
    described = {d["path"] for d in tuning.descriptors(custom_cfg())}
    assert "drive.rear.deadband" in described
    assert "mech.intake.roller.channel" in described
    assert "align.pid.kp" not in described
    assert "loop_hz" not in described


def test_descriptors_carry_what_a_form_needs():
    by_path = {d["path"]: d for d in tuning.descriptors(custom_cfg())}
    channel = by_path["drive.rear.channel"]
    assert channel["kind"] == "int"
    assert (channel["lo"], channel["hi"]) == (0, 15)
    assert channel["live"] is False  # a PWM channel is owned by a constructor
    assert channel["group"] == "actuator:rear"
    assert channel["label"]
