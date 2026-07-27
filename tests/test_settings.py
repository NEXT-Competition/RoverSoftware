"""Base-station settings: the gamepad mapping and the link/UI rates.

The mapping is the interesting half. It decides which physical button is the
E-STOP, so the cases worth pinning down are the ones where an operator has
half-configured it: an unbound action, a stale index, a trigger rest value that
doesn't match the driver.
"""

import pytest

from basestation.controller_input import Trigger
from basestation.settings import (
    UNBOUND,
    ControllerMapping,
    SettingsStore,
    settings_path,
)


@pytest.fixture
def store(tmp_path):
    return SettingsStore(path=str(tmp_path / "basestation.json"))


# --- the store --------------------------------------------------------------

def test_snapshot_is_flat_dotted_paths(store):
    snap = store.snapshot()
    assert snap["base.drive_hz"] == 15.0
    assert snap["controller.axis_steer"] == 2
    assert all("." in key for key in snap)


def test_defaults_seed_from_the_cli(tmp_path):
    store = SettingsStore(defaults={"base.drive_hz": 8}, path=str(tmp_path / "s.json"),
                          load=False)
    assert store.base.drive_hz == 8.0


def test_saved_values_win_over_the_cli_baseline(tmp_path):
    """A flag sets the baseline; what the operator changed since wins."""
    path = str(tmp_path / "s.json")
    SettingsStore(defaults={"base.drive_hz": 30}, path=path).apply(
        {"base.drive_hz": 12})
    reloaded = SettingsStore(defaults={"base.drive_hz": 30}, path=path)
    assert reloaded.base.drive_hz == 12.0


def test_apply_clamps_and_reports(store):
    result = store.apply({"base.drive_hz": 5000, "junk": 1})
    assert result["applied"] == {"base.drive_hz": 60.0}
    assert result["rejected"] == {"junk": "unknown setting"}
    assert result["save_error"] is None


def test_apply_flags_restart_only_settings(store):
    assert store.apply({"base.controller_hz": 60})["restart"] == ["base.controller_hz"]
    assert store.apply({"base.drive_hz": 20})["restart"] == []


def test_a_rejected_field_does_not_block_its_neighbours(store):
    result = store.apply({"base.drive_hz": 20, "base.ui_hz": "fast"})
    assert result["applied"] == {"base.drive_hz": 20.0}
    assert "base.ui_hz" in result["rejected"]
    assert store.base.ui_hz == 30.0  # untouched


def test_changes_are_persisted_and_reloaded(tmp_path):
    path = str(tmp_path / "s.json")
    SettingsStore(path=path).apply({"controller.axis_steer": 4})
    assert SettingsStore(path=path).controller.axis_steer == 4


def test_a_corrupt_file_leaves_the_station_launchable(tmp_path):
    path = tmp_path / "s.json"
    path.write_text("not json at all")
    assert SettingsStore(path=str(path)).base.drive_hz == 15.0


def test_on_change_fires_with_what_changed(store):
    seen = []
    store.on_change = seen.append
    store.apply({"controller.deadzone": 0.2})
    assert seen == [{"controller.deadzone": 0.2}]


def test_on_change_does_not_fire_for_a_fully_rejected_edit(store):
    seen = []
    store.on_change = seen.append
    store.apply({"junk": 1})
    assert seen == []


def test_mapping_is_a_copy_the_gamepad_thread_can_hold(store):
    mapping = store.mapping()
    store.apply({"controller.axis_steer": 7})
    assert mapping.axis_steer == 2  # the copy is unaffected mid-poll
    assert store.mapping().axis_steer == 7


def test_settings_path_honours_the_env_var(monkeypatch):
    monkeypatch.setenv("RS_BASE_SETTINGS", "/tmp/bs.json")
    assert settings_path() == "/tmp/bs.json"


# --- the mapping ------------------------------------------------------------

def test_default_actions_match_the_documented_dualshock_layout():
    assert ControllerMapping().actions() == (
        (1, "estop"), (0, "clear"), (4, "mode:teleop"), (5, "mode:object_align"),
    )


def test_unbound_buttons_produce_no_action():
    """An unbound binding must be absent, not bound to button -1."""
    mapping = ControllerMapping(btn_estop=UNBOUND)
    assert all(name != "estop" for _, name in mapping.actions())
    assert all(idx >= 0 for idx, _ in mapping.actions())


def test_extra_bindings_reach_the_pass_through_actions():
    mapping = ControllerMapping(btn_fire=7, btn_arm_shooter=6, btn_waypoint=3)
    actions = dict((name, idx) for idx, name in mapping.actions())
    assert actions["fire"] == 7
    assert actions["arm_shooter"] == 6
    assert actions["mode:waypoint"] == 3


def test_two_actions_may_share_a_button():
    """Odd, but not something to refuse — a small pad may have to double up."""
    mapping = ControllerMapping(btn_estop=1, btn_clear=1)
    assert len(mapping.actions()) == 4


# --- trigger rest, per mapping ---------------------------------------------

def test_trigger_honours_a_driver_that_rests_at_zero():
    """The 0..1 driver case. With the SDL default of -1 this trigger would
    read half throttle at rest, which is the bug the arming latch exists for."""
    t = Trigger(rest=0.0)
    assert t.value(0.0) == 0.0
    assert t.value(1.0) == 1.0
    assert t.value(0.5) == 0.5


def test_changing_the_rest_value_disarms():
    """A re-tuned rest must be proven again — otherwise a mis-set value could
    leave a stale half-throttle latched in from the previous calibration."""
    t = Trigger(rest=-1.0)
    t.value(-1.0)
    assert t.armed
    t.set_rest(0.0)
    assert not t.armed
    # Dead until it reports the new rest. 0.8 is outside the arming window
    # (rest + 0.5), so this sample proves nothing and commands nothing.
    assert t.value(0.8) == 0.0
    assert t.value(0.0) == 0.0  # at the new rest: arms, still commands nothing
    assert t.armed
    assert t.value(0.8) == 0.8


def test_setting_the_same_rest_does_not_disarm():
    t = Trigger(rest=-1.0)
    t.value(-1.0)
    t.set_rest(-1.0)
    assert t.armed


def test_a_degenerate_rest_reads_as_zero_not_a_crash():
    """rest = 1.0 leaves no span. Fail stopped, don't divide by zero."""
    t = Trigger(rest=1.0)
    t.value(1.0)
    assert t.value(1.0) == 0.0
