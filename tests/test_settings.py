"""Base-station settings: the gamepad mapping and the link/UI rates.

The mapping is the interesting half. It decides which physical button is the
E-STOP, so the cases worth pinning down are the ones where an operator has
half-configured it: an unbound action, a stale index, a trigger rest value that
doesn't match the driver.
"""

import pytest

from basestation.controller_input import ControllerReader, Trigger
from basestation.settings import (
    BY_PATH,
    MECH_SLOTS,
    ROUTINE_SLOTS,
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


# --- routines on buttons ----------------------------------------------------

def test_a_bound_routine_slot_names_the_routine_it_runs():
    mapping = ControllerMapping(btn_routine_1=7, routine_1="collect")
    assert (7, "routine:collect") in mapping.actions()


def test_half_a_routine_binding_binds_nothing():
    """A button with no routine, or a routine with no button, does nothing.

    Half-filled is the state the settings page is in for as long as it takes to
    fill the other half, and either guess — running an arbitrary routine, or
    putting a routine on an arbitrary button — moves a machine nobody aimed.
    """
    bare = ControllerMapping().actions()
    assert ControllerMapping(btn_routine_1=7).actions() == bare
    assert ControllerMapping(routine_1="collect").actions() == bare


def test_a_blank_routine_id_does_not_bind():
    """Whitespace is what a cleared text field can leave behind."""
    assert ControllerMapping(btn_routine_2=3, routine_2="   ").actions() == \
        ControllerMapping().actions()


def test_a_routine_id_is_trimmed_before_it_travels():
    mapping = ControllerMapping(btn_routine_2=3, routine_2="  spin  ")
    assert (3, "routine:spin") in mapping.actions()


def test_every_slot_is_bindable_and_settable():
    """Both halves of every slot must be in the whitelist. One missing is a row
    the settings page renders, accepts an edit into, and silently never saves."""
    for n in range(1, ROUTINE_SLOTS + 1):
        assert f"controller.btn_routine_{n}" in BY_PATH
        assert f"controller.routine_{n}" in BY_PATH


def test_a_routine_binding_survives_being_saved_and_reloaded(tmp_path):
    """It is a base-station setting like any other, and an operator who binds a
    routine in the field expects it there after the service restarts."""
    path = str(tmp_path / "basestation.json")
    first = SettingsStore(path=path)
    first.apply({"controller.btn_routine_1": 9, "controller.routine_1": "collect"})
    reloaded = SettingsStore(path=path)
    assert (9, "routine:collect") in reloaded.mapping().actions()


def test_all_four_slots_can_be_bound_at_once():
    mapping = ControllerMapping(
        btn_routine_1=4, routine_1="a", btn_routine_2=5, routine_2="b",
        btn_routine_3=6, routine_3="c", btn_routine_4=7, routine_4="d")
    bound = [(idx, name) for idx, name in mapping.actions()
             if name.startswith("routine:")]
    assert bound == [(4, "routine:a"), (5, "routine:b"),
                     (6, "routine:c"), (7, "routine:d")]


# --- two actions on one button ----------------------------------------------

class FakePad:
    """The two joystick calls `_edge` makes. Not pygame — a reader can be built
    without it via `__new__`, which keeps this test running anywhere."""

    def __init__(self, down):
        self.down = set(down)
        self.n = 8

    def get_numbuttons(self):
        return self.n

    def get_button(self, idx):
        return 1 if idx in self.down else 0


def reader_with(pad):
    """A reader and the list of actions it fires."""
    reader = ControllerReader.__new__(ControllerReader)
    reader._js = pad
    reader._prev = {}
    fired: list = []
    reader.on_action = fired.append
    return reader, fired


def test_both_actions_on_a_shared_button_fire():
    """`_edge` records a press as it tests it, so a second question about the
    same button in one tick answers False. Every action bound to a pressed
    button used to depend on `actions()` order, which is how binding a preset
    onto the cross that clears the e-stop produced a button that only cleared
    the e-stop — with the settings page showing it as bound to the preset."""
    mapping = ControllerMapping(btn_clear=0, btn_mech_1=0, mech_1="intake",
                                preset_1="on")
    reader, fired = reader_with(FakePad([0]))
    reader._fire_actions(mapping)
    assert fired == ["clear", "mech_preset:intake:on"]


def test_a_held_button_fires_once():
    """The edge is still an edge: the failure above must not be fixed by
    dropping press detection and running an action every tick at 40 Hz."""
    mapping = ControllerMapping(btn_clear=0, btn_mech_1=0, mech_1="intake",
                                preset_1="on")
    pad = FakePad([0])
    reader, fired = reader_with(pad)
    reader._fire_actions(mapping)
    reader._fire_actions(mapping)  # still held
    assert fired == ["clear", "mech_preset:intake:on"]
    pad.down.clear()
    reader._fire_actions(mapping)  # released
    pad.down.add(0)
    reader._fire_actions(mapping)  # pressed again
    assert fired.count("mech_preset:intake:on") == 2


def test_an_unpressed_button_fires_nothing():
    mapping = ControllerMapping(btn_estop=1, btn_mech_1=0, mech_1="intake",
                                preset_1="on")
    reader, fired = reader_with(FakePad([]))
    reader._fire_actions(mapping)
    assert fired == []


# --- mechanism presets on buttons -------------------------------------------

def test_a_bound_mech_slot_names_both_the_mechanism_and_the_preset():
    mapping = ControllerMapping(btn_mech_1=7, mech_1="intake", preset_1="in")
    assert (7, "mech_preset:intake:in") in mapping.actions()


def test_a_mech_binding_missing_any_part_binds_nothing():
    """A button with no preset, or a preset with no button, does nothing. Every
    partial state is one somebody is halfway through typing, and each of the
    guesses available — an arbitrary mechanism, an arbitrary button — starts a
    motor nobody aimed."""
    bare = ControllerMapping().actions()
    assert ControllerMapping(btn_mech_1=7).actions() == bare
    assert ControllerMapping(mech_1="intake", preset_1="in").actions() == bare
    assert ControllerMapping(btn_mech_1=7, mech_1="intake").actions() == bare
    assert ControllerMapping(btn_mech_1=7, preset_1="in").actions() == bare


def test_blank_mech_names_do_not_bind():
    """Whitespace is what a cleared text field can leave behind."""
    assert ControllerMapping(btn_mech_2=3, mech_2="  ", preset_2="in").actions() == \
        ControllerMapping().actions()


def test_mech_names_are_trimmed_before_they_travel():
    mapping = ControllerMapping(btn_mech_2=3, mech_2=" intake ", preset_2=" in ")
    assert (3, "mech_preset:intake:in") in mapping.actions()


def test_every_mech_slot_is_bindable_and_settable():
    """All three parts of every slot must be in the whitelist. One missing is a
    row the settings page renders, accepts an edit into, and never saves."""
    for n in range(1, MECH_SLOTS + 1):
        assert f"controller.btn_mech_{n}" in BY_PATH
        assert f"controller.mech_{n}" in BY_PATH
        assert f"controller.preset_{n}" in BY_PATH


def test_a_mech_binding_survives_being_saved_and_reloaded(tmp_path):
    path = str(tmp_path / "basestation.json")
    first = SettingsStore(path=path)
    first.apply({"controller.btn_mech_1": 9, "controller.mech_1": "intake",
                 "controller.preset_1": "in"})
    reloaded = SettingsStore(path=path)
    assert (9, "mech_preset:intake:in") in reloaded.mapping().actions()


def test_all_four_mech_slots_can_be_bound_at_once():
    """An intake in and out, an arm up and down — which is what four is for."""
    mapping = ControllerMapping(
        btn_mech_1=4, mech_1="intake", preset_1="in",
        btn_mech_2=5, mech_2="intake", preset_2="out",
        btn_mech_3=6, mech_3="arm", preset_3="up",
        btn_mech_4=7, mech_4="arm", preset_4="down")
    bound = [pair for pair in mapping.actions()
             if pair[1].startswith("mech_preset:")]
    assert bound == [(4, "mech_preset:intake:in"), (5, "mech_preset:intake:out"),
                     (6, "mech_preset:arm:up"), (7, "mech_preset:arm:down")]


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
