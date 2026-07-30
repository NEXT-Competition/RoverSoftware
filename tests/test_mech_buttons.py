"""Mechanism presets on gamepad buttons.

A preset is a named whole-mechanism state ("intake -> in") that a build declares
in its layout, and until now the only things that could ask for one were a
routine and the Hardware tab's jog controls — neither of them a thumb while
somebody is driving. So the mapping carries (button, mechanism, preset) slots,
and a bound one emits `mech_preset:<mech>:<preset>` into the same action
vocabulary every other binding uses.

Both names travel because a preset name alone is ambiguous: two mechanisms may
each have a state called "out". Neither is checked here — a rover's layout lives
on the rover — which is the same bet the routine bindings make, for the same
reason, and the robot's half of it is in tests/test_mechanism.py.
"""

import time

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager
from basestation.places import PlaceStore
from basestation.settings import ControllerMapping, SettingsStore


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def start(self):
        pass

    def stop(self):
        pass


class FakeController:
    """Stands in for ControllerReader: build_app binds its callbacks to us."""

    def __init__(self):
        self.on_drive = None
        self.on_action = None
        self.connected = True
        self.name = "fake pad"
        self.mapping = None

    def set_mapping(self, mapping):
        self.mapping = mapping

    def state(self):
        return {"connected": True, "name": self.name, "axes": [], "buttons": []}

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def rig(tmp_path):
    fleet = FleetManager()
    now = time.monotonic()
    for rid in ("rover1", "rover2"):
        fleet.update_from_telemetry(
            {"type": "telemetry", "robot_id": rid, "mode": "teleop", "estop": False,
             "lat": 37.0, "lon": -122.0}, now)
    fleet.select("rover1")
    link, controller = FakeLink(), FakeController()
    settings = SettingsStore(path=str(tmp_path / "basestation.json"))
    app = build_app(fleet, link=link, controller=controller,
                    web_cfg={"tiles": None}, places=PlaceStore(load=False),
                    settings=settings)
    return app, link, controller, fleet, settings


def preset_frames(link):
    return [m for m in link.sent if m.get("type") == "mech_preset"]


def test_a_bound_button_asks_for_its_preset(rig):
    _, link, controller, _, _ = rig
    controller.on_action("mech_preset:intake:in")
    assert preset_frames(link) == [
        {"type": "mech_preset", "mech": "intake", "preset": "in", "to": "rover1"},
    ]


def test_two_buttons_drive_one_mechanism_both_ways(rig):
    """The shape this is for: a preset latches and there is no release edge, so
    "in" and "out" are two bindings rather than one held button."""
    _, link, controller, _, _ = rig
    controller.on_action("mech_preset:intake:in")
    controller.on_action("mech_preset:intake:out")
    assert [(m["mech"], m["preset"]) for m in preset_frames(link)] == [
        ("intake", "in"), ("intake", "out")]


def test_the_same_preset_name_on_two_mechanisms_stays_distinct(rig):
    """Why the mechanism travels too: "out" alone does not say what moves."""
    _, link, controller, _, _ = rig
    controller.on_action("mech_preset:intake:out")
    controller.on_action("mech_preset:arm:out")
    assert [m["mech"] for m in preset_frames(link)] == ["intake", "arm"]


def test_it_acts_on_whichever_rover_is_selected(rig):
    _, link, controller, fleet, _ = rig
    fleet.select("rover2")
    controller.on_action("mech_preset:intake:in")
    assert all(m["to"] == "rover2" for m in preset_frames(link))


def test_a_preset_the_rover_does_not_have_is_still_sent(rig):
    """The base station holds no copy of a rover's layout and must not guess.
    An unknown mechanism or preset is refused by the robot, out loud; checking
    here would break every binding whenever a rover was switched off."""
    _, link, controller, _, _ = rig
    controller.on_action("mech_preset:ghost:nowhere")
    assert preset_frames(link) == [
        {"type": "mech_preset", "mech": "ghost", "preset": "nowhere", "to": "rover1"},
    ]


def test_a_half_written_action_name_sends_nothing(rig):
    """Belt and braces for the encoding: `actions()` never emits these, but a
    frame with an empty mechanism would be a motor command nobody aimed."""
    _, link, controller, _, _ = rig
    controller.on_action("mech_preset:intake")
    controller.on_action("mech_preset::in")
    controller.on_action("mech_preset:")
    assert preset_frames(link) == []


def test_the_saved_mapping_reaches_the_pad(rig):
    """A binding edited on the settings page must take effect without a
    restart — the reader is handed a fresh mapping on every settings change."""
    app, _, controller, _, _ = rig
    app.state.handle_action({
        "action": "set_settings",
        "settings": {"controller.btn_mech_1": 9, "controller.mech_1": "intake",
                     "controller.preset_1": "in"},
    })
    assert (9, "mech_preset:intake:in") in controller.mapping.actions()


def test_an_unbound_slot_emits_nothing():
    """Nothing is on the pad until a slot is filled in."""
    assert not [name for _, name in ControllerMapping().actions()
                if name.startswith("mech_preset:")]
