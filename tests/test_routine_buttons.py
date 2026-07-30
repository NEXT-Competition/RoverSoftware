"""Routines on gamepad buttons.

A routine is the one thing an operator can put on the pad that they wrote
themselves, which makes it the one binding the base station cannot hold a fixed
field for: there is no list of routines in this process, and the ids belong to
a rover that may be switched off while somebody is editing bindings. So the
mapping carries a few (button, routine id) slots, and the button emits
`routine:<id>` into the same action vocabulary every other binding uses.

What is worth pinning down is the pair of messages that come out. Running a
routine is choose-then-switch, and both halves have to reach the rover in that
order or it runs a different routine than the one on the button — see
tests/test_routine_controller.py for the robot's half of that.
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


def routine_frames(link, rid=None):
    return [m for m in link.sent
            if m.get("type") in ("select_routine", "mode")
            and (rid is None or m.get("to") == rid)]


def test_a_bound_button_runs_its_routine(rig):
    _, link, controller, _, _ = rig
    controller.on_action("routine:collect")
    assert routine_frames(link) == [
        {"type": "select_routine", "id": "collect", "to": "rover1"},
        {"type": "mode", "mode": "routine", "to": "rover1"},
    ]


def test_the_choice_goes_out_before_the_mode_switch(rig):
    """The order is the whole feature. A rover that is not already in routine
    mode starts on the mode change, off whatever is selected by then — so a
    select arriving second starts the previous routine and then switches, which
    is a burst of the wrong machine moving."""
    _, link, controller, _, _ = rig
    controller.on_action("routine:collect")
    kinds = [m["type"] for m in routine_frames(link)]
    assert kinds.index("select_routine") < kinds.index("mode")


def test_two_buttons_run_two_different_routines(rig):
    _, link, controller, _, _ = rig
    controller.on_action("routine:collect")
    controller.on_action("routine:go_home")
    ids = [m["id"] for m in link.sent if m.get("type") == "select_routine"]
    assert ids == ["collect", "go_home"]


def test_it_runs_on_whichever_rover_is_selected(rig):
    """Every other binding acts on the selection; this one is no different."""
    _, link, controller, fleet, _ = rig
    fleet.select("rover2")
    controller.on_action("routine:collect")
    assert all(m["to"] == "rover2" for m in routine_frames(link))


def test_an_id_the_rover_does_not_have_is_still_sent(rig):
    """The base station does not know what a rover carries and must not guess.
    An id that no longer exists is refused by the robot, out loud; validating
    here would instead break every binding whenever a rover was switched off."""
    _, link, controller, _, _ = rig
    controller.on_action("routine:deleted")
    assert any(m.get("id") == "deleted" for m in link.sent)


def test_the_saved_mapping_reaches_the_pad(rig):
    """A binding edited on the settings page must take effect without a
    restart — the reader is handed a fresh mapping on every settings change."""
    app, _, controller, _, settings = rig
    app.state.handle_action({
        "action": "set_settings",
        "settings": {"controller.btn_routine_1": 9, "controller.routine_1": "collect"},
    })
    assert (9, "routine:collect") in controller.mapping.actions()


def test_an_unbound_slot_emits_nothing():
    """Nothing is on the pad until both halves of a slot are filled."""
    assert not [name for _, name in ControllerMapping().actions()
                if name.startswith("routine:")]
