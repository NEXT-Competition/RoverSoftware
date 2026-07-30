"""Which link the base station puts configuration on.

The rule, in one line: driving, telemetry and the e-stop go over the radio;
configuration, layouts and routines go over WiFi or not at all. The exception —
and the reason "or not at all" doesn't paint a rover into a corner — is a
set_config carrying nothing but the address of the WiFi link itself.

These tests drive `handle_action` through the real WebSocket, the same way
tests/test_command_bridge.py does, because that is the path a dashboard button
takes and the point is that no button gets to bypass the rule.
"""

import time

import pytest
from fastapi.testclient import TestClient

from basestation.app import build_app
from basestation.fleet import FleetManager


class FakeRadio:
    """An XBee-shaped link: `send_bulk` is what tells the app there IS a radio
    to protect. Without it the app assumes the simulator (in-process robots, no
    wire) and none of the rules below apply."""

    def __init__(self):
        self.sent = []
        self.bulk = []

    def send(self, msg):
        self.sent.append(msg)

    def send_bulk(self, msg):
        self.bulk.append(msg)
        return True

    def start(self):
        pass

    def stop(self):
        pass


class FakeIPServer:
    """Stands in for IPServer: a set of robots currently on WiFi."""

    def __init__(self, connected=()):
        self.connected = set(connected)
        self.sent = []

    def is_connected(self, robot_id):
        return robot_id in self.connected

    def send(self, msg):
        if msg.get("to") not in self.connected:
            return False
        self.sent.append(msg)
        return True

    def start(self):
        pass

    def stop(self):
        pass


def rig(on_wifi=()):
    fleet = FleetManager()
    now = time.monotonic()
    for rid in ("rover1", "rover2"):
        fleet.update_from_telemetry(
            {"type": "telemetry", "robot_id": rid, "mode": "teleop"}, now)
    radio, ip_server = FakeRadio(), FakeIPServer(on_wifi)
    app = build_app(fleet, link=radio, controller=None, web_cfg={"tiles": None},
                    ip_server=ip_server)
    return app, fleet, radio, ip_server


def _act(app, *actions):
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        for action in actions:
            ws.send_json(action)
        # One frame back is enough to know the send was processed: the socket is
        # ordered, so anything the broadcaster emits after our action was queued
        # proves handle_action has run.
        for _ in range(5):
            ws.receive_json()


LAYOUT = {"version": 1, "actuators": [{"name": "left", "channel": 0},
                                      {"name": "right", "channel": 1}]}


# --- config goes over WiFi ---------------------------------------------------

def test_get_config_goes_over_wifi():
    app, _, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {"action": "get_config", "robot_id": "rover1"})
    assert ip_server.sent == [{"type": "get_config", "to": "rover1"}]
    assert radio.sent == [] and radio.bulk == []


def test_set_config_goes_over_wifi():
    app, _, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {"action": "set_config", "robot_id": "rover1",
               "config": {"align.pid.kp": 0.9}})
    assert ip_server.sent[0]["config"] == {"align.pid.kp": 0.9}
    assert radio.sent == []


def test_a_robot_off_wifi_is_not_configured_over_the_radio():
    """The whole rule. rover2 is driving fine — it just isn't on WiFi, and a
    2.9 KB snapshot is not worth the shared channel."""
    app, fleet, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {"action": "get_config", "robot_id": "rover2"})
    assert ip_server.sent == []
    assert radio.sent == [] and radio.bulk == []


def test_the_operator_is_told_why_nothing_happened():
    """A settings page that just stays blank is the failure this replaces."""
    app, fleet, _, _ = rig(on_wifi=())
    _act(app, {"action": "get_config", "robot_id": "rover2"})
    result = fleet.configs()["rover2"]["result"]
    assert "WiFi" in result["error"]


def test_a_real_answer_clears_the_warning():
    app, fleet, _, _ = rig(on_wifi=())
    _act(app, {"action": "get_config", "robot_id": "rover2"})
    fleet.update_from_config({"type": "config", "from": "rover2",
                              "config": {"align.pid.kp": 0.9}})
    assert fleet.configs()["rover2"]["result"]["error"] is None


# --- the bootstrap exception -------------------------------------------------

def test_the_link_address_still_goes_over_the_radio():
    """How a rover that has never been on WiFi is told where the base station
    is. ~60 bytes, by hand, once — see robot/tuning.py::BOOTSTRAP_PATHS."""
    app, _, radio, ip_server = rig(on_wifi=())
    _act(app, {"action": "set_config", "robot_id": "rover2",
               "config": {"comms.base_host": "base.local", "comms.base_port": 5006}})
    assert ip_server.sent == []
    assert radio.sent[0]["config"]["comms.base_host"] == "base.local"


def test_a_bootstrap_prefers_wifi_when_there_is_wifi():
    """The exception is a fallback, not a separate channel: a rover already on
    WiFi is re-pointed over WiFi like any other edit."""
    app, _, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {"action": "set_config", "robot_id": "rover1",
               "config": {"comms.base_host": "other.local"}})
    assert ip_server.sent and radio.sent == []


def test_a_gain_stapled_to_a_hostname_is_not_a_bootstrap():
    """Otherwise the exception swallows the rule: every config edit would ride
    the radio with `comms.base_host` attached."""
    app, _, radio, ip_server = rig(on_wifi=())
    _act(app, {"action": "set_config", "robot_id": "rover2",
               "config": {"comms.base_host": "base.local", "align.pid.kp": 0.9}})
    assert radio.sent == [] and ip_server.sent == []


# --- documents ---------------------------------------------------------------

def test_a_layout_goes_over_wifi():
    app, _, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {"action": "set_layout", "robot_id": "rover1", "doc": LAYOUT})
    assert ip_server.sent and all(f["type"] == "put_layout" for f in ip_server.sent)
    assert radio.bulk == []


def test_a_layout_for_a_robot_off_wifi_is_refused_not_queued():
    """Refused up front, because a document sitting in a queue for a rover that
    never comes back on WiFi is the version that looks saved and isn't."""
    app, fleet, radio, ip_server = rig(on_wifi=())
    _act(app, {"action": "set_layout", "robot_id": "rover2", "doc": LAYOUT})
    assert radio.bulk == [] and ip_server.sent == []
    result = fleet.documents()["rover2"]["layout_result"]
    assert result["ok"] is False and "WiFi" in result["errors"][0]


def test_routines_report_against_their_own_document():
    """Two documents, two results — a refused routine save must not show up in
    the layout editor."""
    app, fleet, _, _ = rig(on_wifi=())
    _act(app, {"action": "set_routines", "robot_id": "rover2",
               "doc": {"version": 1, "routines": []}})
    docs = fleet.documents()["rover2"]
    assert docs["routines_result"]["ok"] is False
    assert docs["layout_result"] is None


# --- what must NOT move ------------------------------------------------------

@pytest.mark.parametrize("action", [
    {"action": "estop"},
    {"action": "clear_estop"},
    {"action": "mode", "mode": "waypoint"},
    {"action": "drive", "throttle": 0.5, "steer": 0.0},
])
def test_driving_and_the_estop_stay_on_the_radio(action):
    """The radio is what has the range. Nothing in this change may quietly make
    an e-stop depend on WiFi being up."""
    app, _, radio, ip_server = rig(on_wifi=("rover1",))
    _act(app, {**action, "robot_id": "rover1"})
    assert ip_server.sent == []
    assert radio.sent and radio.sent[0]["to"] == "rover1"


def test_the_simulator_needs_none_of_this():
    """In-process robots have no radio to protect and no socket to dial in on,
    so `just sim` must keep working with no ip_server at all."""
    fleet = FleetManager()

    class InProcess:
        def __init__(self):
            self.sent = []

        def send(self, msg):
            self.sent.append(msg)

        def start(self):
            pass

        def stop(self):
            pass

    link = InProcess()
    app = build_app(fleet, link=link, controller=None, web_cfg={"tiles": None})
    _act(app, {"action": "get_config", "robot_id": "sim1"},
         {"action": "set_layout", "robot_id": "sim1", "doc": LAYOUT})
    assert {"type": "get_config", "to": "sim1"} in link.sent
    assert any(m["type"] == "put_layout" for m in link.sent)
