"""The bridge's command surface, over a real WebSocket.

What's being checked is the plumbing that the unit tests can't see: that the
socket still accepts JSON after it has been taught to accept binary audio, that
a spoken order reaches the same `handle_action` a browser button does, and that
the confirmation gate survives the trip through the wire.

The regression that motivated most of this: `receive_json()` raises on a binary
frame and drops the socket — which, mid-drive, is the operator losing the
dashboard the moment somebody holds the microphone button.
"""

import json
import time

import pytest
from fastapi.testclient import TestClient

from basestation.app import build_app
from basestation.command.stt import MAX_SAMPLES, Utterance
from basestation.fleet import FleetManager
from basestation.places import PlaceStore


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def rig():
    fleet = FleetManager()
    now = time.monotonic()
    for rid in ("rover1", "rover2"):
        fleet.update_from_telemetry(
            {"type": "telemetry", "robot_id": rid, "mode": "teleop", "estop": False,
             "lat": 37.0, "lon": -122.0}, now)
    places = PlaceStore(load=False)
    places.replace([{"name": "bucket A", "lat": 37.1, "lon": -122.1, "kind": "bucket"}],
                   save=False)
    link = FakeLink()
    app = build_app(fleet, link=link, controller=None,
                    web_cfg={"tiles": None}, places=places)
    return app, link


def _drain(ws, kind, limit=80, has=None):
    """Read frames until one of `kind` arrives. The socket carries telemetry at
    30 Hz, so a command frame is never the next thing on it.

    `has` narrows it further: command frames also go out with no outcome on them
    (on connect, when the model check finishes, when the model is toggled), and
    a test waiting for a *result* has to skip those rather than read the first
    one and find it empty.
    """
    for _ in range(limit):
        msg = ws.receive_json()
        if msg.get("type") == kind and (has is None or has in msg):
            return msg
    raise AssertionError(f"no {kind!r} frame with {has!r} in {limit} messages")


def _outcome(ws):
    return _drain(ws, "command", has="outcome")["outcome"]


# --- the command frame ----------------------------------------------------

def test_a_command_frame_arrives_on_connect(rig):
    app, _ = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        frame = _drain(ws, "command")
        assert frame["vocabulary"]["robots"] == ["rover1", "rover2"]
        assert frame["vocabulary"]["places"] == [{"id": "bucket_a", "name": "bucket A"}]
        assert "model_ok" in frame and "stt" in frame


# --- a typed order goes where a button would ------------------------------

def test_a_typed_order_reaches_the_radio(rig):
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "command_text", "text": "stop rover2", "source": "text"})
        assert _outcome(ws)["status"] == "done"
        assert {"type": "estop", "to": "rover2"} in link.sent


def test_an_order_produces_the_same_command_a_button_would(rig):
    """The claim the whole design rests on: voice has exactly the dashboard's
    authority, because it goes through the dashboard's own dispatcher."""
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "command_text", "text": "waypoint mode"})
        _outcome(ws)
        by_voice = list(link.sent)

        link.sent.clear()
        ws.send_json({"action": "mode", "robot_id": "rover1", "mode": "waypoint"})
        _drain(ws, "fleet")
        assert by_voice == link.sent


# --- the gate over the wire -----------------------------------------------

def test_a_gated_order_sends_nothing_until_confirmed(rig):
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "command_intent", "intent": "fire",
                      "args": {"robot": "rover2"}})
        outcome = _outcome(ws)
        assert outcome["status"] == "pending"
        confirm_id = outcome["confirm_id"]
        assert not any(m.get("type") == "fire" for m in link.sent)

        ws.send_json({"action": "command_confirm", "id": confirm_id})
        assert _outcome(ws)["status"] == "done"
        assert {"type": "fire", "to": "rover2"} in link.sent


def test_cancelling_a_gated_order_sends_nothing(rig):
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "command_intent", "intent": "arm_shooter", "args": {}})
        confirm_id = _outcome(ws)["confirm_id"]
        ws.send_json({"action": "command_cancel", "id": confirm_id})
        _outcome(ws)
        assert not any(m.get("type") == "arm_shooter" for m in link.sent)


# --- audio frames must not break the socket -------------------------------

def test_binary_audio_does_not_drop_the_socket(rig):
    """The regression. Before receive() replaced receive_json(), the first PCM
    frame raised and took the dashboard down with it."""
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "mic", "on": True})
        for _ in range(5):
            ws.send_bytes(b"\x00\x01" * 512)
        ws.send_json({"action": "mic", "on": False})
        # The socket is still usable for ordinary actions afterwards.
        ws.send_json({"action": "estop", "robot_id": "rover1"})
        _drain(ws, "fleet")
        assert {"type": "estop", "to": "rover1"} in link.sent


def test_audio_arriving_with_no_utterance_open_is_ignored(rig):
    """A frame that arrives just after the button is released is normal, not an
    error, and must not start a phantom transcription."""
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_bytes(b"\x00\x01" * 512)
        ws.send_json({"action": "estop", "robot_id": "rover1"})
        _drain(ws, "fleet")
        assert {"type": "estop", "to": "rover1"} in link.sent


def test_a_malformed_frame_does_not_drop_the_socket(rig):
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_text("not json at all")
        ws.send_text(json.dumps([1, 2, 3]))  # valid JSON, wrong shape
        ws.send_json({"action": "estop", "robot_id": "rover1"})
        _drain(ws, "fleet")
        assert {"type": "estop", "to": "rover1"} in link.sent


# --- turning the model off ------------------------------------------------

def test_the_model_can_be_switched_off_without_losing_stop(rig):
    app, link = rig
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        _drain(ws, "command")
        ws.send_json({"action": "command_enable", "on": False})
        ws.send_json({"action": "command_text", "text": "stop"})
        assert _outcome(ws)["status"] == "done"
        assert {"type": "estop", "to": "rover1"} in link.sent


# --- the utterance buffer -------------------------------------------------

def test_an_utterance_collects_only_while_open():
    u = Utterance()
    u.feed(b"\x01\x02")            # before begin — dropped
    assert u.end() == b""
    u.begin()
    u.feed(b"\x01\x02")
    u.feed(b"\x03\x04")
    assert u.end() == b"\x01\x02\x03\x04"


def test_an_utterance_is_capped():
    """A tablet whose pointerup was lost to a dropped socket would otherwise
    stream until the process runs out of memory."""
    u = Utterance()
    u.begin()
    for _ in range(200):
        u.feed(b"\x00" * 8192)
    assert len(u.end()) <= MAX_SAMPLES * 2 + 8192


def test_ending_an_utterance_twice_is_harmless():
    u = Utterance()
    u.begin()
    u.feed(b"\x01\x02")
    assert u.end() == b"\x01\x02"
    assert u.end() == b""
