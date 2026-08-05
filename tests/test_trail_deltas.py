"""Breadcrumb trails as deltas, not as a thing restated thirty times a second.

A trail is up to trail_max points and gains one per telemetry frame. Sending it
whole on every fleet snapshot made it ~94% of the frame and scaled with the
fleet — three rovers came to 37 KB at ui_hz, per open browser, on the same WiFi
carrying the FPV video and the config link. So the bridge sends each trail once
and thereafter only what it has just added.

The whole risk of a delta scheme is a client that quietly falls behind and draws
a trail with a hole in it. These tests are mostly about that: that the counter
which makes a gap DETECTABLE is right, and that asking for a resend fixes it.
"""

import json

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager
from basestation.settings import SettingsStore

fastapi_testclient = pytest.importorskip("fastapi.testclient")
TestClient = fastapi_testclient.TestClient


class NullLink:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def start(self):
        pass

    def stop(self):
        pass


def _moved(rid, lat, lon):
    return {"type": "telemetry", "from": rid, "mode": "teleop",
            "lat": lat, "lon": lon}


def _drive(fleet, rid, n, start=0):
    for k in range(start, start + n):
        fleet.update_from_telemetry(_moved(rid, 37.0 + k * 1e-5, -122.0), k)


# --- the counter -------------------------------------------------------------

def test_the_hot_frame_carries_the_new_points_not_the_trail():
    fleet = FleetManager()
    _drive(fleet, "rover1", 50)
    cursors = {}

    first = fleet.snapshot(1.0, cursors)["robots"][0]
    assert len(first["trail_add"]) == 50  # a cursor at zero is owed all of them
    assert first["trail_seq"] == 50
    cursors["rover1"] = first["trail_seq"]

    _drive(fleet, "rover1", 2, start=50)
    second = fleet.snapshot(2.0, cursors)["robots"][0]
    assert len(second["trail_add"]) == 2, "only what moved since the last frame"
    assert second["trail_seq"] == 52


def test_a_frame_with_no_movement_carries_no_points():
    """The common case by a wide margin: telemetry is 5 Hz and the UI is 30."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 10)
    cursors = {"rover1": 10}
    robot = fleet.snapshot(1.0, cursors)["robots"][0]
    assert robot["trail_add"] == []
    assert robot["trail_seq"] == 10


def test_the_sequence_counts_points_dropped_off_the_front():
    """trail_seq is 'ever appended', not 'currently held'. If it reset when the
    cap trimmed the oldest point, a client would see the count go backwards and
    conclude it was ahead of the bridge."""
    fleet = FleetManager(trail_max=10)
    _drive(fleet, "rover1", 25)
    robot = fleet.snapshot(1.0, {})["robots"][0]
    assert robot["trail_seq"] == 25
    assert len(fleet.trails()["rover1"]["trail"]) == 10


def test_the_full_trail_is_never_on_the_hot_frame():
    """The regression this whole change exists to prevent."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 400)
    cursors = {"rover1": 400}
    snap = fleet.snapshot(1.0, cursors)
    assert "trail" not in snap["robots"][0]
    # Small enough to state as an absolute: a caught-up frame for one rover is
    # a few hundred bytes, not the ~12 KB the trail alone used to cost.
    assert len(json.dumps(snap)) < 1000


def test_callers_that_want_no_trail_data_get_none():
    """The command layer reads snapshots to answer 'which rover is nearest',
    and has no use for a breadcrumb."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 30)
    robot = fleet.snapshot(1.0)["robots"][0]
    assert "trail_add" not in robot and "trail_seq" not in robot


def test_a_client_further_behind_than_the_cap_is_sent_what_is_left():
    """It cannot be caught up by appending, and the arithmetic says so: the
    points offered are fewer than the count claims, which is the client's cue
    to ask for the whole thing."""
    fleet = FleetManager(trail_max=10)
    _drive(fleet, "rover1", 100)
    robot = fleet.snapshot(1.0, {"rover1": 5})["robots"][0]
    assert robot["trail_seq"] == 100
    assert len(robot["trail_add"]) == 10  # everything still held
    assert 100 - 5 > len(robot["trail_add"]), "the gap is visible to the client"


# --- across the socket -------------------------------------------------------

def _first(ws, mtype, tries=60):
    for _ in range(tries):
        msg = ws.receive_json()
        if msg.get("type") == mtype:
            return msg
    raise AssertionError(f"no {mtype} frame arrived")


def test_a_browser_is_given_the_trails_when_it_connects():
    """Without this a page opened mid-match draws a trail starting from the
    moment it connected, which looks like a rover that has not moved."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 40)
    app = build_app(fleet, NullLink(), None, {})
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            trails = _first(ws, "trails")
            assert len(trails["trails"]["rover1"]["trail"]) == 40
            assert trails["trails"]["rover1"]["seq"] == 40


def test_a_client_can_ask_for_the_trails_again():
    """The repair path for a client that has missed points."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 12)
    app = build_app(fleet, NullLink(), None, {})
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            _first(ws, "trails")
            _drive(fleet, "rover1", 3, start=12)
            ws.send_json({"action": "get_trails"})
            again = _first(ws, "trails")
            assert again["trails"]["rover1"]["seq"] == 15
            assert len(again["trails"]["rover1"]["trail"]) == 15


def test_two_browsers_get_byte_identical_frames():
    """Which is what lets the bridge encode each frame once instead of once per
    socket. json.dumps on the event loop, per client, is work that lands ahead
    of the drive actions those same clients are sending up it."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 20)
    app = build_app(fleet, NullLink(), None, {})
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as one:
            with client.websocket_connect("/ws") as two:
                a = _first(one, "fleet")
                b = _first(two, "fleet")
    # Same content, and in particular the same trail deltas: a per-client cursor
    # would make these differ and force a per-client encode.
    assert a["robots"][0]["trail_seq"] == b["robots"][0]["trail_seq"]


def test_the_bridge_tells_the_client_what_to_trim_to():
    """Both ends must drop the same oldest points, or the browser grows a trail
    the bridge has already forgotten. The cap is a dashboard setting, so it is
    shipped on the frame rather than hardcoded at both ends."""
    fleet = FleetManager()
    _drive(fleet, "rover1", 5)
    settings = SettingsStore(defaults={"base.trail_max": 25}, load=False)
    app = build_app(fleet, NullLink(), None, {}, settings=settings)
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            assert _first(ws, "fleet")["trail_max"] == 25
