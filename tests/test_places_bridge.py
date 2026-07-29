"""Places across the WebSocket: browser -> bridge -> disk -> every browser.

Covers the join the unit tests can't: that a place saved on one tablet is on
the next settings frame, which is how a rover captured by one operator becomes
usable by another. No radio is involved anywhere in this path on purpose — a
place only ever reaches a robot as the lat/lon a saved routine resolved it into.
"""

import json

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager
from basestation.places import PlaceStore

fastapi_testclient = pytest.importorskip("fastapi.testclient")
TestClient = fastapi_testclient.TestClient


class NullLink:
    """A radio that goes nowhere. Places must never reach it."""

    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def bridge(tmp_path):
    link = NullLink()
    store = PlaceStore(path=str(tmp_path / "places.json"))
    app = build_app(FleetManager(), link, None, {}, places=store)
    return app, link, store, tmp_path


def _await_places(ws, tries=50):
    """The next settings frame that carries a non-empty places list."""
    for _ in range(tries):
        msg = ws.receive_json()
        if msg.get("type") == "settings" and msg.get("places"):
            return msg
    return None


def test_a_saved_place_comes_back_on_the_settings_frame(bridge):
    app, link, _store, tmp_path = bridge
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json()["places"] == []

            ws.send_json({"action": "set_places", "places": [
                {"name": "Bucket A", "lat": 37.1, "lon": -122.2, "kind": "bucket"},
            ]})

            frame = _await_places(ws)
            assert frame is not None
            assert frame["places"] == [{
                "id": "bucket_a", "name": "Bucket A",
                "lat": 37.1, "lon": -122.2, "kind": "bucket",
            }]
            assert frame["places_result"]["count"] == 1

    # Persisted, so it is still there after a restart before the next match.
    saved = json.loads((tmp_path / "places.json").read_text(encoding="utf-8"))
    assert saved["places"][0]["name"] == "Bucket A"

    # And nothing went over the radio.
    assert link.sent == []


def test_a_refused_place_is_reported_rather_than_vanishing(bridge):
    app, _link, _store, _tmp = bridge
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
            ws.send_json({"action": "set_places", "places": [
                {"name": "good", "lat": 1.0, "lon": 2.0},
                {"name": "no position"},
            ]})
            frame = _await_places(ws)
            assert frame is not None
            assert len(frame["places"]) == 1
            # The operator has to learn that the second one didn't take.
            assert frame["places_result"]["rejected"]


def test_places_load_from_disk_on_startup(tmp_path):
    path = tmp_path / "places.json"
    path.write_text(json.dumps({"places": [
        {"id": "bucket_a", "name": "Bucket A", "lat": 1.0, "lon": 2.0, "kind": "bucket"},
    ]}), encoding="utf-8")

    app = build_app(FleetManager(), NullLink(), None, {},
                    places=PlaceStore(path=str(path)))
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json()["places"][0]["name"] == "Bucket A"


def test_the_bridge_still_works_without_a_place_store():
    # An embedder that passes nothing must still get a working app — the same
    # fallback settings already has, and what keeps these tests off a
    # developer's home directory.
    app = build_app(FleetManager(), NullLink(), None, {})
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json()["places"] == []
