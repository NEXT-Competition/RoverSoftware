"""Putting the Pi on a network, from the base station, over the radio.

Two claims are worth testing hard, and only the second is about WiFi:

  * A credential never lands anywhere it could be read later — not in a log, not
    in a config snapshot, not in the frame that comes back.
  * The request reaches a rover that is on NO network, because that is the only
    rover that needs it. If this path ever quietly required WiFi, the feature
    would work perfectly on the bench and be useless at a venue.
"""

from __future__ import annotations

import json
import subprocess

import pytest

from basestation.fleet import FleetManager
from robot.comms import wifi


# --- parsing what nmcli says ------------------------------------------------

def test_a_colon_in_a_network_name_survives():
    r"""nmcli's terse output escapes a literal colon as `\:`. Splitting on a bare
    colon mangles exactly the network somebody is trying to join."""
    assert wifi._terse(r"Guest\: Floor 2:82:WPA2") == ["Guest: Floor 2", "82", "WPA2"]


def test_an_ordinary_line_splits_normally():
    assert wifi._terse("PitCrew:64:WPA2") == ["PitCrew", "64", "WPA2"]


# --- no NetworkManager is an answer, not a crash ----------------------------

def test_a_pi_without_nmcli_says_so(monkeypatch):
    """A Pi on the older wpa_supplicant stack cannot be configured from here. It
    has to SAY that: an operator told which stack the image uses goes and edits
    the right file, one whose button silently does nothing learns nothing."""
    monkeypatch.setattr(wifi.shutil, "which", lambda _: None)
    status = wifi.status()
    assert status["ok"] is False
    assert status["managed"] is False
    assert "nmcli" in status["error"]
    assert wifi.connect("Venue-Guest", "hunter2")["ok"] is False
    assert wifi.scan()["networks"] == []


def test_a_missing_binary_mid_call_is_not_an_exception(monkeypatch):
    """Nothing here may raise: it is called from a robot holding a drivetrain at
    neutral."""
    monkeypatch.setattr(wifi.shutil, "which", lambda _: "/usr/bin/nmcli")

    def explode(*a, **k):
        raise FileNotFoundError("gone")

    monkeypatch.setattr(wifi.subprocess, "run", explode)
    assert wifi.status()["ok"] is False


def test_a_hung_nmcli_times_out_rather_than_wedging(monkeypatch):
    monkeypatch.setattr(wifi.shutil, "which", lambda _: "/usr/bin/nmcli")

    def hang(*a, **k):
        raise subprocess.TimeoutExpired(cmd="nmcli", timeout=1)

    monkeypatch.setattr(wifi.subprocess, "run", hang)
    assert wifi.status()["ok"] is False


# --- the credential ---------------------------------------------------------

def test_the_password_is_scrubbed_from_anything_nmcli_echoes(monkeypatch):
    """Belt and braces. nmcli does not echo the password in its own errors, but
    "the password ended up in the journal" is discovered much later by somebody
    else, so it is not left to nmcli's discretion."""
    monkeypatch.setattr(wifi.shutil, "which", lambda _: "/usr/bin/nmcli")

    def leaky(args, **kw):
        return subprocess.CompletedProcess(args, 1, "tried hunter2", "with hunter2")

    monkeypatch.setattr(wifi.subprocess, "run", leaky)
    done = wifi._run(["nmcli"], 1.0, secret="hunter2")
    assert "hunter2" not in done.stdout
    assert "hunter2" not in done.stderr
    assert "***" in done.stderr


def test_connect_logs_the_network_but_never_the_password(monkeypatch, capsys):
    monkeypatch.setattr(wifi.shutil, "which", lambda _: "/usr/bin/nmcli")
    monkeypatch.setattr(wifi, "_device", lambda: "wlan0")
    monkeypatch.setattr(wifi, "status", lambda: {"ssid": "PitCrew", "ip": "10.0.0.9",
                                                "signal": 70})

    seen = {}

    def ok(args, **kw):
        seen["args"] = args
        return subprocess.CompletedProcess(args, 0, "", "")

    monkeypatch.setattr(wifi.subprocess, "run", ok)
    result = wifi.connect("Venue-Guest", "hunter2")
    assert result["ok"] is True
    # It IS handed to nmcli — that is the whole job — but never printed.
    assert "hunter2" in seen["args"]
    assert "hunter2" not in capsys.readouterr().out


def test_the_robots_reply_cannot_carry_a_credential():
    """The frame goes back over an unencrypted radio to every listener on the
    channel. Sending the password back would double the exposure for nothing."""
    from robot.config import RobotConfig
    from robot.robot import Robot

    frame = Robot._wifi_frame.__get__(_FakeRobot(RobotConfig()))(
        {"ok": True, "ssid": "Venue-Guest", "psk": "hunter2", "ip": "10.0.0.9"})
    assert frame["ssid"] == "Venue-Guest"
    assert "psk" not in frame
    assert "hunter2" not in json.dumps(frame)


class _FakeRobot:
    """Just the attribute _wifi_frame reads."""

    def __init__(self, cfg):
        self.cfg = cfg


def test_wifi_is_not_a_tunable_path():
    """Config is snapshotted, echoed to every browser and saved to tuning.json.
    A WiFi password must be in none of those, so it is not config at all."""
    from robot import tuning
    assert not [p for p in tuning.PARAMS
                if "psk" in p.path or "wifi" in p.path.split(".")[-1]]


# --- reaching a rover that is on no network ---------------------------------

class _Radio:
    """A radio that records, and admits to having a bulk channel — which is what
    tells the app it is not the in-process simulator."""

    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def send_bulk(self, msg):
        return True

    def start(self):
        pass

    def stop(self):
        pass


def _bridge(ip_server=None):
    """The real app, with a fake radio and an optional fake WiFi server.

    The app must be entered as a context manager — that is what runs the
    lifespan, and the broadcaster it starts is what puts frames on the socket.
    Without it a read blocks forever.
    """
    from fastapi.testclient import TestClient

    from basestation.app import build_app
    from basestation.places import PlaceStore

    fleet = FleetManager()
    fleet.update_from_telemetry(
        {"type": "telemetry", "robot_id": "rover1", "mode": "teleop"}, 100.0)
    radio = _Radio()
    app = build_app(fleet, link=radio, controller=None,
                    web_cfg={"tiles": None, "voice": False},
                    places=PlaceStore(load=False), ip_server=ip_server)
    return TestClient(app), radio


def test_a_join_reaches_a_rover_that_is_on_no_network():
    """The whole feature, through the real bridge. A rover with no WiFi is
    exactly the rover that needs telling about a network, so this path must not
    require the link it configures — if it ever quietly did, the feature would
    work on the bench and be useless at a venue."""
    client, radio = _bridge(ip_server=None)
    with client, client.websocket_connect("/ws") as ws:
        ws.send_json({"action": "set_wifi", "robot_id": "rover1",
                      "ssid": "Venue-Guest", "psk": "hunter2"})
        _settle(ws)
    out = [m for m in radio.sent if m.get("type") == "set_wifi"]
    assert out, "the join never reached the radio"
    assert out[0]["to"] == "rover1"
    assert out[0]["ssid"] == "Venue-Guest"
    # It has to carry the credential — that is the job — and this is precisely
    # why the UI says the radio is unencrypted before you type one.
    assert out[0]["psk"] == "hunter2"


def test_wifi_is_used_when_there_is_a_link_so_no_password_goes_on_air():
    """Not an optimisation, a security property: a rover already on a network is
    moved to another one without its password ever crossing the radio."""
    over_wifi = []

    class LiveIP:
        """A bulk server with a rover connected. `start`/`stop` are there because
        the app's lifespan owns this object's lifecycle."""

        def send(self, msg):
            over_wifi.append(msg)
            return True

        def is_connected(self, rid=None):
            return True

        def start(self):
            pass

        def stop(self):
            pass

    client, radio = _bridge(ip_server=LiveIP())
    with client, client.websocket_connect("/ws") as ws:
        ws.send_json({"action": "set_wifi", "robot_id": "rover1",
                      "ssid": "Venue-Guest", "psk": "hunter2"})
        _settle(ws)
    assert any(m.get("type") == "set_wifi" for m in over_wifi)
    assert not [m for m in radio.sent if m.get("type") == "set_wifi"]


def test_a_scan_request_reaches_the_rover():
    client, radio = _bridge(ip_server=None)
    with client, client.websocket_connect("/ws") as ws:
        ws.send_json({"action": "scan_wifi", "robot_id": "rover1"})
        _settle(ws)
    assert any(m.get("type") == "scan_wifi" for m in radio.sent)


def _settle(ws, frames: int = 6) -> None:
    """Read a few frames so the server has certainly processed what we sent.
    The socket carries telemetry continuously, so anything we asked for has been
    handled by the time a handful of frames have come back."""
    for _ in range(frames):
        ws.receive_json()


# --- the base station's side of the answer ---------------------------------

def test_a_wifi_frame_reaches_the_dashboard():
    fleet = FleetManager()
    fleet.handle({"type": "wifi", "from": "rover1", "ok": True,
                  "ssid": "Venue-Guest", "ip": "10.0.0.9", "signal": 82}, 100.0)
    entry = fleet.wifi()["rover1"]
    assert entry["ssid"] == "Venue-Guest"
    assert entry["rev"] == 1


def test_each_answer_replaces_the_last_rather_than_merging():
    """A failed join merged into a successful scan would leave a panel showing
    both at once."""
    fleet = FleetManager()
    fleet.handle({"type": "wifi", "from": "rover1", "ok": True,
                  "networks": [{"ssid": "PitCrew", "signal": 64, "secure": True}]}, 100.0)
    fleet.handle({"type": "wifi", "from": "rover1", "ok": False,
                  "error": "Secrets were required, but not provided."}, 100.1)
    entry = fleet.wifi()["rover1"]
    assert entry["ok"] is False
    assert "networks" not in entry
    assert entry["rev"] == 2


def test_the_revision_moves_on_every_answer():
    """It is how the dashboard tells a NEW answer from the same one being
    re-pushed — without it a Scan that was never answered looks identical to one
    answered with the same result."""
    fleet = FleetManager()
    for _ in range(3):
        fleet.handle({"type": "wifi", "from": "rover1", "ok": True,
                      "ssid": "PitCrew"}, 100.0)
    assert fleet.wifi()["rover1"]["rev"] == 3


def test_a_wifi_frame_is_not_mistaken_for_telemetry():
    """`handle` routes by type. A wifi frame falling through to the telemetry
    path would mark the robot as having just been seen and invent a mode."""
    fleet = FleetManager()
    fleet.handle({"type": "wifi", "from": "rover1", "ok": True}, 100.0)
    robot = fleet.snapshot(100.0)["robots"][0]
    assert robot["mode"] == "unknown"
    assert robot["age"] is None


def test_a_robot_nobody_asked_has_no_wifi_entry():
    fleet = FleetManager()
    fleet.handle({"type": "telemetry", "from": "rover1", "mode": "teleop"}, 100.0)
    assert fleet.wifi() == {}


# --- the simulator answers in the same shapes ------------------------------

@pytest.fixture
def sim():
    from basestation.simulator import SimulatedFleet
    got = []
    fleet = SimulatedFleet(got.append, n_robots=1)
    return fleet, got


def test_the_simulator_scans(sim):
    fleet, got = sim
    fleet.send({"type": "scan_wifi", "to": "rover1"})
    answer = [m for m in got if m.get("type") == "wifi"][-1]
    assert answer["ok"] is True
    assert any(n["ssid"] == "Venue-Guest" for n in answer["networks"])


def test_the_simulator_refuses_a_wrong_password(sim):
    """The failure path is the one worth having tried before a competition."""
    fleet, got = sim
    fleet.send({"type": "set_wifi", "to": "rover1", "ssid": "Venue-Guest",
                "psk": "wrong"})
    answer = [m for m in got if m.get("type") == "wifi"][-1]
    assert answer["ok"] is False
    assert "Secrets" in answer["error"]
    assert answer["ssid"] is None  # still on nothing


def test_the_simulator_joins_and_reports_an_address(sim):
    fleet, got = sim
    fleet.send({"type": "set_wifi", "to": "rover1", "ssid": "Venue-Guest",
                "psk": "letmein"})
    answer = [m for m in got if m.get("type") == "wifi"][-1]
    assert answer["ok"] is True
    assert answer["ssid"] == "Venue-Guest"
    assert answer["ip"]


def test_the_simulator_joins_an_open_network_with_no_password(sim):
    fleet, got = sim
    fleet.send({"type": "set_wifi", "to": "rover1", "ssid": "FreeWiFi"})
    answer = [m for m in got if m.get("type") == "wifi"][-1]
    assert answer["ok"] is True


def test_the_simulator_forgets(sim):
    fleet, got = sim
    fleet.send({"type": "set_wifi", "to": "rover1", "ssid": "FreeWiFi"})
    fleet.send({"type": "forget_wifi", "to": "rover1", "ssid": "FreeWiFi"})
    answer = [m for m in got if m.get("type") == "wifi"][-1]
    assert answer["forgot"] == "FreeWiFi"
    assert answer["ssid"] is None
