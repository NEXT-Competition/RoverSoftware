"""Restarting a rover from the base station.

A layout only takes effect at start-up — actuators are built in the constructor —
so every hardware change used to end in an ssh session with a rover that was, by
then, on a field or on blocks. This is that ssh session over the radio.

The two properties worth pinning down are both about NOT stranding a machine:
the motors are parked before the process ends (the loop's own `finally` does it,
which is why the restart ends the loop rather than shelling out to systemctl),
and a robot that nothing is supervising REFUSES, because there "restart" would
mean "switch off until someone walks over with a laptop".
"""

import time

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager
from basestation.places import PlaceStore
from basestation.settings import SettingsStore
from robot.config import MechanismConfig, MotorConfig, RobotConfig
from robot.robot import EXIT_RESTART, Robot


# --- the robot ---------------------------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    # Every test here is about a robot that IS a service; the one that isn't
    # deletes this itself.
    monkeypatch.setenv("INVOCATION_ID", "0123456789abcdef")
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.mechanisms = {"intake": MechanismConfig(
        name="intake", kind="power",
        actuators={"roller": MotorConfig(channel=4, name="roller")},
        presets={"in": {"roller": 1.0}})}
    bot = Robot(cfg)
    bot.link.send = lambda msg: None
    return bot


def restart(rover):
    rover._inbox.put({"type": "restart"})
    rover._drain_inbox()


def test_a_restart_ends_the_control_loop(rover):
    rover._running = True
    restart(rover)
    assert rover._running is False


def test_the_exit_status_is_what_brings_it_back(rover):
    """The whole mechanism. systemd's shipped policy is Restart=on-failure, so
    a process that exited 0 would stay dead — the status is not cosmetic."""
    restart(rover)
    assert rover._restarting is True
    assert EXIT_RESTART != 0


def test_a_robot_that_was_not_asked_exits_clean(rover):
    assert rover._restarting is False


def test_a_restart_is_refused_when_nothing_supervises_the_process(rover, monkeypatch):
    """A bench robot started by hand has no supervisor, and "restart" there
    means "switch off". It says so instead, and keeps running."""
    monkeypatch.delenv("INVOCATION_ID", raising=False)
    rover._running = True
    restart(rover)
    assert rover._running is True
    assert rover._restarting is False


def test_the_motors_are_parked_on_the_way_out(rover):
    """The reason this ends the loop instead of calling systemctl: shutdown()
    runs in the loop's `finally`, so the machine is safe before the process
    goes. A systemctl restart from inside the unit races that against SIGTERM."""
    rover.mechanisms["intake"].apply_preset("in")
    assert rover.mechanisms["intake"].motors["roller"].throttle == 1.0
    rover.shutdown()
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_run_reports_the_restart_status(rover, monkeypatch):
    """run() returns the code run_robot.py exits with. Driven through one tick
    by pre-loading the message, so the loop drains it and stops itself."""
    monkeypatch.setattr(rover, "start", lambda: None)
    rover._inbox.put({"type": "restart"})
    rover._running = True
    assert rover.run() == EXIT_RESTART


def test_run_reports_zero_for_an_ordinary_stop(rover, monkeypatch):
    monkeypatch.setattr(rover, "start", lambda: None)
    rover._running = False
    assert rover.run() == 0


# --- the base station --------------------------------------------------------

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
def rig(tmp_path):
    fleet = FleetManager()
    now = time.monotonic()
    for rid in ("rover1", "rover2"):
        fleet.update_from_telemetry(
            {"type": "telemetry", "robot_id": rid, "mode": "teleop",
             "estop": False}, now)
    fleet.select("rover1")
    link = FakeLink()
    app = build_app(fleet, link=link, controller=None, web_cfg={"tiles": None},
                    places=PlaceStore(load=False),
                    settings=SettingsStore(path=str(tmp_path / "b.json")))
    return app, link


def test_the_action_reaches_the_named_robot(rig):
    app, link = rig
    app.state.handle_action({"action": "restart_robot", "robot_id": "rover2"})
    assert {"type": "restart", "to": "rover2"} in link.sent


def test_it_restarts_the_selection_when_no_robot_is_named(rig):
    app, link = rig
    app.state.handle_action({"action": "restart_robot"})
    assert {"type": "restart", "to": "rover1"} in link.sent


def test_it_goes_over_the_radio(rig):
    """Not the WiFi bulk path the rest of configuration takes: a rover worth
    restarting is often one whose WiFi is part of what is wrong with it."""
    app, link = rig
    app.state.handle_action({"action": "restart_robot", "robot_id": "rover1"})
    assert [m for m in link.sent if m.get("type") == "restart"]
