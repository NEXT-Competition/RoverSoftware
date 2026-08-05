"""Who owns the drive channel when two inputs both want it.

The bug this pins down, observed on the radio while a trigger was held at full
forward:

    {"type":"drive","throttle":1.0,"steer":0.0,"to":"rover1"}
    {"type":"drive","throttle":0.0,"steer":0.0,"to":"rover1"}
    {"type":"drive","throttle":1.0,"steer":0.0,"to":"rover1"}

Two senders reached the radio with equal authority — the base station's pygame
pad, and a browser whose on-screen joystick was re-transmitting a resting zero
to keep the robot's command_timeout alive. Interleaved, they made the rover
stutter instead of drive. The pad now wins while it is in use.
"""

import time

import pytest

from basestation.app import build_app
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


class FakeController:
    """Stands in for ControllerReader: build_app binds its callbacks to us."""

    def __init__(self):
        self.on_drive = None
        self.on_action = None
        self.connected = True
        self.name = "fake pad"

    def set_mapping(self, mapping):
        pass

    def state(self):
        return {"connected": True, "name": self.name, "axes": [], "buttons": []}

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
    fleet.select("rover1")
    link, controller = FakeLink(), FakeController()
    app = build_app(fleet, link=link, controller=controller,
                    web_cfg={"tiles": None}, places=PlaceStore(load=False))
    return app, link, controller, fleet


def drives(link, rid=None):
    return [m for m in link.sent
            if m.get("type") == "drive" and (rid is None or m.get("to") == rid)]


def test_browser_zero_cannot_interrupt_a_held_trigger(rig):
    """The reported symptom: no 0.0 lands between two 1.0s."""
    app, link, controller, _ = rig
    for _ in range(3):
        controller.on_drive(1.0, 0.0)
        # The touch joystick's resting keepalive, arriving mid-hold.
        app.state.handle_action({"action": "drive", "throttle": 0.0, "steer": 0.0})
    assert drives(link), "the pad's own frames must still go out"
    assert all(m["throttle"] == 1.0 for m in drives(link)), \
        f"a browser zero reached the radio mid-hold: {drives(link)}"


def test_touch_joystick_works_when_the_pad_is_idle(rig):
    """Arbitration must not cost the tablet its joystick."""
    app, link, _, _ = rig
    app.state.handle_action({"action": "drive", "throttle": 0.5, "steer": -0.25})
    assert drives(link) == [
        {"type": "drive", "throttle": 0.5, "steer": -0.25, "to": "rover1"}]


def test_a_pad_at_rest_does_not_claim_the_channel(rig):
    """Only movement claims it — a plugged-in pad nobody is touching reports
    (0, 0) forever, and must not lock the touch joystick out for good."""
    app, link, controller, _ = rig
    controller.on_drive(0.0, 0.0)
    app.state.handle_action({"action": "drive", "throttle": 0.4, "steer": 0.0})
    assert any(m["throttle"] == 0.4 for m in drives(link)), \
        "an idle pad blocked the touch joystick"


def test_the_claim_lapses_so_letting_go_hands_the_channel_back(rig):
    app, link, controller, _ = rig
    controller.on_drive(1.0, 0.0)
    app.state.handle_action({"action": "drive", "throttle": 0.3, "steer": 0.0})
    assert not any(m["throttle"] == 0.3 for m in drives(link)), "claim not held"
    time.sleep(1.05)
    app.state.handle_action({"action": "drive", "throttle": 0.3, "steer": 0.0})
    assert any(m["throttle"] == 0.3 for m in drives(link)), \
        "the pad kept the channel after the operator let go"


def test_a_second_rover_cannot_be_driven_at_the_same_time(rig):
    """This test used to assert the opposite — that a browser naming rover2
    could drive it while the pad drove rover1 — and the policy has since been
    reversed deliberately.

    Two rovers being driven at once is two drive streams on one shared radio
    channel, and drive is the only traffic the base station STREAMS: it repeats
    at drive_hz and keeps repeating at the keepalive while a stick is held. The
    second stream comes out of the airtime of the rover somebody is actually
    driving. So the stream goes to the selected rover in teleop, and to nobody
    else — see app.py::update_drive_target.

    Note this is enforced on the BRIDGE, not in the browser: `robot_id` arrives
    off the wire, so a client could otherwise drive any rover it named
    regardless of what the dashboard has selected.
    """
    app, link, controller, _ = rig
    controller.on_drive(1.0, 0.0)  # selected robot is rover1
    app.state.handle_action(
        {"action": "drive", "robot_id": "rover2", "throttle": 0.6, "steer": 0.0})
    assert drives(link, "rover2") == []
    assert drives(link, "rover1"), "the selected rover still drives"


def test_estop_is_never_gated_on_who_holds_the_stick(rig):
    """The whole reason the gate is scoped to `drive` alone."""
    app, link, controller, _ = rig
    controller.on_drive(1.0, 0.0)
    app.state.handle_action({"action": "estop"})
    assert {"type": "estop", "to": "rover1"} in link.sent


def test_mode_changes_are_not_gated_either(rig):
    app, link, controller, _ = rig
    controller.on_drive(1.0, 0.0)
    app.state.handle_action({"action": "mode", "mode": "waypoint"})
    assert {"type": "mode", "mode": "waypoint", "to": "rover1"} in link.sent


# --- the e-stop is never rate-limited -----------------------------------------

def test_estop_stops_the_drivetrain_now_not_over_the_decel_ramp(monkeypatch, tmp_path):
    """The manager answers stopped() every tick once the latch is on, and that
    goes through drive() and therefore the slew limiter. With a deceleration
    rate configured, the button that exists to stop the rover would otherwise
    ask it to ease off over the next second."""
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    from robot.config import RobotConfig
    from robot.robot import Robot

    cfg = RobotConfig()
    cfg.gps.enabled = cfg.camera.enabled = cfg.vision.enabled = False
    cfg.imu.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.drive.slew_rate = 1.0
    cfg.drive.decel_rate = 0.5          # deliberately glacial
    rover = Robot(cfg)

    # Get the tracks genuinely moving first. One call is enough: the first
    # command after construction is deliberately unlimited (there is no elapsed
    # time to limit against yet).
    rover.drive.drive(1.0, 1.0)
    assert rover.drive.left.throttle > 0.5

    rover._inbox.put({"type": "estop"})
    rover._drain_inbox()
    rover._apply_estop()

    assert rover.drive.left.throttle == 0.0
    assert rover.drive.right.throttle == 0.0


def test_the_estop_resets_the_limiter_so_it_does_not_resume_at_speed(
        monkeypatch, tmp_path):
    """`Drivetrain.stop` resets the limiter, so its idea of "where I was" is
    zero rather than the speed it was doing when the button went in."""
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    from robot.config import RobotConfig
    from robot.robot import Robot

    cfg = RobotConfig()
    cfg.gps.enabled = cfg.camera.enabled = cfg.vision.enabled = False
    cfg.imu.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.drive.slew_rate = 1.0
    rover = Robot(cfg)
    rover.drive.drive(1.0, 1.0)
    assert rover.drive._limiter._current == [1.0, 1.0]

    rover._inbox.put({"type": "estop"})
    rover._drain_inbox()
    rover._apply_estop()
    assert rover.drive._limiter._current == [0.0, 0.0]
