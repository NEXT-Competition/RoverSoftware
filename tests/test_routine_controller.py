"""The `routine` mode, and the safety properties a user-authored FSM must not
be able to break.

Delegation is the design worth testing hard: a state that says "drive with
object_align" gets the real alignment controller, and this controller — not
ControlManager — owns activating and deactivating it. Getting that wrong means a
delegate whose search timers never reset, or one left running after the mode
changed.
"""

import json

import pytest

from robot.config import MechanismConfig, MotorConfig, RobotConfig, RoutineConfig
from robot.control.commands import DriveCommand
from robot.control.controller import Controller
from robot.control.routine_controller import RoutineController
from robot.routine import schema
from robot.robot import Robot

CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint", "routine")


class Spy(Controller):
    """A stand-in delegate that records its lifecycle."""

    def __init__(self, command=None):
        self.activations = 0
        self.deactivations = 0
        self.messages = []
        self.command = command or DriveCommand.tank(0.7, 0.7)

    def on_activate(self):
        self.activations += 1

    def on_deactivate(self):
        self.deactivations += 1

    def on_message(self, message):
        self.messages.append(message)

    def update(self, dt):
        return self.command


class FakeMech:
    def __init__(self):
        self.stopped = 0
        self.power = 0.0

    def set_power(self, power, actuator=None):
        self.power = power
        return True

    def apply_preset(self, name):
        self.power = 1.0
        return True

    def stop(self):
        self.stopped += 1
        self.power = 0.0

    def ready(self):
        return True


def make(states, mechanisms=None, cfg=None, **routine_kw):
    spies = {"object_align": Spy(), "waypoint": Spy(), "shooter_align": Spy()}
    controllers = dict(spies)
    cfg = cfg or RoutineConfig()
    rc = RoutineController(controllers, mechanisms or {}, cfg)
    controllers["routine"] = rc

    doc = {"version": 1, "routines": [dict(
        id="r1", start=states[0]["id"], states=states, **routine_kw)]}
    result = schema.parse(doc, cfg, CONTROLLERS)
    assert result.ok, result.errors
    rc.set_routines(result.routines)
    return rc, spies


# --- delegation --------------------------------------------------------------

def test_a_delegating_state_activates_the_real_controller():
    rc, spies = make([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    assert spies["object_align"].activations == 1


def test_the_delegates_command_is_what_drives():
    rc, spies = make([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    cmd = rc.update(0.02)
    assert (cmd.left, cmd.right) == pytest.approx((0.7, 0.7))


def test_leaving_a_state_deactivates_its_delegate():
    rc, spies = make([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "always", "to": "nav"}]},
        {"id": "nav", "drive": {"mode": "waypoint"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)  # enters seek's delegate
    rc.update(0.02)  # transitions to nav
    assert spies["object_align"].deactivations == 1
    assert spies["waypoint"].activations == 1


def test_a_delegate_is_not_churned_while_the_state_stays_put():
    """on_activate resets search timers and PID state. Doing it every tick would
    make an alignment loop that can never converge."""
    rc, spies = make([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    for _ in range(5):
        rc.update(0.02)
    assert spies["object_align"].activations == 1


def test_leaving_routine_mode_releases_the_delegate():
    rc, spies = make([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    rc.on_deactivate()
    assert spies["object_align"].deactivations == 1


def test_messages_reach_whoever_is_driving():
    """A route push or a manual fire belongs to the delegate — it is the active
    controller in every sense except the manager's bookkeeping."""
    rc, spies = make([
        {"id": "nav", "drive": {"mode": "waypoint"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    rc.on_message({"type": "route", "waypoints": [[1, 2]]})
    assert spies["waypoint"].messages[-1]["type"] == "route"


def test_a_manual_state_drives_without_any_delegate():
    rc, spies = make([
        {"id": "go", "drive": {"mode": "manual", "throttle": 0.5, "steer": 0.2},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    cmd = rc.update(0.02)
    assert (cmd.left, cmd.right) == pytest.approx((0.7, 0.3))
    assert spies["object_align"].activations == 0


# --- run control -------------------------------------------------------------

def test_with_no_routine_the_robot_holds_still():
    rc = RoutineController({}, {}, RoutineConfig())
    rc.on_activate()
    cmd = rc.update(0.02)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


def test_routine_cmd_stop_ends_the_run():
    rc, _ = make([
        {"id": "go", "drive": {"mode": "manual", "throttle": 0.5},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    rc.on_message({"type": "routine_cmd", "cmd": "stop"})
    assert rc.engine is None
    assert (rc.update(0.02).left, rc.update(0.02).right) == (0.0, 0.0)


def test_routine_cmd_restart_returns_to_the_start_state():
    rc, _ = make([
        {"id": "a", "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    assert rc.engine.state.id == "b"
    rc.on_message({"type": "routine_cmd", "cmd": "restart"})
    assert rc.engine.state.id == "a"


def test_an_event_reaches_the_engine():
    rc, _ = make([
        {"id": "a", "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert rc.engine is None or rc.engine.state.id == "done"


def test_replacing_the_routines_stops_a_running_machine():
    """Continuing to execute state 'shoot' out of a document that no longer
    contains it is worse than stopping."""
    rc, _ = make([
        {"id": "a", "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    assert rc.engine is not None
    rc.set_routines({})
    assert rc.engine is None


def test_selecting_a_routine_that_does_not_exist_is_reported_not_fatal():
    rc, _ = make([{"id": "a", "terminal": True}])
    assert rc.select("nope") is False


# --- safety ------------------------------------------------------------------

def test_ending_a_run_stops_every_mechanism():
    mech = FakeMech()
    rc, _ = make([
        {"id": "go", "on_enter": [{"do": "mech_preset", "mech": "intake",
                                   "preset": "in"}],
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}], mechanisms={"intake": mech})
    rc.on_activate()
    rc.update(0.02)
    assert mech.power == 1.0
    rc.on_deactivate()
    assert mech.power == 0.0


def test_an_estop_aborts_back_to_the_start():
    """Clearing an e-stop must not resume a half-finished sequence whose idea of
    where the robot is went stale several seconds ago."""
    rc, _ = make([
        {"id": "a", "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    assert rc.engine.state.id == "b"
    rc.on_estop()
    assert rc.engine is None


def test_on_estop_hold_keeps_the_machine_where_it_was():
    rc, _ = make([
        {"id": "a", "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}], on_estop="hold")
    rc.on_activate()
    rc.update(0.02)
    rc.on_estop()
    assert rc.engine is not None


def test_a_routine_cannot_arm_when_the_gate_is_off_at_runtime():
    """The schema refuses `arm` at parse time when the gate is off, which gives
    the editor a clear error. This is the other half: turning the gate off must
    stop the routine that is ALREADY running, because the only direction a
    safety gate may be slow in is on."""
    cfg = RoutineConfig(allow_arm=True)
    rc, spies = make([
        {"id": "shoot", "drive": {"mode": "shooter_align"},
         "on_enter": [{"do": "arm"}],
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}], cfg=cfg)

    cfg.allow_arm = False  # the operator flips it off
    rc.on_activate()
    rc.update(0.02)
    assert not any(m.get("type") == "arm_shooter"
                   for m in spies["shooter_align"].messages)


def test_arming_reaches_the_controller_when_the_gate_is_on():
    cfg = RoutineConfig(allow_arm=True)
    rc, spies = make([
        {"id": "shoot", "drive": {"mode": "shooter_align"},
         "on_enter": [{"do": "arm"}],
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}], cfg=cfg)
    rc.on_activate()
    rc.update(0.02)
    assert any(m.get("type") == "arm_shooter"
               for m in spies["shooter_align"].messages)


# --- integration with the robot ---------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    monkeypatch.setenv("RS_LAYOUT_FILE", str(tmp_path / "layout.json"))
    monkeypatch.setenv("RS_ROUTINES_FILE", str(tmp_path / "routines.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.mechanisms = {"intake": MechanismConfig(
        name="intake", kind="power",
        actuators={"roller": MotorConfig(channel=4, name="roller")},
        presets={"in": {"roller": 1.0}})}
    bot = Robot(cfg)
    sent = []

    def take(message):
        sent.append(message)
        return True

    bot.link.send = take
    # Bulk frames are metered against the radio's real byte rate, so the real
    # link would refuse most of a multi-frame reply on any one tick. A test
    # isn't waiting out that pacing; tests/test_airtime.py exercises it.
    bot.link.send_bulk = take
    bot.sent = sent
    # Documents travel over the robot's WiFi link, never the radio (see
    # robot/robot.py::_drain_outbox), so a rover with no link would drop every
    # reply below. `bot.sent` is what the base station receives either way.
    bot.ip_link = _FakeIP(sent)
    return bot


class _FakeIP:
    """A connected IPLink that records into the same list as the fake radio.

    Which link carried what is tested in tests/test_robot_config.py; here it
    only has to exist, so the reply lands somewhere the assertions can see it.
    """

    def __init__(self, sink):
        self.sent = sink
        self.host, self.port = "base.local", 5006

    def is_connected(self):
        return True

    def send(self, msg):
        self.sent.append(msg)
        return True


def deliver(bot, msg):
    bot._inbox.put(msg)
    bot._drain_inbox()
    while bot._outbox:
        bot._drain_outbox()


DOC = {"version": 1, "routines": [{
    "id": "demo", "start": "spin", "states": [
        {"id": "spin", "drive": {"mode": "manual", "throttle": 0.5},
         "on_enter": [{"do": "mech_preset", "mech": "intake", "preset": "in"}],
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}]}]}


def put(bot, doc, mtype="put_routines"):
    from robot.comms.doc_transfer import split
    for frame in split(doc, mtype, txid="t1"):
        bot._inbox.put(frame)
    bot._drain_inbox()
    while bot._outbox:
        bot._drain_outbox()


def test_the_routine_mode_is_registered(rover):
    assert "routine" in rover.manager.controllers


def test_a_routine_document_arrives_and_is_acknowledged(rover):
    put(rover, DOC)
    result = [f for f in rover.sent if f["type"] == "routines_result"]
    assert result and result[-1]["ok"] is True
    assert result[-1]["rev"] == 1


def test_a_bad_routine_document_is_reported_and_not_installed(rover):
    put(rover, DOC)
    put(rover, {"version": 1, "routines": [{"id": "bad", "start": "x", "states": [
        {"id": "a", "transitions": [{"when": "always", "to": "nowhere"}]}]}]})
    result = [f for f in rover.sent if f["type"] == "routines_result"][-1]
    assert result["ok"] is False
    assert result["errors"]
    # the previous good set survives
    assert "demo" in rover.manager.controllers["routine"].routines


def test_a_routine_runs_through_the_control_loop(rover):
    put(rover, DOC)
    deliver(rover, {"type": "mode", "mode": "routine"})
    cmd = rover.manager.update(0.02)
    assert cmd.left == pytest.approx(0.5)
    assert rover.mechanisms["intake"].motors["roller"].throttle == 1.0

    deliver(rover, {"type": "routine_event", "name": "go"})
    rover.manager.update(0.02)
    rover.manager.update(0.02)
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_the_live_state_reaches_telemetry(rover):
    put(rover, DOC)
    deliver(rover, {"type": "mode", "mode": "routine"})
    rover.manager.update(0.02)
    telemetry = rover._telemetry(DriveCommand.stopped())
    assert telemetry["routine"]["state"] == "spin"


def test_an_estop_stops_a_routine_and_its_mechanisms(rover):
    put(rover, DOC)
    deliver(rover, {"type": "mode", "mode": "routine"})
    rover.manager.update(0.02)
    assert rover.mechanisms["intake"].motors["roller"].throttle == 1.0

    deliver(rover, {"type": "estop"})
    rover._apply_estop()
    cmd = rover.manager.update(0.02)

    assert (cmd.left, cmd.right) == (0.0, 0.0)
    assert rover.mechanisms["intake"].motors["roller"].throttle == 0.0


def test_editor_positions_survive_the_round_trip(rover):
    """The dashboard keeps its node positions in the document itself, so the
    diagram a teammate opens is the one you drew. That works because the robot
    stores and echoes the RAW document rather than a re-serialization — which
    makes it a property worth pinning down rather than an accident."""
    doc = json.loads(json.dumps(DOC))
    doc["routines"][0]["states"][0]["x"] = 120
    doc["routines"][0]["states"][0]["y"] = -40
    put(rover, doc)

    echoed = [f for f in rover.sent if f["type"] == "routines"]
    assert echoed, "the robot should echo back what it stored"
    rebuilt = json.loads("".join(f["part"] for f in sorted(echoed, key=lambda f: f["seq"])))
    assert rebuilt["routines"][0]["states"][0]["x"] == 120
    assert rebuilt["routines"][0]["states"][0]["y"] == -40


def test_a_position_is_never_interpreted_by_the_engine(rover):
    """Garbage in an editor-only key must not reach the state machine."""
    doc = json.loads(json.dumps(DOC))
    doc["routines"][0]["states"][0]["x"] = "banana"
    put(rover, doc)
    result = [f for f in rover.sent if f["type"] == "routines_result"][-1]
    assert result["ok"] is True


def test_routines_survive_a_restart(rover, tmp_path):
    put(rover, DOC)
    assert (tmp_path / "routines.json").exists()
    reborn = Robot(rover.cfg)
    assert "demo" in reborn.manager.controllers["routine"].routines
