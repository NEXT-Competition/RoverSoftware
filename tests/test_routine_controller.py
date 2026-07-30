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


class AlignSpy(Spy):
    """A Spy that can be told how near to stop, as the aligning controllers can.

    Kept distinct from the plain Spy on purpose: `standoff_m` is exactly what
    RoutineController duck-types on to decide whether a state's stop distance
    means anything, so a build where every controller had one would never
    exercise the branch that refuses it.
    """

    def __init__(self, command=None, standoff_m=0.0):
        super().__init__(command)
        self.standoff_m = standoff_m


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
    # waypoint stays a plain Spy: it has no standoff, which is what makes
    # "stop within N m" meaningless there and worth refusing.
    spies = {"object_align": AlignSpy(), "waypoint": Spy(),
             "shooter_align": AlignSpy()}
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


# --- what an aligning state aims at ------------------------------------------
#
# The routine BORROWS the detector's target label. Every test here is really
# about the same claim: whatever a routine points the camera at, the operator's
# own choice is what is there afterwards.


class FakeVision:
    """Stands in for VisionConfig — the one field the routine touches."""

    def __init__(self, target_label=""):
        self.target_label = target_label


def test_an_aligning_state_points_the_detector_at_its_target():
    rc, _ = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision()
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    assert vision.target_label == "bucket"


def test_the_operators_own_target_comes_back_when_the_routine_ends():
    """The operator chose "cone" in Settings. A routine that aimed at a bucket
    for four seconds must not leave the detector filtering on buckets for the
    rest of the match, with nothing on screen to explain why."""
    rc, _ = make([
        # Held on an event rather than `always`: the engine takes its transition
        # inside the same tick it entered on, so an `always` state is never
        # observably inhabited — and the point here is what happens WHILE it is.
        # An event, not a delay, because the engine reads a real clock (see
        # tests/test_routine_engine.py) and this controller owns its own engine.
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision("cone")
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    assert vision.target_label == "bucket"
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)  # reaches the terminal state, so the run ends
    assert vision.target_label == "cone"


def test_the_target_comes_back_on_an_estop_too():
    """Every exit path, not just the tidy one."""
    rc, _ = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision("cone")
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    rc.on_estop()
    assert vision.target_label == "cone"


def test_leaving_routine_mode_hands_the_target_back():
    rc, _ = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision("cone")
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    rc.on_deactivate()
    assert vision.target_label == "cone"


def test_consecutive_states_may_aim_at_different_things():
    """And the value restored at the end is the operator's, not the first
    state's — the bug this test exists to catch is treating each state's
    predecessor as the thing to put back."""
    rc, _ = make([
        {"id": "find", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "event", "name": "go", "to": "shoot"}]},
        {"id": "shoot", "drive": {"mode": "shooter_align", "target": "goal"},
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision("cone")
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    assert vision.target_label == "bucket"
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert vision.target_label == "goal"
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert vision.target_label == "cone"


def test_a_state_with_no_target_restores_rather_than_clearing():
    """"" means "whatever is already selected", not "anything". A state that
    doesn't care must not silently widen the filter the operator set."""
    rc, _ = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "always", "to": "coast"}]},
        {"id": "coast", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision("cone")
    rc.set_vision_config(vision)
    rc.on_activate()
    rc.update(0.02)
    rc.update(0.02)
    assert vision.target_label == "cone"


# --- how near an aligning state gets ----------------------------------------
#
# Same borrow-and-hand-back claim as the target above, one field over. The
# failure this guards against is quieter than a wrong target and worse: a
# routine that left the standoff rewritten makes the NEXT alignment — manual,
# spoken, or another routine — stop somewhere nobody chose.

def test_an_aligning_state_sets_the_delegates_stop_distance():
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align", "stop_within_m": 1.5},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 1.5


def test_the_operators_own_stop_distance_comes_back():
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align", "stop_within_m": 1.5},
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    spies["object_align"].standoff_m = 3.0  # what Settings was left on
    rc.on_activate()
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 1.5
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 3.0


def test_the_stop_distance_comes_back_on_an_estop_too():
    """Every exit path, as with the target."""
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align", "stop_within_m": 1.5},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    spies["object_align"].standoff_m = 3.0
    rc.on_activate()
    rc.update(0.02)
    rc.on_estop()
    assert spies["object_align"].standoff_m == 3.0


def test_consecutive_states_may_stop_at_different_distances():
    """And what is restored at the end is the operator's value, not the previous
    state's — the bug is treating each state's predecessor as the thing to
    put back."""
    rc, spies = make([
        {"id": "near", "drive": {"mode": "object_align", "stop_within_m": 1.0},
         "transitions": [{"when": "event", "name": "go", "to": "far"}]},
        {"id": "far", "drive": {"mode": "object_align", "stop_within_m": 4.0},
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    spies["object_align"].standoff_m = 3.0
    rc.on_activate()
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 1.0
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 4.0
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 3.0


def test_each_aligning_controller_gets_its_own_distance_back():
    """object_align and shooter_align are separate instances with separate
    standoffs. Handing one back onto the other would leave the shooter stopping
    where an approach state was told to."""
    rc, spies = make([
        {"id": "find", "drive": {"mode": "object_align", "stop_within_m": 1.0},
         "transitions": [{"when": "event", "name": "go", "to": "shoot"}]},
        {"id": "shoot", "drive": {"mode": "shooter_align", "stop_within_m": 5.0},
         "transitions": [{"when": "event", "name": "go", "to": "done"}]},
        {"id": "done", "terminal": True}])
    spies["object_align"].standoff_m = 2.0
    spies["shooter_align"].standoff_m = 8.0
    rc.on_activate()
    rc.update(0.02)
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    rc.on_message({"type": "routine_event", "name": "go"})
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 2.0
    assert spies["shooter_align"].standoff_m == 8.0


def test_a_state_without_a_stop_distance_leaves_the_standoff_alone():
    """Omitted means "the controller's own", which is what every routine written
    before the field existed means."""
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    spies["object_align"].standoff_m = 3.0
    rc.on_activate()
    rc.update(0.02)
    assert spies["object_align"].standoff_m == 3.0


def test_the_ignored_distance_warning_is_not_printed_every_tick(capsys):
    """_sync_delegate runs twice a tick at 50 Hz. A log line per call would bury
    the journal — and this one only reports something the operator cannot fix
    mid-run anyway."""
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    del spies["object_align"].standoff_m  # a build whose controller has none
    rc.on_activate()
    for _ in range(20):
        rc.update(0.02)
    assert capsys.readouterr().out.count("stop distance") == 0


def test_the_target_is_set_before_the_delegate_is_activated():
    """An alignment controller resets its search timers in on_activate and then
    looks for whatever the detector is filtering on. Setting the label second
    means the first moments of every aiming state hunt the wrong object."""
    seen = []
    rc, spies = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    vision = FakeVision()
    rc.set_vision_config(vision)
    spies["object_align"].on_activate = lambda: seen.append(vision.target_label)
    rc.on_activate()
    rc.update(0.02)
    assert seen == ["bucket"]


def test_a_build_with_no_vision_runs_the_routine_anyway():
    """A target on a robot with no camera is worth a line in the log, not a
    refusal — the rest of the routine still does its job."""
    rc, _ = make([
        {"id": "aim", "drive": {"mode": "object_align", "target": "bucket"},
         "transitions": [{"when": "always", "to": "done"}]},
        {"id": "done", "terminal": True}])
    rc.on_activate()
    rc.update(0.02)
    rc.update(0.02)  # must not raise on the way out either


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
