"""The simulator answers document frames with the real validators.

The repo's own standard — "a settings page you can only test on a real rover is
a settings page that ships broken" — applies double to a state-machine editor.
So the point of these tests is not that the simulator works, it is that it
refuses the same documents the rover refuses and runs the same engine, which is
what makes `python run_basestation.py --sim` a real check of the UI.
"""

import time

import pytest

from basestation.fleet import FleetManager
from basestation.simulator import SimulatedFleet
from robot.comms.doc_transfer import split

LAYOUT = {
    "version": 1,
    "drive": {
        "kind": "servo_steer",
        "actuators": [{"name": "rear", "kind": "esc", "channel": 0},
                      {"name": "steering", "kind": "servo", "channel": 3}],
        "roles": {"throttle": ["rear"], "steer": "steering"},
    },
    "mechanisms": [{"name": "intake", "kind": "power",
                    "actuators": [{"name": "roller", "channel": 4}],
                    "presets": {"in": {"roller": 1.0}}}],
}

ROUTINE = {"version": 1, "routines": [{
    "id": "demo", "start": "go", "states": [
        {"id": "go", "drive": {"mode": "manual", "throttle": 0.6, "steer": 0.0},
         "on_enter": [{"do": "mech_preset", "mech": "intake", "preset": "in"}],
         "transitions": [{"when": "event", "name": "stop", "to": "done"}]},
        {"id": "done", "terminal": True}]}]}


@pytest.fixture
def sim():
    received = []
    fleet = SimulatedFleet(received.append, n_robots=1)
    fleet.received = received
    return fleet


def put(sim, doc, mtype, rid="rover1"):
    for frame in split(doc, mtype, txid="t"):
        sim.send({**frame, "to": rid})


def results(sim, mtype):
    return [m for m in sim.received if m.get("type") == mtype]


# --- documents ---------------------------------------------------------------

def test_a_layout_is_accepted_and_reported_as_needing_a_restart():
    received = []
    sim = SimulatedFleet(received.append, n_robots=1)
    put(sim, LAYOUT, "put_layout")
    result = [m for m in received if m["type"] == "layout_result"][-1]
    assert result["ok"]
    assert result["restart_required"] is True
    assert sim.robots["rover1"].cfg.drive.kind == "servo_steer"


def test_a_bad_layout_is_refused_with_the_rovers_own_message(sim):
    bad = {"version": 1, "drive": {
        "kind": "tank",
        "actuators": [{"name": "a", "channel": 0}, {"name": "b", "channel": 0}],
        "roles": {"left": ["a"], "right": ["b"]}}}
    put(sim, bad, "put_layout")
    result = results(sim, "layout_result")[-1]
    assert not result["ok"]
    assert "channel 0" in " ".join(result["errors"])


def test_a_routine_is_accepted(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    assert results(sim, "routines_result")[-1]["ok"]
    assert "demo" in sim.robots["rover1"].routines


def test_a_bad_routine_is_refused_and_the_last_good_one_survives(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    put(sim, {"version": 1, "routines": [{"id": "bad", "start": "a", "states": [
        {"id": "a", "transitions": [{"when": "always", "to": "nowhere"}]}]}]},
        "put_routines")
    result = results(sim, "routines_result")[-1]
    assert not result["ok"]
    assert "demo" in sim.robots["rover1"].routines


def test_documents_are_answered_in_chunks_like_the_radio(sim):
    """Chunked even though nothing here is bandwidth-limited: the reassembly
    path the dashboard uses in the field is the one it uses on a laptop."""
    put(sim, LAYOUT, "put_layout")
    sim.received.clear()
    sim.send({"type": "get_layout", "to": "rover1"})
    frames = results(sim, "layout")
    assert frames
    assert all("seq" in f and "n" in f for f in frames)


def test_field_descriptors_describe_the_layouts_own_actuators(sim):
    put(sim, LAYOUT, "put_layout")
    sim.received.clear()
    sim.send({"type": "get_fields", "to": "rover1"})
    fleet = FleetManager()
    for frame in results(sim, "fields"):
        fleet.update_from_document(frame)
    described = {f["path"] for f in fleet.documents()["rover1"]["fields"]}
    assert "drive.rear.deadband" in described
    assert "mech.intake.roller.channel" in described


# --- running a routine -------------------------------------------------------

def test_a_routine_actually_drives_the_fake_rover(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    robot = sim.robots["rover1"]
    robot.step(0.02)
    assert robot.telemetry()["routine"]["state"] == "go"
    assert robot.left > 0
    assert robot.mech_power["intake"] == 1.0


def test_an_operator_event_advances_the_machine(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    robot = sim.robots["rover1"]
    robot.step(0.02)
    sim.send({"type": "routine_event", "name": "stop", "to": "rover1"})
    robot.step(0.02)
    robot.step(0.02)
    assert robot.mech_power["intake"] == 0.0
    assert robot.left == 0.0


def test_an_estop_stops_a_running_routine(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    robot = sim.robots["rover1"]
    robot.step(0.02)
    sim.send({"type": "estop", "to": "rover1"})
    robot.step(0.02)
    assert robot.engine is None
    assert robot.mech_power["intake"] == 0.0
    assert (robot.left, robot.right) == (0.0, 0.0)


def test_leaving_routine_mode_stops_the_mechanisms(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    sim.robots["rover1"].step(0.02)
    sim.send({"type": "mode", "mode": "teleop", "to": "rover1"})
    assert sim.robots["rover1"].mech_power["intake"] == 0.0


# --- the drivetrain kind is visible on the map ------------------------------

def test_a_servo_steer_layout_creeps_instead_of_pivoting(sim):
    """The mitigation for a steered chassis being unable to turn in place. It is
    testable here, which is the point — the picker is not a field you need to
    own a rover to try."""
    put(sim, LAYOUT, "put_layout")
    robot = sim.robots["rover1"]
    robot.set_arcade(0.0, 0.4)
    assert (robot.left + robot.right) / 2 == pytest.approx(
        robot.cfg.drive.min_pivot_throttle)


def test_a_tank_layout_still_pivots_in_place(sim):
    robot = sim.robots["rover1"]
    robot.set_arcade(0.0, 0.5)
    assert robot.left == pytest.approx(0.5)
    assert robot.right == pytest.approx(-0.5)


# --- jog ---------------------------------------------------------------------

def test_jog_moves_a_mechanism_in_teleop(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "jog", "mech": "intake", "power": 0.3, "to": "rover1"})
    assert sim.robots["rover1"].mech_power["intake"] == pytest.approx(0.3)


def test_jog_is_refused_while_e_stopped(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "estop", "to": "rover1"})
    sim.send({"type": "jog", "mech": "intake", "power": 0.3, "to": "rover1"})
    assert sim.robots["rover1"].mech_power.get("intake", 0.0) == 0.0


def test_jog_is_refused_outside_teleop(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "mode", "mode": "waypoint", "to": "rover1"})
    sim.send({"type": "jog", "mech": "intake", "power": 0.3, "to": "rover1"})
    assert sim.robots["rover1"].mech_power.get("intake", 0.0) == 0.0


# --- presets from a bound button ---------------------------------------------
#
# Same gates as the rover applies (Robot._mech_preset), so somebody binding a
# button against the simulator learns the real behaviour rather than a friendlier
# one that only exists here.

def test_a_preset_moves_a_mechanism(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "in",
              "to": "rover1"})
    assert sim.robots["rover1"].mech_power["intake"] == pytest.approx(1.0)


def test_a_preset_is_refused_while_e_stopped(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "estop", "to": "rover1"})
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "in",
              "to": "rover1"})
    assert sim.robots["rover1"].mech_power.get("intake", 0.0) == 0.0


def test_a_preset_is_refused_while_a_routine_is_running(sim):
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    sim.robots["rover1"].mech_power["intake"] = 0.0  # whatever the routine set
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "in",
              "to": "rover1"})
    assert sim.robots["rover1"].mech_power["intake"] == 0.0


def test_a_preset_still_works_outside_teleop(sim):
    """Unlike a jog, which is a bench operation. Waypoint drives the DRIVETRAIN;
    an operator watching a rover navigate still owns the intake."""
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "mode", "mode": "waypoint", "to": "rover1"})
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "in",
              "to": "rover1"})
    assert sim.robots["rover1"].mech_power["intake"] == pytest.approx(1.0)


def test_an_unknown_preset_moves_nothing(sim):
    put(sim, LAYOUT, "put_layout")
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "sideways",
              "to": "rover1"})
    sim.send({"type": "mech_preset", "mech": "ghost", "preset": "in",
              "to": "rover1"})
    assert sim.robots["rover1"].mech_power.get("intake", 0.0) == 0.0


# --- restart -----------------------------------------------------------------

def test_a_restart_parks_everything_and_goes_quiet(sim):
    """What an operator sees is a card that goes stale and comes back; what
    matters underneath is that nothing is left running while it is down."""
    put(sim, LAYOUT, "put_layout")
    robot = sim.robots["rover1"]
    sim.send({"type": "mech_preset", "mech": "intake", "preset": "in",
              "to": "rover1"})
    robot.set_arcade(0.5, 0.0)
    sim.send({"type": "restart", "to": "rover1"})
    assert robot.mech_power["intake"] == 0.0
    assert (robot.left, robot.right) == (0.0, 0.0)
    assert robot.rebooting_until > 0


def test_a_restart_drops_a_running_routine(sim):
    """A fresh process has no routine. A simulator that kept one would teach an
    operator something untrue about what the button does."""
    put(sim, LAYOUT, "put_layout")
    put(sim, ROUTINE, "put_routines")
    sim.send({"type": "mode", "mode": "routine", "to": "rover1"})
    sim.robots["rover1"].step(0.02)
    sim.send({"type": "restart", "to": "rover1"})
    assert sim.robots["rover1"].engine is None
    assert sim.robots["rover1"].mode == sim.robots["rover1"].cfg.start_mode


def test_a_restart_clears_a_latched_estop(sim):
    """Not a policy choice — an e-stop latch lives in a process that just
    ended. Worth a test because it is the one part of a restart that makes a
    rover MORE ready to move than it was."""
    sim.send({"type": "estop", "to": "rover1"})
    sim.send({"type": "restart", "to": "rover1"})
    assert sim.robots["rover1"].estop is False


# --- the fleet cache ---------------------------------------------------------

def test_the_fleet_reassembles_what_the_simulator_sends(sim):
    fleet = FleetManager()
    put(sim, LAYOUT, "put_layout")
    sim.received.clear()
    sim.send({"type": "get_layout", "to": "rover1"})
    for frame in sim.received:
        fleet.handle(frame, time.monotonic())
    documents = fleet.documents()
    assert documents["rover1"]["layout"]["drive"]["kind"] == "servo_steer"


def test_a_verdict_reaches_the_fleet_cache(sim):
    fleet = FleetManager()
    put(sim, LAYOUT, "put_layout")
    for frame in sim.received:
        fleet.handle(frame, time.monotonic())
    result = fleet.documents()["rover1"]["layout_result"]
    assert result["ok"] is True
    assert result["restart_required"] is True


def test_doc_revs_move_when_a_document_lands(sim):
    fleet = FleetManager()
    put(sim, LAYOUT, "put_layout")
    for frame in sim.received:
        fleet.handle(frame, time.monotonic())
    assert fleet.doc_revs()["rover1"] != (0, 0, 0, 0)
