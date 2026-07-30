"""What the tuning graphs are drawn from.

A PID is the one part of this robot you cannot tune by watching it: "it wobbles"
does not tell you whether kp is too high or kd is doing nothing, and the loop
runs fifty times a second with every intermediate value thrown away. These tests
are about the trace being TRUE — the numbers on the graph being the numbers that
produced the motion — and about it costing nothing when nobody is watching.
"""

from __future__ import annotations

import json
import time

import pytest

from robot.config import RobotConfig
from robot.control.detection import Detection
from robot.control.object_align import ObjectAlignController
from robot.control.pid import PID
from robot.control.teleop import TeleopController
from robot.control.waypoint import WaypointController


# --- the trace tells the truth ----------------------------------------------

def test_the_terms_are_contributions_not_gains():
    """kp times the error, not kp. A graph of the gains tells you what you
    already typed; a graph of the contributions tells you which term is driving
    the output, which is the question you actually have."""
    pid = PID(kp=2.0, ki=0.0, kd=0.0, out_limit=1.0)
    pid.update(0.25, 0.1)
    trace = pid.trace()
    assert trace["e"] == pytest.approx(0.25)
    assert trace["p"] == pytest.approx(0.5)  # kp * error, not kp
    assert trace["o"] == pytest.approx(0.5)


def test_the_terms_add_up_to_the_output():
    """The reason p/i/d and the output can share one chart: they are the same
    quantity, split. If they ever stop summing, the chart is lying."""
    pid = PID(kp=1.0, ki=0.5, kd=0.1, out_limit=10.0, i_limit=10.0)
    for _ in range(5):
        pid.update(0.2, 0.1)
    t = pid.trace()
    assert t["p"] + t["i"] + t["d"] == pytest.approx(t["o"], abs=2e-3)


def test_saturation_is_reported_because_a_graph_cannot_show_it():
    """A loop pinned at its limit looks like a loop that chose its output. It is
    the one state where turning kp up changes nothing, so it has to be said."""
    pid = PID(kp=10.0, ki=0.0, kd=0.0, out_limit=0.5)
    pid.update(1.0, 0.1)
    assert pid.trace()["o"] == pytest.approx(0.5)
    assert pid.trace()["sat"] is True


def test_an_unsaturated_loop_says_nothing_about_saturation():
    """Absent rather than false: every byte here crosses a shared radio."""
    pid = PID(kp=0.1, ki=0.0, kd=0.0, out_limit=1.0)
    pid.update(0.5, 0.1)
    assert "sat" not in pid.trace()


def test_a_reset_loop_traces_zeroes_not_stale_values():
    """on_activate resets the loop. A graph that opened on the last run's
    numbers would be read as this run's."""
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    pid.update(0.4, 0.1)
    pid.reset()
    t = pid.trace()
    assert (t["e"], t["o"], t["p"]) == (0.0, 0.0, 0.0)


def test_the_trace_is_small_enough_for_the_radio():
    """~60 bytes at the telemetry rate on a 57600-baud line shared with driving.
    This is the whole reason it is behind a switch."""
    pid = PID(kp=0.123456, ki=0.02, kd=0.05, out_limit=1.0)
    pid.update(-0.6543211, 0.02)
    encoded = json.dumps({"nav.heading_pid": pid.trace(setpoint=182.4, measured=170.2)})
    assert len(encoded) < 120, encoded


# --- which loop, named by its own tuning path -------------------------------

def test_object_align_traces_its_steering_loop():
    c = ObjectAlignController(detection_provider=lambda: Detection(
        label="cone", confidence=0.9, error_x=0.5, error_y=0.0, size=0.2,
        stamp=time.monotonic()))
    c.on_activate()
    c.update(0.02)
    traces = c.pid_traces()
    # Keyed by the tuning path, so the graph and the gain fields cannot drift.
    assert "align.pid" in traces
    assert traces["align.pid"]["sp"] == 0.0  # aligned means centred, always
    assert traces["align.pid"]["e"] == pytest.approx(0.5)


def test_a_controller_with_no_loop_traces_nothing():
    assert TeleopController().pid_traces() == {}


def test_waypoint_traces_only_the_loop_that_is_steering():
    """Two loops, separate gains, one running. Reporting the idle one would put
    a frozen curve beside a live one and pay radio for it."""
    c = WaypointController(pose_provider=lambda: (37.0, -122.0, 90.0))
    c.on_activate()
    c.on_message({"type": "route", "waypoints": [[37.001, -122.0]]})
    c.update(0.02)
    traces = c.pid_traces()
    assert list(traces) == ["nav.heading_pid"]
    # Setpoint is the bearing to the leg, measurement is where it is pointing —
    # both in degrees, the units the gains are expressed in.
    assert traces["nav.heading_pid"]["sp"] == pytest.approx(0.0, abs=1.0)
    assert traces["nav.heading_pid"]["m"] == pytest.approx(90.0)


def test_waypoint_names_the_gps_loop_when_that_is_the_one_running():
    """Which loop is live is itself the useful fact: a rover you thought was
    steering on the IMU showing the GPS loop is a rover whose IMU never
    calibrated."""
    c = WaypointController(pose_provider=lambda: (37.0, -122.0, 90.0),
                           absolute_heading_provider=lambda: False)
    c.on_activate()
    c.on_message({"type": "route", "waypoints": [[37.001, -122.0]]})
    c.update(0.02)
    assert list(c.pid_traces()) == ["nav.gps_heading_pid"]


def test_a_loop_that_has_not_run_traces_nothing():
    c = WaypointController(pose_provider=lambda: (37.0, -122.0, 90.0))
    c.on_activate()
    assert c.pid_traces() == {}


# --- the switch --------------------------------------------------------------

def test_tracing_is_off_by_default():
    """It costs airtime on every frame. Tuning is a thing you do deliberately;
    racing is what the radio is for."""
    assert RobotConfig().nav.pid_trace is False


# --- through the robot and the base station ---------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    from robot.robot import Robot
    return Robot(cfg)


def test_no_trace_in_telemetry_until_it_is_switched_on(rover):
    from robot.control.commands import DriveCommand
    rover.manager.set_mode("object_align")
    rover.manager.update(0.02)
    assert "pid" not in rover._telemetry(DriveCommand.stopped())


def test_switching_it_on_puts_the_active_loop_in_telemetry(rover):
    """Live, with no restart: the switch is only useful if you can flip it while
    watching the thing you are trying to understand."""
    from robot.control.commands import DriveCommand
    rover._set_config({"config": {"nav.pid_trace": True}, "save": False})
    rover.manager.set_mode("object_align")
    rover.manager.update(0.02)
    trace = rover._telemetry(DriveCommand.stopped()).get("pid")
    assert trace is not None and "align.pid" in trace


def test_only_the_active_modes_loop_is_reported(rover):
    """Teleop runs no loop, so there is nothing to say — and saying nothing is
    what keeps the frame small when it doesn't matter."""
    from robot.control.commands import DriveCommand
    rover._set_config({"config": {"nav.pid_trace": True}, "save": False})
    rover.manager.set_mode("teleop")
    rover.manager.update(0.02)
    assert "pid" not in rover._telemetry(DriveCommand.stopped())


def test_the_base_station_forwards_the_trace_to_the_browser():
    """It has to be listed in RobotState AND snapshot() or it never arrives —
    see the note in basestation/fleet.py."""
    from basestation.fleet import FleetManager
    fleet = FleetManager()
    fleet.handle({"type": "telemetry", "from": "rover1", "mode": "waypoint",
                  "pid": {"nav.heading_pid": {"sp": 90.0, "e": -3.0, "o": 0.06,
                                              "p": 0.06, "i": 0.0, "d": 0.0}}}, 100.0)
    robot = fleet.snapshot(100.0)["robots"][0]
    assert robot["pid"]["nav.heading_pid"]["e"] == -3.0


def test_a_trace_is_dropped_the_moment_the_loop_stops():
    """Non-sticky, like the shooter and routine blocks. A curve left frozen on
    screen after the mode changed is a graph that lies."""
    from basestation.fleet import FleetManager
    fleet = FleetManager()
    fleet.handle({"type": "telemetry", "from": "rover1", "mode": "waypoint",
                  "pid": {"nav.heading_pid": {"e": -3.0}}}, 100.0)
    fleet.handle({"type": "telemetry", "from": "rover1", "mode": "teleop"}, 100.1)
    assert fleet.snapshot(100.1)["robots"][0]["pid"] is None
