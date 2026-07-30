"""The simulated rover's wheel encoders, and the loop that closes around them.

Same standard as test_simulator_docs.py: a feature you can only try on a real
rover is a feature that ships broken. So the simulated robot has the exact
defect encoders exist to fix — its right side is weaker than its left — and
these tests are the proof that switching the mode on straightens it, using the
REAL RpmTrim with the REAL shipped gains.

That makes this file two things at once: a check on the simulator, and the only
place the shipped default gains are measured against a plant.
"""

import pytest

from basestation.fleet import FleetManager
from basestation.simulator import SIM_MAX_RPM, SIM_RIGHT_GAIN, _SimRobot

DT = 0.02


def drive_straight(mode: str, seconds: float = 8.0, throttle: float = 0.6):
    r = _SimRobot("rover1", 37.0, -122.0, heading=0.0)
    r.cfg.drive.trim.mode = mode
    for _ in range(int(seconds / DT)):
        r.set_arcade(throttle, 0.0)
        r.step(DT)
    return r


def drift(r) -> float:
    """How far off the commanded straight line it ended up, in degrees."""
    return abs((r.heading + 180) % 360 - 180)


def gap(r) -> float:
    return abs(r.wheel_rpm["left"] - r.wheel_rpm["right"])


def test_the_simulated_rover_curves_when_told_to_go_straight():
    """Not a bug in the simulator — the point of it. Without this defect the
    feature would have nothing to demonstrate and nothing to test against."""
    r = drive_straight("off")
    assert gap(r) == pytest.approx(SIM_MAX_RPM * 0.6 * (1 - SIM_RIGHT_GAIN),
                                   abs=1.0)
    assert drift(r) > 5.0


def test_match_mode_straightens_it_with_the_shipped_gains():
    """The headline claim, measured. If a future gain change breaks this, it
    breaks here rather than in a car park."""
    assert gap(drive_straight("match")) < 1.0
    assert drift(drive_straight("match")) < 2.0


def test_velocity_mode_holds_the_speed_the_throttle_asked_for():
    """The extra thing calibration buys: not just equal, but a known speed."""
    r = drive_straight("velocity")
    assert gap(r) < 1.0
    assert r.wheel_rpm["left"] == pytest.approx(0.6 * SIM_MAX_RPM, abs=2.0)


def test_a_wrong_max_rpm_is_visible_rather_than_silently_absorbed():
    """velocity mode is only as truthful as that one number, so the simulated
    hardware must NOT agree with whatever you type — otherwise the calibration
    the mode depends on would be untestable."""
    r = _SimRobot("rover1", 37.0, -122.0)
    r.cfg.drive.trim.mode = "velocity"
    r.cfg.drive.trim.max_rpm = SIM_MAX_RPM / 2  # half the truth
    for _ in range(400):
        r.set_arcade(0.6, 0.0)
        r.step(DT)
    # It holds the speed it was ASKED for, which is now the wrong one.
    assert r.wheel_rpm["left"] < 0.6 * SIM_MAX_RPM


def test_the_reported_speed_lags_the_real_one():
    """The fake encoder has the same two lags the real one does — a measurement
    window and a filter. Without them the simulator would bless gains that make
    a real rover hunt."""
    r = _SimRobot("rover1", 37.0, -122.0)
    r.set_arcade(1.0, 0.0)
    r.step(DT)
    assert r.wheel_rpm["left"] > 0
    assert r.meas_rpm["left"] < r.wheel_rpm["left"]


def test_an_estop_releases_the_loop():
    r = drive_straight("match")
    r.estop = True
    r.step(DT)
    assert r.trim.engaged is False


# --- the wire ----------------------------------------------------------------

def test_encoder_telemetry_reaches_the_browser_snapshot():
    """fleet.py has to carry it in TWO places — the dataclass and snapshot() —
    and forgetting the second is a silent drop the robot cannot detect."""
    r = drive_straight("match", seconds=1.0)
    fleet = FleetManager()
    fleet.handle(r.telemetry(), now=1.0)
    robot = fleet.snapshot(now=1.0)["robots"][0]
    assert set(robot["enc"]["rpm"]) == {"left", "right"}
    assert robot["enc"]["mode"] == "match"


def test_the_loops_trace_rides_the_same_switch_as_the_other_graphs():
    """Keyed by its tuning path, because that is what the settings page matches
    a graph to its gains with (RobotSettings.tsx::loopIn)."""
    r = _SimRobot("rover1", 37.0, -122.0)
    r.cfg.drive.trim.mode = "match"
    r.cfg.nav.pid_trace = True
    for _ in range(100):
        r.set_arcade(0.6, 0.0)
        r.step(DT)
    assert "drive.trim.pid" in r.telemetry()["pid"]

    r.cfg.nav.pid_trace = False
    assert "pid" not in r.telemetry()
