"""WaypointController: the two heading loops and how they differ.

The interesting cases are all about WHERE the heading came from. An IMU heading
is an absolute attitude, fresh every control tick and valid at a standstill. A
GPS heading is a course over ground: it refreshes at the fix rate (1 Hz by
default, against a 50 Hz loop) and only while the antenna is actually moving.
Feed the second one to a loop tuned for the first and the rover weaves.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot.control.waypoint import (  # noqa: E402
    WaypointController,
    _heading_error_deg,
    bearing_deg,
    haversine_m,
)

DT = 0.02  # 50 Hz, the real loop rate

# Two points ~110 m apart, due North of each other.
HERE = (37.0, -122.0)
NORTH = (37.001, -122.0)


def make(heading, absolute=True, rate=None, **kw):
    """A controller whose pose is a mutable dict the test can steer."""
    box = {"lat": HERE[0], "lon": HERE[1], "heading": heading, "abs": absolute,
           "rate": rate}
    c = WaypointController(
        pose_provider=lambda: (box["lat"], box["lon"], box["heading"]),
        waypoints=[NORTH],
        absolute_heading_provider=lambda: box["abs"],
        heading_rate_provider=(lambda: box["rate"]) if rate is not None else None,
        **kw,
    )
    c.on_activate()
    return c, box


# --- the geometry, which is source-independent -----------------------------


def test_bearing_and_distance():
    assert bearing_deg(*HERE, *NORTH) == pytest.approx(0.0, abs=0.01)
    assert haversine_m(*HERE, *NORTH) == pytest.approx(111.0, rel=0.02)


@pytest.mark.parametrize("target,current,expected", [
    (10, 350, 20), (350, 10, -20), (0, 90, -90), (90, 0, 90),
])
def test_heading_error_wraps_to_shortest_turn(target, current, expected):
    assert _heading_error_deg(target, current) == pytest.approx(expected)


def test_heading_error_stays_in_range_at_the_antipode():
    """180° away is a tie; either way round is right, but it must not wrap out."""
    assert abs(_heading_error_deg(180, 0)) == pytest.approx(180)


# --- IMU heading: point-then-go, unchanged ---------------------------------


def test_imu_heading_pivots_in_place():
    """Facing 90° off with an absolute heading: spin, don't drive."""
    c, _ = make(heading=90.0, absolute=True)
    cmd = c.update(DT)
    assert cmd.left + cmd.right == pytest.approx(0.0)  # pure pivot
    assert cmd.left < 0  # target is to the LEFT (bearing 0 from heading 90)


def test_imu_heading_cruises_once_aligned():
    c, _ = make(heading=5.0, absolute=True)
    cmd = c.update(DT)
    assert cmd.left > 0 and cmd.right > 0  # driving forward
    assert cmd.left + cmd.right == pytest.approx(2 * c.cruise_speed, abs=1e-6)


# --- GPS heading: never pivot, or the track angle freezes ------------------


def test_gps_heading_arcs_instead_of_pivoting():
    """A pivot doesn't move the antenna, so the course would never update.

    The rover must keep translating through the turn — that's the whole reason
    acquire_speed exists.
    """
    c, _ = make(heading=90.0, absolute=False)
    cmd = c.update(DT)
    assert cmd.left + cmd.right == pytest.approx(2 * c.acquire_speed, abs=1e-6)
    assert cmd.left < cmd.right  # still turning left, just while moving


def test_gps_loop_is_slower_than_the_imu_loop():
    """Same error, same tick: the GPS course gets less steering authority."""
    imu, _ = make(heading=45.0, absolute=True)
    gps, _ = make(heading=45.0, absolute=False)
    steer_imu = imu.update(DT).right - imu.update(DT).left
    steer_gps = gps.update(DT).right - gps.update(DT).left
    assert steer_gps < steer_imu
    assert gps.gps_heading_pid.kp < imu.heading_pid.kp


def test_switching_heading_source_resets_both_loops():
    """The IMU dropping out mid-route must not hand the GPS loop stale state."""
    c, box = make(heading=45.0, absolute=True, rate=10.0)
    for _ in range(50):
        c.update(DT)
    assert abs(c.heading_pid._integral) > 0.0
    box["abs"] = False
    c.update(DT)
    assert c.heading_pid._integral == 0.0
    # The GPS loop starts from scratch: one tick's worth of error, not the 50
    # ticks the IMU loop had accumulated.
    assert abs(c.gps_heading_pid._integral) == pytest.approx(45 * DT)


# --- the stale-heading hold ------------------------------------------------


def test_stale_heading_holds_the_output_with_no_gyro():
    """50 ticks, one heading sample: the PID must step once, not fifty times.

    Without this, the same stale error is integrated 50x and the finite-
    difference derivative alternates between zero and a one-tick spike.
    """
    c, _ = make(heading=45.0, absolute=False)
    first = c.update(DT)
    for _ in range(49):
        assert c.update(DT) == first  # held flat between fixes
    # One sample's worth of integration, not fifty.
    assert c.gps_heading_pid._prev_error is not None


def test_fresh_heading_sample_steps_the_loop():
    """A new fix, one realistic second later, moves the output again.

    The elapsed second is the point: the PID sees dt=1.0, so the derivative of a
    25° swing reads as a 25 deg/s turn rate rather than the 1250 deg/s a 50 Hz
    finite difference would have invented.
    """
    c, box = make(heading=45.0, absolute=False)
    held = c.update(DT)
    for _ in range(49):
        c.update(DT)  # one second of the same 1 Hz fix
    box["heading"] = 20.0  # a new fix: smaller error, and closing on it
    stepped = c.update(DT)
    assert stepped != held
    assert abs(stepped.left - stepped.right) < abs(held.left - held.right)


def test_gyro_rate_lets_the_loop_run_every_tick():
    """A measured yaw rate is fresh even when the heading is stale."""
    c, _ = make(heading=45.0, absolute=False, rate=30.0)
    c.update(DT)
    before = c.gps_heading_pid._integral
    c.update(DT)
    assert c.gps_heading_pid._integral != before  # stepped again on the same heading


def test_a_frozen_heading_stops_being_steered_on():
    """The circle bug, at the controller: a held steer is a constant curvature.

    The zero-order hold is right between two fixes and wrong forever. Once the
    heading has not moved for stale_heading_s, steering on it is open-loop — and
    an open-loop steer drives the rover round and round, which (being slow and
    turning) is exactly what stops the course from ever refreshing.
    """
    c, _ = make(heading=45.0, absolute=False, stale_heading_s=1.0)
    turning = c.update(DT)
    assert turning.left != turning.right  # steering on the fresh sample

    for _ in range(100):  # two seconds of the same heading
        cmd = c.update(DT)
    assert cmd.left == pytest.approx(cmd.right)  # straight, not a circle
    assert cmd.left == pytest.approx(c.acquire_speed)  # and moving, to re-acquire


def test_a_frozen_heading_expires_even_with_a_live_gyro():
    """A yaw rate keeps the loop STEPPING; it doesn't make the heading real.

    The gyro path resets the "when did the PID last step" clock every tick, so
    heading freshness has to be tracked on its own — otherwise a rover with an
    IMU for rate and a GPS for heading never notices the heading died.
    """
    c, _ = make(heading=45.0, absolute=False, rate=30.0, stale_heading_s=1.0)
    for _ in range(100):
        cmd = c.update(DT)
    assert cmd.left == pytest.approx(cmd.right)


def test_a_fresh_sample_revives_the_loop():
    """Expiry is not a latch — the next real course steers again."""
    c, box = make(heading=45.0, absolute=False, stale_heading_s=1.0)
    for _ in range(100):
        c.update(DT)
    box["heading"] = 40.0
    cmd = c.update(DT)
    assert cmd.left != cmd.right


def test_a_frozen_absolute_heading_stops_the_rover():
    """An IMU that died mid-pivot must not leave the rover spinning."""
    c, _ = make(heading=90.0, absolute=True, stale_heading_s=1.0)
    spinning = c.update(DT)
    assert spinning.left == pytest.approx(-spinning.right)  # pivoting in place
    assert spinning.left != 0.0

    for _ in range(100):
        cmd = c.update(DT)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


# --- unchanged failsafes ---------------------------------------------------


def test_no_heading_drives_straight_to_acquire_a_course():
    c, _ = make(heading=None, absolute=False)
    cmd = c.update(DT)
    assert cmd.left == pytest.approx(c.acquire_speed)
    assert cmd.right == pytest.approx(c.acquire_speed)


def test_no_fix_stops():
    c = WaypointController(pose_provider=lambda: None, waypoints=[NORTH])
    c.on_activate()
    assert c.update(DT).left == 0.0


def test_arrival_advances_and_finishes_the_route():
    c, box = make(heading=0.0, absolute=True)
    box["lat"], box["lon"] = NORTH
    assert c.update(DT).left == 0.0  # one stopped tick between legs
    assert c.route_done()


def test_empty_route_is_done():
    c = WaypointController(pose_provider=lambda: (*HERE, 0.0), waypoints=[])
    assert c.route_done()
