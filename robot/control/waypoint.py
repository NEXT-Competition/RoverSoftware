"""GPS waypoint-navigation controller (autonomy scaffold).

Drives a list of lat/lon waypoints. Sensing is injected so this logic runs
without hardware:

    pose_provider() -> (lat, lon, heading_deg) or None
        heading_deg is the robot's current heading, 0 = North, CW positive,
        or None when the heading isn't known yet.
    heading_rate_provider() -> yaw_rate_deg_s or None   (optional)
        the IMU gyro's yaw rate (CW positive), used as a clean measured
        derivative for the heading PID.

--- Heading: absolute from the IMU, standstill-valid ---
The BNO085 IMU gives an ABSOLUTE heading (via PoseEstimator) that's valid even at
rest, so the rover can point itself at the next waypoint before moving:

  * point-then-go: when the heading error is large, PIVOT IN PLACE toward the
    target; once roughly aligned, cruise forward with steering correction.
  * the PID derivative uses the gyro yaw-rate (derivative-on-measurement), which
    is far cleaner than finite-differencing the heading.

Fallback for when heading is unknown (no IMU calibration AND no GPS track angle
yet — rare): drive STRAIGHT forward to build up a course over ground, rather than
pivoting blind. This is also the whole story with --heading-source gps, where the
GPS track angle is the only heading and it doesn't exist until the rover moves.

The math (bearing + haversine distance) is real; the sensor source is injected
(GPS position + IMU or GPS-track-angle heading, fused by PoseEstimator).
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

from .commands import DriveCommand
from .controller import Controller
from .pid import PID

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)
PoseProvider = Callable[[], Optional[Pose]]
RateProvider = Callable[[], Optional[float]]  # () -> yaw_rate_deg_s | None

_EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _heading_error_deg(target, current) -> float:
    """Signed smallest angle from current heading to target, in [-180, 180]."""
    return (target - current + 540.0) % 360.0 - 180.0


class WaypointController(Controller):
    name = "waypoint"

    def __init__(
        self,
        pose_provider: Optional[PoseProvider] = None,
        waypoints: Optional[List[Tuple[float, float]]] = None,
        arrive_radius_m: float = 2.0,
        cruise_speed: float = 0.35,
        acquire_speed: float = 0.4,
        pivot_threshold_deg: float = 25.0,
        heading_pid: Optional[PID] = None,
        heading_rate_provider: Optional[RateProvider] = None,
    ):
        self.pose_provider = pose_provider
        self.heading_rate_provider = heading_rate_provider
        self.waypoints: List[Tuple[float, float]] = waypoints or []
        self.arrive_radius_m = arrive_radius_m
        self.cruise_speed = cruise_speed
        # Forward throttle used to drive straight and acquire an initial heading
        # (fallback only, when no absolute heading is available yet). Must be brisk
        # enough to exceed the GPS's min_move speed so a course fixes.
        self.acquire_speed = acquire_speed
        # Heading error above which we pivot in place instead of cruising-and-
        # steering. With a standstill-valid IMU heading this gives point-then-go.
        self.pivot_threshold_deg = pivot_threshold_deg
        # Gains retuned for the fast, standstill-valid IMU heading: a bit more
        # proportional bite plus a touch of integral to kill steady-state bias.
        # Final values are tuned on hardware.
        self.heading_pid = heading_pid or PID(kp=0.4, ki=0.02, kd=0.08, out_limit=0.7)
        self._idx = 0

    def set_pose_provider(self, provider: PoseProvider) -> None:
        self.pose_provider = provider

    def set_rate_provider(self, provider: RateProvider) -> None:
        self.heading_rate_provider = provider

    def on_activate(self) -> None:
        self._idx = 0
        self.heading_pid.reset()

    def on_message(self, message: dict) -> None:
        # Base station can push a route: {"type":"route","waypoints":[[lat,lon],...]}
        if message.get("type") == "route":
            self.waypoints = [
                (float(a), float(b)) for a, b in message.get("waypoints", [])
            ]
            self._idx = 0
            self.heading_pid.reset()

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.pose_provider is None or self._idx >= len(self.waypoints):
            return DriveCommand.stopped()

        pose = self.pose_provider()
        if pose is None:
            return DriveCommand.stopped()  # no GPS fix

        lat, lon, heading = pose
        tlat, tlon = self.waypoints[self._idx]

        if haversine_m(lat, lon, tlat, tlon) <= self.arrive_radius_m:
            self._idx += 1
            self.heading_pid.reset()
            return DriveCommand.stopped()  # pause a tick between legs

        # Fallback: no heading at all (IMU uncalibrated AND no GPS course yet).
        # Drive straight to build up a GPS course rather than pivoting blind.
        if heading is None:
            self.heading_pid.reset()
            return DriveCommand.arcade(self.acquire_speed, 0.0)

        err = _heading_error_deg(bearing_deg(lat, lon, tlat, tlon), heading)

        # Derivative-on-measurement: feed the gyro yaw-rate instead of finite-
        # differencing the heading. For a (near-)constant target bearing,
        # d(error)/dt = -yaw_rate, so pass -rate. None -> PID finite-differences.
        rate = self.heading_rate_provider() if self.heading_rate_provider else None
        derivative = -rate if rate is not None else None
        steer = self.heading_pid.update(err, dt, derivative)

        # Point-then-go: a large heading error means pivot in place to face the
        # target (forward=0 -> arcade gives left=steer, right=-steer, a spin).
        # The IMU heading stays valid while stationary, so this is safe. Once
        # roughly aligned, cruise forward and let the PID trim the heading.
        if abs(err) > self.pivot_threshold_deg:
            return DriveCommand.arcade(0.0, steer)
        return DriveCommand.arcade(self.cruise_speed, steer)
