"""GPS waypoint-navigation controller (autonomy scaffold).

Drives a list of lat/lon waypoints. Sensing is injected so this logic runs
without hardware:

    pose_provider() -> (lat, lon, heading_deg) or None
        heading_deg is the robot's current heading, 0 = North, CW positive,
        or None when the heading isn't known yet.

--- No compass: heading comes from motion ---
The NEO-6M has no magnetometer; its only heading is course-over-ground, which
only exists once the robot is *moving*. Before that, pose_provider returns a
None heading. Two rules keep the robot from spinning in place forever (the
chicken-and-egg where a big heading error makes it pivot, which produces no
motion, so no course ever appears):

  1. If heading is None, drive STRAIGHT forward to build up a course, rather
     than trying to turn toward the target with an unknown heading.
  2. Once we have a heading, steer with the PID but keep the vehicle
     *translating* (an arc, never an in-place pivot), so course-over-ground —
     and therefore our heading — stays fresh.

The math (bearing + haversine distance) is real; only the sensor source is a
stub (a GY-GPS6MV2 / u-blox NEO-6M NMEA reader feeds pose_provider).
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

from .commands import DriveCommand
from .controller import Controller
from .pid import PID

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)
PoseProvider = Callable[[], Optional[Pose]]

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
        heading_pid: Optional[PID] = None,
    ):
        self.pose_provider = pose_provider
        self.waypoints: List[Tuple[float, float]] = waypoints or []
        self.arrive_radius_m = arrive_radius_m
        self.cruise_speed = cruise_speed
        # Forward throttle used to drive straight and acquire an initial heading.
        # Must be brisk enough to exceed the GPS's min_move speed so a course fixes.
        self.acquire_speed = acquire_speed
        self.heading_pid = heading_pid or PID(kp=0.2, ki=0.0, kd=0.05, out_limit=0.7)
        self._idx = 0

    def set_pose_provider(self, provider: PoseProvider) -> None:
        self.pose_provider = provider

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

        # No heading yet (GPS has no compass, and we haven't moved enough for a
        # course): drive straight to build one up instead of pivoting in place.
        if heading is None:
            self.heading_pid.reset()
            return DriveCommand.arcade(self.acquire_speed, 0.0)

        err = _heading_error_deg(bearing_deg(lat, lon, tlat, tlon), heading)
        steer = self.heading_pid.update(err, dt)
        # Keep translating while turning: clamp steer so the inner track never
        # reverses (an arc, not an in-place spin). Continuous forward motion is
        # what keeps the GPS course — and thus our heading — alive; a pivot would
        # stall the course and the robot would spin blind.
        forward = self.cruise_speed
        steer = max(-forward, min(forward, steer))
        return DriveCommand.arcade(forward, steer)
