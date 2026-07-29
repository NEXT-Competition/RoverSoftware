"""GPS waypoint-navigation controller (autonomy scaffold).

Drives a list of lat/lon waypoints. Sensing is injected so this logic runs
without hardware:

    pose_provider() -> (lat, lon, heading_deg) or None
        heading_deg is the robot's current heading, 0 = North, CW positive,
        or None when the heading isn't known yet.
    heading_rate_provider() -> yaw_rate_deg_s or None   (optional)
        the IMU gyro's yaw rate (CW positive), used as a clean measured
        derivative for the heading PID.
    absolute_heading_provider() -> bool                 (optional)
        whether the heading in the pose just read is an absolute IMU attitude
        (True) or a GPS course over ground (False). See below.

--- Heading: absolute from the IMU, standstill-valid ---
The BNO085 IMU gives an ABSOLUTE heading (via PoseEstimator) that's valid even at
rest, so the rover can point itself at the next waypoint before moving:

  * point-then-go: when the heading error is large, PIVOT IN PLACE toward the
    target; once roughly aligned, cruise forward with steering correction.
  * the PID derivative uses the gyro yaw-rate (derivative-on-measurement), which
    is far cleaner than finite-differencing the heading.

--- Heading: GPS-only, and why the loop has to slow down ---
Without an absolute heading the only answer to "which way am I facing" is the GPS
track angle, and it is a much worse sensor for a feedback loop: it refreshes at
the fix rate (1 Hz by default, against a 50 Hz control loop), it is a course over
ground rather than an attitude, and it does not exist at all below the GPS's
min-move speed. Three things change when `absolute_heading_provider()` says the
heading came from the GPS:

  * a SEPARATE, slower PID (`gps_heading_pid`) drives the steering — gains that
    are right for a fresh 50 Hz attitude are wild oscillation on a 1 Hz course.
  * NO pivot in place. A pivot freezes the track angle (the antenna isn't going
    anywhere), so the loop would spin blind against an error that never moves.
    Large errors become an arcing turn at `acquire_speed` instead, which keeps
    the course alive while it turns.
  * with no gyro to supply a measured derivative, the PID steps once per FRESH
    heading sample (with the real elapsed dt) and the output is held in between,
    instead of integrating the same stale error fifty times a second and turning
    the finite-difference derivative into a once-a-second spike.

Fallback for when heading is unknown (no IMU calibration AND no GPS track angle
yet): drive STRAIGHT forward to build up a course over ground, rather than
pivoting blind.

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
SourceProvider = Callable[[], bool]  # () -> heading is an absolute attitude?

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
        gps_heading_pid: Optional[PID] = None,
        absolute_heading_provider: Optional[SourceProvider] = None,
    ):
        self.pose_provider = pose_provider
        self.heading_rate_provider = heading_rate_provider
        # Defaults to "the heading is absolute", which is the pre-existing
        # behaviour for anyone injecting a bare pose_provider.
        self.absolute_heading_provider = absolute_heading_provider
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
        # Error is in DEGREES, so a gain of 0.1 saturates a 0.6 output at 6° of
        # error. These are sized to leave the whole sub-pivot-threshold band
        # (0-25°) as a real proportional region instead of bang-bang steering.
        # Final values are tuned on hardware.
        self.heading_pid = heading_pid or PID(
            kp=0.02, ki=0.002, kd=0.008, out_limit=0.6, i_limit=50.0
        )
        # The GPS-course loop: roughly half the authority and heavier damping,
        # because it is closing around a 1 Hz sensor. No integral at all — a
        # course over ground has no steady-state bias to trim out, and
        # integrating an error that only refreshes once a second just winds up.
        self.gps_heading_pid = gps_heading_pid or PID(
            kp=0.008, ki=0.0, kd=0.006, out_limit=0.4, i_limit=50.0
        )
        self._idx = 0
        # Zero-order-hold state for a stale heading (see _steer_for).
        self._absolute = True
        self._last_heading: Optional[float] = None
        self._since_sample = 0.0
        self._steer = 0.0

    def set_pose_provider(self, provider: PoseProvider) -> None:
        self.pose_provider = provider

    def set_rate_provider(self, provider: RateProvider) -> None:
        self.heading_rate_provider = provider

    def set_absolute_heading_provider(self, provider: SourceProvider) -> None:
        self.absolute_heading_provider = provider

    def _reset_loop(self) -> None:
        """Clear both heading loops and the hold state (new leg, new route)."""
        self.heading_pid.reset()
        self.gps_heading_pid.reset()
        self._last_heading = None
        self._since_sample = 0.0
        self._steer = 0.0

    def on_activate(self) -> None:
        self._idx = 0
        self._reset_loop()

    def on_message(self, message: dict) -> None:
        # Base station can push a route: {"type":"route","waypoints":[[lat,lon],...]}
        if message.get("type") == "route":
            self.waypoints = [
                (float(a), float(b)) for a, b in message.get("waypoints", [])
            ]
            self._idx = 0
            self._reset_loop()

    def route_done(self) -> bool:
        """True once every leg has been reached (telemetry, and FSM routines).

        An empty route counts as done: a routine state that waits for "the route
        is finished" before moving on must not hang forever because nobody
        pushed one.
        """
        return self._idx >= len(self.waypoints)

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.pose_provider is None or self.route_done():
            return DriveCommand.stopped()

        pose = self.pose_provider()
        if pose is None:
            return DriveCommand.stopped()  # no GPS fix

        lat, lon, heading = pose
        tlat, tlon = self.waypoints[self._idx]

        if haversine_m(lat, lon, tlat, tlon) <= self.arrive_radius_m:
            self._idx += 1
            self._reset_loop()
            return DriveCommand.stopped()  # pause a tick between legs

        # Is the heading an absolute attitude, or a course over ground? This
        # decides the gains, whether pivoting is allowed, and whether the loop
        # may be stepped every tick. Read it right after pose(), which is what
        # PoseEstimator.heading_is_absolute() describes.
        absolute = (
            self.absolute_heading_provider() if self.absolute_heading_provider else True
        )
        if absolute != self._absolute:
            # Switching sensors mid-route (the IMU finishing calibration, or
            # dropping out) hands the other loop a stale integral and a stale
            # held output. Start it clean.
            self._absolute = absolute
            self._reset_loop()

        # Fallback: no heading at all (IMU uncalibrated AND no GPS course yet).
        # Drive straight to build up a GPS course rather than pivoting blind.
        if heading is None:
            self._reset_loop()
            return DriveCommand.arcade(self.acquire_speed, 0.0)

        err = _heading_error_deg(bearing_deg(lat, lon, tlat, tlon), heading)
        steer = self._steer_for(err, heading, dt, absolute)

        # Point-then-go: a large heading error means pivot in place to face the
        # target (forward=0 -> arcade gives left=steer, right=-steer, a spin).
        # The IMU heading stays valid while stationary, so this is safe.
        #
        # On a GPS course it is NOT safe: a pivot doesn't move the antenna, so
        # the track angle freezes and the loop spins against an error that never
        # updates. Arc at acquire_speed instead — the same throttle that is
        # already specified to out-run the GPS min-move threshold — so the
        # course keeps refreshing all the way round the turn.
        if abs(err) > self.pivot_threshold_deg:
            return DriveCommand.arcade(0.0 if absolute else self.acquire_speed, steer)
        return DriveCommand.arcade(self.cruise_speed, steer)

    def _steer_for(self, err: float, heading: float, dt: float, absolute: bool) -> float:
        """Advance the appropriate heading loop and return the steering output.

        Derivative-on-measurement: feed the gyro yaw-rate instead of finite-
        differencing the heading. For a (near-)constant target bearing,
        d(error)/dt = -yaw_rate, so pass -rate.

        With no gyro the loop can only ever be as fresh as the heading itself.
        Running a 50 Hz loop on a 1 Hz GPS course would integrate the same stale
        error fifty times and make the finite-difference derivative alternate
        between exactly zero and a one-tick spike, so instead the PID steps once
        per fresh sample — with the true elapsed time — and the output is held
        flat in between. An IMU heading changes every tick, so this degrades
        automatically to stepping every tick.
        """
        pid = self.heading_pid if absolute else self.gps_heading_pid
        rate = self.heading_rate_provider() if self.heading_rate_provider else None

        self._since_sample += dt
        if rate is not None:
            # A measured yaw rate is fresh every tick even when the heading
            # isn't, so the loop can run at full rate off it.
            self._steer = pid.update(err, dt, -rate)
            self._since_sample = 0.0
        elif heading != self._last_heading:
            self._steer = pid.update(err, self._since_sample)
            self._since_sample = 0.0
        self._last_heading = heading
        return self._steer
