"""Pose fusion: GPS position + IMU heading behind one pose_provider.

The waypoint controller and telemetry consume a single provider with the shape:

    pose() -> (lat, lon, heading_deg) or None
        heading_deg: 0 = North, clockwise-positive, or None when unknown.

Historically that was just `GPS.pose`. But the NEO-6M has no compass, so its
heading (course-over-ground) is invalid at a standstill — which is exactly when
the rover needs to point itself at the next waypoint. The BNO055 IMU supplies an
ABSOLUTE heading that's valid at rest. `PoseEstimator` keeps the provider shape
identical while swapping in the better heading source:

  * Position (lat, lon) always comes from the GPS — the IMU can't give position
    (no wheel encoders here, so dead-reckoning would drift). No GPS fix => None,
    exactly as before.
  * Heading prefers the IMU when it's calibrated (imu.heading() is not None);
    otherwise it falls back to the GPS course-over-ground. So with the IMU absent
    or still calibrating, behavior is byte-for-byte what it was before.

It also exposes `heading_rate()` (the IMU gyro yaw-rate) so the heading PID can
use a clean measured derivative instead of finite-differencing a noisy heading.

Everything degrades gracefully: a None `gps` (GPS disabled) yields no position;
a None `imu` (IMU disabled) just means heading always comes from the GPS.
"""

from __future__ import annotations

from typing import Optional, Tuple

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)


class PoseEstimator:
    def __init__(self, gps=None, imu=None):
        self.gps = gps
        self.imu = imu

    def pose(self) -> Optional[Pose]:
        """Fused (lat, lon, heading_deg), or None when there's no position fix.

        Position is from the GPS; heading is the IMU's absolute heading when it's
        calibrated, else the GPS course-over-ground.
        """
        gps_fix = self.gps.pose() if self.gps is not None else None
        if gps_fix is None:
            return None  # no position -> no pose (waypoint nav needs a position)
        lat, lon, gps_heading = gps_fix

        imu_heading = self.imu.heading() if self.imu is not None else None
        heading = imu_heading if imu_heading is not None else gps_heading
        return (lat, lon, heading)

    def heading_rate(self) -> Optional[float]:
        """IMU yaw rate in deg/s (CW+) for the heading PID's derivative, or None."""
        return self.imu.yaw_rate() if self.imu is not None else None
