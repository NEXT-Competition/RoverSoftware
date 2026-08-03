"""Pose fusion: GPS position + a heading behind one pose_provider.

The waypoint controller and telemetry consume a single provider with the shape:

    pose() -> (lat, lon, heading_deg) or None
        heading_deg: 0 = North, clockwise-positive, or None when unknown.

Two sensors can answer "which way am I facing", and they fail in opposite ways:

  * The **GPS track angle** (course over ground, from the Adafruit module's
    RMC/VTG) is a true-North heading with no calibration and no declination
    correction — but it describes where the antenna is *travelling*, so it's
    noise at a standstill and blind to a pivot in place.
  * The **BNO085 IMU** gives an ABSOLUTE heading that's valid at rest — which is
    exactly when the rover needs to point itself at the next waypoint — but only
    once its fused calibration has converged.

`PoseEstimator` keeps the provider shape identical while choosing between them:

  * Position (lat, lon) always comes from the GPS — the IMU can't give position
    (no wheel encoders here, so dead-reckoning would drift). No GPS fix => None.
  * Heading follows `heading_source`:
      "auto" (default) — the IMU when it's calibrated, else the GPS track angle.
                         With the IMU absent or still calibrating, this is
                         byte-for-byte the GPS-only behavior.
      "gps"            — the track angle only; the IMU is ignored for heading
                         (it can still supply the yaw rate). This is the
                         no-IMU-needed configuration.
      "imu"            — the IMU only; heading stays None until it calibrates,
                         rather than falling back to a course over ground.

It also exposes `heading_rate()` (the IMU gyro yaw-rate) so the heading PID can
use a clean measured derivative instead of finite-differencing a noisy heading,
and `heading_is_absolute()` — whether the heading it just handed out is an
absolute attitude (IMU) or a course over ground (GPS). The waypoint controller
needs that distinction because a course over ground is only refreshed at the
GPS fix rate and only while the rover is moving, which changes both how fast the
heading loop may be pushed and whether pivoting in place is allowed at all.

Everything degrades gracefully: a None `gps` (GPS disabled) yields no position;
a None `imu` (IMU disabled) just means heading comes from the GPS track angle.
"""

from __future__ import annotations

from typing import Optional, Tuple

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)

HEADING_SOURCES = ("auto", "gps", "imu")


class PoseEstimator:
    def __init__(self, gps=None, imu=None, heading_source: str = "auto"):
        self.gps = gps
        self.imu = imu
        source = (heading_source or "auto").strip().lower()
        if source not in HEADING_SOURCES:
            # A typo here would silently pick a heading sensor you didn't mean,
            # so name it and fall back to the safe default rather than raising
            # (a bad env var must not stop the rover from booting).
            print(
                f"[Pose] unknown heading_source {heading_source!r} — using 'auto' "
                f"(one of {', '.join(HEADING_SOURCES)})"
            )
            source = "auto"
        self.heading_source = source
        # Which sensor actually answered the last pose() call. False until one
        # has: "no heading yet" is not an absolute heading.
        self._absolute = False

    def pose(self) -> Optional[Pose]:
        """Fused (lat, lon, heading_deg), or None when there's no position fix.

        Position is from the GPS; heading follows `heading_source` (see above).
        """
        gps_fix = self.gps.pose() if self.gps is not None else None
        if gps_fix is None:
            self._absolute = False
            return None  # no position -> no pose (waypoint nav needs a position)
        lat, lon, gps_heading = gps_fix

        if self.heading_source == "gps":
            self._absolute = False
            return (lat, lon, gps_heading)

        imu_heading = self.imu.heading() if self.imu is not None else None
        if self.heading_source == "imu":
            self._absolute = imu_heading is not None
            return (lat, lon, imu_heading)

        heading = imu_heading if imu_heading is not None else gps_heading
        self._absolute = imu_heading is not None
        return (lat, lon, heading)

    def heading_is_absolute(self) -> bool:
        """True when the last pose() heading was the IMU's absolute attitude.

        False means it was the GPS track angle (or there was no heading at all):
        a ~1 Hz course over ground that only exists while the rover is moving,
        and that a pivot in place cannot change. Callers use this to back off a
        heading loop that would otherwise be run 50x faster than its sensor.

        Reflects the LAST pose() call — call pose() first, then ask.
        """
        return self._absolute

    def heading_rate(self) -> Optional[float]:
        """IMU yaw rate in deg/s (CW+) for the heading PID's derivative, or None.

        Not gated on `heading_source`: even when the heading itself comes from the
        GPS, the gyro is still the cleanest derivative available, and the GPS has
        nothing to offer here.
        """
        return self.imu.yaw_rate() if self.imu is not None else None
