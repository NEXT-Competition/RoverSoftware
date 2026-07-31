"""How fast the flywheel has to spin to drop a ball in a bucket that far away.

`Rangefinder` turns a bounding box into metres. This turns those metres into a
motor setting, so a routine can say "work out the shot and take it" instead of
carrying a hand-tuned power per distance — which is a number that stops being
true the moment somebody moves the bucket.

--- The model ---
Projectile motion at a FIXED launch angle. The ball leaves the launcher at angle
`launch_angle_deg` from a height `launch_height_m`, and has to arrive at
`target_height_m` after travelling `d` metres horizontally. Solving the standard
trajectory for the speed that puts it there:

    v² = g·d² / (2·cos²θ·(d·tanθ − Δh))          Δh = target_height − launch_height

Then the flywheel. A ball squeezed against a spinning wheel leaves at some
fraction of that wheel's surface speed — never all of it, because the contact
slips and the ball takes spin away as well as speed. That fraction is `transfer`,
and it is the one number here nobody can derive: it depends on compression, ball
condition and wheel material. So:

    surface = v / transfer
    rpm     = surface · 60 / (π · wheel_diameter_m)

--- What this deliberately does not model ---
Drag, which a tennis ball has a lot of, so real shots land SHORT of this and
`transfer` ends up absorbing the difference. Backspin lift, which pushes the
other way. The launcher's own spin-up curve — nothing here knows whether the
wheel has reached the speed it was asked for, because a flywheel on a
`PowerMechanism` has no encoder on it (robot/sensors/encoder.py is drivetrain
only). And the range estimate underneath is a bounding-box guess with its own
error, which lands on `d` before any of this runs.

So this is a starting point that gets the wheel into the right neighbourhood and
makes the remaining error ONE number to tune (`transfer`) instead of a table.
Treat the first shots on a new build as the calibration.

--- Uncalibrated means silent, not wrong ---
`max_rpm` at or below zero means nobody has told this robot how fast its
flywheel actually turns, and every method answers None. A robot that has never
been measured has no business converting a guess into a launch. The same
argument, and the same shape, as `Rangefinder.calibrated`.
"""

from __future__ import annotations

import math
from typing import Optional

from ..config import BallisticsConfig

G = 9.80665  # m/s²


class Ballistics:
    """Distance to a target -> the flywheel setting that reaches it.

    Holds the config OBJECT, not a copy of its fields, and reads them on every
    call. That is what makes the launch angle and the transfer factor live: the
    settings page writes into this same dataclass, so the next shot uses the
    number just typed rather than the one loaded at boot. Same rule as
    `RoutineEngine` and `state_timeout_default`.
    """

    def __init__(self, config: Optional[BallisticsConfig] = None):
        self.cfg = config or BallisticsConfig()

    @property
    def calibrated(self) -> bool:
        """Whether a shot can be computed at all."""
        cfg = self.cfg
        return (cfg.max_rpm > 0.0 and cfg.wheel_diameter_m > 0.0
                and cfg.transfer > 0.0)

    # --- the maths ----------------------------------------------------------

    def speed_for(self, distance_m: float) -> Optional[float]:
        """Exit speed in m/s that lands a ball `distance_m` away, or None.

        None means no speed works, which is a real answer and not a failure to
        compute: at a fixed launch angle a target higher than `d·tanθ` is above
        the steepest line the launcher can throw along, so the ball is still
        climbing when it passes the bucket no matter how hard it is hit.
        """
        cfg = self.cfg
        if distance_m <= 0.0:
            return None
        theta = math.radians(cfg.launch_angle_deg)
        cos_t = math.cos(theta)
        # A launcher pointed at (or past) the vertical has no horizontal reach
        # to solve for, and a 90° tan is an overflow rather than an answer.
        if cos_t <= 1e-6:
            return None
        rise = cfg.target_height_m - cfg.launch_height_m
        denominator = 2.0 * cos_t * cos_t * (distance_m * math.tan(theta) - rise)
        if denominator <= 0.0:
            return None  # cannot be reached on the way up at this angle
        return math.sqrt(G * distance_m * distance_m / denominator)

    def rpm_for(self, distance_m: float) -> Optional[float]:
        """Flywheel RPM for a target `distance_m` away, or None.

        None when the robot is uncalibrated, when the geometry has no solution,
        or when the answer is FASTER THAN THE WHEEL CAN TURN. That last one is
        deliberately not clamped to `max_rpm`: clamping would spin up, fire, and
        drop the ball short, which looks exactly like a miss. None is the shot
        being out of range, and the routine can take the other branch and say so.
        """
        if not self.calibrated:
            return None
        speed = self.speed_for(distance_m)
        if speed is None:
            return None
        cfg = self.cfg
        surface = speed / cfg.transfer
        rpm = surface * 60.0 / (math.pi * cfg.wheel_diameter_m)
        if rpm > cfg.max_rpm:
            return None
        return rpm

    def power_for(self, rpm: float) -> Optional[float]:
        """The 0..1 throttle that holds the flywheel at `rpm`, or None.

        A linear map through `max_rpm`, which is what an open-loop ESC gives you
        and is honest about being approximate — there is no flywheel encoder to
        close the loop with. `idle_power` is the floor: below it a brushless ESC
        may not commutate at all, so a very short shot asking for 4% throttle
        would leave the wheel stalled rather than turning slowly.
        """
        cfg = self.cfg
        if not self.calibrated or rpm <= 0.0:
            return None
        power = rpm / cfg.max_rpm
        if power > 1.0:
            return None
        return max(cfg.idle_power, min(1.0, power))

    def shot_for(self, distance_m: float) -> Optional[tuple]:
        """(rpm, power) for a target `distance_m` away, or None if unreachable.

        The one call an action wants: both numbers come from the same distance,
        and computing them in two steps invites a caller that uses a stale one.
        """
        rpm = self.rpm_for(distance_m)
        if rpm is None:
            return None
        power = self.power_for(rpm)
        if power is None:
            return None
        return (rpm, power)

    def in_range(self, distance_m: Optional[float]) -> bool:
        """Whether a shot at this distance is one this launcher can actually
        take. False for None, so an unmeasured range is never a green light."""
        if distance_m is None:
            return False
        return self.shot_for(distance_m) is not None
