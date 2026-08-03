"""Collision avoidance: don't drive forward into the thing in front of you.

The ultrasonic (sensors/ultrasonic.py) measures how far away the nearest object
straight ahead is. This turns that number into a limit on the drive command,
and it sits BETWEEN the active controller and the drivetrain — after teleop,
after object_align, after a routine — because "do not drive into that" is not a
belief any one mode should have to hold. Every mode gets it, including the ones
written before this file existed.

    guard = CollisionGuard(cfg.ultrasonic, sonar.distance_m)
    cmd = guard.apply(cmd)      # once per control tick, before drive()

--- what it does to the command ---
A `DriveCommand` is a left/right pair, which is the same thing as a forward
component and a turn component:

    forward = (left + right) / 2        turn = (left - right) / 2

Only FORWARD is scaled. Turning keeps its full authority, and reverse is never
touched at all. That is deliberate and it is the difference between a guard and
a trap: a rover stopped a hand's width from a wall must still be able to pivot
away from it and back out, and an operator who has just been overruled needs the
controls to still do something.

Between `slow_m` and `stop_m` the forward component is scaled down linearly,
reaching zero at `stop_m`. The run-in matters more than the stop: braking from
cruise to nothing in one tick is a lurch that the slew limiter then has to
absorb, and on a light chassis it is how you tip a rover onto its nose.

--- hysteresis, and why a latch ---
Once stopped, forward is refused until the measurement clears
`stop_m + release_m`. Without it a rover parked exactly at the threshold sees a
reading that jitters either side of it and answers with a throttle that turns on
and off every 20 ms, which sounds precisely as bad as it is.

--- how it fails ---
OPEN, always. No sensor, no reading, a reader thread that died, a module nobody
wired up: `distance_m()` answers None and this returns the command untouched.

That is the uncomfortable choice and it is made on purpose. An ultrasonic hears
nothing when the path is clear AND when it is broken — the silence is identical
— so a guard that clamped on silence would stop a rover in the middle of an
empty field and give the operator no way to tell why. Worse, it would do it from
the one component that is meant to be a backstop. So this never invents an
obstacle it cannot hear, `telemetry()` says whether the sensor is talking at
all, and the sensor logs when it has never heard an echo since start-up.

Which is the whole disclaimer, stated plainly: this reduces the collisions it
can see coming. It is not a promise that the rover cannot hit anything, and it
is no reason to drive one anywhere you would not have driven it before.
"""

from __future__ import annotations

from typing import Callable, Optional

from ..config import UltrasonicConfig
from .commands import DriveCommand

# Below this much commanded forward motion there is nothing to limit — the
# rover is stopped, pivoting, or reversing. Matched to the drivetrain's own
# sense of "commanded", not to a motor deadband, which differs per actuator.
_MOVING = 1e-3


def _clamp(v, lo=0.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


class CollisionGuard:
    """Clamps the forward part of a drive command against a measured distance.

    Holds the config OBJECT rather than a copy of its numbers, so every
    threshold is live from the dashboard with nothing to push — the same
    arrangement `Ballistics` uses, and for the same reason: these are values you
    get right by driving at a wall and watching where it stops, and a restart per
    attempt means nobody finishes the loop.
    """

    def __init__(self, config: UltrasonicConfig,
                 distance_provider: Optional[Callable[[], Optional[float]]] = None):
        self.cfg = config
        self._distance = distance_provider
        self._blocked = False       # latched; released past stop_m + release_m
        self._distance_m: Optional[float] = None   # what the last apply() saw
        self._scale = 1.0           # what it did about it
        self._warned = False

    def set_distance_provider(self, provider: Callable[[], Optional[float]]) -> None:
        self._distance = provider

    # --- the guard ----------------------------------------------------------

    def apply(self, cmd: DriveCommand) -> DriveCommand:
        """Return the command to actually drive with.

        Cheap and non-blocking: one cached lookup and a little arithmetic, which
        is why it belongs inline in the 50 Hz loop rather than on a thread the
        loop would then have to synchronize with.
        """
        self._scale = 1.0
        self._distance_m = None

        if self._distance is None or not self.cfg.enabled:
            return cmd

        distance = self._distance()
        self._distance_m = distance

        if not self.cfg.avoid:
            return cmd          # measure only: the readout without the veto
        if distance is None:
            # Nothing in range, or nothing talking. Fails open — see the module
            # docstring. The latch is deliberately NOT released here: silence is
            # not evidence the obstacle went away, and the rover has to see real
            # clearance before it drives forward again.
            return cmd

        forward = (cmd.left + cmd.right) / 2.0
        turn = (cmd.left - cmd.right) / 2.0

        self._scale = self._limit(distance)
        if forward <= _MOVING or self._scale >= 1.0:
            # Reversing, pivoting, stopped, or far enough away to be nobody's
            # business. Note the guard still updated its latch above, so backing
            # away from a wall and then driving forward again works off a fresh
            # measurement rather than a stale one.
            return cmd
        if self._scale <= 0.0 and not self._warned:
            self._warned = True
            print(f"[Collision] holding at {distance:.2f} m "
                  f"(stop is {self.cfg.stop_m:.2f} m). Reverse and steering "
                  f"still work; switch `ultrasonic.avoid` off to drive through.")
        return DriveCommand.tank(forward * self._scale + turn,
                                 forward * self._scale - turn)

    def _limit(self, distance: float) -> float:
        """How much of the commanded forward motion survives, in [0, 1]."""
        stop = self.cfg.stop_m
        if self._blocked:
            # Latched. Only real clearance past the release margin gets out of
            # it — see the hysteresis note in the module docstring.
            if distance < stop + self.cfg.release_m:
                return 0.0
            self._blocked = False
            self._warned = False
        if distance <= stop:
            self._blocked = True
            return 0.0
        slow = self.cfg.slow_m
        if distance >= slow or slow <= stop:
            # slow <= stop is a build that asked for a hard stop with no run-in;
            # everything outside `stop` is then full throttle.
            return 1.0
        return _clamp((distance - stop) / (slow - stop))

    def reset(self) -> None:
        """Drop the latch. For a drivetrain stop, and for the tests."""
        self._blocked = False
        self._warned = False
        self._scale = 1.0
        self._distance_m = None

    # --- observability ------------------------------------------------------

    @property
    def blocked(self) -> bool:
        """True while forward motion is being refused."""
        return self._blocked

    @property
    def state(self) -> str:
        """"off" | "clear" | "slow" | "stop" — what forward motion would get now.

        Reported off the last measurement rather than off the last command, so a
        rover sitting still in front of a wall says "stop" instead of "clear",
        which is what an operator about to push the stick needs to know.
        """
        if not self.cfg.enabled or not self.cfg.avoid:
            return "off"
        if self._blocked:
            return "stop"
        return "clear" if self._scale >= 1.0 else "slow"

    def status(self, sensor_telemetry: Optional[dict] = None) -> dict:
        """What the guard saw and did, for a telemetry frame.

        Takes the sensor's own summary rather than reaching for it, so the frame
        carries one `sonar` object instead of two nearly-identical ones — and so
        this stays testable without a sensor at all.
        """
        t: dict = dict(sensor_telemetry or {})
        t["state"] = self.state
        return t
