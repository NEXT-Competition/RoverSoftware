"""Measured shots: what the flywheel was set to, and how far the ball went.

`Ballistics` (control/ballistics.py) derives that relationship from physics and
leaves one number — `transfer` — for reality to absorb. This is the other way to
get it: don't model anything, just go and measure. Set the wheel to an angle,
shoot, walk out with a tape measure, write down the pair. Twenty minutes of that
beats any amount of trajectory algebra on a build whose ball is scuffed and
whose wheel compression nobody recorded.

    table = ShotTable([(10, 1.2), (20, 2.4), (30, 3.9), (40, 5.1)])
    table.angle_for(3.0)      # -> 25.4, interpolated
    table.angle_for(9.0)      # -> None, past the last row

--- the rows are backwards from how they are used, on purpose ---
You MEASURE angle -> distance: the angle is what you chose and the distance is
what happened. You USE distance -> angle: the range is what the world hands you
and the angle is what you have to pick. So the rows are stored the way they were
collected, and inverted here. Storing them pre-inverted would mean editing the
table required doing the inversion by hand, which is a step nobody performs
reliably at a field in the rain.

--- outside the table is None, never an extrapolation ---
The rows say what was OBSERVED. A distance past the last row has not been
observed, and the honest answer is "I don't know", not a straight line drawn off
the end of the data. A flywheel is not linear near its limits — that is where
the ESC saturates and where the ball starts slipping on the wheel — so an
extrapolation is wrong in exactly the region it would be used.

The caller refuses to spin on None (see the auto-shot path), which makes an
out-of-range shot a rover that says why rather than one that fires short. "It
never fired" is a much easier failure to read at a field than "it fired and
missed".

--- non-monotonic data is refused, loudly ---
Two rows that disagree about direction — a higher angle that threw the ball
SHORTER — cannot be inverted: there is no single angle for that distance. It
means a bad measurement, a ball that jammed, or a battery that sagged mid-set.
Being told is what lets you re-take the row; silently picking one of the two
answers is how a table ends up with a hole nobody can find.
"""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple


class ShotTable:
    """Measured (flywheel angle, distance thrown) pairs, usable in reverse."""

    def __init__(self, rows: Iterable[Sequence[float]] = ()):
        self.rows: List[Tuple[float, float]] = []
        self.problems: List[str] = []
        self._load(rows)

    def _load(self, rows: Iterable[Sequence[float]]) -> None:
        clean: List[Tuple[float, float]] = []
        for index, row in enumerate(rows):
            try:
                angle, distance = float(row[0]), float(row[1])
            except (TypeError, ValueError, IndexError):
                self.problems.append(
                    f"row {index + 1}: expected (angle, distance), got {row!r}")
                continue
            if distance <= 0:
                self.problems.append(
                    f"row {index + 1}: distance {distance:g} m is not a throw")
                continue
            clean.append((angle, distance))

        # By ANGLE, because that is the axis the measurement swept: the rows were
        # taken by winding the wheel up, and checking monotonicity in that order
        # is what makes "this row threw it shorter than the one below it" a
        # statement about the data rather than about the sort.
        clean.sort(key=lambda r: r[0])
        for (a1, d1), (a2, d2) in zip(clean, clean[1:]):
            if a1 == a2:
                self.problems.append(
                    f"angle {a1:g} appears twice ({d1:g} m and {d2:g} m) — "
                    "which one is it?")
            elif d2 <= d1:
                self.problems.append(
                    f"angle {a2:g} threw {d2:g} m but the smaller angle {a1:g} "
                    f"threw {d1:g} m — a bigger angle must throw further, so "
                    "one of these rows is a bad measurement")
        self.rows = clean if not self.problems else []

    @property
    def calibrated(self) -> bool:
        """Usable: at least two good rows, so there is something to interpolate
        BETWEEN. One row is a single observation, not a relationship."""
        return len(self.rows) >= 2 and not self.problems

    def range_m(self) -> Optional[Tuple[float, float]]:
        """The distances this table actually covers, or None if uncalibrated."""
        if not self.calibrated:
            return None
        return (self.rows[0][1], self.rows[-1][1])

    def angle_for(self, distance_m: Optional[float]) -> Optional[float]:
        """The flywheel angle that threw the ball this far, or None.

        Linear between the two rows that bracket it. None for an unmeasured
        distance, an uncalibrated table, or anything outside the rows — see the
        module docstring for why that is not extrapolated.
        """
        if distance_m is None or not self.calibrated:
            return None
        lo, hi = self.rows[0][1], self.rows[-1][1]
        if distance_m < lo or distance_m > hi:
            return None
        for (a1, d1), (a2, d2) in zip(self.rows, self.rows[1:]):
            if d1 <= distance_m <= d2:
                if d2 == d1:                      # refused at load; belt and braces
                    return a1
                t = (distance_m - d1) / (d2 - d1)
                return a1 + (a2 - a1) * t
        return None

    def explain(self, distance_m: Optional[float]) -> str:
        """Why `angle_for` said None, in a sentence fit for a log line.

        Separate from the lookup so the hot path returns a number and the
        failure path can afford to build a sentence. Every branch here is a
        different thing to go and fix, which is the whole reason to distinguish
        them: an uncalibrated table is a job for the bench, and a shot past the
        last row is a job for the driver.
        """
        if not self.rows and self.problems:
            return f"the shot table was rejected: {self.problems[0]}"
        if not self.calibrated:
            return ("the shot table has fewer than two rows, so there is "
                    "nothing to interpolate between")
        if distance_m is None:
            return "no range to the target"
        lo, hi = self.rows[0][1], self.rows[-1][1]
        if distance_m < lo:
            return (f"{distance_m:.2f} m is nearer than the closest measured "
                    f"shot ({lo:.2f} m)")
        if distance_m > hi:
            return (f"{distance_m:.2f} m is further than the longest measured "
                    f"shot ({hi:.2f} m)")
        return ""


def throttle_for_angle(cfg, angle: float) -> Optional[float]:
    """The throttle that makes `cfg`'s actuator sit at `angle` degrees.

    The exact inverse of ESCMotor.set_throttle's mapping, which is what lets a
    table measured in RAW SERVO ANGLES drive a codebase that speaks throttle.
    Everything downstream — the e-stop, the jog timeout, `inverted` — keeps
    working, because this produces an ordinary throttle rather than reaching
    past them to the pin.

    `inverted` cancels out here: asking for 50 degrees emits 50 degrees whichever
    way the flag is set, because the sign is undone before it is applied. That
    matters because the flag describes how the motor is WIRED, and a table of
    measurements should not silently change meaning when somebody rewires it.

    None when the angle is unreachable — the throw is symmetric about neutral,
    so the endpoints alone do not tell you what is reachable — or when a
    direction cap would distort the mapping. Refused rather than clamped: a
    clamped angle is a shot at a distance nobody chose.
    """
    throw = min(cfg.max_angle - cfg.neutral_angle,
                cfg.neutral_angle - cfg.min_angle)
    if throw <= 0:
        return None
    # What the mapping needs to see AFTER `inverted` has been applied...
    commanded = (angle - cfg.neutral_angle) / throw
    if abs(commanded) > 1.0:
        return None
    # ...and therefore what has to go in, which is the opposite when inverted.
    throttle = -commanded if cfg.inverted else commanded
    # The caps scale the command on the way through, so anything but 1.0 breaks
    # the inverse. Chosen by the sign of the THROTTLE, which is how set_throttle
    # chooses it — not by the sign of `commanded`, which is the other one on an
    # inverted motor and would consult the wrong cap on exactly those builds.
    # Refused loudly rather than silently emitting a smaller angle than asked.
    cap = cfg.max_forward if throttle > 0 else cfg.max_reverse
    if cap != 1.0:
        return None
    return throttle


def reachable_angles(cfg) -> Tuple[float, float]:
    """The angle range this actuator can actually emit, for an error message."""
    throw = min(cfg.max_angle - cfg.neutral_angle,
                cfg.neutral_angle - cfg.min_angle)
    return (cfg.neutral_angle - throw, cfg.neutral_angle + throw)
