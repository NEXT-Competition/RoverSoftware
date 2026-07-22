"""Tests for the shooter_align trigger logic.

These assert the *safety* properties, which is the whole reason the firing rules
live in a controller and not in the UI. Every one of these is a scenario you do
not want to discover on a rover with a loaded launcher: firing at a target it
only glimpsed, firing the instant an e-stop is cleared, firing while still
turning, or emptying the magazine in one tick.

The alignment behaviour underneath is covered by tests/test_object_align.py —
this file only exercises what shooter_align adds.

    pytest tests/
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from robot.config import ShooterConfig
from robot.control.detection import Detection
from robot.control.shooter_align import ShooterAlignController


class FakeShooter:
    """Records shots and can pretend to be mid-cycle (ShooterLike protocol)."""

    def __init__(self, busy: bool = False):
        self.shots = 0
        self.stops = 0
        self.busy = busy

    def fire(self) -> bool:
        if self.busy:
            return False
        self.shots += 1
        return True

    def stop(self) -> None:
        self.stops += 1


def det(error_x: float = 0.0, size: Optional[float] = 0.9,
        stamp: Optional[float] = None) -> Detection:
    """Centered and past standoff by default, i.e. every fire gate satisfied."""
    return Detection(
        label="target",
        confidence=0.9,
        error_x=error_x,
        error_y=0.0,
        size=size,
        stamp=time.monotonic() if stamp is None else stamp,
    )


def make(shooter=None, armed=True, **cfg_kw):
    """Activated controller with a settable detection and a fake shooter."""
    cfg = ShooterConfig(**cfg_kw)
    box: Dict[str, Optional[Detection]] = {"d": None}
    s = shooter if shooter is not None else FakeShooter()
    c = ShooterAlignController(
        shooter=s,
        config=cfg,
        detection_provider=lambda: box["d"],
        standoff_size=0.45,
    )
    c.on_activate()
    if armed:
        c.on_message({"type": "arm_shooter"})
    return c, box, s


def tick(c, n=1, dt=0.02):
    """Run n control ticks, returning the last drive command."""
    cmd = None
    for _ in range(n):
        cmd = c.update(dt)
    return cmd


# --- dwell -------------------------------------------------------------------

def test_does_not_fire_on_a_single_aligned_frame():
    """The core anti-false-positive rule: one centered frame is not enough."""
    c, box, s = make(dwell=0.5)
    box["d"] = det()
    tick(c)
    assert s.shots == 0


def test_fires_once_alignment_is_held_for_the_dwell():
    c, box, s = make(dwell=0.05)
    box["d"] = det()
    tick(c)
    assert s.shots == 0
    time.sleep(0.06)
    box["d"] = det()  # fresh stamp, still centered
    tick(c)
    assert s.shots == 1


def test_losing_the_target_resets_the_dwell():
    """Half a dwell here plus half there must not add up to a shot."""
    c, box, s = make(dwell=0.05)
    box["d"] = det()
    tick(c)
    box["d"] = None  # target lost
    tick(c)
    time.sleep(0.06)
    box["d"] = det()
    tick(c)  # dwell restarts from here, so still no shot
    assert s.shots == 0


def test_drifting_off_center_resets_the_dwell():
    c, box, s = make(dwell=0.05, require_arrived=False)
    box["d"] = det(error_x=0.0)
    tick(c)
    box["d"] = det(error_x=0.5)  # swung well off target
    tick(c)
    time.sleep(0.06)
    box["d"] = det(error_x=0.5)
    tick(c)
    assert s.shots == 0


# --- holding still -----------------------------------------------------------

def test_holds_still_while_dwelling():
    """Align -> settle -> shoot: no creeping forward with the trigger pending."""
    c, box, _ = make(dwell=1.0, require_arrived=False)
    box["d"] = det(error_x=0.0, size=None)  # aligned, no range info
    cmd = tick(c)
    assert (cmd.left, cmd.right) == (0.0, 0.0)


# --- arming ------------------------------------------------------------------

def test_disarmed_never_fires():
    c, box, s = make(armed=False, dwell=0.0)
    box["d"] = det()
    tick(c, n=3)
    assert s.shots == 0


def test_activating_the_mode_does_not_arm():
    c, box, s = make(armed=False, dwell=0.0)
    assert not c.armed()
    box["d"] = det()
    tick(c, n=3)
    assert s.shots == 0


def test_deactivating_disarms_and_parks():
    c, _, s = make()
    assert c.armed()
    c.on_deactivate()
    assert not c.armed()
    assert s.stops >= 1


def test_estop_disarms():
    """Clearing an e-stop must not resume a shot that was pending when it hit."""
    c, box, s = make(dwell=0.0)
    box["d"] = det()
    c.on_estop()
    assert not c.armed()
    tick(c, n=3)
    assert s.shots == 0


def test_require_arm_false_fires_without_arming():
    c, box, s = make(armed=False, dwell=0.0, require_arm=False)
    box["d"] = det()
    tick(c)
    assert s.shots == 1


# --- stale ticks -------------------------------------------------------------

def test_a_gap_in_ticks_restarts_the_dwell():
    """An e-stop or mode switch freezes update(); the dwell must not 'accrue'
    across the pause, when nothing was being verified."""
    c, box, s = make(dwell=0.05)
    box["d"] = det()
    tick(c)
    time.sleep(0.6)  # longer than _STALE_TICK_S: no ticks happened in here
    box["d"] = det()
    tick(c)
    assert s.shots == 0, "dwell counted time during which nothing was checked"


# --- cooldown and magazine ---------------------------------------------------

def test_cooldown_blocks_a_second_shot():
    c, box, s = make(dwell=0.0, cooldown=10.0)
    box["d"] = det()
    tick(c, n=5)
    assert s.shots == 1


def test_max_shots_limits_the_magazine():
    c, box, s = make(dwell=0.0, cooldown=0.0, max_shots=2)
    for _ in range(6):
        box["d"] = det()
        tick(c)
    assert s.shots == 2


def test_a_busy_shooter_is_retried_not_double_counted():
    """The launcher owns its mechanical cycle; asking every tick is harmless."""
    s = FakeShooter(busy=True)
    c, box, _ = make(shooter=s, dwell=0.0, cooldown=0.0)
    box["d"] = det()
    tick(c, n=3)
    assert s.shots == 0
    assert c.shots() == 0, "a refused fire must not count against the magazine"
    s.busy = False
    box["d"] = det()
    tick(c)
    assert s.shots == 1


# --- range / FOMO ------------------------------------------------------------

def test_waits_for_arrival_when_range_is_available():
    c, box, s = make(dwell=0.0, require_arrived=True)
    box["d"] = det(size=0.1)  # centered but far short of standoff_size=0.45
    tick(c, n=3)
    assert s.shots == 0


def test_fires_on_bearing_alone_when_the_model_gives_no_size():
    """FOMO: arrival can never latch, so requiring it would mean never firing."""
    c, box, s = make(dwell=0.0, require_arrived=True)
    box["d"] = det(size=None)
    tick(c)
    assert s.shots == 1


# --- manual trigger ----------------------------------------------------------

def test_manual_fire_skips_alignment_but_not_arming():
    c, _, s = make()
    c.on_message({"type": "fire"})
    assert s.shots == 1


def test_manual_fire_is_ignored_when_disarmed():
    c, _, s = make(armed=False)
    c.on_message({"type": "fire"})
    assert s.shots == 0


# --- degraded wiring ---------------------------------------------------------

def test_without_a_shooter_it_behaves_like_object_align():
    """No launcher on this build: align normally, never crash on the trigger."""
    box: Dict[str, Optional[Detection]] = {"d": det(error_x=0.8, size=0.1)}
    c = ShooterAlignController(detection_provider=lambda: box["d"])
    c.on_activate()
    c.on_message({"type": "arm_shooter"})
    cmd = c.update(0.02)
    # Far off center -> a pure pivot, exactly as object_align would produce.
    assert cmd.left == -cmd.right
    assert cmd.left != 0.0
    c.on_message({"type": "fire"})  # must not raise
