"""Shooter-align controller: face the target, settle, then fire.

This is `object_align` plus a trigger. It inherits the whole align/approach
state machine unchanged — the PID on horizontal error, the point-then-go pivot,
the latched arrival at standoff, the search sweep — and adds one question asked
once per tick: *should we shoot right now?*

Subclassing rather than copying is deliberate. The alignment logic is the part
with the subtle bugs (see object_align's module docstring on detection-stamped
PID timing), and it is already covered by tests/test_object_align.py. A forked
copy would drift out of sync with the fixes.

--- The firing sequence ---
    aligned + (arrived, when range is available)   -> hold still, start dwelling
    held continuously for `dwell` seconds          -> fire
    then `cooldown` seconds before the next shot

--- Why dwell exists ---
The detector is noisy and runs asynchronously to the control loop. A single
centered frame is not evidence the robot is pointed at anything — it's equally
consistent with a false positive, a mislabeled box, or the target crossing the
center on its way past. Requiring the alignment to *hold* costs half a second
and rejects essentially all of that. It is the difference between a robot that
shoots at its target and a robot that shoots at whatever flickers.

--- Why we command a stop while dwelling ---
Even a well-aligned robot is still creeping or trimming its heading. Firing
mid-motion throws off the aim and, on a spring mechanism, can shove the whole
chassis. So once the fire conditions are met the controller overrides the
alignment command with a stop: align -> settle -> shoot.

--- Safety ---
Three independent gates, because an actuator that launches something is the one
part of this stack where a stuck bit is dangerous rather than merely annoying:

  1. Arming is explicit (`arm_shooter`) and is dropped on mode exit and on
     e-stop. It can never be inherited from a previous run or a previous mode.
  2. The dwell timer resets whenever ticks stop arriving (e-stop, mode switch,
     a stalled loop). Without this, clearing an e-stop while still aligned would
     fire instantly on the very next tick — the dwell would have been "held" for
     the entire duration of the stop, during which nothing was actually checked.
  3. The Shooter owns its mechanical cycle, so asking to fire every tick still
     yields at most one shot per cycle.
"""

from __future__ import annotations

import time
from typing import Optional, Protocol

from ..config import ShooterConfig
from .commands import DriveCommand
from .detection import Detection
from .object_align import ObjectAlignController


class ShooterLike(Protocol):
    """The slice of robot.drive.shooter.Shooter this controller depends on.

    Structural, not an import: controllers stay free of hardware modules (the
    same rule that keeps them out of sensors/), so this one unit-tests against a
    trivial fake and a different launcher — solenoid, relay, flywheel ESC — can
    be dropped in without touching this file.
    """

    def fire(self) -> bool: ...
    def stop(self) -> None: ...

# Ticks should arrive every 1/loop_hz (200ms at the default 5 Hz). A gap longer
# than this means the controller was not being updated — e-stop, a mode switch,
# or a stalled loop — so any alignment we were dwelling on is unverified.
_STALE_TICK_S = 0.5


class ShooterAlignController(ObjectAlignController):
    """Align to a detected object and fire once the aim holds."""

    name = "shooter_align"

    def __init__(self, shooter: Optional[ShooterLike] = None,
                 config: Optional[ShooterConfig] = None, **kwargs):
        super().__init__(**kwargs)
        cfg = config or ShooterConfig()
        self.shooter = shooter
        self.dwell = cfg.dwell
        self.cooldown = cfg.cooldown
        self.require_arm = cfg.require_arm
        self.require_arrived = cfg.require_arrived
        self.max_shots = cfg.max_shots

        self._armed = False
        self._ready_since: Optional[float] = None
        self._last_shot = 0.0
        self._last_tick = 0.0
        self._shots = 0

    def set_shooter(self, shooter: ShooterLike) -> None:
        self.shooter = shooter

    # --- operator commands ---------------------------------------------------

    def on_message(self, message: dict) -> None:
        mtype = message.get("type")
        if mtype == "arm_shooter":
            self._armed = True
            self._ready_since = None  # always dwell afresh after arming
            print("[shooter_align] ARMED")
        elif mtype == "disarm_shooter":
            self.disarm()
        elif mtype == "fire":
            # Manual override: skips alignment (that's the point of a manual
            # trigger) but never skips arming or the mechanical cycle.
            if self._fire_allowed():
                self._fire()
            else:
                print("[shooter_align] fire ignored (disarmed, cooling down, or empty)")

    def disarm(self) -> None:
        """Drop the arm latch and park the mechanism. Safe to call repeatedly."""
        was = self._armed
        self._armed = False
        self._ready_since = None
        if self.shooter is not None:
            self.shooter.stop()
        if was:
            print("[shooter_align] disarmed")

    def on_activate(self) -> None:
        super().on_activate()
        # Entering the mode never arms. The operator arms deliberately, after
        # seeing the robot is pointed somewhere sane.
        self._armed = False
        self._ready_since = None
        self._last_tick = 0.0

    def on_deactivate(self) -> None:
        super().on_deactivate()
        self.disarm()

    def on_estop(self) -> None:
        # Clearing the e-stop must require a deliberate re-arm; it is not a
        # "resume". The dwell gap check would already block an instant shot, but
        # dropping the latch is the guarantee that doesn't depend on timing.
        self.disarm()

    # --- telemetry -----------------------------------------------------------

    def armed(self) -> bool:
        return self._armed

    def shots(self) -> int:
        return self._shots

    def status(self) -> dict:
        """Compact shooter state for the telemetry frame (57600 baud — keep it small)."""
        now = time.monotonic()
        return {
            "armed": self._armed,
            "shots": self._shots,
            "ready": self._ready_since is not None,
            "cool": round(max(0.0, self.cooldown - (now - self._last_shot)), 2),
        }

    # --- the trigger ---------------------------------------------------------

    def update(self, dt: float) -> Optional[DriveCommand]:
        cmd = super().update(dt)

        if self.shooter is None:
            return cmd  # no launcher wired up: behaves exactly like object_align

        now = time.monotonic()
        # Gap detection (safety gate 2 in the module docstring). Also catches the
        # very first tick, where _last_tick is 0.0.
        if self._last_tick and (now - self._last_tick) > _STALE_TICK_S:
            self._ready_since = None
        self._last_tick = now

        if not self._on_target():
            self._ready_since = None
            return cmd

        # On target: stop and settle, regardless of what alignment wanted to do.
        if self._ready_since is None:
            self._ready_since = now
        if (now - self._ready_since) >= self.dwell and self._fire_allowed():
            self._fire()
        return DriveCommand.stopped()

    def _on_target(self) -> bool:
        """True when the aim is good enough to start (or continue) dwelling."""
        d = self.last_detection()
        if d is None or not self.aligned():
            # aligned() is only True on a fresh, centered detection, so this
            # covers lost/stale targets too. Belt and braces.
            return False
        if self._needs_arrival(d):
            return self.arrived()
        return True

    def _needs_arrival(self, d: Detection) -> bool:
        """Whether standoff must also be reached before firing.

        Skipped when the detection carries no size: FOMO models report centroids
        with fixed cell-sized boxes, so arrival can never latch and requiring it
        would mean a robot that aligns perfectly and never shoots. There we fire
        on bearing alone — which is the best this class of model supports.
        """
        return self.require_arrived and self.approach and d.size is not None

    def _fire_allowed(self) -> bool:
        if self.shooter is None:
            return False  # no launcher wired up (also guards the manual `fire`)
        if self.require_arm and not self._armed:
            return False
        if self.max_shots and self._shots >= self.max_shots:
            return False
        return (time.monotonic() - self._last_shot) >= self.cooldown

    def _fire(self) -> None:
        if self.shooter is None or not self.shooter.fire():
            return  # not wired up, or still cycling; try again next tick
        self._shots += 1
        self._last_shot = time.monotonic()
        # Re-dwell before the next shot rather than rapid-firing the instant the
        # cooldown expires: the recoil/mechanism may well have moved the robot.
        self._ready_since = None
