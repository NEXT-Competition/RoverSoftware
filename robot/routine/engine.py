"""The state machine itself: which state is current, and when it changes.

The engine owns state and clocks. It does not own the radio, the config, or the
motors — it is handed a compiled `Routine` and a `RoutineContext`, and it
answers one question per tick: what should be driving right now.

Three rules do most of the work:

**At most one transition per tick.** Transitions are evaluated in the order they
were authored and the first match wins. A cycle of `always` transitions then
advances one state per 20 ms tick instead of spinning inside a single one, which
is what keeps a user-authored loop from freezing the 50 Hz control loop. It also
makes the machine's behaviour predictable to whoever drew it: one box per tick.

**`for_seconds` means continuously.** This is the launcher's dwell rule
generalized (see control/shooter_align.py). A single tick of "aligned" is
equally consistent with a false positive or the target crossing the centre on
its way past; requiring the condition to hold costs half a second and rejects
nearly all of it.

**Every state can be left.** A state carries a timeout, and the routine may
carry one too. Expiry ends the routine rather than advancing it: if a state
overran, the next one's assumptions probably don't hold either.
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional

from ..config import RoutineConfig
from .conditions import RoutineContext
from .schema import Routine, State


class RoutineEngine:
    """Runs one compiled routine against a context."""

    def __init__(self, routine: Routine, ctx: RoutineContext,
                 config: Optional[RoutineConfig] = None,
                 now: Optional[Callable[[], float]] = None):
        self.routine = routine
        self.ctx = ctx
        # Held by reference, not copied: `state_timeout_default` is read on
        # every tick a state is inheriting it, which is what lets the dashboard
        # raise it while a routine is running.
        self.cfg = config or RoutineConfig()
        self._now = now or time.monotonic
        self.state: Optional[State] = None
        self.finished = False
        self.reason = ""  # why it finished, for telemetry
        self._entered_at = 0.0
        self._started_at = 0.0
        self._events: set = set()

    # --- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        self.finished = False
        self.reason = ""
        self._started_at = self._now()
        # Counters are per-RUN, not per-engine: a routine re-run from the
        # dashboard must behave exactly as it did the first time, and a loop
        # that inherited last run's count would fall straight through.
        self.ctx.counters.clear()
        self._enter(self.routine.states.get(self.routine.start))

    def stop(self, reason: str = "stopped") -> None:
        """Leave the machine cleanly, running the current state's exit actions.

        Called on mode exit, e-stop abort, and when the routine ends. Running
        `on_exit` here is the whole reason it is a separate slot: disarming
        something must happen on the way out of a state however that happens,
        not only on a tidy transition.
        """
        if self.state is not None:
            self._run(self.state.on_exit)
        self.state = None
        self.finished = True
        self.reason = reason

    def _enter(self, state: Optional[State]) -> None:
        self.state = state
        self._entered_at = self._now()
        self._events.clear()
        if state is None:
            self.finished = True
            self.reason = "no such state"
            return
        for transition in state.transitions:
            transition.held_since = None
        self._run(state.on_enter)

    def _run(self, effects: List) -> None:
        for effect in effects:
            try:
                effect(self.ctx)
            except Exception as e:
                # An action that throws mid-routine must not take the robot
                # down. Report it and carry on; the state's drive source is
                # still in charge of the motors.
                print(f"[routine] action failed: {e}")

    # --- events -------------------------------------------------------------

    def post_event(self, name: str) -> None:
        """Latch an operator-sent event for the current state.

        Cleared on every state entry, so an event fired while the machine was
        somewhere else cannot satisfy a transition it was never meant for.
        """
        self._events.add(str(name))

    # --- the tick -----------------------------------------------------------

    def update(self, dt: float) -> Optional[State]:
        """Advance the machine and return the state that should be driving."""
        if self.finished or self.state is None:
            return None

        now = self._now()
        self.ctx.state_elapsed = now - self._entered_at
        self.ctx.events = self._events

        if self.routine.timeout > 0 and (now - self._started_at) >= self.routine.timeout:
            self.stop("routine timed out")
            return None

        state = self.state
        if state.terminal:
            self._finish_or_restart("reached " + state.id)
            return None

        target = self._first_matching_transition(state, now)
        if target is not None:
            self._run(state.on_exit)
            self._enter(self.routine.states.get(target))
            # Deliberately NOT re-evaluated this tick: one transition per tick.
            return self.state

        limit = (state.timeout if state.timeout is not None
                 else self.cfg.state_timeout_default)
        if limit > 0 and self.ctx.state_elapsed >= limit:
            self.stop(f"state {state.id!r} timed out")
            return None

        self._run(state.on_tick)
        return state

    def _first_matching_transition(self, state: State, now: float) -> Optional[str]:
        for transition in state.transitions:
            try:
                holds = bool(transition.predicate(self.ctx))
            except Exception as e:
                print(f"[routine] condition failed: {e}")
                holds = False
            if not holds:
                transition.held_since = None
                continue
            if transition.for_seconds <= 0:
                return transition.to
            if transition.held_since is None:
                transition.held_since = now
            elif (now - transition.held_since) >= transition.for_seconds:
                return transition.to
        return None

    def _finish_or_restart(self, reason: str) -> None:
        if self.routine.on_end == "restart":
            self._run(self.state.on_exit) if self.state else None
            self._started_at = self._now()
            self._enter(self.routine.states.get(self.routine.start))
            return
        self.stop(reason)

    # --- telemetry ----------------------------------------------------------

    def status(self) -> dict:
        """A summary small enough for a 57600-baud radio shared with telemetry."""
        return {
            "id": self.routine.id,
            "state": self.state.id if self.state else None,
            "t": round(self.ctx.state_elapsed, 1),
            "drive": (self.state.drive_controller or self.state.drive_source)
                     if self.state else None,
            "done": self.finished,
            "why": self.reason or None,
        }
