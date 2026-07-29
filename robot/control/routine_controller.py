"""The `routine` mode: run a UI-authored state machine.

This is a Controller like any other — it returns a `DriveCommand` each tick and
knows nothing about radios or hardware. What makes it different is that it
mostly doesn't decide the command itself: a state can name another controller,
and this one hands the tick to it.

--- delegation, and who owns a delegate's lifecycle ---
`ControlManager` activates exactly one controller and deactivates the last one.
While `routine` is the active mode, every other controller is inactive as far as
the manager is concerned — but a routine state that says "drive with
object_align" needs that controller running, with its search timers reset on
entry the same as if the operator had switched to it.

So this controller owns the delegate's lifecycle: `on_activate` when a
delegating state is entered, `on_deactivate` when it is left. It holds the SAME
controller instances the manager does, which is what makes delegation free —
`Robot.__init__` already injected their detection and pose providers.

Two things are deliberately not forwarded:

  * `on_estop` — the manager already broadcasts to every controller in the dict,
    delegates included. Forwarding again would disarm twice.
  * driving during an e-stop — the manager returns `stopped()` before this
    controller is even asked, so a routine cannot drive through a latch.
"""

from __future__ import annotations

from typing import Dict, Optional

from ..config import RoutineConfig
from ..routine.conditions import RoutineContext
from ..routine.engine import RoutineEngine
from ..routine.schema import Routine, State
from .commands import DriveCommand
from .controller import Controller


class RoutineController(Controller):
    name = "routine"

    def __init__(self, controllers: Dict[str, Controller],
                 mechanisms: Optional[Dict[str, object]] = None,
                 config: Optional[RoutineConfig] = None):
        # The manager's own dict, not a copy: a delegate must be the instance
        # that had its providers wired, not a second one that sees nothing.
        self.controllers = controllers
        self.mechanisms: Dict[str, object] = mechanisms if mechanisms is not None else {}
        self.cfg = config or RoutineConfig()

        self.routines: Dict[str, Routine] = {}
        self.selected: Optional[str] = None
        self.engine: Optional[RoutineEngine] = None
        self._delegate: Optional[Controller] = None
        self._active = False
        self._ctx = RoutineContext(controllers=self.controllers,
                                   mechanisms=self.mechanisms,
                                   allow_arm=lambda: self.cfg.allow_arm)

    # --- wiring -------------------------------------------------------------

    def set_pose_provider(self, provider) -> None:
        self._ctx.pose = provider

    def set_estop_provider(self, provider) -> None:
        self._ctx.estop = provider

    def set_routines(self, routines: Dict[str, Routine]) -> None:
        """Install a freshly validated routine set.

        Routines hot-swap, unlike a layout — they are data, not constructors.
        But a running machine is stopped first: continuing to execute state
        `shoot` out of a document that no longer contains it is worse than
        stopping.
        """
        if self.engine is not None:
            self._end_run("routines replaced")
        self.routines = routines
        if self.selected not in routines:
            self.selected = next(iter(routines), None)

    def select(self, routine_id: str) -> bool:
        if routine_id not in self.routines:
            print(f"[routine] no routine called {routine_id!r}")
            return False
        if self.engine is not None:
            self._end_run("switched routine")
        self.selected = routine_id
        return True

    # --- Controller hooks ----------------------------------------------------

    def on_activate(self) -> None:
        self._active = True
        self._start()

    def on_deactivate(self) -> None:
        self._active = False
        self._end_run("left routine mode")

    def on_estop(self) -> None:
        """Broadcast to every controller when the latch engages.

        `abort` resets to the start state rather than pausing: clearing an
        e-stop should not resume a half-finished sequence whose assumptions
        about where the robot is are now several seconds stale.
        """
        routine = self._current_routine()
        if routine is not None and routine.on_estop == "hold":
            return
        self._end_run("e-stopped")

    def on_message(self, message: dict) -> None:
        mtype = message.get("type")
        if mtype == "select_routine":
            if self.select(str(message.get("id", ""))) and self._active:
                self._start()
        elif mtype == "routine_cmd":
            cmd = str(message.get("cmd", ""))
            if cmd == "start":
                if "id" in message:
                    self.select(str(message["id"]))
                self._start()
            elif cmd == "stop":
                self._end_run("stopped by the operator")
            elif cmd == "restart":
                self._start()
        elif mtype == "routine_event":
            if self.engine is not None:
                self.engine.post_event(str(message.get("name", "")))
        else:
            # Anything else belongs to whoever is driving — a route push, an
            # arm, a manual fire. The delegate is the active controller in every
            # sense except the manager's bookkeeping.
            if self._delegate is not None:
                self._delegate.on_message(message)

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.engine is None:
            return DriveCommand.stopped()

        # Sync BEFORE evaluating transitions, not after. Conditions like
        # `aligned` and `arrived` read state the delegate publishes from its own
        # update(), so a delegate that has not been activated has none — and a
        # transition waiting on it would never fire. Syncing first also means a
        # state entered and left in quick succession still gets its delegate's
        # lifecycle run, rather than the next state inheriting a stale one.
        if self.engine.state is not None:
            self._sync_delegate(self.engine.state)

        state = self.engine.update(dt)
        if state is None:  # the routine ended
            self._end_run(self.engine.reason or "finished")
            return DriveCommand.stopped()

        self._sync_delegate(state)
        return self._command_for(state, dt)

    # --- internals ------------------------------------------------------------

    def _current_routine(self) -> Optional[Routine]:
        return self.routines.get(self.selected) if self.selected else None

    def _start(self) -> None:
        routine = self._current_routine()
        if routine is None:
            print("[routine] nothing to run: no routine selected")
            self.engine = None
            return
        self._release_delegate()
        self.engine = RoutineEngine(routine, self._ctx, self.cfg)
        self.engine.start()
        print(f"[routine] running {routine.id!r} from state "
              f"{routine.start!r}")

    def _end_run(self, reason: str) -> None:
        if self.engine is not None:
            self.engine.stop(reason)
            print(f"[routine] {self.engine.routine.id}: {reason}")
        self.engine = None
        self._release_delegate()
        # Mechanisms are stopped whatever the reason. A routine that ends with
        # an intake still spinning is a routine that ended unsafely; Robot's
        # e-stop hook covers the latch case, and this covers all the others.
        for mech in self.mechanisms.values():
            try:
                mech.stop()
            except Exception as e:
                print(f"[routine] could not stop a mechanism: {e}")

    def _release_delegate(self) -> None:
        if self._delegate is not None:
            self._delegate.on_deactivate()
            self._delegate = None

    def _sync_delegate(self, state: State) -> None:
        wanted = (self.controllers.get(state.drive_controller)
                  if state.drive_source == "controller" else None)
        if wanted is self._delegate:
            return
        self._release_delegate()
        if wanted is not None:
            wanted.on_activate()
            self._delegate = wanted

    def _command_for(self, state: State, dt: float) -> DriveCommand:
        if state.drive_source == "controller":
            if self._delegate is None:
                return DriveCommand.stopped()
            cmd = self._delegate.update(dt)
            return cmd if cmd is not None else DriveCommand.stopped()
        if state.drive_source == "manual":
            return DriveCommand.arcade(state.drive_throttle, state.drive_steer)
        # "stop" and "hold" both mean the motors are not this state's business.
        return DriveCommand.stopped()

    # --- telemetry ------------------------------------------------------------

    def status(self) -> Optional[dict]:
        if self.engine is None:
            return {"id": self.selected, "state": None, "done": True}
        return self.engine.status()
