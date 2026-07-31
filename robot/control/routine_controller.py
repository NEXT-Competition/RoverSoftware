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

from typing import Any, Dict, Optional

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
        # The detector's live config, if this build has vision, and the target
        # label to put back when we are done borrowing it. See _apply_target.
        self._vision: Optional[Any] = None
        self._target_restore: Optional[str] = None
        # The aligning delegate's own standoff, put back when we stop borrowing
        # it. Keyed by controller because object_align and shooter_align are
        # separate instances with separate standoffs, and a routine that hands
        # one back onto the other would leave a shooter stopping where an
        # approach state was told to.
        self._standoff_restore: Dict[str, float] = {}
        # Which state's stop distance is currently applied. `_sync_delegate` runs
        # twice per tick, so without this the "I can't do that" log line would be
        # printed a hundred times a second.
        self._standoff_state: str = ""
        self._ctx = RoutineContext(controllers=self.controllers,
                                   mechanisms=self.mechanisms,
                                   allow_arm=lambda: self.cfg.allow_arm)

    # --- wiring -------------------------------------------------------------

    def set_pose_provider(self, provider) -> None:
        self._ctx.pose = provider

    def set_estop_provider(self, provider) -> None:
        self._ctx.estop = provider

    def set_ballistics(self, ballistics) -> None:
        """The distance -> flywheel-speed model, for states that work out a shot.

        The object itself, which holds the config object, which the settings
        page writes into — so a launch angle changed between shots applies to
        the next one. Optional in exactly the way vision is: a build with no
        launcher never calls this, and the `spin_up` action says so in the log
        instead of the routine failing to load.
        """
        self._ctx.ballistics = ballistics

    def set_vision_config(self, vision) -> None:
        """The detector's live config, so a state can say what to align to.

        The object itself, not a copy: the detector re-reads `target_label` on
        every frame, which is what makes a change take effect on the run that is
        already going rather than the next one. Optional — a build with no
        camera has no vision config, and a routine that names a target on one
        simply says so in the log instead of failing to load.
        """
        self._vision = vision

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
        # Hand the detector's target back on every exit path — finished,
        # stopped, timed out, e-stopped, or the mode switched out from under it.
        # This is the only place that is guaranteed to run for all of them.
        self._restore_target()
        self._restore_standoffs()
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
        # The target BEFORE the delegate is activated, not after: an alignment
        # controller resets its search timers in on_activate and then looks for
        # whatever the detector is filtering on. Setting the label second means
        # the first fraction of a second of every aiming state hunts the wrong
        # object — or locks onto it.
        self._apply_target(state.drive_target)
        self._apply_standoff(state)
        wanted = (self.controllers.get(state.drive_controller)
                  if state.drive_source == "controller" else None)
        if wanted is self._delegate:
            return
        self._release_delegate()
        if wanted is not None:
            wanted.on_activate()
            self._delegate = wanted

    def _apply_target(self, target: str) -> None:
        """Point the detector at what this state aligns to, remembering what it
        was pointed at before.

        BORROWED, not taken. The operator's own choice — from Settings, or from
        a spoken "track the cone" — is restored the moment the routine stops
        needing it, because a routine that silently re-filtered the detector for
        the rest of the match would make the NEXT thing anybody does behave
        oddly, with nothing on screen to explain why.

        `""` means "whatever is already selected", so a state that does not
        care puts the operator's value back rather than clearing it. That also
        makes every routine written before this field existed behave exactly as
        it did.

        Not pushed back to the base station as a config frame. It is transient
        and self-reverting, and a frame per state entry would spend radio
        airtime on a value that is about to be put back.
        """
        if self._vision is None:
            if target:
                print(f"[routine] cannot align to {target!r}: this build has "
                      "no vision config")
            return
        if not target:
            self._restore_target()
            return
        if self._target_restore is None:
            self._target_restore = str(getattr(self._vision, "target_label", ""))
        if getattr(self._vision, "target_label", "") != target:
            self._vision.target_label = target
            print(f"[routine] aligning to {target!r}")

    def _restore_target(self) -> None:
        if self._vision is None or self._target_restore is None:
            return
        previous, self._target_restore = self._target_restore, None
        if getattr(self._vision, "target_label", "") != previous:
            self._vision.target_label = previous
            print("[routine] target back to "
                  + (repr(previous) if previous else "any label"))

    def _apply_standoff(self, state: State) -> None:
        """Set how near the aligning delegate drives, and remember what it was.

        Borrowed exactly as the target is, and for the same reason: a routine
        that left the operator's standoff rewritten would make the next manual
        alignment stop somewhere nobody chose, with nothing on screen saying why.

        The metres are converted to a box height by the delegate, against a
        calibration that may not exist — an uncalibrated build ignores them and
        stops at its own `standoff_size`. That is deliberately not an error here:
        refusing to run the routine would ground a robot over a number that only
        affects where it pauses, and the fallback is the behaviour the state
        would have had without the field at all.
        """
        if state.id == self._standoff_state:
            return  # already applied for this state; nothing changes per tick
        name = state.drive_controller if state.drive_source == "controller" else ""
        # Duck-typed on purpose: "can this be told to stop short of something"
        # is a question about the controller's capability, not its class, and
        # `Controller` deliberately does not declare alignment concepts.
        delegate: Any = self.controllers.get(name)
        wanted = state.drive_stop_within_m

        # `standoff_m` is what makes a controller able to approach at all —
        # waypoint and teleop have no notion of stopping short of something.
        if delegate is None or not hasattr(delegate, "standoff_m"):
            if wanted > 0.0:
                print(f"[routine] state {state.id!r}: {name or 'this drive mode'} "
                      f"has no stop distance to set; ignoring stop_within_m")
            self._restore_standoffs()
            self._standoff_state = state.id
            return

        if wanted <= 0.0:
            self._restore_standoffs()
            self._standoff_state = state.id
            return

        self._standoff_restore.setdefault(
            name, float(getattr(delegate, "standoff_m", 0.0)))
        if getattr(delegate, "standoff_m", 0.0) != wanted:
            delegate.standoff_m = wanted
            print(f"[routine] state {state.id!r}: stopping within {wanted:g} m")
        self._standoff_state = state.id

    def _restore_standoffs(self) -> None:
        """Hand every borrowed standoff back. Safe to call when none was taken."""
        self._standoff_state = ""
        if not self._standoff_restore:
            return
        for name, previous in self._standoff_restore.items():
            delegate: Any = self.controllers.get(name)
            if delegate is not None:
                delegate.standoff_m = previous
        self._standoff_restore.clear()
        print("[routine] stop distance back to the operator's own")

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

    def pid_traces(self) -> Dict[str, dict]:
        """The delegate's loops, because a routine that aligns is aligning with
        the real controller — the same loop, the same gains, and the same reason
        to want a graph of it."""
        return self._delegate.pid_traces() if self._delegate is not None else {}

    def status(self) -> Optional[dict]:
        if self.engine is None:
            return {"id": self.selected, "state": None, "done": True}
        return self.engine.status()
