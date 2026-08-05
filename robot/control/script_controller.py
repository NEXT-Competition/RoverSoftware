"""The `script` mode: run operator-written Python.

A Controller like any other — it returns a `DriveCommand` each tick and knows
nothing about radios or hardware. What is different is where the decision comes
from: a worker thread running somebody's Python (robot/script/), which reaches
this controller through a mailbox and nothing else.

--- the tick, in order, and why that order ---
    1. apply what the script asked for since the last tick
    2. sync the delegate, if the script handed driving to another mode
    3. work out this tick's drive command
    4. publish a fresh snapshot of every sensor
    5. release anything waiting on step 1

Step 4 before step 5 is the load-bearing one. `rover.mech("kicker").pulse()`
blocks until the control loop says the command landed; if we released the
script before publishing, its very next line would read a snapshot from BEFORE
its own command and see a mechanism that has not moved. That is the race that
makes every beginner's first `pulse(); wait_until(ready)` fall straight through,
and putting the publish first is what removes it rather than documenting it.

--- what stops the motors ---
Not the script. The script is one input; this controller decides, and it
returns `DriveCommand.stopped()` whenever there is no run in progress. So a
script that finishes, crashes, is stopped, hits its time limit, or is abandoned
mid-drive all end the same way, without any of them needing a `finally:`. The
e-stop is further up still — `ControlManager` returns stopped() before this
controller is asked at all.

--- delegation ---
Identical in shape to `RoutineController`, deliberately: `rover.hand_over
("object_align")` activates the very controller instance the manager holds, with
its providers already wired, and this controller owns its lifecycle. The
detector's target and the aligning controller's standoff are BORROWED and handed
back, so a script cannot leave the operator's own settings rewritten.
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from ..config import ScriptConfig
# The same waypoint parser routines use, so a route written in Python and one
# drawn in the editor accept exactly the same shapes and refuse the same typos.
from ..routine.actions import parse_waypoints
from ..script.api import Mailbox, Rover
from ..script.runtime import ScriptError, ScriptRunner
from ..script.schema import Script
from .commands import DriveCommand
from .controller import Controller

# How many command verdicts are kept for the script to read back. Only
# `spin_for` uses one today, and a script cannot have more outstanding than the
# one it is blocked on — this is a bound, not a working set.
MAX_RESULTS = 32

# Modes a script may hand driving to. `script` itself is excluded for the
# obvious reason, and `routine` because a script that started a state machine
# that could start a script is a loop nobody wants to debug at a competition.
DELEGABLE = ("teleop", "object_align", "shooter_align", "ball_intake", "waypoint")


class ScriptController(Controller):
    name = "script"

    def __init__(self, controllers: Dict[str, Controller],
                 mechanisms: Optional[Dict[str, object]] = None,
                 config: Optional[ScriptConfig] = None):
        # The manager's own dict, not a copy — same reason RoutineController
        # takes it: a delegate must be the instance whose providers were wired.
        self.controllers = controllers
        self.mechanisms: Dict[str, Any] = mechanisms if mechanisms is not None else {}
        self.cfg = config or ScriptConfig()

        self.scripts: Dict[str, Script] = {}
        self.selected: Optional[str] = None
        self.runner: Optional[ScriptRunner] = None
        self.mailbox: Optional[Mailbox] = None
        self.last_error: str = ""
        self.last_reason: str = ""

        self._active = False
        self._drive = DriveCommand.stopped()
        self._delegate: Optional[Controller] = None
        self._delegate_name = ""
        self._results: Dict[int, bool] = {}
        self._applied_seq = 0

        # Injected by Robot. Every one optional: a build with no GPS, no
        # camera or no ultrasonic must still run scripts, with the readings
        # those sensors would have provided simply absent — the same rule the
        # routine conditions follow.
        self._pose: Optional[Callable[[], Optional[Tuple[float, float, Optional[float]]]]] = None
        self._estop: Callable[[], bool] = lambda: False
        self._sonar: Optional[Callable[[], Optional[float]]] = None
        self._gps: Optional[Callable[[], dict]] = None
        self._imu: Optional[Callable[[], dict]] = None
        self._vision: Optional[Callable[[], dict]] = None
        self._encoders: Optional[Callable[[], Optional[dict]]] = None
        self._commanded: Callable[[], Tuple[float, float]] = lambda: (0.0, 0.0)
        self._vision_cfg: Optional[Any] = None
        self._ballistics: Optional[Any] = None
        self._hfov: float = 0.0

        # Borrowed detector target / aligning standoffs, handed back on every
        # exit path. Same bookkeeping, and same reasons, as RoutineController.
        self._target_restore: Optional[str] = None
        self._standoff_restore: Dict[str, float] = {}

    # --- wiring --------------------------------------------------------------

    def set_pose_provider(self, provider) -> None:
        self._pose = provider

    def set_estop_provider(self, provider) -> None:
        self._estop = provider

    def set_sonar_provider(self, provider) -> None:
        self._sonar = provider

    def set_gps_provider(self, provider) -> None:
        self._gps = provider

    def set_imu_provider(self, provider) -> None:
        self._imu = provider

    def set_vision_provider(self, provider) -> None:
        """`() -> dict` shaped like the vision block of telemetry, distance
        included. The same summary the dashboard renders, so what a script
        reads and what an operator sees cannot disagree."""
        self._vision = provider

    def set_encoder_provider(self, provider) -> None:
        self._encoders = provider

    def set_command_provider(self, provider) -> None:
        """What the drivetrain was last actually given, after the collision
        guard. Distinct from what the script asked for, which is the point."""
        self._commanded = provider

    def set_vision_config(self, vision) -> None:
        self._vision_cfg = vision
        self._hfov = float(getattr(vision, "hfov_deg", 0.0) or 0.0)

    def set_ballistics(self, ballistics) -> None:
        self._ballistics = ballistics

    def set_scripts(self, scripts: Dict[str, Script]) -> None:
        """Install a freshly validated script set.

        A running script is stopped first, for the reason a running routine is:
        continuing to execute code out of a document that no longer contains it
        is worse than stopping.
        """
        if self.runner is not None:
            self._end_run("scripts replaced")
        self.scripts = scripts
        if self.selected not in scripts:
            self.selected = next(iter(scripts), None)

    def select(self, script_id: str) -> bool:
        if script_id not in self.scripts:
            print(f"[script] no script called {script_id!r}")
            return False
        if self.runner is not None:
            self._end_run("switched script")
        self.selected = script_id
        return True

    # --- Controller hooks ----------------------------------------------------

    def on_activate(self) -> None:
        self._active = True
        self._start()

    def on_deactivate(self) -> None:
        self._active = False
        self._end_run("left script mode")

    def on_estop(self) -> None:
        """Broadcast to every controller when the latch engages.

        The script is aborted rather than paused. Clearing an e-stop must not
        resume a program halfway through a sequence whose idea of where the
        robot is went stale while somebody walked over to it.
        """
        self._end_run("e-stopped")

    def on_message(self, message: dict) -> None:
        mtype = message.get("type")
        if mtype == "select_script":
            if self.select(str(message.get("id", ""))) and self._active:
                self._start()
        elif mtype == "script_cmd":
            cmd = str(message.get("cmd", ""))
            if cmd in ("start", "restart"):
                if cmd == "start" and "id" in message:
                    self.select(str(message["id"]))
                if self._active:
                    self._start()
                else:
                    # Refused rather than started, and this is the one place
                    # where that is the SAFER answer. A script started while
                    # another mode is driving has nothing draining its mailbox:
                    # every actuator call would block until it timed out, its
                    # drive commands would go nowhere, and it would burn to
                    # `max_runtime` — including the `stop_all` it ends with,
                    # which would never be applied. Selecting it and switching
                    # to `script` mode is the two-message form the dashboard
                    # sends, and it is the one that works.
                    print(f"[script] not running {self.selected!r}: this robot "
                          "is not in script mode — send {'type':'mode',"
                          "'mode':'script'} as well")
            elif cmd == "stop":
                self._end_run("stopped by the operator")
        elif self._delegate is not None:
            # Anything else belongs to whoever is driving. The delegate is the
            # active controller in every sense but the manager's bookkeeping.
            self._delegate.on_message(message)

    def update(self, dt: float) -> Optional[DriveCommand]:
        runner = self.runner
        if runner is None:
            return DriveCommand.stopped()

        # A finished thread is the end of the run whatever ended it — returned,
        # raised, aborted, or ran past its time limit.
        if runner.finished and not runner.running:
            self._end_run(runner.reason or "finished")
            return DriveCommand.stopped()

        mailbox = self.mailbox
        applied = self._apply_commands()
        self._sync_delegate()
        command = self._command_for(dt)
        self._publish(command)
        # Released only after the snapshot above is in place: a script blocked
        # on an actuator command must wake into a world where its own command
        # is visible. See the module docstring.
        if applied and mailbox is not None:
            self._applied_seq = applied
            mailbox.note_applied(applied)
        return command

    # --- running -------------------------------------------------------------

    def _start(self) -> None:
        script = self.scripts.get(self.selected) if self.selected else None
        if script is None:
            print("[script] nothing to run: no script selected")
            self.last_error = "no script selected"
            self.runner = None
            return
        if not self.cfg.enabled:
            print("[script] refused: scripts are disabled on this robot "
                  "(RS_SCRIPTS_ENABLED)")
            self.last_error = "scripts are disabled on this robot"
            self.runner = None
            return

        self._end_run("restarted")
        mailbox = Mailbox()
        mailbox.set_output_limit(self.cfg.output_lines)
        rover = Rover(mailbox, self._clock, self.cfg.drive_limit)
        runner = ScriptRunner(script.code, script.id, rover, mailbox,
                              max_runtime=self.cfg.max_runtime,
                              clock=self._clock)
        self.mailbox = mailbox
        self.last_error = ""
        self.last_reason = ""
        self._results = {}
        self._applied_seq = 0
        self._drive = DriveCommand.stopped()
        # A first snapshot BEFORE the thread starts, so the script's very first
        # line can read a sensor instead of an empty dict. Costs one build of a
        # dict that was going to be built 20 ms later anyway.
        self._publish(DriveCommand.stopped(), mailbox=mailbox)
        try:
            runner.start()
        except ScriptError as e:
            # A syntax error should never get here — the schema compiles every
            # script before it is installed — but a script set written straight
            # to disk, or a robot running code newer than the document, can.
            print(f"[script] {script.id}: {e}")
            self.last_error = str(e)
            self.last_reason = "error"
            self.mailbox = None
            self.runner = None
            return
        self.runner = runner
        print(f"[script] running {script.id!r}")

    def _end_run(self, reason: str) -> None:
        runner, self.runner = self.runner, None
        if runner is not None:
            runner.stop(reason)
            # Do NOT block the control loop waiting for it. A cooperative stop
            # lands within a line or two of Python; a script wedged in
            # something that ignores the trace hook would otherwise hold the
            # 50 Hz loop hostage. It is a daemon thread, and once we have
            # dropped the mailbox it cannot reach the robot at all.
            runner.join(timeout=0.0)
            self.last_reason = runner.reason or reason
            self.last_error = runner.error or self.last_error
            if runner.error:
                print(f"[script] {runner.name}: {runner.error.splitlines()[-1]}")
            else:
                print(f"[script] {runner.name}: {self.last_reason}")
        self.mailbox = None
        self._drive = DriveCommand.stopped()
        self._release_delegate()
        self._delegate_name = ""
        # Hand back everything that was borrowed, on every exit path — this is
        # the only place guaranteed to run for all of them.
        self._restore_target()
        self._restore_standoffs()
        # And stop everything that moves. A script that ended with an intake
        # still spinning is a script that ended unsafely, whether it ended by
        # finishing or by raising; Robot's e-stop hook covers the latch case
        # and this covers all the others.
        for mech in self.mechanisms.values():
            try:
                mech.stop()
            except Exception as e:
                print(f"[script] could not stop a mechanism: {e}")

    @staticmethod
    def _clock() -> float:
        return time.monotonic()

    # --- applying what the script asked for -----------------------------------

    def _apply_commands(self) -> int:
        """Drain the mailbox and act. Returns the highest sequence applied."""
        mailbox = self.mailbox
        if mailbox is None:
            return 0
        highest = 0
        for kind, payload, seq in mailbox.drain():
            highest = max(highest, seq)
            try:
                result = self._apply(kind, payload)
            except Exception as e:
                # A command that throws must not take the robot down, exactly
                # as a routine action must not. The script gets a False back
                # and the reason in its console.
                print(f"[script] command {kind!r} failed: {e}")
                mailbox.write_line(f"[script] {kind} failed: {e}")
                result = False
            if result is not None:
                self._results[seq] = bool(result)
                if len(self._results) > MAX_RESULTS:
                    for old in sorted(self._results)[:-MAX_RESULTS]:
                        self._results.pop(old, None)
            if self.runner is None:
                break  # a command ended the run; the rest is from a dead script
        return highest

    def _apply(self, kind: str, payload: Any) -> Optional[bool]:
        if kind == "drive":
            left, right = payload
            self._drive = DriveCommand.tank(left, right)
            return None
        if kind == "arcade":
            throttle, steer = payload
            self._drive = DriveCommand.arcade(throttle, steer)
            return None
        if kind == "delegate":
            self._delegate_name = str(payload or "")
            return None
        if kind == "stop_all":
            self._drive = DriveCommand.stopped()
            for mech in self.mechanisms.values():
                mech.stop()
            return None
        if kind == "target":
            self._apply_target(str(payload or ""))
            return None
        if kind == "standoff":
            name, metres = payload
            self._apply_standoff(str(name), float(metres))
            return None
        if kind == "route":
            return self._apply_route(payload)
        if kind.startswith("mech_"):
            return self._apply_mech(kind, payload)
        if kind.startswith("shooter_"):
            return self._apply_shooter(kind, payload)
        print(f"[script] ignoring unknown command {kind!r}")
        return False

    def _limited(self, command: DriveCommand) -> DriveCommand:
        """Scale a command down to `script.drive_limit`, keeping the turn.

        Scaled rather than clamped per track, for the reason `DriveCommand.
        arcade` gives: clamping each side changes the RATIO, so a limited script
        would drive a different arc than the one it asked for instead of the
        same arc more slowly.
        """
        limit = max(0.0, min(1.0, float(self.cfg.drive_limit)))
        if limit >= 1.0:
            return command
        return DriveCommand.tank(command.left * limit, command.right * limit)

    def _apply_mech(self, kind: str, payload: Any) -> bool:
        name = payload[0] if isinstance(payload, tuple) else payload
        mech = self.mechanisms.get(str(name))
        if mech is None:
            self._say(f"no mechanism named {name!r} on this robot")
            return False
        if kind == "mech_power":
            _, power, actuator = payload
            if not hasattr(mech, "set_power"):
                self._say(f"{name!r} cannot be given a power — it is a pulse "
                          "or sequence mechanism, so use .pulse()")
                return False
            return bool(mech.set_power(power, actuator))
        if kind == "mech_preset":
            _, preset = payload
            if not hasattr(mech, "apply_preset"):
                self._say(f"{name!r} has no presets")
                return False
            if not mech.apply_preset(preset):
                self._say(f"{name!r} has no preset {preset!r}")
                return False
            return True
        if kind == "mech_pulse":
            fire = getattr(mech, "fire", None) or getattr(mech, "activate", None)
            if fire is None:
                self._say(f"{name!r} has nothing to pulse — it is a power "
                          "mechanism, so use .power(...)")
                return False
            return bool(fire())
        if kind == "mech_stop":
            mech.stop()
            return True
        if kind == "mech_shot":
            _, distance, actuator = payload
            return self._apply_shot(mech, str(name), distance, actuator)
        return False

    def _apply_shooter(self, kind: str, payload: Any) -> bool:
        shooter = self.mechanisms.get("shooter")
        if shooter is None:
            self._say("this build has no shooter")
            return False
        if kind == "shooter_spin":
            on, rpm = payload
            # Duck-typed, because "shooter" is not necessarily the built-in
            # launcher: with `shooter.enabled` off, a layout is free to declare
            # a mechanism of its own by that name, and it has a mechanism's
            # verbs rather than a Shooter's. Saying so beats an AttributeError
            # in a command handler.
            if not hasattr(shooter, "spin"):
                self._say("the mechanism named 'shooter' on this build is not "
                          "the launcher — use rover.mech('shooter').power(...)")
                return False
            if on and rpm:
                shooter.set_target_rpm(float(rpm))
            shooter.spin(bool(on))
            return True
        if kind == "shooter_shot":
            return self._apply_shot(shooter, "shooter", float(payload), None)
        if kind == "shooter_fire":
            fire = getattr(shooter, "fire", None)
            if fire is None:
                self._say("the mechanism named 'shooter' cannot be fired")
                return False
            return bool(fire())
        if kind == "shooter_stop":
            shooter.stop()
            return True
        return False

    def _apply_shot(self, mech, name: str, distance: float,
                    actuator: Optional[str]) -> bool:
        """Spin at the speed a shot from this range needs, or not at all.

        The same policy the routine `spin_up` action has, and the same reason
        for it: a wheel spun at some fallback speed throws a ball a distance
        nobody chose, and "it fired but missed" is far harder to read at the
        field than "it never fired".

        Two shapes of launcher land here. The built-in `Shooter` closes a speed
        loop, so it is given the RPM and left to hold it; a layout power
        mechanism has no such loop, so it is given the throttle the model
        converted that RPM into. Both numbers come out of the same call, which
        is why one branch can serve both.
        """
        if self._ballistics is None:
            self._say("cannot work out a shot: this build has no ballistics "
                      "config")
            return False
        if distance <= 0.0:
            self._say("cannot work out a shot: no range given")
            return False
        shot = self._ballistics.shot_for(distance)
        if shot is None:
            self._say(f"no shot at {distance:.2f} m: out of {name!r}'s range "
                      "at this angle")
            return False
        rpm, power = shot
        if hasattr(mech, "set_target_rpm"):
            mech.set_target_rpm(rpm)
            mech.spin(True)
        elif hasattr(mech, "set_power"):
            mech.set_power(power, actuator)
        else:
            self._say(f"{name!r} is not a flywheel: it cannot be spun up")
            return False
        # Always logged, with all three numbers: this is the only line where
        # the range the robot BELIEVED sits next to the speed it chose and the
        # throttle it sent, which is what makes `transfer` tunable at all.
        self._say(f"shot at {distance:.2f} m: {rpm:.0f} rpm -> {power:.2f} "
                  f"throttle on {name!r}")
        return True

    def _apply_route(self, waypoints: Any) -> bool:
        points, problem = parse_waypoints(waypoints)
        if problem:
            self._say(f"bad route: {problem}")
            return False
        wp = self.controllers.get("waypoint")
        if wp is None:
            self._say("cannot set a route: this build has no waypoint mode")
            return False
        wp.on_message({"type": "route", "waypoints": [list(p) for p in points]})
        return True

    def _say(self, message: str) -> None:
        """Tell the operator, in the place they are already looking.

        Both the journal and the script console: the console is where somebody
        debugging their own code will see it, and the journal is where somebody
        debugging the rover will.
        """
        print(f"[script] {message}")
        if self.mailbox is not None:
            self.mailbox.write_line(f"[script] {message}")

    # --- delegation -----------------------------------------------------------

    def _sync_delegate(self) -> None:
        name = self._delegate_name
        wanted = self.controllers.get(name) if name in DELEGABLE else None
        if name and wanted is None:
            self._say(f"cannot hand over to {name!r} — this build has "
                      f"{', '.join(n for n in DELEGABLE if n in self.controllers)}")
            self._delegate_name = ""
        if wanted is self._delegate:
            return
        self._release_delegate()
        if wanted is not None:
            wanted.on_activate()
            self._delegate = wanted

    def _release_delegate(self) -> None:
        """Deactivate the delegate, WITHOUT forgetting which one was asked for.

        Those are two different facts, and conflating them made the delegate
        last exactly one tick: `_sync_delegate` released the old one, installed
        the new one — and cleared the name on the way through, so the next tick
        read "" as "the script wants nobody" and released it again. The name is
        the script's standing instruction and is cleared only when the script
        withdraws it or the run ends.
        """
        if self._delegate is not None:
            self._delegate.on_deactivate()
            self._delegate = None

    def _command_for(self, dt: float) -> DriveCommand:
        if self._delegate is not None:
            # NOT limited. `script.drive_limit` caps what a SCRIPT commands; a
            # delegate is one of the rover's own controllers driving with its own
            # tuned speeds, and scaling those would make `object_align` creep at
            # a different rate here than it does when the operator selects it —
            # which is the kind of difference that gets diagnosed as a broken
            # controller rather than as a setting.
            command = self._delegate.update(dt)
            return command if command is not None else DriveCommand.stopped()
        # Limited HERE rather than where the command was stored, so the setting
        # is genuinely live: a script that set a throttle and then slept for ten
        # seconds slows down when the slider moves, instead of holding the limit
        # that was in force at the moment it happened to issue the command.
        return self._limited(self._drive)

    # --- borrowed settings ----------------------------------------------------

    def _apply_target(self, target: str) -> None:
        """Point the detector at what the script is looking for, remembering
        what the operator had it pointed at. Borrowed, not taken — see
        RoutineController._apply_target for the whole argument."""
        if self._vision_cfg is None:
            if target:
                self._say(f"cannot look for {target!r}: this build has no "
                          "vision config")
            return
        if not target:
            self._restore_target()
            return
        if self._target_restore is None:
            self._target_restore = str(getattr(self._vision_cfg,
                                               "target_label", ""))
        if getattr(self._vision_cfg, "target_label", "") != target:
            self._vision_cfg.target_label = target
            self._say(f"looking for {target!r}")

    def _restore_target(self) -> None:
        if self._vision_cfg is None or self._target_restore is None:
            return
        previous, self._target_restore = self._target_restore, None
        if getattr(self._vision_cfg, "target_label", "") != previous:
            self._vision_cfg.target_label = previous

    def _apply_standoff(self, name: str, metres: float) -> None:
        delegate: Any = self.controllers.get(name)
        if delegate is None or not hasattr(delegate, "standoff_m"):
            if metres > 0.0:
                self._say(f"{name or 'that mode'} has no stop distance to set")
            return
        if metres <= 0.0:
            previous = self._standoff_restore.pop(name, None)
            if previous is not None:
                delegate.standoff_m = previous
            return
        self._standoff_restore.setdefault(
            name, float(getattr(delegate, "standoff_m", 0.0)))
        delegate.standoff_m = metres

    def _restore_standoffs(self) -> None:
        for name, previous in self._standoff_restore.items():
            delegate: Any = self.controllers.get(name)
            if delegate is not None:
                delegate.standoff_m = previous
        self._standoff_restore.clear()

    # --- what the script can see ----------------------------------------------

    def _publish(self, command: DriveCommand,
                 mailbox: Optional[Mailbox] = None) -> None:
        mailbox = mailbox or self.mailbox
        if mailbox is None:
            return
        left, right = self._commanded()
        snapshot: Dict[str, Any] = {
            "estop": bool(self._estop()),
            "drive": {"l": round(left, 3), "r": round(right, 3)},
            "asked": {"l": round(command.left, 3), "r": round(command.right, 3)},
            "delegate": self._delegate_name,
            "mech": {name: self._mech_status(m)
                     for name, m in self.mechanisms.items()},
            "results": dict(self._results),
        }
        if self._pose is not None:
            pose = self._pose()
            if pose is not None:
                snapshot["pose"] = pose
        if self._gps is not None:
            snapshot["gps"] = self._gps()
        if self._imu is not None:
            snapshot["imu"] = self._imu()
        if self._encoders is not None:
            encoders = self._encoders()
            if encoders is not None:
                snapshot["enc"] = {**encoders, "cl": round(command.left, 3),
                                   "cr": round(command.right, 3)}
        if self._vision is not None:
            vision = self._vision()
            if vision is not None:
                # The camera's field of view rides along so `vision.bearing`
                # can turn the normalised offset into degrees without the
                # script needing to know where that number is configured.
                snapshot["vision"] = {**vision, "hfov": self._hfov}
        if self._sonar is not None:
            distance = self._sonar()
            snapshot["sonar"] = {"d": distance}
        shooter = self.mechanisms.get("shooter")
        if shooter is not None:
            # `fitted` means the BUILT-IN launcher, not merely something named
            # "shooter": a script checking it before calling `rover.shooter.fire()`
            # is asking whether that call will work.
            snapshot["has_shooter"] = hasattr(shooter, "spin")
            snapshot["shooter"] = self._mech_status(shooter)
        align = self._aligner()
        if align is not None:
            snapshot["align"] = {
                "aligned": bool(align.aligned()),
                "arrived": bool(align.arrived()),
                "dist": align.distance_m(),
                "seen": align.last_detection() is not None,
            }
        wp = self.controllers.get("waypoint")
        if wp is not None:
            snapshot["route_done"] = bool(wp.route_done())
        mailbox.publish(snapshot)

    def _aligner(self):
        """Whichever alignment controller is driving, or the plain one.

        Prefers the DELEGATE, so a script that handed over to `shooter_align`
        reads that controller's opinion rather than the other instance's stale
        one — which is the difference between "am I lined up" answered by the
        loop that is actually steering and by one that stopped a minute ago.
        """
        for candidate in (self._delegate,
                          self.controllers.get("shooter_align"),
                          self.controllers.get("object_align")):
            if candidate is not None and hasattr(candidate, "aligned"):
                return candidate
        return None

    @staticmethod
    def _mech_status(mech) -> dict:
        try:
            return mech.status()
        except Exception:
            return {}

    # --- telemetry ------------------------------------------------------------

    def pid_traces(self) -> Dict[str, dict]:
        """The delegate's loops. A script that aligns is aligning with the real
        controller, so it is the real controller's graph you want."""
        return self._delegate.pid_traces() if self._delegate is not None else {}

    def take_output(self) -> Tuple[List[str], Dict[str, Any]]:
        """Console lines printed since the last call, and the watched values.

        Drained rather than accumulated here: the caller (Robot) forwards them
        over the bulk link, and holding a second copy would just be another
        place for them to be out of date.
        """
        if self.mailbox is None:
            return [], {}
        lines, _ = self.mailbox.take_output()
        return lines, self.mailbox.watched()

    def status(self) -> Optional[dict]:
        """A summary small enough for a radio shared with telemetry.

        Deliberately not the console: output is kilobytes and rides the bulk
        link. This is the handful of bytes that answer "is my script still
        going, and if not why" from the far side of a field.
        """
        runner = self.runner
        if runner is None:
            return {"id": self.selected, "run": False,
                    "why": self.last_reason or None,
                    "err": (self.last_error.splitlines()[-1][:80]
                            if self.last_error else None)}
        return {
            "id": self.selected,
            "run": True,
            "t": round(runner.elapsed, 1),
            "drive": self._delegate_name or None,
        }
