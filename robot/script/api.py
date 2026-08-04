"""The `rover` object: everything a script is allowed to see and touch.

This is the whole API surface of the coding interface. If it is not reachable
from `rover`, a script cannot do it — which is what makes the surface something
that can be documented, mirrored in the dashboard's reference panel, and
reasoned about.

--- the two directions, and why neither is a direct call ---
A script runs on its own thread (runtime.py). The control loop runs on the main
one. They meet at a `Mailbox` and nowhere else:

    control loop  --publish(snapshot)-->  Mailbox  --read()-->    script
    script        --submit(command)---->  Mailbox  --drain()-->   control loop

**Reads** come from a snapshot the control loop publishes once per tick, so
`rover.distance_ahead()` is a dict lookup that cannot block, cannot be
half-updated, and is never more than one tick (20 ms at 50 Hz) old. Reading the
sensor object directly would put a script's `while` loop in a position to hold
a lock the control loop needs.

**Writes** are commands the control loop applies on its next tick. Nothing here
writes a PWM value, and that is the single most important line in this file: one
thread owns the hardware, and it is the one that always owned it.

--- why actuator calls wait a tick ---
`rover.mech("intake").power(1.0)` does not return until the control loop has
actually applied it. That costs one tick and buys the thing every beginner
assumes is true anyway: the next line of the script runs in a world where the
previous line happened. Without it, the classic first program —

    intake.pulse()
    rover.wait_until(lambda: intake.ready)

— falls straight through, because `ready` is still answering about the state
before the pulse. Fire-and-forget is available (`wait=False`) for the rare loop
that is issuing commands faster than the control rate and does not care.

--- units ---
Distances are metres, angles are degrees (0 = north, clockwise positive, the
same convention as `pose`), speeds are -1..1 normalized track output, and time
is seconds. No exceptions, so nothing here needs a suffix in its name to be read
correctly.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

# How often a waiting helper re-checks. One control tick at 50 Hz: fast enough
# that `wait_until` is not the reason a script reacts late, slow enough that a
# script waiting on a slow event is not a busy loop burning a core.
POLL_S = 0.02

# Ceiling on the mailbox. A script that submits faster than the control loop
# drains — a `while True: rover.forward(1)` with `wait=False` — must not grow a
# queue for the rest of the match. The OLDEST are dropped: they are commands
# that have been superseded by the very next line of the same script.
MAX_QUEUED = 256

# Ceiling on watched values. `rover.watch` is a debugging aid that renders as a
# table in the dashboard, and a table with a thousand rows is not one.
MAX_WATCHED = 32


class ScriptAborted(BaseException):
    """Raised inside a script when something outside it says stop.

    Deliberately a BaseException. A script that wraps its main loop in
    `try: ... except Exception:` — which is a reasonable thing to write, and
    what most people's first retry loop looks like — would otherwise swallow
    the stop button and keep driving.
    """


class ScriptTimeout(Exception):
    """A `wait_until` or a blocking helper gave up.

    A plain Exception, unlike the above: a timeout is a thing a script is
    entitled to catch and handle ("if I can't see the bucket in 10 seconds,
    go look somewhere else").
    """


class Mailbox:
    """The one place the script thread and the control thread meet.

    Every method is safe to call from either side. The lock is held for
    dictionary swaps and list appends only — never across a hardware write,
    which is not done here at all, and never while a script is sleeping.
    """

    def __init__(self, max_queued: int = MAX_QUEUED):
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._snapshot: Dict[str, Any] = {}
        self._queue: List[Tuple[str, Any, int]] = []
        self._max_queued = max_queued
        self._seq = 0
        self._applied = 0
        self._dropped = 0
        # Set when the script must unwind: stopped, e-stopped, timed out, or
        # the mode switched out from under it. An Event rather than a bool so a
        # sleeping script wakes immediately instead of at the end of its nap.
        self.abort = threading.Event()
        self.abort_reason = ""
        # Console output and watched values, read by the controller for the
        # dashboard. Bounded; see MAX_WATCHED and ScriptConfig.output_lines.
        self._output: List[str] = []
        self._output_max = 400
        self._output_seq = 0
        self._watched: Dict[str, Any] = {}

    # --- the control thread's side -------------------------------------------

    def publish(self, snapshot: Dict[str, Any]) -> None:
        """Install this tick's sensor readings. Called once per control tick."""
        with self._lock:
            self._snapshot = snapshot

    def drain(self) -> List[Tuple[str, Any, int]]:
        """Take everything the script has asked for since the last tick."""
        with self._cv:
            queued, self._queue = self._queue, []
            return queued

    def note_applied(self, seq: int) -> None:
        """Mark commands up to `seq` as done, waking anything waiting on them."""
        with self._cv:
            if seq > self._applied:
                self._applied = seq
            self._cv.notify_all()

    def cancel(self, reason: str = "stopped") -> None:
        """Ask the script to unwind at its next API call or line of code."""
        with self._cv:
            if not self.abort.is_set():
                self.abort_reason = reason
            self.abort.set()
            self._cv.notify_all()

    def dropped(self) -> int:
        with self._lock:
            return self._dropped

    # --- the script thread's side --------------------------------------------

    def read(self) -> Dict[str, Any]:
        with self._lock:
            return self._snapshot

    def submit(self, kind: str, payload: Any = None) -> int:
        """Queue one command for the control loop. Returns its sequence number."""
        with self._cv:
            self._seq += 1
            self._queue.append((kind, payload, self._seq))
            if len(self._queue) > self._max_queued:
                # Drop from the FRONT. A queue this long means the script is
                # outrunning the control loop, and the commands worth keeping
                # are the most recent ones — the older ones were superseded
                # before anything could act on them.
                overflow = len(self._queue) - self._max_queued
                del self._queue[:overflow]
                self._dropped += overflow
            return self._seq

    def wait_applied(self, seq: int, timeout: float = 1.0) -> bool:
        """Block until the control loop has applied command `seq`.

        False on timeout rather than raising: a command still unapplied after a
        second means the control loop has stopped calling us — the mode was
        switched, or the run was ended — and the abort check that follows every
        call of this is the thing that turns that into a clean unwind.
        """
        end = time.monotonic() + max(0.0, float(timeout))
        with self._cv:
            while self._applied < seq and not self.abort.is_set():
                remaining = end - time.monotonic()
                if remaining <= 0:
                    return False
                self._cv.wait(timeout=min(POLL_S * 2, remaining))
            return self._applied >= seq

    def check(self) -> None:
        """Raise if something has asked the script to stop. Called everywhere."""
        if self.abort.is_set():
            raise ScriptAborted(self.abort_reason or "stopped")

    def sleep(self, seconds: float) -> None:
        """Wait, wake early on abort. The only sleep a script can reach."""
        self.check()
        if seconds > 0 and self.abort.wait(timeout=seconds):
            raise ScriptAborted(self.abort_reason or "stopped")
        self.check()

    # --- output --------------------------------------------------------------

    def set_output_limit(self, lines: int) -> None:
        with self._lock:
            self._output_max = max(20, int(lines))

    def write_line(self, text: str) -> None:
        with self._lock:
            self._output.append(text)
            self._output_seq += 1
            if len(self._output) > self._output_max:
                del self._output[: len(self._output) - self._output_max]

    def take_output(self) -> Tuple[List[str], int]:
        """Everything printed since the last call, and the running line count."""
        with self._lock:
            lines, self._output = self._output, []
            return lines, self._output_seq

    def set_watch(self, name: str, value: Any) -> None:
        with self._lock:
            if name not in self._watched and len(self._watched) >= MAX_WATCHED:
                return
            self._watched[name] = value

    def watched(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._watched)

    def clear_watch(self) -> None:
        with self._lock:
            self._watched.clear()


def _num(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class _Reading:
    """Base for the little read-only views below.

    Each one is a thin façade over one branch of the published snapshot, built
    fresh on every attribute access from `rover`. That is what makes

        while rover.gps.fix == 0:
            rover.sleep(0.5)

    terminate: `rover.gps` is re-read each time round, so it sees the new fix.
    A cached object would be a snapshot of the moment the script started, which
    is the single most confusing thing this API could do.
    """

    __slots__ = ("_d",)

    def __init__(self, data: Optional[dict]):
        self._d = data or {}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self._d!r})"

    @property
    def raw(self) -> dict:
        """The underlying telemetry dict, for anything this façade has not
        grown a name for yet. Escape hatch, not the main road."""
        return dict(self._d)


class Wheels(_Reading):
    """What the tracks actually did, from the encoders. Empty without them."""

    @property
    def ok(self) -> bool:
        return bool(self._d)

    @property
    def left_rpm(self) -> Optional[float]:
        """Mean measured RPM across the left side, or None with no encoders.

        A mean across the side rather than one wheel, because the useful number
        is how fast the TRACK is going — the per-actuator speeds are in `rpm`
        for finding the one wheel that is dragging.
        """
        return _num(self._d.get("l"))

    @property
    def right_rpm(self) -> Optional[float]:
        return _num(self._d.get("r"))

    @property
    def rpm(self) -> Dict[str, float]:
        """Measured speed per actuator, keyed by the operator's own names."""
        return dict(self._d.get("rpm") or {})


class Gps(_Reading):
    @property
    def ok(self) -> bool:
        """A fix good enough to navigate on."""
        return bool(self._d) and (self._d.get("fix") or 0) > 0

    @property
    def fix(self) -> int:
        return int(self._d.get("fix") or 0)

    @property
    def satellites(self) -> Optional[int]:
        sats = self._d.get("sats")
        return int(sats) if sats is not None else None

    @property
    def hdop(self) -> Optional[float]:
        return _num(self._d.get("hdop"))

    @property
    def speed(self) -> Optional[float]:
        """Ground speed, m/s."""
        return _num(self._d.get("speed"))

    @property
    def track(self) -> Optional[float]:
        """Course over ground in degrees — meaningless standing still."""
        return _num(self._d.get("track"))


class Imu(_Reading):
    @property
    def ok(self) -> bool:
        return self.heading is not None

    @property
    def heading(self) -> Optional[float]:
        return _num(self._d.get("heading"))

    @property
    def calibration(self) -> Optional[int]:
        """0-3. Below `imu.min_calib` the heading falls back to the GPS track."""
        calib = self._d.get("calib")
        return int(calib) if calib is not None else None


class Vision(_Reading):
    """What the detector sees right now — one target, summarised."""

    @property
    def ok(self) -> bool:
        """Is the detector running at all? False with no model or no camera."""
        return bool(self._d.get("ok"))

    @property
    def seen(self) -> bool:
        """Is something in view right now?

        Goes False on its own once a detection ages past `vision.target_timeout`
        — the robot drops the stale one from its summary — so a script polling
        this does not have to think about how old the last box was.
        """
        return self._d.get("ex") is not None

    @property
    def label(self) -> str:
        return str(self._d.get("label") or "")

    @property
    def confidence(self) -> Optional[float]:
        return _num(self._d.get("conf"))

    @property
    def age(self) -> Optional[float]:
        """Seconds since the detection was made."""
        return _num(self._d.get("age"))

    @property
    def offset(self) -> Optional[float]:
        """How far off centre the target is, -1 (hard left) to +1 (hard right).

        The same number `object_align` steers on, so a script that wants its
        own approach loop is working from the same measurement the built-in
        one does rather than a second, subtly different idea of "centred".
        """
        return _num(self._d.get("ex"))

    @property
    def bearing(self) -> Optional[float]:
        """`offset` in degrees, using the camera's configured field of view."""
        offset, hfov = self.offset, _num(self._d.get("hfov"))
        if offset is None or not hfov:
            return None
        return offset * hfov / 2.0

    @property
    def size(self) -> Optional[float]:
        """Box height as a fraction of the frame — the raw thing `distance` is
        derived from, and the one to log while calibrating."""
        return _num(self._d.get("size"))

    @property
    def distance(self) -> Optional[float]:
        """Metres to the target, or None when nobody can say.

        None whenever the answer isn't known — nothing detected, no
        calibration, no ultrasonic echo. A script comparing it must handle
        that; `rover.wait_until` and the helpers below already do.
        """
        return _num(self._d.get("dist"))

    @property
    def fps(self) -> Optional[float]:
        return _num(self._d.get("fps"))


class Align(_Reading):
    """What the alignment controller believes, while one is driving.

    Read straight off the same controller the routine conditions read, which is
    the point: a script and a graph asking "am I lined up" get one answer, not
    two that drift.
    """

    @property
    def aligned(self) -> bool:
        return bool(self._d.get("aligned"))

    @property
    def arrived(self) -> bool:
        return bool(self._d.get("arrived"))

    @property
    def distance(self) -> Optional[float]:
        return _num(self._d.get("dist"))


class MechanismHandle:
    """One thing that moves, addressed by the name the layout gave it.

    Every command here is queued and applied by the control loop, and by
    default the call does not return until it has been (see the module
    docstring). Reads come from the same status dict the dashboard's mechanism
    cards render, so what a script sees is what an operator sees.
    """

    def __init__(self, rover: "Rover", name: str):
        self._rover = rover
        self._name = name

    def __repr__(self) -> str:
        return f"<mechanism {self._name!r}>"

    @property
    def name(self) -> str:
        return self._name

    @property
    def _status(self) -> dict:
        return (self._rover._snapshot().get("mech") or {}).get(self._name) or {}

    # --- commands ------------------------------------------------------------

    def power(self, value: float, actuator: Optional[str] = None,
              wait: bool = True) -> None:
        """Run it at -1..1. `actuator` picks one of a multi-motor mechanism."""
        self._rover._command("mech_power", (self._name, float(value),
                                            actuator), wait)

    def preset(self, name: str, wait: bool = True) -> None:
        """Apply a named position from the layout ("up", "closed", "stow")."""
        self._rover._command("mech_preset", (self._name, str(name)), wait)

    def pulse(self, wait: bool = True) -> None:
        """Start the one cycle this mechanism owns — a kicker's stroke, a
        sequence's whole run. `ready` goes False until it finishes."""
        self._rover._command("mech_pulse", self._name, wait)

    # A launcher's word for the same instruction. Not an alias on the class
    # docstring's whim: `fire()` is what a script about shooting will reach for,
    # and having to remember it is spelled `pulse` is a papercut in the one
    # place a papercut is a misfire.
    fire = pulse

    def stop(self, wait: bool = True) -> None:
        self._rover._command("mech_stop", self._name, wait)

    def spin_for(self, distance_m: float, actuator: Optional[str] = None,
                 wait: bool = True) -> bool:
        """Run a flywheel at the speed a shot from this range needs.

        The one command here that COMPUTES rather than relays: it asks the
        ballistics model (control/ballistics.py) what a throw at `distance_m`
        takes, and sends that. False — and NOTHING SPINS — when the answer
        isn't known: no calibration, or a shot the wheel cannot reach. A
        flywheel at some fallback speed throws a ball a distance nobody chose,
        and "it fired but missed" is much harder to read at a field than "it
        never fired".
        """
        return self._rover._command_result(
            "mech_shot", (self._name, float(distance_m), actuator), wait)

    # --- readings ------------------------------------------------------------

    @property
    def exists(self) -> bool:
        return self._name in self._rover.mechanisms

    @property
    def kind(self) -> str:
        """power | pulse | sequence — which verbs this one answers to."""
        return str(self._status.get("kind") or "")

    @property
    def ready(self) -> bool:
        """True when it is idle and can be asked to do something.

        False for a pulse or a sequence mid-cycle, which is what makes
        `wait_until(lambda: m.ready)` the right way to wait for one to finish.
        A power mechanism reports no cycle at all, so it is always ready.
        """
        status = self._status
        if not status:
            return False
        return bool(status.get("ready", True))

    @property
    def state(self) -> str:
        """rest | firing | retracting | running, for a pulse or a sequence."""
        return str(self._status.get("state") or "")

    @property
    def activations(self) -> int:
        """How many cycles it has run since the robot booted."""
        return int(self._status.get("count") or 0)

    @property
    def powers(self) -> Dict[str, float]:
        """What each of its actuators was last set to, -1..1.

        Plural, and not `power`, because `power` is the verb: a property of
        that name would shadow the method and turn `m.power(0.5)` into a
        TypeError at the field.
        """
        return dict(self._status.get("values") or {})

    @property
    def rpm(self) -> Dict[str, float]:
        """Measured speed per actuator, on the ones that have an encoder."""
        return dict(self._status.get("rpm") or {})

    @property
    def status(self) -> dict:
        return self._status

    def wait_ready(self, timeout: float = 10.0) -> bool:
        """Block until the cycle finishes. False on timeout, never raises."""
        try:
            self._rover.wait_until(lambda: self.ready, timeout=timeout)
            return True
        except ScriptTimeout:
            return False


class ShooterHandle:
    """The built-in launcher, which keeps its own firing policy.

    Separate from `mech()` because it is: a flywheel with a PID, a servo, a
    magazine count and an arm latch that `shooter_align` — not this script —
    is the authority on. What a script gets here is the same set of verbs the
    Hardware tab's buttons send.
    """

    def __init__(self, rover: "Rover"):
        self._rover = rover

    def __repr__(self) -> str:
        return "<shooter>"

    @property
    def _status(self) -> dict:
        return self._rover._snapshot().get("shooter") or {}

    @property
    def fitted(self) -> bool:
        return bool(self._rover._snapshot().get("has_shooter"))

    def spin(self, on: bool = True, rpm: Optional[float] = None,
             wait: bool = True) -> None:
        """Start or stop the flywheel. `rpm` sets a closed-loop target on a
        build with a measured wheel; without one it spins at its configured
        throttle."""
        self._rover._command("shooter_spin", (bool(on), _num(rpm)), wait)

    def spin_for(self, distance_m: float, wait: bool = True) -> bool:
        """Spin at the speed the ballistics model says a shot at this range
        needs. False when there is no solution — out of range, or no
        calibration — and nothing spins, which is the direction this has to
        fail in: a wheel at some fallback speed throws a ball a distance
        nobody chose."""
        return self._rover._command_result("shooter_shot", float(distance_m),
                                           wait)

    def fire(self, wait: bool = True) -> None:
        """Push one ball into the wheel. Refused by the shooter itself if it is
        mid-cycle, cooling down, or out of magazine."""
        self._rover._command("shooter_fire", None, wait)

    def stop(self, wait: bool = True) -> None:
        self._rover._command("shooter_stop", None, wait)

    @property
    def ready(self) -> bool:
        """Idle, retracted and off cooldown — able to take another `fire()`."""
        return bool(self._status.get("ready"))

    @property
    def spinning(self) -> bool:
        """True while the flywheel is being held at a commanded speed."""
        return bool(self._status.get("pid_active"))

    @property
    def shots(self) -> int:
        return int(self._status.get("count") or 0)

    @property
    def state(self) -> str:
        return str(self._status.get("state") or "")

    @property
    def throttle(self) -> Optional[float]:
        """The last throttle the flywheel's speed loop wrote."""
        return _num(self._status.get("pid_throttle"))

    @property
    def status(self) -> dict:
        return self._status


class Rover:
    """What a script talks to. One instance per run.

    Constructed by the controller and injected into the script's globals as
    `rover`, so a script never imports anything to get at the robot — the first
    line of the first program anybody writes is `rover.forward(0.3)`, and it
    works.
    """

    def __init__(self, mailbox: Mailbox, clock: Callable[[], float],
                 drive_limit: float = 1.0):
        self._mailbox = mailbox
        self._clock = clock
        self._started = clock()
        # Advisory here; the controller is the one that enforces it. Kept so a
        # script can SEE the ceiling it is working under rather than wondering
        # why full throttle feels slow.
        self.drive_limit = max(0.0, min(1.0, float(drive_limit)))

    # --- plumbing ------------------------------------------------------------

    def _snapshot(self) -> dict:
        return self._mailbox.read()

    def _command(self, kind: str, payload: Any = None, wait: bool = True) -> None:
        self._mailbox.check()
        seq = self._mailbox.submit(kind, payload)
        if wait:
            self._mailbox.wait_applied(seq)
        self._mailbox.check()

    def _command_result(self, kind: str, payload: Any = None,
                        wait: bool = True) -> bool:
        """A command whose answer the script wants. The control loop records
        the verdict against the sequence number; we read it back once applied."""
        self._mailbox.check()
        seq = self._mailbox.submit(kind, payload)
        if not wait:
            return True
        applied = self._mailbox.wait_applied(seq)
        self._mailbox.check()
        if not applied:
            return False
        return bool((self._snapshot().get("results") or {}).get(seq, False))

    # --- time and flow -------------------------------------------------------

    def sleep(self, seconds: float) -> None:
        """Wait. Wakes early — and raises — when the script is stopped.

        The only way a script can wait. `time.sleep` is deliberately not
        reachable (see runtime.py): it cannot be interrupted, so a script
        napping for thirty seconds would ignore the stop button for thirty
        seconds while the rover kept whatever command it last set.
        """
        self._mailbox.sleep(float(seconds))

    def time(self) -> float:
        """Seconds since this run started."""
        return self._clock() - self._started

    def wait_until(self, condition: Callable[[], Any], timeout: float = 30.0,
                   message: str = "") -> float:
        """Poll `condition` until it is true. Returns how long that took.

        Raises `ScriptTimeout` when it doesn't happen — a script that waits for
        a bucket that is not there must fail loudly rather than hang until the
        run's own time limit ends it with no explanation.
        """
        started = self._clock()
        limit = float(timeout) if timeout and timeout > 0 else None
        while True:
            self._mailbox.check()
            try:
                if condition():
                    return self._clock() - started
            except ScriptAborted:
                raise
            except Exception as e:
                raise ScriptTimeout(f"the condition raised {e!r}") from e
            waited = self._clock() - started
            if limit is not None and waited >= limit:
                what = message or getattr(condition, "__name__", "the condition")
                raise ScriptTimeout(f"waited {waited:.1f}s for {what}")
            self._mailbox.sleep(POLL_S)

    def wait_while(self, condition: Callable[[], Any], timeout: float = 30.0,
                   message: str = "") -> float:
        """The mirror of `wait_until`, for the loops that read better that way."""
        return self.wait_until(lambda: not condition(), timeout, message)

    # --- talking back --------------------------------------------------------

    def log(self, *parts: Any) -> None:
        """Write a line to the script console in the dashboard. `print` does
        the same thing — it is redirected here — so a script written on a
        laptop behaves the same on the rover."""
        self._mailbox.write_line(" ".join(str(p) for p in parts))

    def watch(self, name: str, value: Any) -> None:
        """Show a named live value in the console panel.

        For the numbers a script is deciding on. `print` in a 50 Hz loop
        produces a waterfall nobody can read; this produces one row that
        changes.
        """
        self._mailbox.set_watch(str(name)[:32], value)

    # --- driving -------------------------------------------------------------

    def drive(self, left: float, right: float) -> None:
        """Command the two tracks directly, each -1..1. Holds until changed.

        Fire-and-forget, unlike the actuator calls: a drive command is a
        setpoint that the next one replaces, so waiting a tick for each would
        only add latency to a steering loop. `script.drive_limit` still caps it
        — the controller enforces that, not this.
        """
        self._command("drive", (float(left), float(right)), wait=False)

    def arcade(self, throttle: float, steer: float = 0.0) -> None:
        """Forward/back and turn, mixed the way the joystick is."""
        self._command("arcade", (float(throttle), float(steer)), wait=False)

    def forward(self, speed: float = 0.3, seconds: Optional[float] = None) -> None:
        """Drive straight. With `seconds`, drive that long and then stop."""
        self.arcade(abs(float(speed)), 0.0)
        self._for(seconds)

    def back(self, speed: float = 0.3, seconds: Optional[float] = None) -> None:
        self.arcade(-abs(float(speed)), 0.0)
        self._for(seconds)

    def turn(self, rate: float = 0.3, seconds: Optional[float] = None) -> None:
        """Spin in place. Positive is clockwise (to the right)."""
        self.arcade(0.0, float(rate))
        self._for(seconds)

    def _for(self, seconds: Optional[float]) -> None:
        """Hold the command just set for a while, then stop. Stops on the way
        out of an abort too — a `finally`, because a script that is being
        stopped mid-drive is exactly when the motors must not stay running."""
        if seconds is None:
            return
        try:
            self.sleep(seconds)
        finally:
            self._command("drive", (0.0, 0.0), wait=False)

    def stop(self) -> None:
        """Stop the tracks. Does not stop mechanisms — `rover.stop_all()` does."""
        self.release()
        self._command("drive", (0.0, 0.0))

    def stop_all(self) -> None:
        """Stop the tracks and every mechanism this build has."""
        self.release()
        self._command("stop_all")

    @property
    def commanded(self) -> Tuple[float, float]:
        """The (left, right) the drivetrain was last given, after the collision
        guard and any limit. What a script asked for is not always what went to
        the motors, and this is the half that did."""
        drive = self._snapshot().get("drive") or {}
        return (_num(drive.get("l")) or 0.0, _num(drive.get("r")) or 0.0)

    # --- handing over to the built-in autonomy -------------------------------

    def hand_over(self, controller: str) -> None:
        """Let one of the rover's own modes drive until told otherwise.

        The script keeps running — it can watch, log, work a mechanism, and
        take the wheel back — but the drive command comes from `object_align`,
        `waypoint`, `ball_intake` or `teleop` rather than from `rover.drive`.

        This is the same delegation a routine state does, through the same
        lifecycle: the controller is activated by the control loop, and
        released when the script releases it, finishes, or is stopped.
        """
        self._command("delegate", str(controller))

    def release(self) -> None:
        """Take the wheel back from a delegate. Safe when there isn't one."""
        self._command("delegate", "")

    @property
    def driving(self) -> str:
        """Which mode is producing the drive command: "" for the script's own."""
        return str(self._snapshot().get("delegate") or "")

    def align_to(self, label: Optional[str] = None,
                 within_m: Optional[float] = None,
                 timeout: float = 20.0, hold: bool = False) -> bool:
        """Point at something the camera can see and drive up to it.

        Returns True once the alignment controller says it has arrived, False
        if it runs out of time. The wheel is handed back on the way out unless
        `hold=True`, so the line after this one is driving again.

        `label` borrows the detector's target for the duration — the operator's
        own choice is put back — exactly as a routine state's `target` does.
        """
        if label:
            self._command("target", str(label))
        if within_m:
            self._command("standoff", ("object_align", float(within_m)))
        self.hand_over("object_align")
        try:
            self.wait_until(lambda: self.align.arrived, timeout=timeout,
                            message=f"{label or 'the target'} to be reached")
            return True
        except ScriptTimeout:
            return False
        finally:
            if not hold:
                self.release()
                self._command("target", "")
                self._command("standoff", ("object_align", 0.0))

    def follow_route(self, waypoints: Sequence[Any], timeout: float = 120.0,
                     hold: bool = False) -> bool:
        """Drive a list of (lat, lon) points with the waypoint controller.

        True when the last leg finishes, False on timeout. Points may be
        `(lat, lon)` pairs or `{"lat":…, "lon":…}` — the same shapes a routine's
        `set_route` accepts, because the same parser reads them.
        """
        self._command("route", list(waypoints))
        self.hand_over("waypoint")
        try:
            self.wait_until(lambda: self.route_done, timeout=timeout,
                            message="the route to finish")
            return True
        except ScriptTimeout:
            return False
        finally:
            if not hold:
                self.release()

    @property
    def route_done(self) -> bool:
        return bool(self._snapshot().get("route_done"))

    def turn_to(self, heading: float, tolerance: float = 5.0,
                speed: float = 0.35, timeout: float = 15.0) -> bool:
        """Pivot until the rover is facing a compass heading.

        A plain proportional turn rather than a call into the waypoint
        controller's heading PID, because it has to work with no GPS fix and no
        route — which is the state a rover is in on a bench, where this is
        mostly used. Returns False if it cannot get there in time, and stops
        either way.
        """
        target = float(heading) % 360.0
        tolerance = max(0.5, abs(float(tolerance)))
        speed = max(0.05, min(1.0, abs(float(speed))))
        deadline = self._clock() + max(0.0, float(timeout))
        try:
            while True:
                self._mailbox.check()
                now = self.heading()
                if now is None:
                    self.log("turn_to: no heading — is the IMU calibrated?")
                    return False
                error = (target - now + 180.0) % 360.0 - 180.0
                if abs(error) <= tolerance:
                    return True
                if self._clock() >= deadline:
                    return False
                # Ease off inside 45 degrees so the last few don't overshoot,
                # with a floor: below about a third of throttle a tracked
                # chassis does not turn at all, it just sits and buzzes.
                effort = max(0.35, min(1.0, abs(error) / 45.0))
                self.arcade(0.0, speed * effort * (1.0 if error > 0 else -1.0))
                self.sleep(POLL_S)
        finally:
            self._command("drive", (0.0, 0.0), wait=False)

    # --- sensors -------------------------------------------------------------

    @property
    def estopped(self) -> bool:
        return bool(self._snapshot().get("estop"))

    @property
    def wheels(self) -> Wheels:
        return Wheels(self._snapshot().get("enc"))

    @property
    def gps(self) -> Gps:
        return Gps(self._snapshot().get("gps"))

    @property
    def imu(self) -> Imu:
        return Imu(self._snapshot().get("imu"))

    @property
    def vision(self) -> Vision:
        return Vision(self._snapshot().get("vision"))

    # The word an operator uses for the thing bolted to the front. Same object.
    camera = vision

    @property
    def align(self) -> Align:
        return Align(self._snapshot().get("align"))

    def heading(self) -> Optional[float]:
        """Which way the rover is facing, degrees, 0 = north, clockwise.

        From the IMU when it is calibrated, from the GPS track angle when it is
        not, and None when neither can say — the same fused answer the map and
        the waypoint controller use, rather than a second opinion.
        """
        pose = self._snapshot().get("pose")
        if pose and pose[2] is not None:
            return float(pose[2])
        return self.imu.heading

    def position(self) -> Optional[Tuple[float, float]]:
        """(lat, lon), or None without a fix."""
        pose = self._snapshot().get("pose")
        return (float(pose[0]), float(pose[1])) if pose else None

    def distance_ahead(self) -> Optional[float]:
        """Metres to whatever is straight in front, from the ultrasonic.

        Needs no model and no calibration and knows nothing about what it is
        looking at — which makes it the one to use for "creep forward until
        something is close", and the wrong one for "until the bucket is close"
        in a room with a chair in it. None without an ultrasonic fitted, or
        when the echo never came back.
        """
        return _num((self._snapshot().get("sonar") or {}).get("d"))

    def look_for(self, label: str) -> None:
        """Point the detector at a class of thing. Put back when the run ends."""
        self._command("target", str(label))

    # --- actuators -----------------------------------------------------------

    @property
    def mechanisms(self) -> List[str]:
        """The names this build's layout declares, for `rover.mech(name)`."""
        return sorted((self._snapshot().get("mech") or {}).keys())

    def mech(self, name: str) -> MechanismHandle:
        """One mechanism by name. Returns a handle even for a name this build
        does not have — `handle.exists` says so, and every command on it is a
        logged no-op rather than a crash, so a script written for the rover
        with an arm still runs on the one without."""
        return MechanismHandle(self, str(name))

    @property
    def shooter(self) -> ShooterHandle:
        return ShooterHandle(self)
