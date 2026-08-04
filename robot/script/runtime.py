"""Running operator-written Python: a worker thread that can always be stopped.

Three problems, and this module is the three answers.

**It must not block the control loop.** So it isn't on it. The script runs on
its own daemon thread and reaches the robot only through the mailbox in api.py.
A script that spins forever costs a core; the 50 Hz loop keeps its tick.

**It must be stoppable.** Python cannot kill a thread, so stopping is
cooperative — and the cooperation is not something the script has to opt into.
Every API call checks the abort flag, and a `sys.settrace` line hook checks it
too, so even

    while True:
        pass

unwinds on the stop button. `ScriptAborted` derives from BaseException
precisely so a script's own `except Exception` cannot catch the stop.

**A mistake must not be a rover you can't get back.** Hence the deadline (a run
has a wall-clock limit), the import whitelist, and the trimmed builtins.

--- what the sandbox is and is not ---
It is a guardrail against MISTAKES. `import os` fails, so does `open`, so does
`while True: pass`. What it is NOT is a security boundary: in-process Python has
no such thing, and CPython's introspection makes any claim otherwise false. That
is fine, because it is not where the safety comes from. The properties that
matter hold whatever the script does:

  * it cannot touch hardware — it has no reference to any, and the control loop
    is the only writer;
  * it cannot outlive its stop — the controller stops the motors when the
    thread ends, however it ended;
  * it cannot drive through an e-stop — the manager returns `stopped()` before
    the script controller is asked at all.

The threat model here is a teammate's typo at a competition, not an attacker who
already has the ability to push arbitrary documents to the rover. Anyone who has
that can already reflash it.
"""

from __future__ import annotations

import sys
import threading
import time
import traceback
from typing import Any, Callable, Dict, List, Optional

from .api import Mailbox, Rover, ScriptAborted, ScriptTimeout

# Modules a script may import. Everything here is pure computation: no
# filesystem, no network, no processes, no threads, no clock a script could
# sleep on uninterruptibly (that is what `rover.sleep` is for).
#
# `time` is the notable absence. `time.sleep` cannot be interrupted, so a script
# that naps for thirty seconds would keep whatever drive command it last set for
# thirty seconds after the stop button. `rover.sleep` and `rover.time` cover
# what anybody actually wants from it.
SAFE_MODULES = frozenset({
    "math", "random", "statistics", "json", "re", "string", "textwrap",
    "collections", "itertools", "functools", "operator", "heapq", "bisect",
    "dataclasses", "enum", "typing", "decimal", "fractions", "copy", "uuid",
})

# Builtins a script does not get. Removed rather than whitelisted-around
# because the list of things that ARE fine is long and dull (len, range, sorted,
# every exception type) while the list that isn't is short and interesting.
#
# `open` and `exec`/`eval`/`compile` are the ones with teeth: a script that can
# open files can fill the SD card the logs are on, and one that can `exec` a
# string sidesteps the import hook by building the import at runtime.
BLOCKED_BUILTINS = frozenset({
    "open", "exec", "eval", "compile", "input", "breakpoint", "help",
    "exit", "quit", "memoryview", "globals", "vars",
})

# How often the line hook actually looks at the clock and the abort flag.
# Every line would double the cost of running anything; every 400 lines is
# under a millisecond of latency on the stop button for any loop worth writing,
# and the API calls check on every single one anyway.
CHECK_EVERY_LINES = 400


class ScriptError(Exception):
    """A script that would not compile, or would not start."""


def _safe_import(name: str, globals_=None, locals_=None, fromlist=(),
                 level: int = 0):
    """The only `import` a script has. Whitelisted, and honest about it."""
    root = name.split(".")[0]
    if level != 0:
        raise ImportError("relative imports are not available in a rover script")
    if root not in SAFE_MODULES:
        raise ImportError(
            f"a rover script cannot import {root!r}. Available: "
            f"{', '.join(sorted(SAFE_MODULES))} — everything about the robot "
            f"itself is on `rover` (rover.gps, rover.mech(...), rover.sleep)")
    return __import__(name, globals_, locals_, fromlist, level)


def _build_builtins(write: Callable[[str], None]) -> Dict[str, Any]:
    """The builtins a script sees, with `print` pointed at the console."""
    source = __builtins__ if isinstance(__builtins__, dict) else vars(__builtins__)
    safe = {k: v for k, v in source.items() if k not in BLOCKED_BUILTINS}

    def _print(*parts: Any, sep: str = " ", end: str = "\n", **_ignored) -> None:
        # `file=` and `flush=` are accepted and ignored rather than refused: a
        # script pasted from somewhere else should not fail to run over an
        # argument that has no meaning here. `end` is honoured only for its
        # blank-line effect, since the console is a list of lines.
        text = sep.join(str(p) for p in parts)
        for line in (text + ("" if end == "\n" else str(end))).split("\n"):
            write(line)

    safe["print"] = _print
    safe["__import__"] = _safe_import
    return safe


class ScriptRunner:
    """One run of one script.

    Not reusable: a fresh runner per run, so nothing — module globals, a stale
    thread, a half-finished generator — can survive from the last one into the
    next. Restarting a script must behave exactly like running it the first
    time, which is the same rule `RoutineEngine` follows about counters.
    """

    def __init__(self, code: str, name: str, rover: Rover, mailbox: Mailbox,
                 max_runtime: float = 300.0,
                 clock: Optional[Callable[[], float]] = None):
        self.name = name
        self.mailbox = mailbox
        self.rover = rover
        self.max_runtime = max(0.0, float(max_runtime))
        self._clock = clock or time.monotonic
        self._code = code
        self._thread: Optional[threading.Thread] = None
        # Lines executed, counted by the trace hook. An attribute rather than a
        # closure cell because it is touched on every line of the script.
        self._lines = 0
        self._started = 0.0
        self._finished = False
        self._deadline = 0.0
        self.error: str = ""
        self.reason: str = ""

    # --- lifecycle -----------------------------------------------------------

    def start(self) -> None:
        """Compile the script and run it. Raises ScriptError if it won't compile.

        Compiling on the CALLING thread on purpose: a syntax error is the one
        failure an operator should be told about as a refusal ("line 12: invalid
        syntax") rather than as a run that started and died, and the controller
        can only report it as the former if it happens before the thread does.
        """
        try:
            compiled = compile(self._code, f"<{self.name}>", "exec")
        except SyntaxError as e:
            raise ScriptError(
                f"line {e.lineno}: {e.msg}") from e
        except ValueError as e:  # e.g. a null byte in the source
            raise ScriptError(str(e)) from e

        self._started = self._clock()
        self._deadline = (self._started + self.max_runtime
                          if self.max_runtime > 0 else 0.0)
        self._thread = threading.Thread(
            target=self._run, args=(compiled,),
            name=f"script:{self.name}", daemon=True)
        self._thread.start()

    def stop(self, reason: str = "stopped") -> None:
        """Ask the script to unwind. Returns immediately; see `finished`."""
        self.mailbox.cancel(reason)

    def join(self, timeout: float = 1.0) -> bool:
        """Wait for the thread to actually be gone. False if it wasn't.

        A script that will not unwind is a real possibility — a C-level call
        that ignores the trace hook — so callers treat False as "leave it, it
        is a daemon and it can no longer reach the hardware" rather than
        blocking the control loop until it relents.
        """
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def elapsed(self) -> float:
        if not self._started:
            return 0.0
        return self._clock() - self._started

    # --- the thread ----------------------------------------------------------

    def _run(self, compiled) -> None:
        write = self.mailbox.write_line
        namespace: Dict[str, Any] = {
            "__name__": "__rover_script__",
            "__builtins__": _build_builtins(write),
            "rover": self.rover,
            # The two exception types a script is likely to want to name, so
            # `except ScriptTimeout:` works without an import.
            "ScriptTimeout": ScriptTimeout,
            "ScriptAborted": ScriptAborted,
        }
        sys.settrace(self._trace)
        try:
            exec(compiled, namespace)
            self._run_loop(namespace)
            self.reason = self.reason or "finished"
        except ScriptAborted as e:
            self.reason = str(e) or "stopped"
        except ScriptTimeout as e:
            self.reason = "timed out"
            self.error = str(e)
            write(f"[script] {e}")
        except BaseException as e:  # noqa: BLE001 — a script may raise anything
            self.reason = "error"
            self.error = self._format(e)
            for line in self.error.split("\n"):
                write(line)
        finally:
            sys.settrace(None)
            self._finished = True
            # Whatever happened, the last thing this thread does is ask for a
            # stop. The controller stops the motors on its own when the thread
            # ends — this is belt and braces for the tick in between, and it is
            # cheap.
            try:
                self.mailbox.submit("stop_all")
            except Exception:
                pass

    def _run_loop(self, namespace: Dict[str, Any]) -> None:
        """Call `loop()` repeatedly, if the script defined one.

        The Arduino shape, and it earns its place here: a script that steers on
        a measurement wants to run every tick, and writing that as
        `while True: ...; rover.sleep(0.02)` puts the one line that keeps the
        thing responsive (the sleep) in the place a beginner deletes first.
        With `loop()`, the pacing is ours.

        Top-level code has already run by the time this is called, so it is
        `setup` without needing a name.
        """
        loop = namespace.get("loop")
        if not callable(loop):
            return
        while True:
            self.mailbox.check()
            loop()
            self.mailbox.sleep(0.02)

    def _trace(self, frame, event, arg):
        """Line hook: the only thing that can interrupt a loop with no API call.

        Returns itself so tracing follows into called functions. Most calls do
        nothing but bump a counter — the clock and the abort flag are only
        looked at every CHECK_EVERY_LINES, because this runs on every line of
        the script and a `time.monotonic()` per line would be most of the cost
        of running one.
        """
        if event == "call":
            return self._trace
        count = self._lines + 1
        self._lines = count
        if count % CHECK_EVERY_LINES:
            return self._trace
        if self.mailbox.abort.is_set():
            raise ScriptAborted(self.mailbox.abort_reason or "stopped")
        if self._deadline and self._clock() >= self._deadline:
            self.mailbox.cancel(f"ran longer than {self.max_runtime:.0f}s")
            raise ScriptAborted(f"ran longer than {self.max_runtime:.0f}s")
        return self._trace

    def _format(self, error: BaseException) -> str:
        """A traceback with this codebase's frames taken out.

        An operator debugging their own ten lines does not need to read the
        four frames of runner machinery that called them, and showing them
        makes the one line that matters — theirs — the hardest thing on screen
        to find. Frames from the script's own compiled unit are kept; everything
        else is dropped. If that leaves nothing (an error raised inside the
        rover API rather than in the script), the message stands alone.
        """
        marker = f"<{self.name}>"
        frames = [f for f in traceback.extract_tb(error.__traceback__)
                  if f.filename == marker]
        out: List[str] = []
        for frame in frames:
            out.append(f"line {frame.lineno}: {(frame.line or '').strip()}")
        out.append(f"{type(error).__name__}: {error}")
        return "\n".join(out)
