"""Running operator-written Python: the sandbox, and being able to stop it.

These are the tests that matter most in this package, because the whole feature
rests on three claims:

  * a script cannot reach anything it was not given;
  * a script can ALWAYS be stopped, including one with no API call in its loop;
  * a script that crashes reports the operator's line, not ours.

The runner needs a control loop to talk to, which here is a few lines that
drain the mailbox — the same contract ScriptController implements, without the
robot.
"""

import threading
import time

import pytest

from robot.script.api import Mailbox, Rover, ScriptAborted, ScriptTimeout
from robot.script.runtime import ScriptError, ScriptRunner


class Loop:
    """A stand-in control loop: drains the mailbox and acknowledges.

    Runs on its own thread so the tests read like the real thing — the script
    blocks on an actuator call and something else releases it.
    """

    def __init__(self, mailbox, snapshot=None):
        self.mailbox = mailbox
        self.commands = []
        self.snapshot = snapshot or {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.mailbox.publish(self.snapshot)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            highest = 0
            for kind, payload, seq in self.mailbox.drain():
                self.commands.append((kind, payload))
                highest = max(highest, seq)
            self.mailbox.publish(self.snapshot)
            if highest:
                self.mailbox.note_applied(highest)
            time.sleep(0.005)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        # One last drain. A short script can finish inside a single sleep of
        # the loop above, and its commands are the ones a test is about.
        for kind, payload, _ in self.mailbox.drain():
            self.commands.append((kind, payload))

    def kinds(self):
        return [kind for kind, _ in self.commands]


def run(code, snapshot=None, max_runtime=5.0, wait=3.0):
    """Run a script to completion against a fake control loop."""
    mailbox = Mailbox()
    rover = Rover(mailbox, time.monotonic)
    loop = Loop(mailbox, snapshot).start()
    runner = ScriptRunner(code, "test", rover, mailbox, max_runtime=max_runtime)
    try:
        runner.start()
        runner.join(timeout=wait)
    finally:
        loop.stop()
    return runner, loop, mailbox


# --- the ordinary case -------------------------------------------------------


def test_a_script_runs_and_its_prints_reach_the_console():
    runner, _, mailbox = run("print('hello', 1 + 1)\n")
    assert runner.finished
    assert runner.reason == "finished"
    assert mailbox.take_output()[0] == ["hello 2"]


def test_rover_is_injected_without_an_import():
    runner, loop, _ = run("rover.forward(0.5)\nrover.stop()\n")
    assert not runner.error
    assert ("arcade", (0.5, 0.0)) in loop.commands
    assert ("drive", (0.0, 0.0)) in loop.commands


def test_a_loop_function_is_called_repeatedly():
    code = (
        "count = 0\n"
        "def loop():\n"
        "    global count\n"
        "    count += 1\n"
        "    rover.watch('count', count)\n"
        "    if count >= 3:\n"
        "        raise ScriptTimeout('done')\n"
    )
    runner, _, mailbox = run(code)
    assert mailbox.watched()["count"] == 3


def test_a_syntax_error_is_a_refusal_not_a_run():
    mailbox = Mailbox()
    runner = ScriptRunner("if True\n  pass\n", "test",
                          Rover(mailbox, time.monotonic), mailbox)
    with pytest.raises(ScriptError) as caught:
        runner.start()
    assert "line 1" in str(caught.value)
    assert not runner.running


# --- the sandbox -------------------------------------------------------------


@pytest.mark.parametrize("module", ["os", "sys", "subprocess", "socket",
                                    "threading", "importlib", "shutil",
                                    "pathlib", "ctypes", "time"])
def test_dangerous_imports_are_refused_by_name(module):
    runner, _, mailbox = run(f"import {module}\n")
    assert runner.reason == "error"
    assert "cannot import" in runner.error
    # And it says what IS available, because a refusal that does not is a
    # dead end for whoever is writing the script.
    assert "rover" in runner.error


def test_computation_modules_are_available():
    runner, _, mailbox = run("import math\nprint(round(math.pi, 2))\n")
    assert not runner.error, runner.error
    assert mailbox.take_output()[0] == ["3.14"]


def test_open_and_exec_are_gone():
    for builtin in ("open('/etc/passwd')", "exec('1')", "eval('1')"):
        runner, _, _ = run(f"{builtin}\n")
        assert runner.reason == "error"
        assert "not defined" in runner.error


def test_time_sleep_cannot_be_reached_through_import():
    """The one absence with a safety reason rather than a tidiness one: an
    uninterruptible sleep would ignore the stop button for its whole duration."""
    runner, _, _ = run("import time\ntime.sleep(30)\n")
    assert runner.reason == "error"
    assert runner.elapsed < 5.0


# --- stopping ----------------------------------------------------------------


def test_a_script_with_no_api_call_in_its_loop_still_stops():
    """The load-bearing one. `while True: pass` has nothing to cooperate with,
    so the line hook is what unwinds it."""
    mailbox = Mailbox()
    rover = Rover(mailbox, time.monotonic)
    loop = Loop(mailbox).start()
    runner = ScriptRunner("while True:\n    pass\n", "test", rover, mailbox)
    runner.start()
    time.sleep(0.1)
    assert runner.running
    runner.stop("stopped by the operator")
    assert runner.join(timeout=2.0), "the script would not unwind"
    loop.stop()
    assert runner.reason == "stopped by the operator"


def test_a_script_cannot_swallow_its_own_stop():
    """`except Exception` is what a first retry loop looks like. ScriptAborted
    is a BaseException precisely so that one cannot catch the stop button."""
    code = (
        "while True:\n"
        "    try:\n"
        "        rover.sleep(0.01)\n"
        "    except Exception:\n"
        "        pass\n"
    )
    mailbox = Mailbox()
    loop = Loop(mailbox).start()
    runner = ScriptRunner(code, "test", Rover(mailbox, time.monotonic), mailbox)
    runner.start()
    time.sleep(0.1)
    runner.stop()
    assert runner.join(timeout=2.0), "the script caught its own abort"
    loop.stop()


def test_a_sleeping_script_wakes_immediately_on_stop():
    mailbox = Mailbox()
    loop = Loop(mailbox).start()
    runner = ScriptRunner("rover.sleep(30)\n", "test",
                          Rover(mailbox, time.monotonic), mailbox)
    runner.start()
    time.sleep(0.05)
    started = time.monotonic()
    runner.stop()
    assert runner.join(timeout=1.0)
    loop.stop()
    assert time.monotonic() - started < 0.5


def test_a_run_ends_at_its_time_limit():
    runner, _, _ = run("while True:\n    pass\n", max_runtime=0.2, wait=3.0)
    assert runner.finished
    assert "longer than" in runner.reason


def test_the_last_thing_a_run_does_is_ask_for_everything_to_stop():
    _, loop, _ = run("rover.forward(1.0)\n")
    assert loop.kinds()[-1] == "stop_all"


# --- reporting ---------------------------------------------------------------


def test_a_crash_reports_the_operators_line_and_not_ours():
    runner, _, mailbox = run("x = 1\ny = x / 0\n")
    assert runner.reason == "error"
    assert "line 2" in runner.error
    assert "ZeroDivisionError" in runner.error
    # No frames from this codebase: an operator debugging ten lines of their
    # own should not have to read the runner's stack to find them.
    assert "runtime.py" not in runner.error
    assert any("line 2" in line for line in mailbox.take_output()[0])


def test_a_timeout_is_catchable_by_the_script():
    code = (
        "try:\n"
        "    rover.wait_until(lambda: False, timeout=0.05)\n"
        "except ScriptTimeout:\n"
        "    print('gave up')\n"
    )
    runner, _, mailbox = run(code)
    assert not runner.error
    assert mailbox.take_output()[0] == ["gave up"]


# --- the mailbox itself ------------------------------------------------------


def test_an_actuator_call_does_not_return_until_it_has_been_applied():
    """The race that makes every beginner's `pulse(); wait_until(ready)` fall
    through, if the call returns before the control loop has acted."""
    mailbox = Mailbox()
    rover = Rover(mailbox, time.monotonic)
    seen = []

    def script():
        rover.mech("kicker").pulse()
        seen.append(list(mailbox.drain()))  # must already be empty: drained

    mailbox.publish({"mech": {"kicker": {"ready": True}}})
    thread = threading.Thread(target=script, daemon=True)
    thread.start()
    time.sleep(0.05)
    assert thread.is_alive(), "pulse() returned before anything applied it"
    drained = mailbox.drain()
    assert [k for k, _, _ in drained] == ["mech_pulse"]
    mailbox.note_applied(drained[-1][2])
    thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_the_mailbox_drops_the_oldest_when_a_script_outruns_the_loop():
    mailbox = Mailbox(max_queued=4)
    for n in range(10):
        mailbox.submit("drive", (n, n))
    queued = mailbox.drain()
    assert len(queued) == 4
    # The most RECENT survive: the older ones were superseded by the very next
    # line of the same script before anything could act on them.
    assert queued[-1][1] == (9, 9)
    assert mailbox.dropped() == 6


def test_reads_are_re_read_rather_than_cached():
    """`while rover.gps.fix == 0: ...` has to terminate."""
    mailbox = Mailbox()
    rover = Rover(mailbox, time.monotonic)
    mailbox.publish({"gps": {"fix": 0}})
    assert rover.gps.fix == 0
    mailbox.publish({"gps": {"fix": 1, "sats": 9}})
    assert rover.gps.fix == 1
    assert rover.gps.satellites == 9
    assert rover.gps.ok
