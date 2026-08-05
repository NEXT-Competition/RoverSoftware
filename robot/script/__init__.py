"""Operator-written Python: the other way to program this rover.

`robot/routine/` gives an operator a state machine they draw. It is the right
shape for "line up, wait until you're aligned, shoot, back off" and the wrong
shape for anything with arithmetic in it — a graph cannot hold a loop that
scales throttle by measured range without becoming a picture of a program
rather than a program. So this package is the other half: real Python, written
in the dashboard, running on the rover, against an API that names every sensor
and every actuator the build actually has.

Four modules, and the split is the same one the routine package makes:

    api.py       what a script may touch. The `rover` object, and nothing else.
    runtime.py   how it is run: a worker thread, a sandbox, and a way to stop it.
    schema.py    what a script document is, and what makes one refusable.
    store.py     where scripts live on the robot between power cycles.

The safety story lives in runtime.py and in control/script_controller.py, and
it is worth stating up front because "let the operator run arbitrary Python on
the robot" reads alarming and the reasons it isn't are structural:

  * User code NEVER touches hardware. It submits commands to a mailbox; the
    control loop drains it and applies them. There is exactly one thread
    writing PWM, and it is the same one that has always written PWM.
  * User code never blocks the control loop, because it is not on it. A script
    that spins forever costs a core, not a rover.
  * When the script stops — finished, aborted, crashed, e-stopped, mode
    switched — the drive command becomes `stopped()` on the next tick. That is
    a property of the controller, not a `finally:` block a script could forget.
  * The e-stop preempts everything: ControlManager returns `stopped()` before
    the script controller is asked, exactly as it does for a routine.

The sandbox — the import whitelist and the trimmed builtins in runtime.py — is
a guardrail against MISTAKES, not a security boundary. In-process Python cannot
be one, and pretending otherwise would be the dangerous claim. It is also not
where the safety comes from: everything above holds whatever the script does.
"""

from .api import Rover, ScriptAborted, ScriptTimeout
from .runtime import ScriptRunner
from .schema import Script, parse

__all__ = ["Rover", "Script", "ScriptAborted", "ScriptRunner", "ScriptTimeout",
           "parse"]
