# 7 · Program it in Python

*Settings → Code. Real Python, written in the dashboard, running on the robot.*

A [routine](routines.md) is a state machine you draw. It is the right shape for
"line up, wait until you're aligned, shoot, back off" and the wrong shape for
anything with arithmetic in it — a loop that scales throttle by measured range
is a program, and drawing it as a graph gives you a picture of a program rather
than a program.

So the **Code** tab is the other half. Same rover, same sensors, same
mechanisms, same autonomy modes — reached from Python instead of from a canvas.
Neither replaces the other, and most teams end up with some of each.

```python
# Creep up to whatever is in front, then stop short of it.
while True:
    ahead = rover.distance_ahead()
    rover.watch("ahead", ahead)
    if ahead is not None and ahead <= 0.4:
        break
    rover.forward(0.22)
    rover.sleep(0.05)

rover.stop()
print("stopped at", round(rover.distance_ahead() or 0, 2), "m")
```

Start from a template rather than an empty file. Every one runs as-is on a bare
chassis — they check for a camera or a mechanism before reaching for one — so
"press New, press Run, watch it move" is true before any of the optional
hardware is fitted.

## Everything hangs off `rover`

There is no import and no setup. `rover` is already there, and the panel down
the right-hand side lists every call it has; clicking one types it at the caret.

| | |
|---|---|
| **Driving** | `rover.forward(0.3, seconds=2)` · `back` · `turn` · `arcade(throttle, steer)` · `drive(left, right)` · `stop()` · `turn_to(heading)` |
| **Sensors** | `rover.distance_ahead()` · `rover.heading()` · `rover.position()` · `rover.gps.fix` · `rover.imu.calibration` · `rover.wheels.left_rpm` |
| **Camera** | `rover.vision.seen` · `.label` · `.offset` · `.bearing` · `.distance` · `rover.look_for("bucket")` |
| **Mechanisms** | `rover.mech("intake").power(0.8)` · `.preset("up")` · `.pulse()` · `.ready` · `.wait_ready()` · `.spin_for(metres)` |
| **Launcher** | `rover.shooter.spin()` · `.spin_for(metres)` · `.fire()` · `.ready` · `.shots` |
| **Autonomy** | `rover.align_to("bucket", within_m=1.0)` · `rover.follow_route(points)` · `rover.hand_over("waypoint")` |
| **Waiting** | `rover.sleep(s)` · `rover.wait_until(cond, timeout=10)` · `rover.time()` |
| **Output** | `print(...)` · `rover.log(...)` · `rover.watch(name, value)` |

Distances are **metres**, angles are **degrees** (0 = north, clockwise
positive), speeds are **−1 to 1**, and time is **seconds**. No exceptions, so
nothing needs a suffix in its name to be read correctly.

A reading is `None` whenever nobody can say — no GPS fix, nothing detected, no
ultrasonic fitted, no range calibration. Check for it rather than comparing it;
`None <= 0.4` raises, and it is the first thing every new script gets wrong.

### Handing over to the autonomy that already exists

`rover.align_to("bucket", within_m=1.0)` does not re-implement an approach. It
hands the wheel to the same `object_align` controller the Routines tab
delegates to, with the same gains and the same standoff, waits until that
controller says it has arrived, and hands the wheel back. Your script keeps
running the whole time — it can watch, log and work a mechanism while the rover
drives itself.

The detector target and the standoff are **borrowed**, not taken: whatever the
operator had selected is put back when the script ends, however it ends.

### `loop()`, for anything that steers

Define a function called `loop()` and it is called repeatedly at the control
rate, after your top-level code has run once. That is the shape for anything
steering on a live measurement — and it puts the pacing in the runner rather
than in a `rover.sleep(0.02)` at the bottom of a `while True:` that the next
person to edit the file deletes.

## Running it

1. **Save to robot.** Scripts take effect the moment they land — they are text
   the runner compiles, not hardware a constructor owns. The Run button is
   disabled while you have unsaved edits, because Run starts what the *rover* is
   carrying and watching a bug you already fixed happen again is a miserable
   five minutes.
2. Press **Run** (or <kbd>⌘/Ctrl</kbd> + <kbd>Enter</kbd>). The robot switches
   to `script` mode.
3. Watch the **console** underneath. `print` lands there; `rover.watch` puts a
   named live number in the row above it, which is what you want for anything
   changing at 50 Hz.
4. **Stop** ends the run and puts the rover back in teleop.

Every script the robot carries also appears by name under **Scripts** on the
driving view, beside the routines — so you do not have to be in the editor to
start one mid-match.

> **Test it dry**
>
> The simulator runs the real controller and the real sandbox. You can write,
> run and debug an entire script against `--sim` — including the syntax-error
> path — and only then put it on a rover.

## The rules it runs under

These are worth reading once, because they are what makes "run arbitrary Python
on the robot" a reasonable thing to do.

**Your code never touches the hardware.** It puts commands in a mailbox and the
control loop applies them on its next tick. One thread writes PWM, and it is
the same one that always did.

**Your code is not on the control loop.** It runs on its own thread, so a script
that spins forever costs a core, not a rover. The 50 Hz loop keeps its tick.

**It can always be stopped.** Every `rover` call checks for a stop, and so does
a line hook — so even `while True: pass` unwinds when you press Stop. The
exception that does the unwinding is not a normal one either: a script's own
`try: ... except Exception:` cannot swallow the stop button.

**When it stops, the motors stop.** Finished, crashed, stopped, e-stopped, hit
its time limit, mode switched — all of them end with the drive command back at
zero and every mechanism stopped, without your script needing a `finally:`.

**The e-stop is above all of it.** It is arbitrated before the script controller
is asked at all, exactly as it is for a routine.

**A run always ends.** `scripts.max_runtime` (5 minutes by default) is a
wall-clock ceiling. A bug in a loop condition looks exactly like a rover that
has stopped taking orders, so every run ends whatever it thinks.

### What Python you get

Ordinary Python, minus the parts that could only cause trouble on a robot:

- **Available:** `math`, `random`, `statistics`, `json`, `re`, `collections`,
  `itertools`, `functools`, `dataclasses`, `enum`, `decimal` and friends.
- **Refused:** `os`, `sys`, `subprocess`, `socket`, `threading`, `importlib`,
  `pathlib`, `ctypes` — and `open`, `exec`, `eval`.
- **`time` is refused too**, and that one is a safety rule rather than tidiness:
  `time.sleep` cannot be interrupted, so a script napping for thirty seconds
  would ignore the stop button for thirty seconds while the rover kept driving.
  Use `rover.sleep` and `rover.time`.

That guard is against **mistakes, not malice**. In-process Python cannot be a
security boundary and claiming otherwise would be the dangerous statement —
anyone who can push documents to a rover could already reflash it. It is also
not where the safety comes from: every rule in the section above holds whatever
the script does.

### Syntax errors are refused, not deferred

The robot **compiles** each script as it lands. A missing colon comes back as
`line 12: expected ':'`, the editor puts a red marker on line 12, and nothing is
installed — the rover keeps the last set that was good. That is deliberately
different from "saved fine, died on Run", which is the version you discover at
a field.

Compiling is not running. A document full of `os.system(...)` compiles and
stores without anything happening; it is refused when you press Run, by the
import list above.

## Knobs

Under **Settings → Tuning**, all live:

`scripts.enabled`
: Whether this robot will run one at all.

`scripts.max_runtime`
: Wall-clock ceiling on a run, in seconds.

`scripts.drive_limit`
: Ceiling on what a script may command the tracks, 0.05 to 1.0. Turn it down to
bring a new script up on a bench and full throttle becomes a crawl — it *scales*
the command rather than clamping each track, so the rover drives the same arc
the script asked for, just slower.

`scripts.output_lines`
: How much console output the robot keeps.

## Getting them in and out

**Export .py** writes the open script as a plain `.py` file, which opens in any
editor and diffs in git. **Export all** writes the whole set as the JSON the
robot stores. **Import** takes either: a `.py` becomes one new script, a `.json`
replaces the set.

Console output is the one part of this that needs WiFi — it rides the bulk link
with the config snapshots and the documents, because a script's `print` is
kilobytes of text and the radio is carrying driving, telemetry and the e-stop.
A rover out of WiFi range still runs its script and still tells you on the
driving view whether it is going and why it stopped; it just cannot tell you
what it is saying.
