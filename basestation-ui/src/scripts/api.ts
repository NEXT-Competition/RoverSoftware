// The rover API, as the editor's reference panel lists it.
//
// A hand-kept mirror of robot/script/api.py, exactly as net/types.ts mirrors
// the /ws contract and settings/schema.ts mirrors tuning.py. The robot is the
// authority; this is what makes the surface discoverable without leaving the
// dashboard, on a base station with no internet and no docs open.
//
// Keeping it here rather than shipping it from the robot is deliberate. It is
// ~4 KB of prose, it changes when the code does rather than per build, and the
// alternative is spending a document transfer per rover on text that is
// identical for all of them.
//
// A signature ending in `-> x` returns something; the rest are commands.
// Anything marked `blocks` does not return until the thing has happened, which
// is what makes a script read like a sequence rather than a pile of callbacks.

export interface ApiEntry {
  /** How you write it, with the argument names the robot actually accepts. */
  sig: string;
  /** One line. This is what the operator reads while writing, so it says what
   *  the call DOES and what it answers when it cannot. */
  help: string;
  /** True when the call does not return until the robot has done the thing. */
  blocks?: boolean;
}

export interface ApiGroup {
  key: string;
  title: string;
  /** Why this group exists — the sentence that stops someone reaching for the
   *  wrong one of two similar calls. */
  note?: string;
  entries: ApiEntry[];
}

export const API: ApiGroup[] = [
  {
    key: "drive",
    title: "Driving",
    note:
      "Speeds are −1 to 1. A drive command holds until you change it, so a " +
      "script that sets one and then thinks for ten seconds drives for ten " +
      "seconds. The run ending always stops the motors.",
    entries: [
      { sig: "rover.forward(speed=0.3, seconds=None)", help: "Drive straight. With seconds, drive that long then stop." },
      { sig: "rover.back(speed=0.3, seconds=None)", help: "The same, backwards." },
      { sig: "rover.turn(rate=0.3, seconds=None)", help: "Spin in place. Positive is clockwise (to the right)." },
      { sig: "rover.arcade(throttle, steer=0)", help: "Forward/back and turn together, mixed the way the joystick is." },
      { sig: "rover.drive(left, right)", help: "The two tracks directly, for your own steering loop." },
      { sig: "rover.stop()", help: "Stop the tracks. Mechanisms keep doing what they were doing.", blocks: true },
      { sig: "rover.stop_all()", help: "Stop the tracks and every mechanism.", blocks: true },
      { sig: "rover.turn_to(heading, tolerance=5, speed=0.35, timeout=15) -> bool", help: "Pivot until facing a compass heading. False if it could not get there.", blocks: true },
      { sig: "rover.commanded -> (left, right)", help: "What the drivetrain was actually given, after the collision guard cut it back." },
      { sig: "rover.drive_limit -> float", help: "The ceiling this robot caps a script's throttle at (scripts.drive_limit)." },
    ],
  },
  {
    key: "autonomy",
    title: "Handing over to the built-in modes",
    note:
      "Your script keeps running while one of the rover's own controllers " +
      "drives — it can watch, log, work a mechanism, and take the wheel back. " +
      "This is the same delegation a routine state does, through the same " +
      "lifecycle, so `align_to` aligns with the real loop and the real gains.",
    entries: [
      { sig: "rover.align_to(label=None, within_m=None, timeout=20) -> bool", help: "Point at something the camera sees and drive up to it. False on timeout.", blocks: true },
      { sig: "rover.follow_route(points, timeout=120) -> bool", help: "Drive a list of (lat, lon) with the waypoint controller.", blocks: true },
      { sig: "rover.hand_over(mode)", help: "Let teleop / object_align / shooter_align / ball_intake / waypoint drive.", blocks: true },
      { sig: "rover.release()", help: "Take the wheel back. Safe when nothing has it.", blocks: true },
      { sig: "rover.driving -> str", help: "Which mode is producing the drive command; empty for your own." },
      { sig: "rover.route_done -> bool", help: "Has the waypoint controller finished its last leg?" },
      { sig: "rover.align.aligned / .arrived / .distance", help: "What the alignment controller believes right now." },
    ],
  },
  {
    key: "sensors",
    title: "Sensors",
    note:
      "Every reading is at most one control tick old (20 ms), and re-read each " +
      "time you touch it — so `while rover.gps.fix == 0:` terminates. A reading " +
      "is None when nobody can say: no fix, nothing detected, no ultrasonic " +
      "fitted, no calibration. Check for it rather than comparing it.",
    entries: [
      { sig: "rover.distance_ahead() -> m | None", help: "Metres to whatever is straight in front, from the ultrasonic. Knows nothing about what it is looking at." },
      { sig: "rover.heading() -> deg | None", help: "Which way you are facing. 0 = north, clockwise positive. IMU when calibrated, GPS track when not." },
      { sig: "rover.position() -> (lat, lon) | None", help: "Where you are, or None without a fix." },
      { sig: "rover.estopped -> bool", help: "Is the emergency stop latched?" },
      { sig: "rover.gps.fix / .satellites / .hdop / .speed / .track", help: "Fix health — whether to believe the position above." },
      { sig: "rover.imu.heading / .calibration / .ok", help: "The absolute heading, and how much to trust it (0-3)." },
      { sig: "rover.wheels.left_rpm / .right_rpm / .rpm", help: "What the tracks actually did, from the encoders. Per side, and per actuator." },
      { sig: "rover.vision.seen / .label / .confidence", help: "What the detector sees right now." },
      { sig: "rover.vision.offset -> -1..1", help: "How far off centre the target is. The same number object_align steers on." },
      { sig: "rover.vision.bearing -> deg", help: "That offset in degrees, using the camera's field of view." },
      { sig: "rover.vision.distance -> m | None", help: "Metres to the target: measured by the ultrasonic when it can, inferred from the box otherwise." },
      { sig: "rover.look_for(label)", help: "Point the detector at a class of thing. Handed back when the run ends.", blocks: true },
    ],
  },
  {
    key: "actuators",
    title: "Mechanisms and the launcher",
    note:
      "Addressed by the names your layout gave them. Every command here waits " +
      "for the control loop to actually apply it, so the next line runs in a " +
      "world where the last one happened — which is what makes `pulse()` then " +
      "`wait_ready()` work rather than fall straight through.",
    entries: [
      { sig: "rover.mechanisms -> [name, ...]", help: "What this build has, so a script can check before reaching for one." },
      { sig: "rover.mech(name).power(value, actuator=None)", help: "Run it at −1..1.", blocks: true },
      { sig: "rover.mech(name).preset(name)", help: "Apply a named position from the layout (\"up\", \"stow\").", blocks: true },
      { sig: "rover.mech(name).pulse()", help: "Start the one cycle it owns — a kicker's stroke, a sequence's whole run.", blocks: true },
      { sig: "rover.mech(name).stop()", help: "Stop it.", blocks: true },
      { sig: "rover.mech(name).ready -> bool", help: "Idle and able to be asked for something. False mid-cycle." },
      { sig: "rover.mech(name).wait_ready(timeout=10) -> bool", help: "Wait for the cycle to finish. False on timeout, never raises.", blocks: true },
      { sig: "rover.mech(name).state / .activations / .powers / .rpm", help: "What it is doing, how many cycles it has run, and how fast." },
      { sig: "rover.mech(name).spin_for(distance_m) -> bool", help: "Run a flywheel at the speed a shot from that range needs. False — and nothing spins — with no solution.", blocks: true },
      { sig: "rover.mech(name).exists -> bool", help: "Does this build have it? A command on one it hasn't is a logged no-op, not a crash." },
      { sig: "rover.shooter.spin(on=True, rpm=None)", help: "Start or stop the built-in flywheel.", blocks: true },
      { sig: "rover.shooter.spin_for(distance_m) -> bool", help: "Spin at the speed a shot at that range needs.", blocks: true },
      { sig: "rover.shooter.fire()", help: "Push one ball in. The shooter refuses it if it is mid-cycle or cooling down.", blocks: true },
      { sig: "rover.shooter.ready / .spinning / .shots / .state", help: "The launcher's own state." },
    ],
  },
  {
    key: "flow",
    title: "Waiting, timing and output",
    note:
      "`rover.sleep` is the only way to wait — `time.sleep` cannot be " +
      "interrupted, so a script napping for thirty seconds would ignore the " +
      "stop button for thirty seconds while the rover kept driving.",
    entries: [
      { sig: "rover.sleep(seconds)", help: "Wait. Wakes early, and ends the run, if you press Stop.", blocks: true },
      { sig: "rover.time() -> s", help: "Seconds since this run started." },
      { sig: "rover.wait_until(condition, timeout=30) -> s", help: "Poll until it is true; returns how long that took. Raises ScriptTimeout if it never is.", blocks: true },
      { sig: "rover.wait_while(condition, timeout=30) -> s", help: "The mirror of it, for the loops that read better that way.", blocks: true },
      { sig: "print(...)  /  rover.log(...)", help: "Write a line to the console below." },
      { sig: "rover.watch(name, value)", help: "Show a named live value. For numbers a 50 Hz loop would turn into a waterfall." },
      { sig: "except ScriptTimeout:", help: "Catch a wait that gave up — \"if I can't see it in 10s, go look elsewhere\"." },
    ],
  },
  {
    key: "python",
    title: "The Python you get",
    note:
      "Ordinary Python, minus what could only cause trouble on a robot. The " +
      "guard is against mistakes rather than malice — a rover you can push " +
      "documents to is a rover you could already reflash — so it says no " +
      "clearly and tells you what is available instead.",
    entries: [
      { sig: "import math, random, statistics, json, re", help: "Also collections, itertools, functools, dataclasses, enum, decimal and friends." },
      { sig: "import os / sys / socket / threading / time", help: "Refused. Everything about the robot is on `rover`; use rover.sleep and rover.time." },
      { sig: "open() / exec() / eval()", help: "Not available." },
      { sig: "def loop(): ...", help: "Define one and it is called repeatedly at the control rate after your top-level code runs." },
      { sig: "while True: ...", help: "Fine. It is interrupted by Stop, by the e-stop, and by scripts.max_runtime." },
    ],
  },
];

/** Every name the editor highlights as part of the rover API. Derived from the
 *  reference above so the two cannot drift: a call listed in the panel is a
 *  call the editor colours, and adding one to the list is the whole change. */
export const API_NAMES: ReadonlySet<string> = new Set(
  API.flatMap((group) => group.entries).flatMap((entry) =>
    Array.from(entry.sig.matchAll(/\b([a-z_][a-z0-9_]*)\s*(?=\(|\b)/gi))
      .map((m) => m[1])
  ).concat(["rover", "ScriptTimeout", "ScriptAborted"]),
);
