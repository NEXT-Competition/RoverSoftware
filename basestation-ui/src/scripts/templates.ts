// Starter scripts.
//
// The same argument the routine editor's TemplatePicker makes: an empty canvas
// is the hardest thing to start from, and the first thing anybody writes is a
// version of one of these anyway. Every one runs as-is on a rover with no
// camera, no GPS and no mechanisms — they check before reaching for anything —
// so "press New, press Run, watch it move" is true on a bare chassis.
//
// They are also the documentation that gets read. Nobody opens a reference
// panel to find out that `rover.sleep` is the only sleep; they find out because
// the template they copied used it.

export interface Template {
  key: string;
  name: string;
  /** One line, shown on the card. What this is FOR, not what it contains. */
  blurb: string;
  code: string;
}

export const TEMPLATES: Template[] = [
  {
    key: "hello",
    name: "Drive a square",
    blurb: "The first program. Four sides and four turns, with nothing but the drivetrain.",
    code: `# Drive a square. Nothing here needs a camera, a GPS or a mechanism.
#
# Press Run and watch the map. Press Stop at any point — the rover halts
# wherever it is, mid-side or mid-turn.

SIDE_SECONDS = 2.0
SPEED = 0.35

for corner in range(4):
    print("side", corner + 1)
    rover.forward(SPEED, seconds=SIDE_SECONDS)

    # turn_to is better than "turn for N seconds" whenever there is a
    # heading to steer on: it lands on the angle instead of on however far
    # the rover happened to get in the time.
    heading = rover.heading()
    if heading is None:
        rover.turn(0.4, seconds=1.2)   # no IMU and no GPS course: time it
    else:
        rover.turn_to(heading + 90)

print("back where we started, more or less")
`,
  },
  {
    key: "approach",
    name: "Creep up to something",
    blurb: "Drive forward until the ultrasonic says something is close, then stop. No model needed.",
    code: `# Creep forward until something is in front of us, then stop short of it.
#
# This uses the ultrasonic, which needs no model and no calibration — and
# knows nothing about WHAT it is looking at. That makes it right for "until
# something is close" and wrong for "until the bucket is close" in a room
# with a chair in it.

STOP_AT = 0.4     # metres
CREEP = 0.22

if rover.distance_ahead() is None:
    print("no ultrasonic on this build — nothing to creep towards")
else:
    while True:
        ahead = rover.distance_ahead()

        # None means the echo never came back: too far, too soft, or too
        # angled. Keep creeping rather than treating "don't know" as "clear".
        rover.watch("ahead", "?" if ahead is None else round(ahead, 2))

        if ahead is not None and ahead <= STOP_AT:
            break

        # Ease off as it gets close, so the last few centimetres are not
        # taken at full speed.
        near = 1.0 if ahead is None else min(1.0, max(0.3, ahead / 1.5))
        rover.forward(CREEP * near)
        rover.sleep(0.05)

    rover.stop()
    print("stopped at", round(rover.distance_ahead() or 0, 2), "m")
`,
  },
  {
    key: "vision",
    name: "Find and approach a target",
    blurb: "Sweep until the camera sees something, steer onto it, then hand over to the real aligner.",
    code: `# Look for something, then let the rover's own alignment controller drive
# up to it.
#
# The search here is ours; the approach is not. object_align already has the
# PID, the standoff and the dwell rule — writing a second approach loop would
# be a second, subtly different answer to a question that already has one.

TARGET = "bucket"

rover.look_for(TARGET)

# Sweep, in short bursts, watching between them. A continuous spin outruns
# the detector: at 10 fps a fast turn moves the target across the frame
# between one inference and the next.
found = False
for attempt in range(12):
    if rover.vision.seen:
        found = True
        break
    rover.turn(0.35, seconds=0.25)
    rover.sleep(0.35)

if not found:
    print("no", TARGET, "in view after a full sweep")
else:
    print("saw it", round(rover.vision.bearing or 0, 1), "degrees off centre")

    # Hand over. This blocks until the aligner says it has arrived, or gives
    # up after the timeout; either way we get the wheel back.
    if rover.align_to(TARGET, within_m=1.0, timeout=25):
        print("arrived at", round(rover.align.distance or 0, 2), "m")
    else:
        print("could not get there — lost it, or something is in the way")
`,
  },
  {
    key: "mechanism",
    name: "Work a mechanism",
    blurb: "Run an intake, fire a kicker, wait for its cycle — the pattern every actuator script starts from.",
    code: `# The mechanism pattern: check it exists, command it, wait for it.
#
# Names come from your Hardware layout. rover.mechanisms lists what this
# build actually has, which is what makes a script written for the rover
# with an arm still run on the one without.

print("this build has:", ", ".join(rover.mechanisms) or "no mechanisms")

intake = rover.mech("intake")
kicker = rover.mech("kicker")

if intake.exists:
    intake.power(0.8)          # returns once the control loop has applied it
    rover.sleep(1.5)
    intake.stop()

if kicker.exists:
    for shot in range(3):
        kicker.pulse()

        # ready goes False while the cycle runs. Without this wait the next
        # pulse would be refused mid-stroke and the loop would fire once.
        if not kicker.wait_ready(timeout=5):
            print("kicker did not finish its cycle — stopping here")
            break
        print("shot", shot + 1, "of 3")

rover.stop_all()
`,
  },
  {
    key: "loop",
    name: "Steer on a measurement (loop)",
    blurb: "Define loop() and it runs every control tick — the shape for anything that steers continuously.",
    code: `# Anything that steers on a live measurement wants to run every tick.
#
# Define loop() and the runner calls it at the control rate. Top-level code
# runs once first, so it is a setup step without needing the name.

GAIN = 0.9          # how hard to correct. Too high and it wags.
CRUISE = 0.25

rover.look_for("cone")
print("following whatever cone I can see — press Stop to end")

lost_since = None


def loop():
    global lost_since

    if not rover.vision.seen:
        # Coast briefly on a dropout — a single missed frame is not a lost
        # target — then stop rather than driving on a bearing from a second ago.
        if lost_since is None:
            lost_since = rover.time()
        if rover.time() - lost_since > 0.6:
            rover.stop()
        return

    lost_since = None
    offset = rover.vision.offset or 0.0
    rover.watch("offset", round(offset, 3))
    rover.watch("range", rover.vision.distance)

    # Slow down as it gets close, and stop steering when it is centred
    # enough that the correction would just be noise.
    steer = GAIN * offset if abs(offset) > 0.04 else 0.0
    rover.arcade(CRUISE, steer)
`,
  },
];
