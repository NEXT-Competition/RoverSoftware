# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users

**Primary: a multi-robot fleet operator at the base station.** One person commands
several rovers at once over a single shared XBee channel — monitoring who is live,
who has a fix, who has latched an e-stop — and intervenes on whichever needs it.
They select a robot, drive or re-mode it, then move on to the next. Attention is
divided across the fleet by definition; the station is a watch post as much as a
control.

**Secondary: the builder at the bench.** The same UI brings hardware up before it
ever drives outside — declaring motors, servos and mechanisms, claiming PWM
channels, calibrating ESC endpoints, and jogging one mechanism at a time to see it
move. This is the same person on a different day, not a different audience.

**Primary screen: the 7" Raspberry Pi touchscreen at 800×480**, running the
packaged Chromium kiosk that launches on boot. This is the surface that must be
flawless: touch-only, no keyboard, no room. iPad/tablet, desktop browser with a
gamepad, and the `deno desktop` native window (1280×820) must all work, but the
small kiosk is the one that is actually used.

## Product Purpose

Run and program a fleet of ground rovers from one local base station, over a radio
link, with no internet.

The build is **described from the dashboard rather than compiled in**: declare as
many motors and servos as the robot has, pick a drivetrain (two-motor tank, one
motor plus a steering servo, a single motor, or none), and group the rest into
mechanisms. Then **program it without Python** — the Routines tab is a node-graph
state-machine editor, and the machine it produces runs on the robot itself.

Success is a rover that can be built, calibrated, tuned, programmed and driven
without editing a Python file or restarting a service, and that keeps doing its
job after the radio drops.

## Positioning

Neighboring teleop dashboards send commands to a robot whose capabilities were
fixed at compile time. Here the robot's **structure** crosses the link, not just
its throttle:

- A **layout document** declares what the build has. Actuators are constructed at
  start-up from it, so a new mechanism is a saved document, not a code change.
- **Routines** are finite state machines authored on a canvas and executed on the
  robot, so they survive losing the radio. A state can *delegate* driving to
  object align, shooter align or waypoint — the FSM composes the autonomy that
  already exists rather than reimplementing it.
- Every controller emits the same `DriveCommand(left, right)`, so a steered
  chassis reuses object align and waypoint unchanged.
- `robot/tuning.py` is a whitelist: the dashboard can only reach parameters that
  exist, and values are **clamped rather than refused**, then echoed back so the
  field sees what the robot is actually doing.
- The whole system — including the Hardware and Routines tabs — runs against
  `--sim` using the *real* validators and the *real* state-machine engine. A
  settings page you can only test on a real rover ships broken.

## Operating Context

- **Outdoors, in the field, offline.** Map tiles and fonts are bundled locally;
  satellite imagery is the default basemap so operators navigate by visible
  terrain rather than street names. A local tile server covers true offline use.
- **The radio is the constraint.** Newline-delimited JSON over one shared XBee
  channel; `to` addresses a robot (or `"all"`), robots stamp telemetry with
  `from`. Airtime is scarce — a full config snapshot is ~0.4 s at 57600, so it is
  requested explicitly and never polled. Telemetry runs ~5 Hz, drive ~30 Hz.
- **Documents are not merged.** Config is independent scalars and can be merged;
  a layout is a tree, and half a tree is a robot with one drive motor. Layouts and
  routines are sliced into numbered fragments and nothing is applied until every
  fragment arrives.
- **Deployment is a boot-and-forget appliance.** Robot and base station ship as
  Debian packages with systemd units; the base-station Pi comes up into a
  full-screen kiosk with no login step. Field tuning is saved — on the robot for
  robot settings, on the base station for the rest — and survives power cycling.
- **Input is mixed.** A server-side PS4 pad, the browser Gamepad API, and an
  on-screen joystick all drive the selected robot; the kiosk has touch only.

## Capabilities and Constraints

- **Modes:** `teleop`, `object_align`, `shooter_align`, `waypoint`, `routine`.
  E-stop overrides every mode until explicitly cleared; teleop stops on its own if
  commands stop arriving (`command_timeout`).
- **Settings surfaces:** Tuning (the selected rover's tunables, live over the
  radio), Hardware (layout + mechanisms + bench jog), Routines (the FSM editor),
  Controller (remap by pressing the control you want), Base station (airtime
  budget, refresh rates, basemap, trail length).
- **Restart-gated fields.** Serial ports, PWM channels and enable flags are saved
  but only take effect on the next start, because actuators are built at start-up.
  They are badged `restart` in the UI.
- **Bench safety.** Jogging a mechanism is refused unless the robot is in teleop
  with no e-stop latched. Claiming a PWM channel twice is refused rather than
  silently making two actuators move together.
- **One robot is selected at a time.** The fleet is tracked in full and every
  robot's telemetry is live, but drive input, mode switching, routes and settings
  all target the single selected robot. Commanding two rovers means switching
  between them.
- **Offline by design.** No CDN, no API key required for the default basemap, no
  cloud dependency anywhere in the operating path.
- **Known caveat:** a steered chassis cannot pivot in place, and object align and
  waypoint ask it to (`min_pivot_throttle`, `docs/ARCHITECTURE.md` §4.4).
- **Terminology to preserve:** *layout*, *mechanism*, *routine*, *state*,
  *transition*, *drivetrain*, *actuator*, *jog*, *e-stop*, *teleop*, *waypoint*,
  *fleet*, *base station*, *bridge*, *tunable*.

## Brand Commitments

- The product is called **RoverSoftware**. The `uc-chassis` directory name and the
  `uc-chassis-*.deb` artifacts in `dist/` are stale; the UI brand, README, systemd
  units and Python package are the correct authority.
- It lives under the `NEXT-Competition` GitHub organization.
- Existing assets: `basestation-ui/public/icon.svg` and the PNG/Apple-touch icon
  set, plus the installed PWA manifest.
- No other identity constraints have been set. **No aesthetic direction has been
  established here** — the incumbent look is evidence, not a commitment.

## Evidence on Hand

- `docs/ARCHITECTURE.md` — full end-to-end technical documentation (13 sections).
- `docs/waypoint-navigation.html` — an interactive walkthrough of the navigation
  algorithm.
- A working simulator (`basestation/simulator.py`, `--sim`) that spawns fake
  robots, obeys commands, drives clicked routes, and answers Hardware/Routines
  with the real validators and engine. Anything can be demonstrated with no radio.
- Trained detection models in `model/` (IMX500 `.rpk` pipeline and Edge Impulse
  `.lite` exports) and bring-up tools in `tools/`.
- Real hardware: Raspberry Pi + SunFounder Fusion HAT, ESC-driven motors, XBee
  radios, Adafruit Ultimate GPS, BNO085 IMU, Raspberry Pi AI Camera.
- **Not on hand:** no users, customers, testimonials, benchmarks, adoption
  numbers, pricing, or press. Future work must not invent them.

## Product Principles

1. **The robot is described, not compiled.** Anything about a build that could
   differ between rovers belongs in a document the dashboard can write — never in
   a constant someone has to edit and redeploy.
2. **Assume the radio drops.** Autonomy and routines execute on the robot; the
   base station is an operator's window, not a required participant. Everything
   the operator saves must survive a power cycle.
3. **Clamp, echo, never refuse silently.** Show the operator what the robot is
   actually doing, including when it differs from what they asked for.
4. **The fleet is the subject.** State that an operator checks constantly must be
   readable without selecting a robot; selection is for detail and command.
5. **If it can't be exercised without hardware, it isn't finished.** The simulator
   path is a shipping requirement, not a testing convenience.
6. **It has to work on the 7" panel, outdoors, with a thumb.** Density that only
   resolves at 1280px wide is density the primary operator never gets.

## Accessibility & Inclusion

- Touch-first: large tap targets (the theme sets a 52px tap floor), no hover-only
  affordances, no keyboard requirement on the kiosk.
- Must remain usable at 800×480 and adapt to landscape/portrait with iPad
  safe-area insets.
- Safety-critical controls (E-STOP, mode, latched state) must be unmistakable and
  never rely on color alone.
- No formal conformance standard has been adopted as a requirement.
