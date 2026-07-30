# RoverSoftware — Technical Documentation

How the whole system works, end to end: the tank-drive robot, the base-station
dashboard, the radio protocol, GPS waypoint autonomy, and offline maps. For
usage/quick-start see the top-level [`README.md`](https://github.com/NEXT-Competition/RoverSoftware/blob/main/README.md); for a visual
walkthrough of the navigation algorithm open
[`docs/waypoint-navigation.html`](./waypoint-navigation.html).

## Table of contents

1. [Design principles](#1-design-principles)
2. [System topology](#2-system-topology)
3. [Repository layout](#3-repository-layout)
4. [The robot](#4-the-robot)
   - [Configuration](#41-configuration-robotconfigpy)
   - [Hardware layout](#41b-hardware-layout-robotlayoutpy)
   - [The orchestrator & control loop](#42-the-orchestrator--control-loop-robotpy)
   - [Control layer](#43-control-layer)
   - [Drive layer](#44-drive-layer)
   - [Mechanisms](#44b-mechanisms-drivemechanismpy)
   - [Routines: the FSM engine](#44c-routines-the-fsm-engine-robotroutine)
   - [Comms layer](#45-comms-layer)
   - [Sensors](#46-sensors)
5. [Wire protocol](#5-wire-protocol-xbee)
6. [GPS waypoint autonomy](#6-gps-waypoint-autonomy)
7. [The base station](#7-the-base-station)
7b. [Commanding in words: voice, and MCP](#7b-commanding-in-words-voice-and-mcp)
8. [Offline maps](#8-offline-maps)
9. [Threading model](#9-threading-model)
10. [Configuration reference](#10-configuration-reference-env-vars)
11. [Deployment](#11-deployment)
12. [Running & testing without hardware](#12-running--testing-without-hardware)
13. [Safety mechanisms](#13-safety-mechanisms)

---

## 1. Design principles

Four ideas recur throughout the codebase; understanding them explains most of
the structure.

**One command type.** Every controller — teleop, object-align, waypoint — returns
the same `DriveCommand(left, right)`. The drive layer never knows who produced a
command, so new autonomy modes drop in without touching motors or comms.

**Injected sensing.** Autonomy controllers don't open cameras or GPS receivers
themselves. They receive a *provider callable* (`pose_provider()`,
`target_provider()`) and call it each tick. Sensing is wired in at the top level
(`robot.py`), so the decision logic runs and unit-tests with no hardware.

**Graceful degradation.** Anything hardware-dependent is guarded so the stack
still runs on a laptop: no Fusion HAT → mock servos; no `adafruit_gps`/serial → GPS
disabled; no gamepad → touch-only. A missing piece prints one line and the rest
keeps running.

**Threads feed a single loop.** I/O that blocks (serial read, UDP receive,
gamepad poll) runs on background threads. They hand data to the main control loop
through thread-safe queues or locked state, so controller logic is single-
threaded and never races.

---

## 2. System topology

```
  ┌─────────────────────── BASE STATION (Pi or Mac) ───────────────────────┐
  │  run_basestation.py                                                     │
  │    FleetManager      ← telemetry ─────────────────┐                     │
  │    FastAPI app.py    → commands ─────────────────┐│                     │
  │      /ws   (browser control + fleet snapshots)   ││                     │
  │      /tiles/{z}/{x}/{y}.png  (offline map)       ││                     │
  │    ControllerReader (pygame gamepad)             ││                     │
  └───────────────────────────────────────────────────┼┼─────────────────────┘
             ▲ browser (Leaflet map)                   ││
             │ WebSocket / tiles                        ││ XBee (JSON/serial)
             ▼                                          ││ commands + telemetry
  ┌──────────────────────────── ROBOT (Pi) ───────────┼┼─────────────────────┐
  │  run_robot.py → Robot (50 Hz loop)                 ││                     │
  │    XBeeLink (serial reader thread) ─→ queue ─────→ ControlManager        │
  │    GPS (NMEA reader thread) ─→ pose_provider ─→ WaypointController        │
  │    ControlManager → active Controller → DriveCommand → TankDrive → ESCs  │
  └─────────────────────────────────────────────────────────────────────────┘
```

The robot and base talk over the **XBee radio** — reliable-ish serial carrying
small JSON control/telemetry messages. One shared channel serves the whole fleet;
messages are addressed with a `to` field.

---

## 3. Repository layout

```
robot/                        # runs on the rover Pi (also imported by the base station)
  config.py                   all tuning: actuators, drive, mechanisms, comms, GPS
  layout.py                   the hardware layout document (what this build HAS)
  robot.py                    orchestrator + the 50 Hz control loop
  control/
    controller.py             Controller base class (on_activate/on_message/update)
    manager.py                ControlManager: mode arbitration + e-stop
    commands.py               DriveCommand (tank / arcade / stopped)
    pid.py                    reusable PID with output + integral clamping
    teleop.py                 drive from base-station commands (+ link failsafe)
    object_align.py           Edge Impulse object alignment — inject a detection provider
    shooter_align.py          object_align + a trigger: align, settle, fire
    detection.py              the Detection contract the controller consumes
    waypoint.py               GPS waypoint navigation — inject a pose provider
    routine_controller.py     the `routine` mode: runs a UI-authored state machine
    rpm_trim.py               closed-loop wheel speed: hold the two tracks together
  routine/                    the FSM engine (its documents live in routines.json)
    schema.py                 parse + validate a document into compiled Routines
    conditions.py             what a transition may ask about the robot
    actions.py                what a state may do (never the drivetrain)
    engine.py                 which state is current, and when that changes
    store.py                  routines.json
  drive/
    motor.py                  ESCMotor: throttle [-1,1] → servo angle (mock if no HAT)
    drivetrain.py             tank / servo_steer / single / none + slew limiting + wheel-speed trim
    tank_drive.py             TankDrive — the name the tools and tests import
    mechanism.py              intakes, arms, launchers: power and pulse kinds
    shooter.py                the built-in launcher: non-blocking fire/retract cycle
  comms/
    protocol.py               newline-delimited JSON encode/decode
    xbee_link.py              threaded transparent-mode XBee serial reader
    doc_transfer.py           split/reassemble a whole document across frames
  sensors/
    encoder.py                quadrature wheel encoders → signed RPM (Fusion HAT pins)
    gps.py                    Adafruit GPS reader → (lat, lon, track angle)
    bno085.py                 BNO085 IMU → absolute heading + yaw rate
    pose.py                   GPS position + IMU-or-track-angle heading → pose()
    camera.py                 shared frame capture (IMX500 → picamera2 → OpenCV → none)
    detector.py               Edge Impulse .eim runner → Detection
    imx500.py                 Sony IMX500 on-sensor detection → Detection
    fpv.py                    JPEG-over-UDP live video → base station
  tuning.py                   whitelist of remotely-settable parameters
run_robot.py                  robot entry point (env + CLI → RobotConfig)

basestation/                  # cross-platform dashboard (Pi or Mac)
  app.py                      FastAPI: UI, WebSocket bridge, tiles route
  fleet.py                    FleetManager: per-robot state from telemetry
  settings.py                 gamepad mapping + link/UI rates, persisted
  tiles.py                    TileStore: MBTiles cache + online fallback
  simulator.py                fake fleet (drop-in for the radio)
  controller_input.py         pygame gamepad reader → selected robot
  teleop_sender.py            minimal standalone PS4 → JSON sender (dev tool)
  static/                     Leaflet dashboard (index.html, app.js, style.css)
run_basestation.py            dashboard entry point

tools/
  esc_calibrate.py            interactive ESC bring-up
  servo_sweep.py              raw servo sweep (Fusion HAT hello-world)
  xbee_monitor.py             watch/inject XBee frames to verify the link
  fetch_tiles.py              download a bbox/radius into an .mbtiles cache

packaging/                    .deb build + systemd units + env conffiles
Justfile                      build / deploy / sync / service control recipes
docs/                         this file + the waypoint presentation
```

---

## 4. The robot

### 4.1 Configuration (`robot/config.py`)

All hardware wiring and tuning lives in nested `@dataclass`es so the rest of the
code stays generic. `run_robot.py` constructs a `RobotConfig`, overrides fields
from environment variables / CLI flags, and hands it to `Robot`.

| Dataclass | Key fields (defaults) | Purpose |
|---|---|---|
| `MotorConfig` | `channel`, `inverted`, `neutral_angle`, `max_angle`, `min_angle`, `deadband=0.03`, `max_forward/reverse=1.0`, `name`, `kind="esc"` | One actuator's calibration: maps throttle to servo angle. `name` is how a layout, a mechanism and a tuning path all refer to it. |
| `DriveConfig` | `kind="tank"`, `actuators` (a dict, `left`→ch0 and `right`→ch1 by default), `roles`, `arm_seconds=2.0`, `slew_rate=4.0`, `steer_gain`, `min_pivot_throttle=0.15` | The drivetrain: any number of named actuators, plus who plays which role. |
| `MechanismConfig` | `name`, `kind` (`power`\|`pulse`), `actuators`, `presets`, plus the pulse geometry | A non-drivetrain subsystem: an intake, an arm, a second launcher. |
| `RoutineConfig` | `state_timeout_default=60`, `allow_arm=False` | What a UI-authored state machine is allowed to do. |
| `CommsConfig` | `port="/dev/ttyUSB0"`, `baud=57600`, `command_timeout=0.5` | XBee serial + teleop failsafe window. Must match the base station and the radios' `BD`. |
| `ShooterConfig` | `enabled=False`, `channel=2`, `rest_angle=-30`, `fire_angle=30`, `fire_seconds=0.35`, `retract_seconds=0.35`, `dwell=0.5`, `cooldown=2.0`, `require_arm=True`, `require_arrived=True`, `max_shots=0` | Servo launcher geometry + the firing policy for `shooter_align`. Off by default. |
| `GPSConfig` | `enabled`, `port="/dev/ttyAMA0"`, `baud=57600`, `fix_timeout=5.0`, `min_move_mps=0.5`, `update_rate_ms=200` | Adafruit Ultimate GPS reader settings. 5 Hz fixes; the baud is raised over `PMTK251` to carry them. |
| `PIDConfig` | `kp`, `ki`, `kd`, `out_limit`, `i_limit` | Gains for one loop, so they are tunable rather than edit-and-redeploy constants. |
| `AlignConfig` | `forward_speed=0.25`, `pivot_threshold=0.25`, `aligned_tolerance=0.05`, `search_after=0.5`, `search_timeout=10`, `pid` | The `object_align` / `shooter_align` state machine. |
| `NavConfig` | `arrive_radius_m=2.0`, `cruise_speed=0.35`, `acquire_speed=0.4`, `pivot_threshold_deg=25`, `heading_pid`, `gps_heading_pid` | Waypoint navigation. Two heading loops: `heading_pid` for an absolute IMU heading, the slower `gps_heading_pid` for a GPS course over ground. |
| `RobotConfig` | `drive`, `comms`, `gps`, `align`, `nav`, `loop_hz=50`, `start_mode`, `robot_id`, `telemetry_hz=5`, `heading_source="auto"` | Top-level composition. `heading_source`: `auto` (IMU, else the GPS track angle) \| `gps` \| `imu`. |

> **ESC-as-servo mapping.** An ESC takes the same PWM as a servo — neutral pulse
> = stop, longer = forward, shorter = reverse. Throttle `-1..+1` maps onto servo
> angle `min_angle..max_angle` with `neutral_angle` as stop. On the Fusion HAT,
> `Servo.angle()` runs roughly `-90..+90` with `0` at neutral. Calibrate with
> `tools/esc_calibrate.py` and copy the endpoints into `config.py`.

**Live tuning (`robot/tuning.py`).** The dashboard can change most of the above
over the radio while the robot is running. `RobotConfig` is the right shape for
code but the wrong one for a remote form — a browser must not be able to poke
arbitrary attributes on it, and a slider needs to know a field's range — so
`tuning.py` is the whitelist. Each parameter is declared once with its type,
bounds and whether it applies live; `snapshot(cfg)` reads them all out as flat
dotted paths and `apply(cfg, updates)` writes them back. Unknown paths are
rejected, numbers are **clamped rather than refused** (a slider pinned at its
limit is honest; silently dropping the update looks like a broken UI), and a
malformed frame is reported instead of raised — nothing off a radio may take
the robot down.

`live=True` is a claim about the code, not a wish. It holds either because the
consumer re-reads `cfg` on every use (the motors, the shooter servo, the
detector, the FPV streamer, the loop rates) or because `Robot._push_live_config`
copies the value into the object that cached it at construction (the
controllers, the IMU, the GPS). Retuning a PID copies the gains without
resetting the integrator: changing a gain mid-run should nudge the loop, not
make the robot forget where it was pointing. `live=False` marks anything owned
by a constructor — serial ports, I²C addresses, PWM channels, enable flags — and
those are stored, badged in the UI, and applied on the next start.

**The parameter surface depends on the layout.** `PARAMS` is what a stock build
ships with; `params_for(cfg)` is what *this* robot exposes, derived from its own
actuators and mechanisms. The whole backwards-compatibility story is one line of
set equality, and it has its own test:

```
{p.path for p in params_for(RobotConfig())} == {p.path for p in PARAMS}
```

The default layout names its two actuators `left` and `right`, so the derived
paths *are* `drive.left.*` and `drive.right.*`. Every deployed `tuning.json`, the
hand-written `schema.ts` mirror, and the snapshot-key assertions in the test
suite keep working without knowing any of this happened — held up by
`DriveConfig.__getattr__`, which resolves an unknown attribute to the actuator of
that name so `tuning._resolve`'s plain `getattr`/`setattr` still reaches it.

The dashboard cannot mirror an actuator the operator invented ten seconds ago, so
the robot describes those itself: `descriptors(cfg)` emits field metadata for the
*derived* parameters only. The ~90 static ones are already in `schema.ts`, and
restating them would put 2 KB on a shared radio for something the browser has.

Applied values are persisted on the robot (`RS_TUNING_FILE`, default
`/var/lib/roversoftware/tuning.json`) and re-applied at boot *after* env and CLI:
they are the operator's most recent deliberate decision, and the paths they can
reach are disjoint from the wiring flags (no CLI flag names a PID gain). A
corrupt tuning file is ignored, not fatal — it must leave the robot bootable on
its compiled-in defaults rather than bricked at the side of a field.

### 4.1b Hardware layout (`robot/layout.py`)

`tuning.py` is the surface for SCALARS — a gain, a limit, a timeout, each at a
fixed dotted path. That model cannot express "this build has three intake
motors", because the set of paths itself depends on the answer. So structure
lives in its own versioned JSON document:

| File | Env | Holds |
|---|---|---|
| `layout.json` | `RS_LAYOUT_FILE` | what actuators exist, the drivetrain kind, the mechanisms |
| `tuning.json` | `RS_TUNING_FILE` | what their numbers currently are |
| `routines.json` | `RS_ROUTINES_FILE` | the state machines (§4.8) |

Two files rather than one, because they are edited at completely different
cadences (a layout changes when someone reaches for a screwdriver; a gain changes
every few minutes on a field), they fail differently (a bad layout stops the
rover driving, a bad gain makes it drive badly), and they get separate revision
counters so saving one does not push the other over the radio.

Precedence at boot: **compiled-in defaults -> env/CLI -> `layout.json` ->
`tuning.json`.** The layout comes before tuning because it decides which tuning
paths even exist. No layout file leaves the compiled-in two-motor tank drive
exactly as it was, which is the whole migration story for a deployed rover.

**Validation is code now, not a comment.** Unique actuator names, names that
don't collide with a `DriveConfig` field (they would be unreachable through
`__getattr__`), channels in 0-15 claimed exactly once, roles that name real
actuators, and caps of 16 actuators / 8 mechanisms / 6 KB serialized. A channel
fight is resolved **drivetrain-first**, then mechanisms in declaration order,
then the built-in launcher; the loser is reported and its mechanism disabled
rather than being fatal — a robot that can't drive can't be driven away from
whatever it is about to hit. Nothing raises: a bad layout leaves the rover on its
previous wiring, bootable and drivable.

**A layout never hot-swaps.** It is stored, validated, and answered with
`restart_required`. Actuators are owned by constructors: swapping them mid-loop
means destroying and rebuilding `Servo` objects while the drivetrain is armed,
which is how an ESC ends up holding an undefined pulse. That is the same contract
every `live=false` tuning field already has, and it is why every PWM channel was
already marked that way.

> The built-in launcher is deliberately *not* expressed as a mechanism. It keeps
> its own `ShooterConfig` so the `RS_SHOOTER_*` vars, the `shooter.*` tuning
> paths and `ShooterAlignController`'s firing policy stay exactly as they are;
> `Robot` registers it in the mechanism registry under the reserved name
> `shooter`, and layout validation refuses a user mechanism of that name so two
> things can never answer to it.

### 4.2 The orchestrator & control loop (`robot/robot.py`)

`Robot` wires hardware, comms, and control together, then runs a fixed-rate loop.

**Construction** builds the drive layer, the default controller set
(`teleop`/`object_align`/`waypoint`), the `ControlManager`, the `XBeeLink` (whose
reader thread pushes decoded messages into a `queue.Queue`), and the optional
`GPS`. If GPS is enabled, its `pose()` is injected both into the
`WaypointController` (for navigation) and stored as `self.pose_provider` (for
telemetry).

**`run()`** — the control loop, at `loop_hz` (50 Hz):

```
arm ESCs → start link/GPS threads → install SIGINT/SIGTERM handlers
loop every 1/50 s:
    _drain_inbox()          # pull queued XBee messages, filter by `to`, dispatch
    cmd = manager.update(dt) # active controller decides; e-stop overrides
    drive.drive(cmd.left, cmd.right)
    if telemetry due (telemetry_hz): link.send(_telemetry(cmd))
    sleep the remainder of the period
finally: shutdown() → motors neutral, stop link/GPS
```

`_drain_inbox()` drops any message whose `to` isn't this robot's `robot_id` or
`"all"` — that's how one shared XBee channel serves a whole fleet.
`_telemetry()` includes `lat`/`lon`/`heading` only when `pose_provider()` returns
a fix, which is what makes the robot appear on the base-station map.

### 4.3 Control layer

**`Controller` (base class).** Five hooks: `on_activate()`, `on_deactivate()`,
`on_message(msg)`, `on_estop()`, and `update(dt) → DriveCommand | None`. `update`
is called every tick; returning `None` means "hold/stop". `on_estop()` is
broadcast to **every** controller when the latch engages — stopping the motors
needs no cooperation (the manager just stops calling `update`), so this hook is
purely for state that must not survive an e-stop, like an armed shooter.

**`ControlManager` (`manager.py`).** The single place that decides *who* drives.
It owns the mode and the e-stop latch:

- `handle_message(msg)` routes by type: `estop`/`clear_estop` toggle the latch,
  `mode` switches the active controller (firing `on_deactivate`/`on_activate`),
  everything else is forwarded to the active controller's `on_message`.
- `update(dt)` returns `DriveCommand.stopped()` if e-stopped; otherwise the
  active controller's command (or `stopped()` if it returned `None`).

**`DriveCommand` (`commands.py`).** A frozen dataclass of `left`/`right` in
`[-1, 1]`, with constructors:
- `tank(left, right)` — direct, clamped.
- `arcade(throttle, steer)` — `left = throttle + steer`, `right = throttle − steer`, clamped.
- `stopped()` — zeros.

**`PID` (`pid.py`).** Reusable, with output clamping (`out_limit`) and integral
anti-windup (`i_limit`). `update(error, dt)` returns the clamped `kp·e + ki·∫e +
kd·de/dt`; `reset()` clears the integrator and last-error.

**Controllers:**

- **`TeleopController`** — caches the last `drive` message and returns it each
  tick, **but** returns `stopped()` if no command has arrived within
  `command_timeout` (0.5 s). This is the link failsafe: lose the radio, stop the
  robot.
- **`ObjectAlignController`** — vision autonomy. Calls an injected
  `detection_provider() → Detection | None`, runs a PID on the target's
  horizontal error, and pivots → approaches → stops at a standoff. Sweeps to
  reacquire a lost target, then gives up. Holds still with no provider/target.
  See [§6](#6-gps-waypoint-autonomy) for the detector it's fed by.
- **`ShooterAlignController`** — `ObjectAlignController` **subclass** that adds a
  trigger. It inherits the alignment state machine wholesale (a forked copy would
  drift out of sync with fixes to the subtle timing logic) and asks one extra
  question per tick: should we shoot now? See [§6](#6-gps-waypoint-autonomy).
- **`WaypointController`** — GPS navigation; see [§6](#6-gps-waypoint-autonomy).
- **`RoutineController`** — the `routine` mode. Runs a state machine an operator
  drew in the dashboard, delegating each state's driving to one of the
  controllers above rather than reimplementing it. See §4.4c.

### 4.4 Drive layer

**`Drivetrain` (`drivetrain.py`).** `drive(left, right)` clamps, applies an
optional **slew-rate limit** (`slew_rate` units/second — caps how fast a speed
can change so you don't slam the drivetrain), and forwards to the actuators the
layout named. `arm()` holds every ESC at neutral for `arm_seconds` so they
recognise the signal before commands flow; servo-kind actuators are parked
without the wait, since there is nothing there to arm. `stop()` zeros everything.
`TankDrive` is still the tank drivetrain under its old name, constructed from a
`DriveConfig` exactly as before.

`DriveCommand(left, right)` remains the one command type in the system, and this
module is the only thing that knows what the robot is built like. That is what
lets a one-motor-and-a-steering-servo rover reuse teleop, object_align and
waypoint completely unchanged.

| Kind | What it does with `(left, right)` |
|---|---|
| `tank` | slew-limits each side, fans it out to every actuator on that side |
| `servo_steer` | recovers `throttle = (l+r)/2` and `steer = (l−r)/2` — the exact inverse of `DriveCommand.arcade` — drives the wheels at throttle and the servo at steer, each slew-limited separately |
| `single` | `throttle = (l+r)/2`; steering is discarded (logged once, never per tick) |
| `none` | accepts commands and moves nothing (a build that is only mechanisms) |

> **A steered chassis cannot pivot, and the autonomy modes ask it to.** A
> differential-drive controller expresses "point at it, then go" as
> `arcade(0.0, steer)` — see `object_align.py` and `waypoint.py`. On a tank that
> turns in place; on a steered chassis it is throttle zero with the wheels
> turned, so the robot doesn't rotate and `object_align` would steer at a cone
> until its search timeout. `min_pivot_throttle` (default 0.15) creeps forward
> whenever steering is commanded with no throttle, so the steering has authority.
> This is a **mitigation, not a fix** — proper Ackermann autonomy would need the
> controllers themselves to plan arcs, which they don't. The Hardware tab warns
> when a `servo_steer` layout is saved on a robot whose start mode is an align
> mode.

**`ESCMotor` (`motor.py`).** Wraps one PWM channel. `set_throttle(t)`:
1. clamps to `[−1, 1]`, applies per-motor `inverted`;
2. inside `deadband` → neutral;
3. otherwise scales by `max_forward`/`max_reverse` and maps onto a **symmetric
   throw about `neutral_angle`** — an equal swing on each side,
   `throw = min(max_angle − neutral_angle, neutral_angle − min_angle)`. Using the
   same swing both ways means an off-center neutral doesn't desync the normal vs
   inverted motor: both start at the same throttle and match speed. (`max_angle`/
   `min_angle` are the endpoints; the side closer to neutral sets the throw.)

**Wheel encoders and the speed trim (`sensors/encoder.py`, `control/rpm_trim.py`).**
Everything above commands a *throttle*, and a throttle is an open-loop wish.
`DriveCommand(0.5, 0.5)` says "both sides, half power"; it does not say "both
sides, the same speed", and on real hardware those are different sentences —
different ESCs, different gearbox friction, weight off centre, one track on
grass. The rover drives a slow arc while every number in the system reports a
straight line.

A quadrature encoder on two Pi GPIO pins measures what the wheels actually did.
`Encoder` decodes all four edges of each cycle (X4) through a transition table,
counting only moves it can attribute a direction to — a diagonal jump means two
edges arrived unseen, and inventing a direction for it would bias the rate. Speed
is counts over a `rpm_window`, optionally smoothed by `rpm_tau`. The pins are
the Fusion HAT's digital pins, read through the same `fusion_hat` library that
drives the motors — no second GPIO package and no daemon. `Pin` does the setup;
edge callbacks are registered *without* the bouncetime `Pin.irq()` would impose,
because a 20 ms debounce discards nearly every edge a wheel encoder produces.
The import is optional, and without it every encoder is inert, `rpm()` returns
`None`, and the drivetrain runs open-loop exactly as it always did.

`RpmTrim` closes the loop, in the tank drivetrain only, **after** the slew
limiter — the limiter shapes the operator's intent, the trim corrects what the
hardware then did with it, and rate-limiting a correction would only add lag.

| Mode | Error it closes on | Calibration needed |
|---|---|---|
| `off` | none — RPM is still measured and reported | none |
| `match` | the difference between the two sides, in the commanded direction; split half to each | **none at all** — a shared scale factor cancels out of a difference |
| `velocity` | each side against `throttle × max_rpm`, independently | `max_rpm`, measured by driving flat out and reading telemetry |

`match` engages only while the two sides are commanded within
`straight_tolerance` of each other: a commanded turn is a difference you asked
for, and correcting it would fight the steering. It **holds** its integral
across a turn rather than resetting, because a mismatch between two motors is a
physical property that is still true after the corner.

> **It fails open, and that is the design.** `rpm()` returning `None` means "no
> measurement", never "0 rpm" — a speed loop that cannot tell those apart will
> integrate against a dead sensor and pin that side at full throttle. On top of
> that, a wheel commanded above `min_throttle` for `stall_seconds` with the
> encoder still reading a standstill latches a fault: the loop opens, the
> dashboard says which side, and only `stop()` clears it — so an encoder that
> came loose cannot re-arm the loop while the rover is still moving.

Gains are small because the error is in RPM, the same way `nav.heading_pid`'s are
small because its error is in degrees. The **integral** carries the correction
here (a pair of mismatched motors is a constant bias, which is exactly what an
integrator cancels and a proportional term can only half-fix), `kd` is 0 because
its input would be a differenced noisy measurement, and `out_limit` is the whole
authority the loop has over the drivetrain — 0.2 by default, because a trim is a
correction, not a second throttle. The loop publishes a `pid_trace` under
`drive.trim.pid` on the same switch as the others, so the settings page graphs it.

**Mock fallback.** If `fusion_hat` can't be imported *or* `RS_MOCK_MOTORS=1`,
`ESCMotor` uses a `_MockServo` that just records the last angle. This is what
lets the whole stack run on a laptop — the control/comms/telemetry logic is
exercised with no HAT attached.

### 4.4b Mechanisms (`drive/mechanism.py`)

Everything that moves but doesn't drive: an intake, an arm, a second launcher.
This is `Shooter` generalized, and it keeps that module's two rules because both
were load-bearing.

- **`PowerMechanism`** holds a value across one or more actuators. A named
  *preset* maps actuator → value (`in` = roller 1.0, belt 0.8), which is what a
  routine addresses by name so it reads "intake → in" rather than a column of
  numbers. A preset zeroes the actuators it doesn't mention: it describes the
  whole mechanism's state, and a belt still running because the *previous* preset
  named it is a surprise nobody wants near their hands.
- **`PulseMechanism`** runs a timed `rest → active → recovering` cycle on
  wall-clock deadlines, non-blocking for exactly the reason `shooter.py` gives —
  a `sleep(0.3)` would freeze the 50 Hz loop, trip the slow-tick watchdog, and
  hold the drive outputs at whatever they last were. It owns its own cycle, so
  something asking every tick gets one activation per cycle instead of needing a
  timer. `fire()` is an alias for `activate()`, so a user-declared launcher
  satisfies `ShooterLike` and drops into `shooter_align` untouched.

Two details worth stating because they are easy to get wrong:

**Unchanged writes are elided.** A routine can hold an action every tick. Writing
an unchanged value would cost one I²C transaction per actuator per tick — 300 a
second on a six-actuator rover, inside a 100 ms tick budget. The drivetrain
deliberately does *not* do this: its slew limiter changes the value nearly every
tick anyway, and a drivetrain that stops writing is one whose failsafe stopped
ticking.

**The e-stop had to be extended to reach them.** `ControlManager` broadcasts
`on_estop()` to *controllers* and forces `stopped()` out of `update()`, which
covers the drivetrain completely — but a mechanism is not a controller, so an
intake at full power would have kept spinning through the one button that exists
to stop it. `Robot.run()` edge-detects the latch and stops every mechanism.
Edge-detected rather than held, so an operator can still jog a mechanism while
the robot is safely stopped, which is what bring-up looks like.

### 4.4c Routines: the FSM engine (`robot/routine/`)

Sequencing an action used to mean writing a `Controller` subclass in Python.
`ControlManager` is a flat switch over hand-written modes, and the only ordered
thing in the repo was `WaypointController`'s list of legs. A routine is a state
machine an operator draws in the dashboard, validated and compiled once on
arrival, and run on the **robot** as a fifth mode — so it survives losing the
radio, which is the reason it doesn't live on the base station.

```jsonc
{ "id": "collect", "start": "seek", "on_end": "stop", "on_estop": "abort",
  "states": [
    { "id": "seek",
      // DELEGATE, don't reimplement. `target` is what to align to and
      // `stop_within_m` is how near to get — both BORROWED from the controller
      // and handed back when the state is left.
      "drive": {"mode": "object_align", "target": "bucket", "stop_within_m": 1.5},
      "on_enter": [{"do": "mech_preset", "mech": "intake", "preset": "in"}],
      "transitions": [{"when": "arrived", "for_seconds": 0.4, "to": "shoot"},
                      {"when": "elapsed", "seconds": 20, "to": "done"}] },
    { "id": "shoot", "drive": {"mode": "shooter_align"},
      "on_enter": [{"do": "arm"}],
      "transitions": [{"when": "shots", "at_least": 2, "to": "done"}],
      "on_exit":  [{"do": "disarm"}] },
    { "id": "done", "terminal": true } ] }
```

**Delegation is the design.** A state's `drive` is `stop`, `hold`, `manual`, or
the name of a controller — and naming one hands the tick to the *real* instance,
the one `Robot.__init__` already injected a detection or pose provider into. The
FSM composes the autonomy that exists rather than re-expressing it as
user-editable blocks. `RoutineController` owns the delegate's lifecycle
(`on_activate` on entry, `on_deactivate` on exit), because from `ControlManager`'s
view every non-active controller is inactive; it syncs *before* evaluating
transitions, since conditions like `aligned` read state the delegate publishes
from its own `update()`.

**An aiming state borrows, it does not take.** `target` (which detector class)
and `stop_within_m` (how near, in metres) are set on entry and put back on *every*
exit path — finished, stopped, timed out, e-stopped, or the mode switched out
from under it. A routine that left either rewritten would make the next thing
anybody does behave oddly with nothing on screen to explain why: a detector still
filtering on buckets, or a manual alignment stopping at a distance nobody chose.
Omitting them means "whatever the operator set", which is also exactly what every
routine written before the fields existed means. The metres go through the
bounding-box rangefinder, so an uncalibrated build ignores them and stops at its
own `standoff_size` rather than refusing to run.

**Conditions read what the controllers already publish** — `aligned`, `arrived`,
`last_detection`, `route_done`, `shots` — rather than deriving a second, subtly
different answer that would drift from the first. Each is a pure read through an
injected `RoutineContext`, the same rule that makes the controllers testable, so
a routine unit-tests against stubs on a laptop.

**Three engine rules do most of the safety work.**

1. **At most one transition per tick.** Evaluated in the order they were authored,
   first match wins. A cycle of `always` transitions then advances one state per
   20 ms tick instead of spinning inside a single one — and it makes the machine
   predictable to whoever drew it: one box per tick.
2. **`for_seconds` means *continuously*.** The launcher's dwell rule generalized:
   a single tick of `aligned` is equally consistent with a false positive or a
   target crossing the centre on its way past.
3. **Every state can be left.** A state carries a timeout (inheriting
   `routines.state_timeout_default`, re-read every tick so raising it applies to
   the routine already running), and the routine may carry one too. Expiry *ends*
   the routine rather than advancing: if a state overran, the next one's
   assumptions probably don't hold either. A routine with no terminal state, no
   routine timeout and every per-state limit switched off is a **validation
   error** — an unattended runaway is what the e-stop exists to catch, not
   something to author deliberately.

**Arming is triple-gated**, being the one action a drawn program can take that
makes something physically launch: off by default (`RS_ROUTINE_ALLOW_ARM`),
refused *at parse time* outside a state that drives with `shooter_align` (where
dwell, cooldown and the magazine are enforced), and re-checked at **runtime** so
switching the gate off stops a routine already running. The only direction a
safety gate may be slow in is on.

Actions never touch the drivetrain — that is the state's `drive` source — so
exactly one thing commands the motors at any moment. They run `on_enter` (once),
`on_tick` (every tick, hence the write elision above) and `on_exit`, which fires
however the state is left, *including* a timeout, an abort or an e-stop. That is
what makes `on_exit` the right place to disarm something.

A document that fails validation is stored but never armed: the robot keeps
running the last set that was good, which is the difference between a rejected
edit and a rover that stops mid-field.

### 4.5 Comms layer

**`protocol.py`.** The wire format is newline-delimited JSON — debuggable and
language-agnostic. `encode(dict) → bytes`, `decode(bytes) → dict | None` (returns
`None` on malformed input rather than raising). Swap this module to change
framing; nothing else depends on the format.

**`xbee_link.py`.** The XBee runs in **transparent (AT) mode**: bytes in = bytes
out. `XBeeLink.start()` opens the serial port and spawns a reader thread that
accumulates bytes up to each `\n`, decodes the frame, and calls `on_message` (the
`Robot`'s queue `.put`). `send(msg)` encodes and writes under a lock (safe to
call from the main loop while the reader thread runs). Transient read errors are
logged and skipped so the link survives glitches. To move to API-mode XBee you'd
swap the transport internals; the `start/stop/send + on_message` interface stays.

**`ip_link.py` — bulk transfers over WiFi.** The radio is the *control* link:
drive, telemetry, mode, e-stop. Config snapshots, layouts and routine documents
are none of those — they're bulky, bursty and not realtime — and a ~2.9 KB
snapshot is roughly half a second of exclusive airtime on a channel shared with
every robot, during which telemetry frames are the ones `XBeeLink.send` drops.
So that traffic rides a TCP socket over the same WiFi the FPV video already uses:

```
robot                                     base station
IPLink(comms.base_host, 5006)  --TCP-->   IPServer(5006)
```

Same `protocol.py` framing, so a `config` frame is the identical dict whichever
way it travelled and `FleetManager` can't tell the difference. The robot **dials
out** (mirroring the video path, and avoiding having to discover a rover's
address on a DHCP field network); the base station keys sockets by the `from` on
anything a robot sends, plus a `hello` on connect so the mapping exists before
the first document push.

**Over WiFi or not at all.** `send()` returns a *bool* rather than raising:
`False` means "not connected". There is no radio fallback for bulk traffic — a
snapshot is half a second of a channel shared with every robot's telemetry, and
nothing about it is urgent, so a rover with a blank `comms.base_host`, a base
station that isn't up, or a drive out of WiFi range simply **isn't configurable
until the link is back**. It keeps driving, reporting and answering the e-stop
throughout, because that is what the radio is for. Both ends say so rather than
going quiet: the robot logs the dropped frames (`Robot._drain_outbox`) and the
base station puts the reason where the settings page renders it
(`FleetManager.note_unreachable`), so "not on WiFi" is stated, not inferred.

Pacing follows from the same split. `Robot._drain_outbox` empties the whole
queue over WiFi in one tick because there's no airtime to protect; the metering
in `airtime.py` now only applies to the one frame type below.

**The bootstrap exception.** A `set_config` carrying nothing but
`comms.base_host` / `comms.base_port` *does* go over the radio — it is how a
rover is told where the WiFi link is, and requiring the link to configure the
link is a chicken-and-egg that ends with an SSH session and a text editor. The
test is all-or-nothing (`tuning.is_bootstrap`): a frame that slips a PID gain in
beside the hostname is an ordinary config edit and waits for WiFi like one.
~60 bytes, sent by hand, once. Both settings are `live=True` — `Robot`
re-dials on the spot (`_retarget_ip_link`, on its own thread because `stop()`
can block on a parked reader), so the new address takes effect without the
service restart that would have meant walking out to the rover anyway.

### 4.6 Sensors

**`gps.py` — Adafruit Ultimate GPS reader.** See [§6](#6-gps-waypoint-autonomy).

**`encoder.py` — quadrature wheel encoders.** See [§4.4](#44-drive-layer). Not a
position sensor: wheel odometry on a skid-steer chassis is dead reckoning
through a slipping contact patch and the error grows without bound. It exists so
the speed loop has a *relative* measurement over a fraction of a second, where
the accumulated drift never matters.

---

## 5. Wire protocol (XBee)

Newline-delimited JSON over one shared serial channel. `to` addresses a robot (or
`"all"`); robots stamp telemetry with `from`. The base station's baud
(`basestation.env`, default **57600**) **must match each robot's** `RS_XBEE_BAUD`
— a mismatch delivers only garbage frames.

> Only the realtime half of this message set actually travels on the radio:
> `drive`, `telemetry`, `mode`, `estop`. The bulk half — `config`, `fields`,
> `layout`, `routines` and their `put_*`/`*_result` counterparts — travels over
> WiFi and does not fall back here, because the radio is what has the range and
> a config dump is what spends it. The single exception is a `set_config`
> carrying only `comms.base_host`/`comms.base_port`, which is how a rover is
> told where the WiFi link is. See [§4.5](#45-comms-layer).

```jsonc
// base station -> robot
{"type":"drive","throttle":0.5,"steer":-0.2,"to":"rover1"}  // arcade mixing
{"type":"drive","left":0.4,"right":0.6,"to":"rover1"}        // direct tank
{"type":"mode","mode":"teleop","to":"rover1"}                // teleop|object_align|shooter_align|waypoint
{"type":"route","waypoints":[[lat,lon],...],"to":"rover1"}   // waypoint route
{"type":"estop","to":"rover1"}                               // latch motors off
{"type":"clear_estop","to":"rover1"}                         // release latch
{"type":"arm_shooter","to":"rover1"}                         // shooter_align: permit firing
{"type":"disarm_shooter","to":"rover1"}                      // shooter_align: forbid + park
{"type":"fire","to":"rover1"}                                // shooter_align: manual shot

// documents — structure, not scalars. Chunked; see below.
{"type":"get_layout"}  {"type":"get_routines"}  {"type":"get_fields"}
{"type":"put_layout","txid":"B1","seq":0,"n":3,"part":"{\"vers…","save":true}
{"type":"put_routines","txid":"B2","seq":0,"n":5,"part":"…"}
{"type":"select_routine","id":"collect"}                     // choose which one runs
{"type":"routine_cmd","cmd":"start"}                         // start | stop | restart
{"type":"routine_event","name":"go"}                         // satisfies a `when I press` transition
{"type":"jog","mech":"intake","power":0.3}                   // bench test; teleop only, expires in 0.4 s

// robot -> base station (telemetry, ~5 Hz)
{"type":"telemetry","from":"rover1","mode":"teleop","estop":false,
 "left":0.4,"right":0.6,"battery":87.0,
 "lat":37.77,"lon":-122.41,"heading":30.0,  // lat/lon/heading only when GPS has a fix
 "gps":{"fix":1,"sats":9,"speed":1.2,"hdop":0.9,"track":54.7,"track_age":0.4},  // fix health
 "imu_calib":3,
 "shooter":{"armed":true,"shots":1,"ready":false,"cool":0.0},  // only while in shooter_align
 "mech":{"intake":{"kind":"power","values":{"roller":1.0}}}, // only if the layout has any
 "enc":{"rpm":{"left":118.2,"right":117.9},"mode":"match","tl":-0.01,"tr":0.01}, // wheel speed; only if encoders are wired
 "routine":{"id":"collect","state":"seek","t":3.4,"drive":"object_align"}}  // only in `routine`

// robot -> base station (documents + verdicts)
{"type":"layout","from":"rover1","rev":4,"txid":"L4","seq":0,"n":3,"part":"…"}
{"type":"layout_result","from":"rover1","ok":true,"errors":[],"restart_required":true}
{"type":"routines_result","from":"rover1","ok":false,
 "errors":["state 'shoot': unknown mechanism 'intak'"]}
{"type":"fields","from":"rover1","seq":0,"n":2,"fields":[…]}  // descriptors for layout-derived params
```

**Documents are reassembled whole, never merged.** A config snapshot merges
safely because it is a flat map of independent scalars — half of one is a valid
smaller one. A layout is a *tree*, and half a tree is a robot with one drive
motor; partially applying it is exactly how two motors end up on one channel. So
`comms/doc_transfer.py` slices the JSON text into numbered fragments and applies
nothing until every fragment is in hand. The receiver is bounded: one transfer at
a time, a 5 s timeout, a 32 KB cap. Acking is whole-document — the robot replies
`layout_result` / `routines_result` after validating. No per-part NAKs: a save is
a rare deliberate action, and the editor holds its draft until the ack arrives,
so a save that got lost is visible as a Save button that stayed dirty.

**Multi-frame traffic is metered against the line, not counted in frames.**
`comms/airtime.py` is a token bucket denominated in bytes and refilled at the
link's own baud rate. Both ends used to pace in *frames per tick*, which reads
like pacing and isn't: the robot's two frames per 50 Hz tick is 100 frames a
second, and at ~430 bytes a frame that is 43 kB/s offered to a link that carries
5.8. The buffer fills, `serial.write` hits its 0.2 s timeout, and the frame is
dropped to keep the control loop alive — and what gets dropped is a fragment of
a document, which is all-or-nothing, so the settings page stays blank forever.

So `XBeeLink` has two doors. `send()` is realtime (drive, telemetry, e-stop):
it goes now and is *charged* afterwards, so bulk gives way to it rather than
competing. `send_bulk()` answers whether the caller is **done with the frame** —
`False` means the link is merely busy and the frame stays at the head of the
queue, `True` means written *or* unwritable (a dead port never will take it, and
hoarding it would turn a queue into a leak). Both `Robot._drain_outbox` and the
base station's `drain_documents` pop only on `True`.

The dashboard closes the loop from the other side: `state/fetch.ts` re-asks a
document that hasn't arrived, three times over eight seconds, and then says so
with a button instead of leaving "Fetching…" on screen forever.

The base station rate-limits `drive` frames it sends (see [§7](#7-the-base-station))
so a fast gamepad stream doesn't back up a slow radio.

---

## 6. GPS waypoint autonomy

The `WaypointController` drives a list of `(lat, lon)` waypoints. It's pure
decision logic — sensing is injected as `pose_provider() → (lat, lon,
heading_deg) | None` (heading: `0°` = North, clockwise positive).

**Each `update(dt)` tick:**

1. **Failsafes** — no provider, route finished, or `pose() is None` (no fix) →
   `stopped()`.
2. **Arrived?** — `haversine_m(pos, target) ≤ arrive_radius_m` (2.0 m) → advance
   `_idx`, reset the PID, `stopped()` for one tick (a clean pause between legs).
3. **Steer** — `bearing_deg(pos, target)` gives the target compass direction;
   `_heading_error_deg` wraps `bearing − heading` to the shortest signed turn
   `[−180, 180]`; a heading `PID` turns that error into `steer`. Error is in
   DEGREES, which is why the gains look small — `kp=0.02` saturates a `0.6`
   output at 30° of error, leaving the whole sub-pivot band proportional
   instead of bang-bang.
4. **Mix** — `|error| > pivot_threshold_deg` (25°) turns in place
   (`arcade(0, steer)`); otherwise cruise and let the loop trim
   (`arcade(cruise_speed, steer)`).

**Which heading, and why it changes the loop.** `PoseEstimator.heading_is_absolute()`
tells the controller whether the heading it just read is an IMU attitude or a GPS
course over ground, and all three of these swing on it:

| | IMU heading (absolute) | GPS track angle (course over ground) |
|---|---|---|
| gains | `nav.heading_pid` — `kp=0.02, ki=0.002, kd=0.008, out_limit=0.6` | `nav.gps_heading_pid` — `kp=0.008, ki=0, kd=0.006, out_limit=0.4` |
| large error | pivot in place | **arc** at `acquire_speed` — a pivot doesn't move the antenna, so the track angle would freeze and the loop would spin against an error that never updates |
| loop rate | every tick | every tick when a gyro supplies the derivative; otherwise once per **fresh** heading sample, with the true elapsed `dt`, holding the output in between |

The GPS loop is deliberately about half the authority: it is closing around a
sensor that refreshes at `gps.update_rate_ms` (200 ms — 5 Hz — by default) while
the control loop runs at 50 Hz, and it carries no integral at all — a course over
ground has no steady-state bias to trim, and integrating a stale error only winds
up. Switching source mid-route (the IMU finishing calibration, or dropping out)
resets both loops so neither inherits a stale integrator.

> **These gains were sized for a 1 Hz course** and have not been re-tuned since
> the default moved to 5 Hz. They are wrong in the safe direction — a loop damped
> for a once-a-second sensor is merely sluggish on a five-times-a-second one, not
> unstable — but there is real authority left on the table. Re-tune on hardware
> with `nav.pid_trace` on before trusting the numbers in the table above.

If the GPS heading feels sluggish, the fix is upstream of the gains: raise the
GPS fix rate (`gps.update_rate_ms` — and the module baud with it, see below;
5 Hz is the MTK3339's ceiling), lower `gps.min_move_mps` so slower motion still
yields a course, and keep an IMU in the loop even if only for `heading_rate()` —
a gyro-fed derivative is the one thing that lets this loop run at full rate on a
heading slower than the control loop.

Routes arrive live: `on_message({"type":"route","waypoints":[...]})` swaps the
list in and resets to leg 0.

> An interactive, animated walkthrough of exactly this algorithm lives in
> [`docs/waypoint-navigation.html`](./waypoint-navigation.html).

**The GPS reader (`sensors/gps.py`).** An **Adafruit Ultimate GPS**
(MTK3339/PA1616D — breakout, FeatherWing or HAT) read through Adafruit's own
`adafruit_gps` library over pyserial. A background thread drains sentences and
caches the latest fix; the 50 Hz loop calls `pose()` (a cheap locked lookup) and
never blocks on serial. Details:

- **Configured over PMTK at start-up** — `PMTK314` asks for **GGA + RMC + VTG**
  only (no GSV satellite-detail spam), then `PMTK300` and `PMTK220` set the
  interval (`update_rate_ms`, 200 ms / 5 Hz default) for solving a position and
  for talking about it *respectively*. Both, because `PMTK220` alone speeds up
  the sentences while the fix behind them repeats. A non-MTK receiver — a u-blox
  NEO-6M, say — ignores all three and still parses fine.
- **The baud follows the rate.** GGA+RMC+VTG is ~190 bytes a fix, so 5 Hz is
  ~9500 bps of payload — past what a 9600-baud line holds, and a truncated
  sentence fails its checksum and reads as "no fix", not as a baud problem. So
  the default is **57600**, and `start()` sends `PMTK251` from 9600 (the module's
  cold-boot baud) on *every* start — the change doesn't survive a power cycle
  without the breakout's CR1220 battery. `_check_link_budget` prints the
  arithmetic when the requested rate outruns either the link or the receiver.
- **5 Hz is the MTK3339's ceiling** — its position engine solves no faster,
  whatever `PMTK220` is told. The MT3333-based PA1616D does a genuine 10 Hz, so
  a rate below 200 ms warns rather than being clamped.
- **Heading is the GPS track angle** (course over ground, from RMC/VTG): a
  true-North heading with no compass, no calibration and no declination
  correction. It's what `pose()` returns whenever the IMU isn't supplying one.
  But it's the direction the antenna is *travelling*, so it's noise at a
  standstill and blind to a pivot in place. Below `min_move_mps` (0.5 m/s) it's
  ignored and the last good value is held; until the rover has moved once,
  heading is `None` (never `0°` — that would read as "pointing North").
- **A position needs a fix.** `adafruit_gps` keeps writing `latitude`/`longitude`
  through a no-fix sentence, so the reader gates every update on `has_fix` and
  drops null-island `(0, 0)`.
- **Staleness** — a fix older than `fix_timeout` (5 s) makes `pose()` return
  `None`, so lost satellites stop the robot rather than steering on stale data.
- **Fix health on the radio** — `telemetry()` sends `{fix, sats, speed, hdop,
  alt, track, track_age}`, which is how you tell "the GPS is broken" from "it has
  3 satellites under a tree", and a live heading from one held since the rover
  last moved.
- **Survives garbage** — the library does string arithmetic on whatever arrives,
  so a wrong baud rate or a marginal wire can raise out of `update()`; the reader
  catches, logs at most one line per 5 s, and keeps going.
- **Graceful degradation** — missing `pyserial`/`adafruit_gps`, or a UART that
  won't open, disables GPS (logs one line); waypoint mode simply holds position.

Bring-up: `python tools/gps_monitor.py` prints live position, satellites, HDOP
and track angle — walk it in a straight line and watch the track angle settle.

> On the Pi, freeing `/dev/ttyAMA0` requires disabling the serial console and
> enabling the UART (`raspi-config` → Interface Options → Serial Port). Install
> the driver **system-wide** (the service runs as root): `sudo pip install
> adafruit-circuitpython-gps`.

**Object detection and `object_align`.** Same shape as the GPS and IMU readers: a
background thread owns the perception work, and the control loop only ever calls
`detection()` — a cheap locked read. This is not a stylistic choice. Inference is
50–200 ms and the tick budget is a few ms, so running a model inline would trip
the slow-tick watchdog every frame and make the robot unsteerable.

There are **two interchangeable backends**, chosen by `RS_VISION_BACKEND` and
resolved once at startup by `sensors/imx500.resolve_backend()`:

| | `edge_impulse` (`sensors/detector.py`) | `imx500` (`sensors/imx500.py`) |
|---|---|---|
| Where inference runs | the Pi's CPU, from a compiled `.eim` | **inside the sensor** (Raspberry Pi AI Camera) |
| Pi cost per frame | 50–200 ms, ~one core | a tensor decode, well under 1 ms |
| Model file | `.eim`, a binary the Pi **executes** (`chmod +x`) | `.rpk`, data the sensor loads (no chmod) |
| Camera | any (CSI or USB/V4L2) | the AI Camera only |
| `Detection.size` | `None` on FOMO models — no approach | always available |
| `hfov_deg` | the **post-crop** FOV (~50°) | the camera's **real** FOV (~66°) |

They present the identical interface — `start/stop/detection/overlays/telemetry`,
including the staleness contract below — so everything downstream is unchanged.
`auto` picks `imx500` only when an AI Camera is attached *and* its `.rpk` exists;
otherwise it falls back to Edge Impulse, so an existing rover never has its model
swapped out from under it by an upgrade.

The IMX500's one structural difference is that its results arrive as **libcamera
metadata attached to each frame**, not as something computed from the frame
afterwards. `sensors/camera.py` therefore caches frames and metadata *together*
(`frame_meta_and_stamp()`) and the detector opens nothing itself — pairing them
anywhere else would let boxes drift a frame out of step with the pixels they
describe. The decode itself lives in `imx500.Decoder`, which
`tools/detector_selftest.py` drives directly, so the bring-up tool and the rover
cannot disagree about coordinates.

`ObjectAlignController` consumes an injected `detection_provider() ->
Optional[Detection]`, so it neither knows nor cares which backend is behind
it, and it unit-tests with a stub provider on a laptop with no camera
(`tests/test_object_align.py`). It faces the target, approaches, and stops at a
standoff; on loss it sweeps back toward the last sighting and then gives up.
Details worth knowing:

- **Staleness is the safety mechanism.** A frame with no target doesn't clear the
  cache — it just stops advancing the timestamp, and `detection()` ages the sample
  out after `target_timeout`. One rule covers a dropped frame (coast), a lost
  target (search), and *a dead detector thread* (no new stamps → the target ages
  out → the robot stops). There's no liveness check on the control path because
  it fails safe by construction.
- **The PID advances per detection, not per tick.** The loop sees the same cached
  sample several ticks running, so a fixed-`dt` PID would read a frozen error and
  then a spike. It advances only when the stamp changes, using the true
  inter-detection `dt`, and zero-order-holds the steer between samples.
- **Yaw-rate units differ from waypoint's.** Waypoint's error is in degrees, so it
  feeds `-yaw_rate` (deg/s) straight in. Here the error is normalized to
  `[-1, 1]` across `hfov_deg`, so the same rate must be scaled by `2/hfov_deg` —
  ~25–60× smaller. This is the easiest thing in the file to get wrong.
- **FOMO can't do standoff** (Edge Impulse only). FOMO reports centroids with
  fixed cell-sized boxes, so object size is unavailable; `Detection.size` is
  `None` and the controller degrades to align-only rather than driving at the
  target blind. Export a YOLO-style (`object_detection`) model if you want
  approach. The IMX500 zoo is all real bounding-box detectors, so this caveat
  doesn't apply there.
- **Which rectangle the coordinates are relative to differs.** On Edge Impulse,
  `get_features_from_image()` resizes and *center-crops* to the model input, so
  boxes are normalized against `image_input_width/height`; the crop is centered
  (alignment stays correct) but discards ~25% of the width at 640×480 — which is
  why `hfov_deg` must be the **post-crop** FOV. On the IMX500,
  `convert_inference_coords()` maps the sensor's boxes back to full-frame pixels
  against the same request's crop metadata, so nothing is discarded and
  `hfov_deg` is the camera's real FOV. Both then normalize to the same
  `[-1, 1]` the controller expects — but **`standoff_size` must be recalibrated
  if you switch backends**, since the denominators differ.
- **Graceful degradation** — a missing runtime, a missing model, an `.eim` that
  isn't `chmod +x`, an AI Camera that doesn't come up, or no camera at all each
  log one line and leave the detector inert; `object_align` simply holds still.
  Vision failing must never cost the rover its ESCs or its radio, which is why
  every expensive open (the `.eim` subprocess, the sensor's network upload)
  happens on a background thread rather than in `Robot.start()`.

> Bring-up and standoff calibration both go through `tools/detector_selftest.py`,
> which tests whichever backend the rover would use (or `--backend` to force
> one). It checks deps, the model file, labels and rate, and `--save` dumps a
> frame — the model's cropped input on Edge Impulse (confirm colour order and
> framing by eye), the full frame with the sensor's boxes drawn on the IMX500
> (confirm the boxes land on the objects). Park at your stop distance and read
> the printed `size` — that's `RS_VISION_STANDOFF`.
>
> The same parked reading calibrates the **rangefinder**, which is what lets a
> routine ask for metres. Measure the distance with a tape, and the pair
> (`RS_VISION_RANGE_AT_M`, `RS_VISION_RANGE_SIZE`) is the entire model.

**Distance from the bounding box (`control/rangefinder.py`).** Pinhole optics
make `distance × size` a constant once the object's real height, the focal
length and the frame height are folded together — so one measured pair converts
box heights to metres in both directions, and needs no lens data. That is what
`stop_within_m` on a routine state is expressed in.

It is a placeholder with a seam, and the seam is the point: `distance_m` and
`size_at` are the whole interface, so a regression fitted on collected
`(size, distance)` pairs — or a per-label table — replaces the arithmetic without
the controller or the routine schema noticing. Three things it does not model,
all of which matter before you trust a reading: every target is assumed to be the
same height (the constant folds it in, so a cone and a bucket at one distance
read differently), a box seen from an angle is shorter than one seen square on,
and box jitter at the frame edge lands directly on the estimate.

Failure is downward, never silent. No calibration, no box height (FOMO), or
nothing in view all give `None`, and an aligning controller told to stop at a
distance it cannot compute falls back to `standoff_size` — where it stopped
before distances existed. A stop distance nearer than the target stays fully
visible from would need a box height above 1.0, i.e. an arrival test no frame can
pass and a robot that never stops; that is clamped and logged rather than
obeyed. Telemetry carries `vision.dist` beside `vision.size` on purpose: the
guess next to the measurement it came from is what lets someone with a tape
measure see the drift — and collect the pairs a fitted model would want.

**FPV live video (`sensors/fpv.py`, `comms/video_udp.py`).** The camera is a
single shared reader (`sensors/camera.py`) because a V4L2/CSI device can't be
opened twice — the detector and the FPV streamer both sample its cached frame.
The streamer JPEG-encodes and fires frames at the base station over UDP; the
XBee radio (57600 baud) can't carry video, so this rides the WiFi/LAN and works
only within WiFi range. UDP, not TCP, on purpose: a live feed wants the freshest
frame, so a lost packet is dropped and the next frame shown rather than stalling
to retransmit. The base station's `VideoReceiver` reassembles frames (newest per
robot wins) and `app.py` relays them as browser-native MJPEG at
`/video/{robot_id}.mjpg`, which the dashboard shows in an `<img>`. FPV is
independent of the model — it needs only a camera and OpenCV, so live view works
with no `.eim` at all. Off by default (`RS_FPV_ENABLED`), since it needs the base
station's IP — but that is a starting position, not a commitment; see below.

Whether it streams and where it streams are both **live**, driven from the
Tuning tab over the radio. `_push_live_config` hands `fpv.base_host` /
`fpv.base_port` to `FPVStreamer.retarget()`, which rebuilds its `VideoSender` on
the next frame, and `fpv.enabled` to `start()` / `stop(wait=False)`. This matters
more than it sounds: the address belongs to whichever laptop is running the base
station today, the robot only ever learns it over the radio, and a rover that
needed a service restart to switch its camera on or re-aim it is one you cannot
see out of at exactly the moment you cannot go and get it. Rebuilding rather
than adjusting is also how a hostname that only started resolving later gets
picked up — `VideoSender` resolves once, in its constructor.

Three details make the switch safe. The `Camera` is *constructed* whenever one
is configured but only *opened* by its first consumer, so a robot that booted
with no detector and no feed leaves the device shut and still gets a picture
when you ask for one an hour later — `Camera.start()` is idempotent and opens on
its own thread, so the control loop never blocks on it. The off switch does not
join: the streamer sleeps up to a frame interval, and waiting that out would
stall a control tick to save nothing, so the loop notices the flag and closes
its own socket in a `finally`. And `start()` on a loop that is still winding
down re-arms the flag instead of spawning a second one, which is what makes
flipping the switch off and straight back on harmless.

When a model *is* running, the streamer draws the detection boxes onto each
frame before encoding (green for the tracked target, amber for the rest). The
boxes come from the detector via an injected `overlay_provider`, in full-frame
pixels — Edge Impulse's detector inverts its resize + center-crop
(`_to_full_frame`) so a box reported in the model's cropped input space lands in
the right place on the 640×480 feed, and the IMX500's uses
`convert_inference_coords()` for the same job. Drawing happens on a copy of the frame,
never the shared one, and only the freshest boxes are used, so they lag the
video by at most a frame.

**`shooter_align` and the launcher (`drive/shooter.py`).** `shooter_align` is
`object_align` plus a trigger. The alignment, approach, standoff, and search
behaviour above are inherited unchanged; the only addition is the decision to
fire:

```
aligned + (arrived, when the model reports size)  ->  hold still, start dwelling
held continuously for `dwell` seconds             ->  fire
then `cooldown` seconds before the next shot
```

- **Dwell is the point.** The detector is noisy and asynchronous to the control
  loop, so a single centered frame is equally consistent with a false positive, a
  mislabeled box, or the target crossing the center on its way past. Requiring
  the alignment to *hold* costs half a second and rejects nearly all of it. This
  is the difference between shooting at the target and shooting at whatever
  flickers.
- **It stops before firing.** Once the conditions are met the controller
  overrides the alignment command with `stopped()`: a still-creeping robot aims
  worse, and a spring mechanism can shove the chassis. Align → settle → shoot.
- **Three independent safety gates.** (1) Arming is explicit (`arm_shooter`) and
  is dropped on mode exit *and* on e-stop, so it can never be inherited from a
  previous run or resumed by a bare `clear_estop`. (2) The dwell timer resets
  whenever ticks stop arriving — without this, clearing an e-stop while still
  aligned would fire instantly, the dwell having "held" across a period when
  nothing was checked. (3) The `Shooter` owns its mechanical cycle, so asking to
  fire every tick still yields at most one shot per cycle.
- **Firing is a state machine, not a sleep.** `fire()` returns immediately and
  `update()` advances rest → firing → retracting on wall-clock deadlines. A
  blocking `sleep(0.3)` would exceed the 100 ms slow-tick budget on every shot
  and freeze the drive outputs at whatever they last were.
- **The launcher is ticked by `Robot`, not by the controller.** A mode switch or
  e-stop mid-pulse stops `update()` being called, and the servo must still
  retract instead of stalling against its stop with the mechanism cocked.
- **FOMO fires on bearing alone.** Arrival can never latch without `size`, so
  requiring it there would mean a robot that aligns perfectly and never shoots.
- **No launcher on the build?** `RS_SHOOTER_ENABLED=0` (the default) leaves
  `Robot.shooter` as `None` and `shooter_align` behaves exactly like
  `object_align` — it aligns and never fires.

The controller depends on a structural `ShooterLike` protocol (`fire()`/`stop()`)
rather than importing the hardware module, the same rule that keeps controllers
out of `sensors/`. So it unit-tests against a trivial fake
(`tests/test_shooter_align.py`) and a different launcher — solenoid, relay,
flywheel ESC — drops in without touching the control logic.

> Bring-up is in `packaging/robot.env`, and step 1 is *unloaded, on blocks*. Use
> `tools/servo_sweep.py` on the shooter channel to find the rest/fire angles:
> wrong angles stall the servo against a mechanical stop and it will heat up and
> draw current until someone notices.

---

## 7. The base station

`run_basestation.py` builds a `FleetManager`, chooses a data source
(`XBeeLink` with `--port`, or `SimulatedFleet` with `--sim`), optionally starts a
gamepad reader, and serves the FastAPI app with uvicorn.

**`FleetManager` (`fleet.py`).** Thread-safe store of every robot heard from.
`update_from_telemetry(msg, now)` (called on the link's reader thread) updates a
`RobotState` — mode, e-stop, track speeds, battery, position, and a bounded
position **trail** (last 400 points). A robot is `online` if seen within
`ONLINE_TIMEOUT` (3 s). `snapshot(now)` (read by the web loop) returns the whole
fleet as a dict, including the auto-selected robot.

**`app.py` (FastAPI).**
- `/` and `/static/*` — the dashboard.
- `/ws` — WebSocket, carrying **two outbound channels**:
  - `{"type":"fleet"}` — the hot path. A `broadcast_loop` pushes a fleet
    snapshot at `ui_hz` (30 Hz), enriched with controller status, the tiles URL
    + max zoom, and the server's drive-rate budget.
  - `{"type":"settings"}` — the cold path. Base-station settings, the gamepad
    mapping, and each robot's tunable config. Sent on connect and then only
    when something changes: a robot's config is ~2.4 KB, and restating it 30
    times a second would be the largest thing on the socket by far.

  Inbound: browser actions (select / mode / estop / route / drive / shooter),
  plus `get_config`, `set_config`, `set_settings`, and `watch_gamepad`.
  `watch_gamepad` subscribes one client to raw `{"type":"gamepad"}` frames —
  those stream only while a mapping editor is open, since they are useless
  anywhere else.
- `/tiles/{z}/{x}/{y}.png` — offline map tiles ([§8](#8-offline-maps)).
- **Command dispatch** stamps a `to` field so one radio serves the fleet.
- **Gamepad rate-limiting** — `drive` frames are sent only when the command
  meaningfully changes (`DRIVE_EPS`), capped at `drive_hz`, plus a 0.25 s
  keepalive so the robot's `command_timeout` failsafe doesn't trip while a stick
  is held steady. This keeps a slow XBee link from backing up.

**`SettingsStore` (`settings.py`).** The base station's own editable state: the
gamepad mapping (which axis is steer, which button is E-STOP, dead zone, gains)
and the link/UI rates. CLI and env set the baseline at startup; the saved file
(`RS_BASE_SETTINGS`, default `~/.config/roversoftware/basestation.json`) is
overlaid on top, so a flag still configures anything the operator never touched.
Values are clamped, not refused, and a corrupt file is ignored rather than
fatal — the base station must stay launchable.

**`ControllerReader` (`controller_input.py`).** Reads a PS4/DualShock-style
gamepad via pygame on a background thread, headless (`SDL_VIDEODRIVER=dummy`) so
it works on a Mac or a display-less Pi. Emits `(throttle, steer)` at 40 Hz
(throttle = R2 minus L2, steer = right stick X) and fires edge-triggered actions
(e-stop / clear / mode / arm / fire). Hot-plugging reconnects automatically. The
app binds these to the **currently selected** robot.

This is the *only* physical-controller path. The dashboard once polled the
browser Gamepad API and forwarded `drive` frames over `/ws` as a second reader;
that was removed. A controller is a real-time control surface, and routing one
through a browser, the Deno front door and a WebSocket put three things that can
stall or reconnect between a trigger pull and the radio — while giving two
independent readers authority over the same robot whenever both saw the pad. The
browser now sends `drive` only for its own on-screen joystick, whose input has
nowhere else to come from. The mapping is untouched by this: it lives in
`SettingsStore` and is still edited from *Settings → Controller*, which reads the
pad through `state()` below.

Axis and button indices come from the `ControllerMapping` above, not from
constants: they describe a *driver*, not a controller — the same pad enumerates
differently across macOS, Linux, USB and Bluetooth — so re-binding is a tap in
the settings page rather than an ssh session. The reader also publishes
`state()`, the raw axes and buttons it currently sees, which is what lets that
page offer "press the button you want" instead of asking for an index.

**Routines on buttons.** Every other binding is a fixed field, because the
action it fires ships with the build — there is exactly one E-STOP. Routines are
the opposite: the operator writes them, names them, and keeps them on the
*robot*, so the mapping instead carries `ROUTINE_SLOTS` (4) **slots**, each a
`(btn_routine_N, routine_N)` pair of a button index and a routine id. A bound
slot emits `routine:<id>` into the same action vocabulary as everything else,
and `app.py::on_action` turns it into the two frames the driving view's routine
buttons already send. A slot with only one half filled binds nothing — that is
the state the settings page is in for as long as it takes to fill the other
half, and either guess moves a machine nobody aimed.

The id is **not** validated on the base station. It does not know which routines
a rover carries, the rover may be switched off while somebody is editing
bindings, and a binding that broke whenever a rover was off would be worse than
one the robot rejects out loud (`RoutineController.select` refuses an unknown id
and leaves the previous selection alone). Four slots because a flat whitelist of
settings paths has to enumerate them, and a pad has fewer spare buttons than
that before it has more routines than that.

> **Why choosing a routine is routed past the active controller.** Running one
> is choose-then-switch: `select_routine`, then `mode: routine`. `ControlManager`
> hands anything it doesn't own to the *active* controller, so in teleop the
> select reached `TeleopController`, which drops what it doesn't recognise — and
> the mode switch that followed started whichever routine had been selected
> **before**. Pressing one routine and watching another drive away is a bug you
> can only find outdoors. `Robot._drain_inbox` now routes `select_routine` /
> `routine_cmd` / `routine_event` straight to the routine controller, the same
> treatment `set_config` and `jog` get and for the same reason: choosing what
> runs next is not something the thing currently driving gets a vote on.
> Sending the mode first would only have shortened the wrong run to a burst,
> which is no better when the burst is a motor. This fixed the on-screen and
> voice paths too — the pad was just the first thing to make it obvious.

**Mechanism presets on buttons.** The same slot shape, for the same reason, one
layer along: a preset is a named whole-mechanism state (`intake -> in`) declared
in a *rover's layout*, and this process holds no copy of any layout. So
`MECH_SLOTS` (4) slots each carry a `(btn_mech_N, mech_N, preset_N)` **triple**
— both names, because "out" alone does not say what moves when two mechanisms
each have a state by that name — and a filled slot emits
`mech_preset:<mech>:<preset>`. Layout validation constrains both names to
`[a-z][a-z0-9_]*`, so the colon cannot appear inside either half of the action
name. Until now the only things that could ask for a preset were a routine and
the Hardware tab's jog controls, neither of which is a thumb while somebody is
driving.

`Robot._mech_preset` handles the frame beside `_jog` and past the active
controller, for the reason config is handled there: this is the operator asking
a mechanism for a state, not an instruction to whatever is currently driving. It
is refused while the e-stop is latched — nothing may start a motor through the
one button that exists to stop them — and refused in `routine` mode, where a
routine is writing mechanisms, possibly every tick, so honouring a press would
either be undone 20 ms later or fight a state machine for an actuator (switching
to teleop is how an operator takes a rover back, and that already ends the
routine). Every other mode is allowed: object-align and waypoint drive the
*drivetrain*, and an operator lining up on a bucket still owns the intake.

The binding **latches**, deliberately and with no expiry of its own.
`ControllerReader` fires on the press edge, so there is no release to send, and
a preset is a state rather than a nudge — which is why an intake wants two
buttons, `in` and `out`. A build that wants a mechanism to give up on its own
says so once in its layout (`auto_stop_seconds`), which every caller then gets,
rather than a timeout that only button presses have. A preset does clear a
pending `jog` on the same mechanism, or that jog's 0.4 s failsafe would stop a
motor a button had just deliberately started.

> **Two actions on one button.** `_edge` *records* a press as it tests it, so
> asking it twice about the same button within a tick answers False the second
> time. The reader used to call it once per `(index, action)` pair, which meant
> a button carrying two actions ran only whichever `actions()` listed first —
> and `actions()` order is an implementation detail of a dataclass, not a
> promise. Nobody noticed while every binding had its own button; the first
> mechanism preset bound onto the cross that already clears the e-stop cleared
> the e-stop and never touched the intake, with the settings page showing the
> button as bound to the preset. `_fire_actions` now samples every edge first
> and then dispatches, so sharing a button does what the mapping says it does.

**Restarting a rover from the base station.** A layout takes effect at start-up
and nowhere else, so every hardware change used to end in an ssh session with a
rover that was, by then, on a field or on blocks. `{"type": "restart"}` closes
that loop: `Robot._request_restart` sets a flag and drops out of the control
loop, whose `finally` already parks the motors, the mechanisms and the servos.

It does **not** shell out to `systemctl`. A restart issued from inside the unit
races its own SIGTERM against that cleanup, and the part that matters on a
machine with wheels is that the machine is safe before the process ends. Coming
back is the supervisor's job: `run()` returns `EXIT_RESTART` and `run_robot.py`
exits with it. That status is non-zero **on purpose** — the shipped unit says
`Restart=on-failure`, which would leave a process that exited 0 dead, and the
rovers in the field are running whatever unit came with their `.deb` while
`just sync` pushes only Python. A restart that needed a new unit file would be
a restart that switched a rover off.

The refusal is the other half. `INVOCATION_ID` is set by systemd for every unit
it starts and by nothing else, so a robot started by hand on a bench knows that
nothing would start it again and says so instead of going dark. The base station
cannot know that in advance — which is also why the frame goes out over the
*radio* rather than the WiFi bulk path the rest of configuration takes: a rover
worth restarting is often one whose WiFi is part of what is wrong with it. The
simulator models the whole thing (park, four seconds of silence, back in the
start mode with no routine running), so the button is learnable without
hardware.

Analog triggers arm before they steer: SDL scales a trigger to -1 released /
+1 pulled, but some drivers report a flat `0.0` for a trigger untouched since
the joystick opened — which rescales to *half throttle*. `Trigger` therefore
reports 0 until it has seen the axis genuinely at rest, so a freshly plugged-in
controller can never launch the robot on its own. Retuning the rest value
disarms, for the same reason.

Analog triggers arm before they steer: SDL scales a trigger to -1 released /
+1 pulled, but some drivers report a flat `0.0` for a trigger untouched since
the joystick opened — which rescales to *half throttle*. `Trigger` therefore
reports 0 until it has seen the axis genuinely at rest, so a freshly plugged-in
controller can never launch the robot on its own.

**`SimulatedFleet` (`simulator.py`).** A drop-in for `XBeeLink` (same
`start/stop/send + on_message`). Each fake robot is a unicycle model: tank
commands become linear/angular velocity integrated into lat/lon/heading, and in
`waypoint` mode it actually drives clicked routes (using the *same* `bearing_deg`
/ `haversine_m` as the real controller). Each also carries a real `RobotConfig`
and answers `get_config`/`set_config`, honouring the drive limits and waypoint
tuning it is given — so the settings page can be exercised end to end with no
hardware. A settings page you can only test on a real rover is a settings page
that ships broken. This runs the entire dashboard — map, teleop, mode
switching, routes, settings — with no hardware.

**Dashboard (`basestation-ui/`).** A Leaflet map streams fleet state over the
WebSocket: each robot is a heading arrow with a position trail; the rail lists
mode / battery / link / GPS health / track speeds and selects a robot; route
mode drops waypoints and sends them. The gear opens a full-screen settings view
with five tabs — **Tuning** (the selected rover's tunables, fetched on demand),
**Hardware** (its actuators, drivetrain kind and mechanisms), **Routines** (the
FSM editor), **Controller** (mapping, bound by pressing the control you want),
and **Base station** (link and UI rates, basemap). `settings/schema.ts` mirrors
the two Python whitelists for labels, ranges and help text, exactly as
`net/types.ts` mirrors the `/ws` contract; Python remains the authority and
clamps everything. For actuators the operator declared it has nothing to mirror,
so it builds those groups from the descriptors the robot sends — and with none,
it renders byte-for-byte the page it always did.

**Hardware and Routines are draft-then-Save**, unlike the Tuning tab next door.
Committing each field on its own is right for a scalar the robot clamps and
answers; a layout is a document, half of which is a robot with one drive motor,
so the whole thing is edited locally and sent in one go. The draft is kept until
the robot answers, so a refused edit isn't silently lost, and the robot echoes
the *stored* copy back — the validator clamps, so what was saved is not
necessarily what was sent. The Routines tab is a **node graph**: states are boxes you
drag, transitions are wires you draw by dragging from a box's right edge onto
another, and selecting either puts its detail in an inspector beside the canvas.
The split is deliberate — a graph is good at structure and useless at arguments,
since you cannot pick "hold for 0.4 s" or "preset: in" off a box. Node positions
live in the routine document itself, so the diagram a teammate opens is the one
you drew rather than whatever an auto-layout produces on their screen; the robot
preserves those keys without interpreting them. A routine that has never been
opened in the editor gets a layered left-to-right layout from the start state.
The live state is highlighted from telemetry as it runs, which is what makes this
a debugger rather than only an editor.

---

## 7b. Commanding in words: voice, and MCP

`basestation/command/` turns a spoken or typed sentence into the same actions a
dashboard button produces, and `basestation/mcp_server.py` exposes those same
actions to any MCP client. Both faces, one throat.

```
  microphone ──16 kHz PCM over /ws──┐
                                    ▼
  typed order ──────────────► CommandExecutor ──► handle_action() ──► radio
                                    ▲                 (app.py)
  MCP client ──ws /ws─────── ───────┘
```

### Why everything converges on one executor

Not tidiness — **authority**. `CommandExecutor` is where the whitelist, the
confirmation gate and the audit log live. A path that reached the radio some
other way would need its own copy of all three, and a second copy is a second
thing to get wrong. So voice, typed text and MCP all end at
`handle_action()`, which means **commanding by voice or by AI has exactly the
dashboard's authority and no more**. `tests/test_command_bridge.py` asserts that
a spoken order and the equivalent button produce byte-identical radio traffic.

### The three-stage pipeline

| Stage | Module | What it does |
|---|---|---|
| Speech → text | `command/stt.py` | faster-whisper, on this machine |
| Text → intent | `command/fastpath.py`, `command/llm.py` | keyword first, then a local model |
| Intent → actions | `command/intents.py`, `command/executor.py` | validate, gate, dispatch |

**`stt.py`.** The browser sends **raw 16 kHz mono s16le PCM** with no container
(`basestation-ui/src/net/audio.ts` does the conversion in an AudioWorklet). That
means no ffmpeg, no PyAV and no Opus decoder on a Raspberry Pi — the browser
already has a resampler, so asking its `AudioContext` for Whisper's native rate
costs nothing and deletes the entire decoding problem. Push-to-talk, never VAD:
in a pit with three teams shouting, voice activity detection invents commands
nobody gave, and an invented command is a moving robot.

**`fastpath.py`.** Matched *before* the model is consulted. Two reasons, and only
the first matters: **stopping must not depend on LM Studio being open**. A hit
dispatches with no HTTP request at all. The everyday commands — a rover name, a
bare mode, the camera — are here too, because they are effectively buttons and
should not cost a 700 ms round trip. Every rule must be *certain*; anything
ambiguous returns `None` and falls through to the model.

**`llm.py`.** The model's entire job is **classification**: given a sentence and
the live vocabulary, name one intent and fill in its arguments. It never sees the
wire protocol and its output is never trusted. LM Studio's `json_schema` response
format constrains the sampler to the registry's intent names, with a fallback to
prompted JSON for servers that lack it.

**`vocabulary.py`.** Built fresh per command from the fleet and the place store,
so a rover that came online two seconds ago is addressable in the next sentence.
It also resolves what was *said* into what *exists* — "rover two" → `rover2`,
"bucket a" → the saved place — and **refuses rather than guessing**: "bucket"
with both a bucket A and a bucket B saved resolves to nothing, because a rover
driving to the wrong end of the field is worse than being asked to repeat
yourself.

### Authority, and the confirmation gate

`intents.py` is the security boundary. Every intent carries an `Authority`:

- **`ALWAYS`** — e-stop. Ungated, and matched by keyword before the model is
  asked anything, so a model that is slow, wrong, or not running cannot delay a
  stop.
- **`DIRECT`** — selecting, mode changes, routes, cameras, and *every action that
  makes something safer* (disarm, stop-routine, clear-estop). Reversible, and an
  operator who must confirm every mode change stops using their voice.
- **`CONFIRM`** — firing, arming, jogging, raw drive. These return a pending
  request and a card a human taps. They expire after 45 s, so a stale "fire?"
  from an earlier match cannot be tapped by somebody tidying up.

The model never decides this. It names an intent; the registry decides what that
intent is allowed to do.

### MCP

`basestation/mcp_server.py` is a stdio JSON-RPC server that connects to the
bridge over the **same WebSocket the dashboard uses**. It goes through the front
door on purpose: an endpoint mounted inside FastAPI would have been a second path
into the fleet with a second set of rules.

Its tool list is **generated from `INTENTS`**, so an intent added for voice
reaches every MCP client with the same authority automatically, and a `CONFIRM`
intent's tool says so in its description. It has no `mcp` SDK dependency —
MCP over stdio is newline-delimited JSON-RPC, which costs about a hundred lines
here and keeps a dependency list that has to install on a Pi exactly as long as
it was.

```json
{"mcpServers": {"rover": {
  "command": "python", "args": ["-m", "basestation.mcp_server"],
  "env": {"RS_BASE_WS": "ws://127.0.0.1:8000/ws"}}}}
```

### Degrading honestly

Every piece is optional and each absence is *reported*, never hidden:

| Missing | What still works |
|---|---|
| LM Studio not running | stop, rover names, modes, camera (fast path); typing; MCP |
| faster-whisper not installed | everything except the microphone |
| Neither | typed orders and MCP, plus the whole fast path |

A base station whose voice stack is half-installed still comes up, still drives,
and still stops. Warm-up (loading ~150 MB of Whisper weights, pinging the model
server) runs on a background task after startup, because a base station that
won't accept a teleop connection while it loads a speech model is one that misses
its match.

---

## 8. Offline maps

So the dashboard's map works with no internet (field use). Hybrid by default:
serve from a local cache first, fall back online when reachable.

- **`TileStore` (`basestation/tiles.py`)** reads tiles from an **MBTiles** file
  (SQLite; standard format, opens in QGIS too). On a cache miss it fetches from an
  upstream tile server and **writes the tile back**, so coverage grows as you pan
  online. MBTiles stores rows TMS-flipped (origin bottom-left) vs Leaflet/XYZ
  (top-left), so Y is flipped on every read/write. Set `RS_TILES_OFFLINE=1` for a
  pure air-gap (uncached tiles render blank).
- **Serving** — `/tiles/{z}/{x}/{y}.png`; a miss + offline returns HTTP 204
  (Leaflet shows a blank tile). The browser is told the cache's max zoom so it
  upscales the deepest cached level instead of showing blank when you zoom past
  it (`maxNativeZoom`).
- **Building a cache** — `tools/fetch_tiles.py` downloads a bounding box (or
  `--center … --radius-km …`) across a zoom range into an `.mbtiles`, rate-
  limited and resumable. Then `just bs-fetch-tiles …` / `just bs-push-tiles`
  build it and copy it to `/var/lib/roversoftware/tiles.mbtiles` on the Pi.

> Bulk-downloading from the public OSM server is discouraged by their tile policy
> — keep areas modest and, for anything large, point `--url` at your own or a
> commercial tile source.

---

## 9. Threading model

| Thread | Where | Job | Hands off via |
|---|---|---|---|
| main / control loop | robot | 50 Hz: drain inbox, decide, drive, telemetry | — |
| `xbee-rx` | robot | read serial, decode JSON | `queue.Queue` → main loop |
| `gps-rx` | robot | read NMEA, cache latest fix | locked `pose()` |
| `controller` | base | poll gamepad | callbacks → dispatch |
| `sim` | base | integrate fake robots | `on_message` callback |
| uvicorn/asyncio | base | HTTP + WebSocket | `FleetManager` (locked) |

The rule: **blocking I/O never runs on the loop that makes decisions.** Robot
controller logic is single-threaded (fed by a queue). Shared base-station state
(`FleetManager`, `TileStore`) is guarded by locks; the reader thread writes, the
web loop reads.

---

## 10. Configuration reference (env vars)

Robot settings live in `/etc/roversoftware/robot.env`, base-station settings in
`/etc/roversoftware/basestation.env` (both dpkg conffiles — edits survive upgrades).
Each maps to a CLI flag on the respective entry point.

**Robot (`run_robot.py`):**

| Env | Default | Meaning |
|---|---|---|
| `RS_ROBOT_ID` | `rover1` | Unique id on the shared channel. |
| `RS_XBEE_PORT` / `RS_XBEE_BAUD` | `/dev/ttyUSB0` / `9600` | XBee serial. Baud must match the base station. |
| `RS_BASE_HOST` / `RS_BASE_PORT` | *(blank)* / `5006` | Base station host for WiFi bulk transfers (config, layouts, routines). Blank or unreachable means those simply don't move — driving, telemetry and the e-stop are unaffected. Settable from the base station over the radio (the one config that is) and applied without a restart. |
| `RS_START_MODE` | `teleop` | `teleop` \| `object_align` \| `shooter_align` \| `waypoint` \| `routine`. |
| `RS_LOOP_HZ` / `RS_TELEMETRY_HZ` | `50` / `5` | Control-loop and telemetry rates. |
| `RS_MOCK_MOTORS` | `0` | Force mock servos (no HAT). |
| `RS_GPS_ENABLED` / `RS_GPS_PORT` / `RS_GPS_BAUD` / `RS_GPS_RATE_MS` | `1` / `/dev/ttyAMA0` / `57600` / `200` | Adafruit GPS reader; `RATE_MS` is the PMTK300+PMTK220 fix interval (200 = 5 Hz, the MTK3339's ceiling). The baud is set on the module over PMTK251 at every start. |
| `RS_HEADING_SOURCE` | `auto` | `auto` (IMU when calibrated, else the GPS track angle) \| `gps` \| `imu`. |
| `RS_VISION_BACKEND` | `auto` | `auto` \| `edge_impulse` \| `imx500` (AI Camera, on-sensor). |
| `RS_VISION_MODEL` | `/var/lib/roversoftware/model.eim` | Edge Impulse model. Must be `chmod +x`. |
| `RS_VISION_IMX500_MODEL` | `…/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk` | Network uploaded to the IMX500 sensor (`apt install imx500-all`). |
| `RS_VISION_IMX500_LABELS` / `_IOU` / `_MAX_DET` | `` / `0.65` / `10` | Labels file (empty = embedded in the `.rpk`), NMS overlap, boxes per frame. |
| `RS_VISION_LABEL` / `RS_VISION_CONF` | `` / `0.6` | Label to track (empty = any); score floor. |
| `RS_VISION_STANDOFF` / `RS_VISION_HFOV` | `0.45` / `50` | **Both backend-dependent** — calibrate with `tools/detector_selftest.py`; HFOV is post-crop on Edge Impulse, the real ~66° on the IMX500. |
| `RS_VISION_RANGE_AT_M` / `RS_VISION_RANGE_SIZE` | `1.0` / `0.45` | The bounding-box rangefinder's one calibration pair: "at this distance, the box measured this". **Shipped values are a placeholder** — measure them. `0` disables metre estimates. |
| `RS_CAMERA_DEVICE` | `auto` | `auto` \| `imx500` \| `picamera2` \| `/dev/videoN` \| index. Set for you when the IMX500 backend is selected. |
| `RS_SHOOTER_ENABLED` / `RS_SHOOTER_CHANNEL` | `0` / `2` | Servo launcher. Channels 0–1 are the drive ESCs. |
| `RS_SHOOTER_REST` / `RS_SHOOTER_FIRE` | `-30` / `30` | Home and fire angles (find with `tools/servo_sweep.py`). |
| `RS_SHOOTER_FIRE_S` / `RS_SHOOTER_RETRACT_S` | `0.35` / `0.35` | Hold at the fire angle, then settle before re-arming. |
| `RS_SHOOTER_DWELL` / `RS_SHOOTER_COOLDOWN` | `0.5` / `2.0` | Hold the aim this long before firing; min seconds between shots. |
| `RS_SHOOTER_REQUIRE_ARM` / `RS_SHOOTER_REQUIRE_ARRIVED` / `RS_SHOOTER_MAX_SHOTS` | `1` / `1` / `0` | Firing gates; magazine size (0 = unlimited). |
| `RS_ENCODER_LEFT` / `RS_ENCODER_RIGHT` | *(blank)* | Quadrature encoder pins as `"A,B"` — the Fusion HAT's **digital** pins, numbered as BCM GPIO, not its PWM channels. Blank = no encoder and the drivetrain runs open-loop. For bring-up before there is a layout to edit; a saved layout's pins take over, and a dashboard-set value beats both. Read through `fusion_hat`, so no extra package. |
| `RS_ENCODER_CPR` | `0` | Counts per revolution **of the wheel**, gearbox included. Measure it with `tools/encoder_monitor.py` — turn the wheel one full turn and read the count. |
| `RS_ENCODER_LEFT_INVERT` / `RS_ENCODER_RIGHT_INVERT` | `0` / `0` | Flip so forward reads as a positive RPM. Separate from the motor's own `inverted`: that mirrors the motor, this mirrors the sensor. |
| `RS_TRIM_MODE` / `RS_TRIM_MAX_RPM` | `off` / `200` | Closed-loop wheel speed: `off` \| `match` (hold the two sides to each other; no calibration) \| `velocity` (hold each to `throttle × max_rpm`; measure that number). See [§4.4](#44-drive-layer). |
| `RS_TUNING_FILE` | `/var/lib/roversoftware/tuning.json` | Where values set from the dashboard are saved. Applied *after* env and CLI (see [§4.1](#41-configuration-robotconfigpy)). |
| `RS_LAYOUT_FILE` | `/var/lib/roversoftware/layout.json` | The hardware layout. Applied *before* tuning, because it decides which tuning paths exist. Takes effect on the next start; no file = the stock tank drive. |
| `RS_ROUTINES_FILE` | `/var/lib/roversoftware/routines.json` | UI-authored state machines. Unlike a layout these are hot-swappable. |
| `RS_ROUTINE_ALLOW_ARM` / `RS_ROUTINE_STATE_TIMEOUT` | `0` / `60` | Whether a routine may arm the launcher (leave at 0), and the limit a state inherits when it sets none. |

> **Env, CLI, or the dashboard?** Env and CLI set what the robot boots with;
> the dashboard changes what it is doing *now* and saves it. PID gains, speeds
> and limits have no CLI flag on purpose — they are field-tuning knobs, and
> tuning them over ssh is how they end up never tuned at all.

**Base station (`run_basestation.py`):**

| Env | Default | Meaning |
|---|---|---|
| `RS_XBEE_PORT` / `RS_XBEE_BAUD` | `/dev/ttyUSB0` / `57600` | Radio (or use `RS_SIM=1`). |
| `RS_WEB_HOST` / `RS_WEB_PORT` | `127.0.0.1` / `8000` | Dashboard bind. |
| `RS_SIM` / `RS_SIM_ROBOTS` / `RS_SIM_ORIGIN` | `0` / `3` / SF | Simulator. |
| `RS_NO_CONTROLLER` | `0` | Touch-only (no gamepad). |
| `RS_DRIVE_HZ` / `RS_UI_HZ` | `30` / `30` | Command send / UI refresh rates. |
| `RS_TILES` | OSM URL | Tile URL the browser loads (`/tiles/{z}/{x}/{y}.png` for offline). |
| `RS_TILES_MBTILES` / `RS_TILES_UPSTREAM` / `RS_TILES_OFFLINE` | — / OSM / `0` | Offline cache path / fallback source / air-gap. |
| `RS_BASE_SETTINGS` | `~/.config/roversoftware/basestation.json` | Where the gamepad mapping and dashboard-set rates are saved. Loaded *over* the flags above, so an operator's saved choice wins. |

---

## 11. Deployment

Both halves ship as Debian packages built by `packaging/build-deb.sh` and driven
by the `Justfile`. Each installs to `/opt/…`, drops a systemd unit, and starts on
boot; per-instance settings live in `/etc/roversoftware/*.env` conffiles.

**Robot** — `roversoftware-robot`, service runs as **root** (serial + I2C/GPIO
access). Recipes: `just bootstrap` (once — Fusion HAT drivers), `just deploy`
(build + install `.deb`), `just sync` (fast: rsync `robot/`, `run_robot.py`,
`tools/` into `/opt/roversoftware` + restart — no packaging round-trip), plus
`status`/`logs`/`restart`/`config`. Target a specific robot with
`just host=rover2.local <recipe>`.

**Base station** — `roversoftware-basestation`, a headless server service **plus** a
desktop autostart entry that launches a full-screen Chromium **kiosk**
(`kiosk.sh`, wired for Wayland/X). Recipes: `just deploy-basestation`,
`just sync-basestation`, `just bs-reload` (refresh the kiosk),
`just bs-fetch-tiles` / `just bs-push-tiles` (offline maps).

Notes and gotchas:
- `just sync` uses `sudo rsync` / `sudo systemctl` over **non-interactive** SSH,
  so the Pi user needs **passwordless sudo** (standard on Raspberry Pi OS; if not,
  add a `/etc/sudoers.d/` rule). `install`/`deploy` add `ssh -t` so `apt` can
  prompt.
- **`sync` deploys code only**, not the `.env` conffiles or new apt/pip
  dependencies. New env keys or deps need a `.deb` reinstall (or a manual edit /
  `pip install`).
- Service runs as root → Python deps must be installed **system-wide**
  (`sudo apt install python3-…` or `sudo pip install … --break-system-packages`),
  not in a user's `~/.local`.
- Base station requires Raspberry Pi OS / Debian **Bookworm+** (packaged
  FastAPI/uvicorn) and boots to a **desktop session** for the kiosk.

---

## 12. Running & testing without hardware

Everything runs on a laptop:

```bash
# Robot stack with mock motors (exercises comms/telemetry/control):
RS_MOCK_MOTORS=1 python run_robot.py --port <serial> --no-gps

# Full base-station dashboard with fake robots that drive routes:
python run_basestation.py --sim          # → http://127.0.0.1:8000
```

Because sensing is injected and hardware is mocked, unit-style checks construct a
controller with a stub provider and assert on the returned `DriveCommand` — no
Pi, radio, or GPS required. The `SimulatedFleet` runs the *real* bearing/haversine
math, so waypoint navigation is verifiable end-to-end in the browser before any
GPS exists.

---

## 13. Safety mechanisms

- **E-stop latch** — `ControlManager` forces `stopped()` in every mode until
  explicitly cleared.
- **Teleop link failsafe** — no command within `command_timeout` (0.5 s) → stop.
- **ESC arming** — motors held at neutral for `arm_seconds` on boot before any
  command is accepted.
- **Slew-rate limiting** — bounds how fast track speeds change, protecting the
  drivetrain and ESCs.
- **Deadband** — small throttles snap to neutral, so a drifting stick doesn't
  creep the robot.
- **GPS staleness / no-fix → hold** — waypoint mode stops rather than acting on a
  missing or stale fix.
- **Fleet addressing** — robots ignore any message not addressed to their id or
  `"all"`, so one channel can't cross-drive robots.
- **Clean shutdown** — SIGINT/SIGTERM bring motors to neutral, park the shooter
  at its rest angle, and stop all threads.
- **Shooter arming is never sticky** — firing requires an explicit `arm_shooter`,
  and the latch is dropped on e-stop and on leaving the mode. `clear_estop` alone
  never resumes a pending shot.
- **Shooter dwell** — the aim must hold for `dwell` seconds before a shot, and the
  timer resets on any gap in control ticks, so time spent e-stopped or paused can
  never count toward it.
- **PWM channel collisions are validated**, not prevented by a comment. Two
  actuators on one channel move together and neither answers its own commands,
  which reads as a wiring fault and is not one. The drivetrain wins the tie and
  the losing mechanism is disabled rather than the layout being fatal.
- **The e-stop reaches mechanisms**, not only the drivetrain. `ControlManager`
  broadcasts to controllers, and a mechanism is not one, so `Robot` edge-detects
  the latch and stops every mechanism — an intake at full power would otherwise
  spin through the one button that exists to stop it.
- **A layout needs a restart.** Rebuilding actuators mid-loop with the drivetrain
  armed leaves an ESC holding an undefined pulse, so a saved layout is stored and
  reported as pending, never applied live.
- **Routines cannot run forever** — every state carries a timeout, and a machine
  with no terminal state, no routine timeout and every limit switched off is
  refused at validation.
- **A routine cannot arm the launcher** unless the robot was configured to permit
  it, and then only inside a state that drives with `shooter_align`. The gate is
  re-checked at runtime, so turning it off stops a routine already running.
- **Jog has its own failsafe** — the bench-test control is refused unless the
  robot is in teleop with no e-stop latched, and every jog expires after 0.4 s so
  a dropped command can't leave a motor running.
- **A language model cannot invent a command.** It names an intent from a fixed
  registry; `intents.py` then rebuilds the command from validated pieces. An
  unknown intent, an unknown rover or an unsaved place is refused, never
  forwarded hopefully ([§7b](#7b-commanding-in-words-voice-and-mcp)).
- **Stopping never goes through the model.** "Stop" and its synonyms are matched
  on the raw transcript before any HTTP request, so a model that is closed,
  slow or wrong cannot sit between the word and the rover.
- **Firing, arming, jogging and raw drive need a human**, whoever asked —
  operator, voice, or an AI over MCP. They return a pending confirmation that
  expires after 45 s and can be used exactly once.
- **Every order is logged with its source.** With voice, MCP and two tablets all
  able to command one fleet, "why did rover2 just turn" needs an answer, and
  "an AI did that" is the first half of it.
- **The screen cannot disagree with the base station about who is selected.**
  The dashboard owns its selection locally so a tap feels instant, so a spoken
  "rover three" carries an explicit UI step alongside the radio one — an
  operator driving a rover the screen says they are not driving is the worst
  outcome this feature could produce.
