# RoverSoftware — Technical Documentation

How the whole system works, end to end: the tank-drive robot, the base-station
dashboard, the radio protocol, GPS waypoint autonomy, and offline maps. For
usage/quick-start see the top-level [`README.md`](../README.md); for a visual
walkthrough of the navigation algorithm open
[`docs/waypoint-navigation.html`](./waypoint-navigation.html).

## Table of contents

1. [Design principles](#1-design-principles)
2. [System topology](#2-system-topology)
3. [Repository layout](#3-repository-layout)
4. [The robot](#4-the-robot)
   - [Configuration](#41-configuration-robotconfigpy)
   - [The orchestrator & control loop](#42-the-orchestrator--control-loop-robotpy)
   - [Control layer](#43-control-layer)
   - [Drive layer](#44-drive-layer)
   - [Comms layer](#45-comms-layer)
   - [Sensors](#46-sensors)
5. [Wire protocol](#5-wire-protocol-xbee)
6. [GPS waypoint autonomy](#6-gps-waypoint-autonomy)
7. [The base station](#7-the-base-station)
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
still runs on a laptop: no Fusion HAT → mock servos; no `pynmea2`/serial → GPS
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
  config.py                   all tuning: motors, drive, comms, GPS
  robot.py                    orchestrator + the 50 Hz control loop
  control/
    controller.py             Controller base class (on_activate/on_message/update)
    manager.py                ControlManager: mode arbitration + e-stop
    commands.py               DriveCommand (tank / arcade / stopped)
    pid.py                    reusable PID with output + integral clamping
    teleop.py                 drive from base-station commands (+ link failsafe)
    object_align.py           Edge Impulse object alignment — inject a detection provider
    detection.py              the Detection contract the controller consumes
    waypoint.py               GPS waypoint navigation — inject a pose provider
  drive/
    motor.py                  ESCMotor: throttle [-1,1] → servo angle (mock if no HAT)
    tank_drive.py             left/right + slew-rate limiting
  comms/
    protocol.py               newline-delimited JSON encode/decode
    xbee_link.py              threaded transparent-mode XBee serial reader
  sensors/
    gps.py                    NEO-6M NMEA reader → (lat, lon, heading)
    bno055.py                 BNO055 IMU → absolute heading + yaw rate
    pose.py                   fuses GPS position + IMU heading → one pose()
    camera.py                 frame capture (picamera2 → OpenCV → none)
    detector.py               Edge Impulse .eim runner → Detection
run_robot.py                  robot entry point (env + CLI → RobotConfig)

basestation/                  # cross-platform dashboard (Pi or Mac)
  app.py                      FastAPI: UI, WebSocket bridge, tiles route
  fleet.py                    FleetManager: per-robot state from telemetry
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
| `MotorConfig` | `channel`, `inverted`, `neutral_angle`, `max_angle`, `min_angle`, `deadband=0.03`, `max_forward/reverse=1.0` | Per-motor calibration: maps throttle to servo angle. |
| `DriveConfig` | `left` (ch0), `right` (ch1, `inverted=True`), `arm_seconds=2.0`, `slew_rate=4.0` | Two motors + arming + slew limiting. |
| `CommsConfig` | `port="/dev/ttyUSB0"`, `baud=9600`, `command_timeout=0.5` | XBee serial + teleop failsafe window. |
| `GPSConfig` | `enabled`, `port="/dev/ttyAMA0"`, `baud=9600`, `fix_timeout=5.0`, `min_move_mps=0.5` | NEO-6M reader settings. |
| `RobotConfig` | `drive`, `comms`, `gps`, `loop_hz=50`, `start_mode`, `robot_id`, `telemetry_hz=5` | Top-level composition. |

> **ESC-as-servo mapping.** An ESC takes the same PWM as a servo — neutral pulse
> = stop, longer = forward, shorter = reverse. Throttle `-1..+1` maps onto servo
> angle `min_angle..max_angle` with `neutral_angle` as stop. On the Fusion HAT,
> `Servo.angle()` runs roughly `-90..+90` with `0` at neutral. Calibrate with
> `tools/esc_calibrate.py` and copy the endpoints into `config.py`.

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

**`Controller` (base class).** Four hooks: `on_activate()`, `on_deactivate()`,
`on_message(msg)`, and `update(dt) → DriveCommand | None`. `update` is called
every tick; returning `None` means "hold/stop".

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
- **`WaypointController`** — GPS navigation; see [§6](#6-gps-waypoint-autonomy).

### 4.4 Drive layer

**`TankDrive` (`tank_drive.py`).** `drive(left, right)` clamps, applies an
optional **slew-rate limit** (`slew_rate` units/second — caps how fast a track
speed can change so you don't slam the drivetrain), and forwards to two
`ESCMotor`s. `arm()` holds neutral for `arm_seconds` so the ESCs recognise the
signal and arm before commands flow. `stop()` zeros both.

**`ESCMotor` (`motor.py`).** Wraps one PWM channel. `set_throttle(t)`:
1. clamps to `[−1, 1]`, applies per-motor `inverted`;
2. inside `deadband` → neutral;
3. otherwise scales by `max_forward`/`max_reverse` and maps onto a **symmetric
   throw about `neutral_angle`** — an equal swing on each side,
   `throw = min(max_angle − neutral_angle, neutral_angle − min_angle)`. Using the
   same swing both ways means an off-center neutral doesn't desync the normal vs
   inverted motor: both start at the same throttle and match speed. (`max_angle`/
   `min_angle` are the endpoints; the side closer to neutral sets the throw.)

**Mock fallback.** If `fusion_hat` can't be imported *or* `RS_MOCK_MOTORS=1`,
`ESCMotor` uses a `_MockServo` that just records the last angle. This is what
lets the whole stack run on a laptop — the control/comms/telemetry logic is
exercised with no HAT attached.

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

### 4.6 Sensors

**`gps.py` — NEO-6M reader.** See [§6](#6-gps-waypoint-autonomy).

---

## 5. Wire protocol (XBee)

Newline-delimited JSON over one shared serial channel. `to` addresses a robot (or
`"all"`); robots stamp telemetry with `from`. The base station's baud
(`basestation.env`, default **57600**) **must match each robot's** `RS_XBEE_BAUD`
— a mismatch delivers only garbage frames.

```jsonc
// base station -> robot
{"type":"drive","throttle":0.5,"steer":-0.2,"to":"rover1"}  // arcade mixing
{"type":"drive","left":0.4,"right":0.6,"to":"rover1"}        // direct tank
{"type":"mode","mode":"teleop","to":"rover1"}                // teleop|object_align|waypoint
{"type":"route","waypoints":[[lat,lon],...],"to":"rover1"}   // waypoint route
{"type":"estop","to":"rover1"}                               // latch motors off
{"type":"clear_estop","to":"rover1"}                         // release latch

// robot -> base station (telemetry, ~5 Hz)
{"type":"telemetry","from":"rover1","mode":"teleop","estop":false,
 "left":0.4,"right":0.6,"battery":87.0,
 "lat":37.77,"lon":-122.41,"heading":30.0}   // lat/lon/heading only when GPS has a fix
```

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
   `[−180, 180]`; the heading `PID` (`kp=0.02, ki=0, kd=0.005, out_limit=0.7`)
   turns that error into `steer`.
4. **Mix** — `forward = cruise_speed · max(0.2, 1 − |steer|)` (slow down while
   turning hard, but never below 20% of cruise), then
   `DriveCommand.arcade(forward, steer)`.

Routes arrive live: `on_message({"type":"route","waypoints":[...]})` swaps the
list in and resets to leg 0.

> An interactive, animated walkthrough of exactly this algorithm lives in
> [`docs/waypoint-navigation.html`](./waypoint-navigation.html).

**The GPS reader (`sensors/gps.py`).** A background thread reads NMEA sentences
from the NEO-6M and caches the latest fix; the 50 Hz loop calls `pose()` (a cheap
locked lookup) and never blocks on serial. Details:

- Parses **GGA** (position + fix quality; ignored while `gps_qual == 0`) and
  **RMC** (position + course + speed; ignored unless `status == 'A'`).
- **Heading has no compass.** The NEO-6M's only heading is *course over ground*,
  valid only while moving. Below `min_move_mps` (0.5 m/s) the course is treated as
  noise and the last good heading is held; before the first movement it reports
  `0°`. The waypoint PID self-corrects once real motion produces a course.
- **Staleness** — a fix older than `fix_timeout` (5 s) makes `pose()` return
  `None`, so lost satellites stop the robot rather than steering on stale data.
- **Graceful degradation** — missing `pyserial`/`pynmea2`, or a UART that won't
  open, disables GPS (logs one line); waypoint mode simply holds position.

> On the Pi, freeing `/dev/ttyAMA0` requires disabling the serial console and
> enabling the UART (`raspi-config` → Interface Options → Serial Port). Install
> `pynmea2` **system-wide** (the service runs as root): `sudo apt install
> python3-pynmea2`.

**Object detection (`sensors/detector.py`) and `object_align`.** Same shape as the
GPS and IMU readers: a background thread owns the camera and the Edge Impulse
model, and the control loop only ever calls `detection()` — a cheap locked read.
This is not a stylistic choice. Inference is 50–200 ms and the tick budget is a
few ms, so running the model inline would trip the slow-tick watchdog every frame
and make the robot unsteerable.

`ObjectAlignController` consumes an injected `detection_provider() ->
Optional[Detection]`, so it neither knows nor cares that Edge Impulse is behind
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
- **FOMO can't do standoff.** FOMO reports centroids with fixed cell-sized boxes,
  so object size is unavailable; `Detection.size` is `None` and the controller
  degrades to align-only rather than driving at the target blind. Export a
  YOLO-style (`object_detection`) model if you want approach.
- **Coordinates are the model's, not the camera's.** `get_features_from_image()`
  resizes and *center-crops* to the model input, so boxes are normalized against
  `image_input_width/height`. The crop is centered (alignment stays correct) but
  discards ~25% of the width at 640×480 — which is why `hfov_deg` must be the
  **post-crop** FOV.
- **Graceful degradation** — no `edge_impulse_linux`, no model, a model that isn't
  `chmod +x`, or no camera each log one line and leave the detector inert;
  `object_align` simply holds still.

> Bring-up and standoff calibration both go through `tools/detector_selftest.py`:
> it checks deps/chmod/labels/`model_type`/fps, and `--save` dumps the model's
> cropped input so you can confirm the colour order and framing by eye. Park at
> your stop distance and read the printed `size` — that's `RS_VISION_STANDOFF`.

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
- `/ws` — WebSocket. Inbound: browser actions (select / mode / estop / route /
  drive). Outbound: a `broadcast_loop` pushes a fleet snapshot at `ui_hz` (30 Hz),
  enriched with controller status and the tiles URL + max zoom.
- `/tiles/{z}/{x}/{y}.png` — offline map tiles ([§8](#8-offline-maps)).
- **Command dispatch** stamps a `to` field so one radio serves the fleet.
- **Gamepad rate-limiting** — `drive` frames are sent only when the command
  meaningfully changes (`DRIVE_EPS`), capped at `drive_hz`, plus a 0.25 s
  keepalive so the robot's `command_timeout` failsafe doesn't trip while a stick
  is held steady. This keeps a slow XBee link from backing up.

**`ControllerReader` (`controller_input.py`).** Reads a PS4/DualShock-style
gamepad via pygame on a background thread, headless (`SDL_VIDEODRIVER=dummy`) so
it works on a Mac or a display-less Pi. Emits `(throttle, steer)` at 40 Hz (left
stick Y negated, right stick X, 0.08 dead-zone) and fires edge-triggered actions
(e-stop / clear / mode). Hot-plugging reconnects automatically. The app binds
these to the **currently selected** robot.

**`SimulatedFleet` (`simulator.py`).** A drop-in for `XBeeLink` (same
`start/stop/send + on_message`). Each fake robot is a unicycle model: tank
commands become linear/angular velocity integrated into lat/lon/heading, and in
`waypoint` mode it actually drives clicked routes (using the *same* `bearing_deg`
/ `haversine_m` as the real controller). This runs the entire dashboard —
map, teleop, mode switching, routes — with no hardware.

**Dashboard (`static/`).** A Leaflet map streams fleet state over the WebSocket:
each robot is a heading arrow with a position trail; the sidebar lists mode /
battery / link / track speeds and selects a robot; route mode drops waypoints and
sends them.

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
| `RS_START_MODE` | `teleop` | `teleop` \| `object_align` \| `waypoint`. |
| `RS_LOOP_HZ` / `RS_TELEMETRY_HZ` | `50` / `5` | Control-loop and telemetry rates. |
| `RS_MOCK_MOTORS` | `0` | Force mock servos (no HAT). |
| `RS_GPS_ENABLED` / `RS_GPS_PORT` / `RS_GPS_BAUD` | `1` / `/dev/ttyAMA0` / `9600` | NEO-6M reader. |

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
- **Clean shutdown** — SIGINT/SIGTERM bring motors to neutral and stop all
  threads.
