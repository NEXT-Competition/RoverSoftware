# RoverSoftware

Modular Python stack for a **ground robot** built on a Raspberry Pi
+ SunFounder **Fusion HAT**, with ESC-driven motors and an **XBee** radio
link to a base station.

The build is described from the dashboard rather than compiled in: declare as
many motors and servos as you have, pick a drivetrain (two-motor tank, one motor
plus a steering servo, a single motor), and group the rest into mechanisms — an
intake, an arm, a launcher. Then **program it without Python**: the Routines tab
is a state-machine editor, and the machine runs on the robot itself.

Built teleop-first, but structured so the autonomy (object alignment, GPS
waypoint navigation) and the multi-robot base station drop in without reworking
the core.

> **📖 The handbook: <https://next-competition.github.io/RoverSoftware/>**
> — an illustrated walkthrough of running the base station, driving a rover,
> adding your own motors and mechanisms, and programming a routine without
> Python. Source is [`docs/`](docs/); build it locally with `just book`.
>
> Also in there: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) explains how the
> whole system works end to end (robot, base station, protocol, GPS, offline
> maps), and
> [`docs/src/waypoint-navigation.html`](docs/src/waypoint-navigation.html) is an
> interactive walkthrough of the navigation algorithm.

## Install

Released builds are published as signed Debian packages. On a robot:

```bash
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://next-competition.github.io/RoverSoftware/apt/roversoftware-archive-keyring.asc \
  | sudo tee /etc/apt/keyrings/roversoftware.asc > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/roversoftware.asc] https://next-competition.github.io/RoverSoftware/apt stable main" \
  | sudo tee /etc/apt/sources.list.d/roversoftware.list > /dev/null
sudo apt-get update && sudo apt-get install roversoftware-robot
```

`roversoftware-basestation` is the dashboard. Every release also attaches the
raw `.deb` files, a desktop base station for macOS/Windows/Linux, and a Python
wheel — see [Install from apt](docs/src/install/apt.md) and
[Cutting a release](docs/src/reference/releasing.md). To work from a clone
instead, read on.

## How it fits together

```
                         XBee (JSON over serial)
 base station  <───────────────────────────────────────►  robot (Pi)
 (Pi / Mac)                                                │
 PS4 controller                                            ▼
 map + telemetry                                    ┌──────────────┐
 voice → local LLM                                  │ XBeeLink     │  reader thread → queue
 MCP → any AI                                       │              │
                                                    └──────┬───────┘
                                                           ▼
                                                    ┌──────────────┐
                                                    │ControlManager│  mode arbitration + e-stop
                                                    └──────┬───────┘
                                          teleop / object_align / waypoint
                                                           │  DriveCommand(left,right)
                                                           ▼
                                                    ┌──────────────┐
                                                    │  TankDrive   │  mixing + slew limit
                                                    └──────┬───────┘
                                                    ESCMotor ch0   ESCMotor ch1
                                                    (servo PWM)    (servo PWM)
```

Every controller — teleop today, autonomy tomorrow — emits the same
`DriveCommand`, so the drive layer never changes.

## Layout

```
robot/
  config.py             all hardware wiring & tuning
  layout.py             the hardware layout document (what this build HAS)
  drive/
    motor.py            ESCMotor: throttle [-1,1] -> servo angle (mock if no HAT)
    drivetrain.py       tank / motor+steering servo / single / none
    tank_drive.py       TankDrive — the name the tools and tests import
    mechanism.py        intakes, arms, launchers: power and pulse kinds
  comms/
    protocol.py         newline-delimited JSON encode/decode
    xbee_link.py        threaded serial reader (transparent-mode XBee)
    doc_transfer.py     split/reassemble a whole document across frames
  control/
    commands.py         DriveCommand (tank / arcade / stopped)
    controller.py       Controller base class
    manager.py          ControlManager: modes + e-stop
    teleop.py           drive from base-station commands (with link failsafe)
    pid.py              reusable PID
    object_align.py     object alignment — inject a detection provider
    detection.py        the Detection contract the controller consumes
    waypoint.py         autonomy scaffold — inject a GPS pose provider
    routine_controller.py  the `routine` mode: runs a UI-authored state machine
  routine/              the FSM engine: schema, conditions, actions, engine, store
  tuning.py             whitelist of what the dashboard may change, and its limits
  robot.py              wires it all together; the control loop
run_robot.py            entry point (run on the Pi)
tools/
  esc_calibrate.py      interactive single-channel ESC bring-up
  servo_sweep.py        raw servo sweep (the Fusion HAT hello-world)
  xbee_monitor.py       watch/inject XBee frames to verify the radio link
basestation/            bridge: radio <-> WebSocket + gamepad + tiles — see "Base station"
  command/              spoken and typed orders -> the same actions a button sends
    vocabulary.py       what is sayable right now: rovers, places, modes, labels
    fastpath.py         keyword matching — "stop" never waits on a model
    llm.py              local gemma via LM Studio, used only to classify
    intents.py          the whitelist, and which verbs need a human to confirm
    executor.py         validate -> gate -> dispatch. one throat for every face
    stt.py              faster-whisper, on this machine, offline
  mcp_server.py         the same commands as MCP tools, for Claude or any AI
  app.py                FastAPI: internal WebSocket/tiles API, bridge browser<->radio
  fleet.py              FleetManager: tracks every robot from its telemetry
  simulator.py          fake fleet (drop-in for the radio) so it all runs w/o hardware
  controller_input.py   PS4/gamepad reader (pygame, headless) -> selected robot
  settings.py           gamepad mapping + link/UI rates, editable from the UI
  static/               legacy Leaflet dashboard (kept as internal fallback)
  teleop_sender.py      minimal PS4 -> JSON sender (kept for quick point-to-point tests)
basestation-ui/         touch-first Deno UI (Vite + Preact) — desktop / iPad / kiosk
  server/               Deno.serve front door: serves the SPA, proxies /ws + /tiles
  src/                  Preact app: MapView, DrivePad joystick, fleet, controls
    settings/schema.ts  how each tunable is presented (labels, ranges, help)
    components/settings/ the Settings view: robot / controller / base station
run_basestation.py      entry point for the bridge
```

## Wiring / hardware notes

- **motor1 → Fusion HAT channel 0** (left), **motor2 → channel 1** (right).
- The right motor is mounted mirrored, so `right.inverted = True` in `config.py`.
- **ESC as a servo:** an ESC takes the same PWM a servo does. Neutral pulse =
  stop, longer = forward, shorter = reverse. In code, throttle `-1..+1` maps to a
  **symmetric throw about `neutral_angle`** — an equal swing on each side, where
  `throw = min(max_angle - neutral_angle, neutral_angle - min_angle)`. This keeps
  the normal and inverted (mirrored) motors starting together and matching speed
  even when neutral isn't centered; `max_angle`/`min_angle` are the endpoints.
- **"90 is center"?** The Fusion HAT `Servo.angle()` runs about `-90..+90` with
  the *middle* (0) as neutral. The 0–180/“90 in the middle” convention is for
  positional servos. Neutral needn't be 0 — this rover's ESC stops at
  `neutral_angle = 5.0`; change it and calibrate to match yours.

## Quick start

Install (on the Pi, plus `fusion_hat` from SunFounder):

```bash
pip install -r requirements.txt
```

1. **Confirm a channel moves** (wheels off the ground):
   ```bash
   python tools/servo_sweep.py --channel 0
   ```
2. **Calibrate / find ESC neutral & endpoints**, then copy values into `config.py`:
   ```bash
   python tools/esc_calibrate.py --channel 0
   ```
2b. **Verify the XBee link** — watch frames coming in and send test frames out:
   ```bash
   python tools/xbee_monitor.py --port /dev/serial0 --baud 9600
   #   type  d 0.5 0   to send a drive frame,  blank line for a test telemetry,
   #   q to quit.  Unparseable input prints as raw bytes => baud/wiring mismatch.
   ```
   Run it on the robot to confirm the base station's commands arrive; run it (or
   the base station) on the other end to confirm your sends are received.
3. **Run the robot:**
   ```bash
   python run_robot.py --port /dev/serial0 --baud 9600
   ```
4. **Drive it from the base station** (Mac or Pi, PS4 controller attached):
   ```bash
   python -m basestation.teleop_sender --port /dev/tty.usbserial-XXXX
   ```

No hardware yet? It all still runs: the servo layer falls back to a mock, and
the teleop sender with no `--port` prints frames to your terminal.

## Base station (dashboard)

A local web app — Python backend + browser map dashboard — that runs the same
on your Mac and on the Pi. It talks to the whole fleet over one XBee channel
(messages are addressed with a `to` field; each robot ignores others').

```bash
# Real robots over the radio (only robots that report in are shown):
python run_basestation.py --port /dev/tty.usbserial-XXXX
#   -> open http://127.0.0.1:8000

# Simulator (opt-in, testing only): spawns fake robots that drive around,
# obey commands, and follow waypoint routes you click on the map.
python run_basestation.py --sim
```

Running with neither `--port` nor `--sim` exits with a hint — the dashboard
never invents robots, so anything you see is real telemetry from the radio.

In the browser:
- **Sidebar** lists every robot with mode, battery, link status, and live track speeds. Click one to **select** it.
- **Gamepad** (R2 = forward, L2 = reverse, right stick = steer) drives the *selected* robot. Buttons e-stop / clear / switch mode.
- **Map** shows each robot as a heading arrow with a position trail, over **satellite imagery** (Esri World Imagery by default — no API key) so you navigate by visible terrain rather than street names.
- **Route**: toggle *Add waypoints*, click the map to drop points, **Send route** — the robot switches to waypoint mode and drives it.
- **Settings** (the gear, top-left) — see below.

Useful flags: `--robots N` (sim count), `--origin lat,lon` (sim start),
`--no-controller`, `--tiles <url-template>` (point at a local tile server for
offline field use), `--tiles-upstream <url-template>` (swap the imagery source,
e.g. a MapTiler satellite URL with your key), `--host/--web-port`.

### Touch UI (Deno Desktop / iPad) — `basestation-ui/`

The dashboard is a **touch-first Deno app** (Vite + Preact). The Python command
above is now the *bridge*: it owns the radio, gamepad and tile cache and speaks
an internal `/ws` + `/tiles` API. The Deno app serves the UI and reverse-proxies
those two endpoints to the bridge, so **one build runs three ways** — a native
desktop window (`deno desktop`, Deno ≥ 2.9), a LAN server for an iPad or any
touch screen (`deno serve`), and the Raspberry Pi Chromium kiosk.

What the touch UI adds over the old static page: an **on-screen joystick** so you
can drive from a tablet with no gamepad (up = throttle, sideways = steer; it
release-to-zeros and rate-limits to ~30 Hz to match the radio), a floating
always-visible **E-STOP**, responsive **landscape / portrait (bottom-sheet)**
layouts with iPad safe-area insets, and locally-bundled map + fonts (no CDN, so
it works fully offline). A **physical gamepad plugs into the base station**, not
into the tablet: it is read there by pygame and goes straight out over the radio,
so nothing about driving with a controller depends on a browser being awake or a
WebSocket being healthy. Its bindings are still edited from *Settings →
Controller*.

### Commanding by voice — the Command tab

Tap the microphone in the command dock (or say nothing and just type into it) to
open a full screen built for talking: hold **Space**, say the order, and watch
the transcript, the parsed command and the fleet's reaction in one place.

```bash
# 1. Speech recognition — runs on the base station, no internet, no API key.
pip install faster-whisper
#    Fetch the weights ONCE while you still have signal (~150 MB, cached in
#    ~/.cache/huggingface). A competition car park is not where you want to
#    discover this:
python -c "from faster_whisper import WhisperModel; WhisperModel('base.en')"

# 2. The language model — LM Studio (or any OpenAI-compatible server).
#    Load google/gemma-3n-e4b, start the local server on :1234. That's it;
#    nothing to pip install, the base station reaches it over HTTP.

python run_basestation.py --sim          # both are auto-detected
```

Things that work:

| Say | What happens |
|---|---|
| "stop" / "all stop" / "stop rover2" | e-stop, **without the model** — see below |
| "rover two", "switch to rover three" | selects it, everywhere |
| "teleop", "put it in waypoint mode" | mode change |
| "rover1 align to the bucket" | sets `vision.target_label`, *then* enters object align |
| "send rover2 to bucket A then start" | routes through both saved places |
| "show me rover3's camera" | selects it and pulls up the FPV |
| "what is rover2 doing" | answers from live telemetry |
| "fire" / "arm the shooter" | **asks first** — a card you tap |

**"Stop" never goes through the model.** It is matched on the raw transcript
before any HTTP request, so LM Studio being closed, slow, or mid-download cannot
sit between the word and the rover. The same fast path handles rover names,
modes and the camera, which is why those feel instant while a full sentence takes
~700 ms.

**Nothing dangerous happens because a 4B model was confident.** Firing, arming,
jogging and raw drive return a pending card a human taps; it expires after 45 s.
Everything else is validated against live state first — an unknown rover or an
unsaved place is refused with a message naming what *does* exist.

Both halves are optional and say so on screen. With no model: keyword commands
and typing still work. With no faster-whisper: everything except the microphone.
Flags: `--no-voice`, `--llm-url`, `--llm-model`, `--stt-model`.

### Letting Claude (or any AI) command the fleet — MCP

`basestation/mcp_server.py` is a stdio MCP server. It connects to a **running**
base station over the same WebSocket the dashboard uses, which is the point: an
AI gets exactly the dashboard's authority — same whitelist, same confirmation
gate, same audit log — because it goes through the same front door.

```json
{"mcpServers": {"rover": {
  "command": "python", "args": ["-m", "basestation.mcp_server"],
  "env": {"RS_BASE_WS": "ws://127.0.0.1:8000/ws"}}}}
```

```bash
python -m basestation.mcp_server --list-tools   # see exactly what an AI could do
```

The tool list is *generated* from the same intent registry the voice interface
uses, so the two can't drift: add an intent and every MCP client gets it, with
its authority attached. `get_fleet` translates telemetry into words a model can
reason about ("sees: bucket, centred: true" rather than `ex: 0.03`). Tools whose
intent needs approval say so in their description and return "waiting for a
human" — the operator taps the card at the base station.

### Settings

The gear in the top-left opens a full-screen settings view. Changes apply
immediately and are saved — on the robot for robot settings, on the base station
for the rest — so field tuning survives the next power cycle.

- **Tuning** — the selected rover's tunables, over the radio and live: PID gains
  for object alignment and waypoint heading hold, drive limits and per-motor ESC
  calibration, loop and telemetry rates, vision thresholds, shooter geometry and
  firing policy, GPS/IMU and FPV. Fetched when you open the tab rather than
  streamed, because the full set is ~2.4 KB on a link shared with telemetry. On a
  robot running its own layout, the per-motor groups are the ones *it* declared.
- **Hardware** — what the robot is built from. Pick a drivetrain (two-motor tank,
  one motor plus a steering servo, a single motor, or none), add as many motors
  and servos as the build has, and group the rest into **mechanisms** — an
  intake, an arm, a second launcher — each with named presets like `in` and
  `out`. Every actuator has a name, a PWM channel and its own calibration;
  claiming a channel twice is refused rather than silently making two motors move
  together. A **Test** control jogs one mechanism from the bench, refused unless
  the robot is in teleop with no e-stop latched. A layout is saved whole and
  takes effect on the next start, because actuators are built at start-up.
- **Routines** — program the robot without Python, by drawing it. A routine is a
  state machine on a canvas: drag boxes to arrange them, drag from a box's right
  edge onto another to wire them together, tap a wire to say when it fires. Each
  state says what drives (stop, a fixed throttle, or *delegate to* object align /
  shooter align / waypoint), what happens when it is entered, held and left, and
  what makes it move on — after a delay, when lined up, once the route finishes,
  after N shots, or when you press a button. Saved routines run **on the robot**,
  so they survive losing the radio, and the box the robot is actually in lights
  up as it runs. Every routine a rover carries then appears by name on the
  driving view beside the mode buttons, and answers to that name out loud —
  naming a routine is how you invoke it. Transitions are checked in order, one per tick, and a condition
  can be required to hold continuously before it counts — the same reason the
  launcher waits half a second before firing.
- **Network** — put a rover on the WiFi in front of you, from the dashboard.
  Scan, pick, connect; it reports what NetworkManager said and the address it
  got. Works on a rover that is on **no network at all**, because the request
  falls back to the radio — which is the point, since a rover cannot be told
  about a network over that network. The password is sent, applied and
  forgotten: never saved on the base station, never in a config snapshot, never
  echoed back. It does cross the radio in the clear when there is no WiFi yet
  (the XBee is unencrypted unless you set its AES key), and the page says so
  before you type one.
- **Controller** — remap the gamepad by *pressing the control you want*, with a
  live view of every axis and button and the throttle/steer the current mapping
  produces. Also dead zone, trigger rest value, throttle/steer authority and
  steering inversion. Indices describe a driver, not a controller — the same pad
  enumerates differently on macOS, Linux, USB and Bluetooth — so this replaces
  editing constants and restarting the service.
- **Base station** — radio airtime budget (`drive_hz`), dashboard refresh, video
  frame rate, basemap URL, trail length.

Values are **clamped, not refused**: ask for a gain of 500 and you get the
maximum, echoed back so the field shows what the robot is actually doing. Fields
badged `restart` (serial ports, PWM channels, enable flags) are saved but only
take effect on the next start. Everything works against `--sim` — including the
Hardware and Routines tabs, which the simulator answers with the *real*
validators and runs with the *real* state-machine engine — so the whole page can
be exercised with no hardware. A settings page you can only test on a real rover
is a settings page that ships broken, and that goes double for one that programs
the robot.

**Easiest — one command** (starts the bridge + UI together, opens the browser,
one Ctrl+C stops both):

```bash
./start-basestation.sh                                 # simulator (fake robots)
./start-basestation.sh --port /dev/tty.usbserial-XXXX  # real robots over XBee
./start-basestation.sh --dev                           # UI hot-reload (Vite)
# or, if you use just:  just run   /   just run --port /dev/tty.usbserial-XXXX
```

Or run the two halves yourself:

```bash
# 1) Run the Python bridge (radio or --sim) on its internal port:
python run_basestation.py --sim --web-port 8001

# 2) Dev with hot reload (Vite proxies /ws + /tiles to the bridge):
cd basestation-ui && npm install && npm run dev
#   -> open http://localhost:5173   (also reachable from an iPad on the LAN)

# Production front door (serves the built app + proxies to the bridge):
npm run build
RS_UPSTREAM=127.0.0.1:8001 deno task serve      # binds 0.0.0.0:8000
#   -> iPad Safari: http://<this-host>:8000  (Add to Home Screen = fullscreen)

# Native cross-platform desktop binary (requires Deno >= 2.9):
deno task desktop      # dev window with HMR
deno task bundle       # -> self-contained native binary (macOS/Windows/Linux)
```

## Deploying to a robot (Debian package + systemd)

The robot software ships as a `.deb` that installs to `/opt/roversoftware`, drops a
`roversoftware-robot` systemd service, and starts it on boot. Per-robot settings
(id, serial port, mode) live in `/etc/roversoftware/robot.env` — a conffile, so
your edits survive upgrades.

```
packaging/
  build-deb.sh                     assembles the staging tree -> dist/*.deb
  robot.env                        default per-robot env (installed to /etc/roversoftware/)
  systemd/roversoftware-robot.service the unit
  debian/                          control, conffiles, postinst/prerm/postrm
Justfile                           build / install / sync / service controls
```

Everything is driven by [`just`](https://github.com/casey/just). Point it at a
robot by hostname (default `rover1.local`):

```bash
# ONCE per robot: install SunFounder Fusion HAT drivers + fusion_hat library
# (enables I2C; may need a reboot):
just bootstrap
just reboot                        # if bootstrap asks for it

# First-time / clean install (builds the .deb and installs it, sets up service):
just deploy                        # -> rover1.local
just host=rover2.local deploy      # a different robot

# FAST iteration — push changed code into /opt/roversoftware and restart,
# WITHOUT rebuilding or reinstalling the .deb:
just sync
just host=rover2.local sync

# Service + logs:
just status
just logs                          # journalctl -u roversoftware-robot -f
just restart
just config                        # edit /etc/roversoftware/robot.env, then restart
```

`just sync` rsyncs `robot/`, `run_robot.py`, and `tools/` straight into
`/opt/roversoftware` and restarts the service — no packaging round-trip. Use
`deploy`/`install` only when the systemd unit, dependencies, or the env file
change.

Notes:
- Building the `.deb` needs `dpkg-deb` (already on the Pi; on macOS: `brew install dpkg`). You can also just run `just deploy` from the Pi itself.
- `just sync` uses `sudo rsync`/`sudo systemctl` over SSH, which is passwordless by default on Raspberry Pi OS for the primary user.
- The package `Depends` on `python3-serial`. SunFounder's `fusion_hat` library isn't on apt/PyPI — install it once per Pi with `just bootstrap` (which runs `curl -sSL https://raw.githubusercontent.com/sunfounder/fusion-hat/v1/install.sh | sudo bash`). The motor layer mocks it if absent, so the service still starts without it.
- Give each robot a unique `RS_ROBOT_ID` in its `robot.env`.

### Base station as a touchscreen kiosk

The dashboard ships as its own package, `roversoftware-basestation`, for a
Raspberry Pi with a screen. It installs a headless server service **and** a
desktop autostart entry that launches a full-screen, touch-friendly Chromium
kiosk pointed at it on boot.

```
packaging/basestation/
  basestation.env                    per-instance config (radio, ports, sim)
  roversoftware-basestation.service     the Python bridge (systemd)
  roversoftware-ui.service              the Deno touch-UI front door (systemd)
  kiosk.sh                           waits for the UI, launches Chromium --kiosk
  roversoftware-kiosk.desktop           /etc/xdg/autostart entry -> runs kiosk.sh on login
  debian control/conffiles/scripts
```

The package installs **two services**: the Python bridge on an internal port
(`RS_WEB_PORT`, default 8001) and the Deno touch UI on the public port
(`RS_UI_PORT`, default 8000) that the kiosk and any tablet connect to.

Deploy to the base-station Pi (booted to the desktop, with a display attached):

```bash
just bs_host=base.local bootstrap-deno       # ONCE: install the deno runtime
just bs_host=base.local deploy-basestation   # builds (incl. the UI) + installs the .deb
```
`apt-get` pulls the deps (FastAPI, uvicorn, pygame, chromium); the UI build is
bundled into the `.deb`. On boot the bridge + UI come up and the kiosk browser
opens the touch UI full-screen. **The Deno runtime isn't on apt** — install it
once with `just bootstrap-deno` (or the bridge still runs, just without the UI).

- Config lives in `/etc/roversoftware/basestation.env` (`just bs_host=... bs-config`).
  Set `RS_XBEE_PORT`/`RS_XBEE_BAUD` to match your robots; set `RS_SIM=1` to test
  the kiosk with no radio; `RS_NO_CONTROLLER=1` for a pure touch base station.
- Fast iteration: `just bs_host=... sync-ui` (rebuilds + pushes the touch UI and
  restarts it) or `sync-basestation` (bridge code), then `bs-reload` to refresh
  the kiosk browser. UI logs: `just bs_host=... bs-ui-logs`.
- Requires Raspberry Pi OS / Debian **Bookworm+** (for the packaged FastAPI/uvicorn).
- The UI is touch-first: large tap targets, a big E-STOP, pinch/drag map, and a
  layout that adapts to small panels (e.g. the official 7" 800×480 display).

## Protocol

Newline-delimited JSON over the shared XBee channel. `to` addresses a robot (or
`"all"`); robots stamp telemetry with `from`.

```json
// base station -> robot
{"type": "drive", "throttle": 0.5, "steer": -0.2, "to": "rover1"}   // arcade
{"type": "drive", "left": 0.4, "right": 0.6, "to": "rover1"}         // direct tank
{"type": "mode", "mode": "teleop", "to": "rover1"}                   // or object_align / waypoint / routine
{"type": "route", "waypoints": [[lat, lon], ...], "to": "rover1"}    // waypoint mode
{"type": "estop", "to": "rover1"}                                    // latch motors off
{"type": "clear_estop", "to": "rover1"}
{"type": "get_config", "to": "rover1"}                               // every tunable parameter
{"type": "set_config", "config": {"align.pid.kp": 0.6}, "to": "rover1"}

// documents: structure rather than scalars, sent as numbered fragments
{"type": "get_layout", "to": "rover1"}     // what this build HAS
{"type": "get_routines", "to": "rover1"}   // its state machines
{"type": "put_layout", "txid": "B1", "seq": 0, "n": 3, "part": "{\"vers…", "to": "rover1"}
{"type": "select_routine", "id": "collect", "to": "rover1"}
{"type": "routine_cmd", "cmd": "start", "to": "rover1"}   // start | stop | restart
{"type": "routine_event", "name": "go", "to": "rover1"}   // advance a "when I press" transition
{"type": "jog", "mech": "intake", "power": 0.3, "to": "rover1"}   // bench test, teleop only

// robot -> base station (telemetry, ~5 Hz)
{"type": "telemetry", "from": "rover1", "mode": "teleop", "estop": false,
 "left": 0.4, "right": 0.6, "battery": 87.0, "lat": 37.77, "lon": -122.41, "heading": 30.0}
{"type": "config", "from": "rover1", "config": {"align.pid.kp": 0.6},
 "rejected": {}, "restart": [], "save_error": null}
{"type": "layout_result", "from": "rover1", "ok": true, "errors": [], "restart_required": true}
{"type": "routines_result", "from": "rover1", "ok": false,
 "errors": ["state 'shoot': unknown mechanism 'intak'"]}
```

`config` frames carry flat dotted paths into `RobotConfig`; `robot/tuning.py`
decides which exist and clamps every value, so a browser can't reach an
arbitrary attribute. The payload is a *partial* set the base station merges:
everything in reply to `get_config`, only the applied fields after a `set_config`
— a full snapshot is ~0.4 s of airtime at 57600, so it is requested explicitly
and never polled.

**Documents are not merged.** A `config` payload can be, because it is
independent scalars — half a snapshot is a valid smaller snapshot. A layout is a
tree, and half a tree is a robot with one drive motor, so layouts and routines
are sliced into numbered fragments and nothing is applied until every fragment
arrives. The robot replies with a verdict and echoes the *stored* copy back,
since the validator clamps and what was saved is not always what was sent.

Safety built in: teleop stops if commands stop arriving (`command_timeout`), and
e-stop overrides every mode until cleared. (Position fields appear in telemetry
once a `pose_provider` — i.e. GPS — is attached on the robot.)

## Roadmap (the seams are already here)

- **Object-align autonomy** — ✅ done: a model detects the target and
  `ObjectAlignController` faces it, approaches, and stops at a standoff. Two
  interchangeable detection backends (`RS_VISION_BACKEND`):
  - `imx500` — the **Raspberry Pi AI Camera** (Sony IMX500) runs the network on
    the sensor itself, so the Pi spends no CPU on inference and every model
    reports real sized boxes. `sudo apt install python3-picamera2 imx500-all`,
    then `--mode object_align`.
  - `edge_impulse` — a compiled `.eim` run on the Pi's CPU; works with any
    camera. Drop a model at `RS_VISION_MODEL`. Export a YOLO-style
    (`object_detection`) model — FOMO reports centroids, not sized boxes, so it
    can align but never approach.

  `auto` uses the AI Camera when one is attached, else Edge Impulse.
  `tools/detector_selftest.py` covers bring-up and standoff calibration for
  whichever backend you're on.
- **GPS waypoint autonomy** — ✅ done: an Adafruit Ultimate GPS
  (MTK3339/PA1616D) is read with `adafruit_gps` into `(lat, lon, heading)` and
  passed as `WaypointController`'s `pose_provider`. Heading is the module's
  **track angle** (course over ground) — a true-North heading with no compass and
  no calibration — so the rover navigates without an IMU; the BNO085 is still
  preferred when present, because a track angle is meaningless standing still.
  Choose with `--heading-source auto|gps|imu`. Bring-up: `tools/gps_monitor.py`.
- **FPV live video** — ✅ done: the robot streams its camera as JPEG-over-UDP to
  the base station, which serves it as browser-native MJPEG in the dashboard's
  Camera panel. When a model is loaded, detection boxes are drawn onto the feed
  (green = the object `object_align` is tracking, amber = others). Enable on the
  robot with `--fpv --fpv-host <base-ip>` (needs WiFi/LAN — the XBee radio can't
  carry video). Shares the one camera with object detection.
- **Base station app** — ✅ done: map view + live multi-robot tracking, PS4
  teleop of the selected robot, mode switching, click-to-route waypoints, and the
  FPV camera feed. Next: offline tile caching and a telemetry/log panel.
- **Any hardware layout** — ✅ done: motors, servos and mechanisms are declared in
  a layout document written from the dashboard's Hardware tab, not compiled in.
  `DriveCommand(left, right)` is still the one command type in the system, so a
  steered chassis reuses object align and waypoint unchanged. (One honest caveat:
  a steered chassis cannot pivot in place, and those modes ask it to — see
  `min_pivot_throttle` in `docs/ARCHITECTURE.md` §4.4.)
- **Program it without code** — ✅ done: the Routines tab is a node-graph editor
  for state machines. A state says what drives — including *delegating* to object
  align, shooter align or waypoint, which is how the FSM composes the autonomy
  that already exists — what it does to the mechanisms, and what makes it move
  on. Routines run on the robot, so they survive losing the radio.
- **Voice commanding** — ✅ done: hold a key, say "send rover2 to bucket A" or
  "rover1 align to the cone", and it happens. Speech is recognised **on the base
  station** with faster-whisper and classified by a **local Gemma in LM Studio**,
  so the whole path works with no internet — same constraint that produced the
  offline tiles. "Stop" is matched by keyword *before* the model is consulted, so
  it still works with LM Studio closed. Firing, arming and raw drive return a
  card a human taps rather than executing. See the Command tab, and
  `docs/ARCHITECTURE.md` §7b.
- **MCP: any AI can command the fleet** — ✅ done: `python -m
  basestation.mcp_server` is a stdio MCP server that connects to the running base
  station over the same WebSocket the dashboard uses, so Claude (or anything else
  speaking MCP) gets exactly the dashboard's authority — the same whitelist, the
  same confirmation gate, the same audit log. Its tool list is generated from the
  intent registry, so voice and MCP can never drift apart.
- **Voice → multi-robot planning** — still ahead: today an order goes to one
  rover at a time. Next is a dispatcher that slices "clear the north field" into
  per-vehicle chunks and sequences them.
```
