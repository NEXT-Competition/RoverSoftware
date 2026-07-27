# RoverSoftware

Modular Python stack for a **tank-drive ground robot** built on a Raspberry Pi
+ SunFounder **Fusion HAT**, with two ESC-driven motors and an **XBee** radio
link to a base station.

Built teleop-first, but structured so the planned autonomy (color-detection
alignment, GPS waypoint navigation) and the multi-robot base station drop in
without reworking the core.

> **Docs:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) explains how the whole
> system works end to end (robot, base station, protocol, GPS, offline maps).
> [`docs/waypoint-navigation.html`](docs/waypoint-navigation.html)
> is an interactive walkthrough of the navigation algorithm — open it in a
> browser.

## How it fits together

```
                         XBee (JSON over serial)
 base station  <───────────────────────────────────────►  robot (Pi)
 (Pi / Mac)                                                │
 PS4 controller                                            ▼
 map + telemetry                                    ┌──────────────┐
 voice → LLM plan                                   │ XBeeLink     │  reader thread → queue
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
  drive/
    motor.py            ESCMotor: throttle [-1,1] -> servo angle (mock if no HAT)
    tank_drive.py       left/right mixing + slew-rate limiting
  comms/
    protocol.py         newline-delimited JSON encode/decode
    xbee_link.py        threaded serial reader (transparent-mode XBee)
  control/
    commands.py         DriveCommand (tank / arcade / stopped)
    controller.py       Controller base class
    manager.py          ControlManager: modes + e-stop
    teleop.py           drive from base-station commands (with link failsafe)
    pid.py              reusable PID
    object_align.py     object alignment — inject a detection provider
    detection.py        the Detection contract the controller consumes
    waypoint.py         autonomy scaffold — inject a GPS pose provider
  robot.py              wires it all together; the control loop
run_robot.py            entry point (run on the Pi)
tools/
  esc_calibrate.py      interactive single-channel ESC bring-up
  servo_sweep.py        raw servo sweep (the Fusion HAT hello-world)
  xbee_monitor.py       watch/inject XBee frames to verify the radio link
basestation/            bridge: radio <-> WebSocket + gamepad + tiles — see "Base station"
  app.py                FastAPI: internal WebSocket/tiles API, bridge browser<->radio
  fleet.py              FleetManager: tracks every robot from its telemetry
  simulator.py          fake fleet (drop-in for the radio) so it all runs w/o hardware
  controller_input.py   PS4/gamepad reader (pygame, headless) -> selected robot
  static/               legacy Leaflet dashboard (kept as internal fallback)
  teleop_sender.py      minimal PS4 -> JSON sender (kept for quick point-to-point tests)
basestation-ui/         touch-first Deno UI (Vite + Preact) — desktop / iPad / kiosk
  server/               Deno.serve front door: serves the SPA, proxies /ws + /tiles
  src/                  Preact app: MapView, DrivePad joystick, fleet, controls
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
it works fully offline). Physical gamepads still work via the browser Gamepad
API, and the server-side gamepad path is unchanged.

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
{"type": "mode", "mode": "teleop", "to": "rover1"}                   // or object_align / waypoint
{"type": "route", "waypoints": [[lat, lon], ...], "to": "rover1"}    // waypoint mode
{"type": "estop", "to": "rover1"}                                    // latch motors off
{"type": "clear_estop", "to": "rover1"}

// robot -> base station (telemetry, ~5 Hz)
{"type": "telemetry", "from": "rover1", "mode": "teleop", "estop": false,
 "left": 0.4, "right": 0.6, "battery": 87.0, "lat": 37.77, "lon": -122.41, "heading": 30.0}
```

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
- **Voice → multi-robot planning** — a local LLM turns a high-level spoken order
  into a plan; a dispatcher agent slices it into per-vehicle chunks and sends
  them as `mode`/`route`/task messages over XBee, one manageable step at a time.
```
