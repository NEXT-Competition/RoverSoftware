# 1 · Run the whole thing with no hardware

*Simulated rovers, real validators, real state-machine engine.*

Clone the repo and install the Python dependencies. The `fusion_hat` library
from SunFounder is only needed on the robot — the motor layer mocks it when it
is missing, so the base station installs cleanly on macOS, Linux or a Pi.

```bash
# base station dependencies (FastAPI, uvicorn, pyserial, pygame)
pip install -r requirements.txt

# one command: bridge + touch UI + browser, one Ctrl+C stops both
./start-basestation.sh                # simulator: three fake rovers
./start-basestation.sh --dev          # same, with Vite hot reload
```

Open `http://127.0.0.1:8000`. Three rovers report in, drive around, obey
commands and follow routes you click on the map. They answer Hardware and
Routines edits with the *real* validators and run the *real* engine, which is
the point: a settings page you can only test on a rover is a settings page that
ships broken.

The fake rovers also have a real defect: each one's right side is 6% weaker than
its left, and the reported wheel speed lags the true one exactly as an encoder's
does. So a simulated rover told to drive straight visibly curves on the map, and
switching **Settings → Robot → Wheel speed matching → Mode** to `match`
straightens it while you watch — the closed loop, the shipped gains and the
tuning graph, all before anyone wires an encoder. See
[Tune it in the field](tuning.md#making-both-tracks-turn-together).

> **It never invents robots**
>
> Running with neither `--port` nor `--sim` exits with a hint rather than
> starting empty. Anything you see is either real telemetry or an explicitly
> requested simulator.

## Running the halves separately

The Python program is the *bridge*: it owns the radio, the gamepad and the tile
cache, and speaks an internal `/ws` + `/tiles` API. The Deno app serves the UI
and proxies those two paths, so one build runs as a desktop window, a LAN server
for an iPad, and the Pi's Chromium kiosk.

```bash
# 1) the bridge, on its internal port
python run_basestation.py --sim --web-port 8001

# 2a) development, with hot reload → http://localhost:5173
cd basestation-ui && npm install && npm run dev

# 2b) production front door → binds 0.0.0.0:8000
npm run build
RS_UPSTREAM=127.0.0.1:8001 deno task serve

# 2c) a native desktop window (Deno ≥ 2.9)
deno task desktop
```

## Useful flags on the bridge

| Flag | What it does |
|---|---|
| `--robots N` | How many rovers the simulator spawns |
| `--origin lat,lon` | Where they spawn |
| `--no-controller` | A pure touch base station, no gamepad reader |
| `--tiles <url>` | Point the map at a local tile server for offline field use |
| `--tiles-upstream <url>` | Swap the imagery provider (e.g. a MapTiler key) |
| `--tiles-mbtiles <path>` | Serve a cache built with `tools/fetch_tiles.py` |
| `--host` / `--web-port` | Move the bridge |

If you would rather install than clone, see [Install from apt](../install/apt.md)
— a base station package exists and starts on boot.
