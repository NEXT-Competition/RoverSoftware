#!/usr/bin/env bash
# Start the whole base station locally (macOS or Linux dev box) with ONE command:
# the Python bridge (radio or simulator) + the Deno touch UI, wired together.
# One Ctrl+C stops both.
#
#   ./start-basestation.sh                                  # simulator (fake robots)
#   ./start-basestation.sh --port /dev/tty.usbserial-XXXX   # real robots over XBee
#   ./start-basestation.sh --port /dev/tty.usbserial-XXXX --baud 57600
#   ./start-basestation.sh --dev                            # Vite hot-reload UI (for UI dev)
#
# Flags:
#   --sim                 use the built-in simulator (default if no --port)
#   --port <dev>          talk to real robots over this serial device
#   --baud <n>            serial baud for --port (default 57600)
#   --drive-hz <n>        max drive-command send rate over the radio (default 15)
#   --dev                 serve the UI via Vite with hot reload (else a prod build)
#   --no-open             don't open a browser
#   --no-controller       skip the gamepad reader
#
# --baud must match the rate the XBee modules themselves are programmed to (the
# radios' BD parameter) AND the robots' robot/config.py CommsConfig.baud. A
# mismatch here is silent: the link still "works" but the slower side's serial
# buffer backs up and command latency grows for as long as you keep driving.
#
# --drive-hz is the airtime budget, not a feel knob. One drive frame is ~62 B
# ≈ 11 ms at 57600; telemetry (GPS + IMU + vision @ 5 Hz) already eats ~26% of a
# half-duplex channel, so 15 Hz keeps total utilisation near 40% with headroom
# for retries. Raise it only if you have also cut RS_TELEMETRY_HZ on the robots.
#
# Ports (override via env): RS_WEB_PORT (bridge, default 8001),
#   RS_UI_PORT (UI, default 8000). Python override: BASESTATION_PYTHON=/path/to/python
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

BRIDGE_PORT="${RS_WEB_PORT:-8001}"
UI_PORT="${RS_UI_PORT:-8000}"
VITE_PORT=5173
BAUD="${RS_XBEE_BAUD:-115200}"
DRIVE_HZ="${RS_DRIVE_HZ:-15}"
DEV=0
OPEN=1
CONTROLLER=1
SOURCE_ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --sim) SOURCE_ARGS=(--sim); shift ;;
    --port) SOURCE_ARGS=(--port "$2"); shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    --drive-hz) DRIVE_HZ="$2"; shift 2 ;;
    --dev) DEV=1; shift ;;
    --no-open) OPEN=0; shift ;;
    --no-controller) CONTROLLER=0; shift ;;
    -h | --help) grep '^#' "$0" | sed 's/^# \{0,1\}//;s/^#//'; exit 0 ;;
    *) echo "unknown arg: $1 (try --help)" >&2; exit 1 ;;
  esac
done

[ ${#SOURCE_ARGS[@]} -eq 0 ] && SOURCE_ARGS=(--sim)
[ "${SOURCE_ARGS[0]}" = "--port" ] && SOURCE_ARGS+=(--baud "$BAUD")
[ "$CONTROLLER" -eq 0 ] && SOURCE_ARGS+=(--no-controller)

# --- find a Python with fastapi+uvicorn (bridge deps) ---
pick_python() {
  local cands=()
  [ -n "${BASESTATION_PYTHON:-}" ] && cands+=("$BASESTATION_PYTHON")
  cands+=("$ROOT/.venv/bin/python" "$(command -v python3 || true)" "$(command -v python || true)")
  for c in "${cands[@]}"; do
    [ -n "$c" ] && [ -x "$c" ] || continue
    "$c" -c 'import fastapi, uvicorn' >/dev/null 2>&1 && { echo "$c"; return 0; }
  done
  return 1
}
if ! PY="$(pick_python)"; then
  echo "error: no Python with fastapi+uvicorn found." >&2
  echo "  install the bridge deps, e.g.:  uv sync   (or)   python3 -m pip install -r requirements.txt" >&2
  echo "  or point at one:  BASESTATION_PYTHON=/path/to/python $0 ..." >&2
  exit 1
fi

# --- tool checks ---
command -v node >/dev/null 2>&1 || { echo "error: node/npm not found (needed to build the UI)." >&2; exit 1; }
if [ "$DEV" -eq 0 ]; then
  command -v deno >/dev/null 2>&1 || { echo "error: deno not found — install from https://deno.land (or run with --dev)." >&2; exit 1; }
fi
if [ ! -d basestation-ui/node_modules ]; then
  echo "[start] installing UI dependencies (first run)…"
  (cd basestation-ui && npm install --no-audit --no-fund)
fi

# --- start the Python bridge ---
# --drive-hz applies to the simulator too, on purpose: driving in --sim should
# feel like driving the real radio, so a rate that is wrong shows up in dev.
echo "[start] bridge:  $PY run_basestation.py ${SOURCE_ARGS[*]} --drive-hz $DRIVE_HZ --web-port $BRIDGE_PORT"
"$PY" run_basestation.py "${SOURCE_ARGS[@]}" --drive-hz "$DRIVE_HZ" --web-port "$BRIDGE_PORT" &
BRIDGE_PID=$!

# --- start the UI ---
if [ "$DEV" -eq 1 ]; then
  URL="http://localhost:$VITE_PORT"
  (cd basestation-ui && exec npm run dev) &
  UI_PID=$!
else
  echo "[start] building UI…"
  (cd basestation-ui && npm run build >/dev/null)
  URL="http://localhost:$UI_PORT"
  # Run deno directly (not via `deno task`) so this PID *is* the server and a
  # kill stops it cleanly. --config points at the UI's deno.json import map.
  RS_UPSTREAM="127.0.0.1:$BRIDGE_PORT" RS_UI_PORT="$UI_PORT" \
    deno run --allow-net --allow-read --allow-env \
    --config basestation-ui/deno.json basestation-ui/server/main.ts &
  UI_PID=$!
fi

# Kill a process and all of its descendants (portable: pgrep -P is on macOS+Linux).
kill_tree() {
  local pid="$1" child
  for child in $(pgrep -P "$pid" 2>/dev/null); do kill_tree "$child"; done
  kill "$pid" 2>/dev/null || true
}
cleanup() {
  trap - INT TERM EXIT
  echo; echo "[start] stopping…"
  kill_tree "$BRIDGE_PID"
  kill_tree "$UI_PID"
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

if [ "$OPEN" -eq 1 ]; then
  opener=""
  command -v open >/dev/null 2>&1 && opener=open
  [ -z "$opener" ] && command -v xdg-open >/dev/null 2>&1 && opener=xdg-open
  [ -n "$opener" ] && (sleep 3; "$opener" "$URL" >/dev/null 2>&1) &
fi

echo "[start] base station up  →  $URL   (also reachable from an iPad on this LAN)"
echo "[start] press Ctrl+C to stop both."
wait "$UI_PID" || true
