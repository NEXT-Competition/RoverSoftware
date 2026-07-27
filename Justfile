# RoverSoftware robot build & deployment.
#
# Override the target Pi per-invocation or via env:
#     just host=rover2.local sync
#     ROBOT_HOST=rover2.local just sync
#
# Assumes standard Raspberry Pi OS, where the default user has passwordless
# sudo (so `sudo rsync` / `sudo systemctl` over SSH work without a prompt).

host    := env_var_or_default("ROBOT_HOST", "rover1.local")
user    := env_var_or_default("ROBOT_USER", "pi")
version := "0.1.0"

target  := user + "@" + host
app_dir := "/opt/roversoftware"
deb     := "dist/roversoftware-robot_" + version + "_all.deb"
service := "roversoftware-robot"

# Base-station host (override: just bs_host=base.local deploy-basestation)
bs_host    := env_var_or_default("BASE_HOST", "base-station.local")
bs_user    := env_var_or_default("BASE_USER", "pi")
bs_target  := bs_user + "@" + bs_host
bs_app     := "/opt/roversoftware-basestation"
bs_deb     := "dist/roversoftware-basestation_" + version + "_all.deb"
bs_service := "roversoftware-basestation"
bs_ui_service := "roversoftware-ui"

# Show available recipes.
default:
    @just --list

# Run the WHOLE base station locally on your Mac (bridge + touch UI, one Ctrl+C):
#   just run                                   # simulator (fake robots)
#   just run --port /dev/tty.usbserial-XXXX    # real robots over XBee
#   just run --dev                             # UI hot-reload (Vite)
run *ARGS:
    ./start-basestation.sh {{ARGS}}

# Run the unit tests (pure control logic — no hardware needed).
test *ARGS:
    uv run pytest {{ARGS}}

# One-time Pi setup: install SunFounder Fusion HAT drivers + the fusion_hat
# Python library, and the BNO085 IMU driver. This may enable I2C and require a
# reboot afterwards. Run this ONCE per robot before the first `just deploy`.
#
# BNO085 note: unlike the BNO055 it replaces, the BNO08x speaks SHTP and does NOT
# abuse I2C clock stretching, so it wants the Pi's normal 100 kHz bus. If
#     dtparam=i2c_arm_baudrate=10000
# is still in /boot/firmware/config.txt (older Pi OS: /boot/config.txt) from the
# old sensor, REMOVE it and reboot: at 10 kHz the bus cannot drain the BNO085's
# reports as fast as they arrive, and the driver's start-up wedges. Strap PS0/PS1
# for I2C mode. Verify the sensor with python3 tools/imu_selftest.py (one-shot
# PASS/FAIL), then calibrate with tools/imu_monitor.py.
bootstrap:
    ssh -t {{target}} "curl -sSL https://raw.githubusercontent.com/sunfounder/fusion-hat/v1/install.sh | sudo bash"
    ssh -t {{target}} "pip install --break-system-packages adafruit-circuitpython-bno08x || pip install adafruit-circuitpython-bno08x"
    @echo "==> Fusion HAT + BNO085 driver installed on {{host}}. Remove any dtparam=i2c_arm_baudrate line from config.txt, then: just reboot"

# Reboot the Pi (handy after bootstrap enables I2C).
reboot:
    ssh {{target}} "sudo reboot" || true

# Build the robot .deb (needs dpkg-deb; on macOS: brew install dpkg).
build:
    VERSION={{version}} ./packaging/build-deb.sh robot

# Full install on the Pi: copy the .deb and install it (sets up the service).
install: build
    scp {{deb}} {{target}}:/tmp/
    ssh {{target}} "sudo apt-get install -y /tmp/$(basename {{deb}}) || sudo dpkg -i /tmp/$(basename {{deb}})"

# Alias for a first-time / clean deployment.
deploy: install

# FAST PATH: push updated code straight into place and restart — no deb rebuild.
# Syncs robot/, run_robot.py and tools/ into /opt/roversoftware and restarts.
# Does NOT touch /etc/roversoftware/robot.env or the systemd unit.
sync:
    rsync -az --delete \
        --rsync-path="sudo rsync" \
        --exclude '__pycache__' --exclude '*.pyc' \
        robot run_robot.py tools \
        {{target}}:{{app_dir}}/
    #ssh {{target}} "sudo systemctl restart {{service}}"
    @echo "==> synced to {{host}} and restarted {{service}}"

# Service controls.
restart:
    ssh {{target}} "sudo systemctl restart {{service}}"
start:
    ssh {{target}} "sudo systemctl start {{service}}"
stop:
    ssh {{target}} "sudo systemctl stop {{service}}"
status:
    ssh {{target}} "systemctl status {{service}} --no-pager"

# Follow the live log.
logs:
    ssh {{target}} "journalctl -u {{service}} -f -n 100"

# Edit per-robot config on the Pi, then restart.
config:
    ssh -t {{target}} "sudo nano /etc/roversoftware/robot.env && sudo systemctl restart {{service}}"

# Open a shell on the Pi.
shell:
    ssh {{target}}

# Remove the package from the Pi.
uninstall:
    ssh {{target}} "sudo apt-get remove -y {{service}} || sudo dpkg -r {{service}}"


# ───────────────────── vision model (Sony IMX500 / AI Camera) ─────────────────
# The network runs INSIDE the camera, so getting our model there is a two-machine
# job and neither half can do the other's:
#
#   this machine          the Pi
#   ────────────          ──────
#   best.pt                                    Ultralytics + MCT + imxconv-pt
#     -> packerOut.zip  ──scp──>  network.rpk  imx500-package (apt-only, ARM)
#
# `imx500-package` ships in the Pi's `imx500-tools` apt package and is not on
# PyPI, which is the entire reason `just model-install` exists rather than the
# export just producing an .rpk directly. Full story: docs/MODEL_CONVERSION.md.

imx_out    := "build/imx500-yolo"
imx_data   := "/var/lib/roversoftware"
imx_net    := imx_data + "/network.rpk"

# imx500-tools brings the packager, imx500-all the sensor firmware, picamera2
# the capture path.
#
# One-time per robot: install the AI Camera stack.
model-bootstrap:
    ssh -t {{target}} "sudo apt-get install -y imx500-tools imx500-all python3-picamera2"
    @echo "==> AI Camera stack installed on {{host}}"

# Stages the Label Studio export, then converts. Runs HERE, not on the Pi.
# Extra args pass through to the exporter (--imgsz 480, --conf, ...).
#
# Convert best.pt -> packerOut.zip. Needs `uv sync --group convert` first.
model-export *ARGS:
    uv run tools/prepare_yolo_dataset.py --src model/data --out build/dataset
    uv run tools/imx500_export_yolo.py --model model/best.pt --data build/dataset/data.yaml {{ARGS}}

# Worth running after every export: a badly-calibrated model exports and loads
# fine and simply sees less well, with nothing anywhere reporting an error.
#
# Measure what INT8 quantization cost, against the float checkpoint.
model-validate *ARGS:
    uv run tools/imx500_validate.py --report {{imx_out}}/validation.json {{ARGS}}

# Build the .rpk ON THE PI from packerOut.zip, and install it.
model-install:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f "{{imx_out}}/packerOut.zip" ]; then
        echo "no {{imx_out}}/packerOut.zip — run: just model-export" >&2
        exit 1
    fi
    echo "==> copying packerOut.zip + labels.txt to {{host}}"
    scp "{{imx_out}}/packerOut.zip" "{{imx_out}}/labels.txt" {{target}}:/tmp/
    ssh {{target}} bash -euo pipefail -s <<'REMOTE'
        if ! command -v imx500-package >/dev/null; then
            echo "imx500-package not found. Run: just model-bootstrap" >&2
            exit 1
        fi
        rm -rf /tmp/imx500-network && mkdir -p /tmp/imx500-network
        imx500-package -i /tmp/packerOut.zip -o /tmp/imx500-network
        # The packager names its output itself; take whatever .rpk appeared
        # rather than assuming, so a tool rename does not silently install
        # nothing.
        rpk=$(find /tmp/imx500-network -name '*.rpk' | head -1)
        if [ -z "$rpk" ]; then
            echo "imx500-package produced no .rpk" >&2
            exit 1
        fi
        sudo mkdir -p /var/lib/roversoftware
        sudo cp "$rpk" /var/lib/roversoftware/network.rpk
        sudo cp /tmp/labels.txt /var/lib/roversoftware/labels.txt
        sudo chmod 644 /var/lib/roversoftware/network.rpk /var/lib/roversoftware/labels.txt
        ls -l /var/lib/roversoftware/network.rpk /var/lib/roversoftware/labels.txt
    REMOTE
    # NOT `@echo` — this is a shebang recipe, so the body is a plain script and
    # just's silent-prefix would be executed as a command named "@echo".
    cat <<MSG

    ==> installed {{imx_net}} on {{host}}
        Now point the rover at it — just config — and set:
          RS_VISION_BACKEND=imx500
          RS_VISION_IMX500_MODEL={{imx_net}}
          RS_VISION_IMX500_LABELS={{imx_data}}/labels.txt
          RS_VISION_HFOV=66
        Then: just model-selftest
    MSG

# Export here, build + install on the Pi, in one go.
model-deploy: model-export model-install

# Does the sensor actually see anything? Runs the SAME decode the rover uses.
model-selftest *ARGS:
    ssh -t {{target}} "cd {{app_dir}} && python3 tools/detector_selftest.py --backend imx500 {{ARGS}}"

# What is currently installed on the Pi.
model-status:
    ssh {{target}} "ls -l {{imx_net}} {{imx_data}}/labels.txt 2>/dev/null || echo 'no network installed — just model-deploy'; echo; cat {{imx_data}}/labels.txt 2>/dev/null || true"


# ───────────────────────── base station (dashboard) ─────────────────────────
# Target the base-station Pi with bs_host=..., e.g. just bs_host=base.local deploy-basestation

# Build the base-station .deb.
build-basestation:
    VERSION={{version}} ./packaging/build-deb.sh basestation

# Full install on the base-station Pi (server service + kiosk autostart).
# apt-get pulls the FastAPI/uvicorn/pygame/chromium dependencies.
install-basestation: build-basestation
    scp {{bs_deb}} {{bs_target}}:/tmp/
    ssh -t {{bs_target}} "sudo apt-get install -y /tmp/$(basename {{bs_deb}}) || sudo dpkg -i /tmp/$(basename {{bs_deb}})"

deploy-basestation: install-basestation

# FAST PATH: push updated bridge code + restart the Python server (no deb rebuild).
# This is the bridge only; for the touch UI use `just sync-ui`.
sync-basestation:
    rsync -az --delete \
        --rsync-path="sudo rsync" \
        --exclude '__pycache__' --exclude '*.pyc' \
        basestation robot run_basestation.py \
        {{bs_target}}:{{bs_app}}/
    ssh {{bs_target}} "sudo systemctl restart {{bs_service}}"
    @echo "==> synced bridge to {{bs_host}} and restarted {{bs_service}}"

# ── Deno touch UI ──
# One-time: install Deno on the base-station Pi (needed by roversoftware-ui).
bootstrap-deno:
    ssh -t {{bs_target}} "curl -fsSL https://deno.land/install.sh | sh -s -- -y && sudo ln -sf \$HOME/.deno/bin/deno /usr/local/bin/deno && deno --version"

# Build the touch UI client bundle (Vite). Needs Node/npm locally.
build-ui:
    cd basestation-ui && (npm ci --no-audit --no-fund || npm install --no-audit --no-fund) && npm run build

# FAST PATH: build + push the touch UI (dist/ + server/) and restart it.
# Reload the kiosk afterwards to pick up changes: just bs-reload.
sync-ui: build-ui
    rsync -az --delete \
        --rsync-path="sudo rsync" \
        basestation-ui/dist basestation-ui/server basestation-ui/deno.json \
        {{bs_target}}:{{bs_app}}/ui/
    ssh {{bs_target}} "sudo systemctl restart {{bs_ui_service}}"
    @echo "==> synced touch UI to {{bs_host}} and restarted {{bs_ui_service}}"

# Touch-UI service controls.
bs-ui-restart:
    ssh {{bs_target}} "sudo systemctl restart {{bs_ui_service}}"
bs-ui-status:
    ssh {{bs_target}} "systemctl status {{bs_ui_service}} --no-pager"
bs-ui-logs:
    ssh {{bs_target}} "journalctl -u {{bs_ui_service}} -f -n 100"

# Base-station service controls.
bs-restart:
    ssh {{bs_target}} "sudo systemctl restart {{bs_service}}"
bs-status:
    ssh {{bs_target}} "systemctl status {{bs_service}} --no-pager"
bs-logs:
    ssh {{bs_target}} "journalctl -u {{bs_service}} -f -n 100"

# Reload the kiosk browser (after a UI change) by restarting the desktop session.
bs-reload:
    ssh {{bs_target}} "pkill -f roversoftware-kiosk || pkill chromium || true; sleep 1; /opt/roversoftware-basestation/kiosk.sh >/dev/null 2>&1 &" || true

# Edit base-station config on the Pi, then restart.
bs-config:
    ssh -t {{bs_target}} "sudo nano /etc/roversoftware/basestation.env && sudo systemctl restart {{bs_service}}"

# ── offline maps ──
# Build a satellite-imagery tile cache for your operating area (run WITH
# internet), then push it to the base-station Pi so the dashboard map works
# offline. Imagery is JPEG and ~2-3x heavier than street tiles, so keep the bbox
# tight. Args pass through to tools/fetch_tiles.py --help. Examples:
#   just bs-fetch-tiles --center 37.7749 -122.4194 --radius-km 5 --max-zoom 17
#   just bs-fetch-tiles --bbox -122.52 37.70 -122.36 37.81 --max-zoom 17
bs-fetch-tiles *ARGS:
    python3 tools/fetch_tiles.py --out dist/tiles.mbtiles {{ARGS}}

# Copy the built dist/tiles.mbtiles to the Pi and restart the dashboard.
bs-push-tiles:
    scp dist/tiles.mbtiles {{bs_target}}:/tmp/tiles.mbtiles
    ssh {{bs_target}} "sudo mkdir -p /var/lib/roversoftware && sudo mv /tmp/tiles.mbtiles /var/lib/roversoftware/tiles.mbtiles && sudo systemctl restart {{bs_service}}"
    @echo "==> pushed offline tiles to {{bs_host}}; reload the kiosk: just bs-reload"
