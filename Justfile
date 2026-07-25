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
# Build a tile cache for your operating area (run WITH internet), then push it to
# the base-station Pi so the dashboard map works offline. Args pass through to
# tools/fetch_tiles.py --help. Examples:
#   just bs-fetch-tiles --center 37.7749 -122.4194 --radius-km 5 --max-zoom 17
#   just bs-fetch-tiles --bbox -122.52 37.70 -122.36 37.81 --max-zoom 17
bs-fetch-tiles *ARGS:
    python3 tools/fetch_tiles.py --out dist/tiles.mbtiles {{ARGS}}

# Copy the built dist/tiles.mbtiles to the Pi and restart the dashboard.
bs-push-tiles:
    scp dist/tiles.mbtiles {{bs_target}}:/tmp/tiles.mbtiles
    ssh {{bs_target}} "sudo mkdir -p /var/lib/roversoftware && sudo mv /tmp/tiles.mbtiles /var/lib/roversoftware/tiles.mbtiles && sudo systemctl restart {{bs_service}}"
    @echo "==> pushed offline tiles to {{bs_host}}; reload the kiosk: just bs-reload"
