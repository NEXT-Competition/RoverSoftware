# uc-chassis robot build & deployment.
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
app_dir := "/opt/uc-chassis"
deb     := "dist/uc-chassis-robot_" + version + "_all.deb"
service := "uc-chassis-robot"

# Base-station host (override: just bs_host=base.local deploy-basestation)
bs_host    := env_var_or_default("BASE_HOST", "base-station.local")
bs_user    := env_var_or_default("BASE_USER", "pi")
bs_target  := bs_user + "@" + bs_host
bs_app     := "/opt/uc-chassis-basestation"
bs_deb     := "dist/uc-chassis-basestation_" + version + "_all.deb"
bs_service := "uc-chassis-basestation"

# Show available recipes.
default:
    @just --list

# One-time Pi setup: install SunFounder Fusion HAT drivers + the fusion_hat
# Python library, and the BNO055 IMU driver. This may enable I2C and require a
# reboot afterwards. Run this ONCE per robot before the first `just deploy`.
#
# BNO055 note: it clock-stretches on the Pi's hardware I2C. Add
#     dtparam=i2c_arm_baudrate=100000
# to /boot/firmware/config.txt (older Pi OS: /boot/config.txt) and reboot, or
# IMU reads will be flaky. Verify wiring with: python3 tools/imu_monitor.py
bootstrap:
    ssh -t {{target}} "curl -sSL https://raw.githubusercontent.com/sunfounder/fusion-hat/v1/install.sh | sudo bash"
    ssh -t {{target}} "pip install --break-system-packages adafruit-circuitpython-bno055 || pip install adafruit-circuitpython-bno055"
    @echo "==> Fusion HAT + BNO055 driver installed on {{host}}. Set dtparam=i2c_arm_baudrate=100000 then reboot: just reboot"

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
# Syncs robot/, run_robot.py and tools/ into /opt/uc-chassis and restarts.
# Does NOT touch /etc/uc-chassis/robot.env or the systemd unit.
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
    ssh -t {{target}} "sudo nano /etc/uc-chassis/robot.env && sudo systemctl restart {{service}}"

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

# FAST PATH: push updated dashboard code + restart the server (no deb rebuild).
# NOTE: reload the kiosk browser afterwards to pick up UI/static changes (just bs-reload).
sync-basestation:
    rsync -az --delete \
        --rsync-path="sudo rsync" \
        --exclude '__pycache__' --exclude '*.pyc' \
        basestation robot run_basestation.py \
        {{bs_target}}:{{bs_app}}/
    ssh {{bs_target}} "sudo systemctl restart {{bs_service}}"
    @echo "==> synced dashboard to {{bs_host}} and restarted {{bs_service}}"

# Base-station service controls.
bs-restart:
    ssh {{bs_target}} "sudo systemctl restart {{bs_service}}"
bs-status:
    ssh {{bs_target}} "systemctl status {{bs_service}} --no-pager"
bs-logs:
    ssh {{bs_target}} "journalctl -u {{bs_service}} -f -n 100"

# Reload the kiosk browser (after a UI change) by restarting the desktop session.
bs-reload:
    ssh {{bs_target}} "pkill -f uc-chassis-kiosk || pkill chromium || true; sleep 1; /opt/uc-chassis-basestation/kiosk.sh >/dev/null 2>&1 &" || true

# Edit base-station config on the Pi, then restart.
bs-config:
    ssh -t {{bs_target}} "sudo nano /etc/uc-chassis/basestation.env && sudo systemctl restart {{bs_service}}"

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
    ssh {{bs_target}} "sudo mkdir -p /var/lib/uc-chassis && sudo mv /tmp/tiles.mbtiles /var/lib/uc-chassis/tiles.mbtiles && sudo systemctl restart {{bs_service}}"
    @echo "==> pushed offline tiles to {{bs_host}}; reload the kiosk: just bs-reload"
