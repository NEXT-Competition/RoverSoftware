# RoverSoftware robot build & deployment.
#
# Override the target Pi per-invocation or via env:
#     just host=rover2.local sync
#     ROBOT_HOST=rover2.local just sync
#
# Recipes that run sudo on the Pi use `ssh -t`: without a TTY sudo cannot prompt
# and dies with "a terminal is required to read the password". `-t` costs nothing
# when the account already has passwordless sudo.
#
# The `sync*` recipes are the exception — `rsync --rsync-path="sudo rsync"` has
# nowhere to prompt, so those DO require passwordless sudo on the Pi:
#     echo "pi ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010_pi-nopasswd
# Set up an SSH key too (`ssh-copy-id pi@rover1.local`) or every recipe below
# asks for the login password once per ssh/scp.

host    := env_var_or_default("ROBOT_HOST", "raspberrypi.local")
user    := env_var_or_default("ROBOT_USER", "Lucas")
version := "0.1.0"

target   := user + "@" + host
app_dir  := "/opt/roversoftware"
deb_name := "roversoftware-robot_" + version + "_all.deb"
deb      := "dist/" + deb_name
service  := "roversoftware-robot"

# CircuitPython/Adafruit stack the sensor drivers import. Blinka supplies the
# board/busio/digitalio modules the BNO085 needs; the rest are the drivers
# themselves. Installed with sudo on purpose: the service runs as root, and a
# plain `pip install --user` lands in ~pi and is invisible to it.
#
# pyserial is deliberately absent: the .deb Depends on python3-serial, and a pip
# copy in /usr/local would just shadow the apt one.
adafruit_pkgs := "adafruit-blinka adafruit-circuitpython-bno08x adafruit-circuitpython-gps adafruit-circuitpython-busdevice adafruit-circuitpython-register"

# Apt packages the rover needs beyond the .deb's own Depends. None is on PyPI in
# a form that works here — picamera2 and OpenCV are system packages on Bookworm:
#   imx500-all       sensor firmware + Sony's model zoo (/usr/share/imx500-models)
#   imx500-tools     imx500-package, the ARM-only .rpk builder `just model-install` runs
#   python3-picamera2  the capture path the imx500 vision backend imports
#   python3-opencv   cv2, which the edge_impulse backend imports at load time —
#                    its wheel pulls no deps, so without this it fails with
#                    "No module named 'cv2'", which reads as "not installed"
apt_pkgs := "imx500-all imx500-tools python3-picamera2 python3-opencv"

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

# One-time Pi setup: the Adafruit sensor libraries, the header UART for the GPS
# (see `just uart`), and the SunFounder Fusion HAT drivers + fusion_hat Python
# library. Edits the boot config and enables I2C, so REBOOT afterwards. Run this
# ONCE per robot before the first `just deploy`.
#
# BNO085 note: unlike the BNO055 it replaces, the BNO08x speaks SHTP and does NOT
# abuse I2C clock stretching, so it wants the Pi's normal 100 kHz bus. If
#     dtparam=i2c_arm_baudrate=10000
# is still in /boot/firmware/config.txt (older Pi OS: /boot/config.txt) from the
# old sensor, REMOVE it and reboot: at 10 kHz the bus cannot drain the BNO085's
# reports as fast as they arrive, and the driver's start-up wedges. Strap PS0/PS1
# for I2C mode. Verify the sensor with python3 tools/imu_selftest.py (one-shot
# PASS/FAIL), then calibrate with tools/imu_monitor.py.
bootstrap: adafruit uart
    ssh -t {{target}} "curl -sSL https://raw.githubusercontent.com/sunfounder/fusion-hat/v1/install.sh | sudo bash"
    @echo "==> Fusion HAT + Adafruit drivers installed and UART enabled on {{host}}. Remove any dtparam=i2c_arm_baudrate line from config.txt, then: just reboot"

# Free the header UART for the GPS: enable_uart=1 + dtoverlay=disable-bt in
# config.txt, and the serial console off cmdline.txt. Needs a reboot afterwards.
#
# Without this /dev/ttyAMA0 belongs to the Bluetooth modem and the header gets
# the clock-dependent mini-UART, which mangles NMEA — the GPS then looks like it
# simply never gets a fix. packaging/enable-uart.sh has the full story; it backs
# up both boot files and is safe to re-run.
uart:
    scp packaging/enable-uart.sh {{target}}:/tmp/
    ssh -t {{target}} "sudo bash /tmp/enable-uart.sh"

# Reboot the Pi (handy after bootstrap enables I2C).
reboot:
    ssh -t {{target}} "sudo reboot" || true

# Build the robot .deb (needs dpkg-deb; on macOS: brew install dpkg).
build:
    VERSION={{version}} ./packaging/build-deb.sh robot

# Install the Adafruit/CircuitPython sensor libraries on the Pi. Idempotent, so
# it is safe to re-run; `just install` does it for you.
#
# --break-system-packages is required on Bookworm and later (PEP 668 marks the
# OS Python as externally managed); the fallback covers older Pi OS where the
# flag does not exist.
adafruit:
    ssh -t {{target}} "sudo pip install --break-system-packages --upgrade {{adafruit_pkgs}} \
        || sudo pip install --upgrade {{adafruit_pkgs}}"
    @echo "==> Adafruit libraries installed on {{host}}"

# Install the apt-side dependencies on the Pi (the AI Camera stack — see
# apt_pkgs above). Idempotent; `just install` does it for you. Budget for the
# FIRST run: imx500-all is a ~400 MB download of firmware and models.
apt-deps:
    ssh -t {{target}} "sudo apt-get update && sudo apt-get install -y {{apt_pkgs}}"
    @echo "==> apt dependencies installed on {{host}}"

# Full install on the Pi: system + sensor libraries, then copy the .deb and
# install it (which sets up and starts the service).
install: build apt-deps adafruit
    scp {{deb}} {{target}}:/tmp/
    ssh -t {{target}} "sudo apt-get install -y /tmp/{{deb_name}} || sudo dpkg -i /tmp/{{deb_name}}"

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
    # ssh {{target}} "sudo systemctl restart {{service}}"
    @echo "==> synced to {{host}} and restarted {{service}}"

# Service controls.
restart:
    ssh -t {{target}} "sudo systemctl restart {{service}}"
start:
    ssh -t {{target}} "sudo systemctl start {{service}}"
stop:
    ssh -t {{target}} "sudo systemctl stop {{service}}"
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
    ssh -t {{target}} "sudo apt-get remove -y {{service}} || sudo dpkg -r {{service}}"


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

# Install the AI Camera stack. Kept as its own name because the model recipes
# below point at it, but it is just `just apt-deps` — the package list lives in
# apt_pkgs at the top, and `just install` already runs it.
model-bootstrap: apt-deps

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
    scp "{{imx_out}}/packerOut.zip" "{{imx_out}}/labels.txt" \
        packaging/imx500-package.sh {{target}}:/tmp/
    # -t so the sudo inside the script has a terminal to prompt at. This used to
    # be a heredoc piped into `ssh ... bash -s`, which cannot work: the heredoc
    # IS the remote stdin, so sudo has nowhere to read a password from and the
    # run dies at the install step — after the slow packaging has already run.
    ssh -t {{target}} "bash /tmp/imx500-package.sh {{imx_data}}"
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

# Pull the built network + labels OFF the Pi into the repo, so the .deb can ship
# them and a fresh rover gets a working detector straight from `just install`.
#
# This round trip exists because the .rpk cannot be built here: `imx500-package`
# is ARM-only and ships in the Pi's imx500-tools apt package, so `just
# model-install` has to run first. Once the files are committed, other rovers
# never repeat any of it.
#
# One scp with both paths quoted => one connection, one password prompt.
model-fetch:
    mkdir -p model/imx500
    scp "{{target}}:{{imx_net}} {{imx_data}}/labels.txt" model/imx500/
    @ls -lh model/imx500/network.rpk model/imx500/labels.txt
    @echo "==> fetched from {{host}}. Commit these, then 'just install' ships them."

# Does the sensor actually see anything? Runs the SAME decode the rover uses.
#
# Sources robot.env first, so this tests the network the SERVICE runs. systemd
# hands that file to the service via EnvironmentFile, which an ssh command shell
# knows nothing about — without this the tool falls back to the config.py
# defaults (the COCO zoo .rpk, no labels) and cheerfully reports a working
# detector that is not the one you deployed.
model-selftest *ARGS:
    ssh -t {{target}} "cd {{app_dir}} && set -a; \
        [ -f /etc/roversoftware/robot.env ] && . /etc/roversoftware/robot.env; \
        set +a; python3 tools/detector_selftest.py --backend imx500 {{ARGS}}"

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
    ssh -t {{bs_target}} "sudo systemctl restart {{bs_service}}"
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
    ssh -t {{bs_target}} "sudo systemctl restart {{bs_ui_service}}"
    @echo "==> synced touch UI to {{bs_host}} and restarted {{bs_ui_service}}"

# Touch-UI service controls.
bs-ui-restart:
    ssh -t {{bs_target}} "sudo systemctl restart {{bs_ui_service}}"
bs-ui-status:
    ssh {{bs_target}} "systemctl status {{bs_ui_service}} --no-pager"
bs-ui-logs:
    ssh {{bs_target}} "journalctl -u {{bs_ui_service}} -f -n 100"

# Base-station service controls.
bs-restart:
    ssh -t {{bs_target}} "sudo systemctl restart {{bs_service}}"
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
    ssh -t {{bs_target}} "sudo mkdir -p /var/lib/roversoftware && sudo mv /tmp/tiles.mbtiles /var/lib/roversoftware/tiles.mbtiles && sudo systemctl restart {{bs_service}}"
    @echo "==> pushed offline tiles to {{bs_host}}; reload the kiosk: just bs-reload"

# ── documentation ──
# The handbook is an mdBook under docs/. CI publishes it to
# https://next-competition.github.io/roversoftware/ on every push to main that
# touches docs/ — see docs/src/reference/releasing.md.
mdbook_version := "0.4.40"

# Build the book into docs/book.
book:
    mdbook build docs
    @echo "==> docs/book/index.html"

# Live-reloading preview on http://localhost:3000.
book-serve:
    mdbook serve docs --open

# Install the same mdBook version CI uses, into ~/.local/bin.
book-install:
    #!/usr/bin/env bash
    set -euo pipefail
    mkdir -p ~/.local/bin
    curl -fsSL "https://github.com/rust-lang/mdBook/releases/download/v{{mdbook_version}}/mdbook-v{{mdbook_version}}-x86_64-unknown-linux-gnu.tar.gz" \
      | tar -xz -C ~/.local/bin
    chmod +x ~/.local/bin/mdbook
    ~/.local/bin/mdbook --version

# ── releasing ──
# Cutting a release is `git tag -a vX.Y.Z && git push origin vX.Y.Z`; the rest
# of this section is for reproducing what CI does, locally.

# Build both .deb packages at a given version.
packages VERSION=version:
    VERSION={{VERSION}} ./packaging/build-deb.sh all

# Build an apt repository over dist/*.deb. Pass a key to sign it:
#   just GPG_KEY_ID=<fingerprint> apt-repo
apt-repo:
    ./packaging/apt-repo.sh dist/*.deb
    @echo "==> serve it: python3 -m http.server -d dist/apt 8080"

# Build the desktop base station for THIS platform.
desktop:
    cd basestation-ui && npm ci --no-audit --no-fund && npm run build && deno task bundle

# Build the Python sdist + wheel into dist/.
wheel:
    pipx run build
