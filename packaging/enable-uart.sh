#!/usr/bin/env bash
# Free the Pi's good hardware UART for the GPS. Run on the Pi as root:
#
#     sudo bash packaging/enable-uart.sh        # or, from your Mac: just uart
#
# Then REBOOT — nothing here takes effect until the firmware re-reads config.txt.
#
# Why this is needed at all: on a Pi 3/4/Zero W the PL011 UART is wired to the
# on-board Bluetooth modem, and the 40-pin header gets the "mini-UART" instead.
# The mini-UART derives its baud rate from the VPU core clock, so as the clock
# throttles its timing drifts and it drops characters — NMEA sentences arrive
# truncated, which the GPS driver reports as "no fix" rather than as an error.
# `dtoverlay=disable-bt` hands the PL011 back to the header as /dev/ttyAMA0 and
# leaves Bluetooth unbound; `enable_uart=1` turns the header UART on at all.
#
# It also takes the serial login console off the port: a getty and the GPS
# cannot both own /dev/ttyAMA0, and the getty gets there first at boot.
#
# Idempotent — safe to re-run. Both files are backed up before any edit.
set -euo pipefail

MARK="# --- roversoftware: header UART for GPS ---"

if [ "$(id -u)" -ne 0 ]; then
    echo "error: run me as root (sudo bash $0)" >&2
    exit 1
fi

# Bookworm and later moved the boot partition to /boot/firmware; older Pi OS
# has it at /boot.
BOOT=/boot/firmware
[ -f "$BOOT/config.txt" ] || BOOT=/boot
CONFIG="$BOOT/config.txt"
CMDLINE="$BOOT/cmdline.txt"

if [ ! -f "$CONFIG" ]; then
    echo "error: no config.txt under /boot/firmware or /boot — is this a Pi?" >&2
    exit 1
fi

changed=0
stamp="$(date +%Y%m%d-%H%M%S)"

backup() {
    cp -a "$1" "$1.bak-$stamp"
    echo "    backed up $1 -> $1.bak-$stamp"
}

# ── config.txt ──────────────────────────────────────────────────────────────
if grep -qF "$MARK" "$CONFIG"; then
    echo "==> $CONFIG already configured"
else
    echo "==> editing $CONFIG"
    backup "$CONFIG"

    # Earlier settings can contradict ours. config.txt does not guarantee that
    # the last assignment wins, so comment the old ones out rather than trusting
    # our appended block to override them:
    #   enable_uart=0   turns the header UART back off
    #   miniuart-bt     is the opposite trade (BT keeps working, header keeps
    #                   the flaky mini-UART) and is mutually exclusive with
    #                   disable-bt
    sed -i -E \
        -e "s/^([[:space:]]*enable_uart=.*)$/#\1  # superseded by roversoftware/" \
        -e "s/^([[:space:]]*dtoverlay=miniuart-bt.*)$/#\1  # superseded by roversoftware/" \
        "$CONFIG"

    # [all] cancels any preceding [pi4]/[cm4]/[board-type] filter, so these
    # apply on every model even when appended after a conditional section.
    cat >> "$CONFIG" <<EOF

$MARK
[all]
enable_uart=1
dtoverlay=disable-bt
EOF
    changed=1
fi

# ── cmdline.txt: drop the serial console ────────────────────────────────────
if [ -f "$CMDLINE" ] && grep -Eq 'console=(serial0|ttyAMA0)' "$CMDLINE"; then
    echo "==> removing the serial console from $CMDLINE"
    backup "$CMDLINE"
    # cmdline.txt MUST stay a single line; sed edits in place without adding one.
    # The second expression tidies the leading space left when console= was the
    # first parameter.
    sed -i -E \
        -e 's/[[:space:]]*console=(serial0|ttyAMA0)[^[:space:]]*//g' \
        -e 's/^[[:space:]]+//' \
        "$CMDLINE"
    changed=1
else
    echo "==> $CMDLINE already has no serial console"
fi

# ── services that would fight us for the port ───────────────────────────────
# hciuart attaches the BT modem to the UART we just took away; without this it
# fails noisily on every boot.
systemctl disable --now hciuart.service            >/dev/null 2>&1 || true
systemctl disable --now serial-getty@ttyAMA0.service >/dev/null 2>&1 || true
systemctl disable --now serial-getty@serial0.service >/dev/null 2>&1 || true

echo
echo "==> Bluetooth disabled, header UART enabled."
echo "    The GPS port is /dev/ttyAMA0 (also /dev/serial0 on every Pi model)."
if [ "$changed" = 1 ]; then
    echo "    REBOOT for this to take effect:  sudo reboot   (or: just reboot)"
    echo "    Then check the fix:              python3 tools/gps_monitor.py"
else
    echo "    Nothing changed — already set up."
fi
