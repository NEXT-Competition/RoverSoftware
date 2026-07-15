#!/bin/bash
# Launch the roversoftware dashboard full-screen in Chromium once the server is up.
# Invoked from the desktop session via /etc/xdg/autostart/roversoftware-kiosk.desktop.

ENV_FILE=/etc/roversoftware/basestation.env
[ -f "$ENV_FILE" ] && . "$ENV_FILE"
PORT="${RS_WEB_PORT:-8000}"
URL="http://localhost:${PORT}/"

# When relaunched outside the desktop session (e.g. over SSH via `just bs-reload`)
# the session env is empty and Chromium aborts with "Missing X server or $DISPLAY".
# Raspberry Pi OS (Bookworm+) runs a Wayland compositor (wayfire/labwc), so point
# Chromium at the running Wayland socket; fall back to X11/Xwayland otherwise.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [ -z "${WAYLAND_DISPLAY:-}" ]; then
    # Discover the compositor's socket (wayland-0, wayland-1, ...).
    for _sock in "$XDG_RUNTIME_DIR"/wayland-[0-9]*; do
        [ -S "$_sock" ] && { export WAYLAND_DISPLAY="$(basename "$_sock")"; break; }
    done
fi
# X11/Xwayland fallback if no Wayland socket was found.
export DISPLAY="${DISPLAY:-${RS_KIOSK_DISPLAY:-:0}}"
[ -z "${XAUTHORITY:-}" ] && [ -f "$HOME/.Xauthority" ] && export XAUTHORITY="$HOME/.Xauthority"

# Wait up to ~60s for the dashboard server to start responding.
for _ in $(seq 1 60); do
    curl -sf "$URL" >/dev/null 2>&1 && break
    sleep 1
done

# Best-effort: keep the screen awake (X11; harmless/ignored under Wayland).
xset s off 2>/dev/null || true
xset -dpms 2>/dev/null || true
xset s noblank 2>/dev/null || true

BIN="$(command -v chromium-browser || command -v chromium)"
if [ -z "$BIN" ]; then
    echo "roversoftware kiosk: chromium not installed" >&2
    exit 1
fi

# Clear any 'restore pages?' state from a previous unclean shutdown.
PROFILE="${HOME}/.config/roversoftware-kiosk"

exec "$BIN" \
    --kiosk --app="$URL" \
    --ozone-platform-hint=auto \
    --user-data-dir="$PROFILE" \
    --incognito --noerrdialogs --disable-infobars \
    --disable-session-crashed-bubble --disable-features=TranslateUI \
    --check-for-update-interval=31536000 \
    --overscroll-history-navigation=0 \
    --disable-pinch
