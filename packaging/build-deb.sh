#!/usr/bin/env bash
# Build roversoftware .deb packages from a staging tree.
#
# Works on the Pi or on a Mac/Linux dev box (needs `dpkg-deb`; on macOS:
# `brew install dpkg`). Output: dist/<pkg>_<version>_all.deb
#
#   VERSION=0.1.0 ./packaging/build-deb.sh robot
#   VERSION=0.1.0 ./packaging/build-deb.sh basestation
#   VERSION=0.1.0 ./packaging/build-deb.sh all
set -euo pipefail

PKGSET="${1:-robot}"
VERSION="${VERSION:-0.1.0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIST="$ROOT/dist"

command -v dpkg-deb >/dev/null 2>&1 || {
    echo "error: dpkg-deb not found (macOS: 'brew install dpkg')." >&2
    exit 1
}

strip_caches() {
    find "$1" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
    find "$1" -type f -name '*.pyc' -delete 2>/dev/null || true
}

pack() {  # pack <build-dir> <pkg-name>
    mkdir -p "$DIST"
    local out="$DIST/$2_${VERSION}_all.deb"
    # --root-owner-group => files owned by root:root even when built as a user.
    dpkg-deb --root-owner-group --build "$1" "$out"
    echo "==> built $out"
}

build_robot() {
    local pkg="roversoftware-robot"
    local build="$ROOT/build/${pkg}_${VERSION}"
    echo "==> staging $pkg $VERSION"
    rm -rf "$build"
    mkdir -p "$build/DEBIAN" "$build/opt/roversoftware" "$build/etc/roversoftware" \
             "$build/lib/systemd/system"

    cp -R "$ROOT/robot" "$build/opt/roversoftware/"
    cp "$ROOT/run_robot.py" "$build/opt/roversoftware/"
    [ -d "$ROOT/tools" ] && cp -R "$ROOT/tools" "$build/opt/roversoftware/"
    strip_caches "$build/opt/roversoftware"

    cp "$ROOT/packaging/robot.env" "$build/etc/roversoftware/robot.env"
    cp "$ROOT/packaging/systemd/roversoftware-robot.service" "$build/lib/systemd/system/"

    # The IMX500 network the sensor loads, plus its class names. Shipped to the
    # exact path RS_VISION_IMX500_MODEL points at, so a fresh rover has a
    # working detector without anyone running `just model-deploy` first.
    #
    # Fetch them off a Pi that has them with `just model-fetch` — the .rpk is
    # built by imx500-package, which is ARM-only, so it cannot be produced here.
    # NOT a conffile: dpkg replaces it on upgrade, which is what you want when
    # the repo copy is the model of record. A locally-installed experiment in
    # /var/lib is therefore overwritten by the next `just install`.
    if [ -f "$ROOT/model/imx500/network.rpk" ]; then
        mkdir -p "$build/var/lib/roversoftware"
        cp "$ROOT/model/imx500/network.rpk" "$build/var/lib/roversoftware/network.rpk"
        cp "$ROOT/model/imx500/labels.txt"  "$build/var/lib/roversoftware/labels.txt"
        chmod 0644 "$build/var/lib/roversoftware/network.rpk" \
                   "$build/var/lib/roversoftware/labels.txt"
        echo "==> bundling IMX500 network ($(du -h "$ROOT/model/imx500/network.rpk" | cut -f1))"
    else
        echo "note: no model/imx500/network.rpk — the .deb will ship no vision" \
             "network. Get one onto a Pi with 'just model-deploy', then" \
             "'just model-fetch'." >&2
    fi

    sed "s/@VERSION@/${VERSION}/" "$ROOT/packaging/debian/control" > "$build/DEBIAN/control"
    cp "$ROOT/packaging/debian/conffiles" "$build/DEBIAN/conffiles"
    for f in postinst prerm postrm; do
        cp "$ROOT/packaging/debian/$f" "$build/DEBIAN/$f"; chmod 0755 "$build/DEBIAN/$f"
    done
    pack "$build" "$pkg"
}

build_basestation() {
    local pkg="roversoftware-basestation"
    local build="$ROOT/build/${pkg}_${VERSION}"
    local app="$build/opt/roversoftware-basestation"
    echo "==> staging $pkg $VERSION"
    rm -rf "$build"
    mkdir -p "$build/DEBIAN" "$app" "$build/etc/roversoftware" \
             "$build/lib/systemd/system" "$build/etc/xdg/autostart"

    # App code + the robot package it imports (config/comms/control) + entry point.
    cp -R "$ROOT/basestation" "$app/"
    cp -R "$ROOT/robot" "$app/"
    cp "$ROOT/run_basestation.py" "$app/"
    cp "$ROOT/packaging/basestation/kiosk.sh" "$app/"; chmod 0755 "$app/kiosk.sh"
    strip_caches "$app"

    # Deno touch UI: build the client, then stage the served artifacts under ui/.
    # (Served by roversoftware-ui.service via Deno; needs deno on the target PATH.)
    if [ ! -d "$ROOT/basestation-ui/dist" ] || [ "${BUILD_UI:-1}" = "1" ]; then
        echo "==> building basestation-ui (npm run build)"
        ( cd "$ROOT/basestation-ui" && npm ci --no-audit --no-fund >/dev/null 2>&1 || npm install --no-audit --no-fund >/dev/null 2>&1; npm run build )
    fi
    if [ -d "$ROOT/basestation-ui/dist" ]; then
        mkdir -p "$app/ui"
        cp -R "$ROOT/basestation-ui/dist" "$app/ui/"
        cp -R "$ROOT/basestation-ui/server" "$app/ui/"
        cp "$ROOT/basestation-ui/deno.json" "$app/ui/"
    else
        echo "warning: basestation-ui/dist missing; UI service will have nothing to serve" >&2
    fi

    cp "$ROOT/packaging/basestation/basestation.env" "$build/etc/roversoftware/basestation.env"
    cp "$ROOT/packaging/basestation/roversoftware-basestation.service" "$build/lib/systemd/system/"
    cp "$ROOT/packaging/basestation/roversoftware-ui.service" "$build/lib/systemd/system/"
    cp "$ROOT/packaging/basestation/roversoftware-kiosk.desktop" "$build/etc/xdg/autostart/"

    sed "s/@VERSION@/${VERSION}/" "$ROOT/packaging/basestation/control" > "$build/DEBIAN/control"
    cp "$ROOT/packaging/basestation/conffiles" "$build/DEBIAN/conffiles"
    for f in postinst prerm postrm; do
        cp "$ROOT/packaging/basestation/$f" "$build/DEBIAN/$f"; chmod 0755 "$build/DEBIAN/$f"
    done
    pack "$build" "$pkg"
}

case "$PKGSET" in
    robot) build_robot ;;
    basestation) build_basestation ;;
    all) build_robot; build_basestation ;;
    *) echo "usage: build-deb.sh [robot|basestation|all]" >&2; exit 1 ;;
esac
