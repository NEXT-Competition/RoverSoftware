#!/usr/bin/env bash
# Build uc-chassis .deb packages from a staging tree.
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
    local pkg="uc-chassis-robot"
    local build="$ROOT/build/${pkg}_${VERSION}"
    echo "==> staging $pkg $VERSION"
    rm -rf "$build"
    mkdir -p "$build/DEBIAN" "$build/opt/uc-chassis" "$build/etc/uc-chassis" \
             "$build/lib/systemd/system"

    cp -R "$ROOT/robot" "$build/opt/uc-chassis/"
    cp "$ROOT/run_robot.py" "$build/opt/uc-chassis/"
    [ -d "$ROOT/tools" ] && cp -R "$ROOT/tools" "$build/opt/uc-chassis/"
    strip_caches "$build/opt/uc-chassis"

    cp "$ROOT/packaging/robot.env" "$build/etc/uc-chassis/robot.env"
    cp "$ROOT/packaging/systemd/uc-chassis-robot.service" "$build/lib/systemd/system/"

    sed "s/@VERSION@/${VERSION}/" "$ROOT/packaging/debian/control" > "$build/DEBIAN/control"
    cp "$ROOT/packaging/debian/conffiles" "$build/DEBIAN/conffiles"
    for f in postinst prerm postrm; do
        cp "$ROOT/packaging/debian/$f" "$build/DEBIAN/$f"; chmod 0755 "$build/DEBIAN/$f"
    done
    pack "$build" "$pkg"
}

build_basestation() {
    local pkg="uc-chassis-basestation"
    local build="$ROOT/build/${pkg}_${VERSION}"
    local app="$build/opt/uc-chassis-basestation"
    echo "==> staging $pkg $VERSION"
    rm -rf "$build"
    mkdir -p "$build/DEBIAN" "$app" "$build/etc/uc-chassis" \
             "$build/lib/systemd/system" "$build/etc/xdg/autostart"

    # App code + the robot package it imports (config/comms/control) + entry point.
    cp -R "$ROOT/basestation" "$app/"
    cp -R "$ROOT/robot" "$app/"
    cp "$ROOT/run_basestation.py" "$app/"
    cp "$ROOT/packaging/basestation/kiosk.sh" "$app/"; chmod 0755 "$app/kiosk.sh"
    strip_caches "$app"

    cp "$ROOT/packaging/basestation/basestation.env" "$build/etc/uc-chassis/basestation.env"
    cp "$ROOT/packaging/basestation/uc-chassis-basestation.service" "$build/lib/systemd/system/"
    cp "$ROOT/packaging/basestation/uc-chassis-kiosk.desktop" "$build/etc/xdg/autostart/"

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
