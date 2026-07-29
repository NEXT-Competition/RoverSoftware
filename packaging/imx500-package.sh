#!/usr/bin/env bash
# Build the IMX500 network file ON THE PI and install it for the rover.
#
#     bash imx500-package.sh [data-dir]        # default: /var/lib/roversoftware
#
# `just model-install` copies this here along with /tmp/packerOut.zip and
# /tmp/labels.txt and runs it. It exists as a file rather than a heredoc piped
# into `ssh ... bash -s` because that form makes the remote stdin the script
# itself, leaving sudo with no terminal to prompt at — it dies with "a terminal
# is required to read the password" right at the install step, after doing all
# the slow packaging work. As a file, `ssh -t` can allocate a TTY.
#
# Runs as the normal login user and elevates only for the copy into the data
# dir, so the packager keeps running as whoever owns /tmp/packerOut.zip.
set -euo pipefail

DATA_DIR="${1:-/var/lib/roversoftware}"
WORK=/tmp/imx500-network

if ! command -v imx500-package >/dev/null; then
    echo "imx500-package not found. Run: just model-bootstrap" >&2
    exit 1
fi
if [ ! -f /tmp/packerOut.zip ]; then
    echo "no /tmp/packerOut.zip — run: just model-install" >&2
    exit 1
fi

rm -rf "$WORK" && mkdir -p "$WORK"
imx500-package -i /tmp/packerOut.zip -o "$WORK"

# The packager names its output itself; take whatever .rpk appeared rather than
# assuming, so a tool rename does not silently install nothing.
rpk=$(find "$WORK" -name '*.rpk' | head -1)
if [ -z "$rpk" ]; then
    echo "imx500-package produced no .rpk" >&2
    exit 1
fi

sudo mkdir -p "$DATA_DIR"
sudo cp "$rpk" "$DATA_DIR/network.rpk"
sudo cp /tmp/labels.txt "$DATA_DIR/labels.txt"
sudo chmod 644 "$DATA_DIR/network.rpk" "$DATA_DIR/labels.txt"
ls -l "$DATA_DIR/network.rpk" "$DATA_DIR/labels.txt"
