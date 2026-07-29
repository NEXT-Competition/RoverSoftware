#!/usr/bin/env bash
# Build (and optionally sign) an APT repository out of .deb files.
#
# The output is a plain directory tree of static files, which is the whole
# reason this works on GitHub Pages: apt only ever issues GETs, so a repository
# is just a pool of packages plus some index files sitting next to them.
#
#   ./packaging/apt-repo.sh dist/*.deb                    # unsigned, for a laptop
#   GPG_KEY_ID=ABC123 ./packaging/apt-repo.sh dist/*.deb  # signed, for publishing
#
# INCREMENTAL: point APT_ROOT at an existing tree and the new packages are
# added to the pool alongside the old ones, then every index is regenerated
# over the whole pool. That is what lets `apt-get install pkg=0.1.0` still find
# last month's build after today's release.
#
#   APT_ROOT=/path/to/published/apt ./packaging/apt-repo.sh dist/*.deb
#
# Requires: dpkg-dev (dpkg-scanpackages), apt-utils (apt-ftparchive), gnupg.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APT_ROOT="${APT_ROOT:-$ROOT/dist/apt}"
SUITE="${SUITE:-stable}"
COMPONENT="${COMPONENT:-main}"
ORIGIN="${ORIGIN:-RoverSoftware}"
LABEL="${LABEL:-RoverSoftware}"
# Architecture: all packages still have to be advertised under the concrete
# architectures apt asks for, or a Pi silently sees an empty repository.
ARCHS="${ARCHS:-amd64 arm64 armhf}"
GPG_KEY_ID="${GPG_KEY_ID:-}"

if [ "$#" -eq 0 ]; then
    echo "usage: apt-repo.sh <deb> [deb...]" >&2
    exit 1
fi

for tool in dpkg-scanpackages apt-ftparchive; do
    command -v "$tool" >/dev/null 2>&1 || {
        echo "error: $tool not found (Debian/Ubuntu: apt-get install dpkg-dev apt-utils)" >&2
        exit 1
    }
done

POOL="$APT_ROOT/pool/$COMPONENT"
mkdir -p "$POOL"

echo "==> adding $# package(s) to $POOL"
for deb in "$@"; do
    [ -f "$deb" ] || { echo "error: no such file: $deb" >&2; exit 1; }
    cp -f "$deb" "$POOL/$(basename "$deb")"
    echo "    $(basename "$deb")"
done

# ---- indices -------------------------------------------------------------
# dpkg-scanpackages is run once from APT_ROOT so every Filename: field is a
# path relative to the repository root, which is what apt resolves against the
# sources.list URL.
echo "==> generating Packages indices"
( cd "$APT_ROOT"
  for arch in $ARCHS; do
      dir="dists/$SUITE/$COMPONENT/binary-$arch"
      mkdir -p "$dir"
      # --arch filters to that arch; `all` packages are emitted for every arch,
      # which is exactly what we want here.
      #
      # --multiversion is load-bearing: without it dpkg-scanpackages keeps only
      # the NEWEST version of each package, and the pool's older builds become
      # unreachable. That would quietly break `apt-get install pkg=0.1.0`, which
      # is how you put a whole fleet back on the build you tested.
      dpkg-scanpackages --multiversion --arch "$arch" "pool/$COMPONENT" /dev/null \
          > "$dir/Packages" 2>/dev/null
      gzip -9fk "$dir/Packages"
      printf 'Archive: %s\nComponent: %s\nOrigin: %s\nLabel: %s\nArchitecture: %s\n' \
          "$SUITE" "$COMPONENT" "$ORIGIN" "$LABEL" "$arch" > "$dir/Release"
      echo "    $dir/Packages ($(grep -c '^Package:' "$dir/Packages" || true) entries)"
  done
)

# ---- Release + signatures ------------------------------------------------
echo "==> generating Release"
( cd "$APT_ROOT/dists/$SUITE"
  apt-ftparchive \
      -o "APT::FTPArchive::Release::Origin=$ORIGIN" \
      -o "APT::FTPArchive::Release::Label=$LABEL" \
      -o "APT::FTPArchive::Release::Suite=$SUITE" \
      -o "APT::FTPArchive::Release::Codename=$SUITE" \
      -o "APT::FTPArchive::Release::Components=$COMPONENT" \
      -o "APT::FTPArchive::Release::Architectures=$ARCHS" \
      -o "APT::FTPArchive::Release::Description=RoverSoftware packages" \
      release . > Release
)

if [ -n "$GPG_KEY_ID" ]; then
    echo "==> signing Release as $GPG_KEY_ID"
    ( cd "$APT_ROOT/dists/$SUITE"
      rm -f Release.gpg InRelease
      # Both forms: InRelease is what modern apt fetches, Release.gpg is the
      # fallback an older client still asks for.
      gpg --batch --yes --default-key "$GPG_KEY_ID" -abs -o Release.gpg Release
      gpg --batch --yes --default-key "$GPG_KEY_ID" --clearsign -o InRelease Release
    )
    gpg --batch --yes --armor --export "$GPG_KEY_ID" \
        > "$APT_ROOT/roversoftware-archive-keyring.asc"
    echo "==> exported public key to $APT_ROOT/roversoftware-archive-keyring.asc"
else
    echo "note: GPG_KEY_ID unset — the repository is UNSIGNED." >&2
    echo "      Clients must add [trusted=yes] to their sources.list line." >&2
fi

cat > "$APT_ROOT/index.html" <<HTML
<!doctype html>
<meta charset="utf-8">
<title>RoverSoftware · apt repository</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font: 16px/1.6 system-ui, sans-serif; max-width: 46rem; margin: 6vh auto; padding: 0 1.2rem;
         background: #07090a; color: #e9f1f1; }
  a { color: #2ee6c5; }
  pre { background: #171e20; border: 1px solid rgba(150,180,184,.14); border-radius: 10px;
        padding: 1rem; overflow-x: auto; font-size: 13px; }
  h1 { letter-spacing: -.02em; }
  @media (prefers-color-scheme: light) {
    body { background: #d9dbd4; color: #14160f; } a { color: #0b8d75; }
    pre { background: #fbfbf8; border-color: rgba(20,22,15,.14); }
  }
</style>
<h1>RoverSoftware · apt repository</h1>
<p>Debian packages for the robot and the base station. Add it once:</p>
<pre>sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://next-competition.github.io/roversoftware/apt/roversoftware-archive-keyring.asc \\
  | sudo tee /etc/apt/keyrings/roversoftware.asc > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/roversoftware.asc] https://next-competition.github.io/roversoftware/apt $SUITE $COMPONENT" \\
  | sudo tee /etc/apt/sources.list.d/roversoftware.list > /dev/null

sudo apt-get update
sudo apt-get install roversoftware-robot</pre>
<p>Full instructions: <a href="../install/apt.html">Install from apt</a>.</p>
HTML

echo "==> repository ready at $APT_ROOT"
