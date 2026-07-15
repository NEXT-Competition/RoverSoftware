#!/usr/bin/env python3
"""Download map tiles for an area into an .mbtiles file for offline dashboard use.

Run this ONCE with internet, then ship the result to the base station (see
`just bs-push-tiles`). The dashboard serves it at /tiles/{z}/{x}/{y}.png and,
when configured, falls back online for anything not cached.

    # A ~5 km radius around a point, zoom 12-17:
    python tools/fetch_tiles.py --center 37.7749 -122.4194 --radius-km 5 \
        --min-zoom 12 --max-zoom 17 --out dist/tiles.mbtiles

    # Or an explicit bounding box (west south east north, in lon/lat):
    python tools/fetch_tiles.py --bbox -122.52 37.70 -122.36 37.81 \
        --min-zoom 12 --max-zoom 17 --out dist/tiles.mbtiles

Tiles are stored TMS-flipped per the MBTiles spec, so the file also opens in
QGIS / other MBTiles tools. Re-running resumes: tiles already present are kept.

Be a good citizen: the default source is the public OpenStreetMap server, whose
usage policy discourages bulk downloads. Keep areas modest, leave the rate low,
and for anything large point --url at your own/commercial tile source.
"""

import argparse
import math
import os
import sqlite3
import sys
import time
import urllib.request

SCHEMA = """
CREATE TABLE IF NOT EXISTS metadata (name text, value text);
CREATE TABLE IF NOT EXISTS tiles (
    zoom_level integer, tile_column integer, tile_row integer, tile_data blob);
CREATE UNIQUE INDEX IF NOT EXISTS tile_index
    ON tiles (zoom_level, tile_column, tile_row);
"""

_MAX_LAT = 85.05112878  # web-mercator clamp


def deg2num(lat, lon, z):
    lat = max(min(lat, _MAX_LAT), -_MAX_LAT)
    n = 1 << z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return max(0, min(n - 1, x)), max(0, min(n - 1, y))


def bbox_from_center(lat, lon, radius_km):
    dlat = radius_km / 111.0
    dlon = radius_km / (111.0 * max(0.01, math.cos(math.radians(lat))))
    return (lon - dlon, lat - dlat, lon + dlon, lat + dlat)


def tile_ranges(west, south, east, north, z):
    x0, y0 = deg2num(north, west, z)   # top-left
    x1, y1 = deg2num(south, east, z)   # bottom-right
    return range(min(x0, x1), max(x0, x1) + 1), range(min(y0, y1), max(y0, y1) + 1)


def flip_y(z, y):
    return (1 << z) - 1 - y


def main():
    p = argparse.ArgumentParser(description="Build an offline .mbtiles tile cache")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--bbox", nargs=4, type=float, metavar=("W", "S", "E", "N"),
                   help="bounding box: west south east north (lon/lat degrees)")
    g.add_argument("--center", nargs=2, type=float, metavar=("LAT", "LON"),
                   help="center lat lon (use with --radius-km)")
    p.add_argument("--radius-km", type=float, default=2.0, help="radius for --center")
    p.add_argument("--min-zoom", type=int, default=12)
    p.add_argument("--max-zoom", type=int, default=17)
    p.add_argument("--out", default="dist/tiles.mbtiles", help="output .mbtiles path")
    p.add_argument("--url", default="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                   help="tile URL template")
    p.add_argument("--rate", type=float, default=2.0, help="max tiles/sec (be polite)")
    p.add_argument("--user-agent", default="uc-chassis-basestation tile fetch (offline map)")
    p.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    args = p.parse_args()

    if args.center:
        west, south, east, north = bbox_from_center(args.center[0], args.center[1], args.radius_km)
    else:
        west, south, east, north = args.bbox
    if west > east or south > north:
        sys.exit("error: bbox looks inverted; expected --bbox W S E N (lon/lat)")

    # Plan the work.
    plan = []
    total = 0
    for z in range(args.min_zoom, args.max_zoom + 1):
        xs, ys = tile_ranges(west, south, east, north, z)
        plan.append((z, xs, ys))
        total += len(xs) * len(ys)

    est_mb = total * 20 / 1024  # ~20 KB/tile rule of thumb
    print(f"area: W{west:.4f} S{south:.4f} E{east:.4f} N{north:.4f}")
    print(f"zoom {args.min_zoom}-{args.max_zoom}: {total} tiles (~{est_mb:.0f} MB), "
          f"~{total / max(args.rate, 0.1) / 60:.1f} min at {args.rate}/s")
    if not args.yes:
        if input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
            sys.exit("aborted")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    conn = sqlite3.connect(args.out)
    conn.executescript(SCHEMA)

    def have(z, x, y):
        return conn.execute(
            "SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, flip_y(z, y))).fetchone() is not None

    done = fetched = skipped = failed = 0
    interval = 1.0 / max(args.rate, 0.1)
    for z, xs, ys in plan:
        for x in xs:
            for y in ys:
                done += 1
                if have(z, x, y):
                    skipped += 1
                    continue
                url = args.url.format(z=z, x=x, y=y)
                try:
                    req = urllib.request.Request(url, headers={"User-Agent": args.user_agent})
                    with urllib.request.urlopen(req, timeout=10) as r:
                        data = r.read()
                    conn.execute(
                        "INSERT OR REPLACE INTO tiles "
                        "(zoom_level, tile_column, tile_row, tile_data) VALUES (?,?,?,?)",
                        (z, x, flip_y(z, y), data))
                    fetched += 1
                    if fetched % 25 == 0:
                        conn.commit()
                        print(f"  {done}/{total}  z{z}  fetched={fetched} skipped={skipped} failed={failed}")
                    time.sleep(interval)
                except Exception as e:
                    failed += 1
                    print(f"  ! z{z}/{x}/{y}: {e}")

    # MBTiles metadata (so QGIS et al. and our TileStore know the extent).
    meta = {
        "name": "uc-chassis offline tiles",
        "format": "png",
        "type": "baselayer",
        "version": "1.0",
        "minzoom": str(args.min_zoom),
        "maxzoom": str(args.max_zoom),
        "bounds": f"{west},{south},{east},{north}",
        "center": f"{(west + east) / 2},{(south + north) / 2},{args.max_zoom}",
    }
    conn.execute("DELETE FROM metadata")
    conn.executemany("INSERT INTO metadata (name, value) VALUES (?, ?)", list(meta.items()))
    conn.commit()
    conn.close()

    size_mb = os.path.getsize(args.out) / 1024 / 1024
    print(f"\ndone: {fetched} fetched, {skipped} already present, {failed} failed")
    print(f"wrote {args.out} ({size_mb:.1f} MB)")
    if failed:
        print("some tiles failed; re-run the same command to retry just those.")


if __name__ == "__main__":
    main()
