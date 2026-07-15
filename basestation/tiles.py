"""Offline map tiles for the dashboard, backed by an MBTiles file.

The dashboard's Leaflet map requests tiles as /tiles/{z}/{x}/{y}.png. This store
answers those from a local .mbtiles cache (see tools/fetch_tiles.py) and, unless
run fully offline, fills cache misses from an upstream tile server and writes
them back — so coverage grows as you pan, and the base station keeps working
when the network later goes away.

MBTiles stores rows in TMS order (origin bottom-left) while Leaflet/XYZ uses
origin top-left, so we flip Y on every read and write.

Thread-safety: FastAPI calls get() from worker threads (via asyncio.to_thread);
a single connection (check_same_thread=False) is guarded by a lock. Network
fetches happen OUTSIDE the lock so a slow upstream never blocks cache reads.
"""

from __future__ import annotations

import os
import sqlite3
import threading
import urllib.request
from typing import Optional

_SCHEMA = """
CREATE TABLE IF NOT EXISTS metadata (name text, value text);
CREATE TABLE IF NOT EXISTS tiles (
    zoom_level integer, tile_column integer, tile_row integer, tile_data blob);
CREATE UNIQUE INDEX IF NOT EXISTS tile_index
    ON tiles (zoom_level, tile_column, tile_row);
"""


class TileStore:
    def __init__(
        self,
        mbtiles_path: Optional[str] = None,
        upstream_url: Optional[str] = None,
        allow_upstream: bool = True,
        user_agent: str = "uc-chassis-basestation offline tile cache",
    ):
        self.mbtiles_path = mbtiles_path
        self.upstream_url = upstream_url
        self.allow_upstream = allow_upstream and bool(upstream_url)
        self.user_agent = user_agent
        self.maxzoom: Optional[int] = None

        self._conn: Optional[sqlite3.Connection] = None
        self._lock = threading.Lock()

        if mbtiles_path and os.path.exists(mbtiles_path):
            try:
                self._conn = sqlite3.connect(mbtiles_path, check_same_thread=False)
                self._read_meta()
                print(f"[tiles] serving offline cache {mbtiles_path}"
                      f"{f' (maxzoom {self.maxzoom})' if self.maxzoom else ''}")
            except Exception as e:  # corrupt/locked file -> fall back to upstream only
                print(f"[tiles] could not open {mbtiles_path}: {e}")
                self._conn = None
        if self._conn is None and self.allow_upstream:
            print(f"[tiles] no local cache yet; proxying + caching from {upstream_url}")

    # ---- read ----
    def _read_meta(self) -> None:
        try:
            row = self._conn.execute(
                "SELECT value FROM metadata WHERE name='maxzoom'").fetchone()
            if row:
                self.maxzoom = int(row[0])
        except Exception:
            pass

    @staticmethod
    def _flip_y(z: int, y: int) -> int:
        return (1 << z) - 1 - y

    def _db_get(self, z: int, x: int, y: int) -> Optional[bytes]:
        if self._conn is None:
            return None
        row = self._conn.execute(
            "SELECT tile_data FROM tiles "
            "WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, self._flip_y(z, y)),
        ).fetchone()
        return bytes(row[0]) if row else None

    # ---- write-back cache ----
    def _db_put(self, z: int, x: int, y: int, data: bytes) -> None:
        try:
            if self._conn is None and self.mbtiles_path:
                os.makedirs(os.path.dirname(self.mbtiles_path) or ".", exist_ok=True)
                self._conn = sqlite3.connect(self.mbtiles_path, check_same_thread=False)
                self._conn.executescript(_SCHEMA)
            if self._conn is None:
                return
            self._conn.execute(
                "INSERT OR REPLACE INTO tiles "
                "(zoom_level, tile_column, tile_row, tile_data) VALUES (?,?,?,?)",
                (z, x, self._flip_y(z, y), data),
            )
            self._conn.commit()
        except Exception as e:
            print(f"[tiles] cache write failed: {e}")

    def _fetch_upstream(self, z: int, x: int, y: int) -> Optional[bytes]:
        if not self.allow_upstream:
            return None
        url = self.upstream_url.format(z=z, x=x, y=y)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": self.user_agent})
            with urllib.request.urlopen(req, timeout=5) as resp:
                return resp.read()
        except Exception:
            return None  # offline / upstream error -> blank tile

    def get(self, z: int, x: int, y: int) -> Optional[bytes]:
        """Return PNG bytes for a tile, or None if it can't be served."""
        with self._lock:
            data = self._db_get(z, x, y)
        if data is not None:
            return data
        # Cache miss: fetch upstream OUTSIDE the lock, then store under it.
        data = self._fetch_upstream(z, x, y)
        if data is not None:
            with self._lock:
                self._db_put(z, x, y, data)
        return data
