"""A simulated fleet that stands in for the XBee radio.

Drop-in replacement for XBeeLink (same start/stop/send + on_message interface),
so the entire base station — map, controller teleop, mode switching, waypoint
routes — runs on your laptop with no hardware.

Each fake robot is a simple unicycle model: tank commands become linear/angular
velocity, integrated into lat/lon/heading. In "waypoint" mode a robot will
actually drive a route you click on the map, so you can watch navigation work
end to end before the real GPS exists.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, List, Tuple

from robot.control.waypoint import bearing_deg, haversine_m
from .field import point_in_polygon

V_MAX = 3.0          # m/s at full throttle
YAW_MAX = 60.0       # deg/s at full turn-in-place
M_PER_DEG_LAT = 111_320.0


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


class _SimRobot:
    def __init__(self, rid: str, lat: float, lon: float, heading: float = 0.0):
        self.rid = rid
        self.lat, self.lon, self.heading = lat, lon, heading
        self.left = self.right = 0.0
        self.mode = "teleop"
        self.estop = False
        self.battery = 100.0
        self.waypoints: List[Tuple[float, float]] = []
        self.wp_idx = 0

    def set_arcade(self, throttle: float, steer: float) -> None:
        self.left = _clamp(throttle + steer)
        self.right = _clamp(throttle - steer)

    def _auto_waypoint(self) -> None:
        if self.wp_idx >= len(self.waypoints):
            self.left = self.right = 0.0
            return
        tlat, tlon = self.waypoints[self.wp_idx]
        if haversine_m(self.lat, self.lon, tlat, tlon) <= 2.0:
            self.wp_idx += 1
            self.left = self.right = 0.0
            return
        err = (bearing_deg(self.lat, self.lon, tlat, tlon) - self.heading + 540) % 360 - 180
        steer = _clamp(err / 45.0)
        forward = 0.5 * max(0.2, 1.0 - abs(steer))
        self.set_arcade(forward, steer)

    def step(self, dt: float, boundary=None) -> None:
        if self.estop:
            self.left = self.right = 0.0
        elif self.mode == "waypoint":
            self._auto_waypoint()

        v = (self.left + self.right) / 2.0 * V_MAX
        turn = (self.left - self.right) / 2.0  # +ve => clockwise (heading increases)
        self.heading = (self.heading + turn * YAW_MAX * dt) % 360.0

        north = v * math.cos(math.radians(self.heading)) * dt
        east = v * math.sin(math.radians(self.heading)) * dt
        new_lat = self.lat + north / M_PER_DEG_LAT
        new_lon = self.lon + east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        # ponytail: hard stop at the fence rather than sliding along it —
        # upgrade to a proper wall-slide if robots need to hug the boundary.
        # boundary is None for sites with no measured fence (e.g. open plazas),
        # so those robots just roam freely.
        if boundary is None or point_in_polygon(new_lat, new_lon, boundary):
            self.lat, self.lon = new_lat, new_lon
        self.battery = max(0.0, self.battery - abs(v) * dt * 0.02)

    def telemetry(self) -> dict:
        return {
            "type": "telemetry", "from": self.rid, "mode": self.mode, "estop": self.estop,
            "left": round(self.left, 3), "right": round(self.right, 3),
            "battery": round(self.battery, 1),
            "lat": round(self.lat, 7), "lon": round(self.lon, 7),
            "heading": round(self.heading, 1),
        }


class SimulatedFleet:
    """Duck-typed stand-in for XBeeLink."""

    def __init__(self, on_message: Callable[[dict], None], n_robots: int = 3,
                 origin: Tuple[float, float] = (38.8331773, -77.3232135), hz: float = 10.0,
                 boundary=None):
        self.on_message = on_message
        self.hz = hz
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._n_robots = max(1, n_robots)
        self.boundary = boundary
        self.robots: Dict[str, _SimRobot] = {}
        self._spawn(origin)

    def _spawn(self, origin: Tuple[float, float]) -> None:
        lat0, lon0 = origin
        self.robots = {}
        # Small spread (~15m/robot) so the default fleet spawns inside any
        # measured boundary regardless of where on the site `origin` sits.
        for i in range(self._n_robots):
            rid = f"rover{i + 1}"
            self.robots[rid] = _SimRobot(rid, lat0 + i * 0.00012, lon0 + i * 0.00012, heading=(i * 45) % 360)

    def set_site(self, origin: Tuple[float, float], boundary) -> None:
        """Move the whole fleet to a new site: respawn at its origin and swap
        the fence it's checked against (None disables the fence entirely)."""
        with self._lock:
            self.boundary = boundary
            self._spawn(origin)

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="sim", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / self.hz
        last = time.monotonic()
        while self._running:
            now = time.monotonic()
            dt = now - last
            last = now
            with self._lock:
                for r in self.robots.values():
                    r.step(dt, self.boundary)
                    self.on_message(r.telemetry())
            time.sleep(period)

    def send(self, msg: dict) -> None:
        to = msg.get("to")
        with self._lock:
            if to in (None, "all"):
                targets = list(self.robots.values())
            elif to in self.robots:
                targets = [self.robots[to]]
            else:
                targets = []
            for r in targets:
                self._apply(r, msg)

    @staticmethod
    def _apply(r: _SimRobot, msg: dict) -> None:
        t = msg.get("type")
        if t == "drive":
            if "left" in msg and "right" in msg:
                r.left, r.right = _clamp(float(msg["left"])), _clamp(float(msg["right"]))
            else:
                r.set_arcade(float(msg.get("throttle", 0)), float(msg.get("steer", 0)))
        elif t == "mode":
            r.mode = msg.get("mode", r.mode)
            if r.mode != "waypoint":
                r.left = r.right = 0.0
        elif t == "estop":
            r.estop = True
            r.left = r.right = 0.0
        elif t == "clear_estop":
            r.estop = False
        elif t == "route":
            r.waypoints = [(float(a), float(b)) for a, b in msg.get("waypoints", [])]
            r.wp_idx = 0

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
