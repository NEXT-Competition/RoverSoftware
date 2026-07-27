"""A simulated fleet that stands in for the XBee radio.

Drop-in replacement for XBeeLink (same start/stop/send + on_message interface),
so the entire base station — map, controller teleop, mode switching, waypoint
routes — runs on your laptop with no hardware.

Each fake robot is a simple unicycle model: tank commands become linear/angular
velocity, integrated into lat/lon/heading. In "waypoint" mode a robot will
actually drive a route you click on the map, so you can watch navigation work
end to end before the real GPS exists.

Each one also carries a real `RobotConfig` and answers get_config/set_config
exactly as a robot does, so the dashboard's settings page can be driven — and
demoed — with no hardware. The per-motor limits it honours (dead band, forward
and reverse caps) are the ones you can actually *see* take effect on the map,
which is the point: a settings page you can only test on a real rover is a
settings page that ships broken.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Dict, List, Tuple

from robot import tuning
from robot.config import RobotConfig
from robot.control.waypoint import bearing_deg, haversine_m

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
        self.cfg = RobotConfig(robot_id=rid)

    def _limit(self, value: float, motor) -> float:
        """Apply one motor's dead band and direction caps, as ESCMotor does."""
        value = _clamp(value)
        if abs(value) < motor.deadband:
            return 0.0
        return value * (motor.max_forward if value > 0 else motor.max_reverse)

    def set_arcade(self, throttle: float, steer: float) -> None:
        drive = self.cfg.drive
        self.left = self._limit(throttle + steer, drive.left)
        self.right = self._limit(throttle - steer, drive.right)

    def _auto_waypoint(self) -> None:
        if self.wp_idx >= len(self.waypoints):
            self.left = self.right = 0.0
            return
        tlat, tlon = self.waypoints[self.wp_idx]
        nav = self.cfg.nav
        if haversine_m(self.lat, self.lon, tlat, tlon) <= nav.arrive_radius_m:
            self.wp_idx += 1
            self.left = self.right = 0.0
            return
        err = (bearing_deg(self.lat, self.lon, tlat, tlon) - self.heading + 540) % 360 - 180
        steer = _clamp(err / 45.0)
        # Point-then-go, like the real controller: pivot while badly off bearing.
        if abs(err) > nav.pivot_threshold_deg:
            self.set_arcade(0.0, steer)
            return
        forward = nav.cruise_speed * max(0.2, 1.0 - abs(steer))
        self.set_arcade(forward, steer)

    def step(self, dt: float) -> None:
        if self.estop:
            self.left = self.right = 0.0
        elif self.mode == "waypoint":
            self._auto_waypoint()

        v = (self.left + self.right) / 2.0 * V_MAX
        turn = (self.left - self.right) / 2.0  # +ve => clockwise (heading increases)
        self.heading = (self.heading + turn * YAW_MAX * dt) % 360.0

        north = v * math.cos(math.radians(self.heading)) * dt
        east = v * math.sin(math.radians(self.heading)) * dt
        self.lat += north / M_PER_DEG_LAT
        self.lon += east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
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
                 origin: Tuple[float, float] = (37.7749, -122.4194), hz: float = 10.0):
        self.on_message = on_message
        self.hz = hz
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        lat0, lon0 = origin
        self.robots: Dict[str, _SimRobot] = {}
        for i in range(max(1, n_robots)):
            rid = f"rover{i + 1}"
            self.robots[rid] = _SimRobot(rid, lat0 + i * 0.0004, lon0 + i * 0.0004, heading=(i * 45) % 360)

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
                    r.step(dt)
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

    def _apply(self, r: _SimRobot, msg: dict) -> None:
        t = msg.get("type")
        # Configuration, answered exactly as robot/robot.py answers it: a full
        # snapshot when asked, the applied subset after an edit.
        if t == "get_config":
            # Chunked like the real robot (tuning.chunks), so the dashboard's
            # progressive fill-in is what you see in the simulator too — the
            # point of the simulator is that it behaves like the radio.
            for part in tuning.chunks(tuning.snapshot(r.cfg)):
                self.on_message({"type": "config", "from": r.rid, "config": part})
            return
        if t == "set_config":
            applied, rejected = tuning.apply(r.cfg, msg.get("config") or {})
            self.on_message({"type": "config", "from": r.rid, "config": applied,
                             "rejected": rejected,
                             "restart": tuning.needs_restart(applied),
                             "save_error": None})
            return
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
