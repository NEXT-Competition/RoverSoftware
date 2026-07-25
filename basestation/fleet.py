"""Fleet state: tracks every robot the base station has heard from.

Telemetry frames (from robots or the simulator) flow in via
`update_from_telemetry`; the web layer reads `snapshot()` to push the whole
fleet to the browser. Thread-safe: telemetry arrives on the link's reader
thread while the web loop reads snapshots.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

ONLINE_TIMEOUT = 3.0  # seconds without telemetry before a robot is "offline"


@dataclass
class RobotState:
    robot_id: str
    mode: str = "unknown"
    estop: bool = False
    left: float = 0.0
    right: float = 0.0
    battery: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    heading: Optional[float] = None
    # Vision summary from the robot's object detector: {ok, fps, label, conf, ex,
    # size, age}. Opaque here on purpose — the robot owns the shape; this layer
    # just forwards it. Note fields must be listed here AND in snapshot() or they
    # never reach the browser.
    vision: Optional[dict] = None
    imu_calib: Optional[int] = None  # BNO085 fused-orientation calibration level 0-3
    # Shooter summary {armed, shots, ready, cool}, present only while the robot
    # is in shooter_align. Unlike the fields above this one is NOT sticky — see
    # update_from_telemetry for why a stale arm indicator would be dangerous.
    shooter: Optional[dict] = None
    last_seen: float = 0.0
    trail: List[Tuple[float, float]] = field(default_factory=list)

    def online(self, now: float) -> bool:
        return self.last_seen > 0 and (now - self.last_seen) < ONLINE_TIMEOUT


class FleetManager:
    def __init__(self, trail_max: int = 400):
        self._robots: Dict[str, RobotState] = {}
        self._selected: Optional[str] = None
        self._lock = threading.Lock()
        self.trail_max = trail_max

    def _ensure(self, robot_id: str) -> RobotState:
        st = self._robots.get(robot_id)
        if st is None:
            st = RobotState(robot_id=robot_id)
            self._robots[robot_id] = st
            if self._selected is None:
                self._selected = robot_id  # auto-select the first robot we meet
        return st

    def update_from_telemetry(self, msg: dict, now: float) -> None:
        if msg.get("type") != "telemetry":
            return
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return
        with self._lock:
            st = self._ensure(robot_id)
            st.mode = msg.get("mode", st.mode)
            st.estop = bool(msg.get("estop", st.estop))
            st.left = float(msg.get("left", st.left))
            st.right = float(msg.get("right", st.right))
            if "battery" in msg and msg["battery"] is not None:
                st.battery = float(msg["battery"])
            if "heading" in msg and msg["heading"] is not None:
                st.heading = float(msg["heading"])
            if msg.get("vision") is not None:
                st.vision = msg["vision"]
            if msg.get("imu_calib") is not None:
                st.imu_calib = int(msg["imu_calib"])
            # Assigned unconditionally, breaking the "only overwrite when present"
            # pattern above on purpose. The robot omits this field entirely once
            # shooter_align is no longer active, and a sticky copy would leave the
            # UI showing ARMED for a mode the robot has already left — the one
            # piece of stale telemetry here that could get someone hurt.
            st.shooter = msg.get("shooter")
            if msg.get("lat") is not None and msg.get("lon") is not None:
                st.lat, st.lon = float(msg["lat"]), float(msg["lon"])
                st.trail.append((st.lat, st.lon))
                if len(st.trail) > self.trail_max:
                    del st.trail[: len(st.trail) - self.trail_max]
            st.last_seen = now

    @property
    def selected(self) -> Optional[str]:
        with self._lock:
            return self._selected

    def select(self, robot_id: Optional[str]) -> None:
        with self._lock:
            if robot_id in self._robots:
                self._selected = robot_id

    def snapshot(self, now: float) -> dict:
        with self._lock:
            robots = [
                {
                    "robot_id": st.robot_id,
                    "mode": st.mode,
                    "estop": st.estop,
                    "left": round(st.left, 3),
                    "right": round(st.right, 3),
                    "battery": st.battery,
                    "lat": st.lat,
                    "lon": st.lon,
                    "heading": st.heading,
                    "vision": st.vision,
                    "imu_calib": st.imu_calib,
                    "shooter": st.shooter,
                    "online": st.online(now),
                    "age": round(now - st.last_seen, 2) if st.last_seen else None,
                    "trail": st.trail,
                }
                for st in self._robots.values()
            ]
            return {"type": "fleet", "selected": self._selected, "robots": robots}
