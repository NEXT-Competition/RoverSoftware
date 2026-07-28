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

from robot.comms.doc_transfer import Reassembler

ONLINE_TIMEOUT = 3.0  # seconds without telemetry before a robot is "offline"

# Chunked documents a robot sends, and the verdicts it returns on ones we sent.
_DOC_TYPES = ("layout", "routines", "fields")
_DOC_RESULTS = {"layout_result": "layout", "routines_result": "routines"}


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
    # GPS fix health from the robot: {fix, sats, speed, hdop, alt, track,
    # track_age}. Opaque here for the same reason as `vision` above. This is how
    # you tell "the rover is lost" from "the rover has 3 satellites", and
    # track_age is how you tell a live heading from one held since the last time
    # it moved.
    gps: Optional[dict] = None
    # Shooter summary {armed, shots, ready, cool}, present only while the robot
    # is in shooter_align. Unlike the fields above this one is NOT sticky — see
    # update_from_telemetry for why a stale arm indicator would be dangerous.
    shooter: Optional[dict] = None
    # Layout mechanisms {name: {kind, ...}} and the live FSM state
    # {id, state, t, drive, done}. Both follow `shooter`'s non-sticky rule for
    # the same reason: the robot omits them when they don't apply, and a stale
    # copy would show a state machine still running one the robot has left.
    mech: Optional[dict] = None
    routine: Optional[dict] = None
    last_seen: float = 0.0
    trail: List[Tuple[float, float]] = field(default_factory=list)
    # The robot's tunable parameters (robot/tuning.py), flat dotted paths ->
    # values, as last reported by the robot itself. Merged rather than replaced:
    # a robot answers `get_config` with everything but acknowledges a `set_config`
    # with only the fields it applied, so the base station keeps the union.
    # Empty until someone asks — it costs ~2.4 KB of radio airtime, so it is
    # fetched on demand, never polled.
    config: Dict[str, object] = field(default_factory=dict)
    # Bumped on every config change so the web layer can push the settings page
    # an update without diffing, and without putting 2.4 KB in every 30 Hz frame.
    config_rev: int = 0
    # Result of the last set_config: {"rejected": {...}, "restart": [...],
    # "save_error": str|None}. Shown next to the fields the operator just edited.
    config_result: Optional[dict] = None
    # Documents, each with its own revision so saving a routine doesn't push a
    # layout back over the radio. Reassembled from fragments, never merged —
    # half a layout is not a smaller layout (see robot/comms/doc_transfer.py).
    layout: Optional[dict] = None
    layout_rev: int = 0
    layout_result: Optional[dict] = None
    routines: Optional[dict] = None
    routines_rev: int = 0
    routines_result: Optional[dict] = None
    # Descriptors for the tunable fields the dashboard's schema.ts cannot know
    # about in advance, because the operator invented them.
    fields: List[dict] = field(default_factory=list)
    fields_rev: int = 0

    def online(self, now: float) -> bool:
        return self.last_seen > 0 and (now - self.last_seen) < ONLINE_TIMEOUT


class FleetManager:
    def __init__(self, trail_max: int = 400):
        self._robots: Dict[str, RobotState] = {}
        self._selected: Optional[str] = None
        self._lock = threading.Lock()
        self.trail_max = trail_max
        # robot_id -> document type -> Reassembler
        self._reassemblers: Dict[str, Dict[str, Reassembler]] = {}

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
            if msg.get("gps") is not None:
                st.gps = msg["gps"]
            # Assigned unconditionally, breaking the "only overwrite when present"
            # pattern above on purpose. The robot omits this field entirely once
            # shooter_align is no longer active, and a sticky copy would leave the
            # UI showing ARMED for a mode the robot has already left — the one
            # piece of stale telemetry here that could get someone hurt.
            st.shooter = msg.get("shooter")
            st.mech = msg.get("mech")
            st.routine = msg.get("routine")
            if msg.get("lat") is not None and msg.get("lon") is not None:
                st.lat, st.lon = float(msg["lat"]), float(msg["lon"])
                st.trail.append((st.lat, st.lon))
                if len(st.trail) > self.trail_max:
                    del st.trail[: len(st.trail) - self.trail_max]
            st.last_seen = now

    def handle(self, msg: dict, now: float) -> None:
        """Route one inbound frame from the radio (or the simulator).

        The single entry point for everything a robot sends, so callers don't
        have to know which frame types exist.
        """
        mtype = msg.get("type")
        if mtype == "config":
            self.update_from_config(msg)
        elif mtype in _DOC_TYPES or mtype in _DOC_RESULTS:
            self.update_from_document(msg)
        else:
            self.update_from_telemetry(msg, now)

    def update_from_config(self, msg: dict) -> Optional[str]:
        """Absorb a {"type":"config"} frame from a robot; returns its id.

        The payload is MERGED, because it arrives in two flavours: the full
        snapshot a robot sends when asked, and the applied-fields subset it
        acknowledges an edit with. Merging makes both correct and means a
        rejected field keeps showing the robot's real value, not the operator's
        wish.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        values = msg.get("config")
        with self._lock:
            st = self._ensure(robot_id)
            if isinstance(values, dict):
                st.config.update(values)
            st.config_result = {
                "rejected": msg.get("rejected") or {},
                "restart": msg.get("restart") or [],
                "save_error": msg.get("save_error"),
            }
            st.config_rev += 1
        return robot_id

    def update_from_document(self, msg: dict) -> Optional[str]:
        """Absorb a document fragment, or a robot's verdict on one we sent.

        Unlike `config`, nothing is merged. Documents arrive as numbered
        fragments and are reassembled whole (robot/comms/doc_transfer.py) — a
        half-arrived layout is not a smaller layout, it is a robot with one
        drive motor. One reassembler per robot per document type, so a layout
        and a routine save can't interleave.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        mtype = msg.get("type")
        with self._lock:
            st = self._ensure(robot_id)
            if mtype in _DOC_RESULTS:
                field_name = _DOC_RESULTS[mtype]
                setattr(st, f"{field_name}_result", {
                    "ok": bool(msg.get("ok")),
                    "errors": msg.get("errors") or [],
                    "warnings": msg.get("warnings") or [],
                    "save_error": msg.get("save_error"),
                    "restart_required": bool(msg.get("restart_required")),
                })
                setattr(st, f"{field_name}_rev",
                        getattr(st, f"{field_name}_rev") + 1)
                return robot_id

            rx = self._reassemblers.setdefault(robot_id, {})
            doc = rx.setdefault(mtype, Reassembler()).feed(msg)
            if doc is None:
                return None
            if mtype == "fields":
                st.fields = doc.get("fields") or []
                st.fields_rev += 1
            elif mtype == "layout":
                st.layout = doc
                st.layout_rev += 1
            elif mtype == "routines":
                st.routines = doc
                st.routines_rev += 1
        return robot_id

    def documents(self) -> Dict[str, dict]:
        """Per-robot layout, routines and field descriptors, for the editors.

        On the cold channel with the configs, and for the same reason: these are
        kilobytes that change when someone presses Save, not 30 times a second.
        """
        with self._lock:
            return {
                st.robot_id: {
                    "layout": st.layout,
                    "layout_rev": st.layout_rev,
                    "layout_result": st.layout_result,
                    "routines": st.routines,
                    "routines_rev": st.routines_rev,
                    "routines_result": st.routines_result,
                    "fields": st.fields,
                    "fields_rev": st.fields_rev,
                }
                for st in self._robots.values()
                if st.layout_rev or st.routines_rev or st.fields_rev
            }

    def doc_revs(self) -> Dict[str, tuple]:
        """Every revision counter, so the broadcaster can push on any change."""
        with self._lock:
            return {
                st.robot_id: (st.config_rev, st.layout_rev, st.routines_rev,
                              st.fields_rev)
                for st in self._robots.values()
            }

    def configs(self) -> Dict[str, dict]:
        """Per-robot config + the last edit's result, for the settings page.

        Pushed only when `config_rev` moves — see basestation/app.py. It is not
        part of the fleet snapshot on purpose: at 30 Hz, 2.4 KB per robot is a
        lot of bytes to spend restating a config nobody is looking at.
        """
        with self._lock:
            return {
                st.robot_id: {
                    "rev": st.config_rev,
                    "config": dict(st.config),
                    "result": st.config_result,
                }
                for st in self._robots.values()
                if st.config_rev > 0
            }

    def config_revs(self) -> Dict[str, int]:
        with self._lock:
            return {st.robot_id: st.config_rev for st in self._robots.values()}

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
                    "gps": st.gps,
                    "shooter": st.shooter,
                    "mech": st.mech,
                    "routine": st.routine,
                    "online": st.online(now),
                    "age": round(now - st.last_seen, 2) if st.last_seen else None,
                    "trail": st.trail,
                }
                for st in self._robots.values()
            ]
            return {"type": "fleet", "selected": self._selected, "robots": robots}
