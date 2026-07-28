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

from robot import layout, tuning
from robot.comms.doc_transfer import Reassembler, split
from robot.config import RobotConfig
from robot.control.waypoint import bearing_deg, haversine_m
from robot.routine import schema as routine_schema
from robot.routine import store as routine_store
from robot.routine.conditions import RoutineContext
from robot.routine.engine import RoutineEngine

# The modes a simulated robot advertises, so a routine that delegates to one is
# accepted by the same validator the rover uses.
SIM_CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint", "routine")

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
        # Documents, answered with the REAL validator and run with the REAL
        # engine — see the module docstring. A state-machine editor you can only
        # test on a rover is a state-machine editor that ships broken.
        self.routine_doc: dict = routine_store.empty_doc()
        self.layout_rev = 0
        self.routines_rev = 0
        self.jogging: Dict[str, float] = {}
        self.routines: Dict[str, object] = {}
        self.engine = None
        self.mech_power: Dict[str, float] = {}
        self._events: set = set()

    def _limit(self, value: float, motor) -> float:
        """Apply one motor's dead band and direction caps, as ESCMotor does."""
        value = _clamp(value)
        if motor is None:
            return value
        if abs(value) < motor.deadband:
            return 0.0
        return value * (motor.max_forward if value > 0 else motor.max_reverse)

    def _first(self, names) -> object:
        for name in names:
            motor = self.cfg.drive.actuators.get(name)
            if motor is not None:
                return motor
        return None

    def set_arcade(self, throttle: float, steer: float) -> None:
        """Mix by the drivetrain kind this robot's layout declares.

        A servo_steer layout drives differently in the simulator than a tank
        one — including the pivot creep — so the Hardware tab's drivetrain
        picker is something you can actually watch take effect on the map,
        rather than a field you have to own a rover to test.
        """
        drive = self.cfg.drive
        if drive.kind == "none":
            self.left = self.right = 0.0
            return
        if drive.kind in ("servo_steer", "single"):
            motor = self._first(drive.roles.throttle)
            if drive.kind == "single":
                steer = 0.0
            elif (drive.min_pivot_throttle > 0 and abs(steer) > 0.01
                    and abs(throttle) < drive.min_pivot_throttle):
                # A steered chassis cannot pivot; it creeps so the steering
                # bites. See robot/drive/drivetrain.py.
                throttle = drive.min_pivot_throttle if throttle >= 0 else -drive.min_pivot_throttle
            value = self._limit(throttle, motor)
            steer = _clamp(steer * drive.steer_gain)
            self.left, self.right = value + steer, value - steer
            return
        self.left = self._limit(throttle + steer, self._first(drive.roles.left))
        self.right = self._limit(throttle - steer, self._first(drive.roles.right))

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

    # --- routines ------------------------------------------------------------

    class _SimMech:
        """Stands in for a Mechanism: records what it was told, nothing moves."""

        def __init__(self, name, owner):
            self.name, self._owner = name, owner
            self.activations = 0

        def set_power(self, power, actuator=None):
            self._owner.mech_power[self.name] = float(power)
            return True

        def apply_preset(self, name):
            values = self._owner.cfg.mechanisms[self.name].presets.get(name)
            if values is None:
                return False
            self._owner.mech_power[self.name] = (
                max(values.values(), key=abs) if values else 0.0)
            return True

        def stop(self):
            self._owner.mech_power[self.name] = 0.0

        def ready(self):
            return True

        def fire(self):
            self.activations += 1
            return True

        def status(self):
            return {"kind": self._owner.cfg.mechanisms[self.name].kind,
                    "values": {"*": round(self._owner.mech_power.get(self.name, 0.0), 3)}}

    def _routine_ctx(self) -> RoutineContext:
        mechanisms = {name: self._SimMech(name, self)
                      for name in self.cfg.mechanisms}
        return RoutineContext(
            controllers={}, mechanisms=mechanisms,
            pose=lambda: (self.lat, self.lon, self.heading),
            estop=lambda: self.estop,
            allow_arm=lambda: self.cfg.routines.allow_arm)

    def start_routine(self, routine_id: str = "") -> None:
        routine = self.routines.get(routine_id) or next(
            iter(self.routines.values()), None)
        if routine is None:
            self.engine = None
            return
        self.engine = RoutineEngine(routine, self._routine_ctx(), self.cfg.routines)
        self.engine.start()

    def _run_routine(self) -> None:
        if self.engine is None:
            self.left = self.right = 0.0
            return
        # Events first: posting them after the tick would make an operator's
        # button press take effect one frame late, which reads as a dead button.
        for name in self._events:
            self.engine.post_event(name)
        self._events.clear()
        state = self.engine.update(0.0)
        if state is None:
            # Ended. Mechanisms stop with it, as RoutineController does on the
            # rover — a routine that finishes with the intake still spinning is
            # a routine that finished unsafely.
            self.left = self.right = 0.0
            self.mech_power = {k: 0.0 for k in self.mech_power}
            return
        # Delegation: the simulator has no real controllers, so a delegating
        # state falls back to what that mode does here — waypoint actually
        # drives the clicked route, anything else holds.
        if state.drive_source == "manual":
            self.set_arcade(state.drive_throttle, state.drive_steer)
        elif state.drive_controller == "waypoint":
            self._auto_waypoint()
        else:
            self.left = self.right = 0.0

    def step(self, dt: float) -> None:
        if self.estop:
            self.left = self.right = 0.0
            self.mech_power = {k: 0.0 for k in self.mech_power}
            if self.engine is not None:
                self.engine.stop("e-stopped")
                self.engine = None
        elif self.mode == "waypoint":
            self._auto_waypoint()
        elif self.mode == "routine":
            self._run_routine()

        v = (self.left + self.right) / 2.0 * V_MAX
        turn = (self.left - self.right) / 2.0  # +ve => clockwise (heading increases)
        self.heading = (self.heading + turn * YAW_MAX * dt) % 360.0

        north = v * math.cos(math.radians(self.heading)) * dt
        east = v * math.sin(math.radians(self.heading)) * dt
        self.lat += north / M_PER_DEG_LAT
        self.lon += east / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        self.battery = max(0.0, self.battery - abs(v) * dt * 0.02)

    def telemetry(self) -> dict:
        t = {
            "type": "telemetry", "from": self.rid, "mode": self.mode, "estop": self.estop,
            "left": round(self.left, 3), "right": round(self.right, 3),
            "battery": round(self.battery, 1),
            "lat": round(self.lat, 7), "lon": round(self.lon, 7),
            "heading": round(self.heading, 1),
        }
        if self.cfg.mechanisms:
            t["mech"] = {name: {"kind": mech.kind, "values": {
                "*": round(self.mech_power.get(name, 0.0), 3)}}
                for name, mech in self.cfg.mechanisms.items()}
        if self.mode == "routine":
            t["routine"] = (self.engine.status() if self.engine is not None
                            else {"id": None, "state": None, "done": True})
        return t


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
        # robot_id -> document type -> Reassembler, mirroring the rover's own.
        self._rx: Dict[str, Dict[str, Reassembler]] = {}
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

    def _emit(self, r: _SimRobot, doc: dict, mtype: str, **extra) -> None:
        """Answer with a chunked document, exactly as the radio would.

        Chunked even though nothing here is bandwidth-limited: the point of the
        simulator is that it behaves like the link, so the reassembly path the
        dashboard uses in the field is the one it uses on a laptop.
        """
        for frame in split(doc, mtype, r.rid, txid=f"{mtype}-{extra.get('rev', 0)}",
                           **extra):
            self.on_message(frame)

    def _receive_doc(self, r: _SimRobot, mtype: str, msg: dict) -> None:
        rx = self._rx.setdefault(r.rid, {}).setdefault(mtype, Reassembler())
        doc = rx.feed(msg)
        if doc is None:
            if rx.error:
                self.on_message({"type": f"{mtype[4:]}_result", "from": r.rid,
                                 "ok": False, "errors": [rx.error],
                                 "warnings": []})
            return
        if mtype == "put_layout":
            result = layout.apply(r.cfg, doc)
            if result.ok:
                r.layout_rev += 1
                r.mech_power = {name: 0.0 for name in r.cfg.mechanisms}
            self.on_message({
                "type": "layout_result", "from": r.rid, "ok": result.ok,
                "errors": result.errors, "warnings": result.warnings,
                "rev": r.layout_rev, "save_error": None,
                # The simulator applies a layout immediately because it has no
                # constructors to protect; it still SAYS a restart is needed, so
                # the dashboard shows the same banner it will in the field.
                "restart_required": result.ok})
            return
        result = routine_schema.parse(doc, r.cfg.routines, SIM_CONTROLLERS)
        if result.ok:
            r.routine_doc = doc
            r.routines = result.routines
            r.routines_rev += 1
            r.engine = None
        self.on_message({
            "type": "routines_result", "from": r.rid, "ok": result.ok,
            "errors": result.errors, "warnings": result.warnings,
            "rev": r.routines_rev, "save_error": None})

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
                             "restart": tuning.needs_restart(
                                 applied, tuning.by_path_for(r.cfg)),
                             "save_error": None})
            return
        # --- documents, answered with the REAL validators ---
        if t == "get_fields":
            self._emit(r, {"fields": tuning.descriptors(r.cfg)}, "fields")
            return
        if t == "get_layout":
            self._emit(r, layout.to_doc(r.cfg), "layout", rev=r.layout_rev)
            return
        if t == "get_routines":
            self._emit(r, r.routine_doc, "routines", rev=r.routines_rev)
            return
        if t in ("put_layout", "put_routines"):
            self._receive_doc(r, t, msg)
            return
        if t == "select_routine":
            r.start_routine(str(msg.get("id", "")))
            return
        if t == "routine_cmd":
            cmd = str(msg.get("cmd", ""))
            if cmd in ("start", "restart"):
                r.start_routine(str(msg.get("id", "")))
            elif cmd == "stop":
                r.engine = None
                r.mech_power = {k: 0.0 for k in r.mech_power}
            return
        if t == "routine_event":
            r._events.add(str(msg.get("name", "")))
            return
        if t == "jog":
            # Same gates the rover applies, so the Hardware tab's test control
            # behaves the same way with and without hardware.
            if r.estop or r.mode != "teleop":
                return
            name = str(msg.get("mech", ""))
            if name in r.cfg.mechanisms:
                r.mech_power[name] = _clamp(float(msg.get("power", 0)))
            return
        if t == "drive":
            if "left" in msg and "right" in msg:
                r.left, r.right = _clamp(float(msg["left"])), _clamp(float(msg["right"]))
            else:
                r.set_arcade(float(msg.get("throttle", 0)), float(msg.get("steer", 0)))
        elif t == "mode":
            previous, r.mode = r.mode, msg.get("mode", r.mode)
            if r.mode not in ("waypoint", "routine"):
                r.left = r.right = 0.0
            if r.mode == "routine" and previous != "routine":
                r.start_routine()
            elif previous == "routine" and r.mode != "routine":
                r.engine = None
                r.mech_power = {k: 0.0 for k in r.mech_power}
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
