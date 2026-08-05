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
from typing import Callable, Dict, List, Optional, Tuple

from robot import layout, tuning
from robot.comms.doc_transfer import Reassembler, split
from robot.config import PIDConfig, RobotConfig
from robot.control.commands import DriveCommand
from robot.control.pid import PID
from robot.control.rpm_trim import RpmTrim
from robot.control.waypoint import bearing_deg, haversine_m
from robot.control.script_controller import ScriptController
from robot.routine import schema as routine_schema
from robot.routine import store as routine_store
from robot.routine.conditions import RoutineContext
from robot.routine.engine import RoutineEngine
from robot.script import schema as script_schema
from robot.script import store as script_store

# The modes a simulated robot advertises, so a routine that delegates to one is
# accepted by the same validator the rover uses.
SIM_CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint",
                   "routine", "script")

V_MAX = 3.0          # m/s at full throttle
YAW_MAX = 60.0       # deg/s at full turn-in-place
M_PER_DEG_LAT = 111_320.0

# --- fake wheel encoders ------------------------------------------------------
# The simulated rover has the defect encoders exist to fix: its two sides do not
# turn at the same speed for the same throttle. That is not decoration. It means
# the fake rover drives a visible arc on the map when you tell it to go straight,
# switching drive.trim.mode to "match" straightens it while you watch, and the
# tuning graph draws a real loop closing around a real mismatch — so the whole
# feature can be tried, and got wrong, before anybody wires an encoder.
#
# Wheel RPM at full throttle. Deliberately NOT read from drive.trim.max_rpm: if
# the simulated hardware simply agreed with whatever you typed, `velocity` mode
# would work perfectly at every setting and the one calibration it actually
# depends on would be untestable.
SIM_MAX_RPM = 200.0
# How mismatched the two sides are, as a multiplier on the right side. 6% is a
# bad-but-believable pair of ESCs: enough to curve away over ten metres, small
# enough that it looks like drift rather than a fault.
SIM_RIGHT_GAIN = 0.94
# First-order lag from commanded throttle to actual wheel speed, seconds. Real
# mass and real gearboxes do not respond instantly, and without this the plant
# would be a pure gain — which any set of gains stabilizes, making the tuning
# page a lie.
SIM_SPINUP_S = 0.15


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else hi if v > hi else v


# A venue's worth of fake access points, and the one password that works. Not
# secrets — the simulator has no network to protect; they exist so the Network
# page can be tried, including its failure paths, before a competition.
_SIM_NETWORKS = (
    {"ssid": "Venue-Guest", "signal": 82, "secure": True},
    {"ssid": "PitCrew", "signal": 64, "secure": True},
    {"ssid": "FreeWiFi", "signal": 47, "secure": False},
    {"ssid": "hotspot-5G", "signal": 29, "secure": True},
)
_SIM_PASSWORD = "letmein"


def _retune_pid(pid: PID, cfg: PIDConfig) -> None:
    """Copy gains onto the live loop without touching its integrator — the same
    rule as robot.py::_retune, so a gain typed in Settings nudges the fake rover
    instead of making it forget where it was pointing."""
    pid.kp, pid.ki, pid.kd = cfg.kp, cfg.ki, cfg.kd
    pid.out_limit, pid.i_limit = cfg.out_limit, cfg.i_limit


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
        # And the other authoring surface, run with the REAL ScriptController
        # and the REAL sandbox for the same reason: a code editor you can only
        # test on a rover is a code editor that ships broken.
        self.script_doc: dict = script_store.empty_doc()
        self.layout_rev = 0
        self.routines_rev = 0
        self.scripts_rev = 0
        self.jogging: Dict[str, float] = {}
        # Wall-clock until which this rover is "rebooting" and says nothing.
        # See SimulatedFleet._loop and the `restart` branch of send().
        self.rebooting_until = 0.0
        self.routines: Dict[str, object] = {}
        self.engine = None
        # The real ScriptController, built lazily: constructing one per rover at
        # startup would spin up nothing, but it holds a mechanism set that only
        # exists once a layout has landed.
        self.scripts: Dict[str, object] = {}
        self.script: Optional[ScriptController] = None
        self.mech_power: Dict[str, float] = {}
        self._events: set = set()
        # The heading loop, the real one. A fake rover always knows its own
        # heading exactly, so this is always the absolute-heading loop
        # (nav.heading_pid) and never the GPS-course one.
        self.heading_pid = PID(**vars(self.cfg.nav.heading_pid))
        self._bearing: Optional[float] = None
        # The wheel-speed loop, the real one, closing around fake encoders. The
        # measured speeds lag the commanded throttles and the right side is
        # weaker (see SIM_RIGHT_GAIN), so this has an actual mismatch to correct.
        self.trim = RpmTrim(self.cfg.drive.trim)
        # True wheel speed, and the speed the fake encoder REPORTS. Two values,
        # not one, because the gap between them is dead time — and dead time is
        # what decides whether a set of gains is stable. A simulator whose
        # sensor was instantaneous would bless gains that make a real rover hunt.
        self.wheel_rpm: Dict[str, float] = {"left": 0.0, "right": 0.0}
        self.meas_rpm: Dict[str, float] = {"left": 0.0, "right": 0.0}
        self._meas_at = 0.0
        self._trimmed: Tuple[float, float] = (0.0, 0.0)
        # Which fake network this fake Pi has joined. None = not on WiFi, which
        # is the state the Network page exists to get a rover out of.
        self.wifi_ssid: Optional[str] = None

    def wifi_ip(self) -> Optional[str]:
        """An address once joined, and none before — the thing an operator
        actually looks at to know whether it worked."""
        if not self.wifi_ssid:
            return None
        return f"192.168.4.{20 + int(self.rid[-1]) if self.rid[-1].isdigit() else 20}"

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

    def _auto_waypoint(self, dt: float = 0.02) -> None:
        if self.wp_idx >= len(self.waypoints):
            self.left = self.right = 0.0
            return
        tlat, tlon = self.waypoints[self.wp_idx]
        nav = self.cfg.nav
        if haversine_m(self.lat, self.lon, tlat, tlon) <= nav.arrive_radius_m:
            self.wp_idx += 1
            self.left = self.right = 0.0
            # A new leg gets a clean loop, as WaypointController does: the
            # integral wound up chasing the last bearing means nothing about the
            # next one.
            self.heading_pid.reset()
            self._bearing = None
            return
        bearing = bearing_deg(self.lat, self.lon, tlat, tlon)
        err = (bearing - self.heading + 540) % 360 - 180
        # The REAL PID with this rover's own gains, not a hand-rolled
        # proportional law. Two reasons, and the second is the one that matters:
        # turning a gain in Settings changes how the fake rover drives, which is
        # what makes the tuning graphs testable without owning a robot.
        _retune_pid(self.heading_pid, nav.heading_pid)
        steer = self.heading_pid.update(err, dt)
        self._bearing = bearing
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

    # --- scripts -------------------------------------------------------------

    class _SimWaypointMode:
        """Enough of WaypointController for `rover.hand_over("waypoint")`.

        The simulator has no real controllers — it integrates a unicycle model
        directly — so delegation needs something with the Controller shape that
        drives the clicked route. Everything else a script can hand over to
        holds the rover still, which is what those modes do here anyway.
        """

        def __init__(self, owner):
            self._owner = owner

        def on_activate(self):
            pass

        def on_deactivate(self):
            pass

        def on_message(self, message):
            if message.get("type") == "route":
                self._owner.waypoints = [tuple(p) for p in
                                         message.get("waypoints") or []]
                self._owner.wp_idx = 0

        def update(self, dt):
            self._owner._auto_waypoint(dt)
            return DriveCommand.tank(self._owner.left, self._owner.right)

        def route_done(self):
            return self._owner.wp_idx >= len(self._owner.waypoints)

        def pid_traces(self):
            return {}

    def _script_controller(self) -> ScriptController:
        """Build the real controller against this fake rover's parts.

        Same providers the rover wires (robot/robot.py::_wire_script_controller),
        pointed at the simulation instead of at hardware — so a script that
        reads `rover.heading()` here reads the fake heading through exactly the
        code path it will read the IMU through on the bench.
        """
        controller = ScriptController(
            {"waypoint": self._SimWaypointMode(self)},
            {name: self._SimMech(name, self) for name in self.cfg.mechanisms},
            self.cfg.scripts)
        controller.set_pose_provider(lambda: (self.lat, self.lon, self.heading))
        controller.set_estop_provider(lambda: self.estop)
        controller.set_command_provider(lambda: (self.left, self.right))
        controller.set_imu_provider(lambda: {"heading": self.heading,
                                             "calib": 3})
        controller.set_gps_provider(lambda: {"fix": 1, "sats": 9, "hdop": 0.9,
                                             "speed": 0.0})
        controller.set_encoder_provider(
            lambda: {"rpm": {"left": round(self.meas_rpm["left"], 1),
                             "right": round(self.meas_rpm["right"], 1)},
                     "l": round(self.meas_rpm["left"], 1),
                     "r": round(self.meas_rpm["right"], 1)})
        return controller

    def _script_mode(self) -> ScriptController:
        if self.script is None:
            self.script = self._script_controller()
            self.script.set_scripts(self.scripts)
        return self.script

    def select_script(self, script_id: str) -> None:
        """Choose one. Starts it only if this rover is ALREADY in script mode.

        The rover's own rule (robot/control/script_controller.py): selecting
        while something else is driving must not start a thread nothing is
        draining. The dashboard sends `select_script` and then `mode`, and the
        mode change below is what starts it.
        """
        self._script_mode().on_message({"type": "select_script", "id": script_id})

    def start_script(self, script_id: str = "") -> None:
        script = self._script_mode()
        if script_id:
            script.select(script_id)
        script.on_activate()

    def script_command(self, cmd: str, script_id: str = "") -> None:
        message = {"type": "script_cmd", "cmd": cmd}
        if script_id:
            message["id"] = script_id
        self._script_mode().on_message(message)

    def _run_script(self, dt: float) -> None:
        if self.script is None:
            self.left = self.right = 0.0
            return
        command = self.script.update(dt)
        self.left, self.right = command.left, command.right

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

    def _run_routine(self, dt: float = 0.02) -> None:
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
            self._auto_waypoint(dt)
        else:
            self.left = self.right = 0.0

    def _spin_wheels(self, dt: float) -> Tuple[float, float]:
        """Turn commanded throttles into wheel speeds, and close the trim loop.

        Ordered exactly as the rover does it (robot/drive/drivetrain.py): the
        loop reads the speeds the LAST tick produced and adjusts this tick's
        throttles, which is the one-sample delay a real control loop has and the
        reason a too-hot gain oscillates here as well.
        """
        # A layout replaces cfg.drive wholesale, taking its TrimConfig with it.
        # Rebuild rather than hold a config the operator can no longer edit.
        if self.trim.cfg is not self.cfg.drive.trim:
            self.trim = RpmTrim(self.cfg.drive.trim)
        left, right = self.trim.apply(
            self.left, self.right,
            self.meas_rpm["left"], self.meas_rpm["right"], dt)
        self._trimmed = (left, right)

        # The plant: throttle -> wheel speed, with one side weaker and both
        # lagging. Speeds are what actually moves the rover below, so a trim
        # that equalizes them is a trim you can see equalize on the map.
        alpha = 1.0 if SIM_SPINUP_S <= 0 else min(1.0, dt / SIM_SPINUP_S)
        for side, cmd, gain in (("left", left, 1.0),
                                ("right", right, SIM_RIGHT_GAIN)):
            target = _clamp(cmd) * gain * SIM_MAX_RPM
            self.wheel_rpm[side] += alpha * (target - self.wheel_rpm[side])

        # The sensor, with the same two lags the real one has: a measurement
        # window it only publishes on, and a smoothing constant on top. Both are
        # read live off the config, so raising them in the settings page makes
        # the simulated loop sluggish exactly as it would on the rover.
        self._meas_at += dt
        window = max(self.cfg.drive.trim.rpm_window, dt)
        if self._meas_at >= window:
            tau = self.cfg.drive.trim.rpm_tau
            beta = 1.0 if tau <= 0 else self._meas_at / (tau + self._meas_at)
            for side in self.meas_rpm:
                self.meas_rpm[side] += beta * (
                    self.wheel_rpm[side] - self.meas_rpm[side])
            self._meas_at = 0.0
        return (self.wheel_rpm["left"] / SIM_MAX_RPM,
                self.wheel_rpm["right"] / SIM_MAX_RPM)

    def step(self, dt: float) -> None:
        if self.estop:
            self.left = self.right = 0.0
            self.mech_power = {k: 0.0 for k in self.mech_power}
            self.trim.reset()
            if self.engine is not None:
                self.engine.stop("e-stopped")
                self.engine = None
            if self.script is not None:
                self.script.on_estop()
        elif self.mode == "waypoint":
            self._auto_waypoint(dt)
        elif self.mode == "routine":
            self._run_routine(dt)
        elif self.mode == "script":
            self._run_script(dt)

        # Motion comes from the WHEELS, not the command — that gap is the entire
        # subject of the encoder feature, and a simulator that skipped it could
        # not demonstrate the thing it exists to demonstrate.
        speed_left, speed_right = self._spin_wheels(dt)
        v = (speed_left + speed_right) / 2.0 * V_MAX
        turn = (speed_left - speed_right) / 2.0  # +ve => clockwise (heading increases)
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
        # Fake encoders, in the rover's own shape (robot/drive/drivetrain.py).
        # Always present, because the simulated build always has them — that is
        # what makes the RPM readout and the fault banner something you can look
        # at before owning the hardware.
        t["enc"] = {"rpm": {"left": round(self.meas_rpm["left"], 1),
                            "right": round(self.meas_rpm["right"], 1)},
                    **self.trim.status()}
        if self.cfg.mechanisms:
            t["mech"] = {name: {"kind": mech.kind, "values": {
                "*": round(self.mech_power.get(name, 0.0), 3)}}
                for name, mech in self.cfg.mechanisms.items()}
        if self.mode == "routine":
            t["routine"] = (self.engine.status() if self.engine is not None
                            else {"id": None, "state": None, "done": True})
        if self.mode == "script":
            t["script"] = (self.script.status() if self.script is not None
                           else {"id": None, "run": False})
        # The heading loop, on the same switch and in the same shape the rover
        # uses (robot/robot.py::_telemetry) — keyed by the loop's tuning path,
        # and only while a loop is actually steering.
        if self.cfg.nav.pid_trace:
            traces = {}
            if self._bearing is not None:
                traces["nav.heading_pid"] = self.heading_pid.trace(
                    setpoint=self._bearing, measured=self.heading)
            # The wheel-speed loop, on the same switch and under the same key
            # the rover publishes it as, so the graph on the settings page is
            # fed identically with and without hardware.
            trim_trace = self.trim.trace()
            if trim_trace is not None:
                traces["drive.trim.pid"] = trim_trace
            if traces:
                t["pid"] = traces
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
                    # A rebooting rover is silent, which is the whole of what an
                    # operator sees when they press Restart: the card goes stale,
                    # then comes back. Stepping it too would have it drive on
                    # while it is supposedly down.
                    if r.rebooting_until > now:
                        continue
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

    def _wifi(self, r: _SimRobot, mtype: str, msg: dict) -> None:
        """A fake Pi's WiFi, in the shapes robot/comms/wifi.py returns.

        Here rather than left unimplemented because the Network page is exactly
        the kind of thing you want to have already used once before you are
        standing in a venue with a rover that is not on the network. The fake
        refuses a wrong password and forgets a profile, because those are the two
        paths whose UI is worth having tried.
        """
        if mtype == "scan_wifi":
            self.on_message({"type": "wifi", "from": r.rid, "ok": True,
                             "networks": list(_SIM_NETWORKS)})
            return
        if mtype == "forget_wifi":
            name = str(msg.get("ssid") or "")
            if r.wifi_ssid == name:
                r.wifi_ssid = None
            self.on_message({"type": "wifi", "from": r.rid, "ok": True,
                             "forgot": name, "ssid": r.wifi_ssid,
                             "ip": r.wifi_ip()})
            return
        if mtype == "set_wifi":
            ssid = str(msg.get("ssid") or "")
            known = next((n for n in _SIM_NETWORKS if n["ssid"] == ssid), None)
            if known is None:
                self.on_message({"type": "wifi", "from": r.rid, "ok": False,
                                 "error": f"No network with SSID {ssid!r} found.",
                                 "ssid": r.wifi_ssid, "ip": r.wifi_ip()})
                return
            if known["secure"] and str(msg.get("psk") or "") != _SIM_PASSWORD:
                # What nmcli actually says, because the copy in the UI is only
                # as good as the string it is given to show.
                self.on_message({"type": "wifi", "from": r.rid, "ok": False,
                                 "error": "Secrets were required, but not provided.",
                                 "ssid": r.wifi_ssid, "ip": r.wifi_ip()})
                return
            r.wifi_ssid = ssid
            self.on_message({"type": "wifi", "from": r.rid, "ok": True,
                             "ssid": ssid, "ip": r.wifi_ip(),
                             "signal": known["signal"]})
            return
        self.on_message({"type": "wifi", "from": r.rid, "ok": True,
                         "managed": True, "device": "wlan0",
                         "ssid": r.wifi_ssid, "ip": r.wifi_ip(),
                         "signal": 71 if r.wifi_ssid else None})

    def _apply_scripts(self, r: _SimRobot, doc: dict) -> None:
        """Validate scripts with the REAL validator and install them.

        Which means a syntax error is refused here exactly as it would be on a
        rover, with the same line number — so the editor's error path is
        something you can see working before there is any hardware to see it on.
        """
        result = script_schema.parse(doc)
        if result.ok:
            r.script_doc = doc
            r.scripts = result.scripts
            r.scripts_rev += 1
            if r.script is None:
                r.script = r._script_controller()
            r.script.set_scripts(result.scripts)
        self.on_message({
            "type": "scripts_result", "from": r.rid, "ok": result.ok,
            "errors": result.errors, "warnings": result.warnings,
            "rev": r.scripts_rev, "save_error": None})
        if result.ok:
            self._emit(r, r.script_doc, "scripts", rev=r.scripts_rev)

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
            if result.ok:
                # Echo the stored document, as the rover does — the validator
                # clamps, so what was saved is not always what was sent.
                self._emit(r, layout.to_doc(r.cfg), "layout", rev=r.layout_rev)
            return
        if mtype == "put_scripts":
            self._apply_scripts(r, doc)
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
        if result.ok:
            self._emit(r, r.routine_doc, "routines", rev=r.routines_rev)

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
        if t == "get_scripts":
            self._emit(r, r.script_doc, "scripts", rev=r.scripts_rev)
            return
        if t in ("put_layout", "put_routines", "put_scripts"):
            self._receive_doc(r, t, msg)
            return
        if t in ("get_wifi", "scan_wifi", "set_wifi", "forget_wifi"):
            self._wifi(r, t, msg)
            return
        if t == "select_script":
            r.select_script(str(msg.get("id", "")))
            return
        if t == "script_cmd":
            r.script_command(str(msg.get("cmd", "")), str(msg.get("id", "")))
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
        if t == "restart":
            # What the rover does: park everything, go quiet, come back on a
            # fresh process a few seconds later. RestartSec in the shipped unit
            # is 3 s; the extra second is the boot the rover also has to do.
            #
            # The mode goes back to the configured start mode and the routine is
            # dropped, because a new process has neither — an operator who
            # restarts mid-routine and finds it still running would have learnt
            # something untrue here.
            r.left = r.right = 0.0
            r.mech_power = {k: 0.0 for k in r.mech_power}
            r.engine = None
            r.estop = False
            r.mode = r.cfg.start_mode
            r.rebooting_until = time.monotonic() + 4.0
            return
        if t == "mech_preset":
            # Same gates the rover applies (Robot._mech_preset), so a bound
            # button behaves the same way with and without hardware: refused
            # under an e-stop, refused while a routine owns the mechanisms.
            if r.estop or r.mode == "routine":
                return
            name = str(msg.get("mech", ""))
            mech = r.cfg.mechanisms.get(name)
            if mech is None:
                return
            values = mech.presets.get(str(msg.get("preset", "")))
            if values is None:
                return
            # One number per mechanism here, as _SimMech does: nothing moves, so
            # the biggest actuator's value is what the fleet card has to show.
            r.mech_power[name] = max(values.values(), key=abs) if values else 0.0
            return
        if t == "drive":
            if "left" in msg and "right" in msg:
                r.left, r.right = _clamp(float(msg["left"])), _clamp(float(msg["right"]))
            else:
                r.set_arcade(float(msg.get("throttle", 0)), float(msg.get("steer", 0)))
        elif t == "mode":
            previous, r.mode = r.mode, msg.get("mode", r.mode)
            if r.mode not in ("waypoint", "routine", "script"):
                r.left = r.right = 0.0
                # No loop is steering any more, so stop reporting one. A frozen
                # trace left on screen is a graph that lies about what the rover
                # is doing.
                r._bearing = None
                r.heading_pid.reset()
            if r.mode == "routine" and previous != "routine":
                r.start_routine()
            elif previous == "routine" and r.mode != "routine":
                r.engine = None
                r.mech_power = {k: 0.0 for k in r.mech_power}
            if r.mode == "script" and previous != "script":
                r.start_script()
            elif previous == "script" and r.mode != "script":
                # Through the controller's own hook, not by dropping the
                # reference: leaving the mode has to unwind the script thread
                # and stop the mechanisms, which is exactly what on_deactivate
                # does on a rover.
                if r.script is not None:
                    r.script.on_deactivate()
                r.left = r.right = 0.0
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
