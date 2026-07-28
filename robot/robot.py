"""Top-level robot orchestrator: wires hardware, comms, and control together."""

from __future__ import annotations

import collections
import os
import queue
import signal
import time
from typing import Callable, Dict, Optional, Tuple

from . import tuning
from .config import PIDConfig, RobotConfig
from .comms.xbee_link import XBeeLink
from .control.controller import Controller
from .control.manager import ControlManager
from .control.object_align import ObjectAlignController
from .control.pid import PID
from .control.shooter_align import ShooterAlignController
from .control.teleop import TeleopController
from .control.waypoint import WaypointController
from .drive.drivetrain import build_drivetrain
from .drive.mechanism import Mechanism, build_mechanism
from .drive.shooter import Shooter
from .sensors.bno085 import IMU
from .sensors.camera import Camera
from .sensors.detector import MockDetector, ObjectDetector
from .sensors.fpv import FPVStreamer
from .sensors.gps import GPS
from .sensors.imx500 import IMX500Detector, resolve_backend
from .sensors.pose import PoseEstimator

# Log a warning if a control tick's work (excluding the sleep) exceeds this. A
# healthy tick is a few ms; a stall points at blocking I/O (serial or I2C).
SLOW_TICK_S = 0.1

# Frames drained from the outbox per control tick. At 50 Hz this is 100 frames
# a second of headroom, which empties a ~10-frame config snapshot in 100 ms
# while never handing the radio more than it can write inside one tick. See
# Robot._queue for why bursting is the failure mode worth designing against.
OUTBOX_PER_TICK = 2


def _pid(cfg: PIDConfig) -> PID:
    return PID(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
               out_limit=cfg.out_limit, i_limit=cfg.i_limit)


def _retune(pid: PID, cfg: PIDConfig) -> None:
    """Copy gains onto a live PID without touching its integrator.

    Deliberately not a reset: retuning mid-run should nudge the loop, not make
    the robot forget where it was pointing and lurch.
    """
    pid.kp, pid.ki, pid.kd = cfg.kp, cfg.ki, cfg.kd
    pid.out_limit, pid.i_limit = cfg.out_limit, cfg.i_limit


class Robot:
    def __init__(self, config: RobotConfig, controllers: Optional[Dict[str, Controller]] = None):
        self.cfg = config
        # Whichever drivetrain this robot's layout describes. Tank on a stock
        # build; the command interface is identical either way, which is why
        # nothing below the controllers had to change.
        self.drive = build_drivetrain(config.drive)

        # Servo-actuated launcher for shooter_align. Off unless the build has
        # one: constructing it drives its PWM channel to the rest angle, which
        # on a chassis without a launcher is an unused channel twitching at boot.
        # None -> shooter_align degrades to plain alignment and never fires.
        self.shooter: Optional[Shooter] = (
            Shooter(config.shooter) if config.shooter.enabled else None
        )

        # Everything else that moves: intakes, arms, extra launchers. Built from
        # the layout, empty on a stock build. The built-in shooter is registered
        # alongside them under its reserved name so a routine can address every
        # mechanism the same way, while keeping its own ShooterConfig, its own
        # RS_SHOOTER_* vars and its own firing policy.
        self.mechanisms: Dict[str, Mechanism] = {
            name: build_mechanism(mech)
            for name, mech in config.mechanisms.items() if mech.enabled
        }
        # Edge state for the e-stop hook in run(); see _apply_estop.
        self._estop_latched = False
        # Multi-frame replies (a config snapshot, a layout, a routine set) are
        # drained a couple of frames per tick rather than burst at the radio.
        # See _queue.
        self._outbox: "collections.deque[dict]" = collections.deque()

        # Default controller set. Autonomy controllers are registered here so
        # mode-switching works today; they hold the robot still until their
        # sensor providers (camera target / GPS pose) are attached.
        if controllers is None:
            # Tuning (gains, speeds, thresholds) comes from config.align /
            # config.nav rather than the controllers' own defaults, so the base
            # station's settings page has something to write to. _push_live_config
            # copies later edits back onto these same objects.
            a, v = config.align, config.vision
            controllers = {
                "teleop": TeleopController(config.comms.command_timeout),
                "object_align": ObjectAlignController(
                    forward_speed=a.forward_speed,
                    pivot_threshold=a.pivot_threshold,
                    aligned_tolerance=a.aligned_tolerance,
                    search_after=a.search_after,
                    search_timeout=a.search_timeout,
                    standoff_size=v.standoff_size,
                    search_speed=v.search_speed,
                    hfov_deg=v.hfov_deg,
                    pid=_pid(a.pid),
                ),
                "shooter_align": ShooterAlignController(
                    shooter=self.shooter,
                    config=config.shooter,
                    forward_speed=a.forward_speed,
                    pivot_threshold=a.pivot_threshold,
                    aligned_tolerance=a.aligned_tolerance,
                    search_after=a.search_after,
                    search_timeout=a.search_timeout,
                    standoff_size=v.standoff_size,
                    search_speed=v.search_speed,
                    hfov_deg=v.hfov_deg,
                    pid=_pid(a.pid),
                ),
                "waypoint": WaypointController(
                    arrive_radius_m=config.nav.arrive_radius_m,
                    cruise_speed=config.nav.cruise_speed,
                    acquire_speed=config.nav.acquire_speed,
                    pivot_threshold_deg=config.nav.pivot_threshold_deg,
                    heading_pid=_pid(config.nav.heading_pid),
                ),
            }
        self.manager = ControlManager(controllers, config.start_mode)

        # Messages arrive on the XBee reader thread; process them on the main
        # loop by funneling through a thread-safe queue.
        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self.link = XBeeLink(config.comms.port, config.comms.baud, self._inbox.put)
        self._running = False

        # Adafruit Ultimate GPS feeds waypoint navigation, position telemetry and
        # the track-angle heading. Reads on its own thread; pose() is a cheap
        # cached lookup for the control loop. Disabled -> no position, and
        # waypoint mode holds position.
        self.gps: Optional[GPS] = (
            GPS(config.gps.port, config.gps.baud,
                config.gps.fix_timeout, config.gps.min_move_mps,
                config.gps.update_rate_ms)
            if config.gps.enabled else None
        )

        # BNO085 IMU supplies an absolute, standstill-valid heading (which the
        # GPS track angle is not). Reads on its own thread; heading() is a cheap
        # cached lookup. Disabled/uncalibrated -> heading falls back to the track
        # angle (see heading_source).
        self.imu: Optional[IMU] = (
            IMU(config.imu.i2c_address, config.imu.heading_offset_deg,
                config.imu.invert, config.imu.min_calib,
                config.imu.persist_calibration)
            if config.imu.enabled else None
        )

        # Fuse GPS position + heading behind one pose_provider (unchanged shape:
        # () -> (lat, lon, heading_deg)), so telemetry and the waypoint controller
        # get the best heading without knowing which sensor produced it.
        self.pose_estimator = PoseEstimator(self.gps, self.imu, config.heading_source)
        self.pose_provider: Optional[Callable[[], Optional[Tuple[float, float, Optional[float]]]]] = (
            self.pose_estimator.pose if (self.gps is not None) else None
        )
        self._last_telem = 0.0

        # One camera, shared by every frame consumer (the detector and the FPV
        # streamer) — a V4L2/CSI device can't be opened twice. Only spun up if
        # something actually wants frames; it reads on its own thread so the
        # 50 Hz loop never touches the device.
        mock_det = os.environ.get("RS_MOCK_DETECTOR", "").strip().lower() in ("1", "true", "yes", "on")
        need_camera = config.camera.enabled and (
            (config.vision.enabled and not mock_det) or config.fpv.enabled
        )
        # Which detector we run also decides which capture backend to open (an
        # IMX500 detector needs the camera that loaded the sensor's network), so
        # resolve it BEFORE constructing the camera. Skipped entirely for the
        # mock, which touches no hardware.
        self.vision_backend = (
            resolve_backend(config.vision, config.camera)
            if (config.vision.enabled and not mock_det) else "mock" if mock_det else "off"
        )
        self.camera: Optional[Camera] = (
            Camera(config.camera, config.vision) if need_camera else None
        )

        # Object detection feeds object_align. Either backend reads from the
        # shared camera on its own thread and caches the result; detection() is a
        # cheap cached lookup, so neither Edge Impulse inference (50-200ms) nor a
        # tensor decode ever lands inside a control tick. No model / no camera /
        # no deps -> inert, object_align holds.
        self.detector = None
        if config.vision.enabled:
            if mock_det:
                self.detector = MockDetector()
            elif self.vision_backend == "imx500":
                self.detector = IMX500Detector(config.vision, self.camera)
            else:
                self.detector = ObjectDetector(config.vision, self.camera)

        # First-person live video to the base station. Independent of the model:
        # the feed works with just a camera and no detection stack at all. If a
        # real detector is running, its boxes are drawn onto the feed so you can
        # see what was detected — keyed on the overlays() capability rather than
        # a backend type, so both real detectors qualify and the mock (which has
        # no real frames to annotate) doesn't.
        self.fpv: Optional[FPVStreamer] = (
            FPVStreamer(config.fpv, self.camera, config.robot_id) if config.fpv.enabled else None
        )
        overlays = getattr(self.detector, "overlays", None)
        if self.fpv is not None and overlays is not None:
            self.fpv.set_overlay_provider(overlays)

        # Give the waypoint controller the fused pose source and the IMU yaw-rate
        # (for the heading PID's measured derivative).
        wp = controllers.get("waypoint")
        if isinstance(wp, WaypointController) and self.pose_provider is not None:
            wp.set_pose_provider(self.pose_provider)
            wp.set_rate_provider(self.pose_estimator.heading_rate)

        # Same idea for object_align: the detector is its "where is it", the IMU
        # yaw-rate its measured derivative. Note this is NOT gated on
        # pose_provider like the waypoint wiring above — that's None whenever the
        # GPS is off, but object_align needs no position at all. heading_rate()
        # already returns None without an IMU, so wiring it unconditionally is
        # safe; gating it would silently drop the D term on any --no-gps run.
        # Wired by TYPE, not by mode name: shooter_align is an ObjectAlignController
        # subclass and needs exactly the same perception, so keying off the name
        # would silently leave it blind (it would align to nothing and never fire).
        if self.detector is not None:
            for c in controllers.values():
                if isinstance(c, ObjectAlignController):
                    c.set_detection_provider(self.detector.detection)
                    c.set_rate_provider(self.pose_estimator.heading_rate)

    def _all_mechanisms(self) -> Dict[str, Mechanism]:
        """Layout mechanisms plus the built-in launcher, keyed by name."""
        if self.shooter is None:
            return self.mechanisms
        return {**self.mechanisms, "shooter": self.shooter}

    def _apply_estop(self) -> None:
        """Make mechanisms safe the moment the e-stop latches.

        ControlManager broadcasts on_estop() to *controllers* and then forces
        stopped() out of update(), which covers the drivetrain completely. It
        does not cover mechanisms, because mechanisms are not controllers — and
        an intake left at full power would keep spinning through an e-stop,
        which is precisely the situation the button exists for.

        Edge-detected here rather than added to ControlManager: the manager is
        about who drives, and it owns no actuators. Detecting the edge (rather
        than stopping every tick) leaves an operator free to jog a mechanism
        while the robot is safely stopped, which is what bring-up looks like.
        """
        if self.manager.estop and not self._estop_latched:
            self._estop_latched = True
            for mech in self._all_mechanisms().values():
                mech.stop()
        elif not self.manager.estop:
            self._estop_latched = False

    def _drain_inbox(self) -> None:
        while True:
            try:
                msg = self._inbox.get_nowait()
            except queue.Empty:
                return
            to = msg.get("to")
            if to is not None and to != self.cfg.robot_id and to != "all":
                continue  # addressed to a different robot on the shared channel
            # Configuration is handled here, not in ControlManager: it reaches
            # past the active controller into the drivetrain, the sensors and
            # the loop rate, none of which the manager owns.
            mtype = msg.get("type")
            if mtype == "get_config":
                self._send_config()
            elif mtype == "set_config":
                self._set_config(msg)
            else:
                self.manager.handle_message(msg)

    # --- configuration ------------------------------------------------------

    def _config_frame(self, values: dict, **extra) -> dict:
        """A {"type":"config"} frame. `values` is a partial set the base station
        MERGES into its cached copy — a full snapshot on request, just the
        applied fields after an edit (the radio is shared with telemetry)."""
        return {"type": "config", "from": self.cfg.robot_id, "config": values, **extra}

    def _send_config(self) -> None:
        """Answer get_config, split into radio-sized frames (see tuning.chunks)."""
        for part in tuning.chunks(tuning.snapshot(self.cfg)):
            self._queue(self._config_frame(part))

    def _queue(self, msg: dict) -> None:
        """Hand a frame to the paced outbox instead of the radio directly.

        XBeeLink.send drops the frame and flushes when the radio isn't draining
        within its 0.2 s write timeout, so firing a whole multi-frame reply
        inside one tick is exactly the shape that vanishes under congestion —
        leaving the settings page permanently blank. The outbox spreads them
        over ticks instead. Telemetry still goes direct: it is one small frame,
        it is the thing an operator most needs to be current, and delaying it
        behind a config dump is the wrong trade.
        """
        self._outbox.append(msg)

    def _drain_outbox(self) -> None:
        for _ in range(OUTBOX_PER_TICK):
            if not self._outbox:
                return
            self.link.send(self._outbox.popleft())

    def _set_config(self, msg: dict) -> None:
        applied, rejected = tuning.apply(self.cfg, msg.get("config") or {})
        if applied:
            self._push_live_config()
        # Persist by default: an operator tuning gains in a field expects them
        # to survive the next power cycle. `"save": false` is the escape hatch
        # for trying a value without committing to it.
        error = None
        if applied and msg.get("save", True):
            error = tuning.save_overrides(applied)
            if error:
                print(f"[Robot] config applied but NOT saved: {error}")
        restart = tuning.needs_restart(applied, tuning.by_path_for(self.cfg))
        if applied:
            print(f"[Robot] config: {len(applied)} applied"
                  + (f", {len(rejected)} rejected" if rejected else "")
                  + (f", {len(restart)} need a restart" if restart else ""))
        for path, why in rejected.items():
            print(f"[Robot] config rejected {path}: {why}")
        self._queue(self._config_frame(
            applied, rejected=rejected, restart=restart, save_error=error))

    def _push_live_config(self) -> None:
        """Copy config onto the objects that cached it at construction.

        Most consumers (the motors, the shooter servo, the detector, the FPV
        streamer) read their config dataclass on every use, so mutating the
        config is enough. The ones below took a copy, and this is what makes
        `live=True` in tuning.py true for them. Idempotent and cheap — it runs
        only when a config frame arrives, never in the control loop.
        """
        cfg = self.cfg
        for c in self.manager.controllers.values():
            if isinstance(c, TeleopController):
                c.command_timeout = cfg.comms.command_timeout
            if isinstance(c, ObjectAlignController):
                c.forward_speed = cfg.align.forward_speed
                c.pivot_threshold = cfg.align.pivot_threshold
                c.aligned_tolerance = cfg.align.aligned_tolerance
                c.search_after = cfg.align.search_after
                c.search_timeout = cfg.align.search_timeout
                c.standoff_size = cfg.vision.standoff_size
                c.search_speed = cfg.vision.search_speed
                c.hfov_deg = cfg.vision.hfov_deg
                _retune(c.pid, cfg.align.pid)
            # Not an elif: shooter_align IS an ObjectAlignController and needs
            # the alignment tuning above as well as its own firing policy.
            if isinstance(c, ShooterAlignController):
                c.dwell = cfg.shooter.dwell
                c.cooldown = cfg.shooter.cooldown
                c.require_arm = cfg.shooter.require_arm
                c.require_arrived = cfg.shooter.require_arrived
                c.max_shots = cfg.shooter.max_shots
            if isinstance(c, WaypointController):
                c.arrive_radius_m = cfg.nav.arrive_radius_m
                c.cruise_speed = cfg.nav.cruise_speed
                c.acquire_speed = cfg.nav.acquire_speed
                c.pivot_threshold_deg = cfg.nav.pivot_threshold_deg
                _retune(c.heading_pid, cfg.nav.heading_pid)
        # Safe to assign directly: tuning.py restricts this to the same enum
        # PoseEstimator validates against.
        self.pose_estimator.heading_source = cfg.heading_source
        if self.imu is not None:
            self.imu.heading_offset_deg = cfg.imu.heading_offset_deg
            self.imu.invert = cfg.imu.invert
            self.imu.min_calib = cfg.imu.min_calib
            self.imu.persist_calibration = cfg.imu.persist_calibration
        if self.gps is not None:
            self.gps.fix_timeout = cfg.gps.fix_timeout
            self.gps.min_move_mps = cfg.gps.min_move_mps

    def _telemetry(self, cmd) -> dict:
        t = {
            "type": "telemetry",
            "from": self.cfg.robot_id,
            "mode": self.manager.mode,
            "estop": self.manager.estop,
            "left": round(cmd.left, 3),
            "right": round(cmd.right, 3),
        }
        if self.pose_provider is not None:
            pose = self.pose_provider()
            if pose is not None:
                t["lat"], t["lon"], t["heading"] = pose
        # Fix health (quality, satellites, HDOP, speed, track angle + its age).
        # The lat/lon above says where the robot thinks it is; this says whether
        # to believe it, which is the difference between "the GPS is broken" and
        # "it has 3 satellites under a tree".
        if self.gps is not None:
            t["gps"] = self.gps.telemetry()
        # Surface IMU calibration (sys, gyro, accel, mag) so the base station can
        # tell whether the heading is trustworthy or still falling back to the
        # GPS track angle.
        if self.imu is not None:
            t["imu_calib"] = self.imu.calibration()
        # Vision summary (target, error, size, fps) so the base station can see
        # what the model sees — this is what makes standoff tunable in the field.
        # A summary, never boxes or frames: the radio is 57600 baud and shared.
        if self.detector is not None:
            t["vision"] = self.detector.telemetry()
        # Shooter state (armed, shots, dwelling, cooldown). Only while the mode
        # is active — an operator needs to see the arm latch before it matters,
        # and it's dropped on exit anyway, so there's nothing to report elsewhere.
        active = self.manager.active
        if isinstance(active, ShooterAlignController) and self.shooter is not None:
            t["shooter"] = active.status()
        # Layout mechanisms (not the built-in launcher, which reports above via
        # the controller that owns its firing policy). Only when the build has
        # any, and only a summary — the radio is 57600 baud and shared.
        if self.mechanisms:
            t["mech"] = {name: m.status() for name, m in self.mechanisms.items()}
        return t

    def start(self) -> None:
        print("[Robot] arming ESCs (holding neutral)...")
        self.drive.arm()
        print(f"[Robot] opening XBee link on {self.cfg.comms.port} @ {self.cfg.comms.baud}...")
        self.link.start()
        if self.gps is not None:
            self.gps.start()
        if self.imu is not None:
            self.imu.start()
        # Camera before its consumers (detector, FPV) so frames are flowing when
        # they start; the detector is heaviest (it spawns the .eim subprocess, or
        # waits on the sensor's network upload).
        if self.camera is not None:
            self.camera.start()
        if self.detector is not None:
            self.detector.start()
        if self.fpv is not None:
            self.fpv.start()
        self._running = True

    def run(self) -> None:
        self.start()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        last = time.monotonic()
        print(f"[Robot] running at {self.cfg.loop_hz:.0f} Hz, start mode '{self.manager.mode}'")
        try:
            while self._running:
                # Read every tick, not once: loop_hz is tunable from the base
                # station, and a rate that only applied on reboot would be a
                # slider that appears to do nothing.
                period = 1.0 / max(self.cfg.loop_hz, 1.0)
                now = time.monotonic()
                dt = now - last
                last = now

                self._drain_inbox()
                self._apply_estop()
                t1 = time.monotonic()
                cmd = self.manager.update(dt)
                # Unconditional, and deliberately outside the controller: a mode
                # switch or an e-stop mid-shot stops update() from being called,
                # and the servo must still retract instead of stalling against
                # its stop. See robot/drive/shooter.py. The same argument applies
                # to every mechanism, so they are all ticked here.
                for mech in self._all_mechanisms().values():
                    mech.update()
                t2 = time.monotonic()
                self.drive.drive(cmd.left, cmd.right)
                t3 = time.monotonic()

                if self.cfg.telemetry_hz > 0 and (now - self._last_telem) >= 1.0 / self.cfg.telemetry_hz:
                    self._last_telem = now
                    self.link.send(self._telemetry(cmd))
                self._drain_outbox()
                t4 = time.monotonic()

                # Watchdog: a healthy tick is ~a few ms. If one blocks (I2C/servo
                # glitch, serial stall), log which phase stalled so a freeze shows
                # up in the journal instead of being silent.
                work = t4 - now
                if work > SLOW_TICK_S:
                    print(f"[Robot] slow tick {work*1e3:.0f}ms "
                          f"(inbox={(t1-now)*1e3:.0f} update={(t2-t1)*1e3:.0f} "
                          f"drive={(t3-t2)*1e3:.0f} send={(t4-t3)*1e3:.0f})")

                sleep_for = period - (time.monotonic() - now)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            self.shutdown()

    def _on_signal(self, *_):
        self._running = False

    def shutdown(self) -> None:
        print("\n[Robot] shutting down; stopping motors")
        self.drive.stop()
        # Park every mechanism at rest before anything else winds down: leaving
        # a launcher at the fire angle stalls the servo and leaves the mechanism
        # cocked, and leaving an intake powered is worse.
        for mech in self._all_mechanisms().values():
            mech.stop()
        # Stop the frame consumers, then the camera they read from. The detector
        # owns a subprocess, so stopping it early also halts inference promptly.
        if self.fpv is not None:
            self.fpv.stop()
        if self.detector is not None:
            self.detector.stop()
        if self.camera is not None:
            self.camera.stop()
        self.link.stop()
        if self.gps is not None:
            self.gps.stop()
        if self.imu is not None:
            self.imu.stop()
