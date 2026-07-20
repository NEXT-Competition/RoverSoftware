"""Top-level robot orchestrator: wires hardware, comms, and control together."""

from __future__ import annotations

import os
import queue
import signal
import time
from typing import Callable, Dict, Optional, Tuple

from .config import RobotConfig
from .comms.xbee_link import XBeeLink
from .control.controller import Controller
from .control.manager import ControlManager
from .control.object_align import ObjectAlignController
from .control.teleop import TeleopController
from .control.waypoint import WaypointController
from .drive.tank_drive import TankDrive
from .sensors.bno055 import IMU
from .sensors.detector import MockDetector, ObjectDetector
from .sensors.gps import GPS
from .sensors.pose import PoseEstimator

# Log a warning if a control tick's work (excluding the sleep) exceeds this. A
# healthy tick is a few ms; a stall points at blocking I/O (serial or I2C).
SLOW_TICK_S = 0.1


class Robot:
    def __init__(self, config: RobotConfig, controllers: Optional[Dict[str, Controller]] = None):
        self.cfg = config
        self.drive = TankDrive(config.drive)

        # Default controller set. Autonomy controllers are registered here so
        # mode-switching works today; they hold the robot still until their
        # sensor providers (camera target / GPS pose) are attached.
        if controllers is None:
            controllers = {
                "teleop": TeleopController(config.comms.command_timeout),
                "object_align": ObjectAlignController(
                    standoff_size=config.vision.standoff_size,
                    search_speed=config.vision.search_speed,
                    hfov_deg=config.vision.hfov_deg,
                ),
                "waypoint": WaypointController(),
            }
        self.manager = ControlManager(controllers, config.start_mode)

        # Messages arrive on the XBee reader thread; process them on the main
        # loop by funneling through a thread-safe queue.
        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self.link = XBeeLink(config.comms.port, config.comms.baud, self._inbox.put)
        self._running = False

        # GPS (NEO-6M) feeds waypoint navigation and position telemetry. Reads on
        # its own thread; pose() is a cheap cached lookup for the control loop.
        # Disabled -> no position, and waypoint mode holds position.
        self.gps: Optional[GPS] = (
            GPS(config.gps.port, config.gps.baud,
                config.gps.fix_timeout, config.gps.min_move_mps)
            if config.gps.enabled else None
        )

        # BNO055 IMU supplies an absolute, standstill-valid heading (the compass
        # the GPS lacks). Reads on its own thread; heading() is a cheap cached
        # lookup. Disabled/uncalibrated -> heading falls back to GPS course.
        self.imu: Optional[IMU] = (
            IMU(config.imu.i2c_address, config.imu.heading_offset_deg,
                config.imu.invert, config.imu.min_calib,
                config.imu.calibration_path or None)
            if config.imu.enabled else None
        )

        # Fuse GPS position + IMU heading behind one pose_provider (unchanged shape:
        # () -> (lat, lon, heading_deg)), so telemetry and the waypoint controller
        # get the best heading without knowing which sensor produced it.
        self.pose_estimator = PoseEstimator(self.gps, self.imu)
        self.pose_provider: Optional[Callable[[], Optional[Tuple[float, float, Optional[float]]]]] = (
            self.pose_estimator.pose if (self.gps is not None) else None
        )
        self._last_telem = 0.0

        # Edge Impulse object detection feeds object_align. Runs the camera and
        # the model on its own thread; detection() is a cheap cached lookup, so
        # inference (50-200ms) never lands inside a control tick. No model / no
        # camera / no deps -> stays inert and object_align holds still.
        self.detector = None
        if config.vision.enabled:
            mock = os.environ.get("RS_MOCK_DETECTOR", "").strip().lower() in ("1", "true", "yes", "on")
            self.detector = MockDetector() if mock else ObjectDetector(config.vision, config.camera)

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
        oa = controllers.get("object_align")
        if isinstance(oa, ObjectAlignController) and self.detector is not None:
            oa.set_detection_provider(self.detector.detection)
            oa.set_rate_provider(self.pose_estimator.heading_rate)

    def _drain_inbox(self) -> None:
        while True:
            try:
                msg = self._inbox.get_nowait()
            except queue.Empty:
                return
            to = msg.get("to")
            if to is not None and to != self.cfg.robot_id and to != "all":
                continue  # addressed to a different robot on the shared channel
            self.manager.handle_message(msg)

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
        # Surface IMU calibration (sys, gyro, accel, mag) so the base station can
        # tell whether the heading is trustworthy or still falling back to GPS.
        if self.imu is not None:
            t["imu_calib"] = self.imu.calibration()
        # Vision summary (target, error, size, fps) so the base station can see
        # what the model sees — this is what makes standoff tunable in the field.
        # A summary, never boxes or frames: the radio is 57600 baud and shared.
        if self.detector is not None:
            t["vision"] = self.detector.telemetry()
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
        # Last: it's the heaviest to bring up (spawns the .eim subprocess) and
        # nothing else depends on it.
        if self.detector is not None:
            self.detector.start()
        self._running = True

    def run(self) -> None:
        self.start()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        period = 1.0 / self.cfg.loop_hz
        last = time.monotonic()
        print(f"[Robot] running at {self.cfg.loop_hz:.0f} Hz, start mode '{self.manager.mode}'")
        try:
            while self._running:
                now = time.monotonic()
                dt = now - last
                last = now

                self._drain_inbox()
                t1 = time.monotonic()
                cmd = self.manager.update(dt)
                t2 = time.monotonic()
                self.drive.drive(cmd.left, cmd.right)
                t3 = time.monotonic()

                if self.cfg.telemetry_hz > 0 and (now - self._last_telem) >= 1.0 / self.cfg.telemetry_hz:
                    self._last_telem = now
                    self.link.send(self._telemetry(cmd))
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
        # First: it owns a subprocess, and stopping it early means no more
        # detections can arrive while the rest of the stack winds down.
        if self.detector is not None:
            self.detector.stop()
        self.link.stop()
        if self.gps is not None:
            self.gps.stop()
        if self.imu is not None:
            self.imu.stop()
