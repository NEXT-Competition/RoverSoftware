"""Top-level robot orchestrator: wires hardware, comms, and control together."""

from __future__ import annotations

import queue
import signal
import time
from typing import Callable, Dict, Optional, Tuple

from .config import RobotConfig
from .comms.xbee_link import XBeeLink
from .control.color_align import ColorAlignController
from .control.controller import Controller
from .control.manager import ControlManager
from .control.teleop import TeleopController
from .control.waypoint import WaypointController
from .drive.tank_drive import TankDrive
from .sensors.gps import GPS

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
                "color_align": ColorAlignController(),
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
        # Disabled -> pose_provider stays None and waypoint mode holds position.
        self.gps: Optional[GPS] = (
            GPS(config.gps.port, config.gps.baud,
                config.gps.fix_timeout, config.gps.min_move_mps)
            if config.gps.enabled else None
        )

        # () -> (lat, lon, heading_deg); wired to the GPS so telemetry carries
        # position and the base-station map can track this robot.
        self.pose_provider: Optional[Callable[[], Optional[Tuple[float, float, float]]]] = (
            self.gps.pose if self.gps is not None else None
        )
        self._last_telem = 0.0

        # Give the waypoint controller the same live pose source.
        wp = controllers.get("waypoint")
        if isinstance(wp, WaypointController) and self.pose_provider is not None:
            wp.set_pose_provider(self.pose_provider)

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
        return t

    def start(self) -> None:
        print("[Robot] arming ESCs (holding neutral)...")
        self.drive.arm()
        print(f"[Robot] opening XBee link on {self.cfg.comms.port} @ {self.cfg.comms.baud}...")
        self.link.start()
        if self.gps is not None:
            self.gps.start()
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
        self.link.stop()
        if self.gps is not None:
            self.gps.stop()
