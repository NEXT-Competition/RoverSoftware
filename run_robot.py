#!/usr/bin/env python3
"""Entry point for the tank-drive robot (run this on the Raspberry Pi).

    python run_robot.py --port /dev/serial0 --baud 9600 --id rover1

Defaults are read from the environment first (so the systemd service can be
configured via /etc/uc-chassis/robot.env), then overridden by CLI flags:

    UC_ROBOT_ID, UC_XBEE_PORT, UC_XBEE_BAUD, UC_START_MODE, UC_LOOP_HZ,
    UC_TELEMETRY_HZ, UC_GPS_ENABLED/PORT/BAUD, UC_IMU_ENABLED/ADDRESS/OFFSET

Without the Fusion HAT / XBee present, the servo layer falls back to a mock so
you can still exercise the control and comms logic on a laptop.
"""

import argparse
import os

from robot.config import RobotConfig
from robot.robot import Robot


def main():
    cfg = RobotConfig()
    parser = argparse.ArgumentParser(description="uc-chassis tank drive robot")
    parser.add_argument("--id", dest="robot_id",
                        default=os.environ.get("UC_ROBOT_ID", cfg.robot_id),
                        help="unique robot id on the shared XBee channel")
    parser.add_argument("--port", default=os.environ.get("UC_XBEE_PORT", cfg.comms.port),
                        help="XBee serial port")
    parser.add_argument("--baud", type=int,
                        default=int(os.environ.get("UC_XBEE_BAUD", cfg.comms.baud)))
    parser.add_argument("--mode", default=os.environ.get("UC_START_MODE", cfg.start_mode),
                        choices=["teleop", "color_align", "waypoint"])
    parser.add_argument("--hz", type=float,
                        default=float(os.environ.get("UC_LOOP_HZ", cfg.loop_hz)))
    parser.add_argument("--telemetry-hz", type=float,
                        default=float(os.environ.get("UC_TELEMETRY_HZ", cfg.telemetry_hz)),
                        help="telemetry send rate (lower to free airtime on a slow radio)")
    parser.add_argument("--mock-motors", action="store_true",
                        default=os.environ.get("UC_MOCK_MOTORS", "").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="run without the Fusion HAT: mock the motors (for comms testing)")
    parser.add_argument("--gps-port", default=os.environ.get("UC_GPS_PORT", cfg.gps.port),
                        help="NEO-6M GPS serial port (Pi UART; default /dev/ttyAMA0)")
    parser.add_argument("--gps-baud", type=int,
                        default=int(os.environ.get("UC_GPS_BAUD", cfg.gps.baud)))
    parser.add_argument("--no-gps", dest="gps", action="store_false",
                        default=os.environ.get("UC_GPS_ENABLED", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="disable the GPS reader (waypoint mode will hold position)")
    parser.add_argument("--imu-address", type=lambda x: int(x, 0),
                        default=int(os.environ.get("UC_IMU_ADDRESS", hex(cfg.imu.i2c_address)), 0),
                        help="BNO055 I2C address (default 0x28; 0x29 if ADR high)")
    parser.add_argument("--imu-offset", type=float,
                        default=float(os.environ.get("UC_IMU_OFFSET", cfg.imu.heading_offset_deg)),
                        help="heading offset (deg) to align the IMU yaw with North")
    parser.add_argument("--no-imu", dest="imu", action="store_false",
                        default=os.environ.get("UC_IMU_ENABLED", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="disable the BNO055 IMU (heading falls back to GPS course)")
    args = parser.parse_args()

    if args.mock_motors:
        os.environ["UC_MOCK_MOTORS"] = "1"  # picked up by the motor layer

    cfg.robot_id = args.robot_id
    cfg.comms.port = args.port
    cfg.comms.baud = args.baud
    cfg.start_mode = args.mode
    cfg.loop_hz = args.hz
    cfg.telemetry_hz = args.telemetry_hz
    cfg.gps.enabled = args.gps
    cfg.gps.port = args.gps_port
    cfg.gps.baud = args.gps_baud
    cfg.imu.enabled = args.imu
    cfg.imu.i2c_address = args.imu_address
    cfg.imu.heading_offset_deg = args.imu_offset

    motors = "MOCK" if args.mock_motors else "real"
    gps = f"{cfg.gps.port}@{cfg.gps.baud}" if cfg.gps.enabled else "off"
    imu = f"0x{cfg.imu.i2c_address:02x}" if cfg.imu.enabled else "off"
    print(f"[Robot] id={cfg.robot_id} port={cfg.comms.port} baud={cfg.comms.baud} "
          f"mode={cfg.start_mode} motors={motors} gps={gps} imu={imu}")
    Robot(cfg).run()


if __name__ == "__main__":
    main()
