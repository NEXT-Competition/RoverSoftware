#!/usr/bin/env python3
"""Entry point for the tank-drive robot (run this on the Raspberry Pi).

    python run_robot.py --port /dev/serial0 --baud 9600 --id rover1

Defaults are read from the environment first (so the systemd service can be
configured via /etc/roversoftware/robot.env), then overridden by CLI flags:

    RS_ROBOT_ID, RS_XBEE_PORT, RS_XBEE_BAUD, RS_START_MODE, RS_LOOP_HZ,
    RS_TELEMETRY_HZ, RS_GPS_ENABLED/PORT/BAUD/RATE_MS, RS_HEADING_SOURCE,
    RS_IMU_ENABLED/ADDRESS/OFFSET/SAVE_CALIB,
    RS_CAMERA_ENABLED/DEVICE/WIDTH/HEIGHT/FPS,
    RS_VISION_ENABLED/MODEL/LABEL/CONF/FPS/STANDOFF/HFOV/SEARCH_SPEED,
    RS_FPV_ENABLED/HOST/PORT/FPS/QUALITY,
    RS_SHOOTER_ENABLED/CHANNEL/REST/FIRE/FIRE_S/RETRACT_S/DWELL/COOLDOWN/
        MAX_SHOTS/REQUIRE_ARM/REQUIRE_ARRIVED

Without the Fusion HAT / XBee present, the servo layer falls back to a mock so
you can still exercise the control and comms logic on a laptop. Likewise
RS_MOCK_DETECTOR=1 synthesizes a moving target, so object_align can be driven
end-to-end with no camera and no model:

    RS_MOCK_MOTORS=1 RS_MOCK_DETECTOR=1 python run_robot.py --mode object_align

The same trick drives the shooter end-to-end with no launcher attached — the
servo is mocked, and every shot still prints:

    RS_MOCK_MOTORS=1 RS_MOCK_DETECTOR=1 python run_robot.py \\
        --mode shooter_align --shooter
"""

import argparse
import os

from robot.config import RobotConfig
from robot.robot import Robot


def main():
    cfg = RobotConfig()
    parser = argparse.ArgumentParser(description="RoverSoftware tank drive robot")
    parser.add_argument("--id", dest="robot_id",
                        default=os.environ.get("RS_ROBOT_ID", cfg.robot_id),
                        help="unique robot id on the shared XBee channel")
    parser.add_argument("--port", default=os.environ.get("RS_XBEE_PORT", cfg.comms.port),
                        help="XBee serial port")
    parser.add_argument("--baud", type=int,
                        default=int(os.environ.get("RS_XBEE_BAUD", cfg.comms.baud)))
    parser.add_argument("--mode", default=os.environ.get("RS_START_MODE", cfg.start_mode),
                        choices=["teleop", "object_align", "shooter_align", "waypoint"])
    parser.add_argument("--hz", type=float,
                        default=float(os.environ.get("RS_LOOP_HZ", cfg.loop_hz)))
    parser.add_argument("--telemetry-hz", type=float,
                        default=float(os.environ.get("RS_TELEMETRY_HZ", cfg.telemetry_hz)),
                        help="telemetry send rate (lower to free airtime on a slow radio)")
    parser.add_argument("--mock-motors", action="store_true",
                        default=os.environ.get("RS_MOCK_MOTORS", "").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="run without the Fusion HAT: mock the motors (for comms testing)")
    parser.add_argument("--gps-port", default=os.environ.get("RS_GPS_PORT", cfg.gps.port),
                        help="Adafruit GPS serial port (Pi UART; default /dev/ttyAMA0)")
    parser.add_argument("--gps-baud", type=int,
                        default=int(os.environ.get("RS_GPS_BAUD", cfg.gps.baud)))
    parser.add_argument("--gps-rate", type=int,
                        default=int(os.environ.get("RS_GPS_RATE_MS", cfg.gps.update_rate_ms)),
                        help="ms between fixes (PMTK220; 1000 = 1 Hz). Below ~200 "
                             "the sentences don't fit 9600 baud")
    parser.add_argument("--no-gps", dest="gps", action="store_false",
                        default=os.environ.get("RS_GPS_ENABLED", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="disable the GPS reader (waypoint mode will hold position)")
    parser.add_argument("--heading-source",
                        default=os.environ.get("RS_HEADING_SOURCE", cfg.heading_source),
                        choices=["auto", "gps", "imu"],
                        help="which sensor gives heading: auto (IMU, else the GPS "
                             "track angle), gps (track angle only), imu (no fallback)")
    parser.add_argument("--imu-address", type=lambda x: int(x, 0),
                        default=int(os.environ.get("RS_IMU_ADDRESS", hex(cfg.imu.i2c_address)), 0),
                        help="BNO085 I2C address (default 0x4a; 0x4b if DI/AD0 high)")
    parser.add_argument("--imu-offset", type=float,
                        default=float(os.environ.get("RS_IMU_OFFSET", cfg.imu.heading_offset_deg)),
                        help="heading offset (deg) to align the IMU yaw with North")
    parser.add_argument("--no-imu", dest="imu", action="store_false",
                        default=os.environ.get("RS_IMU_ENABLED", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="disable the BNO085 IMU (heading falls back to the GPS track angle)")
    parser.add_argument("--no-imu-save-calib", dest="imu_save_calib", action="store_false",
                        default=os.environ.get("RS_IMU_SAVE_CALIB", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="don't auto-save the BNO085's calibration to its on-chip flash")
    # Vision. Only the flags you actually reach for in the field get a CLI arg;
    # the rest of the tuning (confidence, fps, hfov, standoff, search speed) is
    # env-only to keep this list readable — see the module docstring.
    parser.add_argument("--vision-model", default=os.environ.get("RS_VISION_MODEL", cfg.vision.model_path),
                        help="path to the Edge Impulse .eim model (must be chmod +x)")
    parser.add_argument("--vision-label", default=os.environ.get("RS_VISION_LABEL", cfg.vision.target_label),
                        help="object label to track ('' = any label the model reports)")
    parser.add_argument("--no-vision", dest="vision", action="store_false",
                        default=os.environ.get("RS_VISION_ENABLED", "1").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="disable object detection (object_align will hold still)")
    parser.add_argument("--camera-device", default=os.environ.get("RS_CAMERA_DEVICE", cfg.camera.device),
                        help="auto | picamera2 | /dev/videoN | index")
    parser.add_argument("--mock-detector", action="store_true",
                        default=os.environ.get("RS_MOCK_DETECTOR", "").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="run without a camera/model: synthesize a moving target")
    parser.add_argument("--fpv", dest="fpv", action="store_true",
                        default=os.environ.get("RS_FPV_ENABLED", "").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="stream first-person video to the base station over UDP (needs WiFi)")
    parser.add_argument("--fpv-host", default=os.environ.get("RS_FPV_HOST", cfg.fpv.base_host),
                        help="base-station host that receives the video feed")
    parser.add_argument("--shooter", dest="shooter", action="store_true",
                        default=os.environ.get("RS_SHOOTER_ENABLED", "").strip().lower()
                        in ("1", "true", "yes", "on"),
                        help="enable the servo launcher (required for shooter_align to fire)")
    args = parser.parse_args()

    if args.mock_motors:
        os.environ["RS_MOCK_MOTORS"] = "1"  # picked up by the motor layer
    if args.mock_detector:
        os.environ["RS_MOCK_DETECTOR"] = "1"  # picked up in Robot.__init__

    cfg.robot_id = args.robot_id
    cfg.comms.port = args.port
    cfg.comms.baud = args.baud
    cfg.start_mode = args.mode
    cfg.loop_hz = args.hz
    cfg.telemetry_hz = args.telemetry_hz
    cfg.gps.enabled = args.gps
    cfg.gps.port = args.gps_port
    cfg.gps.baud = args.gps_baud
    cfg.gps.update_rate_ms = args.gps_rate
    cfg.heading_source = args.heading_source
    cfg.imu.enabled = args.imu
    cfg.imu.i2c_address = args.imu_address
    cfg.imu.heading_offset_deg = args.imu_offset
    cfg.imu.persist_calibration = args.imu_save_calib
    cfg.vision.enabled = args.vision
    cfg.vision.model_path = args.vision_model
    cfg.vision.target_label = args.vision_label
    cfg.camera.device = args.camera_device
    # Env-only vision tuning (see the module docstring for the full list).
    cfg.camera.enabled = os.environ.get("RS_CAMERA_ENABLED", "1").strip().lower() in ("1", "true", "yes", "on")
    cfg.camera.width = int(os.environ.get("RS_CAMERA_WIDTH", cfg.camera.width))
    cfg.camera.height = int(os.environ.get("RS_CAMERA_HEIGHT", cfg.camera.height))
    cfg.camera.fps = int(os.environ.get("RS_CAMERA_FPS", cfg.camera.fps))
    cfg.vision.min_confidence = float(os.environ.get("RS_VISION_CONF", cfg.vision.min_confidence))
    cfg.vision.max_fps = float(os.environ.get("RS_VISION_FPS", cfg.vision.max_fps))
    cfg.vision.standoff_size = float(os.environ.get("RS_VISION_STANDOFF", cfg.vision.standoff_size))
    cfg.vision.hfov_deg = float(os.environ.get("RS_VISION_HFOV", cfg.vision.hfov_deg))
    cfg.vision.search_speed = float(os.environ.get("RS_VISION_SEARCH_SPEED", cfg.vision.search_speed))
    cfg.fpv.enabled = args.fpv
    cfg.fpv.base_host = args.fpv_host
    cfg.fpv.base_port = int(os.environ.get("RS_FPV_PORT", cfg.fpv.base_port))
    cfg.fpv.fps = int(os.environ.get("RS_FPV_FPS", cfg.fpv.fps))
    cfg.fpv.jpeg_quality = int(os.environ.get("RS_FPV_QUALITY", cfg.fpv.jpeg_quality))
    # Shooter. Same policy as vision: only the on/off switch gets a CLI flag,
    # the geometry and firing policy are env-only (see the module docstring).
    cfg.shooter.enabled = args.shooter
    cfg.shooter.channel = int(os.environ.get("RS_SHOOTER_CHANNEL", cfg.shooter.channel))
    cfg.shooter.rest_angle = float(os.environ.get("RS_SHOOTER_REST", cfg.shooter.rest_angle))
    cfg.shooter.fire_angle = float(os.environ.get("RS_SHOOTER_FIRE", cfg.shooter.fire_angle))
    cfg.shooter.fire_seconds = float(os.environ.get("RS_SHOOTER_FIRE_S", cfg.shooter.fire_seconds))
    cfg.shooter.retract_seconds = float(os.environ.get("RS_SHOOTER_RETRACT_S", cfg.shooter.retract_seconds))
    cfg.shooter.dwell = float(os.environ.get("RS_SHOOTER_DWELL", cfg.shooter.dwell))
    cfg.shooter.cooldown = float(os.environ.get("RS_SHOOTER_COOLDOWN", cfg.shooter.cooldown))
    cfg.shooter.max_shots = int(os.environ.get("RS_SHOOTER_MAX_SHOTS", cfg.shooter.max_shots))
    cfg.shooter.require_arm = os.environ.get("RS_SHOOTER_REQUIRE_ARM", "1").strip().lower() in ("1", "true", "yes", "on")
    cfg.shooter.require_arrived = os.environ.get("RS_SHOOTER_REQUIRE_ARRIVED", "1").strip().lower() in ("1", "true", "yes", "on")

    motors = "MOCK" if args.mock_motors else "real"
    gps = f"{cfg.gps.port}@{cfg.gps.baud}" if cfg.gps.enabled else "off"
    imu = f"0x{cfg.imu.i2c_address:02x}" if cfg.imu.enabled else "off"
    if not cfg.vision.enabled:
        vision = "off"
    elif args.mock_detector:
        vision = "MOCK"
    else:
        vision = f"{cfg.vision.model_path}"
        if cfg.vision.target_label:
            vision += f" [{cfg.vision.target_label}]"
    fpv = f"{cfg.fpv.base_host}:{cfg.fpv.base_port}" if cfg.fpv.enabled else "off"
    shooter = f"ch{cfg.shooter.channel}" if cfg.shooter.enabled else "off"
    print(f"[Robot] id={cfg.robot_id} port={cfg.comms.port} baud={cfg.comms.baud} "
          f"mode={cfg.start_mode} motors={motors} gps={gps} imu={imu} "
          f"heading={cfg.heading_source} vision={vision} "
          f"fpv={fpv} shooter={shooter}")
    Robot(cfg).run()


if __name__ == "__main__":
    main()
