#!/usr/bin/env python3
"""Entry point for the tank-drive robot (run this on the Raspberry Pi).

    python run_robot.py --port /dev/serial0 --baud 9600 --id rover1

Defaults are read from the environment first (so the systemd service can be
configured via /etc/roversoftware/robot.env), then overridden by CLI flags:

    RS_ROBOT_ID, RS_XBEE_PORT, RS_XBEE_BAUD, RS_BASE_HOST, RS_BASE_PORT,
    RS_START_MODE, RS_LOOP_HZ,
    RS_TELEMETRY_HZ, RS_GPS_ENABLED/PORT/BAUD/RATE_MS, RS_HEADING_SOURCE,
    RS_IMU_ENABLED/ADDRESS/OFFSET/SAVE_CALIB,
    RS_CAMERA_ENABLED/DEVICE/WIDTH/HEIGHT/FPS,
    RS_VISION_ENABLED/BACKEND/MODEL/LABEL/CONF/FPS/STANDOFF/HFOV/SEARCH_SPEED,
    RS_VISION_IMX500_MODEL/LABELS/IOU/MAX_DET,
    RS_FPV_ENABLED/HOST/PORT/FPS/QUALITY,
    RS_SHOOTER_ENABLED/CHANNEL/REST/FIRE/FIRE_S/RETRACT_S/DWELL/COOLDOWN/
        MAX_SHOTS/REQUIRE_ARM/REQUIRE_ARRIVED,
    RS_ROUTINE_ALLOW_ARM/STATE_TIMEOUT

Two files are read after all of that, in this order:

    RS_LAYOUT_FILE    what actuators this build has, and what they belong to
                      (robot/layout.py). Structural, so it decides which tuning
                      paths even exist — `drive.<name>.deadband` is only real
                      once <name> is. No file leaves the compiled-in two-motor
                      tank drive exactly as it was.
    RS_TUNING_FILE    values tuned from the base station's settings page, saved
                      on the robot and applied LAST (robot/tuning.py). They are
                      the operator's most recent deliberate decision, and
                      nothing they can reach has a CLI flag or env var here.

State machines authored in the dashboard live in RS_ROUTINES_FILE and are loaded
by Robot itself — they are data the engine reads, not wiring, so unlike a layout
they take effect the moment they arrive.

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

from robot import layout, tuning
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
    # Bulk transfers over WiFi (config snapshots, layouts, routines). This is
    # the ONLY path they take — blank or unreachable means the robot can't be
    # configured until it's on WiFi, while driving and telemetry carry on over
    # the radio regardless. The base station can set this over the radio and it
    # applies live; see robot/comms/ip_link.py.
    parser.add_argument("--base-host",
                        default=os.environ.get("RS_BASE_HOST", cfg.comms.base_host),
                        help="base station host for WiFi bulk transfers "
                             "(blank = config/layouts/routines don't transfer)")
    parser.add_argument("--base-port", type=int,
                        default=int(os.environ.get("RS_BASE_PORT", cfg.comms.base_port)),
                        help="base station TCP port for WiFi bulk transfers")
    parser.add_argument("--mode", default=os.environ.get("RS_START_MODE", cfg.start_mode),
                        choices=["teleop", "object_align", "shooter_align",
                                 "waypoint", "routine"])
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
    parser.add_argument("--vision-backend", default=os.environ.get("RS_VISION_BACKEND", cfg.vision.backend),
                        choices=["auto", "edge_impulse", "imx500"],
                        help="detection backend: on-CPU Edge Impulse, or on-sensor "
                             "IMX500 (Raspberry Pi AI Camera). auto = imx500 if one is attached")
    parser.add_argument("--vision-model", default=os.environ.get("RS_VISION_MODEL", cfg.vision.model_path),
                        help="path to the Edge Impulse .eim model (must be chmod +x)")
    parser.add_argument("--imx500-model", default=os.environ.get("RS_VISION_IMX500_MODEL", cfg.vision.imx500_model),
                        help="path to the IMX500 .rpk network uploaded to the sensor")
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
    cfg.comms.base_host = args.base_host
    cfg.comms.base_port = args.base_port
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
    cfg.vision.backend = args.vision_backend
    cfg.vision.model_path = args.vision_model
    cfg.vision.imx500_model = args.imx500_model
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
    cfg.vision.imx500_labels = os.environ.get("RS_VISION_IMX500_LABELS", cfg.vision.imx500_labels)
    cfg.vision.imx500_iou = float(os.environ.get("RS_VISION_IMX500_IOU", cfg.vision.imx500_iou))
    cfg.vision.imx500_max_detections = int(
        os.environ.get("RS_VISION_IMX500_MAX_DET", cfg.vision.imx500_max_detections))
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
    cfg.shooter.target_rpm = float(os.environ.get("RS_SHOOTER_TARGET_RPM", cfg.shooter.target_rpm))
    cfg.shooter.dwell = float(os.environ.get("RS_SHOOTER_DWELL", cfg.shooter.dwell))
    cfg.shooter.cooldown = float(os.environ.get("RS_SHOOTER_COOLDOWN", cfg.shooter.cooldown))
    cfg.shooter.max_shots = int(os.environ.get("RS_SHOOTER_MAX_SHOTS", cfg.shooter.max_shots))
    cfg.shooter.require_arm = os.environ.get("RS_SHOOTER_REQUIRE_ARM", "1").strip().lower() in ("1", "true", "yes", "on")
    cfg.shooter.require_arrived = os.environ.get("RS_SHOOTER_REQUIRE_ARRIVED", "1").strip().lower() in ("1", "true", "yes", "on")
    # Routine policy. Deliberately env-only and deliberately off by default:
    # allow_arm is the one thing a UI-authored program can do that makes
    # something physically launch, so turning it on should be a decision someone
    # made in a config file rather than a checkbox they clicked past.
    cfg.routines.allow_arm = os.environ.get("RS_ROUTINE_ALLOW_ARM", "").strip().lower() in ("1", "true", "yes", "on")
    cfg.routines.state_timeout_default = float(
        os.environ.get("RS_ROUTINE_STATE_TIMEOUT", cfg.routines.state_timeout_default))

    # The hardware layout, if this build has been given one from the dashboard.
    # Applied BEFORE tuning because it decides which tuning paths even exist: a
    # layout names the actuators, and `drive.<name>.deadband` is only a real
    # parameter once <name> is a real actuator. No layout file leaves the
    # compiled-in two-motor tank drive exactly as it was.
    doc = layout.load()
    if doc is not None:
        result = layout.apply(cfg, doc)
        for warning in result.warnings:
            print(f"[Robot] layout: {warning}")
        if result.ok:
            print(f"[Robot] layout: {cfg.drive.kind} drive, "
                  f"{len(cfg.drive.actuators)} drive actuator(s), "
                  f"{len(cfg.mechanisms)} mechanism(s) from {layout.layout_path()}")
        else:
            # Keep driving on the compiled-in layout rather than refusing to
            # boot. A rover that won't start is worse than one on last-known-good
            # wiring, and the errors go out with the next get_layout.
            for error in result.errors:
                print(f"[Robot] layout REJECTED: {error}")
            print("[Robot] layout: falling back to the built-in tank layout")

    # Values tuned from the base station's settings page, saved on this robot.
    # Applied LAST, on purpose: they are the operator's most recent deliberate
    # decision, and the paths they can touch (gains, speeds, limits) are
    # disjoint from the wiring flags above — no CLI flag names a PID gain.
    # Filtered against THIS robot's parameter surface, so a value saved for an
    # actuator the layout no longer has is dropped rather than reported.
    overrides = tuning.load_overrides(known=tuning.by_path_for(cfg))
    if overrides:
        applied, rejected = tuning.apply(cfg, overrides)
        print(f"[Robot] tuning: {len(applied)} saved values from {tuning.overrides_path()}")
        for path, why in rejected.items():
            print(f"[Robot] tuning: ignoring {path}: {why}")

    motors = "MOCK" if args.mock_motors else "real"
    gps = f"{cfg.gps.port}@{cfg.gps.baud}" if cfg.gps.enabled else "off"
    imu = f"0x{cfg.imu.i2c_address:02x}" if cfg.imu.enabled else "off"
    if not cfg.vision.enabled:
        vision = "off"
    elif args.mock_detector:
        vision = "MOCK"
    else:
        # The requested backend; Robot logs the one "auto" actually resolves to.
        model = (cfg.vision.imx500_model if cfg.vision.backend == "imx500"
                 else cfg.vision.model_path)
        vision = f"{cfg.vision.backend}:{os.path.basename(model)}"
        if cfg.vision.target_label:
            vision += f" [{cfg.vision.target_label}]"
    fpv = f"{cfg.fpv.base_host}:{cfg.fpv.base_port}" if cfg.fpv.enabled else "off"
    shooter = f"ch{cfg.shooter.channel}" if cfg.shooter.enabled else "off"
    bulk = (f"{cfg.comms.base_host}:{cfg.comms.base_port}"
            if cfg.comms.base_host else "off (not configurable until set)")
    print(f"[Robot] id={cfg.robot_id} port={cfg.comms.port} baud={cfg.comms.baud} "
          f"mode={cfg.start_mode} motors={motors} gps={gps} imu={imu} "
          f"heading={cfg.heading_source} vision={vision} "
          f"fpv={fpv} shooter={shooter} bulk={bulk}")
    Robot(cfg).run()


if __name__ == "__main__":
    main()
