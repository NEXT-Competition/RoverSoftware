"""Central configuration for the tank-drive robot.

All hardware wiring and tuning lives here so the rest of the code stays generic.
Adjust these to match your build and ESC calibration.

--- A note on the "90 degrees is center" idea ---
The SunFounder Fusion HAT `Servo.angle()` accepts roughly -90..+90, where the
MIDDLE of that range (0) is the neutral pulse (~1500 us). For a bidirectional
ESC, that neutral pulse is "stop", +90 is full forward, -90 is full reverse.

Neutral doesn't have to be 0: this rover's ESC stops at `neutral_angle = 5.0`.
The "90 = center" you may have seen refers to the other common convention where a
positional servo sweeps 0..180 with 90 in the middle. If your particular ESC is
happiest at a different neutral, just change `neutral_angle` (and run
tools/esc_calibrate.py to find the exact endpoints).

The throttle -> angle mapping uses a SYMMETRIC throw about neutral, i.e. an equal
swing on each side: throw = min(max_angle - neutral_angle, neutral_angle -
min_angle). This is deliberate — with an off-center neutral, unequal forward and
reverse spans would make the normal and inverted (mirrored) track motors start at
different throttles and drive at mismatched speeds. `max_angle`/`min_angle` are
the endpoints/clamps; whichever is closer to neutral sets the usable throw.
"""

from dataclasses import dataclass, field


@dataclass
class MotorConfig:
    channel: int  # Fusion HAT PWM/servo channel (motor1=ch0, motor2=ch1)
    inverted: bool = False  # Flip direction for the opposite-facing track motor
    neutral_angle: float = 5.0  # Angle that makes the ESC hold "stop"
    # Endpoints/clamps. The usable swing is SYMMETRIC about neutral: the side
    # closer to neutral sets the throw (see the module docstring), so with
    # neutral=5 and these endpoints the effective range is -10..+20 (+/-15).
    max_angle: float = 20.0  # Full-forward endpoint (upper clamp)
    min_angle: float = -20.0  # Full-reverse endpoint (lower clamp)
    deadband: float = 0.03  # |throttle| below this => treat as neutral
    max_forward: float = 1.0  # Safety cap on forward throttle, [0..1]
    max_reverse: float = 1.0  # Safety cap on reverse throttle, [0..1]


@dataclass
class DriveConfig:
    # motor1 -> channel 0 (left), motor2 -> channel 1 (right, mounted mirrored)
    left: MotorConfig = field(
        default_factory=lambda: MotorConfig(channel=0, inverted=False)
    )
    right: MotorConfig = field(
        default_factory=lambda: MotorConfig(channel=1, inverted=True)
    )
    arm_seconds: float = 2.0  # Hold neutral this long so the ESCs arm on boot
    slew_rate: float = 4.0  # Max throttle change per second (0 disables limiting)


@dataclass
class CommsConfig:
    port: str = "/dev/ttyUSB0"  # XBee serial port (USB adapter; use /dev/serial0 for the GPIO header)
    baud: int = 57600
    command_timeout: float = (
        0.5  # Failsafe: stop if no drive command arrives within this many seconds
    )


@dataclass
class GPSConfig:
    # Adafruit Ultimate GPS (MTK3339/PA1616D — breakout, FeatherWing or HAT) on
    # the Pi GPIO UART, read with the adafruit_gps library. Ships as NMEA @ 9600.
    # Kept separate from the XBee port: XBee is the USB adapter (/dev/ttyUSB0),
    # GPS is the PL011 GPIO header UART (/dev/ttyAMA0). Requires the Pi serial
    # console disabled and the UART enabled (raspi-config).
    #
    # The module reports a TRACK ANGLE (course over ground) — a true-North
    # heading needing no compass, no calibration and no declination correction.
    # It's only valid while moving, though, which is why the IMU is still the
    # preferred heading source at a standstill; see RobotConfig.heading_source.
    enabled: bool = True
    port: str = "/dev/ttyAMA0"
    baud: int = 9600
    fix_timeout: float = 5.0  # drop the fix (return None) after this long w/o an update
    min_move_mps: float = (
        0.5  # below this speed, the track angle is noise; hold last heading
    )
    # Fix interval in ms (PMTK220). 1000 = 1 Hz, the module's default. Lower is a
    # fresher heading, but the sentences have to fit the link: below ~200 ms they
    # won't at 9600 baud, and truncated sentences read as "no fix". Ignored by a
    # non-MTK receiver (a NEO-6M keeps whatever rate it was configured for).
    update_rate_ms: int = 1000


@dataclass
class IMUConfig:
    # CEVA/Bosch BNO085 (BNO08x) 9-DOF IMU on the Pi I2C bus (shared with the
    # Fusion HAT; the bootstrap already enables I2C). Its on-chip fusion gives an
    # ABSOLUTE heading that's valid at a standstill — which the GPS track angle
    # is not — so by default it's the heading source for pose estimation and the
    # waypoint angle loop. Set RobotConfig.heading_source="gps" (or disable this
    # sensor) to navigate on the GPS track angle alone.
    #
    # Wiring note: unlike the BNO055 it replaces, the BNO08x speaks SHTP and does
    # NOT abuse I2C clock stretching, so it runs at the Pi's normal bus speed — no
    # dtparam=i2c_arm_baudrate workaround needed (remove it if it was set for the
    # old sensor). Strap PS0/PS1 for I2C mode. Verify with tools/imu_selftest.py.
    # See packaging/robot.env.
    enabled: bool = True
    i2c_address: int = 0x4A  # BNO085 default; 0x4B if the DI/AD0 pin is pulled high
    # Rotation applied to the sensor's raw yaw to align it with the robot's
    # forward axis and true North (0 = North, CW positive). Tune during bring-up.
    heading_offset_deg: float = 0.0
    invert: bool = False  # flip yaw sign to CW-positive if the board is mounted mirrored
    # Minimum fused-orientation calibration level (0-3) before we trust the
    # heading. Below this, heading() returns None and the fusion falls back to GPS.
    min_calib: int = 1
    # Run the sensor's dynamic calibration and save it to the BNO08x's own flash
    # once it converges, so the board boots calibrated. The chip persists this
    # itself — there is no offsets file to manage (the BNO055 needed one because it
    # forgot its calibration on every power cycle). False disables auto-save.
    persist_calibration: bool = True


@dataclass
class CameraConfig:
    # Frame source for the vision stack. Backend is auto-detected: the Pi Camera
    # (CSI ribbon) via picamera2, else a USB/V4L2 device via OpenCV, else nothing
    # (vision disables itself and the rest of the robot runs unchanged).
    #   AI Camera:  sudo apt install python3-picamera2 imx500-all
    #   Pi Camera:  sudo apt install python3-picamera2
    #   USB webcam: pip install opencv-python
    # Kept separate from VisionConfig because the camera is a *device* — a future
    # live-video feed would want the same frames without any model involved.
    #
    # "imx500" is never chosen by "auto": loading the sensor's network costs tens
    # of seconds on a cold boot, so it happens only when something actually wants
    # on-sensor inference. VisionConfig.backend does that for you — it rewrites
    # this field when it resolves to the IMX500 (see sensors/imx500.py).
    enabled: bool = True
    device: str = "auto"  # auto | imx500 | picamera2 | /dev/videoN | a numeric index
    width: int = 640
    height: int = 480
    fps: int = 15


@dataclass
class VisionConfig:
    # Object detection -> the object_align autonomy mode. Two interchangeable
    # backends; object_align cannot tell them apart.
    #
    #   edge_impulse — a compiled `.eim` binary run on the Pi's CPU. Costs a
    #                  core, 50-200ms/frame, works with any camera.
    #   imx500       — the Raspberry Pi AI Camera (Sony IMX500) runs the network
    #                  INSIDE the sensor and ships boxes out as frame metadata.
    #                  Near-zero Pi cost, but needs that specific camera.
    #
    # `backend` picks one. "auto" = imx500 if an AI Camera is attached and its
    # network file exists, else edge_impulse. Resolving it also points
    # CameraConfig.device at the right capture backend — see
    # sensors/imx500.resolve_backend().
    backend: str = "auto"  # auto | edge_impulse | imx500

    # --- edge_impulse backend ---
    # The model is a compiled `.eim` binary the Edge Impulse runtime EXECUTES, so
    # it must be chmod +x. Download one with:
    #   edge-impulse-linux-runner --download model.eim
    #
    # IMPORTANT — model architecture decides what object_align can do. FOMO models
    # emit centroids with fixed cell-sized boxes, NOT true bounding boxes, so
    # object size is unavailable and standoff/approach is impossible; the mode
    # degrades to align-only (turn to face, never advance). Export a YOLO-style
    # model (model_type == 'object_detection') if you want approach + standoff.
    # (This caveat is Edge Impulse's alone — the IMX500 model zoo is all real
    # bounding-box detectors, so approach always works there.)
    enabled: bool = True
    # Lives alongside the BNO055 calibration in the pi-owned dir postinst creates.
    # Keep .eim binaries out of git; ship them with the .deb or scp them over.
    model_path: str = "/var/lib/roversoftware/model.eim"

    # --- imx500 backend ---
    # The `.rpk` network that gets uploaded into the sensor. `sudo apt install
    # imx500-all` drops the Sony model zoo in /usr/share/imx500-models/; this
    # default is its general-purpose COCO detector. Unlike the .eim, this is data
    # the sensor loads, NOT a binary the Pi runs — no chmod +x needed.
    imx500_model: str = ("/usr/share/imx500-models/"
                         "imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
    # Class-name file, one per line. Empty = use the labels embedded in the .rpk,
    # which is right for every model-zoo network; only a custom export packaged
    # without them needs this.
    imx500_labels: str = ""
    # NMS overlap threshold and cap on boxes per frame. Only the nanodet
    # postprocess path uses the IoU (the `_pp` networks do NMS on-sensor); the
    # cap applies to both.
    imx500_iou: float = 0.65
    imx500_max_detections: int = 10

    # --- shared by both backends ---
    target_label: str = ""  # "" = track any label the model reports
    min_confidence: float = 0.6  # ignore boxes below this score
    max_fps: float = 10.0  # cap inference rate; it costs a core
    # Drop the target (provider returns None) after this long without a fresh
    # detection — the GPS fix_timeout idea. Also what makes a dead detector
    # thread fail safe: no new stamps -> target ages out -> the robot stops.
    target_timeout: float = 0.5
    select: str = "largest"  # largest | confidence | centermost
    # Horizontal FOV the normalized error units span. This scales the IMU
    # yaw-rate into those units; too high and the D term is too small, too low
    # and the steering oscillates.
    #
    # *** Its correct value DEPENDS ON THE BACKEND. ***
    #   edge_impulse — the EFFECTIVE FOV *after* EI's center-crop, not the
    #                  camera's spec: get_features_from_image() resizes then
    #                  center-crops to the model input, discarding ~25% of the
    #                  width at 640x480 -> square. Hence this 50 deg default.
    #   imx500       — the camera's REAL horizontal FOV (~66 deg for the AI
    #                  Camera's stock lens). Boxes are mapped back to the full
    #                  frame, so nothing is cropped away. Set RS_VISION_HFOV=66.
    hfov_deg: float = 50.0
    # Stop once the box height reaches this fraction of the frame height (of the
    # model input height, on the edge_impulse backend). Calibrate it, don't
    # guess: park at the distance you want, run tools/detector_selftest.py, and
    # read off the printed size.
    standoff_size: float = 0.45
    search_speed: float = 0.25  # slow rotate to reacquire a lost target; 0 disables


@dataclass
class FPVConfig:
    # First-person live video streamed to the base station over UDP (see
    # robot/comms/video_udp.py). Off by default: it needs the base station's IP
    # and a shared WiFi/LAN (the XBee radio can't carry video), so it's opt-in
    # rather than something every robot fires into the void on boot.
    enabled: bool = False
    base_host: str = "base-station.local"  # where the base station receives UDP
    base_port: int = 5005
    fps: int = 15           # cap on frames sent per second
    jpeg_quality: int = 60  # 1-100; lower = smaller packets, less bandwidth


@dataclass
class ShooterConfig:
    # Servo-actuated launcher on its own PWM channel -> the shooter_align mode.
    #
    # Off by default: a stock chassis has no launcher, and enabling it would
    # drive an unused channel at boot. Set RS_SHOOTER_ENABLED=1 on builds that
    # have one.
    enabled: bool = False
    # Channels 0 and 1 are the drive ESCs (see DriveConfig), so a shooter starts
    # at 2. Changing this to 0 or 1 would fight the drivetrain for a channel.
    channel: int = 2
    rest_angle: float = -30.0  # home position; also where a disarm/e-stop parks it
    fire_angle: float = 30.0  # position that trips the mechanism
    # How long to HOLD the fire angle. Too short and the servo never reaches it;
    # too long and it stalls against the mechanical stop. Find it with
    # tools/servo_sweep.py, then add a little margin.
    fire_seconds: float = 0.35
    retract_seconds: float = 0.35  # settle at rest before another shot may start

    # --- Firing policy (consumed by ShooterAlignController, not the servo) ---
    # Hold the alignment this long before firing. This is the single most
    # important safety/accuracy knob: the detector is noisy and a single centered
    # frame is not evidence the robot is actually pointed at anything.
    dwell: float = 0.5
    cooldown: float = 2.0  # minimum seconds between shots
    # Require an explicit {"type":"arm_shooter"} before any shot. Leave True.
    # Arming is dropped on mode exit and on e-stop, so it can never be latched on
    # from a previous run.
    require_arm: bool = True
    # Also require the standoff distance to be reached, not just the bearing.
    # Automatically skipped when the model can't measure size (FOMO), since
    # arrival can never latch there — see VisionConfig.
    require_arrived: bool = True
    max_shots: int = 0  # magazine capacity; 0 = unlimited


@dataclass
class RobotConfig:
    drive: DriveConfig = field(default_factory=DriveConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    gps: GPSConfig = field(default_factory=GPSConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    fpv: FPVConfig = field(default_factory=FPVConfig)
    shooter: ShooterConfig = field(default_factory=ShooterConfig)
    # Control loop rate. This is the rate the motors are actually updated at, so
    # it sets both the floor on teleop latency (a command waits up to 1/loop_hz
    # before anything looks at it) and the granularity of the slew limiter, which
    # is what interpolates between the base station's ~15 Hz drive frames. At
    # 5 Hz the outputs moved in 200 ms stair-steps and slew_rate=4.0 allowed a
    # 0.8 jump per tick, i.e. no real limiting: choppy AND laggy. At 50 Hz the
    # step is 0.08 and motion is continuous. docs/ARCHITECTURE.md has always
    # specified 50 Hz.
    loop_hz: float = 50.0
    start_mode: str = "teleop"  # teleop | object_align | waypoint | shooter_align
    # Which sensor answers "which way am I facing" (see sensors/pose.py):
    #   auto - IMU when calibrated, else the GPS track angle (recommended)
    #   gps  - the GPS track angle only; no IMU needed for heading
    #   imu  - the IMU only; no fallback to course over ground
    heading_source: str = "auto"
    robot_id: str = "rover1"  # unique id on the shared XBee channel
    telemetry_hz: float = (
        5.0  # rate of status frames back to the base station (0 disables)
    )
