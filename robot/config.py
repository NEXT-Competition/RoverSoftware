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
    # u-blox NEO-6M (GY-GPS6MV2) on the Pi GPIO UART. Ships as NMEA @ 9600.
    # Kept separate from the XBee port: XBee is the USB adapter (/dev/ttyUSB0),
    # GPS is the PL011 GPIO header UART (/dev/ttyAMA0). Requires the Pi serial
    # console disabled and the UART enabled (raspi-config).
    enabled: bool = True
    port: str = "/dev/ttyAMA0"
    baud: int = 9600
    fix_timeout: float = 5.0  # drop the fix (return None) after this long w/o an update
    min_move_mps: float = (
        0.5  # below this speed, GPS course is noise; hold last heading
    )


@dataclass
class IMUConfig:
    # Bosch BNO055 9-DOF IMU on the Pi I2C bus (shared with the Fusion HAT; the
    # bootstrap already enables I2C). Its on-chip NDOF fusion gives an ABSOLUTE
    # heading that's valid at a standstill — the compass the NEO-6M lacks — so it
    # becomes the heading source for pose estimation and the waypoint angle loop.
    #
    # Wiring note: the BNO055 clock-stretches on the Pi's hardware I2C, which
    # corrupts reads. 100 kHz (the Pi default) is NOT slow enough — set
    # dtparam=i2c_arm_baudrate=10000 in /boot/firmware/config.txt and reboot, or
    # use a software I2C bus (dtoverlay=i2c-gpio) to keep full speed. Verify with
    # tools/imu_selftest.py. See packaging/robot.env.
    enabled: bool = True
    i2c_address: int = 0x28  # BNO055 default; 0x29 if the ADR pin is pulled high
    # Rotation applied to the sensor's raw yaw to align it with the robot's
    # forward axis and true North (0 = North, CW positive). Tune during bring-up.
    heading_offset_deg: float = 0.0
    invert: bool = False  # flip yaw sign to CW-positive if the board is mounted mirrored
    # Minimum system/magnetometer calibration level (0-3) before we trust the
    # heading. Below this, heading() returns None and the fusion falls back to GPS.
    min_calib: int = 1


@dataclass
class RobotConfig:
    drive: DriveConfig = field(default_factory=DriveConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    gps: GPSConfig = field(default_factory=GPSConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    loop_hz: float = 5.0  # Control loop rate
    start_mode: str = "teleop"  # teleop | color_align | waypoint
    robot_id: str = "rover1"  # unique id on the shared XBee channel
    telemetry_hz: float = (
        5.0  # rate of status frames back to the base station (0 disables)
    )
