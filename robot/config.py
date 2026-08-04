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
from typing import Any, Dict, List


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

    # --- identity, for layouts with more than the two stock track motors ---
    # Trailing and defaulted on purpose: every existing MotorConfig(channel=N,
    # inverted=X) construction in the codebase, the tests and the tools keeps
    # working untouched, and a hand-written tuning.json keeps resolving.
    #
    # `name` is how a layout, a mechanism and a tuning path all refer to this
    # actuator ("left", "intake_roller", "hood"). Unique across the robot; the
    # layout loader fills it in when a layout is applied.
    name: str = ""
    # esc   - bidirectional ESC. Throttle maps onto a SYMMETRIC throw about
    #         neutral (see the module docstring), and it is held at neutral on
    #         boot so it arms.
    # servo - positional servo (steering, a hood, a launcher arm). Same mapping,
    #         but there is nothing to arm, so it is simply parked at neutral.
    kind: str = "esc"
    label: str = ""  # what the dashboard calls it; "" => derived from `name`

    # --- quadrature encoder, for closed-loop wheel speed (sensors/encoder.py) --
    # BCM GPIO pin numbers for the encoder's two channels — Pi header pins, NOT
    # Fusion HAT PWM channels; `channel` above is a different bus entirely. -1
    # on either means this actuator has no encoder, which is the default and
    # leaves the drivetrain exactly as open-loop as it has always been.
    encoder_a: int = -1
    encoder_b: int = -1
    # Counts per revolution OF THE WHEEL, as an X4 decoder counts them: the
    # disc's cycles per rev x 4 x the gear ratio. Measure it rather than
    # computing it — tools/encoder_monitor.py, zero, turn the wheel once, read.
    encoder_cpr: float = 0.0
    # Flip the counting direction so "forward throttle" reads as a POSITIVE rpm.
    # Independent of `inverted`: that mirrors the motor, this mirrors the sensor,
    # and on a mirrored track motor you usually need both.
    encoder_invert: bool = False


@dataclass
class PIDConfig:
    """Gains for one PID loop (robot/control/pid.py).

    Split out of the controllers so the loops are tunable from the base station
    instead of being edit-and-redeploy constants. The defaults below ARE the
    values the controllers used to hardcode, so behaviour is unchanged until
    someone turns a knob.

    Declared up here, above the drivetrain, only because `TrimConfig` needs it
    and a dataclass annotation is resolved when the class is created.
    """

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    out_limit: float = 1.0  # clamp on the loop's output
    i_limit: float = 1.0  # clamp on the accumulated integral (anti-windup)


@dataclass
class TrimConfig:
    """Closed-loop wheel speed: keep the tracks turning at the same rate.

    A throttle is an open-loop wish. Two identical motors handed the same pulse
    turn at different speeds — ESCs differ, gearboxes differ, one side carries
    the battery, one track is on grass — and the rover curves away while every
    number in the system insists it is going straight. With encoders fitted
    (`MotorConfig.encoder_a/b`), this is the loop that closes that gap.
    See robot/control/rpm_trim.py.

    OFF by default and that is not timidity: it is the one feature here that can
    ADD throttle the operator did not ask for, and it must be a decision someone
    made after wiring an encoder and watching the RPM readout, not a default
    that surprises a build which has no encoders at all.
    """

    # off      - measure only. RPM still reaches telemetry; nothing is corrected.
    # match    - hold the two sides to EACH OTHER. Needs no calibration at all,
    #            and only engages while driving straight (see straight_tolerance)
    #            because a commanded turn is a difference you asked for.
    # velocity - hold each side to `throttle x max_rpm`. Works while turning too,
    #            but it is only as truthful as `max_rpm`, which you must measure.
    mode: str = "off"
    # Wheel RPM at full throttle, for `velocity`. MEASURE IT: drive flat out on
    # the ground it will run on and read the RPM in telemetry. A value that is
    # too high makes every setpoint unreachable and the loop saturates; too low
    # and it throttles back against a wall that isn't there.
    max_rpm: float = 200.0
    # `match` only. Above this much difference between the two commanded sides,
    # the robot is being asked to turn, so the loop stops correcting and holds
    # its integral rather than fighting the steering. Slew-limited commands mean
    # this wants a little room — a straight line is not exactly 0.000.
    straight_tolerance: float = 0.05
    # Below this commanded magnitude nothing is trimmed and the integrators are
    # released. A stopped robot has no speed to match, and a loop that keeps
    # integrating against a wheel held by the ground is a loop that lurches the
    # moment the throttle comes back.
    min_throttle: float = 0.05
    # Fail-safe. Commanded above min_throttle for this long with the encoder
    # still reading a standstill means the wheel is stalled or the encoder is
    # unplugged — and a speed loop chasing a dead sensor winds that side to full
    # throttle. Trip it and the drivetrain reverts to open-loop until it stops.
    # 0 disables the check, which is only sensible on the bench.
    stall_seconds: float = 1.0
    # Speed is counts over an interval. A longer interval is finer resolution
    # and more lag; see sensors/encoder.py.
    rpm_window: float = 0.1
    rpm_tau: float = 0.05  # extra smoothing, seconds; 0 = none
    # The error this loop sees is in RPM, so useful gains are SMALL — the same
    # reason nav.heading_pid's are (its error is in degrees). kp=0.002 answers
    # 100 rpm of mismatch with 0.2 of throttle.
    #
    # The INTEGRAL is the term that matters here, and that is unusual enough to
    # say out loud: a pair of mismatched motors is a constant bias, which is
    # precisely what an integrator exists to cancel and what a proportional term
    # can only ever half-fix. So ki carries the correction and kp is small
    # damping around it. kd is 0 because its input would be a differenced RPM,
    # and a differenced noisy measurement is noise with a gain on it.
    #
    # These settle the simulated 6% mismatch in about three seconds and sit an
    # order of magnitude below where that plant starts to hunt — headroom left
    # deliberately, because a real drivetrain has more dead time than a
    # simulated one (a measurement window, a filter, and the ESC's own lag).
    #
    # out_limit is the whole authority this loop has over the drivetrain, and
    # 0.2 is deliberately modest: the trim is a correction, not a second
    # throttle. i_limit is in RPM-seconds and is sized just past out_limit/ki,
    # so the integrator can reach full authority and cannot wind far beyond it.
    pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(kp=0.002, ki=0.008, kd=0.0,
                                          out_limit=0.2, i_limit=50.0)
    )


def _default_drive_actuators() -> "Dict[str, MotorConfig]":
    # motor1 -> channel 0 (left), motor2 -> channel 1 (right, mounted mirrored)
    return {
        "left": MotorConfig(channel=0, inverted=False, name="left", label="Left"),
        "right": MotorConfig(channel=1, inverted=True, name="right", label="Right"),
    }


@dataclass
class DriveRoles:
    """Which actuators do what, for the drivetrain kind in use.

    A role is a LIST because a side can have more than one motor — a six-wheel
    tank drives three motors per side off the same track speed.
    """

    left: List[str] = field(default_factory=lambda: ["left"])  # tank
    right: List[str] = field(default_factory=lambda: ["right"])  # tank
    throttle: List[str] = field(default_factory=list)  # servo_steer | single
    steer: str = ""  # servo_steer


@dataclass
class DriveConfig:
    """The drivetrain: a named set of actuators plus who plays which role.

    This used to be exactly two fields, `left` and `right`. It is now a store
    of named actuators so a build can have one drive motor and a steering
    servo, or three motors a side — but `drive.left` and `drive.right` still
    resolve, because the default layout names its two actuators `left` and
    `right` and `__getattr__` below looks names up in the store. That is what
    keeps every deployed tuning.json, every RS_* override and the whole
    existing test suite working unchanged.
    """

    # tank        - left/right track speeds, any number of motors per side
    # servo_steer - one or more drive motors plus a steering servo
    # single      - drive motors only; steering is ignored
    # none        - no drivetrain at all (a build that is only mechanisms)
    kind: str = "tank"
    actuators: Dict[str, MotorConfig] = field(default_factory=_default_drive_actuators)
    roles: DriveRoles = field(default_factory=DriveRoles)
    arm_seconds: float = 2.0  # Hold neutral this long so the ESCs arm on boot
    # Max throttle change per second while the command is moving AWAY from zero,
    # i.e. accelerating. 0 disables rate limiting entirely, in both directions.
    slew_rate: float = 4.0
    # The same, for a command moving TOWARD zero — braking. 0 means "use
    # slew_rate", which is symmetric limiting and exactly what every layout
    # written before this field existed already does.
    #
    # It is separate because one rate forces a bad trade. Slow enough to make
    # pulling away gentle is also slow enough to make stopping sluggish, and of
    # the two, the one you want crisp is stopping: releasing the stick should
    # take the throttle off promptly. So the useful shape is a soft accelerator
    # and a firm brake, which a single number cannot express.
    #
    # Note this is NOT what makes the e-stop immediate — that bypasses the
    # limiter entirely (see Robot._apply_estop). This is about feel.
    decel_rate: float = 0.0
    # servo_steer: how much of the steering servo's throw a full-scale steer
    # command uses. 1.0 = the whole throw.
    steer_gain: float = 1.0
    # servo_steer: a steered chassis cannot pivot in place, but the autonomy
    # controllers ask it to — they express "point, then go" as arcade(0, steer).
    # Below this much throttle with steering commanded, creep forward at this
    # value so the steering has authority instead of the robot sitting still
    # with its wheels turned. 0 disables (and object_align will then stall).
    min_pivot_throttle: float = 0.15
    # Closed-loop wheel speed, off unless this build has encoders wired and
    # somebody switched it on. A real field of DriveConfig rather than an
    # actuator name, so `__getattr__` below never sees it — and so the layout
    # validator's reserved-name check refuses an actuator called "trim".
    trim: TrimConfig = field(default_factory=TrimConfig)

    def __getattr__(self, name: str) -> MotorConfig:
        """Resolve an unknown attribute to the actuator of that name.

        LOAD-BEARING, not a convenience. `tuning._resolve` walks a dotted path
        with plain getattr/setattr, so this is what makes `drive.left.deadband`
        keep reading AND writing through to the real MotorConfig now that
        `left` is a dict key rather than a field. Remove it and every rover's
        saved tuning.json silently stops applying.

        Only called when normal lookup misses, so real fields always win — and
        `actuators` is read out of __dict__ so a lookup before __init__ has
        finished (deepcopy, pickle) raises AttributeError instead of recursing.
        """
        actuators = self.__dict__.get("actuators")
        if actuators is not None and name in actuators:
            return actuators[name]
        raise AttributeError(
            f"{type(self).__name__!s} has no attribute or actuator {name!r}"
        )


@dataclass
class CommsConfig:
    port: str = "/dev/ttyUSB0"  # XBee serial port (USB adapter; use /dev/serial0 for the GPIO header)
    baud: int = 115200
    command_timeout: float = (
        0.5  # Failsafe: stop if no drive command arrives within this many seconds
    )
    # Bulk transfers (config snapshots, layouts, routine documents) over WiFi
    # instead of the radio — see robot/comms/ip_link.py. The robot dials OUT to
    # the base station, same as the FPV video path, so this is the base's
    # hostname or IP. Empty (the default) disables it and everything stays on
    # the radio exactly as before; when it's set but unreachable, every transfer
    # still falls back to the radio, so this is safe to leave configured.
    # Realtime traffic (drive, telemetry, mode, e-stop) NEVER moves here: the
    # radio is what has the range.
    base_host: str = ""
    base_port: int = 5006


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
    # NOT the module's factory 9600: 5 Hz of GGA+RMC+VTG is ~9500 bps of payload,
    # which a 9600-baud line cannot hold. The driver sends PMTK251 on every start
    # to move the module here, because that setting doesn't survive a power cycle
    # without the breakout's CR1220 backup battery fitted.
    baud: int = 57600
    fix_timeout: float = 5.0  # drop the fix (return None) after this long w/o an update
    min_move_mps: float = (
        0.5  # below this speed, the track angle is noise; hold last heading
    )
    # Fix interval in ms, sent as both PMTK300 (solve rate) and PMTK220 (output
    # rate). 200 = 5 Hz, which is the ceiling on the Ultimate GPS's MTK3339 — ask
    # for less and the sentences just repeat positions. A PA1616D (MT3333) does a
    # genuine 10 Hz at 100. Ignored by a non-MTK receiver (a NEO-6M keeps whatever
    # rate it was configured for). Raise `baud` with it; see gps.py.
    update_rate_ms: int = 200


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
    # How the chip is read, which is a WIRING decision the board is strapped for
    # (PS0/PS1) — it cannot be changed without moving wires.
    #
    #   i2c       SHTP on the I2C bus: everything the chip can say, including a
    #             calibration accuracy level, a measured gyro, and commands back
    #             to it. No per-packet checksum, so a corrupted byte arrives as
    #             an unrecognised report — or, worse, a plausible heading.
    #   uart_rvc  19-byte checksummed frames at 100 Hz, output only, on one
    #             wire (the sensor's TX into a Pi RX). Corruption is DETECTED
    #             and the frame dropped, which is the whole argument on a noisy
    #             chassis. The price is real: no calibration level (so
    #             `min_calib` cannot be enforced), no measured gyro (yaw rate is
    #             differentiated from the yaw) and no way to save calibration.
    #             Calibrate over I2C once first — the chip keeps it in flash.
    #             See sensors/bno085_rvc.py, including the two-minute check for
    #             whether this mode's yaw is a compass heading or a drifting one.
    mode: str = "uart_rvc"
    i2c_address: int = 0x4A  # BNO085 default; 0x4B if the DI/AD0 pin is pulled high
    # uart_rvc only. NOT /dev/ttyAMA0 — the GPS has that one — so this needs a
    # spare UART enabled in config.txt, or a USB-TTL adapter. Check
    # `ls /dev/ttyAMA*` after a reboot; the numbering depends on which you
    # enabled. 115200 is fixed by the chip in RVC mode; it is a setting only
    # because an adapter in between might not be.
    port: str = "/dev/ttyAMA1"
    baud: int = 115200
    # Rotation applied to the sensor's raw yaw to align it with the robot's
    # forward axis and true North (0 = North, CW positive). Tune during bring-up.
    heading_offset_deg: float = 0.0
    invert: bool = (
        False  # flip yaw sign to CW-positive if the board is mounted mirrored
    )
    # Minimum fused-orientation calibration level (0-3) before we trust the
    # heading. Below this, heading() returns None and the fusion falls back to GPS.
    min_calib: int = 1
    # How stale a reading may be and still count as the robot's heading. Past
    # this, heading() and yaw_rate() return None and navigation falls back to
    # the GPS course — the same idea as gps.fix_timeout, and needed for a
    # sharper reason: the IMU reader survives bus errors by retrying, so a
    # sensor that has stopped answering looks exactly like one reporting a
    # heading that happens not to be changing. 0 disables the check and restores
    # the old behaviour of trusting the last reading indefinitely.
    #
    # Not tighter than a second or so on purpose: heading source is not a free
    # switch (the waypoint controller runs different gains on an absolute
    # heading than on a GPS course), so flapping on every bus hiccup would be
    # its own bug. See sensors/bno085.py::DEFAULT_SAMPLE_TIMEOUT.
    sample_timeout: float = 2.0
    # Which IMU transport to use. The default I2C implementation matches the
    # existing BNO085 driver; `uart_rvc` targets the UART-RVC parser used by the
    # BNO08x sensor in RVC mode.
    transport: str = "i2c"
    # UART-RVC settings when transport is "uart_rvc".
    serial_port: str = "/dev/serial0"
    serial_baud: int = 115200
    # Run the sensor's dynamic calibration and save it to the BNO08x's own flash
    # once it converges, so the board boots calibrated. The chip persists this
    # itself — there is no offsets file to manage (the BNO055 needed one because it
    # forgot its calibration on every power cycle). False disables auto-save.
    persist_calibration: bool = True


@dataclass
class UltrasonicConfig:
    """An HC-SR04-style ultrasonic rangefinder, and the stop it can enforce.

    Two Fusion HAT DIGITAL pins (BCM GPIO numbers, the same numbering as
    `MotorConfig.encoder_a` and NOT the HAT's PWM channels): TRIG is pulsed,
    ECHO comes back high for as long as the sound took to make the round trip.
    See sensors/ultrasonic.py for the measurement and control/collision.py for
    what the number is used for.

    OFF by default. A stock chassis has no ultrasonic fitted, and enabling it
    claims two pins — which on a build that wired something else to them is a
    conflict, not a feature.

    --- what an ultrasonic can and cannot see ---
    It measures the distance to the nearest thing directly in front of the
    transducer, in a cone of roughly 15 degrees, and it is honest about hard
    flat obstacles a rover would otherwise hit: walls, boxes, table legs at
    close range. It is blind to a surprising amount besides — sound-absorbing
    fabric, anything angled steeply enough to bounce the ping away rather than
    back, a table edge above the beam, a drop in front of the wheels. So
    `avoid` below is a last-resort backstop for the obstacle class it does see,
    not a licence to stop looking where you are driving.
    """

    enabled: bool = False
    # BCM GPIO pins. -1 on either disables the sensor as surely as `enabled`,
    # which is what makes "no pins configured" a safe half-edited state.
    trig_pin: int = 27
    echo_pin: int = 22

    # --- measurement ---
    # Seconds between pings. The HC-SR04's datasheet asks for at least 60 ms so
    # the previous burst has died away; faster and one ping's echo is timed
    # against the next ping's start, which reads as an obstacle that is not
    # there. 15 Hz is already several samples inside a rover's stopping
    # distance, so there is nothing to win by pushing it.
    interval: float = 0.06
    # Readings outside this band are discarded rather than believed. Below
    # min_m the transducer is still ringing from its own burst; above max_m the
    # echo is too weak to be the thing it claims to be. Both are the module's
    # published limits, not a preference.
    min_m: float = 0.03
    max_m: float = 4.0
    # A reading older than this is not a reading. It covers the reader thread
    # dying and the sensor being unplugged, and it is why a stale distance can
    # never keep the drivetrain clamped after the sensor stops answering.
    max_age: float = 0.5
    # Median filter width, in samples. An ultrasonic's characteristic failure is
    # a single wildly short reading — a stray echo off the floor or off another
    # robot's sensor — and a median of three throws that away at the cost of one
    # ping of latency. 1 disables filtering.
    samples: int = 3

    # --- collision avoidance (control/collision.py) ---
    # False = measure only: the distance still reaches telemetry and routines,
    # and nothing is ever clamped. The escape hatch for an operator who wants
    # the readout without the intervention, and live, so it can be switched off
    # from the dashboard mid-match when the sensor is the thing misbehaving.
    avoid: bool = True
    # Forward motion is refused inside this distance. Measured from the FACE OF
    # THE TRANSDUCER, so a sensor mounted behind the bumper wants a larger
    # number by however far it sits back. Tune it against how long the rover
    # actually takes to stop at cruise, not against how close you want it.
    stop_m: float = 0.35
    # Forward throttle is scaled down between here and stop_m, reaching zero at
    # stop_m. Set it at or below stop_m for a hard stop with no run-in, which is
    # harsher on the drivetrain and easier to reason about.
    slow_m: float = 0.90
    # Hysteresis. Once stopped, forward is allowed again only past
    # stop_m + release_m. Without it, a rover sitting exactly at the threshold
    # with a noisy reading alternates between stopped and creeping every tick.
    release_m: float = 0.10


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
    #
    # To run OUR trained model instead of a zoo one, build a .rpk from the
    # Ultralytics checkpoint (see docs/MODEL_CONVERSION.md):
    #   uv run tools/imx500_export_yolo.py --data path/to/data.yaml
    # then set RS_VISION_IMX500_MODEL to the resulting network.rpk. That export
    # runs NMS on the sensor and emits a four-tensor layout the decoder detects
    # on its own — no intrinsics required, unlike the zoo networks.
    imx500_model: str = (
        "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    )
    # Class-name file, one per line. Empty = use the labels embedded in the .rpk,
    # which is right for every model-zoo network. A CUSTOM export needs this: our
    # exporter writes labels.txt next to packerOut.zip, and without it every box
    # comes back labelled "0"/"1"/"2" — which also breaks target_label matching.
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
    #
    # This is the fallback standoff, in box-height units. A routine state that
    # names `stop_within_m` overrides it for as long as that state is current,
    # converting metres through the calibration below.
    standoff_size: float = 0.45
    # Bounding box -> metres, as ONE measured pair: "at range_at_m, the box
    # measured range_size". See control/rangefinder.py for why one pair is the
    # whole model (the frame height and the focal length cancel).
    #
    # *** THESE TWO ARE A PLACEHOLDER, NOT A MEASUREMENT. *** They say the
    # shipped standoff_size of 0.45 sits at 1 m, which is self-consistent but
    # invented — it has no idea how tall your target actually is, and the real
    # object height is the term folded into the constant. One tape measure and
    # one run of tools/detector_selftest.py replaces them, and until that happens
    # every distance the dashboard shows is a guess with two significant figures
    # of false confidence. Set range_at_m to 0 to disable metre estimates
    # entirely and keep everything in box-height units.
    #
    # A rover with an ultrasonic fitted can replace them by itself; see the two
    # switches below. These stay as the fallback for every label it has not yet
    # seen from a measurable distance.
    range_at_m: float = 1.0
    range_size: float = 0.45

    # --- the ultrasonic, where this build has one (sensors/ultrasonic.py) -----
    # Both are inert without one, so both default on: they cost a build with no
    # sonar nothing at all, and a build with one is strictly better off.
    #
    # Answer with the SONAR's metres when it can be justified as looking at the
    # detected target — centred in the beam, in range, and measured at the same
    # moment as the frame (control/rangefinder.py has the gates). A measurement
    # beats an inference from a constant, and it is what lets a FOMO model —
    # which reports no box height at all — approach and hold a standoff for the
    # first time. Off leaves every distance coming from the box height.
    sonar_range: bool = True
    # LEARN the box-height constant from those same pairs, per object label, so
    # the camera goes on reporting real metres past the sonar's few. This is the
    # tape-measure calibration above, done continuously and without the tape.
    # Off if you would rather trust one number you set by hand.
    auto_range: bool = True
    # Pairs before a learned fit is believed. Each one has already survived a
    # row of gates and the fit is their median, so this does not need to be
    # large — and a large one is a rover that drives past the only distances it
    # could have learned anything from.
    range_samples: int = 8
    search_speed: float = 0.25  # slow rotate to reacquire a lost target; 0 disables


@dataclass
class FPVConfig:
    # First-person live video streamed to the base station over UDP (see
    # robot/comms/video_udp.py). Off by default: it needs the base station's IP
    # and a shared WiFi/LAN (the XBee radio can't carry video), so it's opt-in
    # rather than something every robot fires into the void on boot.
    enabled: bool = False
    base_host: str = "Yojans-MacBook-Pro.local"  # where the base station receives UDP
    base_port: int = 5005
    fps: int = 15  # cap on frames sent per second
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

    # --- flywheel launcher (closed-loop RPM path in drive/shooter.py) ---
    # 0 = this is a servo launcher: {"type":"shooter_spin"} does a pulse shot and
    # the RPM controller never runs. Above 0 = this is a flywheel: the same
    # command toggles the wheel between this speed and stopped, and the pulse
    # state machine is not used.
    #
    # On a build with no tachometer the loop is fed a MODELLED rpm (see
    # Shooter._estimated_rpm) rather than a measured one, so it behaves as
    # feed-forward: it holds the commanded speed, but it cannot see or correct
    # for battery sag, ball drag or a stalling wheel. Wire an encoder to
    # set_measured_rpm() and it becomes genuinely closed-loop with no other
    # change.
    target_rpm: float = 0.0

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
class BallisticsConfig:
    """Distance to a target -> the flywheel speed that reaches it.

    Consumed by `control/ballistics.py` and, through it, by the `spin_up`
    routine action. Separate from `ShooterConfig` because that describes the
    SERVO launcher and its firing policy, while this describes a throw: the
    geometry it leaves at and the wheel that throws it. A build can have one,
    both, or neither.

    Uncalibrated by default (`max_rpm` 0), and every conversion answers None
    until it isn't. That is the same honest failure `VisionConfig.range_at_m`
    makes: a robot that has never been measured must not turn a guess into a
    launch.
    """

    # Where the ball leaves, and where it has to land. Both measured from the
    # ground, so the pair works out the climb on its own — a launcher mounted
    # high shooting into a low bucket is a negative rise and perfectly valid.
    launch_angle_deg: float = 45.0  # fixed hood angle, from horizontal
    launch_height_m: float = 0.30
    target_height_m: float = 0.90  # the bucket's rim, not its base

    # The flywheel. `transfer` is the fraction of the wheel's surface speed the
    # ball actually leaves at — always under 1 because the contact slips and
    # some of the energy goes into spin. It is the one number here that cannot
    # be derived, only shot for: if every shot lands long, lower it.
    wheel_diameter_m: float = 0.10
    transfer: float = 0.5
    # Flywheel RPM at full throttle. 0 means "nobody has measured this", which
    # switches the whole model off rather than letting it invent a number.
    max_rpm: float = 0.0
    # Throttle floor. Below this a brushless ESC may not commutate at all, so a
    # very short shot would ask for a wheel speed it silently gets zero of.
    idle_power: float = 0.15


@dataclass
class SequenceStep:
    """One leg of a `sequence` mechanism: what to move, and when to move on.

    --- what `values` means ---
    Actuator name -> what that actuator should be doing for this step, read in
    the units of the actuator's own `kind`:

        esc    throttle, -1..1        (ESCMotor.set_throttle)
        servo  degrees, -90..90       (ESCMotor.set_angle)

    Two units in one map is a real cost, and it buys the thing this whole
    mechanism exists for: a step reads "put the feeder arm at 40 degrees" and
    "hold the flywheel at 0.9", which is how the build is actually described by
    the people who wired it. The alternative — throttle everywhere, as
    PowerMechanism does — means expressing a feeder's travel as a fraction of a
    symmetric throw about neutral, and getting a number nobody can check against
    the arm in front of them.

    AN UNNAMED ACTUATOR IS LEFT EXACTLY AS IT WAS. This is the single most
    important difference from `presets`, which zero what they do not mention.
    A preset describes the whole mechanism at one instant; a step describes a
    CHANGE, and the flywheel spun up in step 1 has to still be spinning while
    step 2 runs the feeder into it. `clear: true` opts a step back into
    preset behaviour when what you want really is "everything else off".

    --- when the step ends ---
    `seconds` is a MINIMUM DWELL, not a duration: the step ends once both the
    dwell has elapsed and `wait_for` is satisfied. Either half may be left out.
    Both together is the common shooter shape — "give the ESC 0.2 s to even
    react, then wait until the wheel is actually at speed".

    `wait_for` is the "and other factor" half, one of:

        {}                                                  no gate; time only
        {"kind": "rpm", "actuator": "fly", "at_least": 3000, "at_most": 0}
        {"kind": "mech_ready", "mech": "launcher"}

    `ramp` is the other half of "when": how long the actuators take to GET to
    `values`, rather than how long they hold once there. 0 (the default) writes
    the target immediately, which is what every step did before this existed.
    Above 0, each named actuator is walked linearly from where it actually was
    when the step began to its target over that many seconds — a flywheel eased
    up to speed instead of slammed there, a feeder arm that arrives at the ball
    instead of hitting it.

    It is a per-step field rather than a per-mechanism one because a shooter
    wants both in the same cycle: a long, gentle spin-up and then a feeder that
    must move NOW, before the wheel bleeds speed. A mechanism-wide slew rate
    would force one number onto both.

    The ramp is served before the step can end: the effective dwell floor is
    max(seconds, ramp), so a step cannot advance while its own actuators are
    still travelling. `seconds` therefore keeps meaning what it meant — extra
    hold AFTER arriving is `ramp + seconds` only if you want it that way; a
    plain `seconds` shorter than `ramp` is simply absorbed.

    `timeout` bounds the gate, because a gate that cannot be satisfied is a
    mechanism that never returns to rest. 0 means "use the mechanism's
    step_timeout". Reaching it runs `on_timeout`:

        abort    (default) stop the whole sequence and park at rest
        advance  carry on to the next step anyway

    `abort` is the default because the reason a shooter gates on RPM is that
    feeding a ball into a flywheel that never reached speed jams the mechanism.
    Advancing on timeout does the exact thing the gate was written to prevent.
    """

    name: str = ""            # what the dashboard calls this leg; "" => "step N"
    values: Dict[str, float] = field(default_factory=dict)
    seconds: float = 0.0      # minimum dwell before the step may end
    ramp: float = 0.0         # seconds to travel to `values`; 0 = jump there
    wait_for: Dict[str, Any] = field(default_factory=dict)
    timeout: float = 0.0      # 0 => inherit the mechanism's step_timeout
    on_timeout: str = "abort"  # abort | advance
    clear: bool = False       # zero the actuators this step does not name


@dataclass
class MechanismConfig:
    """One named non-drivetrain subsystem: an intake, an arm, a second launcher.

    This is `ShooterConfig` generalized. Three shapes cover what a build
    actually needs:

      power - hold a value. An intake spins at +1 to take a ball in, -1 to spit
              it out, 0 to stop. Several actuators move together, which is why
              a preset maps actuator name -> value rather than being a scalar.
      pulse - a timed cycle: swing to `active_angle`, hold `active_seconds`,
              return to `rest_angle`, settle for `recover_seconds`. Exactly the
              launcher's rest -> firing -> retracting machine (drive/shooter.py),
              and non-blocking for exactly the same reason.
   sequence - an ordered queue of `steps`, one at a time: the servo, then one
              motor, then another. The kind for a launcher whose actuators
              CANNOT all move at once, which neither of the above can express —
              `power` writes them together and `pulse` swings them together.
              See SequenceStep above.

    The built-in launcher is deliberately NOT expressed here — it keeps its own
    `ShooterConfig` so the RS_SHOOTER_* env vars, the `shooter.*` tuning paths
    and ShooterAlignController's firing policy stay exactly as they are. The
    name "shooter" is reserved by layout validation to avoid two things
    answering to it.
    """

    name: str = ""
    label: str = ""  # what the dashboard calls it; "" => derived from `name`
    kind: str = "power"  # power | pulse | sequence
    enabled: bool = True
    actuators: Dict[str, MotorConfig] = field(default_factory=dict)

    # --- power ---
    # Named states an operator or a routine can ask for by name, e.g.
    # {"in": {"roller": 1.0, "belt": 0.8}, "out": {"roller": -1.0, "belt": -0.8}}.
    # Presets are what the FSM editor offers, so a routine reads "intake -> in"
    # rather than a column of magic numbers.
    presets: Dict[str, Dict[str, float]] = field(default_factory=dict)
    auto_stop_seconds: float = 0.0  # 0 = run until told to stop

    # --- pulse ---
    rest_angle: float = -30.0
    active_angle: float = 30.0
    active_seconds: float = 0.35
    recover_seconds: float = 0.35
    cooldown: float = 0.0  # minimum seconds between activations
    max_activations: int = 0  # magazine capacity; 0 = unlimited

    # --- sequence ---
    # An ordered queue of legs, run one at a time off the control tick. Shares
    # `rest_angle`, `cooldown` and `max_activations` with `pulse`, which is not
    # a coincidence: a sequence IS a pulse with more than one leg and a gate on
    # each, and a build that outgrows `pulse` should not have to relearn the
    # fields it already set.
    steps: List[SequenceStep] = field(default_factory=list)
    # A ceiling on any step that does not set its own `timeout`. A sequence is
    # the one mechanism that can wait on something other than a clock, so it is
    # also the one that can wait forever; this is the backstop that means it
    # cannot. Never 0 in a validated layout.
    step_timeout: float = 5.0
    # Run the queue again from the top when it finishes, until `stop()`. For a
    # feeder that should keep cycling while a routine holds a state, rather than
    # one shot per activation.
    loop: bool = False


@dataclass
class RoutineConfig:
    """Policy for the UI-authored state machines (see robot/routine/).

    The documents themselves live in routines.json, not here — this is only the
    handful of knobs that decide what a routine is ALLOWED to do.
    """

    # A state that never transitions is a robot that never stops. Any state
    # without its own timeout inherits this one.
    state_timeout_default: float = 60.0
    # Whether a routine may arm the launcher at all. OFF by default, and worth
    # keeping that way: this is the one action a user-authored program can take
    # that makes something physically launch. Even with it on, arming is only
    # accepted inside a state that delegates to shooter_align, and is dropped on
    # every state exit, mode exit and e-stop.
    allow_arm: bool = False


@dataclass
class AlignConfig:
    # Behaviour of the object_align / shooter_align state machine. The geometry
    # it reasons about (standoff_size, hfov_deg, search_speed) stays in
    # VisionConfig, because those describe the *detector*, not the loop.
    forward_speed: float = 0.25  # creep throttle once roughly on bearing
    pivot_threshold: float = 0.25  # |error_x| above this => turn in place
    aligned_tolerance: float = 0.05  # |error_x| below this => "aligned"
    search_after: float = 0.5  # ride out dropouts this long before sweeping
    search_timeout: float = 10.0  # give up (stop) after sweeping this long
    # Tuned for ~100-200 ms of perception dead time, which is what actually
    # limits stability here. Anything hotter oscillates: the robot steers on an
    # error it measured two frames ago. Start low, not high.
    pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(kp=0.5, ki=0.0, kd=0.05, out_limit=0.8)
    )


@dataclass
class BallIntakeConfig:
    # Behaviour of the ball_intake autonomy mode (robot/control/ball_intake.py).
    # Distinct from AlignConfig because the policy is the opposite: object_align
    # stops SHORT at a standoff, this drives THROUGH the ball with the intake
    # running. Sharing one config would mean one forward_speed for "creep up to
    # a bucket" and "drive over a ball", which are not the same number.
    mechanism: str = "intake"  # which layout mechanism to run; "" = none wired
    target_label: str = "ball"  # detections with any other label are ignored
    intake_power: float = 1.0  # +1 takes in, -1 spits (see PowerMechanism)
    # error_y is normalized: 0 = frame centre, +1 = bottom edge. At or below
    # this the ball is at the intake mouth.
    collect_line: float = 0.4
    chase_speed: float = 0.5  # throttle when the ball is far up the frame
    collect_speed: float = 0.3  # creep once at the mouth
    push_speed: float = 0.3  # blind, after the ball drops out of frame
    pivot_threshold: float = 0.35  # |error_x| above this => turn in place
    # Open-loop, because nothing on this robot can see under the intake. The
    # intake timer is the longer of the two: a ball in the throat is still
    # being collected after the robot has stopped moving. Stopwatch values.
    collect_push_s: float = 1.0  # keep DRIVING this long after losing sight
    intake_hold_s: float = 3.0  # keep the INTAKE turning this long
    # Sweep in place, then step forward, and repeat. Spinning alone only ever
    # searches one circle of the field.
    search_spin_s: float = 5.0
    search_advance_s: float = 1.0
    search_spin_speed: float = 0.25
    search_advance_speed: float = 0.3
    pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(kp=0.5, ki=0.0, kd=0.05, out_limit=0.8)
    )


@dataclass
class NavConfig:
    # Waypoint navigation (robot/control/waypoint.py).
    arrive_radius_m: float = 2.0  # a leg is done inside this radius
    cruise_speed: float = 0.35  # throttle once roughly on bearing
    # Straight-line throttle used only to acquire an initial heading when none
    # is available. Must exceed GPSConfig.min_move_mps or no course ever fixes.
    acquire_speed: float = 0.4
    # Heading error above this pivots in place — but only on an absolute (IMU)
    # heading. On a GPS course it becomes an arcing turn at acquire_speed,
    # because a pivot in place freezes the track angle. See waypoint.py.
    pivot_threshold_deg: float = 25.0
    # Gains for the fast, standstill-valid IMU heading. Error is in DEGREES, so
    # these are small on purpose: kp=0.02 saturates out_limit at 30° of error,
    # which leaves the whole 0-25° trim band proportional instead of bang-bang.
    heading_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(
            kp=0.02, ki=0.002, kd=0.008, out_limit=0.6, i_limit=50.0
        )
    )
    # Gains used instead whenever the heading is the GPS track angle rather than
    # an IMU attitude (heading_source="gps", or "auto" with the IMU absent or
    # still calibrating). Deliberately slower: this loop closes around a 5 Hz
    # course over ground against a 50 Hz control loop, so it gets about half the
    # authority, heavier damping, and no integral at all (a course has no
    # steady-state bias worth trimming, and integrating a stale error only winds
    # up). Sized when the fix rate was 1 Hz and not re-tuned since — conservative
    # rather than wrong, but there is authority here to reclaim on hardware.
    gps_heading_pid: PIDConfig = field(
        default_factory=lambda: PIDConfig(
            kp=0.008, ki=0.0, kd=0.006, out_limit=0.4, i_limit=50.0
        )
    )
    # Report the active mode's closed loops in telemetry (setpoint, error,
    # output, and the P/I/D split), so the dashboard can graph them while you
    # turn the gains above.
    #
    # OFF by default, and that is not timidity: it is ~60 bytes per loop on
    # every telemetry frame, on a 57600-baud radio that is also carrying
    # driving. Switch it on to tune, off to race. It applies live, so switching
    # it on costs a tap and no restart.
    pid_trace: bool = False


@dataclass
class RobotConfig:
    drive: DriveConfig = field(default_factory=DriveConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    gps: GPSConfig = field(default_factory=GPSConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    # Ultrasonic rangefinder + the collision guard it feeds. Off unless this
    # build has one fitted; see UltrasonicConfig.
    ultrasonic: UltrasonicConfig = field(default_factory=UltrasonicConfig)
    fpv: FPVConfig = field(default_factory=FPVConfig)
    shooter: ShooterConfig = field(default_factory=ShooterConfig)
    # How hard to throw, given how far away the thing is. Off until measured;
    # see BallisticsConfig.
    ballistics: BallisticsConfig = field(default_factory=BallisticsConfig)
    align: AlignConfig = field(default_factory=AlignConfig)
    ball_intake: BallIntakeConfig = field(default_factory=BallIntakeConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    # Extra subsystems declared by the layout (intake, arm, a second launcher).
    # Empty on a stock build, which is why nothing above changes shape.
    mechanisms: Dict[str, MechanismConfig] = field(default_factory=dict)
    routines: RoutineConfig = field(default_factory=RoutineConfig)
    # Control loop rate. This is the rate the motors are actually updated at, so
    # it sets both the floor on teleop latency (a command waits up to 1/loop_hz
    # before anything looks at it) and the granularity of the slew limiter, which
    # is what interpolates between the base station's ~15 Hz drive frames. At
    # 5 Hz the outputs moved in 200 ms stair-steps and slew_rate=4.0 allowed a
    # 0.8 jump per tick, i.e. no real limiting: choppy AND laggy. At 50 Hz the
    # step is 0.08 and motion is continuous. docs/ARCHITECTURE.md has always
    # specified 50 Hz.
    loop_hz: float = 50.0
    start_mode: str = "teleop"  # teleop | object_align | waypoint |
    #                             shooter_align | ball_intake
    # Which sensor answers "which way am I facing" (see sensors/pose.py):
    #   auto - IMU when calibrated, else the GPS track angle (recommended)
    #   gps  - the GPS track angle only; no IMU needed for heading
    #   imu  - the IMU only; no fallback to course over ground
    heading_source: str = "auto"
    robot_id: str = "rover1"  # unique id on the shared XBee channel
    telemetry_hz: float = (
        5.0  # rate of status frames back to the base station (0 disables)
    )
