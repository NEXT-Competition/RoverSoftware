"""Top-level robot orchestrator: wires hardware, comms, and control together."""

from __future__ import annotations

import collections
import os
import queue
import signal
import threading
import time
from typing import Callable, Dict, Optional, Tuple

from . import layout, tuning
from .config import PIDConfig, RobotConfig
from .comms.doc_transfer import Reassembler, split
from .comms.ip_link import IPLink
from .comms.xbee_link import XBeeLink
from .comms import wifi
from .control.controller import Controller
from .control.manager import ControlManager
from .control.ball_intake import BallIntakeController
from .control.object_align import ObjectAlignController
from .control.pid import PID
from .control.ballistics import Ballistics
from .control.collision import CollisionGuard
from .control.rangefinder import Rangefinder
from .control.routine_controller import RoutineController
from .control.script_controller import ScriptController
from .control.shooter_align import ShooterAlignController
from .control.teleop import TeleopController
from .control.waypoint import WaypointController
from .routine import schema as routine_schema
from .routine import store as routine_store
from .script import schema as script_schema
from .script import store as script_store
from .drive.drivetrain import build_drivetrain
from .drive.mechanism import Mechanism, build_mechanism
from .drive.shooter import Shooter
from .sensors.imu import build_imu
from .sensors.camera import Camera
from .sensors.detector import MockDetector, ObjectDetector
from .sensors.fpv import FPVStreamer
from .sensors.gps import GPS
from .sensors.imx500 import IMX500Detector, resolve_backend
from .sensors.pose import PoseEstimator
from .sensors.ultrasonic import build_ultrasonic
from .sensors.imu import IMU

# Log a warning if a control tick's work (excluding the sleep) exceeds this. A
# healthy tick is a few ms; a stall points at blocking I/O (serial or I2C).
SLOW_TICK_S = 0.1

# Ceiling on queued bulk frames. The outbox is only ever fed by an explicit
# request from the base station, so in normal use it holds one document; this
# exists so a robot whose radio has died doesn't grow a queue for the rest of
# the match. The oldest frames go first, which is the right end to lose — they
# belong to a transfer the other side has long since given up on.
OUTBOX_MAX = 256

# How long a bench jog runs before it stops itself. The Hardware tab repeats the
# command while the control is held, so this is the same shape as teleop's
# command_timeout: lose the link mid-jog and the motor stops on its own.
JOG_TIMEOUT_S = 0.4

# Exit status for "restart me", asked for over the radio and answered by
# stopping cleanly and letting the supervisor start a fresh process.
#
# NON-ZERO on purpose. The shipped unit says `Restart=on-failure`
# (packaging/systemd/roversoftware-robot.service), which would NOT restart a
# process that exited 0 — and the rovers in the field are running whatever unit
# was installed with their .deb, while `just sync` pushes only Python. A restart
# that depended on a new unit file would leave a rover dark until somebody
# walked out to it with a laptop. A distinctive code rather than 1 so the reason
# is legible in `systemctl status`, which reports it verbatim.
EXIT_RESTART = 42

# How often script console output is forwarded, and how much of it goes in one
# frame. A script printing every tick would otherwise put 50 frames a second
# into the outbox; four a second reads as live to a human and costs nothing.
SCRIPT_OUTPUT_PERIOD_S = 0.25
SCRIPT_OUTPUT_LINES = 40


def _pid(cfg: PIDConfig) -> PID:
    return PID(
        kp=cfg.kp, ki=cfg.ki, kd=cfg.kd, out_limit=cfg.out_limit, i_limit=cfg.i_limit
    )


def _retune(pid: PID, cfg: PIDConfig) -> None:
    """Copy gains onto a live PID without touching its integrator.

    Deliberately not a reset: retuning mid-run should nudge the loop, not make
    the robot forget where it was pointing and lurch.
    """
    pid.kp, pid.ki, pid.kd = cfg.kp, cfg.ki, cfg.kd
    pid.out_limit, pid.i_limit = cfg.out_limit, cfg.i_limit


class Robot:
    def __init__(
        self, config: RobotConfig, controllers: Optional[Dict[str, Controller]] = None
    ):
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
            for name, mech in config.mechanisms.items()
            if mech.enabled
        }
        # Everything that can be told to move or stop, including the launcher.
        # Built once and shared by reference, because the routine controller
        # keeps a handle on it — rebuilding it per call would hand the FSM a
        # snapshot that stops reflecting the robot.
        self._registry: Dict[str, Mechanism] = dict(self.mechanisms)
        if self.shooter is not None:
            self._registry["shooter"] = self.shooter
        # Hand every mechanism the same registry, so a sequence step can wait on
        # another mechanism being ready. By reference, for the reason above.
        for mech in self.mechanisms.values():
            mech.bind(self._registry)
        # Edge state for the e-stop hook in run(); see _apply_estop.
        self._estop_latched = False
        # The last command that actually reached the drivetrain, guard and all.
        # Read by the script API; see _wire_script_controller.
        self._last_command: Tuple[float, float] = (0.0, 0.0)
        # Multi-frame replies (a config snapshot, a layout, a routine set) wait
        # here for the WiFi link. Each entry is (frame, radio_ok), where the flag
        # marks the one kind of frame still allowed onto the radio. See _queue.
        self._outbox: "collections.deque[Tuple[dict, bool]]" = collections.deque(
            maxlen=OUTBOX_MAX
        )

        # Bounding box -> metres, so an aligning controller can be told to stop
        # a distance away rather than at a box height. ONE of them, shared: it
        # describes the camera and the target, not which loop is driving, and
        # two copies would be two things to re-calibrate. Built even when
        # controllers are injected, because _push_live_config re-calibrates it.
        # Given an ultrasonic, it also stops being a guess: the sonar's metres
        # answer directly whenever the target is centred in its beam, and every
        # such frame is a free calibration pair that teaches the box-height
        # constant for that label (see control/rangefinder.py). The sonar
        # provider is wired below, once the sensor exists.
        self.rangefinder = Rangefinder(
            config.vision.range_at_m,
            config.vision.range_size,
            hfov_deg=config.vision.hfov_deg,
            min_samples=config.vision.range_samples,
            learn=config.vision.auto_range,
            prefer_sonar=config.vision.sonar_range,
        )

        # Metres -> flywheel speed, the other half of the same idea: the
        # rangefinder says how far away the bucket is, this says how hard to
        # throw to reach it. Holds the config OBJECT, so every knob on it is
        # live and _push_live_config has nothing to copy. Built unconditionally
        # and inert until measured — `max_rpm` of 0 makes every answer None.
        self.ballistics = Ballistics(config.ballistics)

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
                    # A standoff said in metres, converted through the
                    # rangefinder on the way in. 0 leaves standoff_size alone.
                    standoff_m=v.standoff_m,
                    rangefinder=self.rangefinder,
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
                    # Same standoff as object_align, so `require_arrived` gates
                    # firing at the same distance with no shooter-side change.
                    standoff_m=v.standoff_m,
                    rangefinder=self.rangefinder,
                    search_speed=v.search_speed,
                    hfov_deg=v.hfov_deg,
                    pid=_pid(a.pid),
                ),
                # Ball collection. The intake mechanism is attached below,
                # if the layout declares one - the controller drives and steers
                # without it, it just cannot collect.
                "ball_intake": BallIntakeController(
                    target_label=config.ball_intake.target_label,
                    intake_power=config.ball_intake.intake_power,
                    collect_line=config.ball_intake.collect_line,
                    chase_speed=config.ball_intake.chase_speed,
                    collect_speed=config.ball_intake.collect_speed,
                    push_speed=config.ball_intake.push_speed,
                    pivot_threshold=config.ball_intake.pivot_threshold,
                    collect_push_s=config.ball_intake.collect_push_s,
                    intake_hold_s=config.ball_intake.intake_hold_s,
                    search_spin_s=config.ball_intake.search_spin_s,
                    search_advance_s=config.ball_intake.search_advance_s,
                    search_spin_speed=config.ball_intake.search_spin_speed,
                    search_advance_speed=config.ball_intake.search_advance_speed,
                    pid=_pid(config.ball_intake.pid),
                ),
                "waypoint": WaypointController(
                    arrive_radius_m=config.nav.arrive_radius_m,
                    cruise_speed=config.nav.cruise_speed,
                    acquire_speed=config.nav.acquire_speed,
                    pivot_threshold_deg=config.nav.pivot_threshold_deg,
                    heading_pid=_pid(config.nav.heading_pid),
                    gps_heading_pid=_pid(config.nav.gps_heading_pid),
                ),
            }
            # The FSM mode. Constructed with the SAME dict it will be registered
            # in, so a state that says "drive with object_align" delegates to
            # the instance whose providers get wired below — not a second one
            # that would see nothing. It holds no routines until one is loaded,
            # and until then it simply holds the robot still.
            controllers["routine"] = RoutineController(
                controllers, self._registry, config.routines
            )
            # The other authoring mode: the operator's own Python, run on a
            # worker thread against the API in robot/script/. Constructed with
            # the same dict for the same reason the FSM is — a script that says
            # `rover.hand_over("object_align")` must reach the instance whose
            # providers get wired below.
            controllers["script"] = ScriptController(
                controllers, self._registry, config.scripts
            )
        self.manager = ControlManager(controllers, config.start_mode)

        # Messages arrive on the XBee reader thread; process them on the main
        # loop by funneling through a thread-safe queue.
        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self.link = XBeeLink(config.comms.port, config.comms.baud, self._inbox.put)
        # Bulk transfers go over WiFi, full stop: a ~2.9 KB config snapshot is
        # half a second of airtime on a channel shared with every robot, and it
        # is telemetry that gets dropped to make room. Unconfigured or
        # unreachable means config and documents simply don't move — the radio
        # keeps carrying driving, telemetry and the e-stop, which is what it is
        # for. The one exception is the address of this link itself; see
        # tuning.BOOTSTRAP_PATHS and _retarget_ip_link. Inbound frames join the
        # SAME inbox, so _drain_inbox can't tell how one arrived.
        self.ip_link: Optional[IPLink] = (
            IPLink(
                config.comms.base_host,
                config.comms.base_port,
                self._inbox.put,
                config.robot_id,
            )
            if config.comms.base_host
            else None
        )
        self._running = False
        # One WiFi request at a time (see _wifi_command). Plain bool rather than
        # a lock: only the inbox drain sets it True, and only the worker clears
        # it, so there is no window where two threads could both claim it.
        self._wifi_busy = False

        # Adafruit Ultimate GPS feeds waypoint navigation, position telemetry and
        # the track-angle heading. Reads on its own thread; pose() is a cheap
        # cached lookup for the control loop. Disabled -> no position, and
        # waypoint mode holds position.
        self.gps: Optional[GPS] = (
            GPS(
                config.gps.port,
                config.gps.baud,
                config.gps.fix_timeout,
                config.gps.min_move_mps,
                config.gps.update_rate_ms,
            )
            if config.gps.enabled
            else None
        )

        # BNO085 IMU supplies an absolute, standstill-valid heading (which the
        # GPS track angle is not). Reads on its own thread; heading() is a cheap

        # cached lookup. Disabled/uncalibrated -> heading falls back to the track
        # angle (see heading_source).
        self.imu: Optional[IMU] = (
            IMU(
                config.imu.i2c_address,
                config.imu.heading_offset_deg,
                config.imu.invert,
                config.imu.min_calib,
                config.imu.persist_calibration,
                sample_timeout=config.imu.sample_timeout,
                transport=config.imu.transport,
                serial_port=config.imu.serial_port,
                serial_baud=config.imu.serial_baud,
            )
            if config.imu.enabled
            else None
        )

        # Ultrasonic rangefinder: how far away the thing straight ahead is.
        # Pings on its own thread; distance_m() is a cached lookup, so a ping
        # that blocks for its echo timeout never lands inside a control tick.
        # None on a build with no ultrasonic fitted, which is the default.
        self.ultrasonic = build_ultrasonic(config.ultrasonic)

        # Don't drive forward into it. Built whether or not there is a sensor —
        # it holds the config object, so switching `avoid` off from the
        # dashboard is live — and inert without one: no distance means no
        # intervention, which is how it fails on every build that has never had
        # an ultrasonic. Sits between the active controller and the drivetrain
        # (see run()), so every mode gets it rather than each one re-deciding.
        self.collision = CollisionGuard(
            config.ultrasonic,
            self.ultrasonic.distance_m if self.ultrasonic is not None else None,
        )

        # The second thing the ultrasonic is good for: telling the camera how
        # far away things actually are. `stamped_m` rather than `distance_m`
        # because pairing a distance with a FRAME needs to know when it was
        # measured — the two sensors run at unrelated rates, and a reading from
        # the wrong moment is a calibration sample that quietly lies.
        if self.ultrasonic is not None:
            self.rangefinder.set_sonar_provider(self.ultrasonic.stamped_m)

        # Fuse GPS position + heading behind one pose_provider (unchanged shape:
        # () -> (lat, lon, heading_deg)), so telemetry and the waypoint controller
        # get the best heading without knowing which sensor produced it.
        self.pose_estimator = PoseEstimator(self.gps, self.imu, config.heading_source)
        self.pose_provider: Optional[
            Callable[[], Optional[Tuple[float, float, Optional[float]]]]
        ] = self.pose_estimator.pose if (self.gps is not None) else None
        self._last_telem = 0.0
        # When the slow tier of the telemetry frame last went out. Zero so the
        # first frame carries everything: a base station that has just come up
        # should have a full picture immediately, not a second later.
        self._last_detail = 0.0

        # One camera, shared by every frame consumer (the detector and the FPV
        # streamer) — a V4L2/CSI device can't be opened twice. Only spun up if
        # something actually wants frames; it reads on its own thread so the
        # 50 Hz loop never touches the device.
        mock_det = os.environ.get("RS_MOCK_DETECTOR", "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        # Constructing a Camera opens nothing — start() does, on its own thread —
        # so one is built whenever the robot has a camera configured, even if
        # nothing wants frames yet. That is what lets FPV be switched on from the
        # base station later: the first consumer to arrive opens the device.
        self._detector_wants_frames = config.vision.enabled and not mock_det
        # Which detector we run also decides which capture backend to open (an
        # IMX500 detector needs the camera that loaded the sensor's network), so
        # resolve it BEFORE constructing the camera. Skipped entirely for the
        # mock, which touches no hardware.
        self.vision_backend = (
            resolve_backend(config.vision, config.camera)
            if (config.vision.enabled and not mock_det)
            else "mock"
            if mock_det
            else "off"
        )
        self.camera: Optional[Camera] = (
            Camera(config.camera, config.vision) if config.camera.enabled else None
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
        # Built whether or not the feed is currently on, so the base station can
        # switch it on later — there is nothing to construct at that point, and a
        # streamer that only existed when it was already running would make
        # `fpv.enabled` a restart-only setting for no reason but this line.
        self.fpv: Optional[FPVStreamer] = FPVStreamer(
            config.fpv, self.camera, config.robot_id
        )
        overlays = getattr(self.detector, "overlays", None)
        if self.fpv is not None and overlays is not None:
            self.fpv.set_overlay_provider(overlays)

        # Give the waypoint controller the fused pose source, the IMU yaw-rate
        # (for the heading PID's measured derivative), and which sensor actually
        # answered — a GPS course over ground gets the slower gains and never
        # pivots in place, because a pivot doesn't move the antenna.
        wp = controllers.get("waypoint")
        if isinstance(wp, WaypointController) and self.pose_provider is not None:
            wp.set_pose_provider(self.pose_provider)
            wp.set_rate_provider(self.pose_estimator.heading_rate)
            wp.set_absolute_heading_provider(self.pose_estimator.heading_is_absolute)

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
                elif isinstance(c, BallIntakeController):
                    # Same perception, no rate provider: this loop steers on the
                    # detection alone and has no heading to hold. It also needs
                    # the vision config, so the detector filters to balls BEFORE
                    # picking its one box per frame — see set_vision_config.
                    c.set_detection_provider(self.detector.detection)
                    if config.vision.enabled:
                        c.set_vision_config(config.vision)

        # And its actuator, if the layout declares one. A build with no intake
        # still gets a controller that drives and steers - it simply collects
        # nothing - rather than a mode that fails to construct.
        for c in controllers.values():
            if isinstance(c, BallIntakeController):
                mech = self.mechanisms.get(config.ball_intake.mechanism)
                if mech is not None:
                    c.set_intake(mech)
                elif config.ball_intake.mechanism:
                    print(
                        f"[ball_intake] no mechanism named "
                        f"{config.ball_intake.mechanism!r} in the layout - "
                        f"will chase balls but not collect them"
                    )

        # How near an approach is allowed to get, whatever it was asked for: the
        # collision guard clamps the command these controllers return, so a
        # standoff inside its stop distance is unreachable. Told rather than
        # enforced — the guard needs no help — so that the mismatch can be said
        # out loud instead of presenting as a routine state that never finishes.
        for c in controllers.values():
            if isinstance(c, ObjectAlignController):
                c.set_min_standoff(self._min_standoff())

        # The FSM's own sensing. Its conditions read the controllers' published
        # state (aligned, arrived, route_done) rather than re-deriving any of
        # it, so this is only what no controller owns: position and the latch.
        self.routine_controller: Optional[RoutineController] = next(
            (c for c in controllers.values() if isinstance(c, RoutineController)), None
        )
        if self.routine_controller is not None:
            if self.pose_provider is not None:
                self.routine_controller.set_pose_provider(self.pose_provider)
            self.routine_controller.set_estop_provider(lambda: self.manager.estop)
            # Metres to whatever is straight ahead, so a state can transition on
            # "something is close" with no model involved — see the
            # `target_distance` condition. Only on a build that has one; without
            # it the condition simply never fires, and the document still loads.
            if self.ultrasonic is not None:
                self.routine_controller.set_sonar_provider(self.ultrasonic.distance_m)
            # What a state aligns to. The config object, shared by reference
            # with the detector, so a state's target takes effect on the frame
            # after it is set and is put back when the state is left.
            if config.vision.enabled:
                self.routine_controller.set_vision_config(config.vision)
            # Unconditional, unlike vision: an uncalibrated model already
            # declines every shot on its own, and handing it over anyway is what
            # lets `spin_up` explain WHY it isn't spinning ("no max_rpm") rather
            # than the vaguer "this build has no ballistics config".
            self.routine_controller.set_ballistics(self.ballistics)

        # A script's own sensing. Wider than the FSM's on purpose: a routine
        # asks yes/no questions the controllers already answer, while a script
        # does arithmetic on the readings, so it gets the numbers themselves —
        # every one of them optional, so a build with no GPS, no camera or no
        # ultrasonic still runs scripts with those readings simply absent.
        self.script_controller: Optional[ScriptController] = next(
            (c for c in controllers.values() if isinstance(c, ScriptController)), None
        )
        self._wire_script_controller(config)

        # Documents the base station has saved on this robot. A layout needs a
        # restart to take effect, so it is loaded in run_robot.py before the
        # hardware is built; routines and scripts are just data, so they load
        # here.
        self._routine_doc: dict = routine_store.empty_doc()
        self._load_routines()
        self._script_doc: dict = script_store.empty_doc()
        self._load_scripts()

        # Reassembly buffers for chunked documents arriving off the radio, one
        # per document type so a layout, a routine and a script save can't
        # interleave.
        self._rx = {
            "put_layout": Reassembler(),
            "put_routines": Reassembler(),
            "put_scripts": Reassembler(),
        }
        self._layout_rev = 0
        self._routine_rev = 0
        self._script_rev = 0
        # Script console output, forwarded over the bulk link rather than the
        # radio: it is kilobytes of text and nothing about it is urgent. See
        # _drain_script_output.
        self._script_out_at = 0.0
        # Bench-jog failsafe; see _jog.
        self._jog_mech = ""
        self._jog_until = 0.0
        # Set by a restart asked for over the radio; read by run() on the way
        # out to decide what to tell the supervisor. See _request_restart.
        self._restarting = False

    def _wire_script_controller(self, config: RobotConfig) -> None:
        """Hand the script mode its view of this particular robot.

        Every provider is a plain callable rather than the sensor itself, the
        same rule the routine layer follows: `robot/script/` must stay testable
        against stubs on a laptop, and nothing in it imports `sensors/`. It is
        also what makes "this build has no GPS" a missing key in a snapshot
        rather than a branch in the API.
        """
        script = self.script_controller
        if script is None:
            return
        if self.pose_provider is not None:
            script.set_pose_provider(self.pose_provider)
        script.set_estop_provider(lambda: self.manager.estop)
        script.set_command_provider(lambda: self._last_command)
        if self.gps is not None:
            script.set_gps_provider(self.gps.telemetry)
        if self.imu is not None:
            # Paired with the fused heading rather than the raw one, and gated
            # on freshness for the reason telemetry gates it: a calibration
            # level beside a heading the sensor stopped reporting is the
            # dashboard being reassuring about a dead sensor.
            script.set_imu_provider(
                lambda: {
                    "heading": self.imu.heading() if self.imu.fresh() else None,
                    "calib": self.imu.calibration() if self.imu.fresh() else None,
                }
            )
        if self.ultrasonic is not None:
            script.set_sonar_provider(self.ultrasonic.distance_m)
        if self.detector is not None:
            script.set_vision_provider(self._vision_summary)
        if self.drive is not None:
            script.set_encoder_provider(self._wheel_speeds)
        if config.vision.enabled:
            script.set_vision_config(config.vision)
        # Unconditional, like the routine controller's: an uncalibrated model
        # declines every shot on its own, and handing it over anyway is what
        # lets `spin_for` explain WHY it isn't spinning rather than the vaguer
        # "this build has no ballistics config".
        script.set_ballistics(self.ballistics)

    def _vision_summary(self) -> dict:
        """What the detector sees, with the range estimate folded in.

        The same summary telemetry carries — deliberately, so a script and the
        operator watching the dashboard are reading one number, not two that
        were derived slightly differently and disagree by the time anybody
        notices.
        """
        if self.detector is None:
            return {}
        summary = self.detector.telemetry()
        detection = self.detector.detection()
        distance = self.rangefinder.distance_for(detection)
        if distance is not None:
            summary = {**summary, "dist": round(distance, 2)}
        return summary

    def _wheel_speeds(self) -> Optional[dict]:
        """Measured track speed, per actuator and per side.

        Per side as well as per actuator because that is the number a script
        actually steers on — "how fast is the left track going" — while the
        per-actuator names are what you read to find the one wheel dragging.
        None on a build with no encoders, which is most of them.
        """
        status = self.drive.status()
        if status is None:
            return None
        side_rpm = getattr(self.drive, "side_rpm", None)
        roles = getattr(self.cfg.drive, "roles", None)
        if side_rpm is None or roles is None:
            return status
        return {**status, "l": side_rpm(roles.left), "r": side_rpm(roles.right)}

    def _min_standoff(self) -> float:
        """The nearest an approach can actually finish, given the guard.

        0 when nothing is clamping — no ultrasonic, or avoidance switched off.
        """
        u = self.cfg.ultrasonic
        return u.stop_m if (u.enabled and u.avoid) else 0.0

    def _learn_range(self, cmd) -> None:
        """Turn this tick's (box height, measured distance) into calibration.

        Only on a build that has both sensors, and cheap enough for the loop:
        two cached reads and a row of comparisons, with the work skipped
        entirely unless the detector has produced a NEW frame since last time
        (see Rangefinder.observe). The throttle goes with it because the vision
        pipeline's latency is unknown and one-signed, so a sample taken while
        moving is a sample biased in a consistent direction — and a bias is the
        one error a median cannot filter out.
        """
        if self.detector is None or self.ultrasonic is None:
            return
        self.rangefinder.observe(
            self.detector.detection(), throttle=(cmd.left + cmd.right) / 2.0
        )

    def _all_mechanisms(self) -> Dict[str, Mechanism]:
        """Layout mechanisms plus the built-in launcher, keyed by name."""
        return self._registry

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
            elif mtype == "get_fields":
                self._send_fields()
            elif mtype == "get_layout":
                self._send_doc("layout", layout.to_doc(self.cfg), self._layout_rev)
            elif mtype == "get_routines":
                self._send_doc("routines", self._routine_doc, self._routine_rev)
            elif mtype == "get_scripts":
                self._send_doc("scripts", self._script_doc, self._script_rev)
            elif mtype in ("put_layout", "put_routines", "put_scripts"):
                self._receive_doc(mtype, msg)
            # WiFi. Handled here for the same reason config is — it reaches past
            # every controller — and worked off-thread because nmcli blocks for
            # seconds (see _wifi_task).
            elif mtype in ("get_wifi", "scan_wifi", "set_wifi", "forget_wifi"):
                self._wifi_command(msg)
            # Whether a browser currently has this rover's camera open. Not a
            # config change — it is not the operator's setting and must not be
            # persisted — so it reaches the streamer directly, like a jog.
            elif mtype == "fpv":
                self._set_fpv_wanted(msg)
            elif mtype == "jog":
                self._jog(msg)
            # Applying a preset reaches past the active controller for the same
            # reason `jog` does: it is the operator asking a mechanism for a
            # named state, not an instruction to whatever is currently driving.
            elif mtype == "mech_preset":
                self._mech_preset(msg)
            # Working the shooter by hand is the same kind of thing, and for the
            # same reason it does not go through the active controller.
            elif mtype == "shooter_spin":
                self._toggle_shooter(msg)
            # Restarting is the one command that ends this loop, so it is
            # handled here rather than by any controller.
            elif mtype == "restart":
                self._request_restart()
            # Which routine is chosen reaches past the active controller, for
            # the same reason config does: picking what to run next is not
            # something the thing currently driving should get a vote on.
            #
            # It has to be here rather than in ControlManager, which forwards
            # to the active controller only. Selecting a routine while the
            # rover was in teleop therefore handed `select_routine` to
            # TeleopController, which drops what it doesn't recognise — and the
            # `mode: routine` that follows a moment later then started
            # whichever routine had been selected BEFORE. The operator pressed
            # one routine and watched a different one drive away. Sending the
            # mode first only shortens that to a burst, which is no better when
            # the burst is a motor.
            elif mtype in ("select_routine", "routine_cmd", "routine_event"):
                routine = self.manager.controllers.get("routine")
                if routine is not None:
                    routine.on_message(msg)
            # And the same for scripts, for exactly the same reason: which
            # program runs next is not something the thing currently driving
            # should get a vote on, and routing `select_script` through the
            # active controller is how an operator ends up watching a different
            # script drive away than the one they pressed.
            elif mtype in ("select_script", "script_cmd"):
                script = self.manager.controllers.get("script")
                if script is not None:
                    script.on_message(msg)
            else:
                self.manager.handle_message(msg)

    # --- configuration ------------------------------------------------------

    def _config_frame(self, values: dict, **extra) -> dict:
        """A {"type":"config"} frame. `values` is a partial set the base station
        MERGES into its cached copy — a full snapshot on request, just the
        applied fields after an edit (the radio is shared with telemetry)."""
        return {"type": "config", "from": self.cfg.robot_id, "config": values, **extra}

    def _send_config(self) -> None:
        """Answer get_config, split into small frames (see tuning.chunks).

        The chunking predates the WiFi link and outlives it: it is also what
        keeps a partial transfer useful, since the base station MERGES config
        frames rather than replacing on each one.
        """
        for part in tuning.chunks(tuning.snapshot(self.cfg)):
            self._queue(self._config_frame(part))

    def _queue(self, msg: dict, radio_ok: bool = False) -> None:
        """Hand a frame to the paced outbox instead of the radio directly.

        A multi-frame reply written all at once overruns the radio: the write
        blocks, hits XBeeLink's 0.2 s timeout, and the frame is dropped to keep
        the control loop alive — which is exactly how a settings page ends up
        permanently blank, because a fragment of a document is not something the
        other side knows to ask for again. The outbox meters them out at the
        line rate instead (comms/airtime.py). Telemetry still goes direct: it is
        one small frame, it is the thing an operator most needs to be current,
        and delaying it behind a config dump is the wrong trade.

        This queue is the traffic that belongs on WiFi: bulk, non-realtime, and
        too big for a channel shared with every robot on the field. It now goes
        there or nowhere — `radio_ok=True` is the bootstrap exception (see
        tuning.BOOTSTRAP_PATHS) and nothing else should set it. See
        _drain_outbox.
        """
        self._outbox.append((msg, radio_ok))

    def _drain_outbox(self) -> None:
        """Empty the outbox over WiFi. The radio carries only bootstrap frames.

        On WiFi there is no airtime to protect and no write timeout to lose
        frames to, so the whole queue goes at once — a config snapshot that took
        the better part of a second on the radio lands in one tick.

        Without WiFi, an ordinary bulk frame is DROPPED rather than sent, which
        is the point of all this: the radio carries driving and telemetry, and a
        2.9 KB config snapshot is half a second of the shared channel spent on
        something nobody is being hurt by waiting for. Dropping is safe because
        every request that produces one of these can only have arrived over the
        same link — the base station won't ask over the radio (see
        basestation/app.py dispatch_config), so the only way to get here is a
        link that broke mid-transfer, and the operator's page re-asks.

        The radio path stays for `radio_ok` frames — the acknowledgement of a
        bootstrap set_config, which by definition has no WiFi to go back over.
        A `False` from send_bulk means the radio has no airtime this tick, not
        that the frame is gone, so it keeps its place in the queue.
        """
        ip = (
            self.ip_link
            if (self.ip_link is not None and self.ip_link.is_connected())
            else None
        )
        dropped = 0
        while self._outbox:
            msg, radio_ok = self._outbox[0]
            if ip is not None:
                if ip.send(msg):
                    self._outbox.popleft()
                    continue
                ip = None  # link broke mid-drain; the rest follows the rules below
            if radio_ok:
                if not self.link.send_bulk(msg):
                    break  # no airtime this tick — keep it and try the next one
                self._outbox.popleft()
                continue
            self._outbox.popleft()
            dropped += 1
        if dropped:
            print(
                f"[Robot] dropped {dropped} bulk frame(s): no link to "
                f"{self.cfg.comms.base_host or '(no base_host set)'} "
                f"— config and documents do not go over the radio"
            )

    def _set_config(self, msg: dict) -> None:
        requested = msg.get("config") or {}
        # A frame that carries only the bulk link's address is the bootstrap
        # case: it is the one thing the base station may send over the radio,
        # because it is how a rover is told where the WiFi link even is. Its
        # acknowledgement has to go back the same way — there is by definition
        # no WiFi to answer over yet. See tuning.BOOTSTRAP_PATHS.
        bootstrap = tuning.is_bootstrap(requested)
        applied, rejected = tuning.apply(self.cfg, requested)
        if applied:
            self._push_live_config()
        if any(path in tuning.BOOTSTRAP_PATHS for path in applied):
            self._retarget_ip_link()
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
            print(
                f"[Robot] config: {len(applied)} applied"
                + (f", {len(rejected)} rejected" if rejected else "")
                + (f", {len(restart)} need a restart" if restart else "")
            )
        for path, why in rejected.items():
            print(f"[Robot] config rejected {path}: {why}")
        self._queue(
            self._config_frame(
                applied, rejected=rejected, restart=restart, save_error=error
            ),
            radio_ok=bootstrap,
        )

    # --- WiFi ---------------------------------------------------------------

    def _wifi_command(self, msg: dict) -> None:
        """Run one WiFi request on a worker thread and report the result.

        Off-thread without exception. `nmcli device wifi connect` takes seconds
        — association, then a DHCP lease — and a scan with `--rescan yes` takes
        a few more. Any of that inside the control loop stops the drivetrain
        being commanded for long enough to trip the teleop failsafe, which on a
        moving rover means it coasts to a halt mid-manoeuvre because somebody
        pressed a button on a settings page.

        One at a time: two overlapping `connect` calls would fight over the same
        interface, and the second answer would describe a state the first one was
        still changing.
        """
        if self._wifi_busy:
            self._queue(
                self._wifi_frame(
                    {"ok": False, "error": "still working on the last WiFi request"}
                ),
                radio_ok=True,
            )
            return
        self._wifi_busy = True
        threading.Thread(
            target=self._wifi_task, args=(msg,), name="wifi", daemon=True
        ).start()

    def _wifi_task(self, msg: dict) -> None:
        mtype = msg.get("type")
        try:
            if mtype == "scan_wifi":
                result = wifi.scan()
                result["networks"] = result.get("networks") or []
            elif mtype == "set_wifi":
                # The country first, when one was given: with no regulatory
                # domain set, a Pi has 5 GHz soft-blocked, so the network the
                # operator is standing next to is simply missing from the scan.
                country_error = None
                if msg.get("country"):
                    country_error = wifi.set_country(str(msg["country"]))
                result = wifi.connect(
                    str(msg.get("ssid") or ""),
                    str(msg.get("psk") or ""),
                    bool(msg.get("hidden")),
                )
                if country_error and result.get("ok"):
                    result["error"] = country_error
            elif mtype == "forget_wifi":
                result = wifi.forget(str(msg.get("ssid") or ""))
                result.update(
                    {
                        k: v
                        for k, v in wifi.status().items()
                        if k in ("ssid", "ip", "signal")
                    }
                )
            else:
                result = wifi.status()
        except Exception as e:  # nothing about a WiFi button may take a robot down
            result = {"ok": False, "error": str(e)}
        finally:
            self._wifi_busy = False
        # radio_ok, and this is the entire point of the feature: a rover that
        # just failed to join a network has no WiFi to answer over, and a rover
        # that succeeded has just changed which one it is on. Either way the
        # answer goes by radio — it is one small frame, asked for by hand.
        self._queue(self._wifi_frame(result), radio_ok=True)
        # A successful join usually means a new address on the base station's
        # network, so re-dial the bulk link rather than waiting for a restart.
        if result.get("ok") and mtype == "set_wifi":
            self._redial_ip_link()

    def _wifi_frame(self, result: dict) -> dict:
        """The reply. Never carries a credential — only what a scanner standing
        next to the rover could see anyway."""
        return {
            "type": "wifi",
            "from": self.cfg.robot_id,
            **{k: v for k, v in result.items() if k != "psk"},
        }

    def _redial_ip_link(self) -> None:
        """Rebuild the bulk link on the same address after the network changed.

        `_retarget_ip_link` returns early when the address is unchanged, which is
        correct for a config edit and wrong here: the address is the same, the
        network underneath it is not, and the old socket is dead.
        """
        old, self.ip_link = self.ip_link, None
        host, port = self.cfg.comms.base_host, self.cfg.comms.base_port

        def swap() -> None:
            if old is not None:
                old.stop()
            if not host:
                return
            link = IPLink(host, port, self._inbox.put, self.cfg.robot_id)
            link.start()
            self.ip_link = link

        threading.Thread(target=swap, name="ip-redial", daemon=True).start()

    def _retarget_ip_link(self) -> None:
        """Re-dial the bulk link after comms.base_host/base_port changed.

        This is what makes the bootstrap worth having. Pointing a rover at a new
        base station over the radio would be pointless if the new address only
        took effect on the next start — you would have to walk out to the rover,
        and at that point you may as well edit robot.env. So the link is torn
        down and re-dialled here, including the case where there was no link at
        all (a fresh install ships with base_host blank).

        On its own thread because IPLink.stop() joins a reader that may be parked
        in a blocking recv: up to a couple of seconds, which inside the control
        loop is long enough to trip the teleop failsafe and stop the drivetrain
        mid-command. The loop keeps ticking; the next _drain_outbox picks up
        whichever link exists by then.
        """
        host, port = self.cfg.comms.base_host, self.cfg.comms.base_port
        old = self.ip_link
        if old is not None and old.host == host and old.port == port:
            return
        self.ip_link = None  # nothing goes out over the old link from here on

        def swap() -> None:
            if old is not None:
                old.stop()
            if not host:
                print("[Robot] bulk link disabled (base_host cleared)")
                return
            link = IPLink(host, port, self._inbox.put, self.cfg.robot_id)
            link.start()
            self.ip_link = link

        threading.Thread(target=swap, name="ip-retarget", daemon=True).start()

    # --- documents (layout, routines) ---------------------------------------

    def _send_fields(self) -> None:
        """Describe the parameters the dashboard's schema.ts cannot know about.

        Only the layout-derived ones: the ~90 static fields are already mirrored
        in TypeScript, and restating them would cost 2 KB of shared airtime for
        something the browser already has.
        """
        fields = tuning.descriptors(self.cfg)
        for frame in split(
            {"fields": fields}, "fields", self.cfg.robot_id, txid=f"F{self._layout_rev}"
        ):
            self._queue(frame)

    def _send_doc(self, mtype: str, doc: dict, rev: int) -> None:
        for frame in split(
            doc, mtype, self.cfg.robot_id, txid=f"{mtype[0].upper()}{rev}", rev=rev
        ):
            self._queue(frame)

    def _receive_doc(self, mtype: str, msg: dict) -> None:
        """Absorb one fragment; act only once the whole document has landed.

        Documents are never partially applied — see comms/doc_transfer.py. Half
        a layout is two motors on one channel, not a smaller layout.
        """
        reassembler = self._rx[mtype]
        doc = reassembler.feed(msg)
        if doc is None:
            if reassembler.error:
                self._queue(self._result_frame(mtype, False, [reassembler.error]))
            return
        save = bool(msg.get("save", True))
        if mtype == "put_layout":
            self._apply_layout(doc, save)
        elif mtype == "put_scripts":
            self._apply_scripts(doc, save)
        else:
            self._apply_routines(doc, save)

    def _result_frame(self, mtype: str, ok: bool, errors, warnings=(), **extra) -> dict:
        return {
            "type": f"{mtype[4:]}_result",
            "from": self.cfg.robot_id,
            "ok": ok,
            "errors": list(errors),
            "warnings": list(warnings),
            **extra,
        }

    def _apply_layout(self, doc: dict, save: bool) -> None:
        """Validate a layout and store it. It does NOT take effect now.

        Actuators are owned by constructors: rebuilding them mid-loop with the
        drivetrain armed is how an ESC ends up holding an undefined pulse. So a
        layout is checked, saved, and reported as needing a restart — the same
        contract every `live=False` tuning field already has.
        """
        result = layout.validate(
            doc,
            {self.cfg.shooter.channel: "the built-in shooter"}
            if self.cfg.shooter.enabled
            else {},
        )
        error = None
        if result.ok and save:
            error = layout.save(doc)
            if error:
                print(f"[Robot] layout valid but NOT saved: {error}")
        if result.ok:
            self._layout_rev += 1
            print(
                f"[Robot] layout accepted (rev {self._layout_rev}); "
                "restart the service to apply it"
            )
        for message in result.errors:
            print(f"[Robot] layout rejected: {message}")
        self._queue(
            self._result_frame(
                "put_layout",
                result.ok,
                result.errors,
                result.warnings,
                rev=self._layout_rev,
                save_error=error,
                restart_required=result.ok,
            )
        )
        if result.ok:
            # Echo the stored document back, the same reason _set_config echoes
            # the values it applied rather than the ones it was asked for: the
            # validator clamps, so what was saved is not necessarily what was
            # sent, and the editor must show the truth. Without this the page
            # would fall back to its last cached copy — which is the version
            # that was just replaced.
            self._send_doc("layout", doc, self._layout_rev)

    def _apply_routines(self, doc: dict, save: bool) -> None:
        """Validate routines and install them. These DO take effect now.

        Unlike a layout, a routine is data the engine reads — there is nothing
        to reconstruct. A document that fails validation is not installed at
        all: the robot keeps running the last set that was good, which is the
        difference between a rejected edit and a rover that stops mid-field.
        """
        result = routine_schema.parse(
            doc, self.cfg.routines, tuple(self.manager.controllers)
        )
        error = None
        if result.ok:
            self._routine_doc = doc
            self._routine_rev += 1
            if self.routine_controller is not None:
                self.routine_controller.set_routines(result.routines)
            if save:
                error = routine_store.save(doc)
                if error:
                    print(f"[Robot] routines applied but NOT saved: {error}")
            print(
                f"[Robot] routines accepted: {len(result.routines)} "
                f"(rev {self._routine_rev})"
            )
        for message in result.errors:
            print(f"[Robot] routines rejected: {message}")
        self._queue(
            self._result_frame(
                "put_routines",
                result.ok,
                result.errors,
                result.warnings,
                rev=self._routine_rev,
                save_error=error,
            )
        )
        if result.ok:
            self._send_doc("routines", doc, self._routine_rev)

    def _apply_scripts(self, doc: dict, save: bool) -> None:
        """Validate scripts and install them. These DO take effect now.

        Like routines and unlike a layout: a script is text the runner compiles,
        so there is nothing to reconstruct. And like routines, a document that
        fails validation is not installed AT ALL — the robot keeps the last set
        that was good, which is the difference between a rejected edit and a
        rover with nothing left to run.

        The validator compiles every script (robot/script/schema.py), so a
        missing colon is refused here, with a line number, rather than at the
        moment somebody presses Run at the field.
        """
        result = script_schema.parse(doc)
        error = None
        if result.ok:
            self._script_doc = doc
            self._script_rev += 1
            if self.script_controller is not None:
                self.script_controller.set_scripts(result.scripts)
            if save:
                error = script_store.save(doc)
                if error:
                    print(f"[Robot] scripts applied but NOT saved: {error}")
            print(
                f"[Robot] scripts accepted: {len(result.scripts)} "
                f"(rev {self._script_rev})"
            )
        for message in result.errors:
            print(f"[Robot] scripts rejected: {message}")
        self._queue(
            self._result_frame(
                "put_scripts",
                result.ok,
                result.errors,
                result.warnings,
                rev=self._script_rev,
                save_error=error,
            )
        )
        if result.ok:
            self._send_doc("scripts", doc, self._script_rev)

    def _load_scripts(self) -> None:
        doc = script_store.load()
        if doc is None:
            return
        result = script_schema.parse(doc)
        if result.ok:
            self._script_doc = doc
            if self.script_controller is not None:
                self.script_controller.set_scripts(result.scripts)
            print(
                f"[Robot] scripts: {len(result.scripts)} loaded from "
                f"{script_store.scripts_path()}"
            )
        else:
            for message in result.errors:
                print(f"[Robot] scripts REJECTED at boot: {message}")

    def _drain_script_output(self, now: float) -> None:
        """Forward console output to the base station over the bulk link.

        NOT over the radio, and that is the whole design of this: a script's
        `print` is kilobytes of text on a channel that carries driving,
        telemetry and the e-stop. It goes over WiFi or it is dropped, exactly
        like a config snapshot — see `_queue`. A dropped console line costs a
        debugging convenience; a dropped telemetry frame costs the operator's
        picture of a moving robot.

        Paced rather than sent per tick, because a script printing in a 50 Hz
        loop would otherwise put fifty frames a second into the outbox.
        """
        script = self.script_controller
        if script is None or now - self._script_out_at < SCRIPT_OUTPUT_PERIOD_S:
            return
        self._script_out_at = now
        lines, watched = script.take_output()
        if not lines and not watched:
            return
        self._queue(
            {
                "type": "script_output",
                "from": self.cfg.robot_id,
                "id": script.selected,
                "lines": lines[-SCRIPT_OUTPUT_LINES:],
                "watch": watched,
            }
        )

    def _load_routines(self) -> None:
        doc = routine_store.load()
        if doc is None:
            return
        result = routine_schema.parse(
            doc, self.cfg.routines, tuple(self.manager.controllers)
        )
        if result.ok:
            self._routine_doc = doc
            if self.routine_controller is not None:
                self.routine_controller.set_routines(result.routines)
            print(
                f"[Robot] routines: {len(result.routines)} loaded from "
                f"{routine_store.routines_path()}"
            )
        else:
            for message in result.errors:
                print(f"[Robot] routines REJECTED at boot: {message}")

    def _set_fpv_wanted(self, msg: dict) -> None:
        """Start or stop the camera feed on the base station's demand.

        Distinct from `fpv.enabled`, which is the operator's setting and says
        whether this rover HAS a feed at all. This says whether anyone is
        looking at it right now, and only the base station knows that — it is
        the end holding the browsers. A rover streamed regardless, so on a
        multi-rover field the unwatched ones were spending the shared Wi-Fi on
        frames that were decoded by nobody.

        Missing `on` is treated as True: an ambiguous frame should leave the
        feed working, not silently switch it off.
        """
        if self.fpv is None:
            return
        if not self.fpv.set_wanted(bool(msg.get("on", True))):
            return  # already in that state; nothing to start or stop
        if self.cfg.fpv.enabled and self.fpv.wanted():
            self.fpv.start()
        else:
            # Not joined — this is the control loop, and waiting out a frame
            # interval to save nothing would stall a tick.
            self.fpv.stop(wait=False)

    def _jog(self, msg: dict) -> None:
        """Move one actuator briefly, for bring-up from the Hardware tab.

        This is a new way to make hardware move, so it gets the same discipline
        as the thing it resembles: it is refused unless the robot is in teleop
        with the drivetrain at rest and no e-stop latched, and every jog carries
        its own expiry so a dropped "stop" can't leave a motor running. Teleop's
        command_timeout is the same idea; this is that idea for a bench.
        """
        name = str(msg.get("mech", ""))
        actuator = msg.get("actuator")
        try:
            power = float(msg.get("power", 0.0))
        except (TypeError, ValueError):
            power = 0.0

        if self.manager.estop:
            print("[Robot] jog refused: e-stop is latched")
            return
        if self.manager.mode != "teleop":
            print(
                f"[Robot] jog refused: only in teleop (mode is {self.manager.mode!r})"
            )
            return

        mech = self._registry.get(name)
        if mech is None:
            print(f"[Robot] jog refused: no mechanism named {name!r}")
            return
        if not hasattr(mech, "set_power"):
            print(f"[Robot] jog refused: {name!r} is not a powered mechanism")
            return
        mech.set_power(power, str(actuator) if actuator else None)
        self._jog_until = time.monotonic() + JOG_TIMEOUT_S if power else 0.0
        self._jog_mech = name if power else ""

    def _mech_preset(self, msg: dict) -> None:
        """Put one mechanism into a named preset, from a bound gamepad button.

        A preset is a whole-mechanism state the layout already carries and the
        routine editor already offers ("intake -> in"); this is the same thing
        reached with a thumb while somebody is driving, so the rules are the
        mechanism's own rather than a second set invented here:

          - Refused while the e-stop is latched. Nothing may start a motor
            through the one button that exists to stop them.
          - Refused in `routine` mode, where a routine is writing mechanisms —
            possibly every tick — and a press would either be undone 20 ms later
            or fight a state machine for an actuator. Switching to teleop is how
            an operator takes a rover back, and that already stops the routine.
          - Allowed in every other mode. Object-align and waypoint drive the
            DRIVETRAIN; an operator lining up on a bucket still owns the intake.

        Deliberately latching, with no expiry of its own. There is no release
        edge to send — `ControllerReader` fires on press — and a preset is a
        state, not a nudge, so "on" stays on until an "off" preset, a mode
        change or the e-stop. A build that wants a mechanism to give up on its
        own says so in the layout (`auto_stop_seconds`), which is one rule for
        every caller instead of a timeout only button presses get.
        """
        name = str(msg.get("mech", ""))
        preset = str(msg.get("preset", ""))

        if self.manager.estop:
            print("[Robot] preset refused: e-stop is latched")
            return
        if self.manager.mode == "routine":
            print(
                f"[Robot] preset refused: a routine is running and owns the "
                f"mechanisms (switch to teleop to take {name!r} back)"
            )
            return

        mech = self._registry.get(name)
        if mech is None:
            print(f"[Robot] preset refused: no mechanism named {name!r}")
            return
        apply_preset = getattr(mech, "apply_preset", None)
        if apply_preset is None:
            print(f"[Robot] preset refused: {name!r} is not a powered mechanism")
            return
        if not apply_preset(preset):
            # Out loud, because this is where a binding typed against one rover
            # and used on another lands, and the base station cannot catch it:
            # it does not know what any rover carries.
            print(f"[Robot] preset refused: {name!r} has no preset {preset!r}")
            return
        # A jog pending on this same mechanism would stop it mid-preset when its
        # 0.4 s failsafe expired — the bench control and the button are two ways
        # of moving one motor, and the last one asked wins.
        if self._jog_mech == name:
            self._jog_mech = ""
        print(f"[Robot] {name} -> {preset}")

    def _toggle_shooter(self, msg: dict) -> None:
        """Spin the flywheel up or down, or pulse a servo launcher.

        Which one happens is decided by `shooter.target_rpm`: above zero this is
        a flywheel and the command toggles it between that speed and stopped;
        at zero it is a servo launcher and this fires one shot. That keeps a
        single gamepad button meaning "work the shooter" on either build.

        Distinct from the `fire` message on purpose. That one belongs to
        ShooterAlignController and carries its whole safety policy — arming,
        dwell, alignment, magazine. This is the manual teleop equivalent and
        claims none of that, so it is gated the way `mech_preset` is: refused
        under a latched e-stop, and refused in `routine` mode, where a routine
        owns the mechanisms and a press would either be undone a tick later or
        fight a state machine for the channel. Firing rules for autonomous shots
        are unchanged and still live in that controller.

        Idempotent when told explicitly (`{"on": true}`) so a repeated frame
        cannot invert the wheel; a bare message toggles from the state the robot
        is actually in, which is what the base station sends — it keeps no
        shadow copy of mechanism state, and one would go stale the first time an
        e-stop stopped the wheel from underneath it.
        """
        shooter = self.shooter
        if shooter is None:
            print(
                "[Robot] shooter_spin refused: no shooter on this robot "
                "(RS_SHOOTER_ENABLED=0)"
            )
            return
        if self.manager.estop:
            print("[Robot] shooter_spin refused: e-stop is latched")
            return
        if self.manager.mode == "routine":
            print(
                "[Robot] shooter_spin refused: a routine is running and owns "
                "the mechanisms (switch to teleop to take the shooter back)"
            )
            return

        if float(getattr(self.cfg.shooter, "target_rpm", 0.0)) <= 0.0:
            shooter.fire()  # servo launcher: one shot, it owns its own cycle
            return

        want = bool(msg["on"]) if "on" in msg else not shooter.spinning
        shooter.spin(want)
        print(
            f"[Robot] shooter flywheel -> "
            f"{f'{self.cfg.shooter.target_rpm:.0f} rpm' if want else 'stop'}"
        )

    def _request_restart(self) -> None:
        """Come back on a fresh process, asked for from the base station.

        A layout only takes effect at start-up — actuators are built in the
        constructor — so every hardware change used to end in an ssh session
        with a rover that was, by then, usually somewhere inconvenient. This is
        that ssh session, over the radio.

        It ends the control loop rather than calling `systemctl` itself. The
        loop's `finally` already parks the motors, the mechanisms and the
        servos, which is the part that matters on a machine with wheels; a
        `systemctl restart` issued from inside the unit would race that cleanup
        against its own SIGTERM. Coming back is the supervisor's job, and
        `run()` reports EXIT_RESTART so it does it.

        REFUSED when nothing is supervising us, because then nothing would
        bring the rover back and "restart" would mean "switch off until someone
        walks over with a laptop". `INVOCATION_ID` is set by systemd for every
        unit it starts and by nothing else, which makes it the honest test for
        "will I be restarted?" — better than checking for a systemctl binary
        that exists on a Pi whether or not this process is a service.
        """
        if not os.environ.get("INVOCATION_ID"):
            print(
                "[Robot] restart refused: not running under systemd, so "
                "nothing would start me again (try `just restart`)"
            )
            return
        print("[Robot] restart requested; stopping cleanly")
        self._restarting = True
        self._running = False

    def _expire_jog(self) -> None:
        if not self._jog_mech:
            return
        if time.monotonic() >= self._jog_until:
            mech = self._registry.get(self._jog_mech)
            if mech is not None:
                mech.stop()
            self._jog_mech = ""

    def _push_live_config(self) -> None:
        """Copy config onto the objects that cached it at construction.

        Most consumers (the motors, the shooter servo, the detector, the FPV
        streamer) read their config dataclass on every use, so mutating the
        config is enough. The ones below took a copy, and this is what makes
        `live=True` in tuning.py true for them. Idempotent and cheap — it runs
        only when a config frame arrives, never in the control loop.
        """
        cfg = self.cfg
        # Re-measured from the dashboard without a restart, which is the whole
        # point of a calibration you get right by parking the rover and reading
        # a number: the next frame uses the value you just typed.
        # The hand-set pair, plus the lens it is a claim about. Learned fits are
        # NOT dropped by this: they are measurements, and the pair is only the
        # fallback for labels that have none.
        self.rangefinder.calibrate(
            cfg.vision.range_at_m, cfg.vision.range_size, hfov_deg=cfg.vision.hfov_deg
        )
        self.rangefinder.learn = cfg.vision.auto_range
        self.rangefinder.prefer_sonar = cfg.vision.sonar_range
        self.rangefinder.min_samples = cfg.vision.range_samples
        # Wheel encoders, for exactly the same reason: counts-per-rev is a
        # number you get right by turning a wheel and typing what you counted,
        # and the speed-matching mode is switched on, watched, and switched off
        # again in the pit. Everything else the loop reads (mode, gains, limits)
        # comes off the shared config object every tick, so this only has to
        # reach what was cached.
        self.drive.recalibrate()
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
                # Not pushed while a routine is running: a routine state BORROWS
                # this same field for its `stop_within_m` and hands it back when
                # the state ends (control/routine_controller.py). Writing it
                # here mid-routine would move a stop distance the routine chose,
                # and the routine would hand back the old value regardless — so
                # the edit lands the moment the rover is the operator's again.
                if self.manager.mode != "routine":
                    c.standoff_m = cfg.vision.standoff_m
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
            if isinstance(c, ObjectAlignController):
                # Live, because `ultrasonic.stop_m` is: an operator who lowers
                # the guard's threshold has just changed how near an approach
                # can finish, and the warning about the two disagreeing should
                # go quiet the moment they stop disagreeing.
                c.set_min_standoff(self._min_standoff())
            if isinstance(c, WaypointController):
                c.arrive_radius_m = cfg.nav.arrive_radius_m
                c.cruise_speed = cfg.nav.cruise_speed
                c.acquire_speed = cfg.nav.acquire_speed
                c.pivot_threshold_deg = cfg.nav.pivot_threshold_deg
                _retune(c.heading_pid, cfg.nav.heading_pid)
                _retune(c.gps_heading_pid, cfg.nav.gps_heading_pid)
        # Safe to assign directly: tuning.py restricts this to the same enum
        # PoseEstimator validates against.
        self.pose_estimator.heading_source = cfg.heading_source
        # The camera feed: whether it runs, and where it goes. Both are settable
        # from the Tuning tab, over the radio, with no service restart — which is
        # the whole point, since the address belongs to whichever laptop is
        # running the base station today and the robot only learns it over the
        # radio. The streamer resolves the host once when it builds its socket,
        # so a new one is handed over rather than read out of cfg; and start() is
        # idempotent and opens the camera itself, so switching the feed on works
        # even on a robot that booted with nothing wanting frames.
        if self.fpv is not None:
            self.fpv.retarget(cfg.fpv.base_host, cfg.fpv.base_port)
            if cfg.fpv.enabled:
                self.fpv.start()
            else:
                # Not joined: this is the control loop, and waiting out a frame
                # interval to save nothing would stall a tick.
                self.fpv.stop(wait=False)
        if self.imu is not None:
            self.imu.heading_offset_deg = cfg.imu.heading_offset_deg
            self.imu.invert = cfg.imu.invert
            self.imu.min_calib = cfg.imu.min_calib
            self.imu.sample_timeout = cfg.imu.sample_timeout

            self.imu.transport = cfg.imu.transport
            self.imu.serial_port = cfg.imu.serial_port
            self.imu.serial_baud = cfg.imu.serial_baud
        if self.gps is not None:
            self.gps.fix_timeout = cfg.gps.fix_timeout
            self.gps.min_move_mps = cfg.gps.min_move_mps
        # The guard itself needs nothing pushed — it holds the config object, so
        # its thresholds are already live. This is the sensor, which took a copy
        # at construction the way the GPS and the IMU do. The pins are not here:
        # they are claimed once, by a constructor, and are `live=False`.
        if self.ultrasonic is not None:
            self.ultrasonic.min_m = cfg.ultrasonic.min_m
            self.ultrasonic.max_m = cfg.ultrasonic.max_m
            self.ultrasonic.interval = cfg.ultrasonic.interval
            self.ultrasonic.samples = cfg.ultrasonic.samples
            self.ultrasonic.max_age = cfg.ultrasonic.max_age

    def _telemetry(self, cmd, detail: bool = True) -> dict:
        """One status frame for the base station.

        `detail=False` leaves out the slow tier — GPS fix health, the vision
        summary, mechanism states, IMU calibration — and lists what it left out
        under `keep`. See RobotConfig.telemetry_detail_hz for why: the frame had
        grown to ~600 bytes, and the channel it shares with every other rover
        does not grow when one joins.

        `keep` rather than a bare omission because these blocks do not all mean
        the same thing by their absence. A missing `imu_calib` means the IMU has
        stopped answering and the pips must come off the screen; a missing `mech`
        means the layout has no mechanisms. Neither may be confused with "this
        did not change since the last frame", which is all `keep` says.
        """
        t = {
            "type": "telemetry",
            "from": self.cfg.robot_id,
            "mode": self.manager.mode,
            "estop": self.manager.estop,
            "left": round(cmd.left, 3),
            "right": round(cmd.right, 3),
        }
        held: list = []
        # What the wheels ACTUALLY did with those two numbers, which is the
        # whole reason encoders exist: `left`/`right` above are what the robot
        # was told, and on real hardware they are not the same thing. Absent
        # entirely on a build with no encoders wired, so it costs those builds
        # nothing. See robot/drive/drivetrain.py::status.
        encoders = self.drive.status()
        if encoders is not None:
            t["enc"] = encoders
        if self.pose_provider is not None:
            pose = self.pose_provider()
            if pose is not None:
                t["lat"], t["lon"], t["heading"] = pose
        # Fix health (quality, satellites, HDOP, speed, track angle + its age).
        # The lat/lon above says where the robot thinks it is; this says whether
        # to believe it, which is the difference between "the GPS is broken" and
        # "it has 3 satellites under a tree".
        if self.gps is not None:
            if detail:
                t["gps"] = self.gps.telemetry()
            else:
                held.append("gps")
        # Surface IMU calibration so the base station can tell whether the
        # heading is trustworthy or still falling back to the GPS track angle.
        #
        # None once the sensor has gone quiet, rather than the last level it
        # reported. A calibration reading is a claim about a heading, and there
        # is no current heading to make it about — three pips beside a stale
        # bearing is the dashboard being reassuring about a sensor that stopped
        # answering, which is the exact failure sample_timeout exists to catch.
        if self.imu is not None:
            if detail:
                t["imu_calib"] = self.imu.calibration() if self.imu.fresh() else None
            else:
                held.append("imu_calib")
        # Vision summary (target, error, size, fps) so the base station can see
        # what the model sees — this is what makes standoff tunable in the field.
        # A summary, never boxes or frames: the radio is 57600 baud and shared.
        if self.detector is not None and not detail:
            held.append("vision")
        elif self.detector is not None:
            t["vision"] = self.detector.telemetry()
            # Estimated metres to the target, alongside the box height it was
            # derived from. Both, deliberately: `size` is what the model actually
            # measured and `dist` is a guess built on one calibration pair, so
            # showing them together is what lets someone standing next to the
            # rover with a tape measure see the guess drift — and collect the
            # pairs a fitted model would need. ~8 bytes a frame.
            detection = self.detector.detection()
            distance = self.rangefinder.distance_for(detection)
            if distance is not None:
                t["vision"]["dist"] = round(distance, 2)
                # WHICH sensor said so, and how many samples the label's fit is
                # standing on. Both are a few bytes and neither can be inferred
                # from the distance: "0.80 m" measured by a transducer and
                # "0.80 m" divided out of a constant somebody typed are
                # different claims, and an operator deciding whether to believe
                # one needs to know which they are looking at.
                t["vision"].update(
                    self.rangefinder.status(
                        detection.label if detection is not None else ""
                    )
                )
        # Distance to whatever is straight ahead, and what the collision guard
        # is doing about it. Only on a build that has an ultrasonic, and tiny —
        # a rounded distance and one short state word. `state` is the half that
        # cannot be inferred from the distance: an operator whose rover has just
        # stopped responding to forward needs to see WHY on the same frame.
        if self.ultrasonic is not None:
            t["sonar"] = self.collision.status(self.ultrasonic.telemetry())
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
            if detail:
                t["mech"] = {name: m.status() for name, m in self.mechanisms.items()}
            else:
                held.append("mech")
        # Which state the FSM is in, so the Routines tab can highlight the live
        # card. Rides the hot path because that is the whole point of it — a
        # state highlight that lags by a second is worse than none.
        if isinstance(active, RoutineController):
            t["routine"] = active.status()
        # Whether the operator's script is still going, and why it stopped if
        # it isn't. A handful of bytes on the hot frame, deliberately: the
        # console it printed rides the bulk link and may not arrive at all, so
        # this is the half that has to survive a rover on the far side of a
        # field with no WiFi.
        if isinstance(active, ScriptController):
            t["script"] = active.status()
        # Whatever closed loops the active mode is running: setpoint, error,
        # output and the P/I/D split, so a gain can be tuned against a picture
        # of what it did rather than by watching the rover and guessing.
        #
        # Only the ACTIVE controller, only loops it is actually stepping, and
        # only while `nav.pid_trace` is on. It is off by default: this is ~60
        # bytes per loop per frame on a radio shared with driving, and a graph
        # nobody is looking at is not worth the airtime a graph costs.
        if self.cfg.nav.pid_trace:
            traces = active.pid_traces() if active is not None else {}
            # The wheel-speed loop rides the same switch, but it is NOT the
            # active controller's — it lives under the drivetrain and runs in
            # every mode, which is exactly why it needs a graph of its own: the
            # gains are the only ones here you cannot tune by watching, because
            # what they act on is invisible from outside the rover.
            trim = getattr(self.drive, "trim", None)
            trace = trim.trace() if trim is not None else None
            if trace is not None:
                traces = {**traces, "drive.trim.pid": trace}
            if traces:
                t["pid"] = traces
        # Only when something was actually withheld: on a build with no GPS, no
        # detector and no mechanisms there is nothing to say, and an empty list
        # would be bytes spent saying it five times a second.
        if held:
            t["keep"] = held
        return t

    def start(self) -> None:
        print("[Robot] arming ESCs (holding neutral)...")
        self.drive.arm()
        # After the drivetrain, which is what actually holds the ESCs at neutral
        # for arm_seconds: a mechanism's encoders want claiming against the same
        # known standstill, and none of them can be turning before this point.
        # Layout mechanisms only — the built-in launcher is not one of these
        # objects (it keeps its own ShooterConfig) and has no GPIO to claim.
        for mech in self.mechanisms.values():
            mech.start()
        print(
            f"[Robot] opening XBee link on {self.cfg.comms.port} @ {self.cfg.comms.baud}..."
        )
        self.link.start()
        # Opportunistic, and started after the radio on purpose: the radio is the
        # link the robot cannot run without, this one only makes bulk transfers
        # cheaper. It retries in the background, so a base station that isn't up
        # yet costs nothing here.
        if self.ip_link is not None:
            self.ip_link.start()
        if self.gps is not None:
            self.gps.start()
        if self.imu is not None:
            self.imu.start()
        # After the ESCs are armed, deliberately: claiming the pins takes a
        # moment and a failure here must not delay the drivetrain reaching
        # neutral. A sensor that won't start says why and stays inert.
        if self.ultrasonic is not None:
            self.ultrasonic.start()
        # Camera before its consumers (detector, FPV) so frames are flowing when
        # they start; the detector is heaviest (it spawns the .eim subprocess, or
        # waits on the sensor's network upload). Only opened for a consumer that
        # actually wants frames — FPV starts it itself if it is switched on, now
        # or later, so a robot with no detector and no feed leaves the device shut.
        if self.camera is not None and self._detector_wants_frames:
            self.camera.start()
        if self.detector is not None:
            self.detector.start()
        if self.fpv is not None:
            self.fpv.start()
        self._running = True

    def run(self) -> int:
        """Drive until told to stop. Returns the process's exit status.

        Zero for a signal or a plain stop; EXIT_RESTART when the base station
        asked to be restarted, which is what makes the supervisor start a fresh
        process instead of leaving the rover switched off.
        """
        self.start()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        last = time.monotonic()
        print(
            f"[Robot] running at {self.cfg.loop_hz:.0f} Hz, start mode '{self.manager.mode}'"
        )
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
                self._expire_jog()
                t1 = time.monotonic()
                cmd = self.manager.update(dt)
                # Between the controller and the drivetrain, on purpose: "do
                # not drive into that" is not a belief any one mode should have
                # to hold, and putting it here gives it to teleop, the autonomy
                # modes and every routine at once. Untouched command on a build
                # with no ultrasonic. See control/collision.py.
                cmd = self.collision.apply(cmd)
                # What the drivetrain is ACTUALLY about to be given, after the
                # guard. Kept for the script API's `rover.commanded`, which is
                # the only way a script can tell that its own full-throttle
                # request was cut back by something in front of the rover.
                self._last_command = (cmd.left, cmd.right)
                # Unconditional, and deliberately outside the controller: a mode
                # switch or an e-stop mid-shot stops update() from being called,
                # and the servo must still retract instead of stalling against
                # its stop. See robot/drive/shooter.py. The same argument applies
                # to every mechanism, so they are all ticked here.
                for mech in self._all_mechanisms().values():
                    mech.update()
                t2 = time.monotonic()
                self.drive.drive(cmd.left, cmd.right)
                # After the motors, never before: this is bookkeeping about how
                # far away things are, and nothing about it belongs in the path
                # between a command and the wheels.
                self._learn_range(cmd)
                t3 = time.monotonic()

                if (
                    self.cfg.telemetry_hz > 0
                    and (now - self._last_telem) >= 1.0 / self.cfg.telemetry_hz
                ):
                    self._last_telem = now
                    # The slow tier rides along at telemetry_detail_hz, not on
                    # every frame. Its own timer rather than a frame counter, so
                    # changing telemetry_hz from the dashboard doesn't silently
                    # change how often the diagnostics update too.
                    detail_period = 1.0 / max(self.cfg.telemetry_detail_hz, 0.01)
                    detail = (now - self._last_detail) >= detail_period
                    if detail:
                        self._last_detail = now
                    self.link.send(self._telemetry(cmd, detail=detail))
                self._drain_script_output(now)
                self._drain_outbox()
                t4 = time.monotonic()

                # Watchdog: a healthy tick is ~a few ms. If one blocks (I2C/servo
                # glitch, serial stall), log which phase stalled so a freeze shows
                # up in the journal instead of being silent.
                work = t4 - now
                if work > SLOW_TICK_S:
                    print(
                        f"[Robot] slow tick {work * 1e3:.0f}ms "
                        f"(inbox={(t1 - now) * 1e3:.0f} update={(t2 - t1) * 1e3:.0f} "
                        f"drive={(t3 - t2) * 1e3:.0f} send={(t4 - t3) * 1e3:.0f})"
                    )

                sleep_for = period - (time.monotonic() - now)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            self.shutdown()
        # After shutdown, never instead of it: the motors coming to rest is not
        # something an exit status gets to skip.
        return EXIT_RESTART if self._restarting else 0

    def _on_signal(self, *_):
        self._running = False

    def shutdown(self) -> None:
        print("\n[Robot] shutting down; stopping motors")
        self.drive.stop()
        # Then hand the encoder GPIO back. After stop(), never instead of it:
        # releasing the pins is housekeeping, and the motors coming to rest is
        # the only part of this that matters if the next line raises.
        self.drive.shutdown()
        # Park every mechanism at rest before anything else winds down: leaving
        # a launcher at the fire angle stalls the servo and leaves the mechanism
        # cocked, and leaving an intake powered is worse.
        for mech in self._all_mechanisms().values():
            mech.stop()
        # Then their encoder pins, after the motors are at rest and after
        # drive.shutdown() above — which owns closing the shared GPIO backend,
        # so this only hands back what each mechanism claimed.
        for mech in self.mechanisms.values():
            mech.shutdown()
        # Stop the frame consumers, then the camera they read from. The detector
        # owns a subprocess, so stopping it early also halts inference promptly.
        if self.fpv is not None:
            self.fpv.stop()
        if self.detector is not None:
            self.detector.stop()
        if self.camera is not None:
            self.camera.stop()
        self.link.stop()
        if self.ip_link is not None:
            self.ip_link.stop()
        if self.gps is not None:
            self.gps.stop()
        if self.imu is not None:
            self.imu.stop()
        if self.ultrasonic is not None:
            self.ultrasonic.stop()
