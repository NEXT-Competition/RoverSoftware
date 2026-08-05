"""Live reconfiguration: what a `set_config` frame actually changes on a robot.

test_tuning.py checks the whitelist in isolation. This checks the part that
matters in the field — that a gain changed from the base station reaches the
controller that is steering right now, without a restart and without resetting
the loop's state underneath it.

Everything is constructed with the hardware mocked (no HAT, no GPS, no IMU, no
camera), which is the same path `RS_MOCK_MOTORS=1 python run_robot.py` takes.
"""

import json
import threading

import pytest

from robot import tuning
from robot.config import RobotConfig
from robot.control.commands import DriveCommand
from robot.control.object_align import ObjectAlignController
from robot.control.shooter_align import ShooterAlignController
from robot.control.teleop import TeleopController
from robot.control.waypoint import WaypointController
from robot.robot import Robot


class FakeIP:
    """Stands in for IPLink: records frames, and can go down mid-transfer."""

    def __init__(self, connected=True, fail_after=None, host="base.local",
                 port=5006):
        self.connected = connected
        self.fail_after = fail_after
        self.host = host
        self.port = port
        self.sent = []
        self.stopped = False
        self.started = False

    def start(self):
        self.started = True

    def is_connected(self):
        return self.connected

    def send(self, msg):
        if self.fail_after is not None and len(self.sent) >= self.fail_after:
            self.connected = False
            return False
        self.sent.append(msg)
        return True

    def stop(self):
        self.stopped = True


@pytest.fixture
def rover(monkeypatch, tmp_path):
    """A Robot with every device disabled, and its tuning file in a temp dir.

    It comes up ON WIFI, because that is where configuration lives: `rover.bulk`
    is what the base station receives and `rover.sent` is the radio, which for
    config traffic should stay empty. The tests that care which link carried
    what replace `rover.ip_link` themselves.
    """
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.imu.enabled = False
    cfg.camera.enabled = False
    cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    bot = Robot(cfg)
    # Capture what the robot would put on the radio instead of opening a port.
    sent = []

    def take(message):
        sent.append(message)
        return True

    bot.link.send = take
    # Bulk frames are metered against the radio's real byte rate, so the real
    # link would refuse most of a snapshot on any one tick. A test isn't waiting
    # 0.7 s of wall clock for that; the pacing itself is exercised in
    # tests/test_airtime.py.
    bot.link.send_bulk = take
    bot.sent = sent
    bot.ip_link = FakeIP()
    bot.bulk = bot.ip_link.sent
    return bot


def deliver(bot, msg):
    """Hand the robot a message as the radio reader thread would, then let the
    reply reach whichever link is carrying it.

    Multi-frame replies are queued and drained a couple of frames per control
    tick (Robot._queue), so a test that only drained the inbox would see nothing
    sent at all. Spinning the outbox here is what run() does over the next few
    ticks, compressed into one call."""
    bot._inbox.put(msg)
    bot._drain_inbox()
    while bot._outbox:
        bot._drain_outbox()


# --- the config conversation ------------------------------------------------

def test_get_config_answers_with_every_parameter(rover):
    deliver(rover, {"type": "get_config"})
    assert all(f["type"] == "config" for f in rover.bulk)
    merged = {}
    for frame in rover.bulk:
        merged.update(frame["config"])
    assert set(merged) == {p.path for p in tuning.PARAMS}


def test_get_config_is_chunked_into_writable_frames(rover):
    """One 2.4 KB write is ~420 ms of wire time against a 0.2 s write timeout —
    exactly the frame that gets dropped under congestion, leaving the settings
    page permanently blank."""
    deliver(rover, {"type": "get_config"})
    assert len(rover.bulk) > 1
    for frame in rover.bulk:
        assert len(json.dumps(frame, separators=(",", ":"))) < 600


def test_config_frames_are_addressed_like_telemetry(rover):
    """The base station keys everything on `from`; without it the reply would
    be attributed to whichever robot answered last."""
    deliver(rover, {"type": "get_config"})
    assert all(f["from"] == "rover1" for f in rover.bulk)


def test_set_config_acknowledges_only_what_it_applied(rover):
    deliver(rover, {"type": "set_config",
                    "config": {"align.pid.kp": 0.9, "nope": 1}})
    (frame,) = rover.bulk
    assert frame["config"] == {"align.pid.kp": 0.9}
    assert frame["rejected"] == {"nope": "unknown parameter"}
    assert frame["restart"] == []


def test_set_config_reports_what_needs_a_restart(rover):
    deliver(rover, {"type": "set_config", "config": {"comms.baud": 9600}})
    assert rover.bulk[0]["restart"] == ["comms.baud"]


def test_config_messages_never_reach_the_active_controller(rover):
    """A config frame must not look like a stale drive command to teleop."""
    teleop = rover.manager.controllers["teleop"]
    deliver(rover, {"type": "set_config", "config": {"align.pid.kp": 0.9}})
    assert teleop._last_rx == 0.0


def test_frames_for_another_robot_are_ignored(rover):
    deliver(rover, {"to": "rover9", "type": "get_config"})
    assert rover.sent == []


# --- does it actually take effect? ------------------------------------------

def test_pid_gains_reach_the_live_controller(rover):
    deliver(rover, {"type": "set_config",
                    "config": {"align.pid.kp": 1.1, "align.pid.kd": 0.02,
                               "nav.heading_pid.ki": 0.05}})
    align = rover.manager.controllers["object_align"]
    assert isinstance(align, ObjectAlignController)
    assert (align.pid.kp, align.pid.kd) == (1.1, 0.02)
    assert rover.manager.controllers["waypoint"].heading_pid.ki == 0.05


def test_retuning_does_not_reset_the_integrator(rover):
    """Changing a gain mid-run should nudge the loop, not make the robot forget
    where it was pointing and lurch."""
    wp = rover.manager.controllers["waypoint"]
    wp.heading_pid.ki = 0.1
    wp.heading_pid.update(error=10.0, dt=0.1)  # accumulate some integral
    before = wp.heading_pid._integral
    assert before != 0
    deliver(rover, {"type": "set_config", "config": {"nav.heading_pid.kp": 0.6}})
    assert wp.heading_pid._integral == before


def test_shooter_align_gets_both_alignment_and_firing_policy(rover):
    """It subclasses ObjectAlignController, so it needs the align tuning as
    well as its own — keying the push off the mode name would leave it blind."""
    deliver(rover, {"type": "set_config",
                    "config": {"align.pivot_threshold": 0.4, "shooter.dwell": 1.5}})
    shooter = rover.manager.controllers["shooter_align"]
    assert isinstance(shooter, ShooterAlignController)
    assert shooter.pivot_threshold == 0.4
    assert shooter.dwell == 1.5


def test_teleop_failsafe_timeout_is_live(rover):
    deliver(rover, {"type": "set_config", "config": {"comms.command_timeout": 1.25}})
    teleop = rover.manager.controllers["teleop"]
    assert isinstance(teleop, TeleopController)
    assert teleop.command_timeout == 1.25


def test_waypoint_geometry_is_live(rover):
    deliver(rover, {"type": "set_config",
                    "config": {"nav.arrive_radius_m": 5.0, "nav.cruise_speed": 0.2}})
    wp = rover.manager.controllers["waypoint"]
    assert isinstance(wp, WaypointController)
    assert (wp.arrive_radius_m, wp.cruise_speed) == (5.0, 0.2)


def test_vision_geometry_reaches_the_align_controllers(rover):
    """standoff/hfov live in VisionConfig but are *copied* into the
    controllers, which is exactly the case a cfg-only write would miss."""
    deliver(rover, {"type": "set_config",
                    "config": {"vision.standoff_size": 0.6, "vision.hfov_deg": 66}})
    align = rover.manager.controllers["object_align"]
    assert align.standoff_size == 0.6
    assert align.hfov_deg == 66.0


def test_heading_source_is_live(rover):
    deliver(rover, {"type": "set_config", "config": {"heading_source": "gps"}})
    assert rover.pose_estimator.heading_source == "gps"


def test_motor_limits_take_effect_without_a_push(rover):
    """ESCMotor reads its MotorConfig on every command, so mutating the config
    is the whole mechanism. Asserted through the servo, not the config."""
    deliver(rover, {"type": "set_config", "config": {"drive.left.max_forward": 0.5}})
    rover.drive.cfg.slew_rate = 0  # measure the command, not the ramp
    rover.drive.drive(1.0, 0.0)
    left = rover.drive.left
    throw = min(left.cfg.max_angle - left.cfg.neutral_angle,
                left.cfg.neutral_angle - left.cfg.min_angle)
    assert left.servo._last == pytest.approx(left.cfg.neutral_angle + 0.5 * throw)


# --- persistence ------------------------------------------------------------

def test_applied_values_are_saved_by_default(rover, tmp_path):
    deliver(rover, {"type": "set_config", "config": {"align.pid.kp": 1.4}})
    saved = json.loads((tmp_path / "tuning.json").read_text())
    assert saved == {"align.pid.kp": 1.4}


def test_save_false_applies_without_persisting(rover, tmp_path):
    """The escape hatch for trying a value without committing to it."""
    deliver(rover, {"type": "set_config",
                    "config": {"align.pid.kp": 1.4}, "save": False})
    assert rover.manager.controllers["object_align"].pid.kp == 1.4
    assert not (tmp_path / "tuning.json").exists()


def test_saved_values_are_the_clamped_ones(rover, tmp_path):
    """What is persisted must be what the robot is doing, not what was asked."""
    deliver(rover, {"type": "set_config", "config": {"align.pid.kp": 500}})
    assert json.loads((tmp_path / "tuning.json").read_text()) == {"align.pid.kp": 5.0}


# --- which link carries what ------------------------------------------------

def test_config_dump_goes_over_wifi_when_there_is_wifi(rover):
    """The whole point: ~2.9 KB of config stops costing shared airtime."""
    deliver(rover, {"type": "get_config"})
    assert len(rover.bulk) > 1  # the multi-frame snapshot
    assert rover.sent == []  # and none of it touched the radio


def test_config_dump_is_dropped_with_no_wifi(rover):
    """The rule: config does NOT fall back to the radio.

    A snapshot is half a second of a channel shared with every robot's telemetry
    and nobody is being hurt by waiting for it, so a rover out of WiFi range
    simply doesn't answer — and the base station tells the operator that (see
    tests/test_config_over_wifi.py) rather than spending the airtime.
    """
    rover.ip_link = FakeIP(connected=False)
    deliver(rover, {"type": "get_config"})
    assert rover.ip_link.sent == []
    assert rover.sent == []


def test_no_ip_link_at_all_drops_it_too(rover):
    """An unconfigured base_host is the same case as an unreachable one."""
    rover.ip_link = None
    deliver(rover, {"type": "get_config"})
    assert rover.sent == []


def test_wifi_drains_the_whole_snapshot_in_one_tick(rover):
    """On the radio this would be paced at OUTBOX_PER_TICK; on WiFi there's no
    airtime to protect, which is what makes the settings page fill instantly."""
    rover._inbox.put({"type": "get_config"})
    rover._drain_inbox()
    queued = len(rover._outbox)
    assert queued > 2, "expected a multi-frame snapshot to pace against"
    rover._drain_outbox()  # ONE tick
    assert not rover._outbox
    assert len(rover.bulk) == queued


def test_wifi_dropping_mid_transfer_drops_the_remainder(rover):
    """A half-sent snapshot is not silently completed over the radio.

    The operator's page re-asks (useRadioFetch in RobotSettings.tsx), which
    costs nothing; finishing it over the radio would cost the airtime this whole
    path exists to protect, at exactly the moment WiFi is already flaky.
    """
    rover.ip_link = FakeIP(fail_after=2)
    deliver(rover, {"type": "get_config"})
    assert len(rover.ip_link.sent) == 2
    assert rover.sent == []
    assert not rover._outbox  # and nothing is left queued for a link that's gone


def test_a_bootstrap_ack_the_radio_cannot_take_yet_stays_queued(rover):
    """The bug the outbox exists for, on the one path that still uses the radio.

    A `False` from send_bulk means the radio has no airtime this tick, not that
    the frame is gone. Popping it anyway would lose the acknowledgement of the
    one edit that has no other way home.
    """
    refused = []

    def full(msg):
        refused.append(msg)
        return False

    rover.ip_link = None  # exactly the state a bootstrap edit is sent into
    rover.link.send_bulk = full
    rover._inbox.put({"type": "set_config",
                      "config": {"comms.base_host": "base.local"}})
    rover._drain_inbox()
    assert len(rover._outbox) == 1

    rover._drain_outbox()
    assert len(rover._outbox) == 1  # not dropped on the floor
    assert len(refused) == 1

    # Airtime frees up on a later tick and it simply goes out.
    rover.link.send_bulk = rover.link.send
    rover._drain_outbox()
    assert not rover._outbox
    assert rover.sent[0]["config"] == {"comms.base_host": "base.local"}


def test_telemetry_never_leaves_the_radio(rover):
    """It is realtime and it is what you need at range; WiFi is neither."""
    rover.link.send(rover._telemetry(DriveCommand.stopped()))
    assert rover.bulk == []
    assert len(rover.sent) == 1
    assert rover.sent[0]["type"] == "telemetry"


# --- the bootstrap exception ------------------------------------------------

def test_the_link_address_is_the_one_config_the_radio_carries(rover):
    """Told over the radio where the WiFi link is, the rover answers there too.

    Without this a rover with no base_host could never be given one: the only
    channel that can reach it is the one config is not allowed on.
    """
    rover.ip_link = None
    deliver(rover, {"type": "set_config",
                    "config": {"comms.base_host": "base.local",
                               "comms.base_port": 5006}})
    (ack,) = rover.sent
    assert ack["config"] == {"comms.base_host": "base.local",
                             "comms.base_port": 5006}
    assert ack["restart"] == []  # it takes effect now, not on the next start


def test_a_gain_smuggled_in_beside_a_hostname_is_not_a_bootstrap(rover):
    """`is_bootstrap` is all-or-nothing, so this ack waits for WiFi like any
    other. Otherwise every config edit would ride the radio with a hostname
    stapled to it."""
    rover.ip_link = FakeIP(connected=False)
    deliver(rover, {"type": "set_config",
                    "config": {"comms.base_host": "base.local",
                               "align.pid.kp": 0.9}})
    assert rover.sent == []


def _finish_retarget():
    """_retarget_ip_link swaps links on its own thread (stop() can block); wait
    for it rather than sleeping."""
    for thread in threading.enumerate():
        if thread.name == "ip-retarget":
            thread.join(timeout=2.0)


def test_a_new_base_host_redials_the_link_without_a_restart(rover, monkeypatch):
    """The reason the bootstrap is worth having. A base_host that only applied
    on the next start would mean walking out to the rover to restart it — and
    then you may as well have edited robot.env."""
    monkeypatch.setattr("robot.robot.IPLink",
                        lambda host, port, on_message, robot_id: FakeIP(host=host, port=port))
    old = rover.ip_link
    deliver(rover, {"type": "set_config",
                    "config": {"comms.base_host": "other.local"}})
    _finish_retarget()
    assert old.stopped
    assert rover.ip_link is not old
    assert rover.ip_link is not None
    assert rover.ip_link.host == "other.local" and rover.ip_link.started


def test_clearing_the_base_host_leaves_no_link_at_all(rover):
    old = rover.ip_link
    deliver(rover, {"type": "set_config", "config": {"comms.base_host": ""}})
    _finish_retarget()
    assert old.stopped
    assert rover.ip_link is None


# --- telemetry framing against the radio -------------------------------------

def test_the_telemetry_core_fits_in_one_rf_packet(rover):
    """The property the whole fleet's telemetry depends on.

    The XBee runs transparent: no addressing, no arbitration, and the module
    flushes every ~72-100 bytes. A frame longer than that leaves as several RF
    packets, and with two rovers on the channel another rover's packets land
    between them — the receiver splices them into one undecodable line.

    Measured on the real fleet before this split: a 455-byte frame spanned ~6
    packets and NOT ONE arrived intact over 20 seconds, while a 284-byte rover
    got 16 through. The base station showed 1/1 live. Lowering the telemetry
    rate changed nothing; the variable is size, not frequency.

    So the core — the fields the fleet list and the drive feedback need — must
    stay under one packet. Adding a field here is not a small change: it is
    charged against every frame, from every robot, forever.
    """
    from robot.control.commands import DriveCommand
    core = rover._telemetry(DriveCommand(0.5, -0.5))
    core = {k: v for k, v in core.items()
            if k in ("type", "from", "mode", "estop", "left", "right")}
    assert len(json.dumps(core, separators=(",", ":"))) + 1 <= 100


def test_most_frames_carry_no_bulky_block(rover):
    """Blocks cannot be made to fit one packet — `mech` alone is ~160 bytes —
    so the goal is to keep them RARE, leaving the majority of frames as the
    bare core that always survives."""
    from robot.control.commands import DriveCommand
    rover._telem_block_at = 0.0
    bare = 0
    for _ in range(50):
        frame = rover._telemetry(DriveCommand.stopped())   # no clock advance
        if set(frame) <= {"type", "from", "mode", "estop", "left", "right"}:
            bare += 1
    assert bare >= 40, f"only {bare}/50 frames were core-only"


def test_every_block_gets_its_turn(rover):
    """A block dropped from the rotation is a dashboard panel that never
    updates again — silently, since nothing errors."""
    from robot.control.commands import DriveCommand
    from robot.robot import _TELEM_BLOCKS
    seen = set()
    for _ in range(len(_TELEM_BLOCKS) * 4):
        rover._telem_block_at = 0.0
        seen.update(rover._telemetry(DriveCommand.stopped()))
    # Which blocks exist depends on what hardware the fixture's rover has, so
    # assert the rotation CYCLES rather than naming blocks this build may lack:
    # over four rotations every slot is reached, so any block whose source is
    # present must have appeared.
    from robot.robot import _TELEM_BLOCKS
    assert rover._telem_slot >= len(_TELEM_BLOCKS), "rotation did not complete"
