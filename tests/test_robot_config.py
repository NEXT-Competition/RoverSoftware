"""Live reconfiguration: what a `set_config` frame actually changes on a robot.

test_tuning.py checks the whitelist in isolation. This checks the part that
matters in the field — that a gain changed from the base station reaches the
controller that is steering right now, without a restart and without resetting
the loop's state underneath it.

Everything is constructed with the hardware mocked (no HAT, no GPS, no IMU, no
camera), which is the same path `RS_MOCK_MOTORS=1 python run_robot.py` takes.
"""

import json

import pytest

from robot import tuning
from robot.config import RobotConfig
from robot.control.commands import DriveCommand
from robot.control.object_align import ObjectAlignController
from robot.control.shooter_align import ShooterAlignController
from robot.control.teleop import TeleopController
from robot.control.waypoint import WaypointController
from robot.robot import Robot


@pytest.fixture
def rover(monkeypatch, tmp_path):
    """A Robot with every device disabled, and its tuning file in a temp dir."""
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
    bot.link.send = sent.append
    bot.sent = sent
    return bot


def deliver(bot, msg):
    """Hand the robot a message as the radio reader thread would, then let the
    reply reach the radio.

    Multi-frame replies are queued and drained a couple of frames per control
    tick (Robot._queue), so a test that only drained the inbox would see an
    empty radio. Spinning the outbox here is what run() does over the next few
    ticks, compressed into one call."""
    bot._inbox.put(msg)
    bot._drain_inbox()
    while bot._outbox:
        bot._drain_outbox()


# --- the config conversation ------------------------------------------------

def test_get_config_answers_with_every_parameter(rover):
    deliver(rover, {"type": "get_config"})
    assert all(f["type"] == "config" for f in rover.sent)
    merged = {}
    for frame in rover.sent:
        merged.update(frame["config"])
    assert set(merged) == {p.path for p in tuning.PARAMS}


def test_get_config_is_chunked_into_writable_frames(rover):
    """One 2.4 KB write is ~420 ms of wire time against a 0.2 s write timeout —
    exactly the frame that gets dropped under congestion, leaving the settings
    page permanently blank."""
    deliver(rover, {"type": "get_config"})
    assert len(rover.sent) > 1
    for frame in rover.sent:
        assert len(json.dumps(frame, separators=(",", ":"))) < 600


def test_config_frames_are_addressed_like_telemetry(rover):
    """The base station keys everything on `from`; without it the reply would
    be attributed to whichever robot answered last."""
    deliver(rover, {"type": "get_config"})
    assert all(f["from"] == "rover1" for f in rover.sent)


def test_set_config_acknowledges_only_what_it_applied(rover):
    deliver(rover, {"type": "set_config",
                    "config": {"align.pid.kp": 0.9, "nope": 1}})
    (frame,) = rover.sent
    assert frame["config"] == {"align.pid.kp": 0.9}
    assert frame["rejected"] == {"nope": "unknown parameter"}
    assert frame["restart"] == []


def test_set_config_reports_what_needs_a_restart(rover):
    deliver(rover, {"type": "set_config", "config": {"comms.baud": 9600}})
    assert rover.sent[0]["restart"] == ["comms.baud"]


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

class FakeIP:
    """Stands in for IPLink: records frames, and can go down mid-transfer."""

    def __init__(self, connected=True, fail_after=None):
        self.connected = connected
        self.fail_after = fail_after
        self.sent = []

    def is_connected(self):
        return self.connected

    def send(self, msg):
        if self.fail_after is not None and len(self.sent) >= self.fail_after:
            self.connected = False
            return False
        self.sent.append(msg)
        return True


def test_config_dump_goes_over_wifi_when_there_is_wifi(rover):
    """The whole point: ~2.9 KB of config stops costing shared airtime."""
    rover.ip_link = FakeIP()
    deliver(rover, {"type": "get_config"})
    assert len(rover.ip_link.sent) > 1  # the multi-frame snapshot
    assert rover.sent == []  # and none of it touched the radio


def test_config_dump_uses_the_radio_with_no_wifi(rover):
    """Unchanged behaviour when the base station isn't reachable."""
    rover.ip_link = FakeIP(connected=False)
    deliver(rover, {"type": "get_config"})
    assert rover.ip_link.sent == []
    assert len(rover.sent) > 1


def test_no_ip_link_at_all_is_the_old_path(rover):
    assert rover.ip_link is None  # not configured by default
    deliver(rover, {"type": "get_config"})
    assert len(rover.sent) > 1


def test_wifi_drains_the_whole_snapshot_in_one_tick(rover):
    """On the radio this is paced at OUTBOX_PER_TICK; on WiFi there's no airtime
    to protect, which is what makes the settings page fill instantly."""
    rover.ip_link = FakeIP()
    rover._inbox.put({"type": "get_config"})
    rover._drain_inbox()
    queued = len(rover._outbox)
    assert queued > 2, "expected a multi-frame snapshot to pace against"
    rover._drain_outbox()  # ONE tick
    assert not rover._outbox
    assert len(rover.ip_link.sent) == queued


def test_wifi_dropping_mid_transfer_falls_back_to_the_radio(rover):
    """No frame may be lost in the handover — that's what leaves a blank page."""
    rover.ip_link = FakeIP(fail_after=2)
    deliver(rover, {"type": "get_config"})
    assert len(rover.ip_link.sent) == 2
    assert rover.sent  # the remainder went out over the radio
    total = len(rover.ip_link.sent) + len(rover.sent)
    params = {k for f in (rover.ip_link.sent + rover.sent) for k in f["config"]}
    assert total > 2 and params == set(tuning.snapshot(rover.cfg))


def test_telemetry_never_leaves_the_radio(rover):
    """It is realtime and it is what you need at range; WiFi is neither."""
    rover.ip_link = FakeIP()
    rover.link.send(rover._telemetry(DriveCommand.stopped()))
    assert rover.ip_link.sent == []
    assert len(rover.sent) == 1
    assert rover.sent[0]["type"] == "telemetry"
