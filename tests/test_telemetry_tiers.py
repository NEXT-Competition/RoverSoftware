"""The two speeds inside one telemetry frame, and the `keep` list that makes
holding a block back different from saying it is gone.

A telemetry frame had grown to ~600 bytes. At telemetry_hz=5 that is one rover
using about two thirds of a 57600-baud channel, and the channel is SHARED — so a
second rover oversubscribes it and drive commands start queueing behind status
updates. That is what "the base station lags once more than one rover connects"
is, on the radio side.

The fix is that not every reading needs restating five times a second. GPS fix
health, the vision summary, mechanism states and the IMU calibration level ride
at telemetry_detail_hz instead. The readings an operator acts on immediately —
sonar distance, encoder RPM, the shooter's arm latch, the routine's state — do
not move tier, because a stale one of those is a dashboard lying about something
that is moving right now.

The subtle half is `keep`. For several of these blocks an ABSENCE already means
something: no `imu_calib` means the IMU stopped answering and the calibration
pips must come off the screen. "Unchanged" and "gone" cannot be the same signal,
so a withheld block is named rather than merely left out.
"""

import time

import pytest

from basestation.fleet import FleetManager
from robot.comms import protocol
from robot.config import RobotConfig
from robot.control.commands import DriveCommand
from robot.robot import Robot


# --- the base station's half -------------------------------------------------

def _telem(**extra):
    return {"type": "telemetry", "from": "rover1", "mode": "teleop", **extra}


def test_a_kept_block_holds_its_last_value():
    fleet = FleetManager()
    fleet.update_from_telemetry(
        _telem(gps={"fix": 1, "sats": 9}, vision={"ok": True, "label": "ball"},
               mech={"arm": {"v": -30.0}}, imu_calib=3), 1.0)
    fleet.update_from_telemetry(_telem(keep=["gps", "vision", "mech", "imu_calib"]), 2.0)

    robot = fleet.snapshot(2.0)["robots"][0]
    assert robot["gps"] == {"fix": 1, "sats": 9}
    assert robot["vision"] == {"ok": True, "label": "ball"}
    assert robot["mech"] == {"arm": {"v": -30.0}}
    assert robot["imu_calib"] == 3


def test_an_absent_block_that_is_NOT_kept_still_clears():
    """The property the `keep` list exists to protect. A rover whose IMU has
    stopped answering sends no imu_calib and does not name it — and the pips
    must come off the screen, because they are a claim about a heading there is
    no longer any reason to believe."""
    fleet = FleetManager()
    fleet.update_from_telemetry(_telem(imu_calib=3, mech={"arm": {"v": 0.0}}), 1.0)
    fleet.update_from_telemetry(_telem(), 2.0)

    robot = fleet.snapshot(2.0)["robots"][0]
    assert robot["imu_calib"] is None
    assert robot["mech"] is None


def test_keeping_one_block_does_not_hold_the_others():
    fleet = FleetManager()
    fleet.update_from_telemetry(_telem(gps={"fix": 1}, imu_calib=2), 1.0)
    fleet.update_from_telemetry(_telem(keep=["gps"]), 2.0)

    robot = fleet.snapshot(2.0)["robots"][0]
    assert robot["gps"] == {"fix": 1}, "named, so held"
    assert robot["imu_calib"] is None, "not named, so genuinely gone"


def test_a_robot_that_sends_no_keep_at_all_behaves_exactly_as_before():
    """Backward compatibility with a rover running older code: no `keep` means
    nothing is held, which is the rule this file is a change to."""
    fleet = FleetManager()
    fleet.update_from_telemetry(_telem(gps={"fix": 1}, vision={"ok": True}), 1.0)
    fleet.update_from_telemetry(_telem(), 2.0)

    robot = fleet.snapshot(2.0)["robots"][0]
    assert robot["gps"] == {"fix": 1}, "gps is sticky and always was"
    assert robot["vision"] == {"ok": True}, "so is vision"


def test_the_blocks_that_never_move_tier_still_clear_on_absence():
    """sonar, shooter, routine and script are not on the slow tier, so nothing
    about this change may make a stale one of them stick."""
    fleet = FleetManager()
    fleet.update_from_telemetry(
        _telem(sonar={"d": 0.4, "state": "blocked"}, shooter={"armed": True},
               routine={"state": "aim"}, script={"running": True},
               keep=["gps"]), 1.0)
    fleet.update_from_telemetry(_telem(keep=["gps"]), 2.0)

    robot = fleet.snapshot(2.0)["robots"][0]
    assert robot["sonar"] is None
    assert robot["shooter"] is None, "a stale ARMED is the dangerous one"
    assert robot["routine"] is None
    assert robot["script"] is None


# --- the size of it ----------------------------------------------------------

def test_withholding_the_slow_tier_is_worth_the_complexity():
    """If this ever stops being a big saving, the `keep` protocol is not paying
    for itself and should go."""
    fast = {"type": "telemetry", "from": "rover1", "mode": "teleop",
            "estop": False, "left": 0.512, "right": 0.498,
            "lat": 37.774912, "lon": -122.419412, "heading": 91.5,
            "enc": {"rpm": {"left": 112.0, "right": 109.4}, "mode": "match",
                    "tl": 0.51, "tr": 0.49, "fault": False},
            "sonar": {"d": 1.42, "state": "clear", "mute": False, "off": False}}
    detailed = dict(fast, gps={"fix": 1, "sats": 9, "speed": 1.24, "hdop": 0.9,
                               "alt": 12.3, "track": 91.2, "track_age": 0.2},
                    imu_calib=3,
                    vision={"ok": True, "fps": 12.4, "label": "ball", "conf": 0.81,
                            "ex": 0.02, "size": 0.11, "age": 0.05, "dist": 0.8},
                    mech={"intake": {"kind": "roller", "v": 0.0},
                          "arm": {"kind": "servo", "v": -30.0}})
    thin = dict(fast, keep=["gps", "imu_calib", "vision", "mech"])

    saved = 1 - len(protocol.encode(thin)) / len(protocol.encode(detailed))
    assert saved > 0.35, f"only {saved:.0%} off the frame"


# --- the robot's half --------------------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    """A Robot with every device mocked off, as tests/test_robot_config.py does."""
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.imu.enabled = False
    cfg.camera.enabled = False
    cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    bot = Robot(cfg)
    bot.link.send = lambda msg: True
    return bot


class _Stub:
    """Whichever one-method sensor the block under test comes from."""

    def __init__(self, value):
        self.value = value

    def telemetry(self):
        return self.value

    def calibration(self):
        return self.value

    def fresh(self):
        return True

    def detection(self):
        return None  # nothing detected, so the rangefinder adds nothing


def test_a_full_frame_carries_every_block(rover):
    rover.gps = _Stub({"fix": 1, "sats": 9})
    rover.imu = _Stub(3)
    rover.detector = _Stub({"ok": True, "label": "ball"})

    frame = rover._telemetry(DriveCommand.stopped(), detail=True)
    assert frame["gps"] == {"fix": 1, "sats": 9}
    assert frame["imu_calib"] == 3
    assert frame["vision"] == {"ok": True, "label": "ball"}
    assert "keep" not in frame, "nothing was withheld, so nothing to name"


def test_a_thin_frame_withholds_those_blocks_and_names_them(rover):
    rover.gps = _Stub({"fix": 1, "sats": 9})
    rover.imu = _Stub(3)
    rover.detector = _Stub({"ok": True, "label": "ball"})

    frame = rover._telemetry(DriveCommand.stopped(), detail=False)
    for block in ("gps", "imu_calib", "vision"):
        assert block not in frame
        assert block in frame["keep"]
    # ...and the frame is still a frame: what the operator drives on is there.
    assert frame["mode"] == rover.manager.mode
    assert frame["estop"] is rover.manager.estop


def test_a_build_with_none_of_those_sensors_sends_no_keep_at_all(rover):
    """An empty list would be bytes spent saying there is nothing to say, five
    times a second, on the channel this whole change is trying to free up."""
    frame = rover._telemetry(DriveCommand.stopped(), detail=False)
    assert "keep" not in frame


def test_detail_defaults_on(rover):
    """Every existing caller — the script API's rover.telemetry, the tests that
    read a frame directly — must keep seeing a complete one."""
    rover.gps = _Stub({"fix": 1})
    assert "gps" in rover._telemetry(DriveCommand.stopped())


def test_the_first_frame_after_boot_carries_everything(rover):
    """_last_detail starts at zero so a base station that has just come up gets
    a full picture immediately rather than a second of blanks."""
    assert rover._last_detail == 0.0
    assert time.monotonic() - rover._last_detail >= 1.0 / rover.cfg.telemetry_detail_hz
