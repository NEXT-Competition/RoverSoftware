"""Pointing the camera feed at a different base station, while it is running.

`fpv.base_host` is the address of whichever laptop is running the base station
today. The robot only ever learns it over the radio, so needing a service
restart to change it means a rover you cannot see out of until someone can SSH
in — which is exactly when you can't.
"""

import threading
import time

import pytest

from robot.config import FPVConfig
from robot.sensors.fpv import FPVStreamer


class FakeCamera:
    """Never produces a frame: this is about the socket and the switch, not JPEG."""

    def __init__(self):
        self.starts = 0

    def start(self):
        self.starts += 1

    def frame_and_stamp(self):
        return None, 0.0


class FakeSender:
    built = []

    def __init__(self, host, port, robot_id, *a, **kw):
        FakeSender.built.append((host, port))
        self.closed = False

    def send_frame(self, jpeg):
        pass

    def close(self):
        self.closed = True


@pytest.fixture
def senders(monkeypatch):
    import robot.comms.video_udp as video_udp

    FakeSender.built = []
    monkeypatch.setattr(video_udp, "VideoSender", FakeSender)
    return FakeSender.built


def _run(streamer) -> threading.Thread:
    streamer._running = True
    thread = threading.Thread(target=streamer._loop, daemon=True)
    thread.start()
    return thread


def _wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_it_starts_out_aimed_at_the_configured_host(senders):
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005, fps=200)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    assert streamer.target() == ("base.local", 5005)
    thread = _run(streamer)
    assert _wait_for(lambda: senders == [("base.local", 5005)])
    streamer.stop()
    thread.join(timeout=1)


def test_retargeting_moves_the_feed_without_a_restart(senders):
    cfg = FPVConfig(enabled=True, base_host="old.local", base_port=5005, fps=200)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    thread = _run(streamer)
    assert _wait_for(lambda: senders)

    assert streamer.retarget("192.168.1.50", 5005) is True
    assert _wait_for(lambda: len(senders) == 2)
    assert senders[1] == ("192.168.1.50", 5005)

    streamer.stop()
    thread.join(timeout=1)


def test_retargeting_to_the_same_place_rebuilds_nothing():
    """A config frame re-sending every field must not churn the socket."""
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    assert streamer.retarget("base.local", 5005) is False


def test_the_port_moves_with_the_host_or_not_at_all():
    """Read as a pair: a half-applied edit must not aim at a new host, old port."""
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    assert streamer.retarget("base.local", 6000) is True
    assert streamer.target() == ("base.local", 6000)


# --- the on/off switch -------------------------------------------------------

def test_switching_it_on_opens_the_camera_and_starts_streaming(senders):
    """A robot that booted with no frame consumer still gets a feed on demand."""
    cfg = FPVConfig(enabled=False, base_host="base.local", base_port=5005, fps=200)
    camera = FakeCamera()
    streamer = FPVStreamer(cfg, camera, "rover1")

    streamer.start()  # boot: switched off, so nothing happens
    assert camera.starts == 0
    assert senders == []

    cfg.enabled = True  # what a config frame off the radio does
    streamer.start()
    assert _wait_for(lambda: senders == [("base.local", 5005)])
    assert camera.starts == 1  # opened by its first consumer, not at boot
    streamer.stop()


def test_switching_it_off_stops_the_feed_without_blocking_the_caller(senders):
    """stop(wait=False) runs on the control loop; it must not join a sleeping thread."""
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005, fps=200)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    streamer.start()
    assert _wait_for(lambda: senders)

    cfg.enabled = False
    streamer.stop(wait=False)
    assert streamer._running is False
    # The loop tidies up after itself — the socket belongs to that thread.
    assert _wait_for(lambda: streamer._sender is None)


def test_it_can_be_switched_back_on(senders):
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005, fps=200)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    streamer.start()
    assert _wait_for(lambda: len(senders) == 1)
    streamer.stop()

    streamer.start()
    assert _wait_for(lambda: len(senders) == 2)
    streamer.stop()


def test_off_and_straight_back_on_does_not_race_two_loops(senders):
    """The old loop picks the flag back up rather than being duplicated."""
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005, fps=200)
    streamer = FPVStreamer(cfg, FakeCamera(), "rover1")
    streamer.start()
    assert _wait_for(lambda: senders)
    first = streamer._thread

    streamer.stop(wait=False)
    streamer.start()  # before the loop has necessarily noticed
    assert streamer._running is True
    assert streamer._thread is first or not first.is_alive()
    streamer.stop()


def test_no_camera_means_no_feed_however_hard_you_switch_it(senders):
    cfg = FPVConfig(enabled=True, base_host="base.local", base_port=5005)
    streamer = FPVStreamer(cfg, None, "rover1")
    streamer.start()
    assert senders == []
    assert streamer._running is False


def test_a_config_frame_off_the_radio_retargets_the_running_feed(monkeypatch, tmp_path):
    """End to end: the Tuning tab sets fpv.base_host, the rover re-aims."""
    from robot import tuning
    from robot.config import RobotConfig

    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.imu.enabled = False
    cfg.camera.enabled = False
    cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0

    from robot.robot import Robot

    bot = Robot(cfg)
    bot.fpv = FPVStreamer(cfg.fpv, FakeCamera(), "rover1")

    applied, rejected = tuning.apply(cfg, {"fpv.base_host": "192.168.4.2",
                                           "fpv.base_port": 5010,
                                           "fpv.enabled": True})
    assert not rejected, rejected
    # These have to be live, or the dashboard tells the operator to restart the
    # service and this whole path is moot.
    assert not tuning.needs_restart(applied, tuning.by_path_for(cfg))

    bot._push_live_config()
    assert bot.fpv.target() == ("192.168.4.2", 5010)
    assert bot.fpv._running is True  # and the feed was switched on

    # ...and off again, from the same place.
    tuning.apply(cfg, {"fpv.enabled": False})
    bot._push_live_config()
    assert bot.fpv._running is False


def test_a_robot_builds_its_streamer_even_with_the_feed_off(monkeypatch, tmp_path):
    """Otherwise `fpv.enabled` is restart-only for no reason but a missing object."""
    from robot.config import RobotConfig
    from robot.robot import Robot

    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    assert cfg.fpv.enabled is False

    bot = Robot(cfg)
    assert bot.fpv is not None
