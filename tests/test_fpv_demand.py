"""Cameras run because somebody is looking, not because a rover booted.

FPVStreamer used to start the moment `fpv.enabled` was set and stream for the
rest of the match. The dashboard shows one feed at a time, so on a three-rover
field that was two unwatched 640x480 streams — several Mbit/s each of unpaced
UDP — sharing a channel with the rover actually being driven, the config link
and the dashboard's own socket. The frames were encoded, transmitted and thrown
away.

So the base station counts open MJPEG streams and tells each rover whether it
has any. Two things about that are load-bearing and are what these tests pin:

  - The gate DEFAULTS OPEN on the rover. A rover that has never heard from the
    base station behaves exactly as it always did. This gate can only ever be
    closed by an explicit instruction, so a lost command costs bandwidth — never
    the feed.
  - The instruction goes over the RADIO. "Stop streaming" is needed most when
    the WiFi is drowning in streams, and sending it over that same link would
    make it least likely to arrive precisely then.
"""

import asyncio
import contextlib
import time

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager
from robot.config import FPVConfig
from robot.sensors.fpv import FPVStreamer

fastapi_testclient = pytest.importorskip("fastapi.testclient")
TestClient = fastapi_testclient.TestClient


class FakeRadio:
    """An XBee-shaped link. `send_bulk` is what tells the app there is a real
    radio to protect rather than the in-process simulator."""

    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def send_bulk(self, msg):
        return True

    def start(self):
        pass

    def stop(self):
        pass


class FakeVideo:
    """Stands in for VideoReceiver: its presence is what puts the base station
    in charge of rover cameras at all."""

    def __init__(self, live=()):
        self.live = list(live)

    def robots(self):
        return self.live

    def latest(self, robot_id):
        return None

    def start(self):
        pass

    def stop(self):
        pass


def rig(robots=("rover1", "rover2"), video=True):
    fleet = FleetManager()
    for rid in robots:
        fleet.update_from_telemetry(
            {"type": "telemetry", "from": rid, "mode": "teleop"}, time.monotonic())
    radio = FakeRadio()
    app = build_app(fleet, radio, None, {"tiles": None},
                    video_rx=FakeVideo() if video else None)
    return app, radio


def _fpv_frames(radio):
    return [m for m in radio.sent if m.get("type") == "fpv"]


# --- the rover's half --------------------------------------------------------

class _Camera:
    def start(self):
        pass

    def frame_and_stamp(self):
        return None, 0.0


def _streamer(enabled=True):
    cfg = FPVConfig()
    cfg.enabled = enabled
    return FPVStreamer(cfg, _Camera(), "rover1")


def test_a_rover_that_has_heard_nothing_still_streams():
    """The default that keeps a lost command from costing the feed."""
    assert _streamer().wanted() is True


def test_closing_the_gate_stops_start_from_streaming():
    fpv = _streamer()
    assert fpv.set_wanted(False) is True
    fpv.start()
    assert fpv._thread is None, "nobody is watching; no sender thread"


def test_opening_it_again_lets_the_feed_start():
    fpv = _streamer()
    fpv.set_wanted(False)
    fpv.start()
    fpv.set_wanted(True)
    fpv.start()
    assert fpv._thread is not None
    fpv.stop()


def test_setting_the_same_state_twice_reports_no_change():
    """What lets the robot skip a start/stop it does not need."""
    fpv = _streamer()
    assert fpv.set_wanted(False) is True
    assert fpv.set_wanted(False) is False


def test_the_gate_does_not_override_the_operator_switching_the_feed_off():
    """`fpv.enabled` is the operator's setting and still wins."""
    fpv = _streamer(enabled=False)
    fpv.set_wanted(True)
    fpv.start()
    assert fpv._thread is None


# --- the base station's half -------------------------------------------------

def test_rovers_are_told_to_stop_when_nobody_is_watching():
    app, radio = rig()
    with TestClient(app):
        for _ in range(50):
            if len(_fpv_frames(radio)) >= 2:
                break
            time.sleep(0.01)
    told = {m["to"]: m["on"] for m in _fpv_frames(radio)}
    assert told == {"rover1": False, "rover2": False}


def test_the_instruction_goes_over_the_radio_not_wifi():
    """See the module docstring: the WiFi is the thing being rescued."""
    app, radio = rig()
    with TestClient(app):
        for _ in range(50):
            if _fpv_frames(radio):
                break
            time.sleep(0.01)
    assert _fpv_frames(radio), "nothing reached the radio"


def test_a_base_station_with_no_video_receiver_commands_nothing():
    """`--no-video` does not even bind the UDP port, so there is nothing to
    switch a camera on for, and `fpv.enabled` is left to mean what it says."""
    app, radio = rig(video=False)
    with TestClient(app):
        time.sleep(0.15)
    assert _fpv_frames(radio) == []


def test_an_open_stream_counts_as_a_viewer_and_a_closed_one_stops_counting():
    """The MJPEG response body is driven directly rather than over HTTP: it is
    an endless stream by design, so a client that fetched it would never see the
    end of it. What matters here is that the count is taken when the body starts
    and given back when it is closed — the response outlives the handler, so the
    body is the only place that can be true."""
    app, _radio = rig()
    route = next(r for r in app.routes
                 if getattr(r, "path", None) == "/video/{robot_id}.mjpg")

    async def scenario():
        response = await route.endpoint("rover1")
        body = response.body_iterator
        # An async generator does nothing until it is first pulled from; that
        # first pull is the browser actually starting to receive.
        pull = asyncio.ensure_future(body.__anext__())
        await asyncio.sleep(0.05)
        assert app.state.video_viewers.get("rover1") == 1
        # Cancelling the pull is what a browser closing the tab amounts to: the
        # generator is torn down at its await and its finally clause runs.
        pull.cancel()
        with contextlib.suppress(asyncio.CancelledError, StopAsyncIteration):
            await pull
        await body.aclose()  # a no-op if the cancel already finished it
        assert "rover1" not in app.state.video_viewers

    asyncio.run(scenario())


def test_a_watched_rover_is_asked_to_stream():
    """The join that matters: an <img src=/video/rover1.mjpg> in a browser is
    what turns rover1's camera on — and only rover1's."""
    app, radio = rig()
    with TestClient(app):
        app.state.video_viewers["rover1"] = 1
        for _ in range(100):
            if any(m["to"] == "rover1" and m["on"] for m in _fpv_frames(radio)):
                break
            time.sleep(0.01)
    assert [m for m in _fpv_frames(radio) if m["to"] == "rover1" and m["on"]], \
        "the open stream did not reach the rover"
    assert not [m for m in _fpv_frames(radio) if m["to"] == "rover2" and m["on"]], \
        "a rover nobody is watching was asked to stream"
