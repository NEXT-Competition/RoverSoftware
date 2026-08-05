"""One rover at a time gets the drive stream.

Drive is the only traffic the base station STREAMS. It repeats at drive_hz while
a stick moves and keeps repeating at the keepalive while one is held perfectly
still, because teleop's command_timeout deliberately stops a rover that stops
hearing from us. Everything else — mode, e-stop, jogs, routine and script
commands, configuration — goes out when somebody presses something and is silent
otherwise.

On a shared radio channel a second stream is a second rover's worth of airtime
taken from the rover somebody is actually driving. So the stream goes to exactly
one: the rover selected in the dashboard, and only while it is in teleop.

Withholding drive frames looks like it ought to be dangerous, and these tests
are mostly about why it isn't:

  - e-stop and mode are handled by ControlManager itself and never reach a
    controller, so nothing here can gate them;
  - a rover in an autonomous mode ignores `drive` outright and its
    command_timeout is not even running, so losing the stream does nothing to
    it at all;
  - a rover in teleop that loses the stream stops, which is the entire purpose
    of command_timeout and what should happen to a rover nobody is driving.
"""

import time

import pytest

from basestation.app import build_app
from basestation.fleet import FleetManager


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)

    def start(self):
        pass

    def stop(self):
        pass


def rig(modes=None, selected="rover1"):
    """Two rovers, both in teleop unless `modes` says otherwise."""
    modes = modes or {}
    fleet = FleetManager()
    now = time.monotonic()
    for rid in ("rover1", "rover2"):
        fleet.update_from_telemetry(
            {"type": "telemetry", "robot_id": rid, "mode": modes.get(rid, "teleop")},
            now)
    fleet.select(selected)
    link = FakeLink()
    app = build_app(fleet, link=link, controller=None, web_cfg={"tiles": None})
    return app, link, fleet


def drives(link, rid=None):
    return [m for m in link.sent
            if m.get("type") == "drive" and (rid is None or m.get("to") == rid)]


def drive(app, rid, throttle=0.5, steer=0.0):
    app.state.handle_action({"action": "drive", "robot_id": rid,
                             "throttle": throttle, "steer": steer})


# --- who gets the stream -----------------------------------------------------

def test_the_selected_rover_in_teleop_is_driven():
    app, link, _ = rig()
    drive(app, "rover1")
    assert drives(link, "rover1") == [
        {"type": "drive", "throttle": 0.5, "steer": 0.0, "to": "rover1"}]


def test_a_rover_that_is_not_selected_is_not_driven():
    """`robot_id` arrives off the wire, so this is the check that stops a client
    driving whatever it names regardless of what the dashboard has selected."""
    app, link, _ = rig()
    drive(app, "rover2")
    assert drives(link, "rover2") == []


def test_the_selected_rover_is_not_driven_outside_teleop():
    """A drive frame to a rover running a routine is ignored by the rover and
    spends airtime the rover being driven needs."""
    app, link, _ = rig(modes={"rover1": "routine"})
    drive(app, "rover1")
    assert drives(link) == []


@pytest.mark.parametrize("mode", ["waypoint", "object_align", "script", "routine"])
def test_no_autonomous_mode_receives_the_stream(mode):
    app, link, _ = rig(modes={"rover1": mode})
    drive(app, "rover1")
    assert drives(link) == []


def test_the_stream_moves_with_the_selection():
    app, link, fleet = rig()
    drive(app, "rover1")
    assert drives(link, "rover1")

    app.state.handle_action({"action": "select", "robot_id": "rover2"})
    drive(app, "rover2")
    assert drives(link, "rover2"), "the newly selected rover drives"
    # Cleared only now, so the release stop the handover sends to rover1 (see
    # test_deselecting_a_driven_rover_stops_it_at_once) is behind us and what
    # follows is purely what the old rover would still be sent.
    link.sent.clear()
    drive(app, "rover1", throttle=0.9)
    assert drives(link, "rover1") == [], "the old one no longer does"


# --- releasing the rover that loses it ---------------------------------------

def test_deselecting_a_driven_rover_stops_it_at_once():
    """command_timeout would halt it within 0.5 s anyway, but that is a failsafe
    and this is a certainty — one frame, and the rover is stopped now."""
    app, link, _ = rig()
    drive(app, "rover1", throttle=1.0)
    link.sent.clear()

    app.state.handle_action({"action": "select", "robot_id": "rover2"})
    drive(app, "rover2")
    assert {"type": "drive", "throttle": 0.0, "steer": 0.0, "to": "rover1"} \
        in link.sent


def test_a_rover_leaving_teleop_is_not_sent_a_pointless_stop():
    """It is now driven by a controller that ignores `drive`, and teleop resets
    itself to stopped whenever it comes back. A stop here would be a frame on a
    shared channel that no rover anywhere acts on."""
    app, link, _ = rig()
    drive(app, "rover1", throttle=1.0)
    link.sent.clear()

    app.state.handle_action({"action": "mode", "robot_id": "rover1",
                             "mode": "waypoint"})
    drive(app, "rover1")
    assert drives(link) == []


# --- what must NOT be gated --------------------------------------------------

@pytest.mark.parametrize("action", [
    {"action": "estop"},
    {"action": "clear_estop"},
    {"action": "mode", "mode": "teleop"},
])
def test_safety_and_mode_reach_a_rover_that_has_no_stream(action):
    """The gate is `drive` alone. An e-stop for a rover nobody has selected is
    exactly the e-stop you cannot afford to drop."""
    app, link, _ = rig(modes={"rover2": "routine"})
    app.state.handle_action({**action, "robot_id": "rover2"})
    assert any(m.get("to") == "rover2" and m.get("type") != "drive"
               for m in link.sent)


def test_a_routine_can_still_be_started_on_an_unselected_rover():
    app, link, _ = rig()
    app.state.handle_action({"action": "routine_cmd", "robot_id": "rover2",
                             "cmd": "start"})
    assert any(m.get("to") == "rover2" and m.get("type") == "routine_cmd"
               for m in link.sent)


# --- the mode we have just asked for -----------------------------------------

def test_pressing_teleop_lets_the_stick_work_immediately():
    """Telemetry is 5 Hz, so believing only telemetry would leave the stick dead
    for up to 200 ms after pressing TELEOP — which is exactly when an operator
    decides the controls are broken."""
    app, link, _ = rig(modes={"rover1": "routine"})
    app.state.handle_action({"action": "mode", "robot_id": "rover1",
                             "mode": "teleop"})
    link.sent.clear()
    drive(app, "rover1")
    assert drives(link, "rover1"), "the commanded mode was not believed"


def test_the_optimism_expires_so_a_refused_mode_does_not_stick(monkeypatch):
    """If the rover REFUSED the mode, telemetry is the truth and we want it back
    quickly rather than streaming at a rover that is not listening."""
    app, link, fleet = rig(modes={"rover1": "routine"})
    app.state.handle_action({"action": "mode", "robot_id": "rover1",
                             "mode": "teleop"})
    # The rover keeps reporting `routine`: it did not take the mode.
    clock = [time.monotonic() + 5.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    fleet.update_from_telemetry(
        {"type": "telemetry", "robot_id": "rover1", "mode": "routine"}, clock[0])
    link.sent.clear()
    drive(app, "rover1")
    assert drives(link) == []
