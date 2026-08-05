"""Script console output on its way from a rover to the editor's log pane.

Output is the one part of this feature that rides the BULK link rather than the
radio, and is dropped rather than queued when there isn't one. That is a
deliberate trade — a script's prints are kilobytes of text on a channel that
carries driving, telemetry and the e-stop — so these tests pin the properties
that make dropping safe: it is bounded, it is self-contained per frame, and
what cannot be dropped (whether the run is still going, and why it stopped)
travels separately on the hot frame instead.
"""

import pytest

from basestation.fleet import SCRIPT_CONSOLE_MAX, FleetManager


def frame(robot_id="rover1", lines=(), watch=None):
    return {"type": "script_output", "from": robot_id, "id": "prog",
            "lines": list(lines), "watch": watch or {}}


def test_output_lands_under_the_robot_that_printed_it():
    fleet = FleetManager()
    fleet.handle(frame(lines=["one", "two"]), now=0.0)
    console = fleet.script_console()
    assert console["rover1"]["lines"] == ["one", "two"]


def test_frames_accumulate_rather_than_replacing():
    """Each frame is a quarter second of output, not a full snapshot — so one
    lost to a WiFi hiccup costs those lines and nothing else."""
    fleet = FleetManager()
    fleet.handle(frame(lines=["one"]), now=0.0)
    fleet.handle(frame(lines=["two"]), now=0.1)
    assert fleet.script_console()["rover1"]["lines"] == ["one", "two"]


def test_watched_values_replace_rather_than_accumulate():
    """The opposite rule, and for the opposite reason: a watched value is a
    live number, so the newest is the only one that is true."""
    fleet = FleetManager()
    fleet.handle(frame(watch={"range": 1.0}), now=0.0)
    fleet.handle(frame(watch={"range": 0.5}), now=0.1)
    assert fleet.script_console()["rover1"]["watch"] == {"range": 0.5}


def test_the_buffer_is_bounded_and_keeps_the_newest():
    fleet = FleetManager()
    for n in range(SCRIPT_CONSOLE_MAX + 50):
        fleet.handle(frame(lines=[f"line {n}"]), now=0.0)
    lines = fleet.script_console()["rover1"]["lines"]
    assert len(lines) == SCRIPT_CONSOLE_MAX
    assert lines[-1] == f"line {SCRIPT_CONSOLE_MAX + 49}"


def test_each_robot_keeps_its_own():
    fleet = FleetManager()
    fleet.handle(frame("rover1", ["a"]), now=0.0)
    fleet.handle(frame("rover2", ["b"]), now=0.0)
    console = fleet.script_console()
    assert console["rover1"]["lines"] == ["a"]
    assert console["rover2"]["lines"] == ["b"]


def test_clearing_is_per_robot_and_bumps_the_revision():
    fleet = FleetManager()
    fleet.handle(frame("rover1", ["a"]), now=0.0)
    before = fleet.console_revs()["rover1"]
    fleet.clear_console("rover1")
    assert fleet.script_console()["rover1"]["lines"] == []
    # The revision has to move even though the content shrank, or the broadcast
    # loop never pushes the empty console and the operator's Clear does nothing.
    assert fleet.console_revs()["rover1"] > before


def test_a_robot_that_printed_nothing_is_absent_rather_than_empty():
    """Keeps the frame small on a fleet where one rover is running a script and
    three are being driven."""
    fleet = FleetManager()
    fleet.handle(frame("rover1", ["a"]), now=0.0)
    fleet.handle({"type": "telemetry", "from": "rover2", "mode": "teleop"},
                 now=0.0)
    assert "rover2" not in fleet.script_console()


def test_an_empty_frame_changes_nothing():
    fleet = FleetManager()
    fleet.handle(frame(lines=[]), now=0.0)
    assert fleet.script_console() == {}


# --- the half that must NOT depend on WiFi ----------------------------------


def test_whether_a_run_is_going_rides_the_hot_frame_instead():
    """The console can be dropped; this cannot. A rover out of WiFi range still
    tells the operator its script is running and why it stopped."""
    fleet = FleetManager()
    fleet.handle({"type": "telemetry", "from": "rover1", "mode": "script",
                  "script": {"id": "prog", "run": False, "why": "error",
                             "err": "ZeroDivisionError: division by zero"}},
                 now=0.0)
    robot = fleet.snapshot(0.0)["robots"][0]
    assert robot["script"]["why"] == "error"
    assert "ZeroDivisionError" in robot["script"]["err"]


def test_scripts_are_a_document_like_routines_are():
    """Same reassembly path, same per-document revision — so saving a script
    does not push a layout back over the radio."""
    fleet = FleetManager()
    fleet.handle({"type": "scripts", "from": "rover1", "txid": "S1", "seq": 0,
                  "n": 1, "rev": 3,
                  "part": '{"version":1,"scripts":[{"id":"go","code":"pass"}]}'},
                 now=0.0)
    documents = fleet.documents()["rover1"]
    assert documents["scripts"]["scripts"][0]["id"] == "go"
    assert documents["scripts_rev"] == 1
    assert documents["layout_rev"] == 0


def test_a_refusal_reaches_the_editor():
    fleet = FleetManager()
    fleet.handle({"type": "scripts_result", "from": "rover1", "ok": False,
                  "errors": ["script 'go': line 4: invalid syntax"],
                  "warnings": []}, now=0.0)
    result = fleet.documents()["rover1"]["scripts_result"]
    assert result["ok"] is False
    assert "line 4" in result["errors"][0]
