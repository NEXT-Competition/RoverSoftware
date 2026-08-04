"""Writing and running a script with no hardware at all.

The simulator answers `get_scripts`/`put_scripts` with the REAL validator and
runs them with the REAL ScriptController and sandbox, for the reason its module
docstring already gives about routines: a code editor you can only test on a
rover is a code editor that ships broken. These tests are the proof of that —
if the editor's save, refusal and run paths work here, they work before anybody
owns a robot.
"""

import time

import pytest

from basestation.simulator import _SimRobot
from robot.script import schema


def load(rover, code, sid="prog"):
    result = schema.parse({"version": 1,
                           "scripts": [{"id": sid, "code": code}]})
    assert result.ok, result.errors
    rover.scripts = result.scripts
    return result


def rover():
    return _SimRobot("sim1", 51.5, -0.12, heading=0.0)


def run(r, seconds=1.5, dt=0.02):
    """Step the simulated rover, giving the script thread real time to run."""
    for _ in range(int(seconds / dt)):
        r.step(dt)
        time.sleep(0.004)
        if r.script is not None and r.script.runner is None:
            return


def test_a_script_drives_the_simulated_rover():
    r = rover()
    load(r, "rover.forward(0.6)\nrover.sleep(1.0)\nrover.stop()\n")
    r.mode = "script"
    r.start_script("prog")
    lat_before = r.lat
    run(r, seconds=0.6)
    assert r.left > 0.5 and r.right > 0.5
    run(r, seconds=1.2)
    assert r.lat != lat_before, "the fake rover never moved"


def test_a_script_reads_the_simulated_sensors():
    r = rover()
    load(r, "rover.watch('heading', rover.heading())\n"
            "rover.watch('fix', rover.gps.fix)\n"
            "rover.sleep(5)\n")
    r.mode = "script"
    r.start_script("prog")
    run(r, seconds=0.4)
    watched = r.script.mailbox.watched()
    assert watched["heading"] == pytest.approx(0.0)
    assert watched["fix"] == 1


def test_turn_to_lands_on_a_heading():
    """The one helper with a loop in it, closed around the simulated IMU."""
    r = rover()
    load(r, "print('ok' if rover.turn_to(90, tolerance=6) else 'gave up')\n")
    r.mode = "script"
    r.start_script("prog")
    run(r, seconds=12.0)
    assert abs(((r.heading - 90.0 + 180.0) % 360.0) - 180.0) < 15.0


def test_the_estop_aborts_a_running_script():
    r = rover()
    load(r, "rover.forward(1.0)\nrover.sleep(30)\n")
    r.mode = "script"
    r.start_script("prog")
    run(r, seconds=0.3)
    r.estop = True
    r.step(0.02)
    assert r.script.runner is None
    assert (r.left, r.right) == (0.0, 0.0)


def test_the_dashboards_two_messages_select_then_run_it():
    """`select_script` then `mode` — the pair the Run button sends. Selecting
    alone must NOT start a thread nothing is draining, which is the rover's own
    rule; the mode change is what starts it."""
    r = rover()
    load(r, "rover.forward(0.4)\nrover.sleep(30)\n")
    r.select_script("prog")
    assert r.script.runner is None, "selecting alone started it"
    r.mode = "script"
    # The mode handler on SimulatedFleet does this; call it the same way.
    r.start_script()
    run(r, seconds=0.3)
    assert r.script.runner is not None
    assert r.left > 0.3


def test_leaving_script_mode_unwinds_the_run():
    r = rover()
    load(r, "rover.forward(1.0)\nrover.sleep(30)\n")
    r.mode = "script"
    r.start_script("prog")
    run(r, seconds=0.3)
    assert r.script.runner is not None
    r.script.on_deactivate()
    assert r.script.runner is None


def test_a_syntax_error_is_refused_with_its_line_number():
    """Through the simulator's own document path, which is what the editor
    talks to — so the marker the editor draws is one you can see appear with
    no rover switched on."""
    seen = []
    from basestation.simulator import SimulatedFleet

    fleet = SimulatedFleet(on_message=seen.append)
    fleet.send({"to": "rover1", "type": "put_scripts", "txid": "T1", "seq": 0,
                "n": 1, "save": False,
                "part": '{"version":1,"scripts":[{"id":"bad",'
                        '"code":"x = 1\\nif True\\n  pass\\n"}]}'})
    results = [m for m in seen if m.get("type") == "scripts_result"]
    assert results and results[-1]["ok"] is False
    assert "line 2" in results[-1]["errors"][0]


def test_a_good_document_is_stored_and_echoed_back():
    seen = []
    from basestation.simulator import SimulatedFleet

    fleet = SimulatedFleet(on_message=seen.append)
    fleet.send({"to": "rover1", "type": "put_scripts", "txid": "T1", "seq": 0,
                "n": 1, "save": False,
                "part": '{"version":1,"scripts":[{"id":"go",'
                        '"code":"rover.stop()\\n"}]}'})
    results = [m for m in seen if m.get("type") == "scripts_result"]
    assert results and results[-1]["ok"] is True
    # Echoed back, as the rover does: the editor must show what was STORED, not
    # what it hopefully sent.
    assert any(m.get("type") == "scripts" for m in seen)
