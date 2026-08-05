"""A script's whole journey on a real `Robot`, hardware mocked.

The unit tests above this one check the sandbox, the controller and the base
station separately. This checks that the wiring between them is right on the
object that actually runs in the field: a document arrives over the link, is
compiled, is stored, is run when the mode changes, drives the mocked motors,
prints to the bulk link, and stops everything when it ends.

Same construction path as tests/test_robot_config.py, which is the same one
`RS_MOCK_MOTORS=1 python run_robot.py` takes.
"""

import json
import time

import pytest

from robot.comms.doc_transfer import split
from robot.config import RobotConfig
from robot.robot import Robot
from robot.script import store as script_store


class FakeIP:
    """Stands in for IPLink. Documents and console output go over WiFi or not at
    all (Robot._drain_outbox), so a rover with no link here would drop every
    frame these tests are about — which is itself the designed behaviour, and is
    what test_console_output_is_dropped_without_wifi checks."""

    host, port, connected, stopped, started = "base.local", 5006, True, False, False

    def __init__(self):
        self.sent = []

    def start(self):
        self.started = True

    def is_connected(self):
        return self.connected

    def send(self, msg):
        self.sent.append(msg)
        return True

    def stop(self):
        self.stopped = True


@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    monkeypatch.setenv("RS_SCRIPTS_FILE", str(tmp_path / "scripts.json"))
    monkeypatch.setenv("RS_ROUTINES_FILE", str(tmp_path / "routines.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = False
    cfg.imu.enabled = False
    cfg.camera.enabled = False
    cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    bot = Robot(cfg)
    sent = []

    def take(message):
        sent.append(message)
        return True

    bot.link.send = take
    bot.link.send_bulk = take
    # The radio, and the WiFi link the bulk frames actually take.
    bot.radio = sent
    bot.ip_link = FakeIP()
    bot.sent = bot.ip_link.sent
    return bot


def deliver(bot, msg):
    bot._inbox.put(msg)
    bot._drain_inbox()
    while bot._outbox:
        bot._drain_outbox()


def save(bot, code, sid="prog", save_to_disk=True):
    """Push a script document the way the base station does — in fragments."""
    doc = {"version": 1, "scripts": [{"id": sid, "name": "Test", "code": code}]}
    for frame in split(doc, "put_scripts", txid="B1", save=save_to_disk):
        deliver(bot, frame)
    return [m for m in bot.sent if m.get("type") == "scripts_result"][-1]


def spin(bot, ticks=60, dt=0.02):
    """Run the control loop by hand, at something like the real rate."""
    for _ in range(ticks):
        bot._drain_inbox()
        command = bot.manager.update(dt)
        bot.drive.drive(command.left, command.right)
        bot._last_command = (command.left, command.right)
        bot._drain_script_output(time.monotonic())
        while bot._outbox:
            bot._drain_outbox()
        time.sleep(0.004)
        if bot.script_controller.runner is None:
            return command
    return command


# --- the document -------------------------------------------------------------


def test_a_script_is_accepted_stored_and_echoed_back(rover):
    result = save(rover, "rover.stop()\n")
    assert result["ok"] is True
    assert json.load(open(script_store.scripts_path()))["scripts"][0]["id"] == "prog"
    # Echoed back like every other document: the validator can normalise, so the
    # editor must show what was STORED rather than what it hopefully sent.
    assert any(m.get("type") == "scripts" for m in rover.sent)


def test_a_syntax_error_is_refused_with_a_line_number_and_nothing_is_stored(rover):
    result = save(rover, "x = 1\nif True\n    pass\n")
    assert result["ok"] is False
    assert "line 2" in result["errors"][0]
    with pytest.raises(FileNotFoundError):
        open(script_store.scripts_path())


def test_a_refused_document_leaves_the_last_good_one_running(rover):
    save(rover, "rover.stop()\n")
    save(rover, "def nope(\n")
    assert "prog" in rover.script_controller.scripts
    assert rover.script_controller.scripts["prog"].code == "rover.stop()\n"


def test_get_scripts_answers_with_what_it_has(rover):
    save(rover, "rover.stop()\n")
    rover.sent.clear()
    deliver(rover, {"type": "get_scripts"})
    assert any(m.get("type") == "scripts" for m in rover.sent)


def test_scripts_survive_a_restart(rover, monkeypatch, tmp_path):
    save(rover, "rover.forward(0.2)\n")
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    fresh = Robot(cfg)
    assert "prog" in fresh.script_controller.scripts


# --- running it ---------------------------------------------------------------


def test_selecting_and_running_reaches_the_motors(rover):
    save(rover, "rover.forward(0.5)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    command = spin(rover, ticks=30)
    assert command.left == pytest.approx(0.5)
    assert command.right == pytest.approx(0.5)
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_a_selection_reaches_the_script_mode_from_any_other_mode(rover):
    """The bug this routing exists to prevent: sent through the ACTIVE
    controller, `select_script` would be dropped by teleop, and the `mode` frame
    a moment later would start whichever script had been selected before."""
    save(rover, "rover.stop()\n", sid="one")
    for frame in split({"version": 1, "scripts": [
            {"id": "one", "code": "rover.stop()\n"},
            {"id": "two", "code": "rover.forward(0.4)\nrover.sleep(30)\n"}]},
            "put_scripts", txid="B2"):
        deliver(rover, frame)
    assert rover.manager.mode == "teleop"
    deliver(rover, {"type": "select_script", "id": "two"})
    assert rover.script_controller.selected == "two"


def test_a_start_without_the_mode_is_refused_rather_than_left_hanging(rover):
    """A script started while teleop is driving has nothing draining its
    mailbox: every actuator call would block until it timed out and its drive
    commands would go nowhere. Refusing says so; starting would look like a
    hang."""
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "script_cmd", "cmd": "start", "id": "prog"})
    assert rover.manager.mode == "teleop"
    assert rover.script_controller.runner is None
    # And the two-message form the dashboard sends does work.
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=10)
    assert rover.script_controller.runner is not None
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_leaving_script_mode_stops_the_motors(rover):
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=20)
    deliver(rover, {"type": "mode", "mode": "teleop"})
    assert rover.script_controller.runner is None
    assert rover.manager.update(0.02).left == 0.0


def test_the_estop_aborts_the_run(rover):
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=20)
    deliver(rover, {"type": "estop"})
    assert rover.script_controller.runner is None
    assert rover.manager.update(0.02) .left == 0.0


def test_a_stop_command_ends_it_without_leaving_the_mode(rover):
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=20)
    deliver(rover, {"type": "script_cmd", "cmd": "stop"})
    assert rover.script_controller.runner is None
    assert rover.manager.mode == "script"
    assert rover.manager.update(0.02).left == 0.0


def test_saving_a_new_document_stops_the_running_script(rover):
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=20)
    assert rover.script_controller.runner is not None
    save(rover, "rover.stop()\n")
    # Continuing to execute code out of a document that no longer contains it is
    # worse than stopping.
    assert rover.script_controller.runner is None


# --- telemetry and output ------------------------------------------------------


def test_the_hot_frame_says_whether_a_run_is_going(rover):
    save(rover, "rover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=10)
    telemetry = rover._telemetry(rover.manager.update(0.02))
    assert telemetry["script"]["run"] is True
    assert telemetry["script"]["id"] == "prog"
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_console_output_goes_out_as_bulk_and_never_on_the_hot_frame(rover):
    save(rover, "print('hello from the rover')\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=40)
    frames = [m for m in rover.sent if m.get("type") == "script_output"]
    assert frames, "nothing was forwarded"
    assert any("hello from the rover" in f["lines"] for f in frames)
    # And none of it went on the radio, which is the whole point of putting it
    # on the bulk link: that channel carries driving, telemetry and the e-stop.
    assert not [m for m in rover.radio if m.get("type") == "script_output"]
    telemetry = rover._telemetry(rover.manager.update(0.02))
    assert "lines" not in json.dumps(telemetry)
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_console_output_is_dropped_rather_than_queued_without_wifi(rover):
    """The designed failure. A rover out of WiFi range still RUNS its script and
    still reports on the hot frame whether it is going — it just cannot say what
    it printed, and the outbox must not grow for the rest of the match."""
    # Saved while the link is up — a document that could not be DELIVERED is a
    # different failure, and the base station already reports that one.
    save(rover, "print('into the void')\nrover.sleep(30)\n")
    rover.ip_link = None
    rover.sent.clear()
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=40)
    assert not [m for m in rover.radio if m.get("type") == "script_output"]
    assert len(rover._outbox) == 0
    telemetry = rover._telemetry(rover.manager.update(0.02))
    assert telemetry["script"]["run"] is True
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_a_crash_is_reported_on_the_hot_frame_not_only_in_the_console(rover):
    """A rover out of WiFi range still has to be able to say what went wrong."""
    save(rover, "1 / 0\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=60)
    telemetry = rover._telemetry(rover.manager.update(0.02))
    assert telemetry["script"]["run"] is False
    assert "ZeroDivisionError" in telemetry["script"]["err"]


# --- the knobs ------------------------------------------------------------------


def test_the_drive_limit_is_live_and_scales_without_changing_the_arc(rover):
    save(rover, "rover.drive(1.0, 0.5)\nrover.sleep(30)\n")
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    spin(rover, ticks=20)
    deliver(rover, {"type": "set_config", "config": {"scripts.drive_limit": 0.5},
                    "save": False})
    command = spin(rover, ticks=10)
    assert command.left == pytest.approx(0.5)
    assert command.right == pytest.approx(0.25)
    deliver(rover, {"type": "mode", "mode": "teleop"})


def test_a_robot_with_scripts_disabled_runs_nothing(rover):
    save(rover, "rover.forward(1.0)\nrover.sleep(30)\n")
    deliver(rover, {"type": "set_config", "config": {"scripts.enabled": False},
                    "save": False})
    deliver(rover, {"type": "select_script", "id": "prog"})
    deliver(rover, {"type": "mode", "mode": "script"})
    assert rover.script_controller.runner is None
    assert rover.manager.update(0.02).left == 0.0
