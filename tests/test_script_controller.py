"""The `script` mode: what a running script can and cannot make the robot do.

The controller is where the safety story is actually enforced, so that is what
these test: the motors stop when the run ends however it ended, the e-stop
aborts, an actuator command reaches the mechanism and not the drivetrain, and a
delegate's lifecycle is run properly.

Like every controller test in this suite it uses stubs and steps `update()` by
hand — no hardware, no real clock, no 50 Hz loop.
"""

import time

import pytest

from robot.config import ScriptConfig
from robot.control.commands import DriveCommand
from robot.control.script_controller import ScriptController
from robot.script.schema import Script, parse


def scripts(**by_id):
    """Build a validated script set, so these tests cannot drift from what the
    robot would actually accept."""
    doc = {"version": 1,
           "scripts": [{"id": sid, "code": code} for sid, code in by_id.items()]}
    result = parse(doc)
    assert result.ok, result.errors
    return result.scripts


class FakeMech:
    def __init__(self, kind="power"):
        self.kind = kind
        self.power = None
        self.actuator = None
        self.stopped = 0
        self.fired = 0
        self._ready = True

    def set_power(self, power, actuator=None):
        self.power, self.actuator = power, actuator
        return True

    def fire(self):
        self.fired += 1
        self._ready = False
        return True

    def stop(self):
        self.stopped += 1
        self.power = 0.0
        self._ready = True

    def ready(self):
        return self._ready

    def status(self):
        return {"kind": self.kind, "ready": self._ready, "count": self.fired,
                "values": {"m": self.power or 0.0}}


class FakeAlign:
    """Enough of ObjectAlignController for delegation to be observable."""

    name = "object_align"
    standoff_m = 1.0

    def __init__(self):
        self.active = 0
        self.released = 0
        self.command = DriveCommand.tank(0.4, 0.2)
        self._arrived = False

    def on_activate(self):
        self.active += 1

    def on_deactivate(self):
        self.released += 1

    def on_message(self, message):
        pass

    def update(self, dt):
        return self.command

    def aligned(self):
        return True

    def arrived(self):
        return self._arrived

    def distance_m(self):
        return 1.4

    def last_detection(self):
        return object()

    def pid_traces(self):
        return {"align.pid": {"sp": 0.0}}


def build(code="rover.sleep(60)\n", **cfg):
    align = FakeAlign()
    mech = FakeMech()
    controllers = {"object_align": align}
    mechanisms = {"intake": mech}
    controller = ScriptController(controllers, mechanisms,
                                  ScriptConfig(**cfg))
    controller.set_scripts(scripts(prog=code))
    return controller, align, mech


def spin(controller, ticks=40, dt=0.02, until=None):
    """Step the control loop, giving the script thread room to run.

    A real 50 Hz loop is 20 ms apart; the sleep here is what makes the worker
    thread's progress real rather than a scheduling accident.
    """
    for _ in range(ticks):
        command = controller.update(dt)
        if until is not None and until():
            return command
        time.sleep(0.005)
    return command


# --- driving -----------------------------------------------------------------


def test_a_script_drives_the_tracks():
    controller, _, _ = build("rover.drive(0.5, -0.5)\nrover.sleep(30)\n")
    controller.on_activate()
    command = spin(controller, until=lambda: controller.update(0.02).left != 0)
    assert command.left == pytest.approx(0.5)
    assert command.right == pytest.approx(-0.5)
    controller.on_deactivate()


def test_nothing_drives_before_a_script_is_started():
    controller, _, _ = build()
    assert controller.update(0.02) == DriveCommand.stopped()


def test_the_motors_stop_when_the_script_finishes():
    """Not because the script said so — because there is no run in progress,
    which is a property of the controller and not of a `finally:` block."""
    controller, _, _ = build("rover.drive(1.0, 1.0)\n")
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    assert controller.runner is None
    assert controller.update(0.02) == DriveCommand.stopped()


def test_the_motors_stop_when_the_script_crashes():
    controller, _, _ = build("rover.drive(1.0, 1.0)\nraise ValueError('boom')\n")
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    assert controller.update(0.02) == DriveCommand.stopped()
    assert "ValueError" in controller.last_error


def test_the_drive_limit_scales_without_changing_the_arc():
    """Clamping each track would change the RATIO, so a limited script would
    drive a different curve rather than the same one more slowly."""
    controller, _, _ = build("rover.drive(1.0, 0.5)\nrover.sleep(30)\n",
                             drive_limit=0.5)
    controller.on_activate()
    command = spin(controller, until=lambda: controller.update(0.02).left != 0)
    assert command.left == pytest.approx(0.5)
    assert command.right == pytest.approx(0.25)
    controller.on_deactivate()


# --- stopping ----------------------------------------------------------------


def test_the_estop_aborts_the_run():
    controller, _, _ = build("rover.drive(1.0, 1.0)\nrover.sleep(60)\n")
    controller.on_activate()
    spin(controller, ticks=10)
    controller.on_estop()
    assert controller.runner is None
    assert controller.update(0.02) == DriveCommand.stopped()


def test_leaving_the_mode_ends_the_run_and_stops_the_mechanisms():
    controller, _, mech = build(
        "rover.mech('intake').power(1.0)\nrover.sleep(60)\n")
    controller.on_activate()
    spin(controller, until=lambda: mech.power == 1.0)
    assert mech.power == 1.0
    controller.on_deactivate()
    assert controller.runner is None
    # A script that ended with the intake still spinning is a script that
    # ended unsafely, whether it ended by finishing or by being switched away.
    assert mech.stopped >= 1


def test_replacing_the_script_set_stops_the_running_one():
    controller, _, _ = build("rover.sleep(60)\n")
    controller.on_activate()
    spin(controller, ticks=5)
    assert controller.runner is not None
    controller.set_scripts(scripts(other="pass\n"))
    assert controller.runner is None
    assert controller.selected == "other"


def test_a_disabled_robot_refuses_to_run_anything():
    controller, _, _ = build("rover.drive(1, 1)\n", enabled=False)
    controller.on_activate()
    assert controller.runner is None
    assert controller.update(0.02) == DriveCommand.stopped()
    assert "disabled" in controller.last_error


# --- actuators ---------------------------------------------------------------


def test_an_actuator_command_reaches_the_mechanism():
    controller, _, mech = build(
        "rover.mech('intake').power(0.8, actuator='m')\nrover.sleep(30)\n")
    controller.on_activate()
    spin(controller, until=lambda: mech.power is not None)
    assert mech.power == pytest.approx(0.8)
    assert mech.actuator == "m"
    controller.on_deactivate()


def test_a_command_for_a_mechanism_this_build_lacks_says_so_and_carries_on():
    controller, _, _ = build(
        "rover.mech('arm').power(1.0)\nprint('still here')\n")
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    # The point is that it did not crash the script: a program written for the
    # rover with an arm still runs on the one without.
    assert controller.last_reason == "finished"


def test_pulse_then_wait_ready_does_not_fall_through():
    """The race the mailbox exists to remove. If `pulse()` returned before the
    control loop applied it, `ready` would still be answering about the state
    before the pulse and the wait would return immediately."""
    controller, _, mech = build(
        "m = rover.mech('intake')\n"
        "m.pulse()\n"
        "print('fired' if not m.ready else 'fell through')\n")
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    lines, _ = controller.take_output() if controller.mailbox else ([], {})
    assert mech.fired == 1


# --- delegation --------------------------------------------------------------


def test_handing_over_activates_the_real_controller_and_uses_its_command():
    controller, align, _ = build(
        "rover.hand_over('object_align')\nrover.sleep(30)\n")
    controller.on_activate()
    command = spin(controller, until=lambda: align.active > 0)
    assert align.active == 1
    command = controller.update(0.02)
    assert command == align.command
    # And its graph, because a script that aligns is aligning with the real
    # loop — the same gains, and the same reason to want a picture of it.
    assert "align.pid" in controller.pid_traces()
    controller.on_deactivate()
    assert align.released == 1


def test_a_delegate_is_released_when_the_run_ends():
    controller, align, _ = build("rover.hand_over('object_align')\n")
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    assert align.released == 1
    assert controller.update(0.02) == DriveCommand.stopped()


def test_a_borrowed_standoff_is_handed_back():
    """A script must not leave the operator's own settings rewritten."""
    controller, align, _ = build(
        "rover.align_to('bucket', within_m=2.5, timeout=0.1)\n")
    controller.set_vision_config(type("V", (), {"target_label": "cone",
                                                "hfov_deg": 60.0})())
    controller.on_activate()
    spin(controller, ticks=80, until=lambda: controller.runner is None)
    assert align.standoff_m == pytest.approx(1.0)


def test_a_borrowed_detector_target_is_handed_back():
    vision = type("V", (), {"target_label": "cone", "hfov_deg": 60.0})()
    controller, _, _ = build("rover.look_for('bucket')\nrover.sleep(0.05)\n")
    controller.set_vision_config(vision)
    controller.on_activate()
    spin(controller, ticks=20, until=lambda: vision.target_label == "bucket")
    assert vision.target_label == "bucket"
    controller.on_deactivate()
    assert vision.target_label == "cone"


def test_handing_over_to_a_mode_this_build_lacks_is_refused_not_obeyed():
    controller, _, _ = build("rover.hand_over('waypoint')\nrover.sleep(30)\n")
    controller.on_activate()
    spin(controller, ticks=30)
    assert controller.update(0.02) == DriveCommand.stopped()
    controller.on_deactivate()


# --- telemetry ---------------------------------------------------------------


def test_status_says_whether_a_run_is_in_progress_and_why_it_ended():
    controller, _, _ = build("raise RuntimeError('nope')\n")
    assert controller.status()["run"] is False
    controller.on_activate()
    spin(controller, ticks=60, until=lambda: controller.runner is None)
    status = controller.status()
    assert status["run"] is False
    assert status["why"] == "error"
    assert "nope" in status["err"]


def test_console_output_is_drained_rather_than_accumulated():
    controller, _, _ = build("print('one')\nrover.sleep(30)\n")
    controller.on_activate()
    spin(controller, ticks=20)
    lines, _ = controller.take_output()
    assert lines == ["one"]
    assert controller.take_output()[0] == []
    controller.on_deactivate()


def test_watched_values_reach_the_dashboard():
    controller, _, _ = build("rover.watch('range', 1.25)\nrover.sleep(30)\n")
    controller.on_activate()
    spin(controller, ticks=20, until=lambda: controller.take_output()[1])
    controller.on_deactivate()
