"""Trigger throttle mapping: R2 = forward, L2 = reverse.

The interesting case is the arming latch. A driver that reports a flat 0.0 for
an untouched trigger would, under a naive (v+1)/2 rescale, hand the robot half
throttle the moment a controller is plugged in. These tests pin that down.
"""

from basestation.controller_input import TRIGGER_REST, Trigger, _dz


def test_released_trigger_is_zero():
    t = Trigger()
    assert t.value(TRIGGER_REST) == 0.0


def test_full_pull_is_one():
    t = Trigger()
    t.value(TRIGGER_REST)  # arm
    assert t.value(1.0) == 1.0


def test_half_pull_is_half():
    t = Trigger()
    t.value(TRIGGER_REST)
    assert t.value(0.0) == 0.5


def test_stays_dead_until_it_reports_rest():
    """A trigger that only ever reports 0.0 must never command throttle."""
    t = Trigger()
    for _ in range(100):
        assert t.value(0.0) == 0.0
    assert not t.armed


def test_arms_once_rest_is_seen_then_tracks():
    t = Trigger()
    assert t.value(0.0) == 0.0        # unknown -> refuse
    assert t.value(TRIGGER_REST) == 0.0  # rest seen -> armed
    assert t.armed
    assert t.value(0.0) == 0.5        # same raw value, now trusted


def test_reset_disarms():
    t = Trigger()
    t.value(TRIGGER_REST)
    assert t.armed
    t.reset()
    assert not t.armed
    assert t.value(0.0) == 0.0


def test_value_is_clamped():
    t = Trigger()
    t.value(TRIGGER_REST)
    assert t.value(-5.0) == 0.0
    assert t.value(5.0) == 1.0


def _throttle(r2_raw, l2_raw):
    """Mirror of the reader's throttle mix, with both triggers armed."""
    r2, l2 = Trigger(), Trigger()
    r2.value(TRIGGER_REST)
    l2.value(TRIGGER_REST)
    return _dz(r2.value(r2_raw) - l2.value(l2_raw))


def test_r2_drives_forward_l2_reverses():
    assert _throttle(1.0, TRIGGER_REST) == 1.0
    assert _throttle(TRIGGER_REST, 1.0) == -1.0


def test_both_triggers_cancel():
    assert _throttle(1.0, 1.0) == 0.0
    assert _throttle(0.0, 0.0) == 0.0


def test_neutral_triggers_are_stopped():
    assert _throttle(TRIGGER_REST, TRIGGER_REST) == 0.0


def test_feathered_trigger_is_deadzoned():
    # Just off rest -> below the 0.08 dead-zone -> a hard zero, no creep.
    assert _throttle(TRIGGER_REST + 0.1, TRIGGER_REST) == 0.0


# --- a mechanism on an analog trigger ----------------------------------------
#
# L2/R2 on a DualShock report ONLY as axes, with no button behind them, so a
# flywheel "on R2" cannot be a binding in `actions()` at all. These pin down the
# separate path that carries it, and above all that it fails STOPPED: an unarmed
# trigger, a half-filled binding and a released trigger must all mean no power.

import pygame  # noqa: E402  (skipped below when it isn't installed)
import pytest  # noqa: E402

from basestation.controller_input import ControllerReader  # noqa: E402
from basestation.settings import ControllerMapping  # noqa: E402


class FakeJoystick:
    def __init__(self, axes, buttons=()):
        self._axes, self._buttons = list(axes), list(buttons)

    def get_numaxes(self):
        return len(self._axes)

    def get_numbuttons(self):
        return len(self._buttons)

    def get_axis(self, i):
        return self._axes[i]

    def get_button(self, i):
        return self._buttons[i]


def reader(mapping, axes, armed=True):
    r = ControllerReader.__new__(ControllerReader)   # no pygame init, no thread
    r.on_mech_axis = None
    r._map = mapping
    r._js = FakeJoystick(axes)
    from basestation.controller_input import Trigger
    r._mech_axis = Trigger(mapping.trigger_rest)
    if armed:
        r._mech_axis.armed = True
    return r


def capture(r, mapping):
    seen = []
    r.on_mech_axis = lambda mech, power: seen.append((mech, power))
    r._push_mech_axis(mapping, r._js.get_numaxes())
    return seen


def test_a_pulled_trigger_reports_its_position_as_power():
    m = ControllerMapping(axis_mech=5, mech_axis="flywheel", trigger_rest=-1.0)
    # -1 released .. +1 fully pulled, so 0.0 is exactly half.
    assert capture(reader(m, [0, 0, 0, 0, -1.0, 0.0]), m) == [("flywheel", 0.5)]
    assert capture(reader(m, [0, 0, 0, 0, -1.0, 1.0]), m) == [("flywheel", 1.0)]


def test_a_released_trigger_reports_zero_every_tick():
    """The release frame is what stops the motor promptly; the robot's jog
    timeout is only the backstop for when nobody is left to send one."""
    m = ControllerMapping(axis_mech=5, mech_axis="flywheel", trigger_rest=-1.0)
    assert capture(reader(m, [0, 0, 0, 0, -1.0, -1.0]), m) == [("flywheel", 0.0)]


def test_a_barely_touched_trigger_is_deadzoned_to_zero():
    m = ControllerMapping(axis_mech=5, mech_axis="flywheel", trigger_rest=-1.0,
                          axis_mech_deadzone=0.1)
    seen = capture(reader(m, [0, 0, 0, 0, -1.0, -0.9]), m)   # ~5% pulled
    assert seen == [("flywheel", 0.0)]


def test_an_unarmed_trigger_reports_nothing_but_zero():
    """A driver that reports a flat 0.0 for an untouched trigger would other-
    wise spin the flywheel at half power the moment the pad was plugged in."""
    m = ControllerMapping(axis_mech=5, mech_axis="flywheel", trigger_rest=-1.0)
    r = reader(m, [0, 0, 0, 0, -1.0, 0.0], armed=False)
    assert capture(r, m) == [("flywheel", 0.0)]


def test_an_unbound_axis_sends_nothing_at_all():
    m = ControllerMapping(mech_axis="flywheel")           # no axis
    assert capture(reader(m, [0, 0, 0, 0, -1.0, 1.0]), m) == []


def test_an_axis_with_no_mechanism_sends_nothing():
    m = ControllerMapping(axis_mech=5)                    # no mechanism
    assert capture(reader(m, [0, 0, 0, 0, -1.0, 1.0]), m) == []


def test_an_axis_off_the_end_of_the_pad_is_safe():
    """A mapping made against a 6-axis pad, used on a smaller one."""
    m = ControllerMapping(axis_mech=11, mech_axis="flywheel", trigger_rest=-1.0)
    assert capture(reader(m, [0, 0]), m) == [("flywheel", 0.0)]


def test_the_slot_needs_both_halves():
    assert ControllerMapping(axis_mech=5, mech_axis="fly").mech_axis_slot() == (5, "fly")
    assert ControllerMapping(axis_mech=5).mech_axis_slot() == (-1, "")
    assert ControllerMapping(mech_axis="fly").mech_axis_slot() == (-1, "")
    assert ControllerMapping(axis_mech=5, mech_axis="  ").mech_axis_slot() == (-1, "")
