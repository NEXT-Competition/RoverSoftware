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
