"""How a pad sample becomes (throttle, steer).

Two things are pinned here. The default two-stick layout — left stick throttle,
right stick steering — and the trigger fallback's arming latch: a driver that
reports a flat 0.0 for an untouched trigger would, under a naive (v+1)/2
rescale, hand the robot half throttle the moment a controller is plugged in.
"""

from basestation.controller_input import TRIGGER_REST, Trigger, _dz, mix
from basestation.settings import UNBOUND, ControllerMapping


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


# --- two-stick drive (the default layout) ----------------------------------


def _armed():
    """Two triggers that have been seen at rest, as a live pad's would be."""
    l2, r2 = Trigger(), Trigger()
    l2.value(TRIGGER_REST)
    r2.value(TRIGGER_REST)
    return l2, r2


def _sample(**axes):
    """A raw axis list with the named indices set and the rest resting."""
    out = [0.0, 0.0, 0.0, 0.0, TRIGGER_REST, TRIGGER_REST]
    for idx, value in axes.items():
        out[int(idx.lstrip("a"))] = value
    return out


def _mix(axes, m=None):
    return mix(m or ControllerMapping(), axes, *_armed())


def test_two_sticks_is_the_default_layout():
    m = ControllerMapping()
    assert m.axis_throttle >= 0, "throttle should default to a stick, not triggers"
    assert m.axis_throttle != m.axis_steer, "throttle and steer want separate sticks"


def test_left_stick_forward_drives_forward():
    # Sticks report UP as negative; invert_throttle defaults on.
    assert _mix(_sample(a1=-1.0)) == (1.0, 0.0)


def test_left_stick_back_reverses():
    assert _mix(_sample(a1=1.0)) == (-1.0, 0.0)


def test_right_stick_steers_without_touching_throttle():
    throttle, steer = _mix(_sample(a2=1.0))
    assert throttle == 0.0
    assert steer == 1.0


def test_the_two_sticks_are_independent():
    """The point of the layout: hold a speed and steer at the same time."""
    throttle, steer = _mix(_sample(a1=-0.5, a2=-1.0))
    assert throttle == 0.5
    assert steer == -1.0


def test_triggers_are_ignored_while_a_stick_throttle_is_bound():
    """Otherwise a resting trigger would fight the stick for the drivetrain."""
    assert _mix(_sample(a1=-1.0, a4=1.0, a5=1.0)) == (1.0, 0.0)


def test_centred_sticks_are_stopped():
    assert _mix(_sample()) == (0.0, 0.0)


def test_stick_throttle_is_deadzoned():
    m = ControllerMapping()
    assert _mix(_sample(a1=-(m.deadzone / 2)), m)[0] == 0.0


def test_clearing_the_throttle_axis_falls_back_to_the_triggers():
    m = ControllerMapping(axis_throttle=UNBOUND)
    assert _mix(_sample(a5=1.0), m)[0] == 1.0   # R2 forward
    assert _mix(_sample(a4=1.0), m)[0] == -1.0  # L2 reverse
    # ...and the stick that used to drive it now does nothing.
    assert _mix(_sample(a1=-1.0), m)[0] == 0.0


def test_gains_and_inversion_apply_to_a_stick_throttle():
    m = ControllerMapping(throttle_gain=0.5, invert_throttle=False)
    assert _mix(_sample(a1=1.0), m)[0] == 0.5
