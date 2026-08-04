"""The three things that made the stick feel wrong, pinned so they stay fixed.

Each one is a case where the OLD code was defensible in isolation and only
misbehaved at an edge — a saturated mix, a stick leaving centre, an inverted
motor — which is exactly the kind of thing that gets re-simplified back in.
"""

from basestation.controller_input import _dz
from robot.config import MotorConfig
from robot.control.commands import DriveCommand
from robot.drive.motor import ESCMotor


def test_arcade_keeps_the_turn_radius_when_the_mix_saturates():
    # 0.8 throttle + 0.4 steer overshoots on the left track. The old code
    # clamped left to 1.0 and left right at 0.4, flattening a 3:1 turn into
    # 2.5:1 — the turn loosens exactly when the stick asks for more.
    cmd = DriveCommand.arcade(0.8, 0.4)
    assert cmd.left / cmd.right == 3.0
    assert cmd.left <= 1.0 and cmd.right <= 1.0
    assert cmd.left == 1.0  # the pair is scaled to fit, not shrunk further


def test_arcade_is_unchanged_below_saturation():
    # The fix must be invisible for ordinary inputs, or it is a feel change
    # rather than a bug fix.
    for throttle, steer in ((0.0, 0.0), (0.5, 0.2), (-0.3, 0.6), (0.0, 1.0)):
        cmd = DriveCommand.arcade(throttle, steer)
        assert cmd.left == throttle + steer
        assert cmd.right == throttle - steer


def test_leaving_the_dead_zone_does_not_step():
    # Just outside the dead zone must be near zero, not 0.08.
    assert _dz(0.081, 0.08) < 0.005
    assert _dz(-0.081, 0.08) > -0.005
    assert _dz(0.0, 0.08) == 0.0
    # Full travel is still reachable, both ways.
    assert _dz(1.0, 0.08) == 1.0
    assert _dz(-1.0, 0.08) == -1.0


def test_reverse_cap_limits_reverse_on_a_mirrored_motor():
    """max_reverse must follow the ROVER's direction, not the ESC's.

    Applied after inversion it capped the mirrored track going forward while
    the other ran uncapped — a left/right power offset produced by a knob that
    says it limits reverse.
    """
    left = ESCMotor(MotorConfig(channel=0, inverted=False, max_reverse=0.5))
    right = ESCMotor(MotorConfig(channel=1, inverted=True, max_reverse=0.5))

    left.set_throttle(1.0)
    right.set_throttle(1.0)
    # Full forward: neither side is capped, and they are mirrored about neutral.
    n = left.cfg.neutral_angle
    assert left.servo._last - n == -(right.servo._last - n)
    assert abs(left.servo._last - n) == left.cfg.max_angle - n

    left.set_throttle(-1.0)
    right.set_throttle(-1.0)
    # Full reverse: both capped by the same half, still mirrored.
    assert left.servo._last - n == -(right.servo._last - n)
    assert abs(left.servo._last - n) == 0.5 * (left.cfg.max_angle - n)


# --- acceleration and deceleration, separately --------------------------------
#
# One rate forces a bad trade: slow enough for a gentle start is also slow
# enough for a sluggish stop, and stopping is the one you want crisp. These pin
# down that the two are independent, that a layout written before `decel_rate`
# existed still limits symmetrically, and above all that the e-stop is not
# rate-limited by either of them.

import time  # noqa: E402

import pytest  # noqa: E402

from robot.drive.drivetrain import _SlewLimiter  # noqa: E402


def step(lim, targets, dt):
    """Advance the limiter by `dt` of pretend time."""
    lim._last = time.monotonic() - dt
    return lim.apply(targets)


def test_pulling_away_from_zero_uses_the_accel_rate():
    lim = _SlewLimiter(2.0, 1, 10.0)
    lim._current = [0.0]
    assert step(lim, [1.0], 0.1) == pytest.approx([0.2], abs=1e-3)   # 2.0/s for 0.1s


def test_coming_back_toward_zero_uses_the_decel_rate():
    lim = _SlewLimiter(2.0, 1, 10.0)
    lim._current = [1.0]
    assert step(lim, [0.0], 0.1) == pytest.approx([0.0])   # 10.0/s covers it


def test_a_firm_brake_does_not_make_the_accelerator_firm():
    """The whole point: these must not be the same number."""
    lim = _SlewLimiter(1.0, 1, 20.0)
    lim._current = [0.0]
    assert step(lim, [1.0], 0.1) == pytest.approx([0.1], abs=1e-3)   # gentle away
    lim._current = [1.0]
    assert step(lim, [0.0], 0.1) == pytest.approx([0.0])   # immediate back


def test_zero_decel_rate_means_symmetric_which_is_the_old_behaviour():
    """Every layout written before this field existed has decel_rate 0, and
    must be bit-for-bit unchanged."""
    old = _SlewLimiter(4.0, 1)
    new = _SlewLimiter(4.0, 1, 0.0)
    for current, target in ((0.0, 1.0), (1.0, 0.0), (0.5, -0.5), (-1.0, 1.0)):
        old._current = [current]
        new._current = [current]
        a = step(old, [target], 0.05)[0]
        b = step(new, [target], 0.05)[0]
        assert a == pytest.approx(b, abs=1e-3)


def test_a_zero_crossing_brakes_first_then_accelerates():
    """0.5 -> -0.3 is nearer zero in magnitude, so it is braking until it
    passes through; reversing is not a special case."""
    lim = _SlewLimiter(1.0, 1, 10.0)
    lim._current = [0.5]
    # Decel rate governs while |target| < |current|, so the fast rate applies.
    assert step(lim, [-0.3], 0.1) == pytest.approx([-0.3])
    # ...but growing the magnitude in reverse is acceleration again.
    lim = _SlewLimiter(1.0, 1, 10.0)
    lim._current = [-0.3]
    assert step(lim, [-1.0], 0.1) == pytest.approx([-0.4], abs=1e-3)


def test_rate_zero_still_disables_limiting_entirely():
    lim = _SlewLimiter(0.0, 1, 5.0)
    lim._current = [0.0]
    assert step(lim, [1.0], 0.001) == [1.0]


def test_both_channels_are_limited_independently():
    """One track braking while the other pulls away is an ordinary turn."""
    lim = _SlewLimiter(1.0, 2, 10.0)
    lim._current = [0.0, 1.0]
    left, right = step(lim, [1.0, 0.0], 0.1)
    assert left == pytest.approx(0.1, abs=1e-3)    # accelerating
    assert right == pytest.approx(0.0)   # braking
