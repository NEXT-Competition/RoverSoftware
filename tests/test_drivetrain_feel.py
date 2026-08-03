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
