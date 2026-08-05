"""The flywheel tachometer, and what it lets the shooter's PID finally do.

Two things are worth pinning. The measurement itself — speed is the time spanned
by N pulses, and a stale reading must decay to zero rather than linger. And the
consequences for the controller: with a real sensor the loop trims against a
real error, and the stall guard becomes possible at all.

No GPIO here: the pins are faked, which is the whole reason the poll loop takes
its readings through an object rather than calling the library directly.
"""

import os
import time

import pytest

os.environ.setdefault("RS_MOCK_MOTORS", "1")

from robot.config import EncoderConfig, ShooterConfig
from robot.drive.shooter import Shooter
from robot.sensors.encoder import FlywheelEncoder


def make_encoder(**kw):
    kw.setdefault("pulses_per_rev", 16)
    kw.setdefault("window_pulses", 8)
    return FlywheelEncoder(EncoderConfig(enabled=True, **kw))


def feed_pulses(enc, count, interval_s, now=None):
    """Push pulse timestamps in directly, as the poll thread would."""
    now = now if now is not None else time.monotonic()
    for i in range(count):
        enc._pulse_times.append(now - (count - 1 - i) * interval_s)
    enc._pulses += count


# --- the measurement ---------------------------------------------------------

def test_no_pulses_reads_zero():
    assert make_encoder().rpm() == 0.0


def test_a_single_pulse_is_not_a_speed():
    """One timestamp spans no time; a speed needs two."""
    enc = make_encoder()
    feed_pulses(enc, 1, 0.001)
    assert enc.rpm() == 0.0


def test_rpm_is_the_time_spanned_by_the_pulse_window():
    """8 timestamps 1 ms apart span 7 ms and 7 pulses; at 16 pulses per rev
    that is 7/16 of a revolution in 7 ms = 3750 RPM. Timing a fixed number of
    pulses rather than counting pulses in a fixed window is what keeps this off
    a ~19 RPM quantisation grid."""
    enc = make_encoder()
    feed_pulses(enc, 8, 0.001)
    assert enc.rpm() == pytest.approx(3750.0, rel=1e-3)


def test_a_stale_reading_decays_to_zero_rather_than_lingering():
    """A stopped wheel and a dead poll thread must both read 0. The shooter's
    stall guard depends on it: a lingering speed would mask a jam."""
    enc = make_encoder(stale_seconds=0.05)
    feed_pulses(enc, 8, 0.001, now=time.monotonic() - 1.0)
    assert enc.rpm() == 0.0


def test_pulses_per_rev_scales_the_reading_linearly():
    """The direction of this error is what matters: a PPR set too LOW makes the
    wheel read faster than it is; too high and it under-reports, the controller
    pushes harder, and the wheel ends up quicker than the display says."""
    slow = make_encoder(pulses_per_rev=32)
    fast = make_encoder(pulses_per_rev=16)
    for enc in (slow, fast):
        feed_pulses(enc, 8, 0.001)
    assert fast.rpm() == pytest.approx(2 * slow.rpm())


def test_telemetry_reports_the_poll_rate():
    """A starved poll thread and a stopped wheel both read 0 RPM, and only one
    is a problem. The rate is how you tell them apart."""
    enc = make_encoder()
    assert set(enc.telemetry()) >= {"ok", "rpm", "pulses", "poll_khz"}


def test_it_runs_without_gpio_and_reports_nothing_turning():
    """On a laptop the pins are mocked flat, deliberately: a fake that spun up
    would make the PID look healthy where the one thing worth knowing is that
    there is no encoder."""
    enc = make_encoder()
    enc.start()
    try:
        time.sleep(0.05)
        assert enc.rpm() == 0.0
    finally:
        enc.stop()


# --- what the sensor changes for the controller ------------------------------

def cfg(**kw):
    return ShooterConfig(enabled=True, target_rpm=3400.0, **kw)


def test_a_real_reading_takes_over_from_the_model_for_good():
    s = Shooter(cfg())
    s.set_target_rpm(3400.0)
    assert not s._pid_has_sensor
    s.set_measured_rpm(1200.0)
    assert s._pid_has_sensor
    s.update()
    # The model would have supplied 3400 (zero error); the sensor says 1200.
    assert s._pid_measured_rpm == 1200.0


def test_an_underspeed_wheel_gets_more_throttle_than_feed_forward_alone():
    """The point of closing the loop: a wheel dragged down by a ball or a sagging
    battery is pushed back up, which open-loop cannot do."""
    s = Shooter(cfg())
    s.set_target_rpm(3400.0)
    s.set_measured_rpm(3400.0)
    s.update()
    on_target = s.pid_throttle

    s2 = Shooter(cfg())
    s2.set_target_rpm(3400.0)
    for _ in range(5):
        s2.set_measured_rpm(2800.0)   # dragged down
        s2._pid_last_control = 0.0    # let each update run a control step
        s2.update()
    assert s2.pid_throttle > on_target


def test_the_target_is_capped_against_a_failing_encoder():
    s = Shooter(cfg(max_target_rpm=4000.0))
    s.set_target_rpm(9000.0)
    assert s._pid_target_rpm == 4000.0


# --- the stall guard ---------------------------------------------------------

def test_commanded_but_not_turning_cuts_power():
    """A jam, a dead ESC and an unplugged encoder all land here. Without this
    the controller reads zero, calls it a huge error, and pushes harder against
    something that is not moving."""
    s = Shooter(cfg(stall_seconds=0.5))
    s.set_target_rpm(3400.0)
    s.set_measured_rpm(0.0)
    s._pid_moving_at = time.monotonic() - 1.0   # stalled for longer than allowed
    s._pid_last_control = 0.0
    s.update()
    assert not s.pid_active
    assert s._pid_target_rpm == 0.0
    assert s.status()["stalled"] is True


def test_a_turning_wheel_never_trips_the_stall_guard():
    s = Shooter(cfg(stall_seconds=0.5))
    s.set_target_rpm(3400.0)
    for _ in range(5):
        s.set_measured_rpm(3300.0)
        s._pid_last_control = 0.0
        s.update()
    assert s.pid_active


def test_an_open_loop_build_cannot_stall_guard():
    """_estimated_rpm returns the target, so a build with no sensor always looks
    like it is turning. That is a limitation of having no encoder, not a reason
    to fake one — and it is why the guard is gated on _pid_has_sensor."""
    s = Shooter(cfg(stall_seconds=0.5))
    s.set_target_rpm(3400.0)
    s._pid_moving_at = time.monotonic() - 10.0
    s._pid_last_control = 0.0
    s.update()               # no set_measured_rpm ever called
    assert s.pid_active

def test_pressing_spin_again_clears_a_stall_trip():
    s = Shooter(cfg(stall_seconds=0.5))
    s.spin(True)
    s.set_measured_rpm(0.0)
    s._pid_moving_at = time.monotonic() - 1.0
    s._pid_last_control = 0.0
    s.update()
    assert s.status()["stalled"] is True
    s.spin(True)
    assert s.status()["stalled"] is False


# --- the competition limit ---------------------------------------------------

def test_over_the_legal_rim_speed_is_marked_not_refused():
    """Rule 5.5 caps a launch at 12.0 m/s (~3008 RPM on this 3in wheel). Tuning
    above it on blocks is legitimate, so it is flagged rather than clamped — a
    silent cap is how you end up trusting a number that was never applied."""
    s = Shooter(cfg())
    s.set_target_rpm(3400.0)
    s.set_measured_rpm(3400.0)
    s.update()
    st = s.status()
    assert st["over"] is True
    assert st["mps"] == pytest.approx(13.57, abs=0.05)

    s.set_measured_rpm(2500.0)
    s.update()
    assert s.status()["over"] is False


def test_status_says_whether_the_loop_is_actually_closed():
    """A healthy display on a robot with no encoder looks identical to a healthy
    display on one with a working loop, unless this says so."""
    s = Shooter(cfg())
    s.set_target_rpm(3400.0)
    s.update()
    assert s.status()["sensor"] is False
    s.set_measured_rpm(3000.0)
    assert s.status()["sensor"] is True
