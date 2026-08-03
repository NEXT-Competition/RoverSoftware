"""Camera + ultrasonic: measuring a distance, and learning to infer one.

The two sensors fail in opposite directions — the camera sees a named object
anywhere in a wide frame but only reports a box height, the sonar measures a
real distance to whatever is in a narrow cone and has no idea what it is — so
the whole design is about the conditions under which a pair of readings may be
believed to be about the SAME OBJECT.

That is what most of these tests are: the gates. A bad pair is worse than no
pair, because a wrong number that looks measured is one an operator will steer
by, a routine will stop on, and `spin_up` will turn into a flywheel speed.
"""

from typing import Optional

import pytest

from robot.config import RobotConfig
from robot.control.commands import DriveCommand
from robot.control.detection import Detection
from robot.control.object_align import ObjectAlignController
from robot.control.rangefinder import Rangefinder

STAMP = 1000.0


def seen(size: Optional[float] = 0.4, error_x=0.0, error_y=0.0,
         label="bucket", stamp=STAMP):
    """One sighting. `size=None` is a FOMO model: a centroid with no box."""
    return Detection(label=label, confidence=0.9, error_x=error_x,
                     error_y=error_y, size=size, stamp=stamp)


def sonar(distance, stamp=STAMP):
    """A `() -> (metres, stamp)` provider, as sensors/ultrasonic.stamped_m is."""
    return lambda: None if distance is None else (distance, stamp)


def fitted(distance=1.0, size=0.4, samples=8, **kw):
    """A rangefinder taught `samples` clean pairs. k = distance * size.

    Frames a hundredth of a second apart, so every one of them still pairs with
    the sonar sample — the skew gate is 0.15 s, and a helper that quietly
    violated it would be testing the gate rather than the fit.
    """
    r = Rangefinder(hfov_deg=50.0, min_samples=samples, **kw)
    r.set_sonar_provider(sonar(distance))
    for i in range(samples):
        assert r.observe(seen(size=size, stamp=STAMP + i * 0.01)) is True
    return r


# --- can the sonar possibly be looking at this detection? --------------------

def test_a_centred_target_inside_the_cone_pairs():
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.2))
    assert r.sonar_for(seen(error_x=0.0)) == pytest.approx(1.2)


def test_a_target_outside_the_beam_does_not():
    """error_x is normalised over the FOV, so this is arithmetic: at hfov 50,
    the frame edge is 25 degrees off axis and the sonar's cone is 7.5, so
    anything past error_x 0.3 is echoing off something else entirely."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.2))
    assert r.sonar_for(seen(error_x=0.29)) == pytest.approx(1.2)
    assert r.sonar_for(seen(error_x=0.31)) is None


def test_a_wider_lens_narrows_the_usable_fraction_of_the_frame():
    """Same cone, more frame: on the AI Camera's 66 degrees the same normalised
    offset is further off axis, so the gate has to tighten on its own."""
    r = Rangefinder(hfov_deg=66.0)
    r.set_sonar_provider(sonar(1.2))
    assert r.sonar_for(seen(error_x=0.22)) is not None
    assert r.sonar_for(seen(error_x=0.24)) is None


def test_readings_from_a_different_moment_do_not_pair():
    """The detector runs at ~10 fps and the sonar at ~15 Hz on unrelated clocks.
    A reading half a second after the frame was classified describes a different
    moment — and on a moving rover, a different distance."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.2, stamp=STAMP + 0.5))
    assert r.sonar_for(seen()) is None
    r.set_sonar_provider(sonar(1.2, stamp=STAMP + 0.05))
    assert r.sonar_for(seen()) is not None


def test_no_echo_is_not_a_distance():
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(None))
    assert r.sonar_for(seen()) is None


def test_no_sonar_at_all_pairs_nothing():
    assert Rangefinder(hfov_deg=50.0).sonar_for(seen()) is None


# --- learning the constant ---------------------------------------------------

def test_pairs_teach_the_box_height_constant():
    """The whole point: k = distance x size, measured rather than typed. A
    target at 1.2 m whose box fills 0.4 of the frame gives k = 0.48, so the same
    box at half the size is twice as far away."""
    r = fitted(distance=1.2, size=0.4)
    assert r.k_for("bucket") == pytest.approx(0.48)
    assert r.distance_m(0.2, "bucket") == pytest.approx(2.4)


def test_a_fit_is_not_believed_until_it_has_enough_samples():
    r = Rangefinder(hfov_deg=50.0, min_samples=5)
    r.set_sonar_provider(sonar(1.0))
    for i in range(4):
        r.observe(seen(size=0.4, stamp=STAMP + i * 0.01))
    assert r.k_for("bucket") is None          # nothing to fall back on either
    r.observe(seen(size=0.4, stamp=STAMP + 0.09))
    assert r.k_for("bucket") == pytest.approx(0.4)


def test_the_constant_is_per_label_because_k_folds_in_object_height():
    """A cone and a bucket at the same distance give different boxes. One
    constant for both is exactly the error this table exists to avoid."""
    r = Rangefinder(hfov_deg=50.0, min_samples=2)
    r.set_sonar_provider(sonar(1.0))
    r.observe(seen(size=0.4, label="bucket", stamp=STAMP + 0.01))
    r.observe(seen(size=0.4, label="bucket", stamp=STAMP + 0.02))
    r.observe(seen(size=0.8, label="cone", stamp=STAMP + 0.03))
    r.observe(seen(size=0.8, label="cone", stamp=STAMP + 0.04))
    assert r.k_for("bucket") == pytest.approx(0.4)
    assert r.k_for("cone") == pytest.approx(0.8)


def test_an_unseen_label_falls_back_to_the_hand_set_pair_not_another_label():
    r = fitted(distance=1.0, size=0.4)        # 'bucket' learned, k = 0.4
    r.calibrate(2.0, 0.25)                    # hand-set k = 0.5
    assert r.k_for("bucket") == pytest.approx(0.4)
    assert r.k_for("cone") == pytest.approx(0.5)


def test_recalibrating_by_hand_does_not_throw_away_measurements():
    """The learned fit is a measurement; the pair is a fallback for labels that
    have none. Dropping the former because someone nudged the latter would be
    backwards."""
    r = fitted(distance=1.0, size=0.4)
    r.calibrate(3.0, 0.3)
    assert r.k_for("bucket") == pytest.approx(0.4)


def test_the_median_shrugs_off_a_bad_sample_that_gets_through():
    r = Rangefinder(hfov_deg=50.0, min_samples=3)
    for i, distance in enumerate([1.0, 1.0, 1.4, 1.0, 1.0]):
        r.set_sonar_provider(sonar(distance, stamp=STAMP + i * 0.01))
        r.observe(seen(size=0.4, stamp=STAMP + i * 0.01))
    assert r.k_for("bucket") == pytest.approx(0.4)


# --- what is refused ---------------------------------------------------------

def test_a_clipped_box_is_refused():
    """The box spans error_y +/- size, so this one runs off the bottom of the
    frame. Its height was cut off, and an understated height reads as a target
    further away than it is."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen(size=0.4, error_y=0.7)) is False
    assert r.observe(seen(size=0.4, error_y=0.3, stamp=STAMP + 0.01)) is True


def test_a_box_filling_the_frame_is_refused():
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(0.3))
    assert r.observe(seen(size=0.9)) is False


def test_learning_while_driving_is_refused():
    """A matched pair still carries the vision pipeline's own latency, which is
    unknown and one-signed — the frame is always older than its classification.
    That is a BIAS, and a median cannot filter out a bias."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen(), throttle=0.9) is False
    assert r.observe(seen(), throttle=0.2) is True   # a creeping approach is fine


def test_a_target_off_axis_teaches_nothing():
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen(error_x=0.6)) is False


def test_a_model_with_no_box_height_teaches_nothing():
    """FOMO. There is no size to pair the distance WITH."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen(size=None)) is False


def test_one_sample_per_frame_however_fast_the_loop_runs():
    """The control loop is 50 Hz and the detector ~10 — without this the same
    frame would be counted five times and the window would fill with copies of
    one measurement wearing the clothes of five."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen()) is True
    assert r.observe(seen()) is False
    assert r.observe(seen(stamp=STAMP + 0.01)) is True


def test_a_frame_whose_sonar_has_not_arrived_yet_is_retried():
    """The sensors are on unrelated clocks, so 'no reading on this tick' is not
    'no reading for this frame'. The frame is only spent once it pairs."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(None))
    assert r.observe(seen()) is False
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen()) is True          # same stamp, and it still counts


def test_a_wildly_disagreeing_sample_is_treated_as_a_nearer_object():
    """Once a fit exists, a pair implying a very different k is far more likely
    to be the sonar finding a chair leg in front of the bucket than a sudden
    change in the laws of optics."""
    r = fitted(distance=1.0, size=0.4)        # k = 0.4
    r.set_sonar_provider(sonar(0.1, stamp=STAMP + 99))
    assert r.observe(seen(size=0.4, stamp=STAMP + 99)) is False
    assert r.k_for("bucket") == pytest.approx(0.4)


def test_learning_can_be_switched_off():
    r = Rangefinder(hfov_deg=50.0, learn=False)
    r.set_sonar_provider(sonar(1.0))
    assert r.observe(seen()) is False


def test_it_says_once_when_a_label_becomes_calibrated(capsys):
    r = fitted(distance=1.0, size=0.4, samples=8)
    out = capsys.readouterr().out
    assert out.count("calibrated 'bucket'") == 1
    # The pair to write down, because a learned fit lives in memory.
    assert "vision.range_at_m" in out
    r.set_sonar_provider(sonar(1.0, stamp=STAMP + 50))
    r.observe(seen(size=0.4, stamp=STAMP + 50))
    assert "calibrated 'bucket'" not in capsys.readouterr().out


# --- which sensor answers ----------------------------------------------------

def test_a_measurement_beats_an_inference():
    """Both are available here and they disagree; the transducer wins."""
    r = Rangefinder(1.0, 0.45, hfov_deg=50.0)      # hand-set k = 0.45
    r.set_sonar_provider(sonar(0.8))
    assert r.distance_for(seen(size=0.45)) == pytest.approx(0.8)
    assert r.source == "sonar"


def test_out_of_the_beam_it_falls_back_to_the_box_height():
    r = Rangefinder(1.0, 0.45, hfov_deg=50.0)
    r.set_sonar_provider(sonar(0.8))
    assert r.distance_for(seen(size=0.45, error_x=0.9)) == pytest.approx(1.0)
    assert r.source == "vision"


def test_past_the_sonars_range_the_camera_carries_on_alone():
    """The point of learning: the sonar reaches a few metres, and the fit it
    leaves behind is what answers everywhere beyond that."""
    r = fitted(distance=1.2, size=0.4)             # k = 0.48, learned
    r.set_sonar_provider(sonar(None))              # nothing within range now
    assert r.distance_for(seen(size=0.06)) == pytest.approx(8.0)
    assert r.source == "vision"


def test_a_fomo_model_gets_a_distance_for_the_first_time():
    """No box height at all, so vision cannot answer and never could. The sonar
    can, and this is what unlocks approach and standoff on those models."""
    r = Rangefinder(1.0, 0.45, hfov_deg=50.0)
    r.set_sonar_provider(sonar(0.7))
    assert r.distance_for(seen(size=None)) == pytest.approx(0.7)
    assert r.source == "sonar"


def test_preferring_the_sonar_can_be_switched_off():
    r = Rangefinder(1.0, 0.45, hfov_deg=50.0, prefer_sonar=False)
    r.set_sonar_provider(sonar(0.8))
    assert r.distance_for(seen(size=0.45)) == pytest.approx(1.0)


def test_nothing_in_view_is_no_distance_and_no_source():
    r = fitted()
    assert r.distance_for(None) is None
    assert r.source == ""


def test_status_reports_the_source_and_the_sample_count():
    r = fitted(distance=1.0, size=0.4, samples=8)
    r.distance_for(seen(size=0.4))
    assert r.status("bucket") == {"src": "s", "kn": 8}


# --- through the controller --------------------------------------------------

def align(rangefinder, detection, **kw):
    c = ObjectAlignController(detection_provider=lambda: detection,
                              rangefinder=rangefinder, **kw)
    c.on_activate()
    return c


def test_a_metre_standoff_is_judged_on_the_measurement_when_there_is_one():
    r = Rangefinder(1.0, 0.45, hfov_deg=50.0)
    r.set_sonar_provider(sonar(0.9))
    c = align(r, seen(size=0.45), standoff_m=0.5)
    assert c.update(0.02) != DriveCommand.stopped()   # 0.9 m away: keep coming
    r.set_sonar_provider(sonar(0.45))
    assert c.update(0.02) == DriveCommand.stopped()
    assert c.arrived()


def test_a_fomo_model_can_now_approach_and_stop():
    """It could only ever face the target before: no box height meant no way to
    know when to stop, so it correctly refused to advance at all."""
    r = Rangefinder(hfov_deg=50.0)                    # no hand calibration
    r.set_sonar_provider(sonar(1.5))
    c = align(r, seen(size=None), standoff_m=0.5)
    assert c.update(0.02).left > 0                    # advancing, on sound alone
    r.set_sonar_provider(sonar(0.4))
    assert c.update(0.02) == DriveCommand.stopped()
    assert c.arrived()


def test_a_fomo_model_with_no_sonar_still_refuses_to_advance():
    """The old behaviour, and it must survive: never drive blind at something."""
    c = align(Rangefinder(hfov_deg=50.0), seen(size=None), standoff_m=0.5)
    assert c.update(0.02) == DriveCommand.stopped()


def test_the_metre_arrival_latch_has_hysteresis():
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(0.4))
    c = align(r, seen(size=None), standoff_m=0.5, standoff_hysteresis_m=0.1)
    c.update(0.02)
    assert c.arrived()
    r.set_sonar_provider(sonar(0.55))       # drifted, but not meaningfully
    c.update(0.02)
    assert c.arrived()
    r.set_sonar_provider(sonar(0.7))        # genuinely receding
    c.update(0.02)
    assert not c.arrived()


def test_losing_the_measurement_does_not_un_arrive_the_robot():
    """Having stopped for a reason and then lost sight of the reason is not
    grounds to drive forward again."""
    r = Rangefinder(hfov_deg=50.0)          # nothing can judge box heights here
    r.set_sonar_provider(sonar(0.4))
    c = align(r, seen(size=None), standoff_m=0.5)
    c.update(0.02)
    assert c.arrived()
    r.set_sonar_provider(sonar(None))
    assert c.update(0.02) == DriveCommand.stopped()
    assert c.arrived()


def test_a_standoff_the_collision_guard_will_not_allow_is_called_out(capsys):
    """The symptom otherwise is a routine state that times out every time, for
    no visible reason, nowhere near the cause."""
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    c = align(r, seen(size=0.4), standoff_m=0.2)
    c.set_min_standoff(0.35)
    c.update(0.02)
    out = capsys.readouterr().out
    assert "collision avoidance holds at 0.35" in out
    c.update(0.02)
    assert "collision avoidance holds" not in capsys.readouterr().out


def test_a_standoff_beyond_the_guard_says_nothing(capsys):
    r = Rangefinder(hfov_deg=50.0)
    r.set_sonar_provider(sonar(1.0))
    c = align(r, seen(size=0.4), standoff_m=0.8)
    c.set_min_standoff(0.35)
    c.update(0.02)
    assert "collision avoidance" not in capsys.readouterr().out


# --- through the robot -------------------------------------------------------

@pytest.fixture
def rover(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_MOCK_DETECTOR", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = False
    cfg.drive.arm_seconds = 0.0
    cfg.ultrasonic.enabled = True
    from robot.robot import Robot
    return Robot(cfg)


def test_the_robot_hands_the_sonar_to_the_rangefinder(rover):
    """Stamped, not bare metres: pairing a distance with a frame needs to know
    when the distance was measured."""
    assert rover.rangefinder._sonar == rover.ultrasonic.stamped_m


def test_a_build_with_no_ultrasonic_wires_nothing(monkeypatch, tmp_path):
    monkeypatch.setenv("RS_MOCK_MOTORS", "1")
    monkeypatch.setenv("RS_TUNING_FILE", str(tmp_path / "tuning.json"))
    cfg = RobotConfig()
    cfg.gps.enabled = cfg.imu.enabled = False
    cfg.camera.enabled = cfg.vision.enabled = False
    cfg.drive.arm_seconds = 0.0
    from robot.robot import Robot
    bot = Robot(cfg)
    assert bot.rangefinder._sonar is None
    bot._learn_range(DriveCommand.stopped())      # must be a no-op, not a crash


def test_the_loop_learns_from_what_the_detector_and_the_sonar_agree_on(rover):
    """The tick's worth of it: `_learn_range` is what Robot.run calls after the
    motors, and this is that call with a target sitting in front of a sonar."""
    rover.rangefinder.set_sonar_provider(sonar(1.0, stamp=STAMP))
    monkeypatched = seen(size=0.4, stamp=STAMP)
    rover.detector.detection = lambda: monkeypatched
    rover._learn_range(DriveCommand.stopped())
    assert rover.rangefinder.learned("bucket") == (pytest.approx(0.4), 1)


def test_the_distance_and_its_source_reach_telemetry(rover):
    rover.rangefinder.set_sonar_provider(sonar(0.75, stamp=STAMP))
    rover.detector.detection = lambda: seen(size=0.4, stamp=STAMP)
    vision = rover._telemetry(DriveCommand.stopped())["vision"]
    assert vision["dist"] == pytest.approx(0.75)
    assert vision["src"] == "s"                   # measured, not inferred


def test_the_switches_are_live(rover):
    rover._set_config({"config": {"vision.sonar_range": False,
                                  "vision.auto_range": False}, "save": False})
    assert rover.rangefinder.prefer_sonar is False
    assert rover.rangefinder.learn is False


def test_the_guard_teaches_the_controllers_how_near_they_may_finish(rover):
    align_c = rover.manager.controllers["object_align"]
    assert align_c.min_standoff_m == pytest.approx(rover.cfg.ultrasonic.stop_m)
    rover._set_config({"config": {"ultrasonic.avoid": False}, "save": False})
    assert align_c.min_standoff_m == 0.0
