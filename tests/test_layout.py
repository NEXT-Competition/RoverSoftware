"""The hardware layout document.

This is the only thing standing between a browser and how the rover is wired, so
the interesting cases are all about malformed input: a name that collides with a
config field, two actuators on one PWM channel, a document too big for the
radio, a file someone hand-edited badly. None of it may raise, and none of it
may leave the robot un-bootable.
"""

import json

from robot import layout
from robot.config import RobotConfig


def tank_doc(**drive):
    doc = layout.default_doc()
    doc["drive"].update(drive)
    return doc


# --- the default -------------------------------------------------------------

def test_the_default_document_describes_the_compiled_in_robot():
    """default_doc() is built from RobotConfig() rather than written out by
    hand, so it cannot drift from the defaults it claims to describe."""
    cfg = RobotConfig()
    doc = layout.default_doc()
    assert doc["drive"]["kind"] == "tank"
    channels = {a["name"]: a["channel"] for a in doc["drive"]["actuators"]}
    assert channels == {"left": 0, "right": 1}
    assert doc["drive"]["roles"]["left"] == ["left"]
    assert cfg.drive.actuators["right"].inverted is True


def test_the_default_document_round_trips():
    cfg = RobotConfig()
    result = layout.apply(cfg, layout.default_doc())
    assert result.ok, result.errors
    assert layout.to_doc(cfg) == layout.default_doc()


def test_the_default_document_fits_comfortably_on_the_radio():
    encoded = len(json.dumps(layout.default_doc(), separators=(",", ":")))
    assert encoded < layout.MAX_DOC_BYTES


# --- names -------------------------------------------------------------------

def test_duplicate_actuator_names_are_rejected():
    doc = tank_doc()
    doc["drive"]["actuators"].append(
        {"name": "left", "kind": "esc", "channel": 5})
    assert "duplicate" in " ".join(layout.validate(doc).errors)


def test_a_name_that_collides_with_a_drivetrain_field_is_rejected():
    """`slew_rate` would be unreachable through DriveConfig.__getattr__ because
    the real field wins — better a message than a silent mystery."""
    doc = tank_doc()
    doc["drive"]["actuators"][0]["name"] = "slew_rate"
    doc["drive"]["roles"]["left"] = ["slew_rate"]
    assert "reserved" in " ".join(layout.validate(doc).errors)


def test_malformed_names_are_rejected():
    for bad in ("Left", "9lives", "has space", "", "x" * 30):
        doc = tank_doc()
        doc["drive"]["actuators"][0]["name"] = bad
        assert not layout.validate(doc).ok, f"{bad!r} should be refused"


def test_a_mechanism_may_not_be_called_shooter():
    """The built-in launcher keeps its own ShooterConfig and `shooter.*` paths;
    two things answering to the name is how a routine fires the wrong one."""
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "shooter", "kind": "pulse",
                          "actuators": [{"name": "arm", "channel": 7}]}]
    assert "reserved" in " ".join(layout.validate(doc).errors)


# --- channels ----------------------------------------------------------------

def test_two_actuators_on_one_channel_are_rejected():
    """Previously prevented by a comment and nothing else. Two ESCs on one
    channel move together and neither answers its own commands, which reads as a
    wiring fault and is not one."""
    doc = tank_doc()
    doc["drive"]["actuators"][1]["channel"] = 0
    errors = " ".join(layout.validate(doc).errors)
    assert "channel 0" in errors


def test_the_drivetrain_wins_a_channel_fight_and_the_mechanism_is_disabled():
    """A robot that can't drive can't be driven away from what it is about to
    hit, so the drivetrain claims first and the loser is disabled, not fatal."""
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "intake", "kind": "power",
                          "actuators": [{"name": "roller", "channel": 0}]}]
    result = layout.validate(doc)
    assert not result.ok
    assert result.mechanisms["intake"].enabled is False
    assert result.drive.actuators["left"].channel == 0


def test_the_built_in_shooters_channel_is_reserved():
    cfg = RobotConfig()
    cfg.shooter.enabled = True
    cfg.shooter.channel = 6
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "intake", "kind": "power",
                          "actuators": [{"name": "roller", "channel": 6}]}]
    result = layout.apply(cfg, doc)
    assert "the built-in shooter" in " ".join(result.errors)


def test_channels_outside_the_hats_range_are_rejected():
    for bad in (-1, 16, 99):
        doc = tank_doc()
        doc["drive"]["actuators"][0]["channel"] = bad
        assert not layout.validate(doc).ok


# --- drivetrain kinds and roles ---------------------------------------------

def test_an_unknown_drivetrain_kind_is_rejected():
    assert not layout.validate(tank_doc(kind="mecanum")).ok


def test_a_role_naming_an_actuator_that_does_not_exist_is_rejected():
    doc = tank_doc()
    doc["drive"]["roles"]["left"] = ["nope"]
    assert "no actuator named" in " ".join(layout.validate(doc).errors)


def test_a_tank_needs_both_sides():
    doc = tank_doc()
    doc["drive"]["roles"]["right"] = []
    assert not layout.validate(doc).ok


def test_an_actuator_cannot_be_on_both_sides():
    doc = tank_doc()
    doc["drive"]["roles"]["right"] = ["left"]
    assert "both sides" in " ".join(layout.validate(doc).errors)


def test_a_servo_steer_layout_needs_a_steering_servo():
    doc = {"version": 1, "drive": {
        "kind": "servo_steer",
        "actuators": [{"name": "rear", "kind": "esc", "channel": 0}],
        "roles": {"throttle": ["rear"]}}}
    assert "steering servo" in " ".join(layout.validate(doc).errors)


def test_a_servo_steer_actuator_cannot_both_steer_and_drive():
    doc = {"version": 1, "drive": {
        "kind": "servo_steer",
        "actuators": [{"name": "rear", "kind": "esc", "channel": 0}],
        "roles": {"throttle": ["rear"], "steer": "rear"}}}
    assert "cannot both steer and drive" in " ".join(layout.validate(doc).errors)


def test_a_valid_servo_steer_layout_is_accepted():
    cfg = RobotConfig()
    doc = {"version": 1, "drive": {
        "kind": "servo_steer",
        "actuators": [{"name": "rear", "kind": "esc", "channel": 0},
                      {"name": "steering", "kind": "servo", "channel": 3}],
        "roles": {"throttle": ["rear"], "steer": "steering"}}}
    result = layout.apply(cfg, doc)
    assert result.ok, result.errors
    assert cfg.drive.kind == "servo_steer"
    assert cfg.drive.roles.steer == "steering"


# --- mechanisms --------------------------------------------------------------

def test_a_mechanism_needs_at_least_one_actuator():
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "intake", "kind": "power", "actuators": []}]
    assert not layout.validate(doc).ok


def test_a_preset_naming_an_unknown_actuator_is_rejected():
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "intake", "kind": "power",
                          "actuators": [{"name": "roller", "channel": 4}],
                          "presets": {"in": {"belt": 1.0}}}]
    assert "unknown actuator" in " ".join(layout.validate(doc).errors)


def test_preset_values_are_clamped_rather_than_refused():
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "intake", "kind": "power",
                          "actuators": [{"name": "roller", "channel": 4}],
                          "presets": {"in": {"roller": 4.0}}}]
    result = layout.validate(doc)
    assert result.ok, result.errors
    assert result.mechanisms["intake"].presets["in"]["roller"] == 1.0


def test_actuator_numbers_are_clamped_rather_than_refused():
    """Same rule as tuning.apply: a value pinned at its limit is honest, and
    dropping a whole layout over one mistyped endpoint is not."""
    doc = tank_doc()
    doc["drive"]["actuators"][0]["max_angle"] = 500
    result = layout.validate(doc)
    assert result.ok, result.errors
    assert result.drive.actuators["left"].max_angle == 90


def test_a_pulse_mechanism_keeps_its_geometry():
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "kicker", "kind": "pulse",
                          "actuators": [{"name": "arm", "kind": "servo", "channel": 7}],
                          "rest_angle": -25, "active_angle": 25,
                          "active_seconds": 0.2, "cooldown": 1.5}]
    result = layout.validate(doc)
    assert result.ok, result.errors
    kicker = result.mechanisms["kicker"]
    assert (kicker.rest_angle, kicker.active_angle) == (-25, 25)
    assert kicker.cooldown == 1.5


# --- caps and junk -----------------------------------------------------------

def test_an_oversized_document_is_refused():
    doc = tank_doc()
    doc["padding"] = "x" * layout.MAX_DOC_BYTES
    assert "at most" in " ".join(layout.validate(doc).errors)


def _mechs(count):
    """`count` power mechanisms on DISTINCT channels, starting past the drive.

    Distinct matters: every one of these on channel 2 is refused for a channel
    conflict whether or not the count cap exists, so the old version of this
    test passed for the wrong reason and would have kept passing if somebody
    deleted the cap it was named after.
    """
    return [{"name": f"m{i}", "kind": "power",
             "actuators": [{"name": "a", "channel": 2 + i}]}
            for i in range(count)]


def test_too_many_mechanisms_are_refused():
    doc = tank_doc()
    doc["mechanisms"] = _mechs(layout.MAX_MECHANISMS + 1)
    result = layout.validate(doc)
    assert not result.ok
    assert "at most" in " ".join(result.errors)


def test_exactly_the_cap_is_accepted():
    """The other half: a cap nobody can reach is indistinguishable from a bug,
    and there must be enough PWM channels left to actually wire this many."""
    doc = tank_doc()
    doc["mechanisms"] = _mechs(layout.MAX_MECHANISMS)
    result = layout.validate(doc)
    assert result.errors == []
    assert len(result.mechanisms) == layout.MAX_MECHANISMS


def test_an_unsupported_version_is_refused():
    assert not layout.validate({"version": 99, "drive": {}}).ok


def test_junk_never_raises():
    """Anything can arrive off a radio; none of it may take the robot down."""
    for junk in (None, [], "layout", 7, {}, {"version": 1},
                 {"version": 1, "drive": "tank"},
                 {"version": 1, "drive": {"actuators": "left"}}):
        result = layout.validate(junk)
        assert not result.ok
        assert result.errors


def test_a_failed_document_leaves_the_config_untouched():
    """A bad layout must leave the robot on its previous wiring, not half of
    each."""
    cfg = RobotConfig()
    before = layout.to_doc(cfg)
    assert not layout.apply(cfg, {"version": 1, "drive": {"kind": "wat"}}).ok
    assert layout.to_doc(cfg) == before


# --- wheel encoders ----------------------------------------------------------

def encoder_doc(**pins):
    """The stock tank layout with an encoder on the left motor."""
    doc = tank_doc()
    doc["drive"]["actuators"][0].update(
        {"encoder_a": 17, "encoder_b": 27, "encoder_cpr": 1200, **pins})
    return doc


def test_a_layout_without_encoders_does_not_carry_encoder_fields():
    """Four extra fields on every actuator of a sixteen-motor build is a few
    hundred bytes of a radio spent saying "no encoder" over and over, and the
    document is capped."""
    for actuator in layout.default_doc()["drive"]["actuators"]:
        assert "encoder_a" not in actuator


def test_encoder_wiring_round_trips_through_the_document():
    """A build that HAS encoders must always carry them, or a round trip
    through the editor would quietly unplug one."""
    cfg = RobotConfig()
    result = layout.apply(cfg, encoder_doc())
    assert result.ok, result.errors
    assert cfg.drive.actuators["left"].encoder_a == 17
    assert cfg.drive.actuators["left"].encoder_cpr == 1200
    assert layout.to_doc(cfg)["drive"]["actuators"][0]["encoder_b"] == 27


def test_two_actuators_reading_one_pin_are_rejected():
    """Nastier than a PWM clash: both count the same edges, so a robot with one
    genuinely dragging track reports both wheels at exactly the same speed —
    the single symptom that would convince you the encoders were working."""
    doc = encoder_doc()
    doc["drive"]["actuators"][1].update(
        {"encoder_a": 17, "encoder_b": 22, "encoder_cpr": 1200})
    errors = " ".join(layout.validate(doc).errors)
    assert "GPIO 17" in errors


def test_both_channels_on_one_pin_are_rejected():
    """A quadrature decoder needs two phases to have a phase relationship."""
    errors = " ".join(layout.validate(encoder_doc(encoder_b=17)).errors)
    assert "two separate pins" in errors


def test_half_an_encoder_is_rejected():
    """One channel decodes nothing at all, so this is a typo, not a choice."""
    errors = " ".join(layout.validate(encoder_doc(encoder_b=-1)).errors)
    assert "both A and B" in errors


def test_an_out_of_range_pin_is_clamped_rather_than_refusing_the_layout():
    """Same rule as every other actuator number: a field pinned at its limit is
    honest, dropping the whole layout over one typo is not."""
    result = layout.validate(encoder_doc(encoder_a=99, encoder_b=22))
    assert result.ok, result.errors
    assert result.drive.actuators["left"].encoder_a == 27


def test_a_pin_clamped_onto_a_neighbours_pin_is_still_caught():
    """The clamp runs first and the conflict check runs on what it produced, so
    a wild number cannot land on a real pin and be waved through."""
    result = layout.validate(encoder_doc(encoder_a=99, encoder_b=27))
    assert not result.ok


# --- persistence -------------------------------------------------------------

def test_save_and_load_round_trip(tmp_path):
    path = str(tmp_path / "layout.json")
    doc = layout.default_doc()
    assert layout.save(doc, path) is None
    assert layout.load(path) == doc


def test_a_missing_file_is_not_an_error(tmp_path):
    assert layout.load(str(tmp_path / "nope.json")) is None


def test_a_corrupt_file_is_ignored_not_fatal(tmp_path):
    """A half-written layout must leave the robot bootable on its defaults."""
    path = tmp_path / "layout.json"
    path.write_text("{ this is not json")
    assert layout.load(str(path)) is None


def test_save_failure_is_reported_not_raised(tmp_path):
    blocked = tmp_path / "afile"
    blocked.write_text("")
    assert layout.save({}, str(blocked / "sub" / "layout.json"))


def test_layout_path_honours_the_env_var(monkeypatch):
    monkeypatch.setenv("RS_LAYOUT_FILE", "/tmp/custom-layout.json")
    assert layout.layout_path() == "/tmp/custom-layout.json"
