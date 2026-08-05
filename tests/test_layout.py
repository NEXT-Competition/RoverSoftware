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


# --- the ESC throttle band ---------------------------------------------------

def test_an_esc_endpoint_outside_the_throttle_band_is_warned_about():
    """The HAT spans 500..2500us; an ESC only listens to 1000..2000us.

    An angle past +-45 therefore reads to the ESC as a lost signal, not as more
    throttle: it cuts the motor and re-arms partway through the throw. It looks
    like more authority in the document, which is exactly why it needs saying.
    """
    doc = tank_doc()
    doc["drive"]["actuators"][0]["min_angle"] = -60
    result = layout.validate(doc)
    assert result.ok, result.errors      # a warning, not a refusal
    assert any("min_angle -60" in w and "833us" in w for w in result.warnings), \
        result.warnings


def test_an_endpoint_inside_the_band_is_not_warned_about():
    assert layout.validate(tank_doc()).warnings == []


def test_a_positional_servo_may_use_the_hats_whole_range():
    """Only ESCs are bound by the throttle band — a servo really does travel
    the full 500..2500us, so warning about one would be noise."""
    doc = tank_doc()
    doc["mechanisms"] = [{"name": "hood", "kind": "power",
                          "actuators": [{"name": "arm", "kind": "servo",
                                         "channel": 7, "min_angle": -90,
                                         "max_angle": 90}]}]
    result = layout.validate(doc)
    assert result.ok, result.errors
    assert result.warnings == []


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


def test_too_many_mechanisms_are_refused():
    doc = tank_doc()
    doc["mechanisms"] = [
        {"name": f"m{i}", "kind": "power",
         "actuators": [{"name": "a", "channel": 2}]}
        for i in range(layout.MAX_MECHANISMS + 1)]
    assert not layout.validate(doc).ok


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
