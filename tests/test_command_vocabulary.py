"""Resolving what somebody said into something that exists.

This is where a voice interface is actually won or lost. The model can be
perfect and the whole thing is still useless if "rover two" doesn't reach
rover2 — and worse than useless if "bucket" silently picks between bucket A and
bucket B. Both halves are tested here: what must resolve, and what must refuse.
"""

import pytest

from basestation.command.vocabulary import (
    MODES,
    Vocabulary,
    normalise,
)


@pytest.fixture
def vocab():
    return Vocabulary(
        robots=["rover1", "rover2", "rover3"],
        online=["rover1", "rover2"],
        selected="rover1",
        places=[
            {"id": "bucket_a", "name": "bucket A", "lat": 1.0, "lon": 2.0, "kind": "bucket"},
            {"id": "bucket_b", "name": "bucket B", "lat": 3.0, "lon": 4.0, "kind": "bucket"},
            {"id": "start", "name": "start", "lat": 5.0, "lon": 6.0, "kind": "start"},
        ],
        routines={"rover2": ["collect", "return_home"]},
        labels=["bucket", "cone"],
    )


# --- normalise -----------------------------------------------------------

def test_normalise_folds_case_punctuation_and_spoken_numbers():
    assert normalise("Rover-2!") == "rover 2"
    assert normalise("rover two") == "rover 2"
    assert normalise("  BUCKET   A  ") == "bucket a"


def test_normalise_does_not_expand_homophones():
    """The bug this guards: expanding "to" -> "2" globally rewrote "switch to
    rover 3" as "switch 2 rover 3", which then no longer began with the lead-in
    "switch to" and stopped parsing entirely. Homophones belong to rover-name
    resolution alone (see resolve_robot)."""
    assert normalise("switch to rover 3") == "switch to rover 3"
    assert normalise("go to the bucket") == "go to the bucket"


# --- rovers ---------------------------------------------------------------

@pytest.mark.parametrize("said,expected", [
    ("rover2", "rover2"),
    ("rover 2", "rover2"),
    ("rover two", "rover2"),
    ("Rover-2", "rover2"),
    ("ROVER TWO", "rover2"),
    ("rover three", "rover3"),
])
def test_resolve_robot_accepts_how_people_actually_say_it(vocab, said, expected):
    assert vocab.resolve_robot(said) == expected


def test_resolve_robot_handles_whisper_homophones(vocab):
    """"rover to" and "rover too" are what Whisper produces for rover2 more
    often than "rover two" is."""
    assert vocab.resolve_robot("rover to") == "rover2"
    assert vocab.resolve_robot("rover too") == "rover2"


@pytest.mark.parametrize("said", ["rover9", "banana", "the blue one", ""])
def test_resolve_robot_refuses_rather_than_guessing(vocab, said):
    assert vocab.resolve_robot(said) is None


def test_resolve_robot_does_not_match_a_sentence_containing_a_name(vocab):
    """Regression: token overlap alone scored "select rover1" as a match for
    rover1, so the fast path reported the operator had named a rover called
    "select rover1". Naming a thing is not the same as talking about it."""
    assert vocab.resolve_robot("select rover1") is None
    assert vocab.resolve_robot("tell me about rover2 please") is None


def test_all_is_a_recognised_target(vocab):
    for phrase in ("all", "everyone", "all rovers", "the fleet", "every rover"):
        assert vocab.resolve_robot(phrase) == "*", phrase


# --- places ---------------------------------------------------------------

def test_resolve_place_matches_by_name(vocab):
    assert vocab.resolve_place("bucket A")["id"] == "bucket_a"
    assert vocab.resolve_place("bucket a")["id"] == "bucket_a"
    assert vocab.resolve_place("the bucket b")["id"] == "bucket_b"


def test_ambiguous_place_refuses(vocab):
    """"bucket" describes two saved places equally well. Picking one would be a
    rover driving to the wrong end of the field."""
    assert vocab.resolve_place("bucket") is None


def test_unambiguous_prefix_resolves():
    """With only one bucket, the same word is no longer ambiguous."""
    single = Vocabulary(places=[
        {"id": "bucket_a", "name": "bucket A", "lat": 1.0, "lon": 2.0, "kind": "bucket"}])
    assert single.resolve_place("bucket")["id"] == "bucket_a"


# --- modes ----------------------------------------------------------------

@pytest.mark.parametrize("said,expected", [
    ("teleop", "teleop"),
    ("manual", "teleop"),
    ("drive", "teleop"),
    ("object align", "object_align"),
    ("object_align", "object_align"),
    ("align", "object_align"),
    ("tracking", "object_align"),
    ("waypoint", "waypoint"),
    ("route", "waypoint"),
    ("shooter", "shooter_align"),
    ("aim", "shooter_align"),
    ("routine", "routine"),
])
def test_resolve_mode_accepts_aliases(vocab, said, expected):
    assert vocab.resolve_mode(said) == expected


def test_resolve_mode_refuses_a_non_mode(vocab):
    assert vocab.resolve_mode("hyperdrive") is None
    assert vocab.resolve_mode("") is None


def test_modes_match_the_dashboards_list():
    """basestation-ui/src/components/ModeControls.tsx has the same five. If one
    list grows without the other, an operator can say a mode they cannot click
    or click one they cannot say."""
    assert MODES == ("teleop", "object_align", "shooter_align", "waypoint", "routine")


# --- routines and labels --------------------------------------------------

def test_resolve_routine_is_scoped_to_one_robot(vocab):
    assert vocab.resolve_routine("rover2", "collect") == "collect"
    assert vocab.resolve_routine("rover2", "return home") == "return_home"
    # rover1 has no routines loaded, so the same word resolves to nothing.
    assert vocab.resolve_routine("rover1", "collect") is None


def test_resolve_label_prefers_what_the_detector_reports(vocab):
    assert vocab.resolve_label("bucket") == "bucket"
    assert vocab.resolve_label("cones") == "cone"


def test_resolve_label_falls_back_when_the_fleet_reports_nothing():
    """A bench with no camera attached must still accept "track the bucket" —
    the robot logs that nothing will ever match, which is the right place for
    that complaint."""
    blind = Vocabulary(robots=["rover1"])
    assert blind.resolve_label("red ball") == "red ball"


# --- the prompt context ---------------------------------------------------

def test_prompt_context_names_everything_sayable(vocab):
    text = vocab.as_prompt_context()
    for expected in ("rover1", "rover2", "bucket A", "bucket", "cone",
                     "object_align", "collect"):
        assert expected in text, expected


def test_prompt_context_survives_an_empty_fleet():
    text = Vocabulary().as_prompt_context()
    assert "none on the air" in text
    assert "none saved" in text
