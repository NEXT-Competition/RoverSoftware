"""Recognising a command without a model.

Two jobs, and only the first one is about safety: stopping a rover must never
depend on LM Studio being open, and the handful of things an operator says
constantly should not cost a round trip to a model.

The second half of this file is the more important half — everything the fast
path must *refuse* to recognise. A rule that fires on a maybe is worse than no
rule, because it silently bypasses the model that would have got it right.
"""

import pytest

from basestation.command import fastpath
from basestation.command.vocabulary import Vocabulary


@pytest.fixture
def vocab():
    return Vocabulary(
        robots=["rover1", "rover2", "rover3"],
        online=["rover1", "rover2", "rover3"],
        selected="rover1",
        places=[
            {"id": "bucket_a", "name": "bucket A", "lat": 1.0, "lon": 2.0, "kind": "bucket"},
            {"id": "start", "name": "start", "lat": 3.0, "lon": 4.0, "kind": "start"},
        ],
        labels=["bucket", "cone"],
    )


# --- stopping -------------------------------------------------------------

@pytest.mark.parametrize("said", [
    "stop", "Stop!", "stop it", "stop now", "halt", "freeze", "abort",
    "emergency stop", "e stop", "estop", "shut it down",
])
def test_every_way_of_saying_stop_is_an_estop(vocab, said):
    assert fastpath.match(said, vocab) == ("estop", {}) or \
        fastpath.match(said, vocab)[0] == "estop", said


@pytest.mark.parametrize("said", [
    "all stop", "stop all", "stop everyone", "stop everything",
    "everybody stop", "all rovers stop",
])
def test_stopping_everything_is_distinct_from_stopping_one(vocab, said):
    assert fastpath.match(said, vocab) == ("estop", {"robot": "all"}), said


def test_a_stop_can_name_a_rover(vocab):
    assert fastpath.match("stop rover2", vocab) == ("estop", {"robot": "rover2"})
    assert fastpath.match("rover 3 stop", vocab) == ("estop", {"robot": "rover 3"})


@pytest.mark.parametrize("said", [
    "uh stop stop stop", "no no stop the rover", "wait stop", "hey stop that",
])
def test_a_stop_inside_a_sentence_is_caught_by_is_stop(vocab, said):
    """These are not clean matches, so `match` may decline them — but `is_stop`
    must not, because the executor uses it as the last check before handing a
    sentence to a model to think about."""
    assert fastpath.is_stop(said), said


def test_is_stop_does_not_fire_on_ordinary_orders(vocab):
    for said in ("send rover2 to bucket A", "align to the cone", "show the camera"):
        assert not fastpath.is_stop(said), said


# --- the everyday commands ------------------------------------------------

@pytest.mark.parametrize("said,expected", [
    ("rover 2", "rover 2"),
    ("rover two", "rover 2"),
    ("switch to rover three", "rover 3"),
    ("go to rover 2", "rover 2"),
    ("select rover1", "rover1"),
    ("use rover 3", "rover 3"),
    ("rover 3 please", "rover 3"),
])
def test_selecting_a_rover(vocab, said, expected):
    assert fastpath.match(said, vocab) == ("select_robot", {"robot": expected}), said


@pytest.mark.parametrize("said", [
    "teleop", "manual", "waypoint mode", "put it in waypoint mode",
    "shooter align", "go to teleop", "switch to routine mode",
])
def test_bare_mode_names(vocab, said):
    hit = fastpath.match(said, vocab)
    assert hit is not None and hit[0] == "set_mode", said
    assert vocab.resolve_mode(hit[1]["mode"]) is not None, said


@pytest.mark.parametrize("said,expected", [
    ("camera", {}),
    ("fpv", {}),
    ("the camera", {}),
    ("show me the camera", {}),
    ("pull up the camera", {}),
    ("bring up rover 2's camera", {"robot": "rover 2"}),
    ("let me see rover 3's feed", {"robot": "rover 3"}),
])
def test_pulling_up_the_camera(vocab, said, expected):
    assert fastpath.match(said, vocab) == ("show_camera", expected), said


def test_clearing_an_estop(vocab):
    assert fastpath.match("clear", vocab) == ("clear_estop", {})
    assert fastpath.match("clear the e stop", vocab) == ("clear_estop", {})


# --- what it must NOT recognise -------------------------------------------

@pytest.mark.parametrize("said", [
    # Has an object in it — the model resolves what to track.
    "align to the bucket",
    "track the cone instead",
    # Has a destination — needs the place vocabulary and ordering.
    "send rover2 to bucket A",
    "rover1 go to start then bucket A",
    # A question, not an order.
    "what is rover2 doing",
    "how much battery does rover1 have",
    # Multi-part.
    "put rover2 in waypoint and rover3 in teleop",
    # Not a command at all.
    "what's the weather tomorrow",
    "",
])
def test_anything_needing_understanding_falls_through_to_the_model(vocab, said):
    assert fastpath.match(said, vocab) is None, said


def test_a_camera_request_for_an_unknown_rover_falls_through(vocab):
    """"show me the front camera" names something that is not a rover. Guessing
    which rover was meant is exactly the kind of confident wrong answer the
    fast path exists to avoid."""
    assert fastpath.match("show me the front camera", vocab) is None


def test_an_unknown_bare_word_is_not_a_command(vocab):
    assert fastpath.match("banana", vocab) is None
    assert fastpath.match("rover9", vocab) is None


def test_a_rover_named_after_a_mode_keeps_its_own_name():
    """Rovers are checked before modes, so a fleet with a rover called "auto"
    can still be addressed by name."""
    odd = Vocabulary(robots=["auto", "rover2"], selected="rover2")
    assert fastpath.match("auto", odd) == ("select_robot", {"robot": "auto"})
