"""Sending a whole document over a radio that drops frames.

The property that matters is all-or-nothing. A config snapshot merges safely
because it is independent scalars; a layout is a tree, and applying half of one
is how two motors end up on the same PWM channel. So every test here is really
asking the same question: can a partial transfer ever produce a document?
"""

import json

from robot.comms.doc_transfer import PART_CHARS, Reassembler, split

DOC = {"version": 1, "drive": {"kind": "tank", "actuators": [
    {"name": f"m{i}", "channel": i, "label": "x" * 40} for i in range(8)]}}


def frames(doc=DOC, **kw):
    return list(split(doc, "layout", "rover1", txid="L1", **kw))


def test_a_document_round_trips():
    rx = Reassembler()
    out = None
    for frame in frames():
        out = rx.feed(frame) or out
    assert out == DOC


def test_a_small_document_still_produces_one_frame():
    assert len(frames({"a": 1})) == 1


def test_fragments_are_small_enough_for_the_radio_to_write():
    """XBeeLink drops the frame if the radio isn't draining inside its 0.2 s
    write timeout, so a fragment that is too big is one that vanishes."""
    for frame in frames():
        assert len(json.dumps(frame, separators=(",", ":"))) < 600


def test_every_fragment_names_its_transfer_and_position():
    fs = frames()
    assert all(f["txid"] == "L1" for f in fs)
    assert [f["seq"] for f in fs] == list(range(len(fs)))
    assert all(f["n"] == len(fs) for f in fs)


def test_fragments_may_arrive_in_any_order():
    rx = Reassembler()
    out = None
    for frame in reversed(frames()):
        out = rx.feed(frame) or out
    assert out == DOC


def test_a_missing_fragment_produces_nothing():
    """The whole point. Half a tree is not a smaller tree."""
    rx = Reassembler()
    fs = frames()
    assert len(fs) > 1
    for frame in fs[1:]:
        assert rx.feed(frame) is None
    assert rx.pending


def test_a_new_transfer_abandons_an_unfinished_one():
    """A retry means the sender gave up on the last attempt too — merging the
    two would produce a document neither side ever wrote."""
    rx = Reassembler()
    rx.feed(frames()[0])
    out = None
    for frame in split(DOC, "layout", "rover1", txid="L2"):
        out = rx.feed(frame) or out
    assert out == DOC


def test_a_stalled_transfer_is_abandoned_after_the_timeout():
    rx = Reassembler(timeout=1.0)
    fs = frames()
    rx.feed(fs[0], now=0.0)
    assert rx.feed(fs[1], now=100.0) is None
    assert "timed out" in (rx.error or "")


def test_an_oversized_document_is_refused_rather_than_buffered():
    """This buffer is fed by anything that can put bytes on a shared radio."""
    rx = Reassembler(max_chars=100)
    big = {"pad": "x" * 5000}
    out = None
    for frame in split(big, "layout", "rover1", txid="B"):
        out = rx.feed(frame) or out
    assert out is None
    assert "exceeds" in (rx.error or "")
    assert not rx.pending


def test_fragments_that_do_not_reassemble_into_json_are_reported():
    rx = Reassembler()
    assert rx.feed({"txid": "x", "seq": 0, "n": 1, "part": "{not json"}) is None
    assert "JSON" in (rx.error or "")


def test_a_non_object_document_is_refused():
    rx = Reassembler()
    assert rx.feed({"txid": "x", "seq": 0, "n": 1, "part": "[1,2,3]"}) is None
    assert rx.error


def test_junk_headers_never_raise():
    """Anything can arrive off a radio; none of it may take the robot down."""
    rx = Reassembler()
    for junk in (None, [], "frame", 7,
                 {"txid": "x", "seq": "one", "n": 1, "part": ""},
                 {"txid": "x", "seq": 5, "n": 1, "part": ""},
                 {"txid": "x", "seq": -1, "n": 1, "part": ""},
                 {"txid": "x", "seq": 0, "n": 0, "part": ""},
                 {"txid": "x", "seq": 0, "n": 1, "part": 42}):
        assert rx.feed(junk) is None


def test_the_sender_is_stamped_so_a_fleet_can_share_one_channel():
    assert all(f["from"] == "rover1" for f in frames())


def test_extra_fields_ride_along_on_every_fragment():
    fs = list(split(DOC, "layout", "rover1", txid="L1", rev=7))
    assert all(f["rev"] == 7 for f in fs)


def test_the_default_fragment_size_is_what_split_actually_uses():
    fs = frames()
    assert all(len(f["part"]) <= PART_CHARS for f in fs)
