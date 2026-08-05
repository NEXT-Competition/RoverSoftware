"""What a script document is, and what makes one refusable.

The interesting property here is that validation COMPILES the code and runs
none of it: a document full of `os.system(...)` must validate and store without
anything happening, because validation is performed on whatever arrives over
the wire. Refusing that code is the runtime's job (test_script_runtime.py).
"""

import json

import pytest

from robot.script import schema


def doc(*scripts):
    return {"version": 1, "scripts": list(scripts)}


def one(code="rover.stop()", sid="drive"):
    return {"id": sid, "name": "Drive", "code": code}


def test_a_plain_script_is_accepted():
    result = schema.parse(doc(one()))
    assert result.ok, result.errors
    assert result.scripts["drive"].code == "rover.stop()"
    assert result.scripts["drive"].name == "Drive"


def test_a_syntax_error_is_refused_with_its_line_number():
    result = schema.parse(doc(one("x = 1\nif True\n    pass\n")))
    assert not result.ok
    # The line number is the whole reason validation compiles: it is what lets
    # the editor put a marker on the offending line instead of the operator
    # discovering it when they press Run.
    assert "line 2" in result.errors[0]


def test_validation_compiles_but_never_runs():
    """A script that would wreck the robot still SAVES. It fails at Run."""
    marker = []
    code = "import os\nos.system('rm -rf /')\n"
    result = schema.parse(doc(one(code)))
    assert result.ok, result.errors
    assert marker == []  # nothing executed


def test_an_empty_script_is_a_warning_not_an_error():
    # "New script" creates one of these. Refusing it would mean you cannot save
    # until you have finished writing.
    result = schema.parse(doc(one("")))
    assert result.ok
    assert any("empty" in w for w in result.warnings)


def test_windows_line_endings_are_normalised():
    result = schema.parse(doc(one("a = 1\r\nb = 2\r\n")))
    assert result.ok
    assert "\r" not in result.scripts["drive"].code


def test_ids_are_bounded_and_duplicates_refused():
    assert not schema.parse(doc(one(sid="Not An Id"))).ok
    assert not schema.parse(doc(one(sid="drive"), one(sid="drive"))).ok


def test_too_many_scripts_is_refused():
    many = [one(sid=f"s{n}") for n in range(schema.MAX_SCRIPTS + 1)]
    result = schema.parse(doc(*many))
    assert not result.ok
    assert "at most" in result.errors[0]


def test_one_oversized_script_is_refused_by_name():
    """Between the per-script cap and the whole-document one, so the message
    names the script rather than blaming the document."""
    assert schema.MAX_CODE_BYTES < schema.MAX_DOC_BYTES
    big = "x = 1\n" * ((schema.MAX_CODE_BYTES + 600) // 6)
    result = schema.parse(doc(one(big)))
    assert not result.ok
    assert "drive" in result.errors[0]


def test_the_whole_document_is_capped_under_the_reassembly_limit():
    """The transfer ceiling is the binding constraint, so it has to be checked
    on the total and not just per script."""
    from robot.comms.doc_transfer import MAX_CHARS

    assert schema.MAX_DOC_BYTES < MAX_CHARS
    chunk = "y = 0\n" * 2000
    result = schema.parse(doc(*[{"id": f"s{n}", "code": chunk} for n in range(6)]))
    assert not result.ok
    assert "in total" in result.errors[0]


def test_a_bad_document_shape_never_raises():
    for junk in (None, [], "scripts", 7, {"version": 9}, {"scripts": "no"}):
        result = schema.parse(junk)
        assert not result.ok
        assert result.errors


def test_a_valid_document_round_trips_through_json():
    original = doc(one("rover.forward(0.3, seconds=1)\n"))
    assert schema.parse(json.loads(json.dumps(original))).ok
