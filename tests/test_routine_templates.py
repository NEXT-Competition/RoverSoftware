"""The dashboard's starter routines must actually run.

A template is the first routine most competitors will ever open, and it is
offered from the UI as "here is a working example". If one of them is refused by
the robot's validator, the operator meets a wall of parse errors on a document
they didn't write — the worst possible first experience of the feature.

So the specs live in basestation-ui/src/routines/templates.json and this parses
every one of them with the REAL schema, under the DEFAULT config: no arming
permission, no layout mechanisms, stock controllers. A template that needs a
special robot to validate is not a starter template.
"""

import json
from pathlib import Path

import pytest

from robot.config import RoutineConfig
from robot.routine import schema

TEMPLATES_JSON = (Path(__file__).resolve().parents[1]
                  / "basestation-ui/src/routines/templates.json")

# What a stock build offers a routine to delegate driving to.
CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint")


def templates():
    return json.loads(TEMPLATES_JSON.read_text())


def test_the_templates_file_exists_and_is_not_empty():
    """templates.ts imports this; a missing file breaks the UI build silently
    in a way no Python test would otherwise notice."""
    assert TEMPLATES_JSON.exists(), f"{TEMPLATES_JSON} is missing"
    assert len(templates()) >= 3


@pytest.mark.parametrize("tpl", templates(), ids=lambda t: t["key"])
def test_every_template_parses_under_a_stock_config(tpl):
    doc = {"version": 1, "routines": [tpl["spec"]]}
    result = schema.parse(doc, RoutineConfig(), CONTROLLERS)
    assert result.ok, f"{tpl['key']}: {result.errors}"


@pytest.mark.parametrize("tpl", templates(), ids=lambda t: t["key"])
def test_every_template_reaches_every_state(tpl):
    """An unreachable box in a worked example teaches the wrong thing."""
    doc = {"version": 1, "routines": [tpl["spec"]]}
    result = schema.parse(doc, RoutineConfig(), CONTROLLERS)
    unreachable = [w for w in result.warnings if "never" in w and "reached" in w]
    assert not unreachable, f"{tpl['key']}: {unreachable}"


@pytest.mark.parametrize("tpl", templates(), ids=lambda t: t["key"])
def test_no_template_arms_a_launcher(tpl):
    """`arm` is refused unless the robot opts in, so a template using it would
    fail to save on a stock rover — and a starter that cannot be saved is worse
    than no starter."""
    text = json.dumps(tpl["spec"])
    assert '"do": "arm"' not in text, f"{tpl['key']} arms a launcher"


@pytest.mark.parametrize("tpl", templates(), ids=lambda t: t["key"])
def test_counting_never_lands_in_the_tick_slot(tpl):
    """The mistake that ends a 3-lap loop fifty times early. The schema warns
    about it; a shipped example must not demonstrate it."""
    for state in tpl["spec"].get("states", []):
        for action in state.get("on_tick", []) or []:
            assert action.get("do") not in ("count", "count_set"), \
                f"{tpl['key']}: state {state.get('id')!r} counts on_tick"


def test_templates_carry_the_words_an_operator_needs():
    for tpl in templates():
        assert tpl.get("title"), tpl["key"]
        assert tpl.get("blurb"), tpl["key"]
