"""The `target_distance` transition: fire at a distance you name.

The robot already had three ways of asking about how far away something is, and
none of them was this one:

    arrived    the aligning controller stopped at ITS OWN standoff. Only means
               anything while that controller is driving, and the distance is
               the one that controller was told.
    in_range   the ballistics model says a shot has a solution. Can be false
               because the target is too NEAR as well as too far.
    distance_m how far the ROVER is from a saved place, by GPS.

This is the plain one: a number in metres, tested against the measured range to
the thing in front, whatever happens to be driving. That is what lets a state
hand over at 2 m on the way in rather than waiting for an approach to finish.

The rule that matters most is the one at the bottom: an unknown distance is
never true. A state waiting for a distance should wait, not proceed on a number
nobody has.
"""

from robot.routine.conditions import RoutineContext, compile_condition


class FakeAlign:
    """The slice this condition reads: how far away the detected target is."""

    def __init__(self, distance=None):
        self.distance = distance

    def distance_m(self):
        return self.distance

    def last_detection(self):
        return None


def predicate(spec):
    compiled, problems = compile_condition({"when": "target_distance", **spec})
    assert problems == [], problems
    return compiled


def seeing(distance, **kw):
    return RoutineContext(controllers={"object_align": FakeAlign(distance)}, **kw)


# --- the comparison ----------------------------------------------------------

def test_it_fires_once_the_target_is_within_the_distance():
    within = predicate({"at_most": 1.5})
    assert within(seeing(3.0)) is False
    assert within(seeing(1.5)) is True          # the boundary counts as within
    assert within(seeing(0.4)) is True


def test_a_lower_bound_fires_once_the_target_is_far_enough_away():
    """The other direction, which `not(within)` cannot express: that would also
    be true with nothing in view, and "I cannot see it" is not "it is far"."""
    beyond = predicate({"at_least": 2.0})
    assert beyond(seeing(1.0)) is False
    assert beyond(seeing(2.5)) is True


def test_both_bounds_make_a_band():
    """The shape a standoff wants: close enough to be worth shooting at, far
    enough that a fixed launch angle can still arc into it."""
    band = predicate({"at_least": 1.0, "at_most": 2.0})
    assert band(seeing(0.5)) is False
    assert band(seeing(1.5)) is True
    assert band(seeing(2.5)) is False


def test_a_zero_lower_bound_is_the_off_position():
    """What the editor writes when the field is left alone: every distance is at
    least zero, so the test is exactly 'within'."""
    within = predicate({"at_most": 1.5, "at_least": 0})
    assert within(seeing(0.2)) is True
    assert within(seeing(3.0)) is False


# --- which distance ----------------------------------------------------------

def test_by_default_it_measures_the_detected_target():
    assert predicate({"at_most": 1.0})(seeing(0.5)) is True


def test_ahead_measures_whatever_the_ultrasonic_can_see():
    """No model, no calibration, no idea what it is looking at — which is the
    point: 'creep until something is close' needs no vision at all."""
    ahead = predicate({"at_most": 0.4, "source": "ahead"})
    ctx = RoutineContext(sonar=lambda: 0.3)
    assert ahead(ctx) is True
    assert ahead(RoutineContext(sonar=lambda: 1.2)) is False


def test_the_two_sources_are_genuinely_different_questions():
    """A chair between the rover and the bucket: the sonar reports the chair,
    the vision stack reports the bucket. Asking the wrong one is how a routine
    stops at the wrong thing, so they must not be interchangeable."""
    ctx = RoutineContext(controllers={"object_align": FakeAlign(3.0)},
                         sonar=lambda: 0.5)
    assert predicate({"at_most": 1.0})(ctx) is False              # the bucket
    assert predicate({"at_most": 1.0, "source": "ahead"})(ctx) is True  # the chair


# --- unknown is never true ---------------------------------------------------

def test_nothing_in_view_is_not_a_distance():
    assert predicate({"at_most": 1.0})(seeing(None)) is False


def test_no_aligning_controller_at_all_is_not_a_distance():
    assert predicate({"at_most": 1.0})(RoutineContext()) is False


def test_a_build_with_no_ultrasonic_never_fires_the_ahead_test():
    """A routine written on a rover that has one still LOADS on a rover that
    does not — it simply never takes that transition."""
    assert predicate({"at_most": 0.4, "source": "ahead"})(RoutineContext()) is False


def test_an_echo_that_never_came_back_is_not_a_clear_road():
    """An ultrasonic hears nothing both when the path is clear and when it is
    broken. Neither is a measured distance, so neither fires this."""
    ctx = RoutineContext(sonar=lambda: None)
    assert predicate({"at_most": 0.4, "source": "ahead"})(ctx) is False


# --- what a bad spec does ----------------------------------------------------

def test_a_condition_with_no_bound_at_all_is_refused():
    """It would be a transition that reads as a distance test and fires on
    everything. Refused at load, with both field names in the message."""
    _, problems = compile_condition({"when": "target_distance"})
    assert problems and "at_most" in problems[0] and "at_least" in problems[0]


def test_an_unknown_source_is_refused_rather_than_guessed():
    _, problems = compile_condition(
        {"when": "target_distance", "at_most": 1.0, "source": "sonar"})
    assert problems and "unknown 'source'" in problems[0]


def test_it_is_in_the_published_vocabulary():
    """The palette in the dashboard mirrors this list by hand; a condition the
    robot accepts but never advertises is one nobody discovers."""
    from robot.routine.conditions import CONDITIONS
    assert "target_distance" in CONDITIONS


# --- through a document ------------------------------------------------------

def test_a_routine_can_transition_on_it():
    """End to end through the schema, because a condition that compiles in
    isolation and is rejected in a document helps nobody."""
    from robot.config import RoutineConfig
    from robot.routine import schema

    result = schema.parse({
        "routines": [{
            "id": "approach",
            "name": "Approach",
            "start": "drive",
            "states": [
                {"id": "drive", "drive": {"mode": "object_align"},
                 "transitions": [
                     {"when": "target_distance", "at_most": 1.5, "to": "done"}]},
                {"id": "done", "terminal": True},
            ],
        }],
    }, RoutineConfig(), ("teleop", "object_align", "routine"))
    assert result.ok, result.errors


def test_a_document_asking_for_an_impossible_distance_is_refused():
    from robot.config import RoutineConfig
    from robot.routine import schema

    result = schema.parse({
        "routines": [{
            "id": "approach", "name": "Approach", "start": "drive",
            "states": [
                {"id": "drive", "transitions": [
                    {"when": "target_distance", "source": "elsewhere",
                     "at_most": 1.0, "to": "done"}]},
                {"id": "done", "terminal": True},
            ],
        }],
    }, RoutineConfig(), ("teleop", "routine"))
    assert not result.ok
