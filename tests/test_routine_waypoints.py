"""How a routine's `set_route` action reads its waypoints.

The bug this pins down: `waypoints` used to be accepted as anything, and
anything that wasn't a list silently became an EMPTY route. An empty route
reads as `route_done` on the first tick, so the rover falls straight through
its own drive state — which at the field looks like the routine being ignored,
and gives the operator nothing to search for.

The rule now:

    a value that isn't a route          -> error, the document is refused
    a leg that isn't a coordinate       -> error, the document is refused
    no waypoints at all                 -> allowed, with a warning

The middle one matters as much as the first. Skipping the one waypoint with a
typo in it leaves a route that loads, runs, and quietly drives a different
shape than the one on the operator's screen.
"""

import pytest

from robot.config import RoutineConfig
from robot.routine import schema
from robot.routine.actions import MAX_WAYPOINTS, parse_waypoints


def doc(waypoints, extra=None):
    """One routine whose first state loads a route."""
    action = {"do": "set_route", "waypoints": waypoints}
    action.update(extra or {})
    return {
        "version": 1,
        "routines": [{
            "id": "r", "name": "r", "start": "load",
            "states": [
                {"id": "load", "on_enter": [action],
                 "transitions": [{"when": "always", "to": "done"}]},
                {"id": "done", "terminal": True},
            ],
        }],
    }


# --- the shapes a route can be written in ------------------------------------

def test_a_list_of_pairs_is_the_normal_case():
    points, problem = parse_waypoints([[1.0, 2.0], [3.0, 4.0]])
    assert problem == ""
    assert points == [(1.0, 2.0), (3.0, 4.0)]


def test_text_pasted_out_of_a_spreadsheet_is_accepted():
    points, problem = parse_waypoints("1,2\n3,4\n")
    assert problem == ""
    assert points == [(1.0, 2.0), (3.0, 4.0)]


def test_hand_written_objects_are_accepted():
    points, problem = parse_waypoints([{"lat": 1.0, "lon": 2.0}])
    assert problem == ""
    assert points == [(1.0, 2.0)]


def test_a_missing_key_is_an_empty_route_not_an_error():
    assert parse_waypoints(None) == ([], "")


# --- what gets refused -------------------------------------------------------

@pytest.mark.parametrize("bad", [
    {"lat": 1.0},                     # an object that isn't a list of legs
    42,
    True,
])
def test_a_value_that_is_not_a_route_is_refused(bad):
    points, problem = parse_waypoints(bad)
    assert points == [] and problem


@pytest.mark.parametrize("bad", [
    [[1.0, 2.0], [3.0]],              # a leg missing its longitude
    [[1.0, 2.0], ["north", 4.0]],
    [[1.0, 2.0], [91.0, 4.0]],        # off the planet
    [[1.0, 2.0], [float("nan"), 4.0]],
])
def test_a_bad_leg_refuses_the_whole_route(bad):
    # NOT "skips the bad leg": a route that quietly lost a waypoint still runs,
    # and drives somewhere nobody drew.
    points, problem = parse_waypoints(bad)
    assert points == [] and problem


def test_a_route_longer_than_the_cap_is_refused():
    points, problem = parse_waypoints([[1.0, 2.0]] * (MAX_WAYPOINTS + 1))
    assert points == [] and str(MAX_WAYPOINTS) in problem


def test_nan_never_reaches_the_controller():
    # A NaN latitude is never <= arrive_radius_m, so the leg can never be
    # finished and the rover drives at it until the state times out.
    points, problem = parse_waypoints([[float("nan"), 2.0]])
    assert points == [] and problem


# --- through the real validator ----------------------------------------------

def test_a_string_route_is_refused_rather_than_silently_emptied():
    # The original bug, in the shape the editor used to write it.
    result = schema.parse(doc("not coordinates"), RoutineConfig())
    assert not result.ok
    # The message has to name the state and the offending leg, because the
    # operator reads it in the editor with a dozen nodes on screen.
    assert any("'load'" in e and "set_route" in e for e in result.errors)


def test_an_empty_route_is_allowed_but_warned_about():
    result = schema.parse(doc([]), RoutineConfig())
    assert result.ok
    assert any("no waypoints" in w for w in result.warnings)


def test_a_route_with_waypoints_passes_clean():
    result = schema.parse(doc([[37.1, -122.2], [37.2, -122.3]]), RoutineConfig())
    assert result.ok
    assert not result.warnings


def test_a_set_route_with_no_waypoints_key_at_all_still_validates():
    # What a freshly dropped node looks like before any place is picked.
    body = doc([])
    body["routines"][0]["states"][0]["on_enter"] = [{"do": "set_route"}]
    result = schema.parse(body, RoutineConfig())
    assert result.ok


def test_the_editors_place_ids_are_preserved_but_never_interpreted():
    # `places` is editor-only: the robot drives `waypoints` and echoes the raw
    # document back, which is what lets the editor still show a name.
    body = doc([[37.1, -122.2]], extra={"places": ["bucket_a"]})
    result = schema.parse(body, RoutineConfig())
    assert result.ok


def test_the_route_reaches_the_waypoint_controller():
    class FakeWaypoint:
        def __init__(self):
            self.message = None

        def on_message(self, message):
            self.message = message

    from robot.routine.actions import compile_action
    from robot.routine.conditions import RoutineContext

    effect, problems, _ = compile_action(
        {"do": "set_route", "waypoints": [[37.1, -122.2], [37.2, -122.3]]})
    assert problems == []

    controller = FakeWaypoint()
    effect(RoutineContext(controllers={"waypoint": controller}))
    assert controller.message == {
        "type": "route", "waypoints": [[37.1, -122.2], [37.2, -122.3]]}
