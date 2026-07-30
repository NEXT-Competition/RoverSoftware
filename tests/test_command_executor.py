"""The safety boundary: what reaches a robot, and what does not.

Every test here is about the same claim — a language model cannot cause anything
the dashboard could not have caused, and cannot cause the dangerous half at all
without a human. The recognition tests are in test_command_fastpath.py; these
are about authority.
"""

import time

import pytest

from basestation.command.executor import CommandExecutor, CONFIRM_TTL
from basestation.command.intents import INTENTS, Authority, plan
from basestation.command.llm import LLMError
from basestation.command.vocabulary import Vocabulary


class FakeFleet:
    """Just enough FleetManager for the executor: a snapshot and documents."""

    def __init__(self, robots=("rover1", "rover2"), selected="rover1", **overrides):
        self._robots = [
            {"robot_id": r, "mode": "teleop", "estop": False, "online": True,
             "age": 0.0, "lat": 1.0, "lon": 2.0, "heading": 0.0, "battery": 90.0,
             "vision": {"ok": True, "fps": 10, "label": "bucket", "conf": 0.9, "ex": 0.02},
             **overrides}
            for r in robots
        ]
        self._selected = selected

    def snapshot(self, now):
        return {"selected": self._selected, "robots": [dict(r) for r in self._robots]}

    def documents(self):
        return {"rover2": {"routines": {"routines": [
            {"id": "collect"},
            # The shape the editor actually produces: a generated id, and the
            # name the operator typed and will say out loud.
            {"id": "routine2", "name": "Collect cones"},
        ]}}}


class FakePlaces:
    def __init__(self, places=None):
        self._places = places if places is not None else [
            {"id": "bucket_a", "name": "bucket A", "lat": 10.0, "lon": 20.0, "kind": "bucket"},
        ]

    def snapshot(self):
        return [dict(p) for p in self._places]


class DeadLLM:
    """A model server that isn't running — the normal state on a bench."""
    base_url = "http://127.0.0.1:1234/v1"
    model = ""

    def health(self):
        return False, "no model server"

    def classify(self, text, vocab):
        raise LLMError("no model server")


@pytest.fixture
def rig():
    sent = []
    ex = CommandExecutor(FakeFleet(), FakePlaces(), dispatch=sent.append, llm=DeadLLM())
    return ex, sent


# --- the whitelist ---------------------------------------------------------

def test_an_unknown_intent_is_refused_not_forwarded(rig):
    """The expected shape of a hallucination. It must not be passed through
    hopefully on the theory that the bridge will ignore it."""
    ex, sent = rig
    out = ex.execute("delete_everything", {})
    assert out.status == "refused"
    assert sent == []


def test_a_command_for_a_nonexistent_rover_is_refused(rig):
    ex, sent = rig
    out = ex.execute("select_robot", {"robot": "rover99"})
    assert out.status == "refused"
    assert "rover1, rover2" in out.say  # tells the operator what IS there
    assert sent == []


def test_a_nonexistent_place_is_refused(rig):
    ex, sent = rig
    out = ex.execute("route_to", {"robot": "rover1", "places": ["Narnia"]})
    assert out.status == "refused"
    assert sent == []


def test_garbage_arguments_are_refused_not_crashed(rig):
    ex, _ = rig
    assert ex.execute("set_mode", "not a dict").status == "refused"
    assert ex.execute("drive", {"throttle": "sideways"}).status == "refused"
    assert ex.execute("", {}).status == "refused"


def test_out_of_range_numbers_are_clamped_not_refused(rig):
    """A model that says "full speed" as 1.5 meant 1.0. The robot's own limits
    are the ones that matter; refusing here would just be pedantry."""
    ex, sent = rig
    out = ex.execute("drive", {"throttle": 9.0, "steer": -50.0})
    assert out.status == "pending"  # drive is gated
    assert out.actions[0]["throttle"] == 1.0
    assert out.actions[0]["steer"] == -1.0


# --- the confirmation gate -------------------------------------------------

@pytest.mark.parametrize("name", [n for n, i in INTENTS.items()
                                  if i.authority is Authority.CONFIRM])
def test_every_gated_intent_sends_nothing_until_confirmed(rig, name):
    ex, sent = rig
    out = ex.execute(name, {"robot": "rover2", "mech": "intake", "routine": "collect"})
    assert out.status == "pending", name
    assert out.confirm_id
    assert sent == [], f"{name} reached the radio without approval"


def test_confirming_dispatches_exactly_what_was_shown(rig):
    ex, sent = rig
    out = ex.execute("fire", {"robot": "rover2"})
    assert sent == []
    done = ex.confirm(out.confirm_id)
    assert done.status == "done"
    assert sent == [{"action": "fire", "robot_id": "rover2"}]


def test_cancelling_dispatches_nothing(rig):
    ex, sent = rig
    out = ex.execute("arm_shooter", {"robot": "rover1"})
    ex.cancel(out.confirm_id)
    assert sent == []
    # And it cannot then be confirmed.
    assert ex.confirm(out.confirm_id).status == "refused"


def test_a_confirmation_cannot_be_used_twice(rig):
    ex, sent = rig
    out = ex.execute("fire", {"robot": "rover1"})
    assert ex.confirm(out.confirm_id).status == "done"
    assert ex.confirm(out.confirm_id).status == "refused"
    assert len(sent) == 1


def test_confirmations_expire(rig, monkeypatch):
    """A "fire?" card left over from an earlier match, tapped by somebody
    tidying up, is the exact accident the gate exists to prevent."""
    ex, sent = rig
    out = ex.execute("fire", {"robot": "rover1"})
    real = time.monotonic
    monkeypatch.setattr(time, "monotonic", lambda: real() + CONFIRM_TTL + 1)
    assert ex.confirm(out.confirm_id).status == "refused"
    assert sent == []


def test_an_unknown_confirm_id_is_refused(rig):
    ex, sent = rig
    assert ex.confirm("c999").status == "refused"
    assert sent == []


# --- e-stop is never gated -------------------------------------------------

def test_estop_is_authority_always():
    assert INTENTS["estop"].authority is Authority.ALWAYS


def test_estop_dispatches_immediately_with_no_model(rig):
    ex, sent = rig
    out = ex.recognise("stop")
    assert out.status == "done"
    assert out.route == "fastpath"       # never touched the model
    assert sent == [{"action": "estop", "robot_id": "rover1"}]


def test_estop_all_reaches_every_rover(rig):
    ex, sent = rig
    out = ex.recognise("all stop")
    assert out.status == "done"
    assert sent == [{"action": "estop", "robot_id": "rover1"},
                    {"action": "estop", "robot_id": "rover2"}]


def test_a_stop_buried_in_a_sentence_still_stops(rig):
    """"uh, stop, stop the rover" is not one of the literal phrases, but it is
    unmistakably a stop and must not be sent to a model to think about."""
    ex, sent = rig
    out = ex.recognise("uh stop stop the rover")
    assert out.status == "done"
    assert out.route == "fastpath"
    assert sent == [{"action": "estop", "robot_id": "rover1"}]


def test_a_routine_is_run_by_name_with_no_model_and_one_confirmation(rig):
    """The whole feature, end to end: an operator programs a routine, names it,
    and says that name. It reaches the radio as exactly what the Routines panel
    would have sent — and only after a human taps confirm, because a routine
    drives the robot on its own."""
    ex, sent = rig
    out = ex.recognise("rover 2 run collect cones")
    assert out.route == "fastpath", "starting a routine must not need a model"
    assert out.status == "pending"
    assert sent == []
    # Named back as it was said. "routine2" is not something an operator can
    # check a confirm card against.
    assert "Collect cones" in out.say
    ex.confirm(out.confirm_id)
    assert sent[0] == {"action": "select_routine", "robot_id": "rover2",
                       "id": "routine2"}
    assert {"action": "mode", "robot_id": "rover2", "mode": "routine"} in sent


def test_a_routine_named_for_another_rover_is_refused_not_redirected(rig):
    """rover1 carries no routines. Starting "collect cones" on it because
    rover2 happens to have one is how the wrong machine drives off."""
    ex, sent = rig
    out = ex.execute("start_routine", {"robot": "rover1", "routine": "collect cones"})
    assert out.status == "refused"
    assert sent == []


def test_making_something_safe_is_never_gated():
    """Disarming and stopping a routine are DIRECT even though their opposites
    need approval. A safety action that needs a confirmation is a safety action
    nobody reaches in time."""
    assert INTENTS["disarm_shooter"].authority is Authority.DIRECT
    assert INTENTS["stop_routine"].authority is Authority.DIRECT
    assert INTENTS["clear_estop"].authority is Authority.DIRECT


# --- a dead model must not disable the interface ---------------------------

def test_direct_commands_work_with_no_model(rig):
    ex, sent = rig
    assert ex.recognise("rover two").status == "done"
    assert ex.recognise("teleop").status == "done"
    assert ex.recognise("show me the camera").status == "done"


def test_a_sentence_needing_the_model_reports_why(rig):
    ex, sent = rig
    out = ex.recognise("send rover2 to bucket A and then come back")
    assert out.status == "error"
    assert "still work" in out.say  # says what DOES work
    assert sent == []


def test_disabling_the_model_leaves_the_fast_path_alone(rig):
    ex, sent = rig
    ex.enabled = False
    assert ex.recognise("stop").status == "done"
    out = ex.recognise("send rover2 somewhere clever")
    assert out.status == "unheard"
    assert "turned off" in out.say


# --- what the commands actually become -------------------------------------

def test_align_to_an_object_sets_the_target_before_the_mode():
    """One spoken sentence, two commands, in the order that matters: a mode
    switch that lands before the class filter starts by locking onto whatever
    happens to be nearest."""
    vocab = Vocabulary(robots=["rover1"], selected="rover1", labels=["bucket", "cone"])
    _intent, actions, sentence = plan("set_mode", {"mode": "align", "target": "cone"}, vocab)
    assert actions[0]["action"] == "set_config"
    assert actions[0]["config"] == {"vision.target_label": "cone"}
    assert actions[0]["save"] is False  # a target said out loud is not permanent
    assert actions[1] == {"action": "mode", "robot_id": "rover1", "mode": "object_align"}
    assert "cone" in sentence


def test_align_with_no_object_clears_a_stale_target():
    """Otherwise "align" silently inherits whatever the last order was tracking,
    which the operator has long since forgotten about."""
    vocab = Vocabulary(robots=["rover1"], selected="rover1", labels=["bucket"])
    _intent, actions, _ = plan("set_mode", {"mode": "object align"}, vocab)
    assert actions[0]["config"] == {"vision.target_label": ""}


def test_route_sends_waypoints_before_switching_mode():
    """A rover put into waypoint mode with no route yet is a rover in an
    autonomy mode with nowhere to go."""
    vocab = Vocabulary(robots=["rover1"], selected="rover1", places=[
        {"id": "a", "name": "bucket A", "lat": 1.0, "lon": 2.0, "kind": "bucket"},
        {"id": "b", "name": "start", "lat": 3.0, "lon": 4.0, "kind": "start"}])
    _intent, actions, sentence = plan(
        "route_to", {"places": ["bucket A", "start"]}, vocab)
    assert actions[0]["action"] == "route"
    assert actions[0]["waypoints"] == [[1.0, 2.0], [3.0, 4.0]]
    assert actions[1]["mode"] == "waypoint"
    assert sentence == "send rover1 to bucket A then start"


def test_selecting_a_rover_also_tells_the_dashboard(rig):
    """Regression, and the nastiest one found building this.

    The dashboard keeps its own selection so a tap feels instant, and adopts the
    server's only on the first frame (basestation-ui/src/net/ws.ts). Without a
    `ui` mirror, a spoken "rover three" moved the base station's selection —
    which is where the gamepad's sticks go — while the screen carried on
    highlighting rover1. An operator driving a rover the screen says they are
    not driving is the worst thing this feature could produce.
    """
    ex, sent = rig
    out = ex.execute("select_robot", {"robot": "rover2"})
    assert sent == [{"action": "select", "robot_id": "rover2"}]
    assert {"ui": "select", "robot_id": "rover2"} in out.actions


def test_show_camera_selects_the_rover_and_moves_the_screen(rig):
    """"pull up rover2's camera" is one sentence that has to do both — and the
    screen half must never reach the radio."""
    ex, sent = rig
    out = ex.execute("show_camera", {"robot": "rover2"})
    assert out.status == "done"
    assert sent == [{"action": "select", "robot_id": "rover2"}]
    assert {"ui": "camera", "robot_id": "rover2"} in out.actions


def test_ui_actions_never_reach_the_dispatcher(rig):
    ex, sent = rig
    ex.execute("show_view", {"view": "settings"})
    assert sent == []


def test_an_order_with_no_rover_named_uses_the_selected_one(rig):
    ex, sent = rig
    out = ex.execute("set_mode", {"mode": "waypoint"})
    assert out.status == "done"
    assert sent[-1] == {"action": "mode", "robot_id": "rover1", "mode": "waypoint"}


def test_with_nothing_selected_the_operator_is_asked_to_name_one():
    ex = CommandExecutor(FakeFleet(selected=None), FakePlaces(),
                         dispatch=lambda m: None, llm=DeadLLM())
    out = ex.execute("set_mode", {"mode": "teleop"})
    assert out.status == "refused"
    assert "say which one" in out.say


# --- the log ---------------------------------------------------------------

def test_every_outcome_is_logged_with_who_gave_it(rig):
    ex, _ = rig
    ex.recognise("stop", source="voice")
    ex.execute("select_robot", {"robot": "rover2"}, source="mcp")
    log = ex.status()["history"]
    assert [row["source"] for row in log[-2:]] == ["voice", "mcp"]
    assert log[-2]["transcript"] == "stop"


def test_status_answers_from_live_telemetry_rather_than_echoing(rig):
    ex, _ = rig
    out = ex.execute("status", {"robot": "rover1"})
    assert out.status == "done"
    assert out.say == "rover1 is in teleop."


def test_status_reports_an_estopped_rover_as_such():
    ex = CommandExecutor(FakeFleet(estop=True), FakePlaces(),
                         dispatch=lambda m: None, llm=DeadLLM())
    assert "e-stopped" in ex.execute("status", {"robot": "rover1"}).say


# --- the shape MCP and the UI both read ------------------------------------

def test_status_snapshot_has_everything_the_ui_renders(rig):
    ex, _ = rig
    snap = ex.status()
    assert set(snap) >= {"enabled", "model_ok", "model", "model_note", "base_url",
                         "pending", "history", "vocabulary"}
    assert snap["vocabulary"]["robots"] == ["rover1", "rover2"]
    assert snap["vocabulary"]["places"] == [{"id": "bucket_a", "name": "bucket A"}]


def test_describe_fleet_translates_the_wire_into_words(rig):
    ex, _ = rig
    described = ex.describe_fleet()
    robot = described["robots"][0]
    # `ex` is a normalised horizontal error; an MCP client shouldn't need to know.
    assert robot["vision"]["centred"] is True
    assert robot["vision"]["sees"] == "bucket"
    assert "estopped" in robot
