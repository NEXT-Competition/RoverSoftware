"""Running a state machine: which state is current, and when that changes.

The engine is pure decision logic over an injected context, exactly like the
controllers, so all of this runs with no hardware and no clock of its own — the
tests drive time forward by hand.
"""

import pytest

from robot.config import RoutineConfig
from robot.routine import schema
from robot.routine.conditions import RoutineContext
from robot.routine.engine import RoutineEngine

CONTROLLERS = ("teleop", "object_align", "shooter_align", "waypoint")


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


class FakeAlign:
    def __init__(self):
        self._aligned = False
        self._arrived = False
        self._detection = None

    def aligned(self):
        return self._aligned

    def arrived(self):
        return self._arrived

    def last_detection(self):
        return self._detection


class FakeMech:
    def __init__(self):
        self.stopped = 0
        self.preset = None
        self.power = None
        self.activations = 0

    def set_power(self, power, actuator=None):
        self.power = power
        return True

    def apply_preset(self, name):
        self.preset = name
        return True

    def stop(self):
        self.stopped += 1

    def ready(self):
        return True

    def fire(self):
        self.activations += 1
        return True


def build(states, cfg=None, ctx=None, **routine_kw):
    doc = {"version": 1, "routines": [dict(
        id="r1", start=states[0]["id"], states=states, **routine_kw)]}
    cfg = cfg or RoutineConfig()
    result = schema.parse(doc, cfg, CONTROLLERS)
    assert result.ok, result.errors
    clock = Clock()
    ctx = ctx or RoutineContext()
    ctx.now = clock
    engine = RoutineEngine(result.routines["r1"], ctx, cfg, now=clock)
    engine.start()
    return engine, clock


# --- transitions -------------------------------------------------------------

def test_the_machine_starts_in_its_start_state():
    engine, _ = build([{"id": "a", "transitions": [{"when": "never", "to": "b"}]},
                       {"id": "b", "terminal": True}])
    assert engine.state.id == "a"


def test_elapsed_advances_the_machine():
    engine, clock = build([
        {"id": "a", "transitions": [{"when": "elapsed", "seconds": 1.0, "to": "b"}]},
        {"id": "b", "terminal": True}])
    engine.update(0.02)
    assert engine.state.id == "a"
    clock.advance(1.1)
    engine.update(0.02)
    assert engine.state.id == "b"


def test_transitions_are_evaluated_in_the_order_they_were_authored():
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "always", "to": "first"},
                                    {"when": "always", "to": "second"}]},
        {"id": "first", "terminal": True},
        {"id": "second", "terminal": True}])
    engine.update(0.02)
    assert engine.state.id == "first"


def test_at_most_one_transition_happens_per_tick():
    """The central no-blocking guarantee. A chain of `always` transitions
    advances one box per tick instead of spinning inside a single one — and it
    is what makes the machine's behaviour predictable to whoever drew it."""
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "transitions": [{"when": "always", "to": "c"}]},
        {"id": "c", "transitions": [{"when": "always", "to": "d"}]},
        {"id": "d", "terminal": True}])
    engine.update(0.02)
    assert engine.state.id == "b"
    engine.update(0.02)
    assert engine.state.id == "c"


# --- for_seconds -------------------------------------------------------------

def test_for_seconds_requires_the_condition_to_hold():
    """The launcher's dwell rule, generalized: one tick of 'aligned' is equally
    consistent with a false positive or a target crossing the centre."""
    align = FakeAlign()
    ctx = RoutineContext(controllers={"object_align": align})
    engine, clock = build([
        {"id": "a", "transitions": [
            {"when": "aligned", "for_seconds": 0.5, "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)

    align._aligned = True
    engine.update(0.02)
    assert engine.state.id == "a"  # holding, not yet held
    clock.advance(0.6)
    engine.update(0.02)
    assert engine.state.id == "b"


def test_a_flicker_resets_the_hold_timer():
    align = FakeAlign()
    ctx = RoutineContext(controllers={"object_align": align})
    engine, clock = build([
        {"id": "a", "transitions": [
            {"when": "aligned", "for_seconds": 0.5, "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)

    align._aligned = True
    engine.update(0.02)
    clock.advance(0.4)
    align._aligned = False       # lost it
    engine.update(0.02)
    align._aligned = True        # found it again
    engine.update(0.02)
    clock.advance(0.4)           # 0.8 s since first sighting, 0.4 since re-acquire
    engine.update(0.02)
    assert engine.state.id == "a"


# --- actions -----------------------------------------------------------------

def test_entry_actions_run_once_on_entry():
    mech = FakeMech()
    ctx = RoutineContext(mechanisms={"intake": mech})
    engine, _ = build([
        {"id": "a", "on_enter": [{"do": "mech_preset", "mech": "intake",
                                  "preset": "in"}],
         "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    assert mech.preset == "in"
    mech.preset = None
    engine.update(0.02)
    engine.update(0.02)
    assert mech.preset is None  # not re-run


def test_tick_actions_run_every_tick():
    mech = FakeMech()
    ctx = RoutineContext(mechanisms={"intake": mech})
    engine, _ = build([
        {"id": "a", "on_tick": [{"do": "mech_power", "mech": "intake",
                                 "power": 0.4}],
         "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    engine.update(0.02)
    assert mech.power == 0.4
    mech.power = None
    engine.update(0.02)
    assert mech.power == 0.4


def test_exit_actions_run_when_the_state_is_left():
    mech = FakeMech()
    ctx = RoutineContext(mechanisms={"intake": mech})
    engine, _ = build([
        {"id": "a", "on_exit": [{"do": "mech_stop", "mech": "intake"}],
         "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    engine.update(0.02)
    assert mech.stopped == 1


def test_exit_actions_run_on_an_abort_too():
    """The whole reason on_exit is a separate slot: disarming something has to
    happen on the way out however that happens, not only on a tidy transition."""
    mech = FakeMech()
    ctx = RoutineContext(mechanisms={"intake": mech})
    engine, _ = build([
        {"id": "a", "on_exit": [{"do": "mech_stop", "mech": "intake"}],
         "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    engine.stop("aborted")
    assert mech.stopped == 1


def test_an_action_that_throws_does_not_take_the_robot_down():
    class Exploding(FakeMech):
        def apply_preset(self, name):
            raise RuntimeError("boom")

    ctx = RoutineContext(mechanisms={"intake": Exploding()})
    engine, _ = build([
        {"id": "a", "on_enter": [{"do": "mech_preset", "mech": "intake",
                                  "preset": "in"}],
         "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    assert engine.state.id == "a"  # survived


# --- events ------------------------------------------------------------------

def test_an_event_satisfies_a_transition():
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "event", "name": "go", "to": "b"}]},
        {"id": "b", "terminal": True}])
    engine.update(0.02)
    assert engine.state.id == "a"
    engine.post_event("go")
    engine.update(0.02)
    assert engine.state.id == "b"


def test_events_are_cleared_on_state_entry():
    """An event fired while the machine was somewhere else must not satisfy a
    transition it was never meant for."""
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "transitions": [{"when": "event", "name": "go", "to": "c"}]},
        {"id": "c", "terminal": True}])
    engine.post_event("go")
    engine.update(0.02)          # a -> b, clearing the event
    engine.update(0.02)
    assert engine.state.id == "b"


# --- ending ------------------------------------------------------------------

def test_a_terminal_state_ends_the_routine():
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "always", "to": "done"}]},
        {"id": "done", "terminal": True}])
    engine.update(0.02)
    engine.update(0.02)
    assert engine.finished


def test_on_end_restart_loops_back_to_the_start():
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "always", "to": "done"}]},
        {"id": "done", "terminal": True}], on_end="restart")
    engine.update(0.02)
    engine.update(0.02)
    assert not engine.finished
    assert engine.state.id == "a"


def test_a_state_timeout_ends_the_routine():
    """Expiry ends the routine rather than advancing it: if a state overran, the
    next one's assumptions probably don't hold either."""
    engine, clock = build([
        {"id": "a", "timeout": 1.0,
         "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}])
    clock.advance(1.5)
    engine.update(0.02)
    assert engine.finished
    assert "timed out" in engine.reason


def test_an_unset_state_timeout_is_read_from_config_every_tick():
    """Which is what makes state_timeout_default a live parameter: raising it
    from the dashboard applies to the routine that is already running."""
    cfg = RoutineConfig(state_timeout_default=1.0)
    engine, clock = build([
        {"id": "a", "transitions": [{"when": "never", "to": "b"}]},
        {"id": "b", "terminal": True}], cfg=cfg)
    clock.advance(1.5)
    cfg.state_timeout_default = 100.0   # operator raises it mid-run
    engine.update(0.02)
    assert not engine.finished


def test_a_whole_routine_timeout_ends_it():
    engine, clock = build([
        {"id": "a", "timeout": 0,
         "transitions": [{"when": "always", "to": "b"}]},
        {"id": "b", "timeout": 0,
         "transitions": [{"when": "always", "to": "a"}]}], timeout=5.0)
    clock.advance(6.0)
    engine.update(0.02)
    assert engine.finished
    assert "routine timed out" in engine.reason


# --- conditions against the real controller surface -------------------------

def test_aligned_and_arrived_read_the_controllers_own_state():
    """Not recomputed here. An FSM with its own idea of 'lined up' would be a
    second, subtly different answer to a question the controller already
    answers, and the two would drift."""
    align = FakeAlign()
    ctx = RoutineContext(controllers={"object_align": align})
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "arrived", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    engine.update(0.02)
    assert engine.state.id == "a"
    align._arrived = True
    engine.update(0.02)
    assert engine.state.id == "b"


def test_shots_reads_the_mechanisms_counter():
    mech = FakeMech()
    ctx = RoutineContext(mechanisms={"shooter": mech})
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "shots", "at_least": 2, "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    mech.activations = 1
    engine.update(0.02)
    assert engine.state.id == "a"
    mech.activations = 2
    engine.update(0.02)
    assert engine.state.id == "b"


def test_a_condition_that_throws_reads_as_false():
    class Exploding(FakeAlign):
        def aligned(self):
            raise RuntimeError("boom")

    ctx = RoutineContext(controllers={"object_align": Exploding()})
    engine, _ = build([
        {"id": "a", "transitions": [{"when": "aligned", "to": "b"}]},
        {"id": "b", "terminal": True}], ctx=ctx)
    engine.update(0.02)
    assert engine.state.id == "a"


# --- telemetry ---------------------------------------------------------------

def test_status_names_the_live_state_for_the_dashboard():
    engine, _ = build([
        {"id": "seek", "drive": {"mode": "object_align"},
         "transitions": [{"when": "never", "to": "done"}]},
        {"id": "done", "terminal": True}])
    status = engine.status()
    assert status["id"] == "r1"
    assert status["state"] == "seek"
    assert status["drive"] == "object_align"
    assert status["done"] is False


# --- counters: the bounded loop ---------------------------------------------

def test_counter_and_count_make_a_bounded_loop():
    """Three laps of one state, then out. Without counters this needs the same
    state drawn three times, and a routine you redraw to re-tune is one nobody
    re-tunes."""
    ctx = RoutineContext()
    engine, _ = build([
        {"id": "work", "on_enter": [{"do": "count", "name": "lap"}],
         "transitions": [
             {"when": "counter", "name": "lap", "at_least": 3, "to": "done"},
             {"when": "always", "to": "work"}]},
        {"id": "done", "terminal": True},
    ], ctx=ctx)
    assert ctx.counters["lap"] == 1  # counted on entry, once per visit
    for _ in range(8):
        engine.update(0.02)
        if engine.state is None or engine.state.id == "done":
            break
    assert engine.state is not None and engine.state.id == "done"
    assert ctx.counters["lap"] == 3


def test_counters_reset_on_every_run():
    """A re-run must behave exactly as the first did — a loop inheriting the
    previous count would fall straight through."""
    ctx = RoutineContext()
    engine, _ = build([{"id": "a", "on_enter": [{"do": "count", "name": "n"}],
                        "transitions": []}], ctx=ctx)
    assert ctx.counters["n"] == 1
    engine.start()
    assert ctx.counters["n"] == 1


def test_count_set_rearms_a_loop():
    ctx = RoutineContext()
    build([{"id": "a",
            "on_enter": [{"do": "count", "name": "n", "by": 5},
                         {"do": "count_set", "name": "n", "to": 0}],
            "transitions": []}], ctx=ctx)
    assert ctx.counters["n"] == 0


def test_unknown_counter_reads_as_zero():
    """A half-drawn graph still runs rather than refusing to compile."""
    from robot.routine.conditions import compile_condition
    pred, problems = compile_condition(
        {"when": "counter", "name": "nope", "at_least": 1})
    assert problems == []
    assert pred(RoutineContext()) is False


def test_counting_on_tick_is_warned_about_not_refused():
    """It is legal — you might mean to count at the loop rate — but it is the
    mistake that makes a 3-lap loop end 50x early, so it must be said out loud."""
    doc = {"version": 1, "routines": [{
        "id": "r", "start": "a", "states": [
            {"id": "a", "on_tick": [{"do": "count", "name": "n"}],
             "transitions": []}]}]}
    result = schema.parse(doc, RoutineConfig(), CONTROLLERS)
    assert result.ok
    assert any("on_tick" in w and "count" in w for w in result.warnings), result.warnings
