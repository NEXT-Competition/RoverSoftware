"""UI-authored finite state machines.

An operator draws a state machine in the dashboard's Routines tab; it arrives
here as a JSON document, is validated and compiled once, and is then run by
`RoutineController` as another control mode. The point of the design is that a
state DELEGATES driving to the controllers that already exist — a state can say
"drive with object_align" and get the real alignment loop, providers and all —
so the FSM composes autonomy instead of re-expressing it.

    schema.py      parse + validate a document into compiled Routines
    conditions.py  the questions a transition may ask (pure, injected sensing)
    actions.py     the things a state may do (never the drivetrain)
    engine.py      which state is current, and when that changes
    store.py       routines.json
"""

from .conditions import CONDITIONS, RoutineContext
from .actions import ACTIONS
from .engine import RoutineEngine
from .schema import Routine, State, Transition, parse

__all__ = [
    "ACTIONS", "CONDITIONS", "Routine", "RoutineContext", "RoutineEngine",
    "State", "Transition", "parse",
]
