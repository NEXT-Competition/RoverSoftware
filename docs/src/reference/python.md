# Extending it in Python

You do not need Python to program behaviour — that is what
[step 6](../bringup/routines.md) is for. You need it to teach the robot
something genuinely new: a different sensor, a different way of deciding where
to go.

| Where | What lives there |
|---|---|
| `robot/control/` | Controllers. Subclass `Controller`, return a `DriveCommand`, register a mode — nothing downstream changes. |
| `robot/drive/` | `ESCMotor`, the drivetrain kinds, and mechanisms. Mocks the HAT when it is absent. |
| `robot/routine/` | The FSM: schema, conditions, actions, engine, store. Adding a condition here is what makes it appear in the editor's menu. |
| `robot/tuning.py` | The whitelist of what the dashboard may change, and each field's limits. |
| `robot/comms/` | The protocol, the threaded XBee reader, and document fragmenting. |
| `robot/layout.py` | The hardware layout document — what this build HAS. |
| `basestation/command/` | Vocabulary, keyword fast path, the LLM classifier, the intent whitelist and the executor. |
| `basestation/fleet.py` | `FleetManager` — tracks every robot from its telemetry. |

## Two conventions worth knowing first

**Controllers get their inputs injected.** `ObjectAlignController` takes a
detection provider and `WaypointController` takes a pose provider, which is why
both are testable with no hardware and why the simulator can run the real
engine.

**Anything a browser can change must be declared twice.** Once in
`robot/tuning.py`, which decides what is legal and clamps it, and once in
`basestation-ui/src/settings/schema.ts`, which adds the words a person reads.
Python is the authority; the schema is presentation. The same pairing holds for
`robot/routine/conditions.py` + `actions.py` against
`basestation-ui/src/routines/vocab.ts`.

## Adding a control mode

1. Subclass `Controller` in `robot/control/`, returning a `DriveCommand` from
   `update()`.
2. Register it with `ControlManager` so a `mode` frame can select it.
3. Add its tunables to `robot/tuning.py` and mirror them in `schema.ts`.
4. If a routine should be able to delegate to it, add it to `DRIVE_MODES` in
   `vocab.ts` and to the engine's delegation table.

Nothing in `robot/drive/` changes. That is the whole point of the one-command
design.

## Tests

```bash
pytest -q
```

The suite runs with no hardware and no radio: the motor layer mocks the HAT, the
simulator stands in for the fleet, and the document-transfer tests exercise the
real fragmenting path. If you add a condition, add a test next to
`tests/test_routine_engine.py` — the editor will happily offer a verb the robot
refuses.
