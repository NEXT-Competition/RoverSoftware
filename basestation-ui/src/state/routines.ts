// Editing state for the Routines tab.
//
// Same draft-then-save shape as state/hardware.ts, for the same reason: a half
// applied state machine is meaningless. Routines differ from a layout in one
// way that matters to the operator — they take effect immediately when they
// land, because they are data the engine reads rather than hardware a
// constructor owns.
//
// Validation here is advisory. It catches the mistakes worth catching before a
// round trip (a transition pointing at a state you renamed, a routine with no
// way to end) so the editor can mark them inline. The robot re-checks
// everything and is the authority — robot/routine/schema.py.

import { computed, signal } from "@preact/signals";
import type {
  ActionSpec,
  MechanismSpec,
  RoutineDoc,
  RoutineSpec,
  RoutineStateSpec,
  TransitionSpec,
} from "../net/types.ts";
import { robotDocuments, selectedRobot, send } from "../net/ws.ts";
import { layout } from "./hardware.ts";
import { targetRobot } from "./settings.ts";

const draft = signal<RoutineDoc | null>(null);
const draftOwner = signal<string | null>(null);

/** Which routine the editor is showing. */
export const editing = signal<string | null>(null);

const EMPTY: RoutineDoc = { version: 1, routines: [] };

export const serverRoutines = computed<RoutineDoc | null>(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.routines ?? null : null;
});

export const routinesResult = computed(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.routines_result ?? null : null;
});

export const routines = computed<RoutineDoc>(() => {
  if (draft.value && draftOwner.value === targetRobot.value) return draft.value;
  return serverRoutines.value ?? EMPTY;
});

export const routinesDirty = computed(() =>
  draft.value != null && draftOwner.value === targetRobot.value
);

export const current = computed<RoutineSpec | null>(() => {
  const all = routines.value.routines;
  return all.find((r) => r.id === editing.value) ?? all[0] ?? null;
});

/** The state the robot is in right now, for highlighting the live card. Comes
 *  off the hot fleet frame, so it tracks at the UI rate. */
export const liveState = computed<string | null>(() => {
  const robot = selectedRobot.value;
  if (!robot || robot.mode !== "routine") return null;
  return robot.routine?.state ?? null;
});

function edit(mutate: (doc: RoutineDoc) => void): void {
  const next: RoutineDoc = JSON.parse(JSON.stringify(routines.value));
  mutate(next);
  draftOwner.value = targetRobot.value;
  draft.value = next;
}

function editCurrent(mutate: (routine: RoutineSpec) => void): void {
  const id = current.value?.id;
  if (!id) return;
  edit((doc) => {
    const routine = doc.routines.find((r) => r.id === id);
    if (routine) mutate(routine);
  });
}

export function discardRoutines(): void {
  draft.value = null;
  draftOwner.value = null;
}

export function refreshRoutines(): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "get_routines", robot_id: rid });
}

export function saveRoutines(): void {
  const rid = targetRobot.value;
  if (!rid) return;
  send({ action: "set_routines", robot_id: rid, doc: routines.value });
}

export function routinesAccepted(): void {
  discardRoutines();
}

// --- running -----------------------------------------------------------------

export function runRoutine(): void {
  const rid = targetRobot.value;
  const id = current.value?.id;
  if (!rid || !id) return;
  // Select first, then switch modes: arriving in `routine` with nothing chosen
  // would hold the robot still and look like the button did nothing.
  send({ action: "select_routine", robot_id: rid, id });
  send({ action: "mode", robot_id: rid, mode: "routine" });
}

export function stopRoutine(): void {
  const rid = targetRobot.value;
  if (!rid) return;
  send({ action: "routine_cmd", robot_id: rid, cmd: "stop" });
  send({ action: "mode", robot_id: rid, mode: "teleop" });
}

export function fireEvent(name: string): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "routine_event", robot_id: rid, name });
}

// --- editing routines --------------------------------------------------------

function uniqueId(taken: Set<string>, stem: string): string {
  if (!taken.has(stem)) return stem;
  for (let n = 2; n < 100; n++) if (!taken.has(`${stem}${n}`)) return `${stem}${n}`;
  return `${stem}_x`;
}

export function addRoutine(): void {
  edit((doc) => {
    const id = uniqueId(new Set(doc.routines.map((r) => r.id)), "routine");
    doc.routines.push({
      id,
      name: "New routine",
      start: "start",
      on_end: "stop",
      on_estop: "abort",
      states: [
        {
          id: "start",
          drive: { mode: "stop" },
          transitions: [{ when: "elapsed", seconds: 1, to: "done" }],
        },
        { id: "done", terminal: true },
      ],
    });
    editing.value = id;
  });
}

export function duplicateRoutine(): void {
  const source = current.value;
  if (!source) return;
  edit((doc) => {
    const copy: RoutineSpec = JSON.parse(JSON.stringify(source));
    copy.id = uniqueId(new Set(doc.routines.map((r) => r.id)), `${source.id}_copy`);
    copy.name = `${source.name ?? source.id} (copy)`;
    doc.routines.push(copy);
    editing.value = copy.id;
  });
}

export function removeRoutine(id: string): void {
  edit((doc) => {
    doc.routines = doc.routines.filter((r) => r.id !== id);
    if (editing.value === id) editing.value = doc.routines[0]?.id ?? null;
  });
}

export function setRoutineField<K extends keyof RoutineSpec>(
  key: K,
  value: RoutineSpec[K],
): void {
  editCurrent((routine) => {
    routine[key] = value;
  });
}

// --- editing states ----------------------------------------------------------

export function addState(): void {
  editCurrent((routine) => {
    const id = uniqueId(new Set(routine.states.map((s) => s.id)), "state");
    routine.states.push({ id, drive: { mode: "stop" }, transitions: [] });
  });
}

export function removeState(id: string): void {
  editCurrent((routine) => {
    routine.states = routine.states.filter((s) => s.id !== id);
  });
}

export function setStateField<K extends keyof RoutineStateSpec>(
  id: string,
  key: K,
  value: RoutineStateSpec[K],
): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (!state) return;
    const previous = state.id;
    state[key] = value;
    // Renaming a state follows through into everything pointing at it, so the
    // editor never produces a dangling `to` the operator didn't type.
    if (key === "id" && typeof value === "string" && value !== previous) {
      if (routine.start === previous) routine.start = value;
      for (const other of routine.states) {
        for (const transition of other.transitions ?? []) {
          if (transition.to === previous) transition.to = value;
        }
      }
    }
  });
}

export function setDrive(id: string, mode: string): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state) {
      state.drive = mode === "manual"
        ? { mode, throttle: 0, steer: 0 }
        : { mode };
    }
  });
}

export function setDriveValue(
  id: string,
  key: "throttle" | "steer",
  value: number,
): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state?.drive) state.drive[key] = value;
  });
}

// --- editing actions and transitions ----------------------------------------

export type Slot = "on_enter" | "on_tick" | "on_exit";

export function addAction(id: string, slot: Slot, action: ActionSpec): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (!state) return;
    state[slot] = [...(state[slot] ?? []), action];
  });
}

export function setAction(
  id: string,
  slot: Slot,
  index: number,
  action: ActionSpec,
): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state?.[slot]) state[slot]![index] = action;
  });
}

export function removeAction(id: string, slot: Slot, index: number): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state?.[slot]) state[slot] = state[slot]!.filter((_, n) => n !== index);
  });
}

export function addTransition(id: string): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (!state) return;
    const elsewhere = routine.states.find((s) => s.id !== id)?.id ?? id;
    state.transitions = [
      ...(state.transitions ?? []),
      { when: "elapsed", seconds: 1, to: elsewhere },
    ];
  });
}

export function setTransition(
  id: string,
  index: number,
  transition: TransitionSpec,
): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state?.transitions) state.transitions[index] = transition;
  });
}

export function removeTransition(id: string, index: number): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state?.transitions) {
      state.transitions = state.transitions.filter((_, n) => n !== index);
    }
  });
}

// --- advisory validation -----------------------------------------------------

/** Mechanisms this robot's layout declares, for the action pickers. The
 *  built-in launcher is added because it is registered under a reserved name
 *  rather than declared in the layout (see robot/layout.py). */
export const availableMechanisms = computed<MechanismSpec[]>(() => {
  const declared = layout.value?.mechanisms ?? [];
  const robot = selectedRobot.value;
  if (robot?.shooter && !declared.some((m) => m.name === "shooter")) {
    return [...declared, { name: "shooter", kind: "pulse", actuators: [] }];
  }
  return declared;
});

export interface Problem {
  state?: string;
  message: string;
}

/** Problems worth showing before a round trip. The robot re-checks all of this
 *  and refuses anything it doesn't like — this exists so the editor can point
 *  at the offending card instead of printing a list of strings. */
export const problems = computed<Problem[]>(() => {
  const routine = current.value;
  if (!routine) return [];
  const found: Problem[] = [];
  const ids = new Set(routine.states.map((s) => s.id));

  if (!ids.has(routine.start)) {
    found.push({ message: `Start state "${routine.start}" does not exist.` });
  }
  for (const state of routine.states) {
    for (const transition of state.transitions ?? []) {
      if (!ids.has(transition.to)) {
        found.push({
          state: state.id,
          message: `Goes to "${transition.to}", which is not a state.`,
        });
      }
    }
  }

  // The termination rule, mirrored from robot/routine/schema.py. A machine that
  // can never end is a robot that runs until somebody hits the e-stop.
  const canEnd = (routine.timeout ?? 0) > 0 ||
    routine.states.some((s) => s.terminal) ||
    routine.states.some((s) => s.timeout == null || s.timeout > 0);
  if (!canEnd) {
    found.push({
      message: "Nothing can stop this routine — give it a terminal state, a " +
        "routine timeout, or leave the per-state timeouts alone.",
    });
  }

  const reachable = new Set([routine.start]);
  const frontier = [routine.start];
  while (frontier.length) {
    const state = routine.states.find((s) => s.id === frontier.pop());
    for (const transition of state?.transitions ?? []) {
      if (!reachable.has(transition.to)) {
        reachable.add(transition.to);
        frontier.push(transition.to);
      }
    }
  }
  for (const state of routine.states) {
    if (!reachable.has(state.id)) {
      found.push({ state: state.id, message: "This state can never be reached." });
    }
  }
  return found;
});
