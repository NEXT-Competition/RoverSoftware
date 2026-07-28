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

/** Which node is selected on the canvas, and so which state the inspector edits.
 *  A node graph can show structure but not detail — you cannot pick a condition's
 *  arguments off a box — so selection is what connects the two halves. */
export const selectedState = signal<string | null>(null);

/** A selected edge, so it can be deleted without hunting for it in a list. */
export const selectedEdge = signal<{ state: string; index: number } | null>(null);

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

// --- the canvas: node positions ----------------------------------------------
//
// Positions live in the document (see net/types.ts), so they travel with the
// routine to the robot and back. A state without usable coordinates is laid out
// rather than dropped at the origin — an imported routine, or one written by hand,
// must not open as a pile of boxes on top of each other.

export const NODE_W = 158;
export const NODE_H = 58;
const COL = 300; // horizontal gap between layers — room for a wire's label
const ROW = 96; // vertical gap between siblings in a layer

function usable(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

export interface Placed {
  state: RoutineStateSpec;
  x: number;
  y: number;
}

/**
 * Every state with a definite position.
 *
 * Stored coordinates win. Anything missing gets a layered left-to-right layout:
 * BFS depth from the start state sets the column, order within the column sets
 * the row. That reads the way a sequence is meant to — first thing on the left —
 * and it is stable, so the same routine always lays out the same way.
 */
export const placed = computed<Placed[]>(() => {
  const routine = current.value;
  if (!routine) return [];

  const depth = new Map<string, number>([[routine.start, 0]]);
  const queue = [routine.start];
  while (queue.length) {
    const id = queue.shift()!;
    const state = routine.states.find((s) => s.id === id);
    for (const transition of state?.transitions ?? []) {
      if (!depth.has(transition.to)) {
        depth.set(transition.to, (depth.get(id) ?? 0) + 1);
        queue.push(transition.to);
      }
    }
  }
  // Unreachable states are still real states someone is mid-way through wiring
  // up. Park them in a column past the end rather than hiding them.
  const deepest = Math.max(0, ...depth.values());
  const perColumn = new Map<number, number>();

  return routine.states.map((state) => {
    const column = depth.get(state.id) ?? deepest + 1;
    const row = perColumn.get(column) ?? 0;
    perColumn.set(column, row + 1);
    return {
      state,
      x: usable(state.x) ? state.x : column * COL,
      y: usable(state.y) ? state.y : row * ROW,
    };
  });
});

export function setStatePosition(id: string, x: number, y: number): void {
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === id);
    if (state) {
      state.x = Math.round(x);
      state.y = Math.round(y);
    }
  });
}

/** Throw away stored positions and let the layered layout take over. */
export function relayout(): void {
  editCurrent((routine) => {
    for (const state of routine.states) {
      delete state.x;
      delete state.y;
    }
  });
}

// --- editing states ----------------------------------------------------------

/**
 * A spot near `x, y` that no node already occupies.
 *
 * Two "+ State" clicks both want the middle of the view, and dropping the second
 * exactly on the first makes it look as though the button did nothing — the new
 * node is there, perfectly hidden. Cascade instead, the way new windows do.
 */
function freeSpotNear(x: number, y: number): { x: number; y: number } {
  const taken = placed.value;
  const clear = (px: number, py: number) =>
    !taken.some((n) => Math.abs(n.x - px) < NODE_W * 0.6 && Math.abs(n.y - py) < NODE_H * 0.8);
  for (let step = 0; step < 40; step++) {
    const px = x + step * 26;
    const py = y + step * 26;
    if (clear(px, py)) return { x: px, y: py };
  }
  return { x, y };
}

export function addState(at?: { x: number; y: number }): void {
  const spot = at ? freeSpotNear(at.x, at.y) : null;
  editCurrent((routine) => {
    const id = uniqueId(new Set(routine.states.map((s) => s.id)), "state");
    const state: RoutineStateSpec = { id, drive: { mode: "stop" }, transitions: [] };
    if (spot) {
      state.x = Math.round(spot.x);
      state.y = Math.round(spot.y);
    }
    routine.states.push(state);
    selectedState.value = id;
  });
}

export function removeState(id: string): void {
  editCurrent((routine) => {
    routine.states = routine.states.filter((s) => s.id !== id);
    // Drop the wires that pointed at it too, rather than leaving edges the graph
    // would draw to nowhere and the robot would refuse on save.
    for (const state of routine.states) {
      state.transitions = (state.transitions ?? []).filter((t) => t.to !== id);
    }
  });
  if (selectedState.value === id) selectedState.value = null;
  selectedEdge.value = null;
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

/**
 * Wire two nodes together, from dragging on the canvas.
 *
 * Defaults to `after a delay` rather than `immediately`: a transition drawn and
 * then forgotten should make the robot pause, not tear through every state in
 * five ticks. Selects the new edge so its condition can be set straight away —
 * the wire is the gesture, the condition is the point.
 */
export function connect(from: string, to: string): void {
  let index = -1;
  editCurrent((routine) => {
    const state = routine.states.find((s) => s.id === from);
    if (!state || !routine.states.some((s) => s.id === to)) return;
    const transitions = state.transitions ?? [];
    // A duplicate wire between the same pair is almost always a mis-drag, and
    // two identical conditions means the second can never fire.
    if (transitions.some((t) => t.to === to && t.when === "elapsed")) return;
    state.transitions = [...transitions, { when: "elapsed", seconds: 1, to }];
    index = state.transitions.length - 1;
  });
  if (index >= 0) selectedEdge.value = { state: from, index };
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
    // Pop ONCE, into a variable. Popping inside the find predicate drains the
    // frontier as it scans, so most states never get visited and perfectly
    // reachable ones come back flagged.
    const id = frontier.pop()!;
    const state = routine.states.find((s) => s.id === id);
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
