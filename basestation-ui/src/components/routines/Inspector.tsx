// What the canvas cannot show: the selected node's actions and its wires' details.
//
// A node graph is good at structure and useless at arguments — you cannot pick
// "hold for 0.4 s" or "preset: in" off a box. So the graph owns *shape* and this
// owns *detail*, joined by the selection. Selecting a wire on the canvas
// highlights its condition here, which is the whole reason drawing one is useful.

import type { RoutineSpec } from "../../net/types.ts";
import {
  removeState,
  selectedEdge,
  selectedState,
  setRoutineField,
} from "../../state/routines.ts";
import { StateCard } from "./StateCard.tsx";

export function Inspector(
  { routine, live, problems }: {
    routine: RoutineSpec;
    live: string | null;
    problems: Map<string, string[]>;
  },
) {
  const state = routine.states.find((s) => s.id === selectedState.value);

  if (!state) {
    return (
      <aside class="inspector empty">
        <p class="hint">
          Tap a box to edit what it does. Drag from a box's right edge onto
          another to wire them together, then tap the wire to say when it fires.
        </p>
      </aside>
    );
  }

  const edge = selectedEdge.value;
  return (
    <aside class="inspector">
      <div class="inspector-head">
        <span class="eyebrow">State</span>
        <span class="inspector-title">{state.id}</span>
        <div class="inspector-actions">
          {state.id !== routine.start && (
            <button
              type="button"
              class="btn ghost small"
              onClick={() => setRoutineField("start", state.id)}
              title="Begin the routine here"
            >
              Make start
            </button>
          )}
          <button
            type="button"
            class="btn ghost small danger"
            onClick={() => removeState(state.id)}
          >
            Delete
          </button>
        </div>
      </div>

      <StateCard
        state={state}
        states={state ? routine.states.map((s) => s.id) : []}
        isStart={state.id === routine.start}
        isLive={state.id === live}
        problems={problems.get(state.id) ?? []}
        highlightTransition={edge?.state === state.id ? edge.index : null}
      />
    </aside>
  );
}
