// A read-only picture of the machine, so the shape is visible at a glance.
//
// Hand-rolled SVG rather than a graph library: the layout is a single column
// with curved back-edges, which is ~100 lines here and a megabyte of dependency
// otherwise. Clicking a node scrolls to its card, which is the only interaction
// it needs — editing happens in the cards, where the words are.

import type { RoutineSpec } from "../../net/types.ts";
import { describeCondition } from "../../routines/vocab.ts";

const NODE_W = 132;
const NODE_H = 34;
const GAP = 26;
const PAD = 12;
const LANE = 46; // room to the right for the back-edges

export function GraphPreview(
  { routine, live }: { routine: RoutineSpec; live: string | null },
) {
  const states = routine.states;
  if (states.length === 0) return null;

  const index = new Map(states.map((s, n) => [s.id, n]));
  const y = (n: number) => PAD + n * (NODE_H + GAP);
  const height = y(states.length - 1) + NODE_H + PAD;
  const width = NODE_W + LANE + PAD * 2;

  const edges: { from: number; to: number; label: string }[] = [];
  for (const state of states) {
    for (const transition of state.transitions ?? []) {
      const from = index.get(state.id);
      const to = index.get(transition.to);
      if (from == null || to == null) continue;
      edges.push({
        from,
        to,
        label: describeCondition(transition.when, transition as Record<string, unknown>),
      });
    }
  }

  return (
    <svg
      class="routine-graph"
      viewBox={`0 0 ${width} ${height}`}
      role="img"
      aria-label="State machine overview"
    >
      <defs>
        <marker
          id="rg-arrow"
          viewBox="0 0 8 8"
          refX="7"
          refY="4"
          markerWidth="6"
          markerHeight="6"
          orient="auto"
        >
          <path d="M0,0 L8,4 L0,8 z" fill="currentColor" />
        </marker>
      </defs>

      {edges.map((edge, n) => {
        const forward = edge.to > edge.from;
        const y1 = y(edge.from) + NODE_H;
        const y2 = y(edge.to);
        if (forward && edge.to === edge.from + 1) {
          return (
            <g key={n} class="rg-edge">
              <line
                x1={PAD + NODE_W / 2}
                y1={y1}
                x2={PAD + NODE_W / 2}
                y2={y2}
                marker-end="url(#rg-arrow)"
              />
              <text x={PAD + NODE_W / 2 + 6} y={(y1 + y2) / 2 + 3} class="rg-label">
                {edge.label}
              </text>
            </g>
          );
        }
        // Anything that isn't a step to the next box arcs out to the right, so
        // loops and skips read as loops and skips.
        const startY = forward ? y1 : y(edge.from);
        const endY = forward ? y2 : y(edge.to) + NODE_H;
        const bow = PAD + NODE_W + LANE - 10;
        return (
          <g key={n} class="rg-edge back">
            <path
              d={`M ${PAD + NODE_W} ${startY + NODE_H / 4}
                  C ${bow} ${startY}, ${bow} ${endY}, ${PAD + NODE_W} ${endY - NODE_H / 4}`}
              fill="none"
              marker-end="url(#rg-arrow)"
            />
          </g>
        );
      })}

      {states.map((state, n) => (
        <g
          key={state.id}
          class={`rg-node${state.id === live ? " live" : ""}${
            state.terminal ? " terminal" : ""
          }${state.id === routine.start ? " start" : ""}`}
          onClick={() =>
            document.getElementById(`state-${state.id}`)?.scrollIntoView({
              behavior: "smooth",
              block: "center",
            })}
        >
          <rect x={PAD} y={y(n)} width={NODE_W} height={NODE_H} rx={8} />
          <text x={PAD + NODE_W / 2} y={y(n) + NODE_H / 2 + 4}>{state.id}</text>
        </g>
      ))}
    </svg>
  );
}
