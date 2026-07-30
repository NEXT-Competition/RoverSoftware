import { selected, selectedRobot, send } from "../net/ws.ts";
import type { Mode } from "../net/types.ts";

// The four modes that are a way of DRIVING. `routine` is the fifth
// (net/types.ts, and MODES in basestation/command/vocabulary.py) but it is not
// a button here, because "routine" on its own is not a thing an operator wants
// — a particular routine is. RoutineControls below lists them by name, and
// entering the mode is what tapping one does.
const MODES: { key: Mode; label: string }[] = [
  { key: "teleop", label: "Teleop" },
  { key: "object_align", label: "Object align" },
  { key: "shooter_align", label: "Shooter" },
  { key: "waypoint", label: "Waypoint" },
];

export function ModeControls() {
  const rid = selected.value;
  const current = selectedRobot.value?.mode;
  // A grid rather than a wrapping row: with mode names of unequal length,
  // flex-wrap left "Object align" on two lines and "Waypoint" alone on a third.
  // Equal cells also make the active mode easier to spot at a glance.
  return (
    <div class="mode-grid">
      {MODES.map((m) => (
        <button
          key={m.key}
          class={`btn${current === m.key ? " active" : ""}`}
          disabled={!rid}
          onClick={() => rid && send({ action: "mode", robot_id: rid, mode: m.key })}
        >
          {m.label}
        </button>
      ))}
    </div>
  );
}
