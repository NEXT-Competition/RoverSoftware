import { selected, selectedRobot, send } from "../net/ws.ts";
import type { Mode } from "../net/types.ts";

// The modes that are a way of DRIVING, and that an operator picks directly.
//
// Two of the five are deliberately absent. `routine` (net/types.ts, and MODES in
// basestation/command/vocabulary.py) is not a button because "routine" on its
// own is not a thing anyone wants — a particular routine is, and RoutineControls
// lists them by name. `object_align` is not a button because bare "drive at the
// thing you can see" is a step inside a sequence, not a way to drive a match: on
// its own it creeps at whatever standoff Settings was last left on and then sits
// there. It remains a drive mode for routine states (routines/vocab.ts), which
// is where the surrounding "and then what" lives — and where the state says how
// near to get. The robot still accepts the mode over the wire, so a spoken
// command or an older base station is unaffected.
const MODES: { key: Mode; label: string }[] = [
  { key: "teleop", label: "Teleop" },
  { key: "shooter_align", label: "Shooter" },
  { key: "waypoint", label: "Waypoint" },
];

export function ModeControls() {
  const rid = selected.value;
  const current = selectedRobot.value?.mode;
  // A grid rather than a wrapping row: with mode names of unequal length,
  // flex-wrap put one per line at unpredictable widths. Equal cells also make
  // the active mode easier to spot at a glance. An odd count is handled in CSS
  // — the last button spans both columns rather than sitting alone in a row.
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
