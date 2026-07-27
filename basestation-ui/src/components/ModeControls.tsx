import { selected, selectedRobot, send } from "../net/ws.ts";
import type { Mode } from "../net/types.ts";

const MODES: { key: Mode; label: string }[] = [
  { key: "teleop", label: "Teleop" },
  { key: "object_align", label: "Object align" },
  { key: "shooter_align", label: "Shooter" },
  { key: "waypoint", label: "Waypoint" },
];

export function ModeControls() {
  const rid = selected.value;
  const current = selectedRobot.value?.mode;
  // A 2x2 grid rather than a wrapping row: with four modes of unequal name
  // length, flex-wrap left "Object align" on two lines and "Waypoint" alone on
  // a third. Equal cells also make the active mode easier to spot at a glance.
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
