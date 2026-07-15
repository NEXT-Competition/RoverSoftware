import { selected, selectedRobot, send } from "../net/ws.ts";
import type { Mode } from "../net/types.ts";

const MODES: { key: Mode; label: string }[] = [
  { key: "teleop", label: "Teleop" },
  { key: "color_align", label: "Color align" },
  { key: "waypoint", label: "Waypoint" },
];

export function ModeControls() {
  const rid = selected.value;
  const current = selectedRobot.value?.mode;
  return (
    <div class="btn-row" style="flex-wrap:wrap">
      {MODES.map((m) => (
        <button
          key={m.key}
          class={`btn${current === m.key ? " active" : ""}`}
          style="flex:1 1 30%"
          disabled={!rid}
          onClick={() => rid && send({ action: "mode", robot_id: rid, mode: m.key })}
        >
          {m.label}
        </button>
      ))}
    </div>
  );
}
