import { robots, selected, selectRobot } from "../net/ws.ts";
import type { Robot } from "../net/types.ts";

function statusText(r: Robot): string {
  if (r.online) return "live";
  if (r.age != null) return `${r.age}s ago`;
  return "no signal";
}

function RobotCard({ r, sel }: { r: Robot; sel: boolean }) {
  const batt = r.battery == null ? "—" : `${r.battery.toFixed(0)}%`;
  return (
    <li class={`robot${sel ? " sel" : ""}`} onClick={() => selectRobot(r.robot_id)}>
      <div class="top">
        <span class="name">
          <span class={`dot ${r.online ? "online" : "offline"}`} />
          {r.robot_id}
        </span>
        <span class="mode">
          {r.estop ? <span class="estop-flag">E‑STOP</span> : r.mode}
        </span>
      </div>
      <div class="meta">
        <span>🔋 {batt}</span>
        <span>{statusText(r)}</span>
      </div>
    </li>
  );
}

export function FleetPanel() {
  const list = robots.value;
  const sel = selected.value;
  return (
    <section class="rail-section">
      <div class="section-title" style="margin-bottom:10px">
        <span class="eyebrow">Fleet</span>
        <span class="eyebrow">{list.length}</span>
      </div>
      {list.length === 0
        ? <p class="hint">No robots yet — waiting for telemetry…</p>
        : (
          <ul class="fleet">
            {list.map((r) => (
              <RobotCard key={r.robot_id} r={r} sel={r.robot_id === sel} />
            ))}
          </ul>
        )}
    </section>
  );
}
