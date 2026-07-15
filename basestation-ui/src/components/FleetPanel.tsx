import { robots, selected, selectRobot } from "../net/ws.ts";
import type { Robot } from "../net/types.ts";

// Split fill bar for a signed -1..1 track output (ports app.js::bar()).
function barStyle(v: number): string {
  const pct = Math.min(Math.abs(v), 1) * 50;
  const geom = v >= 0 ? `left:50%;width:${pct}%` : `left:${50 - pct}%;width:${pct}%`;
  const color = v >= 0 ? "var(--accent)" : "var(--warn)";
  return `${geom};background:${color}`;
}

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
      <div class="bars">
        <div class="bar-wrap">
          <div class="bar-label">L {r.left.toFixed(2)}</div>
          <div class="bar-track"><div class="bar-fill" style={barStyle(r.left)} /></div>
        </div>
        <div class="bar-wrap">
          <div class="bar-label">R {r.right.toFixed(2)}</div>
          <div class="bar-track"><div class="bar-fill" style={barStyle(r.right)} /></div>
        </div>
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
