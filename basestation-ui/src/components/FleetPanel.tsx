import { robots, selected, selectRobot } from "../net/ws.ts";
import type { Robot } from "../net/types.ts";

// Split fill bar for a signed -1..1 track output (ports app.js::bar()).
//
// Driven by transform rather than by `left`/`width`: this bar moves on every
// telemetry frame, once per robot per track, and animating the box means the
// browser re-lays-out the rail at telemetry rate on a Raspberry Pi. The element
// is the full width of its track and gets scaled down from its left edge, so
// the same geometry comes out of the compositor instead.
function barStyle(v: number): string {
  const pct = Math.min(Math.abs(v), 1) * 50;
  const from = v >= 0 ? 50 : 50 - pct;
  const color = v >= 0 ? "var(--accent)" : "var(--warn)";
  return `transform:translateX(${from}%) scaleX(${pct / 100});background:${color}`;
}

/* Drawn, not the 🔋 emoji it replaces. An emoji is painted from the system's
   colour font: it ignored `currentColor`, so on the selected card — the one
   surface in the rail that is filled teal — it stayed a small green cell on a
   teal ground, and it was the only glyph in the console that did not follow
   the rendition. */
function BatteryIcon() {
  return (
    <svg viewBox="0 0 24 24" width="12" height="12" aria-hidden="true">
      <rect
        x="1.6"
        y="6.4"
        width="17.6"
        height="11.2"
        rx="2.6"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
      />
      <rect x="20.8" y="9.8" width="2.4" height="4.4" rx="1.2" fill="currentColor" />
    </svg>
  );
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
        <span class="batt">
          <BatteryIcon />
          <span aria-label={`battery ${batt}`}>{batt}</span>
        </span>
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
