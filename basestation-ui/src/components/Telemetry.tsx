import { selectedRobot } from "../net/ws.ts";

function fmt(v: number | null | undefined, digits = 5): string {
  return v == null ? "—" : v.toFixed(digits);
}

export function Telemetry() {
  const r = selectedRobot.value;
  if (!r) return null;
  return (
    <div class="telemetry">
      <div class="tel-row">
        <span class="k">mode</span>
        <span class="v">{r.mode}</span>
      </div>
      <div class="tel-row">
        <span class="k">position</span>
        <span class="v">{fmt(r.lat)}, {fmt(r.lon)}</span>
      </div>
      <div class="tel-row">
        <span class="k">heading</span>
        <span class="v">{fmt(r.heading, 1)}°</span>
      </div>
      <div class="tel-row">
        <span class="k">battery</span>
        <span class="v">{r.battery == null ? "—" : r.battery.toFixed(1) + "%"}</span>
      </div>
      <div class="tel-row">
        <span class="k">link</span>
        <span class="v" style={r.online ? "color:var(--ok)" : "color:var(--danger)"}>
          {r.online ? "online" : "offline"}
        </span>
      </div>
    </div>
  );
}
