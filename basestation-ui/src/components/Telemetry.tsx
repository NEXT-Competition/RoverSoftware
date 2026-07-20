import { selectedRobot } from "../net/ws.ts";
import type { VisionStatus } from "../net/types.ts";

function fmt(v: number | null | undefined, digits = 5): string {
  return v == null ? "—" : v.toFixed(digits);
}

/** What the detector sees. This is the readout you tune RS_VISION_STANDOFF
 *  against: drive to your stop distance and read `size`. */
function vision(v: VisionStatus): { text: string; color: string } {
  if (!v.ok) return { text: "no model", color: "var(--danger)" };
  if (v.label == null) return { text: `searching · ${v.fps.toFixed(0)} fps`, color: "var(--muted)" };
  // size is null on FOMO models, which report centroids without a size.
  const size = v.size == null ? "n/a" : v.size.toFixed(2);
  const ex = v.ex == null ? "—" : (v.ex >= 0 ? "+" : "") + v.ex.toFixed(2);
  return {
    text: `${v.label} ${((v.conf ?? 0) * 100).toFixed(0)}% · ex ${ex} · size ${size}`,
    color: "var(--ok)",
  };
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
      {r.vision && (
        <div class="tel-row">
          <span class="k">vision</span>
          <span class="v" style={`color:${vision(r.vision).color}`}>
            {vision(r.vision).text}
          </span>
        </div>
      )}
      <div class="tel-row">
        <span class="k">link</span>
        <span class="v" style={r.online ? "color:var(--ok)" : "color:var(--danger)"}>
          {r.online ? "online" : "offline"}
        </span>
      </div>
    </div>
  );
}
