import { selectedRobot } from "../net/ws.ts";
import type { GpsStatus, VisionStatus } from "../net/types.ts";

function fmt(v: number | null | undefined, digits = 5): string {
  return v == null ? "—" : v.toFixed(digits);
}

/** What the detector sees. This is the readout you tune the standoff size
 *  against: drive to your stop distance and read `size`. */
function vision(v: VisionStatus): { text: string; color: string } {
  if (!v.ok) return { text: "no model", color: "var(--danger)" };
  if (v.label == null) {
    return { text: `searching · ${v.fps.toFixed(0)} fps`, color: "var(--muted)" };
  }
  // size is null on FOMO models, which report centroids without a size.
  const size = v.size == null ? "n/a" : v.size.toFixed(2);
  const ex = v.ex == null ? "—" : (v.ex >= 0 ? "+" : "") + v.ex.toFixed(2);
  // Metres are absent on an uncalibrated robot, and stay absent — showing a
  // dash here would read as "range broken" when the truth is "never set up".
  const dist = v.dist == null ? "" : ` · ${v.dist.toFixed(1)} m`;
  return {
    text: `${v.label} ${((v.conf ?? 0) * 100).toFixed(0)}% · ex ${ex} · size ${size}${dist}`,
    color: "var(--ok)",
  };
}

/**
 * GPS fix health. The position row says where the robot thinks it is; this
 * says whether to believe it — the difference between "the GPS is broken" and
 * "it has three satellites under a tree".
 */
function gps(g: GpsStatus): { text: string; color: string } {
  if (!g.fix) {
    return { text: `no fix · ${g.sats ?? 0} sats`, color: "var(--danger)" };
  }
  const parts = [`${g.sats ?? "?"} sats`];
  if (g.hdop != null) parts.push(`hdop ${g.hdop.toFixed(1)}`);
  parts.push(`${g.speed.toFixed(1)} m/s`);
  // Under ~2 HDOP is a fix worth navigating on; past 5 it is metres of wander.
  const color = g.hdop == null || g.hdop <= 2
    ? "var(--ok)"
    : g.hdop <= 5
    ? "var(--warn)"
    : "var(--danger)";
  return { text: parts.join(" · "), color };
}

/** BNO085 fused-orientation calibration, 0-3. Below the robot's configured
 *  minimum the heading isn't trusted and navigation falls back to the GPS
 *  track angle — which only exists while moving. Worth a glance before a run. */
function CalibPips({ level }: { level: number }) {
  return (
    <span class="pips" title={`IMU calibration ${level}/3`}>
      {[1, 2, 3].map((i) => (
        <span key={i} class={`pip${i <= level ? " on" : ""}`} />
      ))}
    </span>
  );
}

function Row(
  { k, children, color }: {
    k: string;
    children: preact.ComponentChildren;
    color?: string;
  },
) {
  return (
    <div class="tel-row">
      <span class="k">{k}</span>
      <span class="v" style={color ? `color:${color}` : undefined}>{children}</span>
    </div>
  );
}

export function Telemetry() {
  const r = selectedRobot.value;
  if (!r) return null;
  const v = r.vision ? vision(r.vision) : null;
  const g = r.gps ? gps(r.gps) : null;
  return (
    <div class="telemetry">
      <Row k="mode">
        {r.estop ? <b style="color:var(--danger)">E‑STOP</b> : r.mode}
      </Row>
      <Row k="position">{fmt(r.lat)}, {fmt(r.lon)}</Row>
      <Row k="heading">
        {fmt(r.heading, 1)}°
        {r.imu_calib != null && <CalibPips level={r.imu_calib} />}
      </Row>
      {g && <Row k="gps" color={g.color}>{g.text}</Row>}
      <Row k="battery">{r.battery == null ? "—" : r.battery.toFixed(1) + "%"}</Row>
      {v && <Row k="vision" color={v.color}>{v.text}</Row>}
      <Row k="link" color={r.online ? "var(--ok)" : "var(--danger)"}>
        {r.online ? "online" : "offline"}
        {r.age != null && <span class="tel-age">{r.age.toFixed(1)}s</span>}
      </Row>
    </div>
  );
}
