import { selectedRobot } from "../net/ws.ts";
import type {
  EncoderStatus,
  GpsStatus,
  SonarStatus,
  VisionStatus,
} from "../net/types.ts";

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
  const parts = [
    `${v.label} ${((v.conf ?? 0) * 100).toFixed(0)}%`,
    `ex ${ex}`,
    `size ${size}`,
  ];
  // A tilde means INFERRED — box height divided by a constant — and a bare
  // number means the ultrasonic measured it. One character, because the
  // difference decides whether this is something to drive on or something to
  // sanity-check, and there is no room on a 57600-baud radio for a legend.
  if (v.dist != null) {
    parts.push(`${v.src === "v" ? "~" : ""}${v.dist.toFixed(2)} m`);
  }
  return { text: parts.join(" · "), color: "var(--ok)" };
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

/**
 * Wheel speed, and whether the two sides agree.
 *
 * The mismatch is the number, not the speeds: "142 / 137 rpm" reads as two
 * facts you then have to subtract, and the thing you are watching for is the
 * gap. So the gap leads, and it is what carries the colour — this is the
 * readout you tune `drive.trim` against, and the one that tells you the rover
 * is curving before the map does.
 */
function encoders(e: EncoderStatus): { text: string; color: string } {
  if (e.fault) {
    return {
      text: `${e.fault} encoder not turning — matching off`,
      color: "var(--danger)",
    };
  }
  const speeds = Object.values(e.rpm);
  if (speeds.length === 0) return { text: "no reading", color: "var(--muted)" };
  const fastest = Math.max(...speeds);
  const slowest = Math.min(...speeds);
  const spread = fastest - slowest;
  const parts = Object.entries(e.rpm).map(([k, v]) => `${k} ${v.toFixed(0)}`);
  const text = speeds.length > 1
    ? `Δ${spread.toFixed(0)} rpm · ${parts.join(" · ")}`
    : `${parts[0]} rpm`;
  // A tenth of the faster wheel is where a straight line becomes a visible arc.
  // Scaled rather than absolute, because 5 rpm apart is nothing at 200 and a
  // rover driving in a circle at 20. At a standstill everything is 0 and the
  // ratio is meaningless, so say nothing rather than shout.
  //
  // Coloured whatever the mode, including `off`. That is deliberate: `off` is
  // when a drift is going UNCORRECTED, so it is the one time the number most
  // needs to catch an eye — greying it would hide precisely the reading that
  // should send someone to switch the loop on.
  const scale = Math.max(Math.abs(fastest), Math.abs(slowest));
  const color = scale < 1
    ? "var(--muted)"
    : spread <= scale * 0.05
    ? "var(--ok)"
    : spread <= scale * 0.1
    ? "var(--warn)"
    : "var(--danger)";
  return { text, color };
}

/**
 * The ultrasonic, and what the collision guard is doing with it.
 *
 * The STATE leads, not the distance, because that is the question an operator
 * actually has: a rover that has stopped answering the forward stick needs to
 * say why on the row you were already looking at. "0.28 m" alone leaves you to
 * remember what the stop distance was set to.
 *
 * The awkward case is a missing distance, which means two opposite things —
 * nothing is in range, or nothing is listening. `mute` is the robot's own
 * verdict on that (it has pinged and never once heard an echo), and it is
 * coloured as the fault it almost always is, because a silent sensor on a rover
 * whose driver believes it is protected is worse than no sensor at all.
 */
function sonar(s: SonarStatus): { text: string; color: string } {
  if (s.off) return { text: "not running", color: "var(--muted)" };
  if (s.mute) {
    return { text: "no echo since boot — check the wiring", color: "var(--danger)" };
  }
  const distance = s.d == null ? "clear" : `${s.d.toFixed(2)} m`;
  if (s.state === "stop") {
    return { text: `${distance} · HOLDING`, color: "var(--danger)" };
  }
  if (s.state === "slow") return { text: `${distance} · slowing`, color: "var(--warn)" };
  if (s.state === "off") {
    // Fitted and reporting, but not allowed to intervene. Said plainly rather
    // than left to be inferred from a distance that never does anything.
    return { text: `${distance} · avoidance off`, color: "var(--muted)" };
  }
  return { text: distance, color: "var(--ok)" };
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
  const e = r.enc ? encoders(r.enc) : null;
  const s = r.sonar ? sonar(r.sonar) : null;
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
      {/* Kept up with heading and position rather than down with vision: a
          wheel mismatch turns into a heading that drifts off the line you
          asked for, so these are the rows you read together. */}
      {e && <Row k="wheels" color={e.color}>{e.text}</Row>}
      {/* Directly under the wheels: both rows answer "why is the rover not
          doing what I asked?", and this is the one that can be answering it
          on purpose. */}
      {s && <Row k="ahead" color={s.color}>{s.text}</Row>}
      <Row k="battery">{r.battery == null ? "—" : r.battery.toFixed(1) + "%"}</Row>
      {v && <Row k="vision" color={v.color}>{v.text}</Row>}
      <Row k="link" color={r.online ? "var(--ok)" : "var(--danger)"}>
        {r.online ? "online" : "offline"}
        {r.age != null && <span class="tel-age">{r.age.toFixed(1)}s</span>}
      </Row>
    </div>
  );
}
