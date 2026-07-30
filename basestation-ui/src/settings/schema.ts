// How the settings page presents each tunable parameter.
//
// The AUTHORITY on what may be changed and to what range is Python — the robot
// enforces robot/tuning.py::PARAMS and the base station enforces
// basestation/settings.py::PARAMS, both of which clamp rather than trust us.
// This file mirrors those lists to add what a form needs and a validator
// doesn't: grouping, labels, units, step size, and the sentence that explains
// what the knob actually does. Same arrangement as net/types.ts mirroring the
// /ws contract; keep the pairs in sync, and let Python win any disagreement
// (an out-of-range value comes back clamped and the field re-renders).
//
// `live: false` marks a field the running process only picks up on restart —
// serial ports, PWM channels, enable flags. Shown and saved, but badged, so
// nobody spends ten minutes wondering why a new baud rate changed nothing.

import type { FieldDescriptor, SettingValue } from "../net/types.ts";

export type FieldKind = "float" | "int" | "bool" | "enum" | "text";

export interface Field {
  path: string;
  label: string;
  kind: FieldKind;
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  choices?: string[];
  help?: string;
  live?: boolean; // default true
}

export interface Group {
  title: string;
  blurb?: string;
  fields: Field[];
}

// --- terse constructors, so the tables below read as data ------------------

function f(
  path: string,
  label: string,
  min: number,
  max: number,
  step: number,
  extra: Partial<Field> = {},
): Field {
  return { path, label, kind: "float", min, max, step, ...extra };
}

function i(
  path: string,
  label: string,
  min: number,
  max: number,
  extra: Partial<Field> = {},
): Field {
  return { path, label, kind: "int", min, max, step: 1, ...extra };
}

function b(path: string, label: string, extra: Partial<Field> = {}): Field {
  return { path, label, kind: "bool", ...extra };
}

function e(
  path: string,
  label: string,
  choices: string[],
  extra: Partial<Field> = {},
): Field {
  return { path, label, kind: "enum", choices, ...extra };
}

function t(path: string, label: string, extra: Partial<Field> = {}): Field {
  return { path, label, kind: "text", ...extra };
}

/** The five gains of one PID loop, for a named prefix.
 *
 * `gainStep` exists because the loops disagree on units: object align sees a
 * normalized [-1, 1] error, the heading loops see DEGREES, so their useful
 * gains are two orders of magnitude smaller and a 0.01 step can't even reach
 * them.
 */
function pid(prefix: string, iLimitMax = 5, gainStep = 0.01): Field[] {
  return [
    f(`${prefix}.kp`, "Kp", 0, 5, gainStep, {
      help: "Proportional: how hard it corrects for the error it sees right now.",
    }),
    f(`${prefix}.ki`, "Ki", 0, 5, gainStep / 2, {
      help: "Integral: removes steady-state bias, but winds up if you overdo it.",
    }),
    f(`${prefix}.kd`, "Kd", 0, 5, gainStep / 2, {
      help: "Derivative: damping. Fed from the IMU yaw-rate, not a noisy difference.",
    }),
    f(`${prefix}.out_limit`, "Output limit", 0, 1, 0.01, {
      help: "Clamp on the steering output this loop may command.",
    }),
    f(`${prefix}.i_limit`, "Integral limit", 0, iLimitMax, 0.01, {
      help: "Anti-windup clamp on the accumulated integral.",
    }),
  ];
}

/** One actuator's calibration, at any dotted prefix.
 *
 * The two stock track motors reach this as `drive.left` / `drive.right`, which
 * is simply what the default layout names them — the general rule is
 * `drive.<name>` for a drive actuator and `mech.<m>.<name>` for a mechanism's.
 * A robot running a custom layout describes its own actuators instead (see
 * `dynamicGroups`), and this is the shape that gets built from those. */
function motorGroup(p: string, title: string): Group {
  return {
    title,
    blurb:
      "Angles are the ESC's servo range. The usable throw is symmetric about " +
      "neutral, so whichever endpoint is closer to neutral sets it.",
    fields: [
      b(`${p}.inverted`, "Inverted", {
        help: "Flip direction for a track motor mounted facing the other way.",
      }),
      f(`${p}.neutral_angle`, "Neutral angle", -90, 90, 0.5, {
        unit: "°",
        help: "The angle at which this ESC holds stop. Find it with tools/esc_calibrate.py.",
      }),
      f(`${p}.max_angle`, "Forward endpoint", -90, 90, 0.5, { unit: "°" }),
      f(`${p}.min_angle`, "Reverse endpoint", -90, 90, 0.5, { unit: "°" }),
      f(`${p}.deadband`, "Dead band", 0, 0.5, 0.005, {
        help: "Throttle magnitudes below this are treated as neutral.",
      }),
      f(`${p}.max_forward`, "Forward cap", 0, 1, 0.01, {
        help: "Safety cap on forward throttle.",
      }),
      f(`${p}.max_reverse`, "Reverse cap", 0, 1, 0.01, {
        help: "Safety cap on reverse throttle.",
      }),
      i(`${p}.channel`, "PWM channel", 0, 15, {
        live: false,
        help: "Fusion HAT channel. Unique across the robot — the layout refuses two actuators on one.",
      }),
    ],
  };
}

// --- robot config (mirrors robot/tuning.py) --------------------------------

export const ROBOT_GROUPS: Group[] = [
  {
    title: "Control loop",
    blurb:
      "The rate the motors are actually updated at — the floor on teleop latency " +
      "and the granularity of the slew limiter.",
    fields: [
      f("loop_hz", "Loop rate", 1, 200, 1, {
        unit: "Hz",
        help: "50 Hz keeps motion continuous. Below ~20 Hz the slew limiter stops interpolating and steering turns choppy.",
      }),
      f("telemetry_hz", "Telemetry rate", 0, 20, 0.5, {
        unit: "Hz",
        help: "Status frames back to the base station. Lower it to free radio airtime; 0 disables.",
      }),
      f("comms.command_timeout", "Failsafe timeout", 0.05, 5, 0.05, {
        unit: "s",
        help: "Stop if no drive command arrives within this long. Must stay above the base station's 0.25 s keepalive.",
      }),
      // Lives here, next to the telemetry rate, because that is what it spends:
      // it adds the active loop's setpoint, error, output and P/I/D split to
      // every frame. The graphs it feeds appear in the loop groups below.
      b("nav.pid_trace", "Graph the loops", {
        help:
          "Report the active mode's PID loop so the graphs in Object align and " +
          "Waypoint navigation can draw it. Costs radio airtime on every frame — " +
          "switch it on to tune, off to race.",
      }),
      e("heading_source", "Heading source", ["auto", "gps", "imu"], {
        help: "auto = IMU when calibrated, else the GPS track angle.",
      }),
      e("start_mode", "Start mode", [
        "teleop",
        "object_align",
        "shooter_align",
        "waypoint",
        "routine",
      ], { live: false }),
    ],
  },
  {
    title: "Drive",
    fields: [
      f("drive.slew_rate", "Slew rate", 0, 20, 0.1, {
        unit: "/s",
        help: "Max throttle change per second. This is what smooths the base station's ~15 Hz drive frames; 0 disables limiting.",
      }),
      f("drive.arm_seconds", "Arm hold", 0, 10, 0.1, {
        unit: "s",
        live: false,
        help: "Hold neutral this long at boot so the ESCs arm.",
      }),
    ],
  },
  motorGroup("drive.left", "Left motor"),
  motorGroup("drive.right", "Right motor"),
  {
    title: "Object align",
    blurb:
      "Face a detected object, creep toward it, stop at the standoff distance. " +
      "Shooter align uses the same loop.",
    fields: [
      f("align.forward_speed", "Approach speed", 0, 1, 0.01, {
        help: "Creep throttle once roughly on bearing.",
      }),
      f("align.pivot_threshold", "Pivot threshold", 0, 1, 0.01, {
        help: "Horizontal error above this turns in place instead of driving.",
      }),
      f("align.aligned_tolerance", "Aligned tolerance", 0, 0.5, 0.005, {
        help: "Error below this counts as centred — what shooter dwell measures against.",
      }),
      f("align.search_after", "Search after", 0, 10, 0.1, {
        unit: "s",
        help: "Ride out dropouts this long before sweeping to reacquire.",
      }),
      f("align.search_timeout", "Search timeout", 0, 120, 1, {
        unit: "s",
        help: "Give up and stop after sweeping this long.",
      }),
      ...pid("align.pid"),
    ],
  },
  {
    title: "Waypoint navigation",
    blurb:
      "Point-then-go along a route: pivot onto the bearing, then cruise while " +
      "the heading loop trims. There are two heading loops — the fast one runs " +
      "on the IMU's absolute heading, the slow one on the GPS course over " +
      "ground, which only refreshes at the fix rate and only while moving.",
    fields: [
      f("nav.arrive_radius_m", "Arrive radius", 0.2, 50, 0.1, { unit: "m" }),
      f("nav.cruise_speed", "Cruise speed", 0, 1, 0.01),
      f("nav.acquire_speed", "Acquire speed", 0, 1, 0.01, {
        help: "Straight-line throttle used to fix a GPS course when no heading is known, and the arc speed for GPS-heading turns. Must exceed the GPS min-move speed.",
      }),
      f("nav.pivot_threshold_deg", "Pivot threshold", 1, 180, 1, {
        unit: "°",
        help: "Heading error above this pivots in place — on an IMU heading. On a GPS course it arcs at the acquire speed instead, because a pivot freezes the track angle.",
      }),
      ...pid("nav.heading_pid", 180, 0.002),
    ],
  },
  {
    title: "Waypoint heading (GPS only)",
    blurb:
      "Used instead of the gains above whenever heading comes from the GPS " +
      "track angle — heading source 'gps', or 'auto' with no calibrated IMU. " +
      "Keep these well below the IMU gains: this loop closes around a ~1 Hz " +
      "sensor, so pushing it hard just makes the rover weave.",
    fields: pid("nav.gps_heading_pid", 180, 0.002),
  },
  {
    title: "Vision",
    blurb: "What the detector reports, and the geometry the align loop reasons about.",
    fields: [
      f("vision.min_confidence", "Min confidence", 0, 1, 0.01),
      f("vision.max_fps", "Max inference rate", 0.5, 60, 0.5, {
        unit: "fps",
        help: "Inference costs a core; cap it.",
      }),
      f("vision.standoff_size", "Standoff size", 0.05, 1, 0.01, {
        help: "Stop once the box height reaches this fraction of the frame. Calibrate it: park at your stop distance and read the size in telemetry.",
      }),
      f("vision.hfov_deg", "Horizontal FOV", 10, 180, 1, {
        unit: "°",
        help: "Depends on the backend: ~50° post-crop for Edge Impulse, ~66° for the IMX500's real FOV. Scales the D term.",
      }),
      f("vision.search_speed", "Search speed", 0, 1, 0.01, {
        help: "Rotate speed when reacquiring a lost target; 0 disables the sweep.",
      }),
      f("vision.target_timeout", "Target timeout", 0.1, 10, 0.1, {
        unit: "s",
        help: "Drop the target after this long without a fresh detection. Also what makes a dead detector fail safe.",
      }),
      t("vision.target_label", "Target label", {
        help: "Empty tracks any label the model reports.",
      }),
      e("vision.select", "Selection", ["largest", "confidence", "centermost"], {
        help: "Which box to follow when the model reports several.",
      }),
      b("vision.enabled", "Detection enabled", { live: false }),
      e("vision.backend", "Backend", ["auto", "edge_impulse", "imx500"], {
        live: false,
        help: "On-CPU Edge Impulse, or on-sensor IMX500 (AI Camera).",
      }),
      t("vision.model_path", "Edge Impulse model", { live: false }),
      t("vision.imx500_model", "IMX500 network", { live: false }),
      f("vision.imx500_iou", "IMX500 NMS IoU", 0, 1, 0.01, { live: false }),
      i("vision.imx500_max_detections", "IMX500 max boxes", 1, 100, {
        live: false,
      }),
    ],
  },
  {
    title: "Camera",
    fields: [
      b("camera.enabled", "Camera enabled", { live: false }),
      t("camera.device", "Device", {
        live: false,
        help: "auto | imx500 | picamera2 | /dev/videoN | a numeric index.",
      }),
      i("camera.width", "Width", 64, 4096, { unit: "px", live: false }),
      i("camera.height", "Height", 64, 4096, { unit: "px", live: false }),
      i("camera.fps", "Capture rate", 1, 120, { unit: "fps", live: false }),
    ],
  },
  {
    title: "Shooter",
    blurb:
      "Servo geometry and the firing policy. Every rule about when it is safe " +
      "to shoot lives on the robot; this is what those rules read.",
    fields: [
      f("shooter.rest_angle", "Rest angle", -90, 90, 1, {
        unit: "°",
        help: "Home position; also where a disarm or e-stop parks it.",
      }),
      f("shooter.fire_angle", "Fire angle", -90, 90, 1, { unit: "°" }),
      f("shooter.fire_seconds", "Fire hold", 0.05, 5, 0.05, {
        unit: "s",
        help: "Too short and the servo never arrives; too long and it stalls against the stop.",
      }),
      f("shooter.retract_seconds", "Retract", 0.05, 5, 0.05, { unit: "s" }),
      f("shooter.dwell", "Dwell", 0, 10, 0.05, {
        unit: "s",
        help: "Hold the alignment this long before firing. The most important accuracy knob: one centred frame is not evidence.",
      }),
      f("shooter.cooldown", "Cooldown", 0, 60, 0.1, { unit: "s" }),
      i("shooter.max_shots", "Magazine", 0, 999, { help: "0 = unlimited." }),
      b("shooter.require_arm", "Require arming", {
        help: "Leave on. Arming is dropped on mode exit and on e-stop.",
      }),
      b("shooter.require_arrived", "Require standoff", {
        help: "Also require the standoff distance, not just the bearing. Skipped automatically when the model reports no size.",
      }),
      b("shooter.enabled", "Shooter fitted", { live: false }),
      i("shooter.channel", "PWM channel", 0, 15, {
        live: false,
        help: "Start at 2 — 0 and 1 are the drive ESCs.",
      }),
    ],
  },
  {
    title: "GPS",
    fields: [
      f("gps.fix_timeout", "Fix timeout", 0.5, 60, 0.5, {
        unit: "s",
        help: "Treat the fix as lost after this long without an update.",
      }),
      f("gps.min_move_mps", "Min move speed", 0, 5, 0.05, {
        unit: "m/s",
        help: "Below this the track angle is noise, so the last heading is held.",
      }),
      b("gps.enabled", "GPS enabled", { live: false }),
      t("gps.port", "Serial port", { live: false }),
      i("gps.baud", "Baud", 1200, 921600, { live: false }),
      i("gps.update_rate_ms", "Fix interval", 100, 10000, {
        unit: "ms",
        live: false,
        help: "1000 = 1 Hz. Below ~200 ms the sentences don't fit at 9600 baud and read as no fix.",
      }),
    ],
  },
  {
    title: "IMU",
    fields: [
      f("imu.heading_offset_deg", "Heading offset", -180, 180, 0.5, {
        unit: "°",
        help: "Rotation that aligns the sensor's yaw with the robot's forward axis and true North.",
      }),
      b("imu.invert", "Invert yaw", {
        help: "Flip the sign if the board is mounted mirrored.",
      }),
      i("imu.min_calib", "Min calibration", 0, 3, {
        help: "Below this the heading isn't trusted and the fusion falls back to GPS.",
      }),
      b("imu.persist_calibration", "Save calibration", {
        help: "Let the BNO08x save its converged calibration to its own flash.",
      }),
      b("imu.enabled", "IMU enabled", { live: false }),
      i("imu.i2c_address", "I²C address", 0x08, 0x77, {
        live: false,
        help: "0x4a by default; 0x4b if DI/AD0 is pulled high.",
      }),
    ],
  },
  {
    title: "FPV video",
    blurb:
      "The picture needs shared WiFi — the XBee radio cannot carry video. " +
      "Switching it on and pointing it somewhere do not: those go over the " +
      "radio and take effect on the next frame.",
    fields: [
      b("fpv.enabled", "Streaming enabled", {
        help:
          "Starts and stops the feed on the robot, opening its camera the " +
          "first time it is needed. No restart either way.",
      }),
      t("fpv.base_host", "Base station host", {
        help:
          "Hostname or IP of the machine running this base station — the one " +
          "the robot fires video at. Change it here and the rover re-aims.",
      }),
      i("fpv.base_port", "UDP port", 1, 65535),
      i("fpv.fps", "Frame rate", 1, 60, { unit: "fps" }),
      i("fpv.jpeg_quality", "JPEG quality", 1, 100, {
        help: "Lower is smaller packets and less bandwidth.",
      }),
    ],
  },
  {
    title: "Radio & identity",
    blurb: "Changing these needs a restart of the robot service.",
    fields: [
      t("robot_id", "Robot ID", {
        live: false,
        help: "Unique on the shared XBee channel.",
      }),
      t("comms.port", "XBee port", { live: false }),
      i("comms.baud", "XBee baud", 1200, 921600, {
        live: false,
        help: "Must match the base station's, or the slower side's buffer backs up and latency grows without bound.",
      }),
      t("comms.base_host", "Base station host (WiFi)", {
        live: false,
        help: "Send config, layouts and routines over WiFi instead of the radio — a snapshot is ~2.9 KB, which is half a second of shared airtime. Blank disables it. Out of WiFi range these fall back to the radio automatically; driving and telemetry never move off it.",
      }),
      i("comms.base_port", "Base station port (WiFi)", 1, 65535, {
        live: false,
        help: "Must match the base station's --bulk-port (default 5006).",
      }),
    ],
  },
];

// --- base station (mirrors basestation/settings.py) ------------------------

export const BASE_GROUPS: Group[] = [
  {
    title: "Link & refresh",
    blurb:
      "An airtime budget, not a feel knob. A drive frame is ~62 B ≈ 11 ms at " +
      "57600, and telemetry is already sharing the channel.",
    fields: [
      f("base.drive_hz", "Drive command rate", 1, 60, 1, {
        unit: "Hz",
        help: "Max rate drive frames go out over the radio. The touch joystick obeys the same budget. Lower it for a slow or 9600 link.",
      }),
      f("base.ui_hz", "Dashboard refresh", 1, 60, 1, {
        unit: "Hz",
        help: "How often the fleet snapshot is pushed to the browser.",
      }),
      f("base.video_hz", "Video frame rate", 1, 60, 1, {
        unit: "fps",
        help: "Max MJPEG rate served to browsers.",
      }),
      f("base.controller_hz", "Gamepad poll rate", 5, 120, 1, {
        unit: "Hz",
        live: false,
      }),
    ],
  },
  {
    title: "Map",
    fields: [
      t("base.tiles", "Tile URL template", {
        help: "Set to /tiles/{z}/{x}/{y}.png to serve the offline cache.",
      }),
      i("base.trail_max", "Trail length", 0, 5000, {
        unit: "pts",
        help: "Breadcrumb points kept per robot.",
      }),
    ],
  },
];

// --- gamepad mapping -------------------------------------------------------

export const AXIS_FIELDS: Field[] = [
  i("controller.axis_steer", "Steer axis", 0, 15),
  i("controller.axis_r2", "Forward trigger axis", 0, 15),
  i("controller.axis_l2", "Reverse trigger axis", 0, 15),
];

/** Buttons the mapping can bind, in the order the editor lists them. */
export const BUTTON_FIELDS: { path: string; label: string; help?: string }[] = [
  { path: "controller.btn_estop", label: "E‑STOP" },
  { path: "controller.btn_clear", label: "Clear e‑stop" },
  { path: "controller.btn_teleop", label: "Mode: teleop" },
  { path: "controller.btn_object_align", label: "Mode: object align" },
  { path: "controller.btn_shooter_align", label: "Mode: shooter align" },
  { path: "controller.btn_waypoint", label: "Mode: waypoint" },
  { path: "controller.btn_arm_shooter", label: "Arm shooter" },
  { path: "controller.btn_fire", label: "Fire" },
];

export const FEEL_FIELDS: Field[] = [
  f("controller.deadzone", "Dead zone", 0, 0.5, 0.005, {
    help: "Stick and trigger movement smaller than this reads as zero.",
  }),
  f("controller.trigger_rest", "Trigger rest value", -1, 1, 0.05, {
    help: "What an untouched trigger reports. SDL uses -1; some drivers report 0. Wrong here and the triggers stay dead until you pull and release one.",
  }),
  f("controller.throttle_gain", "Throttle authority", 0.1, 1, 0.05, {
    help: "Scales throttle after the dead zone — a trainer mode.",
  }),
  f("controller.steer_gain", "Steering authority", 0.1, 1, 0.05, {
    help: "Lower it for a chassis that darts.",
  }),
  b("controller.invert_steer", "Invert steering"),
];

/** Index used by a binding that is deliberately unassigned. */
export const UNBOUND = -1;

const ALL_FIELDS: Field[] = [
  ...ROBOT_GROUPS.flatMap((g) => g.fields),
  ...BASE_GROUPS.flatMap((g) => g.fields),
  ...AXIS_FIELDS,
  ...FEEL_FIELDS,
  ...BUTTON_FIELDS.map((btn) =>
    i(btn.path, btn.label, UNBOUND, 31, { help: btn.help })
  ),
];

export const FIELD_BY_PATH: Record<string, Field> = Object.fromEntries(
  ALL_FIELDS.map((field) => [field.path, field]),
);

// --- fields this file cannot know about ------------------------------------
//
// Everything above is a hand-written mirror of a Python list, which works
// because a stock build's parameters are fixed. A robot running its own layout
// has parameters named after actuators the operator invented ten seconds ago,
// so it describes those itself (robot/tuning.py::descriptors) and we build the
// form from the description. Python stays the authority either way — it clamps
// whatever we send.

/** Turn a robot's field descriptors into groups, one per actuator/mechanism. */
export function dynamicGroups(fields: FieldDescriptor[]): Group[] {
  const byGroup = new Map<string, FieldDescriptor[]>();
  for (const descriptor of fields) {
    const key = descriptor.group || "other";
    const bucket = byGroup.get(key);
    if (bucket) bucket.push(descriptor);
    else byGroup.set(key, [descriptor]);
  }
  return [...byGroup.entries()].map(([key, described]) => ({
    title: groupTitle(key),
    blurb: key.startsWith("actuator:")
      ? "Angles are the ESC's servo range. The usable throw is symmetric about " +
        "neutral, so whichever endpoint is closer to neutral sets it."
      : undefined,
    fields: described.map(toField),
  }));
}

function groupTitle(key: string): string {
  const [kind, name = ""] = key.split(":");
  const pretty = name.replace(/_/g, " ");
  if (kind === "actuator") return `${sentenceCase(pretty)} motor`;
  if (kind === "mech") return sentenceCase(pretty);
  return "Other";
}

function sentenceCase(text: string): string {
  return text ? text[0].toUpperCase() + text.slice(1) : text;
}

function toField(d: FieldDescriptor): Field {
  return {
    path: d.path,
    label: sentenceCase(d.label || d.path.split(".").pop() || d.path),
    kind: d.kind,
    min: d.lo ?? undefined,
    max: d.hi ?? undefined,
    step: d.step ?? (d.kind === "int" ? 1 : 0.01),
    choices: d.choices ?? undefined,
    help: d.help || undefined,
    live: d.live,
    unit: d.unit || undefined,
  };
}

/**
 * The groups to render for a robot, given whatever descriptors it has sent.
 *
 * With none — an older robot, or one whose descriptors haven't arrived yet —
 * this is exactly `ROBOT_GROUPS`, so the page looks and behaves as it always
 * did. With descriptors, the hand-written `drive.left`/`drive.right` groups are
 * replaced by the robot's own, in the same position.
 */
export function robotGroupsFor(fields: FieldDescriptor[] | undefined): Group[] {
  if (!fields || fields.length === 0) return ROBOT_GROUPS;
  const generated = dynamicGroups(fields);
  const stockMotors = new Set(["Left motor", "Right motor"]);
  const kept = ROBOT_GROUPS.filter((g) => !stockMotors.has(g.title));
  const at = kept.findIndex((g) => g.title === "Drive") + 1;
  return [...kept.slice(0, at), ...generated, ...kept.slice(at)];
}

/** A field's presentation, static table first, then the robot's descriptors. */
export function fieldFor(
  path: string,
  fields: FieldDescriptor[] | undefined,
): Field | undefined {
  const known = FIELD_BY_PATH[path];
  if (known) return known;
  const described = fields?.find((d) => d.path === path);
  return described ? toField(described) : undefined;
}

/** Format a value the way its field wants to be read. */
export function formatValue(field: Field, value: SettingValue | undefined): string {
  if (value == null) return "—";
  if (field.kind === "bool") return value ? "on" : "off";
  if (field.kind === "int" && field.path === "imu.i2c_address") {
    return `0x${Number(value).toString(16)}`;
  }
  if (typeof value !== "number") return String(value);
  // Show as many decimals as the step implies, so 0.005 steps don't render as
  // "0.01" and look like the field ignored the change.
  const step = field.step ?? 1;
  const decimals = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)));
  return value.toFixed(decimals);
}
