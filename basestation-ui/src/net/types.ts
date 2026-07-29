// Wire types — mirror the Python bridge's /ws contract exactly.
//   inbound : basestation/app.py broadcast_loop() + fleet.py snapshot()
//   outbound: basestation/app.py handle_action()

/** Drive/autonomy mode a robot can be in. */
export type Mode =
  | "teleop"
  | "object_align"
  | "shooter_align"
  | "waypoint"
  | "routine"
  | (string & {});

export type LatLon = [number, number];

/** What the robot's Edge Impulse detector currently sees (robot/sensors/detector.py).
 *  Target fields are absent when nothing is detected. */
export interface VisionStatus {
  ok: boolean; // model loaded and the detector loop is alive
  fps: number; // achieved inference rate
  mock?: boolean; // synthesized target (RS_MOCK_DETECTOR), not a real camera
  label?: string;
  conf?: number; // 0..1
  ex?: number; // horizontal error, -1..1 (0 = centered)
  size?: number | null; // bbox height fraction; null on FOMO (no size available)
  age?: number; // seconds since this detection
}

/** Shooter state, present only while shooter_align is the active mode
 *  (robot/control/shooter_align.py::status). */
export interface ShooterStatus {
  armed: boolean; // operator has permitted firing
  shots: number; // rounds fired this session
  ready: boolean; // on target and dwelling toward a shot
  cool: number; // seconds left on the cooldown, 0 when clear
}

/** One layout mechanism's live state (robot/drive/mechanism.py::status).
 *  Absent on a build whose layout declares no mechanisms. */
export interface MechStatus {
  kind: "power" | "pulse" | (string & {});
  values?: Record<string, number>; // power: actuator -> throttle
  state?: string; // pulse: rest | active | recovering
  count?: number; // pulse: activations so far
  ready?: boolean;
  cool?: number;
}

/** Which state the FSM is in, present only while `routine` is the active mode
 *  (robot/routine/engine.py::status). Non-sticky, like ShooterStatus. */
export interface RoutineStatus {
  id: string | null;
  state: string | null; // the live state's id; null once the routine has ended
  t?: number; // seconds in that state
  drive?: string | null; // the delegate driving, or stop/hold/manual
  done?: boolean;
  why?: string | null; // why it ended
}

/** GPS fix health (gps.py::GPS.telemetry). Optional fields are absent until
 * the module reports them — hdop/alt need a GGA, track needs the rover to move. */
export interface GpsStatus {
  fix: number; // 0 = no fix, 1 = GPS, 2 = DGPS
  sats: number | null; // satellites in use
  speed: number; // ground speed, m/s
  hdop?: number; // horizontal dilution of precision; under ~2 is good
  alt?: number; // altitude, metres
  track?: number; // track angle (course over ground), degrees, 0=N, CW+
  track_age?: number; // seconds since that track angle was measured
}

/** One robot in a fleet snapshot (fleet.py::FleetManager.snapshot). */
export interface Robot {
  robot_id: string;
  mode: Mode;
  estop: boolean;
  left: number; // last commanded left-track output, -1..1
  right: number; // last commanded right-track output, -1..1
  battery: number | null; // percent, or null (real robots don't emit it yet)
  lat: number | null;
  lon: number | null;
  heading: number | null; // degrees, 0=N, CW-positive (BNO085, or the GPS track angle)
  vision: VisionStatus | null; // null when the robot has vision disabled
  imu_calib: number | null; // BNO085 fused-orientation calibration level, 0-3
  gps: GpsStatus | null; // null when the robot has GPS disabled
  shooter?: ShooterStatus | null; // absent unless shooter_align is active
  mech?: Record<string, MechStatus> | null; // absent unless the layout has any
  routine?: RoutineStatus | null; // absent unless `routine` is active
  online: boolean;
  age: number | null; // seconds since last telemetry, or null
  trail: LatLon[]; // breadcrumb of past positions
}

export interface ControllerStatus {
  connected: boolean;
  name: string | null;
}

/** A settings value. Everything crossing the wire is one of these four. */
export type SettingValue = number | boolean | string;

/** Raw gamepad sample from the server-side pygame reader
 *  (basestation/controller_input.py::state). Unmapped on purpose — the mapping
 *  editor's job is to show what the hardware emits so it can be bound. */
export interface GamepadState {
  connected: boolean;
  name: string | null;
  axes: number[];
  buttons: boolean[];
}

/** One robot's tunable config, as the base station has it cached
 *  (basestation/fleet.py::configs). */
export interface RobotConfigEntry {
  rev: number; // bumped on every change; the UI re-syncs when it moves
  config: Record<string, SettingValue>; // flat dotted paths -> values
  result: {
    rejected: Record<string, string>; // path -> why it was refused
    restart: string[]; // applied, but only takes effect after a restart
    save_error: string | null; // applied but not persisted (read-only FS)
    // The edit never reached the robot at all. Config travels over the WiFi
    // link, never the radio (robot/comms/ip_link.py), so a rover that is
    // driving perfectly well can still be unconfigurable — which is exactly the
    // case an operator would otherwise read as "the page is broken".
    error?: string | null;
  } | null;
}

// ---- Documents: the hardware layout and the FSM routines. ----
// Structure, not scalars — see robot/layout.py for why these live apart from
// the tunable config, and why a layout needs a restart while routines do not.

/** One motor or servo (robot/config.py::MotorConfig). */
export interface ActuatorSpec {
  name: string; // unique; also its tuning path, drive.<name>.* / mech.<m>.<name>.*
  label?: string;
  kind: "esc" | "servo";
  channel: number; // Fusion HAT PWM channel, 0-15, unique across the robot
  inverted?: boolean;
  neutral_angle?: number;
  max_angle?: number;
  min_angle?: number;
  deadband?: number;
  max_forward?: number;
  max_reverse?: number;
}

export type DriveKind = "tank" | "servo_steer" | "single" | "none";

/** Which actuators play which role, for the drivetrain kind in use. */
export interface DriveRoles {
  left: string[]; // tank
  right: string[]; // tank
  throttle: string[]; // servo_steer | single
  steer: string; // servo_steer
}

export interface DrivetrainSpec {
  kind: DriveKind;
  actuators: ActuatorSpec[];
  roles: DriveRoles;
  arm_seconds?: number;
  slew_rate?: number;
  steer_gain?: number;
  /** A steered chassis cannot pivot; this creeps so the steering has authority
   *  when a controller asks for arcade(0, steer). 0 disables it. */
  min_pivot_throttle?: number;
}

export interface MechanismSpec {
  name: string;
  label?: string;
  kind: "power" | "pulse";
  enabled?: boolean;
  actuators: ActuatorSpec[];
  presets?: Record<string, Record<string, number>>; // power
  auto_stop_seconds?: number; // power
  rest_angle?: number; // pulse
  active_angle?: number;
  active_seconds?: number;
  recover_seconds?: number;
  cooldown?: number;
  max_activations?: number;
}

export interface LayoutDoc {
  version: number;
  drive: DrivetrainSpec;
  mechanisms: MechanismSpec[];
}

/** A transition's condition. `when` names the check; the rest are its arguments
 *  (robot/routine/conditions.py). `for_seconds` requires it to hold that long
 *  CONTINUOUSLY — the launcher's dwell rule, generalized. */
export interface TransitionSpec {
  when: string;
  to: string;
  for_seconds?: number;
  [arg: string]: unknown;
}

/** Something a state does. Never the drivetrain — that is `drive`'s job, so
 *  exactly one thing commands the motors (robot/routine/actions.py). */
export interface ActionSpec {
  do: string;
  [arg: string]: unknown;
}

export interface RoutineStateSpec {
  id: string;
  /** stop | hold | manual | the name of a controller to delegate driving to. */
  drive?: { mode: string; throttle?: number; steer?: number };
  timeout?: number; // omitted = inherit routines.state_timeout_default
  terminal?: boolean;
  on_enter?: ActionSpec[];
  on_tick?: ActionSpec[];
  on_exit?: ActionSpec[];
  transitions?: TransitionSpec[];
  /** Where this node sits on the editor's canvas. Editor-only: the robot never
   *  interprets these, it just preserves them (robot/routine/schema.py), so the
   *  diagram a teammate opens is the one you drew. Missing or non-finite values
   *  mean "lay me out", not "put me at zero". */
  x?: number;
  y?: number;
}

export interface RoutineSpec {
  id: string;
  name?: string;
  start: string;
  states: RoutineStateSpec[];
  on_end?: "stop" | "restart";
  on_estop?: "abort" | "hold";
  timeout?: number;
}

export interface RoutineDoc {
  version: number;
  routines: RoutineSpec[];
}

/** A robot's verdict on a document we sent it. */
export interface DocResult {
  ok: boolean;
  errors: string[];
  warnings: string[];
  save_error: string | null;
  restart_required?: boolean;
}

/** A tunable field the dashboard could not know about in advance, because the
 *  operator declared the actuator it belongs to (robot/tuning.py::descriptors). */
export interface FieldDescriptor {
  path: string;
  kind: "float" | "int" | "bool" | "enum" | "text";
  lo: number | null;
  hi: number | null;
  choices: string[] | null;
  live: boolean;
  label: string;
  unit: string;
  step: number | null;
  help: string;
  group: string; // "actuator:<name>" | "mech:<name>"
}

/** Everything the Hardware and Routines tabs read, per robot
 *  (basestation/fleet.py::documents). Each has its own revision so saving one
 *  does not push the other back over the radio. */
export interface RobotDocuments {
  layout: LayoutDoc | null;
  layout_rev: number;
  layout_result: DocResult | null;
  routines: RoutineDoc | null;
  routines_rev: number;
  routines_result: DocResult | null;
  fields: FieldDescriptor[];
  fields_rev: number;
}

/** The cold channel: sent on connect and then only when something changes.
 *  Kept out of the 30 Hz fleet frame because a robot's config is ~2.4 KB. */
export interface SettingsMessage {
  type: "settings";
  settings: Record<string, SettingValue>; // "base.*" and "controller.*"
  /** Outcome of the last set_settings — the base station's equivalent of a
   *  robot's RobotConfigEntry.result. Null before anything has been changed. */
  settings_result: {
    applied: Record<string, SettingValue>;
    rejected: Record<string, string>;
    restart: string[];
    save_error: string | null;
  } | null;
  configs: Record<string, RobotConfigEntry>; // by robot_id
  documents: Record<string, RobotDocuments>; // by robot_id
  /** Named field positions, shared by the whole fleet (basestation/places.py). */
  places: Place[];
  places_result: PlacesResult | null;
  gamepad: GamepadState | null;
}

/** What a place is, so the map can draw it differently and an operator can
 *  find the buckets among the markers. Mirrors basestation/places.py::KINDS. */
export type PlaceKind = "bucket" | "marker" | "start" | "hazard";

/** A point somebody stood a rover on and named.
 *
 *  Fleet property, not robot property: the whole point is that one rover is
 *  driven up to a bucket in teleop and every OTHER rover can then use where it
 *  was. They live on the base station and are resolved into plain lat/lon when
 *  a routine is saved, so a robot never has to learn the word. */
export interface Place {
  id: string;
  name: string;
  lat: number;
  lon: number;
  kind: PlaceKind;
}

/** Outcome of the last set_places — which entries the bridge refused, and
 *  whether the file was written. Same shape idea as settings_result. */
export interface PlacesResult {
  count: number;
  rejected: Record<string, string>;
  save_error: string | null;
}

/** Streamed at ui_hz, but only to clients that asked (watch_gamepad). */
export interface GamepadMessage {
  type: "gamepad";
  gamepad: GamepadState | null;
}

// ---- Commanding by voice, by typing, or from an AI over MCP. ----
// Mirrors basestation/command/executor.py. Event-driven rather than a channel:
// these frames go out when somebody says something, and are silent otherwise.

/** What became of one order. `status` is the whole story:
 *  done — it was dispatched; pending — a human has to approve it;
 *  refused — it was understood and rejected (unknown rover, no such place);
 *  unheard — nothing recognisable was said; error — the model or mic failed.
 *
 *  `refused` and `unheard` are deliberately distinct: "you may not do that" and
 *  "I didn't understand you" call for different things from the operator. */
export interface CommandOutcome {
  status: "done" | "pending" | "refused" | "unheard" | "error";
  say: string;
  intent: string | null;
  args: Record<string, unknown>;
  /** Exactly what was (or would be) dispatched — the operator's proof that the
   *  sentence and the command agree. Entries with a `ui` key never leave the
   *  base station; they move this screen (see applyUiActions in state/command.ts). */
  actions: CommandAction[];
  confirm_id: string | null;
  /** voice | text | mcp | ui — who gave the order. Rendered on every row,
   *  because "an AI did that" is the first thing you want to know. */
  source: string;
  transcript: string;
  /** fastpath | model | direct | confirmed — which path recognised it. */
  route: string;
  at: number;
  /** How long recognition took, ms. Null when nothing had to be recognised. */
  ms: number | null;
}

/** One planned step. Either a bridge action (same shape as `Action` below) or a
 *  `ui` instruction that only moves this screen. */
export interface CommandAction {
  ui?: "camera" | "view" | "status" | (string & {});
  view?: string;
  robot_id?: string;
  action?: string;
  [k: string]: unknown;
}

export interface PendingCommand {
  id: string;
  intent: string;
  say: string;
  source: string;
  at: number;
  expires_in: number;
}

/** What the operator is allowed to say right now — the live half, built from
 *  the fleet and the saved places (basestation/command/vocabulary.py). */
export interface CommandVocabulary {
  robots: string[];
  online: string[];
  selected: string | null;
  places: { id: string; name: string }[];
  routines: Record<string, string[]>;
  labels: string[];
  modes: string[];
}

export interface SttStatus {
  available: boolean;
  model: string;
  error: string | null;
  sample_rate: number;
}

export interface CommandMessage {
  type: "command";
  /** False when the language model is switched off. The keyword fast path still
   *  runs — stop, rover names, modes and the camera never needed a model. */
  enabled: boolean;
  model_ok: boolean;
  model: string | null;
  /** The loaded model's id, or why it couldn't be reached. */
  model_note: string;
  base_url: string;
  pending: PendingCommand[];
  history: CommandOutcome[];
  vocabulary: CommandVocabulary;
  stt: SttStatus;
  /** Present on the frame sent the moment a transcript is ready, before the
   *  model has classified it — so the operator sees what was heard while the
   *  model is still thinking. */
  heard?: string;
  heard_conf?: number;
  stt_error?: string;
  /** Present on the frame that reports one command's result. */
  outcome?: CommandOutcome;
}

/** The single message type the bridge pushes at ui_hz (default 30 Hz). */
export interface FleetMessage {
  type: "fleet";
  selected: string | null;
  robots: Robot[];
  controller: ControllerStatus;
  tiles: string | null; // tile URL template the map should load
  tiles_maxzoom: number | null; // for offline upscaling (Leaflet maxNativeZoom)
  tiles_attribution: string | null; // basemap credit line, derived from the source URL
  video?: string[]; // robot_ids with a live FPV feed right now
  drive_hz?: number; // server's radio airtime budget; the touch joystick obeys it
}

// ---- Outbound actions (browser -> bridge). ----

export type Action =
  | { action: "select"; robot_id: string }
  | { action: "mode"; robot_id: string; mode: Mode }
  | { action: "estop"; robot_id: string }
  | { action: "clear_estop"; robot_id: string }
  | { action: "route"; robot_id: string; waypoints: LatLon[] }
  | { action: "drive"; robot_id: string; throttle: number; steer: number }
  | { action: "arm_shooter"; robot_id: string }
  | { action: "disarm_shooter"; robot_id: string }
  | { action: "fire"; robot_id: string }
  // Ask a robot for its full tunable config. Explicit rather than polled: the
  // reply is ~2.4 KB over a radio shared with telemetry.
  | { action: "get_config"; robot_id: string }
  // Change some of it. `save` persists on the robot (default true).
  | {
    action: "set_config";
    robot_id: string;
    config: Record<string, SettingValue>;
    save?: boolean;
  }
  // Documents. Fetched explicitly for the same reason as get_config: they are
  // kilobytes on a radio shared with telemetry, and the editors ask on open.
  | { action: "get_layout"; robot_id: string }
  | { action: "get_routines"; robot_id: string }
  | { action: "get_fields"; robot_id: string }
  // Saved whole, not per field. A half-applied state machine is meaningless,
  // and the transfer is all-or-nothing to match.
  | { action: "set_layout"; robot_id: string; doc: LayoutDoc; save?: boolean }
  | { action: "set_routines"; robot_id: string; doc: RoutineDoc; save?: boolean }
  // Running a routine. Pass-through: the robot owns every rule about what one
  // may do, and the base station is the half that can be disconnected.
  | { action: "select_routine"; robot_id: string; id: string }
  | { action: "routine_cmd"; robot_id: string; cmd: "start" | "stop" | "restart"; id?: string }
  | { action: "routine_event"; robot_id: string; name: string }
  // Bench-test one mechanism. Refused by the robot unless it is in teleop with
  // no e-stop latched, and it expires on its own after 0.4 s.
  | { action: "jog"; robot_id: string; mech: string; actuator?: string; power: number }
  // Base-station settings + gamepad mapping. Local; no radio involved.
  | { action: "set_settings"; settings: Record<string, SettingValue> }
  // Named field positions. Local like set_settings — no radio, no robot: a
  // place only ever reaches a robot as the lat/lon a saved routine resolved it
  // into. Sent as the whole list, because that is what the map is holding.
  | { action: "set_places"; places: Place[] }
  // Subscribe this socket to raw gamepad frames (mapping editor only).
  | { action: "watch_gamepad"; on: boolean }
  // ---- commanding in words ----
  // Push-to-talk. Between {on:true} and {on:false} this socket sends binary
  // frames of raw 16 kHz PCM (net/audio.ts); the bridge transcribes on release.
  | { action: "mic"; on: boolean }
  // A typed order, or one an MCP client wrote. Goes through exactly the same
  // recogniser a spoken one does.
  | { action: "command_text"; text: string; source?: string }
  // A structured intent — from a UI button, or a client that already knows what
  // it wants. Skips recognition, NOT the confirmation gate.
  | { action: "command_intent"; intent: string; args: Record<string, unknown>; source?: string }
  // A human approving (or dismissing) a gated command.
  | { action: "command_confirm"; id: string }
  | { action: "command_cancel"; id: string }
  // Turn the language model off — the fast path keeps working without it.
  | { action: "command_enable"; on: boolean }
  // Re-check whether a model server has appeared. Explicit, never polled: it is
  // a button pressed after starting LM Studio.
  | { action: "command_check" };

export type ConnState = "connecting" | "live" | "reconnecting";
