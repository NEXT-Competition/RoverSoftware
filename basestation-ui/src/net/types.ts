// Wire types — mirror the Python bridge's /ws contract exactly.
//   inbound : basestation/app.py broadcast_loop() + fleet.py snapshot()
//   outbound: basestation/app.py handle_action()

/** Drive/autonomy mode a robot can be in. */
export type Mode =
  | "teleop"
  | "object_align"
  | "shooter_align"
  | "waypoint"
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
  online: boolean;
  age: number | null; // seconds since last telemetry, or null
  trail: LatLon[]; // breadcrumb of past positions
}

export interface ControllerStatus {
  connected: boolean;
  name: string | null;
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
  | { action: "fire"; robot_id: string };

export type ConnState = "connecting" | "live" | "reconnecting";
