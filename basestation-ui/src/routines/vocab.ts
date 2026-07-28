// What a state can ask, and what it can do.
//
// A hand-written mirror of robot/routine/conditions.py::BUILDERS and
// actions.py::BUILDERS, same arrangement as settings/schema.ts mirroring the
// tunable list: Python decides what is legal and refuses anything else; this
// adds the words a person needs to pick from a menu. Keep the pairs in sync.

export interface ArgSpec {
  key: string;
  label: string;
  kind: "number" | "text" | "mech" | "preset" | "actuator" | "state";
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  fallback?: number | string;
}

export interface VerbSpec {
  key: string;
  label: string;
  /** A few words, for a wire on the canvas. The full `label` is a phrase written
   *  to read in a menu; hung on an edge it overruns onto the nodes either side. */
  chip?: string;
  help?: string;
  args: ArgSpec[];
}

/** Conditions, in the order a person is likely to want them. */
export const CONDITIONS: VerbSpec[] = [
  {
    key: "elapsed",
    label: "after a delay",
    args: [{ key: "seconds", label: "seconds", kind: "number", min: 0, max: 600, step: 0.1, fallback: 1 }],
  },
  {
    key: "always",
    label: "immediately",
    help: "Useful for a pass-through state. The engine takes at most one transition per tick, so a chain of these steps rather than spins.",
    args: [],
  },
  {
    key: "target_visible",
    label: "when the camera sees a target",
    chip: "target seen",
    args: [],
  },
  {
    key: "aligned",
    label: "when lined up on the target",
    chip: "aligned",
    help: "Read straight off the alignment controller, so it means exactly what that mode means by it.",
    args: [],
  },
  { key: "arrived", label: "when at the standoff distance", chip: "at standoff", args: [] },
  { key: "route_done", label: "when the route is finished", chip: "route done", args: [] },
  {
    key: "shots",
    label: "after N shots",
    args: [
      { key: "mech", label: "mechanism", kind: "mech", fallback: "shooter" },
      { key: "at_least", label: "shots", kind: "number", min: 1, max: 99, step: 1, fallback: 1 },
    ],
  },
  {
    key: "mech_ready",
    label: "when a mechanism is ready",
    chip: "ready",
    args: [{ key: "mech", label: "mechanism", kind: "mech" }],
  },
  {
    key: "heading",
    label: "when pointing a direction",
    chip: "heading",
    args: [
      { key: "of", label: "heading", kind: "number", min: 0, max: 360, step: 1, unit: "°", fallback: 0 },
      { key: "within_deg", label: "within", kind: "number", min: 1, max: 180, step: 1, unit: "°", fallback: 10 },
    ],
  },
  {
    key: "distance_m",
    label: "when near a point",
    chip: "near point",
    args: [
      { key: "lat", label: "latitude", kind: "number", step: 0.000001, fallback: 0 },
      { key: "lon", label: "longitude", kind: "number", step: 0.000001, fallback: 0 },
      { key: "at_most", label: "within", kind: "number", min: 0.5, max: 500, step: 0.5, unit: "m", fallback: 2 },
    ],
  },
  {
    key: "event",
    label: "when I press a button",
    help: "Fired from the Event button while the routine runs. Cleared whenever a new state is entered, so a press can't satisfy a transition it wasn't meant for.",
    args: [{ key: "name", label: "event name", kind: "text", fallback: "go" }],
  },
  { key: "never", label: "never (hold here)", chip: "never", args: [] },
];

/** Actions. Note that none of them drive — that is the state's `drive` source,
 *  so exactly one thing commands the motors at any moment. */
export const ACTIONS: VerbSpec[] = [
  {
    key: "mech_preset",
    label: "set a mechanism to a preset",
    args: [
      { key: "mech", label: "mechanism", kind: "mech" },
      { key: "preset", label: "preset", kind: "preset" },
    ],
  },
  {
    key: "mech_power",
    label: "run a mechanism at a power",
    args: [
      { key: "mech", label: "mechanism", kind: "mech" },
      { key: "actuator", label: "motor (blank = all)", kind: "actuator" },
      { key: "power", label: "power", kind: "number", min: -1, max: 1, step: 0.05, fallback: 1 },
    ],
  },
  {
    key: "mech_stop",
    label: "stop a mechanism",
    args: [{ key: "mech", label: "mechanism", kind: "mech" }],
  },
  {
    key: "fire",
    label: "fire a pulse mechanism",
    help: "The mechanism owns its own cycle, so asking repeatedly still yields one activation per cycle.",
    args: [{ key: "mech", label: "mechanism", kind: "mech", fallback: "shooter" }],
  },
  {
    key: "arm",
    label: "arm the launcher",
    help: "Only allowed in a state that drives with Shooter align — that is where dwell, cooldown and the magazine are enforced. Off unless the robot has been told to permit it.",
    args: [],
  },
  { key: "disarm", label: "disarm the launcher", args: [] },
  {
    key: "log",
    label: "write a note to the log",
    args: [{ key: "message", label: "message", kind: "text", fallback: "" }],
  },
];

/** Where a state's driving comes from. The controller options delegate to the
 *  real thing, providers and all — the FSM composes autonomy rather than
 *  re-expressing it. */
export const DRIVE_MODES = [
  { value: "stop", label: "Stop" },
  { value: "hold", label: "Hold position" },
  { value: "manual", label: "Fixed throttle/steer" },
  { value: "teleop", label: "Teleop (driver takes over)" },
  { value: "object_align", label: "Object align" },
  { value: "shooter_align", label: "Shooter align" },
  { value: "waypoint", label: "Waypoint route" },
];

export const CONDITION_BY_KEY = Object.fromEntries(
  CONDITIONS.map((c) => [c.key, c]),
);
export const ACTION_BY_KEY = Object.fromEntries(ACTIONS.map((a) => [a.key, a]));

/** A human sentence for one transition, used in the graph and the card header. */
export function describeCondition(when: string, args: Record<string, unknown>): string {
  const spec = CONDITION_BY_KEY[when];
  if (!spec) return when;
  if (when === "elapsed") return `after ${args.seconds ?? "?"}s`;
  if (when === "shots") return `${args.at_least ?? "?"} shots`;
  if (when === "event") return `on “${args.name ?? ""}”`;
  return spec.chip ?? spec.label;
}
