// What a state can ask, and what it can do.
//
// A hand-written mirror of robot/routine/conditions.py::BUILDERS and
// actions.py::BUILDERS, same arrangement as settings/schema.ts mirroring the
// tunable list: Python decides what is legal and refuses anything else; this
// adds the words a person needs to pick from a menu. Keep the pairs in sync.

export interface ArgSpec {
  key: string;
  label: string;
  kind: "number" | "text" | "mech" | "preset" | "actuator" | "state" | "counter";
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  fallback?: number | string;
}

/** Which drawer of the palette a verb lives in. The palette is the only place a
 *  newcomer learns what the robot can be asked to do, so the grouping IS the
 *  documentation — an alphabetical list of 17 verbs teaches nobody anything. */
export type VerbGroup =
  | "timing"
  | "vision"
  | "navigation"
  | "mechanism"
  | "logic"
  | "operator";

export interface VerbSpec {
  key: string;
  label: string;
  group: VerbGroup;
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
    group: "timing",
    label: "after a delay",
    args: [{ key: "seconds", label: "seconds", kind: "number", min: 0, max: 600, step: 0.1, fallback: 1 }],
  },
  {
    key: "always",
    group: "timing",
    label: "immediately",
    help: "Useful for a pass-through state. The engine takes at most one transition per tick, so a chain of these steps rather than spins.",
    args: [],
  },
  {
    key: "target_visible",
    group: "vision",
    label: "when the camera sees a target",
    chip: "target seen",
    args: [],
  },
  {
    key: "aligned",
    group: "vision",
    label: "when lined up on the target",
    chip: "aligned",
    help: "Read straight off the alignment controller, so it means exactly what that mode means by it.",
    args: [],
  },
  { key: "arrived", group: "vision", label: "when at the standoff distance", chip: "at standoff", args: [] },
  { key: "route_done", group: "navigation", label: "when the route is finished", chip: "route done", args: [] },
  {
    key: "shots",
    group: "mechanism",
    label: "after N shots",
    args: [
      { key: "mech", label: "mechanism", kind: "mech", fallback: "shooter" },
      { key: "at_least", label: "shots", kind: "number", min: 1, max: 99, step: 1, fallback: 1 },
    ],
  },
  {
    key: "mech_ready",
    group: "mechanism",
    label: "when a mechanism is ready",
    chip: "ready",
    args: [{ key: "mech", label: "mechanism", kind: "mech" }],
  },
  {
    key: "heading",
    group: "navigation",
    label: "when pointing a direction",
    chip: "heading",
    args: [
      { key: "of", label: "heading", kind: "number", min: 0, max: 360, step: 1, unit: "°", fallback: 0 },
      { key: "within_deg", label: "within", kind: "number", min: 1, max: 180, step: 1, unit: "°", fallback: 10 },
    ],
  },
  {
    key: "distance_m",
    group: "navigation",
    label: "when near a point",
    chip: "near point",
    help:
      "Distance from a saved place, in metres. Pick the place rather than typing coordinates — then moving the place moves every routine that refers to it.",
    args: [
      { key: "place", label: "place", kind: "place" },
      { key: "at_most", label: "within", kind: "number", min: 0.5, max: 500, step: 0.5, unit: "m", fallback: 2 },
    ],
  },
  {
    key: "event",
    group: "operator",
    label: "when I press a button",
    help: "Fired from the Event button while the routine runs. Cleared whenever a new state is entered, so a press can't satisfy a transition it wasn't meant for.",
    args: [{ key: "name", label: "event name", kind: "text", fallback: "go" }],
  },
  { key: "never", group: "timing", label: "never (hold here)", chip: "never", args: [] },
  {
    key: "counter",
    group: "logic",
    label: "after this has happened N times",
    chip: "count",
    help:
      "Paired with the “count this” action, this is how you repeat something a fixed number of times without drawing the same state over and over. Counters reset every time the routine is run.",
    args: [
      { key: "name", label: "counter", kind: "counter", fallback: "laps" },
      { key: "at_least", label: "times", kind: "number", min: 1, max: 999, step: 1, fallback: 3 },
    ],
  },
  {
    key: "estopped",
    group: "operator",
    label: "when the e-stop is latched",
    chip: "e-stopped",
    help:
      "For a state that has to tidy up after an abort. The engine already stops the machine on an e-stop, so use this only when you want the graph itself to react.",
    args: [],
  },
  {
    key: "all",
    group: "logic",
    label: "when ALL of several things are true",
    chip: "all of",
    help:
      "Combine conditions — “the camera sees a target AND we have been here 2 seconds”. Edit the parts in the inspector.",
    args: [],
  },
  {
    key: "any",
    group: "logic",
    label: "when ANY of several things are true",
    chip: "any of",
    help: "The first one to become true wins. Useful for “finished, or gave up”.",
    args: [],
  },
  {
    key: "not",
    group: "logic",
    label: "when something is NOT true",
    chip: "not",
    help: "Inverts the conditions inside it.",
    args: [],
  },
];

/** Actions. Note that none of them drive — that is the state's `drive` source,
 *  so exactly one thing commands the motors at any moment. */
export const ACTIONS: VerbSpec[] = [
  {
    key: "mech_preset",
    group: "mechanism",
    label: "set a mechanism to a preset",
    args: [
      { key: "mech", label: "mechanism", kind: "mech" },
      { key: "preset", label: "preset", kind: "preset" },
    ],
  },
  {
    key: "mech_power",
    group: "mechanism",
    label: "run a mechanism at a power",
    args: [
      { key: "mech", label: "mechanism", kind: "mech" },
      { key: "actuator", label: "motor (blank = all)", kind: "actuator" },
      { key: "power", label: "power", kind: "number", min: -1, max: 1, step: 0.05, fallback: 1 },
    ],
  },
  {
    key: "mech_stop",
    group: "mechanism",
    label: "stop a mechanism",
    args: [{ key: "mech", label: "mechanism", kind: "mech" }],
  },
  {
    key: "fire",
    group: "mechanism",
    label: "fire a pulse mechanism",
    help: "The mechanism owns its own cycle, so asking repeatedly still yields one activation per cycle.",
    args: [{ key: "mech", label: "mechanism", kind: "mech", fallback: "shooter" }],
  },
  {
    key: "arm",
    group: "mechanism",
    label: "arm the launcher",
    help: "Only allowed in a state that drives with Shooter align — that is where dwell, cooldown and the magazine are enforced. Off unless the robot has been told to permit it.",
    args: [],
  },
  { key: "disarm", group: "mechanism", label: "disarm the launcher", args: [] },
  {
    key: "set_route",
    group: "navigation",
    label: "load a GPS route",
    help:
      "Hands a list of waypoints to the waypoint controller, so a state that drives with Waypoint route follows them. This is what makes a fully autonomous run possible from the graph alone — pair it with “when the route is finished”. Draw the route on the map, then paste it here.",
    args: [{ key: "waypoints", label: "waypoints (lat,lon per line)", kind: "text", fallback: "" }],
  },
  {
    key: "pulse",
    group: "mechanism",
    label: "pulse a mechanism once",
    help:
      "The same one-cycle activation the launcher uses, for any pulse mechanism that is not the launcher.",
    args: [{ key: "mech", label: "mechanism", kind: "mech" }],
  },
  {
    key: "count",
    group: "logic",
    label: "count this",
    help:
      "Adds one to a named counter, so a transition can fire after N passes. Put it in “on enter” — in “every tick” it counts at the control-loop rate, which ends a 3-lap loop about fifty times too early.",
    args: [
      { key: "name", label: "counter", kind: "counter", fallback: "laps" },
      { key: "by", label: "add", kind: "number", min: -99, max: 99, step: 1, fallback: 1 },
    ],
  },
  {
    key: "count_set",
    group: "logic",
    label: "reset a counter",
    help: "Set a counter back to a value so its loop can run again.",
    args: [
      { key: "name", label: "counter", kind: "counter", fallback: "laps" },
      { key: "to", label: "to", kind: "number", min: 0, max: 999, step: 1, fallback: 0 },
    ],
  },
  {
    key: "log",
    group: "operator",
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

/** Which drawer of the palette a state belongs to, from what it actually does.
 *
 * The canvas and the palette have to speak one vocabulary: if "fire the
 * launcher" wears a circle in the menu, the box it produced wears a circle in
 * the graph. Without that the menu teaches a language the graph doesn't use.
 *
 * Driving wins over actions when a state does both, because the drive source is
 * what the state IS for the whole time it is current — the actions are things
 * it does on the way in and out.
 */
export function groupForState(
  state: {
    drive?: { mode?: string };
    on_enter?: { do?: string }[];
    on_tick?: { do?: string }[];
    on_exit?: { do?: string }[];
  },
): VerbGroup | null {
  const mode = state.drive?.mode;
  if (mode && mode !== "stop" && mode !== "hold") return "navigation";
  for (const slot of [state.on_enter, state.on_tick, state.on_exit]) {
    for (const action of slot ?? []) {
      const spec = action?.do ? ACTION_BY_KEY[action.do] : undefined;
      if (spec) return spec.group;
    }
  }
  return null;
}
