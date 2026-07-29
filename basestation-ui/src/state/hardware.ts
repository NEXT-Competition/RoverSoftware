// Editing state for the Hardware tab.
//
// Unlike the tuning page, this is DRAFT-then-SAVE. state/settings.ts commits
// each field on its own, which is right for a scalar — the robot clamps it and
// answers, and a slider that lagged a round trip would be unusable. A layout is
// not a scalar. Half of one is a robot with one drive motor, and the transfer
// is deliberately all-or-nothing to match (robot/comms/doc_transfer.py), so the
// editor holds a whole document locally and sends it when you press Save.
//
// The draft is seeded from whatever the robot reported. A robot that has never
// been given a layout still reports one — the stock tank layout it is running —
// so nobody ever faces a blank page.

import { computed, signal } from "@preact/signals";
import type {
  ActuatorSpec,
  DriveKind,
  DrivetrainSpec,
  LayoutDoc,
  MechanismSpec,
} from "../net/types.ts";
import { robotDocuments, send } from "../net/ws.ts";
import { targetRobot } from "./settings.ts";

/** The document being edited, or null when it matches what the robot has. */
const draft = signal<LayoutDoc | null>(null);
/** The robot the draft belongs to, so switching rovers can't cross-contaminate. */
const draftOwner = signal<string | null>(null);

/** True while a jog control is unlocked. Deliberately not persisted: "the robot
 *  is on blocks" is a claim about right now, and it should not survive a
 *  reload, a rover switch, or someone else opening the page. */
export const jogUnlocked = signal(false);

export const serverLayout = computed<LayoutDoc | null>(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.layout ?? null : null;
});

export const layoutResult = computed(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.layout_result ?? null : null;
});

/** What the editor shows: the local draft if there is one, else the robot's. */
export const layout = computed<LayoutDoc | null>(() => {
  if (draft.value && draftOwner.value === targetRobot.value) return draft.value;
  return serverLayout.value;
});

export const layoutDirty = computed(() =>
  draft.value != null && draftOwner.value === targetRobot.value
);

function edit(mutate: (doc: LayoutDoc) => void): void {
  const base = layout.value;
  if (!base) return;
  const next: LayoutDoc = JSON.parse(JSON.stringify(base));
  mutate(next);
  draftOwner.value = targetRobot.value;
  draft.value = next;
}

export function discardLayout(): void {
  draft.value = null;
  draftOwner.value = null;
}

export function refreshLayout(): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "get_layout", robot_id: rid });
}

export function saveLayout(): void {
  const rid = targetRobot.value;
  const doc = layout.value;
  if (!rid || !doc) return;
  send({ action: "set_layout", robot_id: rid, doc });
  // The draft is kept until the robot answers. Dropping it here would snap the
  // form back to the old layout for the whole round trip, and a refused layout
  // would silently lose the edit that caused the refusal.
}

/** Called when a save comes back clean, from the Hardware tab's effect. */
export function layoutAccepted(): void {
  discardLayout();
}

// --- drivetrain --------------------------------------------------------------

export function setDriveKind(kind: DriveKind): void {
  edit((doc) => {
    doc.drive.kind = kind;
    // Roles that don't apply to the new kind are cleared rather than left
    // dangling: a `steer` naming an actuator the operator later deletes would
    // fail validation with a message about a role nobody can see.
    if (kind === "tank") {
      doc.drive.roles.throttle = [];
      doc.drive.roles.steer = "";
    } else if (kind === "servo_steer" || kind === "single") {
      doc.drive.roles.left = [];
      doc.drive.roles.right = [];
      if (kind === "single") doc.drive.roles.steer = "";
    } else {
      doc.drive.roles = { left: [], right: [], throttle: [], steer: "" };
    }
  });
}

export function setDriveField<K extends keyof DrivetrainSpec>(
  key: K,
  value: DrivetrainSpec[K],
): void {
  edit((doc) => {
    doc.drive[key] = value;
  });
}

export function toggleRole(
  role: "left" | "right" | "throttle",
  name: string,
): void {
  edit((doc) => {
    const list = doc.drive.roles[role];
    const at = list.indexOf(name);
    if (at >= 0) list.splice(at, 1);
    else list.push(name);
  });
}

export function setSteerActuator(name: string): void {
  edit((doc) => {
    doc.drive.roles.steer = name;
  });
}

// --- actuators ---------------------------------------------------------------

function uniqueName(taken: Set<string>, stem: string): string {
  if (!taken.has(stem)) return stem;
  for (let n = 2; n < 100; n++) {
    if (!taken.has(`${stem}${n}`)) return `${stem}${n}`;
  }
  return `${stem}_x`;
}

/** The lowest PWM channel nothing has claimed, so a new actuator starts valid.
 *  The robot validates this too — two actuators on one channel move together
 *  and neither answers its own commands, which reads as a wiring fault. */
export function freeChannel(doc: LayoutDoc): number {
  const used = new Set<number>();
  for (const a of doc.drive.actuators) used.add(a.channel);
  for (const m of doc.mechanisms) for (const a of m.actuators) used.add(a.channel);
  for (let ch = 0; ch < 16; ch++) if (!used.has(ch)) return ch;
  return 15;
}

/** Every channel claimed more than once, for the inline warning. */
export const channelConflicts = computed<Set<number>>(() => {
  const doc = layout.value;
  const seen = new Set<number>();
  const clashes = new Set<number>();
  if (!doc) return clashes;
  const claim = (ch: number) => (seen.has(ch) ? clashes.add(ch) : seen.add(ch));
  for (const a of doc.drive.actuators) claim(a.channel);
  for (const m of doc.mechanisms) for (const a of m.actuators) claim(a.channel);
  return clashes;
});

function allNames(doc: LayoutDoc): Set<string> {
  const names = new Set(doc.drive.actuators.map((a) => a.name));
  for (const m of doc.mechanisms) {
    names.add(m.name);
    for (const a of m.actuators) names.add(a.name);
  }
  return names;
}

function newActuator(doc: LayoutDoc, stem: string): ActuatorSpec {
  return {
    name: uniqueName(allNames(doc), stem),
    kind: "esc",
    channel: freeChannel(doc),
    inverted: false,
  };
}

export function addDriveActuator(): void {
  edit((doc) => {
    doc.drive.actuators.push(newActuator(doc, "motor"));
  });
}

export function removeDriveActuator(name: string): void {
  edit((doc) => {
    doc.drive.actuators = doc.drive.actuators.filter((a) => a.name !== name);
    // Drop it from every role too, or the layout refuses with a message about a
    // role naming an actuator that no longer exists.
    for (const role of ["left", "right", "throttle"] as const) {
      doc.drive.roles[role] = doc.drive.roles[role].filter((n) => n !== name);
    }
    if (doc.drive.roles.steer === name) doc.drive.roles.steer = "";
  });
}

export function setActuatorField<K extends keyof ActuatorSpec>(
  mech: string | null,
  name: string,
  key: K,
  value: ActuatorSpec[K],
): void {
  edit((doc) => {
    const list = mech
      ? doc.mechanisms.find((m) => m.name === mech)?.actuators
      : doc.drive.actuators;
    const actuator = list?.find((a) => a.name === name);
    if (!actuator) return;
    const previous = actuator.name;
    actuator[key] = value;
    // Renaming has to follow through into the roles and presets that point at
    // it, or saving fails on a dangling reference the operator never typed.
    if (key === "name" && typeof value === "string" && value !== previous) {
      if (!mech) {
        for (const role of ["left", "right", "throttle"] as const) {
          doc.drive.roles[role] = doc.drive.roles[role].map((n) =>
            n === previous ? value : n
          );
        }
        if (doc.drive.roles.steer === previous) doc.drive.roles.steer = value;
      } else {
        const owner = doc.mechanisms.find((m) => m.name === mech);
        for (const preset of Object.values(owner?.presets ?? {})) {
          if (previous in preset) {
            preset[value] = preset[previous];
            delete preset[previous];
          }
        }
      }
    }
  });
}

// --- mechanisms --------------------------------------------------------------

export function addMechanism(kind: "power" | "pulse"): void {
  edit((doc) => {
    const name = uniqueName(allNames(doc), kind === "pulse" ? "launcher" : "intake");
    const mech: MechanismSpec = {
      name,
      kind,
      enabled: true,
      actuators: [newActuator(doc, `${name}_motor`)],
    };
    if (kind === "power") {
      mech.presets = { on: {}, off: {} };
    }
    doc.mechanisms.push(mech);
  });
}

export function removeMechanism(name: string): void {
  edit((doc) => {
    doc.mechanisms = doc.mechanisms.filter((m) => m.name !== name);
  });
}

export function setMechanismField<K extends keyof MechanismSpec>(
  name: string,
  key: K,
  value: MechanismSpec[K],
): void {
  edit((doc) => {
    const mech = doc.mechanisms.find((m) => m.name === name);
    if (mech) mech[key] = value;
  });
}

export function addMechanismActuator(name: string): void {
  edit((doc) => {
    const mech = doc.mechanisms.find((m) => m.name === name);
    if (mech) mech.actuators.push(newActuator(doc, `${name}_motor`));
  });
}

export function removeMechanismActuator(name: string, actuator: string): void {
  edit((doc) => {
    const mech = doc.mechanisms.find((m) => m.name === name);
    if (!mech) return;
    mech.actuators = mech.actuators.filter((a) => a.name !== actuator);
    for (const preset of Object.values(mech.presets ?? {})) delete preset[actuator];
  });
}

export function setPreset(
  mech: string,
  preset: string,
  actuator: string,
  value: number,
): void {
  edit((doc) => {
    const owner = doc.mechanisms.find((m) => m.name === mech);
    if (!owner) return;
    owner.presets = owner.presets ?? {};
    owner.presets[preset] = { ...(owner.presets[preset] ?? {}), [actuator]: value };
  });
}

export function addPreset(mech: string, name: string): void {
  edit((doc) => {
    const owner = doc.mechanisms.find((m) => m.name === mech);
    if (!owner) return;
    owner.presets = { ...(owner.presets ?? {}), [name]: {} };
  });
}

export function removePreset(mech: string, name: string): void {
  edit((doc) => {
    const owner = doc.mechanisms.find((m) => m.name === mech);
    if (owner?.presets) delete owner.presets[name];
  });
}

// --- jog ---------------------------------------------------------------------

/** Move one mechanism briefly. The robot refuses this unless it is in teleop
 *  with no e-stop latched, and expires it after 0.4 s — so the control repeats
 *  while held and simply stops if the link drops. */
export function jog(mech: string, power: number, actuator?: string): void {
  const rid = targetRobot.value;
  if (!rid) return;
  send({ action: "jog", robot_id: rid, mech, actuator, power });
}
