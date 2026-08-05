// Editing state for the Code tab.
//
// Same draft-then-save shape as state/routines.ts, for the same reason: a
// half-saved program is meaningless. Scripts behave like routines and unlike a
// layout in the way the operator feels — they take effect the moment they land,
// because they are text the runner compiles rather than hardware a constructor
// owns.
//
// Validation is NOT duplicated here. The routine editor mirrors a chunk of the
// robot's schema so it can mark a broken graph inline; a Python parser in the
// browser would be a second, subtly different idea of what compiles, and the
// robot already answers the only question worth asking ("does this compile?")
// with a line number. So the editor sends and shows the verdict.

import { computed, signal } from "@preact/signals";
import type { ScriptDoc, ScriptSpec } from "../net/types.ts";
import { robotDocuments, scriptConsole, selectedRobot, send } from "../net/ws.ts";
import { targetRobot } from "./settings.ts";

const draft = signal<ScriptDoc | null>(null);
const draftOwner = signal<string | null>(null);

/** Which script the editor is showing. */
export const editing = signal<string | null>(null);

const EMPTY: ScriptDoc = { version: 1, scripts: [] };

export const serverScripts = computed<ScriptDoc | null>(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.scripts ?? null : null;
});

export const scriptsResult = computed(() => {
  const rid = targetRobot.value;
  return rid ? robotDocuments.value[rid]?.scripts_result ?? null : null;
});

export const scripts = computed<ScriptDoc>(() => {
  if (draft.value && draftOwner.value === targetRobot.value) return draft.value;
  return serverScripts.value ?? EMPTY;
});

export const scriptsDirty = computed(() =>
  draft.value != null && draftOwner.value === targetRobot.value
);

export const current = computed<ScriptSpec | null>(() => {
  const all = scripts.value.scripts;
  return all.find((s) => s.id === editing.value) ?? all[0] ?? null;
});

/** Whether the robot is running a script right now, off the hot frame. */
export const liveScript = computed(() => {
  const robot = selectedRobot.value;
  if (!robot || robot.mode !== "script") return null;
  return robot.script ?? null;
});

/** This robot's console output. Empty until a script prints something. */
export const console_ = computed(() => {
  const rid = targetRobot.value;
  return rid ? scriptConsole.value[rid] ?? null : null;
});

function edit(mutate: (doc: ScriptDoc) => void): void {
  const next: ScriptDoc = JSON.parse(JSON.stringify(scripts.value));
  mutate(next);
  draftOwner.value = targetRobot.value;
  draft.value = next;
}

export function discardScripts(): void {
  draft.value = null;
  draftOwner.value = null;
}

export function refreshScripts(): void {
  const rid = targetRobot.value;
  if (rid) requestScripts(rid);
}

/** Ask one named robot for its scripts.
 *
 * Separate from `refreshScripts` for the reason `requestRoutines` is: the
 * driving view wants the scripts of the rover being DRIVEN, while the editor
 * wants the one being edited, and those are allowed to differ. */
export function requestScripts(robotId: string): void {
  send({ action: "get_scripts", robot_id: robotId });
}

/** The scripts loaded on a robot, as the driving view lists them. Read off the
 *  fleet rather than the editor's draft: a button here starts something that
 *  moves a machine, so it must run what the ROBOT is carrying. */
export function scriptsOn(robotId: string | null): ScriptSpec[] {
  if (!robotId) return [];
  return robotDocuments.value[robotId]?.scripts?.scripts ?? [];
}

export function saveScripts(): void {
  const rid = targetRobot.value;
  if (!rid) return;
  send({ action: "set_scripts", robot_id: rid, doc: scripts.value });
}

export function scriptsAccepted(): void {
  discardScripts();
}

// --- running -----------------------------------------------------------------

/**
 * Run one named script on one named robot.
 *
 * Select first, then switch modes — the same two messages `startRoutine` sends,
 * and for the same reason. A robot already in script mode starts on the select
 * itself, so pressing the running script restarts it; one that isn't starts
 * when the mode change activates the controller.
 */
export function startScript(robotId: string, id: string): void {
  send({ action: "clear_console", robot_id: robotId });
  send({ action: "select_script", robot_id: robotId, id });
  send({ action: "mode", robot_id: robotId, mode: "script" });
}

export function runScript(): void {
  const rid = targetRobot.value;
  const id = current.value?.id;
  if (rid && id) startScript(rid, id);
}

/** Stop the script AND put the rover back in teleop.
 *
 * Both, because stopping the script alone leaves the rover in a mode whose only
 * behaviour is to hold still — which looks identical to a rover that has
 * stopped responding. */
export function stopScript(robotId?: string): void {
  const rid = robotId ?? targetRobot.value;
  if (!rid) return;
  send({ action: "script_cmd", robot_id: rid, cmd: "stop" });
  send({ action: "mode", robot_id: rid, mode: "teleop" });
}

export function clearConsole(): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "clear_console", robot_id: rid });
}

// --- editing -----------------------------------------------------------------

function uniqueId(taken: Set<string>, stem: string): string {
  if (!taken.has(stem)) return stem;
  for (let n = 2; n < 100; n++) if (!taken.has(`${stem}${n}`)) return `${stem}${n}`;
  return `${stem}_x`;
}

export function addScript(name: string, code: string): void {
  edit((doc) => {
    const id = uniqueId(
      new Set(doc.scripts.map((s) => s.id)),
      // An id the robot will accept, derived from the name so it is
      // recognisable in a log line: lower case, and nothing but letters,
      // digits, underscore and hyphen (robot/script/schema.py).
      (name.toLowerCase().replace(/[^a-z0-9_-]+/g, "_").replace(/^[^a-z]+/, "")
        || "script").slice(0, 24),
    );
    doc.scripts.push({ id, name, code });
    editing.value = id;
  });
}

export function duplicateScript(): void {
  const source = current.value;
  if (!source) return;
  edit((doc) => {
    const copy: ScriptSpec = JSON.parse(JSON.stringify(source));
    copy.id = uniqueId(new Set(doc.scripts.map((s) => s.id)), `${source.id}_copy`);
    copy.name = `${source.name ?? source.id} (copy)`;
    doc.scripts.push(copy);
    editing.value = copy.id;
  });
}

export function removeScript(id: string): void {
  edit((doc) => {
    doc.scripts = doc.scripts.filter((s) => s.id !== id);
    if (editing.value === id) editing.value = doc.scripts[0]?.id ?? null;
  });
}

export function setScriptCode(id: string, code: string): void {
  edit((doc) => {
    const script = doc.scripts.find((s) => s.id === id);
    if (script) script.code = code;
  });
}

export function setScriptName(id: string, name: string): void {
  edit((doc) => {
    const script = doc.scripts.find((s) => s.id === id);
    if (script) script.name = name;
  });
}

// --- what the robot said about it --------------------------------------------

/**
 * The line number the robot refused a script on, if it did.
 *
 * The robot's message is `script 'drive': line 12: invalid syntax`. Pulling the
 * number back out lets the editor put a marker on that line, which is the whole
 * reason validation compiles on the robot rather than just storing the text.
 * Deliberately tolerant: an unparseable message simply means no marker, not a
 * broken editor.
 */
export const errorLine = computed<{ id: string; line: number; message: string } | null>(
  () => {
    const result = scriptsResult.value;
    if (!result || result.ok) return null;
    for (const message of result.errors ?? []) {
      const match = /script '([^']+)': line (\d+): (.*)/.exec(message);
      if (match) {
        return { id: match[1], line: Number(match[2]), message: match[3] };
      }
    }
    return null;
  },
);

// --- import / export ---------------------------------------------------------
//
// Saving puts scripts on the ROBOT, which is what makes them survive a power
// cycle. These are the other half a competition needs: a file to put in git,
// hand to another team, or restore onto a fresh SD card. A single script goes
// out as a .py so it opens in any editor; the whole set goes out as the .json
// the robot actually stores.

function download(filename: string, text: string, mime: string): void {
  const url = URL.createObjectURL(new Blob([text], { type: mime }));
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

export function exportScript(): void {
  const script = current.value;
  if (!script) return;
  download(`${script.id}.py`, script.code, "text/x-python");
}

export function exportScripts(): void {
  download("scripts.json", JSON.stringify(scripts.value, null, 2),
    "application/json");
}

/** Read a picked file into the draft. Returns an error string, or null.
 *
 * A `.py` file becomes one new script; a `.json` file replaces the set. The
 * robot stays the authority on what is legal — this only rejects a file that
 * isn't a script document at all, because that would corrupt the draft rather
 * than be refused on save.
 */
export function importScripts(filename: string, text: string): string | null {
  if (!filename.toLowerCase().endsWith(".json")) {
    addScript(filename.replace(/\.py$/i, "") || "Imported", text);
    return null;
  }
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch (e) {
    return `That file is not JSON: ${e instanceof Error ? e.message : e}`;
  }
  const doc = parsed as ScriptDoc;
  if (!doc || typeof doc !== "object" || !Array.isArray(doc.scripts)) {
    return "That file is not a script document — it has no 'scripts' list.";
  }
  edit((into) => {
    into.version = 1;
    into.scripts = doc.scripts
      .filter((s) => s && typeof s.id === "string" && typeof s.code === "string")
      .map((s) => ({ id: s.id, name: s.name ?? s.id, code: s.code }));
    editing.value = into.scripts[0]?.id ?? null;
  });
  return null;
}
