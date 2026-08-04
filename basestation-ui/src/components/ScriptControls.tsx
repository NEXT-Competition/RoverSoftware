// The scripts this rover carries, as buttons.
//
// The sibling of RoutineControls, and it exists for the same reason: a program
// somebody wrote is a mode, and a mode you can only start by opening an editor
// is a mode nobody starts mid-match. Same rules, too — the list is read off the
// ROBOT's saved document rather than the editor's draft, because a button here
// starts something that moves a machine and it must run what is actually
// loaded, not what a teammate has half-edited on another tab.
//
// Renders nothing at all on a fleet with no scripts. The rail is already dense,
// and a permanently empty panel headed "Scripts" is a row of pixels that only
// ever says "no". The Routines panel earns its empty state by being the place
// you go to build the first one; this one is reachable from there and from the
// gear, which is enough.

import { robotDocuments, selected, selectedRobot } from "../net/ws.ts";
import { requestScripts, scriptsOn, startScript, stopScript } from "../state/scripts.ts";
import { useRadioFetch } from "../state/fetch.ts";
import { configTarget, tab } from "../state/settings.ts";
import { showView } from "../state/view.ts";

export function ScriptControls() {
  const rid = selected.value;
  const robot = selectedRobot.value;
  const have = !!(rid && robotDocuments.value[rid]?.scripts_rev);

  // Asking here is also what makes a script sayable and startable without ever
  // opening the tab — the same job RoutineControls does for routines.
  useRadioFetch(rid && `${rid}:scripts`, have, () => rid && requestScripts(rid));

  const list = scriptsOn(rid);
  const running = robot?.mode === "script" ? robot.script : null;
  const runningId = running?.run ? running.id : null;

  function open() {
    if (rid) configTarget.value = rid;
    tab.value = "code";
    showView("settings");
  }

  // Nothing loaded and nothing running: stay out of the rail entirely.
  if (list.length === 0 && !running) return null;

  return (
    <div class="routines-panel">
      <div class="section-title">
        <span class="eyebrow">Scripts</span>
        <button type="button" class="btn ghost small" onClick={open}>Edit</button>
      </div>

      <div class="routine-list">
        {list.map((script) => {
          const live = runningId === script.id;
          return (
            <button
              key={script.id}
              type="button"
              class={`btn routine-btn${live ? " active" : ""}`}
              disabled={!rid}
              onClick={() => rid && startScript(rid, script.id)}
              title={live
                ? "Running — press to restart"
                : `Run ${script.name || script.id}`}
            >
              <span class="routine-name">{script.name || script.id}</span>
              {live && (
                <span class="routine-state">
                  {running?.t !== undefined ? `${running.t.toFixed(0)}s` : "…"}
                  {running?.drive ? ` · ${running.drive}` : ""}
                </span>
              )}
            </button>
          );
        })}
      </div>

      {/* An error is the one thing about a finished run that has to reach the
          driving view. The console it printed rides WiFi and may never arrive;
          this rides the same hot frame as the telemetry beside it. */}
      {running && !running.run && running.err && (
        <p class="hint script-error">{running.err}</p>
      )}

      {running && (
        <button
          type="button"
          class="btn ghost danger routine-stop"
          onClick={() => rid && stopScript(rid)}
        >
          {running.run ? "Stop script" : "Back to teleop"}
        </button>
      )}
    </div>
  );
}
