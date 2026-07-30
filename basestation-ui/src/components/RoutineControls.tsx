// The routines this rover carries, as buttons.
//
// A routine is a mode the operator wrote themselves — "collect the near
// bucket", "shoot three and come home" — and the driving view treated all of
// them as one button labelled "Routine", which ran whichever one the robot
// happened to have selected. That is the one control on this panel where the
// operator could not tell what pressing it would do, so it is the one that had
// to become a list.
//
// The list is read off the ROBOT's saved document, not the editor's draft: a
// button here starts something that moves a machine, and it must run what is
// actually loaded rather than what somebody has half-edited on another tab.
//
// Fetching it here is also what makes routines sayable. The vocabulary the
// recogniser matches against is built from the documents the base station has
// (basestation/command/vocabulary.py::build), and before this the only thing
// that ever asked for them was the editor — so "run collect" worked only if
// somebody had opened the Routines tab first.

import { robotDocuments, selected, selectedRobot } from "../net/ws.ts";
import { requestRoutines, routinesOn, startRoutine, stopRoutine } from "../state/routines.ts";
import { useRadioFetch } from "../state/fetch.ts";
import { configTarget, tab } from "../state/settings.ts";
import { showView } from "../state/view.ts";

export function RoutineControls() {
  const rid = selected.value;
  const robot = selectedRobot.value;
  const have = !!(rid && robotDocuments.value[rid]?.routines_rev);
  const fetch = useRadioFetch(
    rid && `${rid}:routines`,
    have,
    () => rid && requestRoutines(rid),
  );

  const list = routinesOn(rid);
  // Which one is running, and how far in. `routine` rides the hot frame, so
  // the live state name updates at telemetry rate — it is the only evidence on
  // this panel that an autonomous run is progressing rather than stuck.
  const running = robot?.mode === "routine" ? robot.routine : null;
  const runningId = running?.id ?? null;

  function open() {
    // Point the editor at the rover being driven. Without this the tab opens on
    // whatever was last configured, which is rarely the one whose empty list
    // the operator just tapped.
    if (rid) configTarget.value = rid;
    tab.value = "routines";
    showView("settings");
  }

  return (
    <div class="routines-panel">
      <div class="section-title">
        <span class="eyebrow">Routines</span>
        <button type="button" class="btn ghost small" onClick={open}>
          {list.length ? "Edit" : "Build one"}
        </button>
      </div>

      {list.length > 0 && (
        <div class="routine-list">
          {list.map((r) => {
            const live = runningId === r.id;
            return (
              <button
                key={r.id}
                type="button"
                class={`btn routine-btn${live ? " active" : ""}`}
                disabled={!rid}
                onClick={() => rid && startRoutine(rid, r.id)}
                title={live ? "Running — press to restart" : `Run ${r.name || r.id}`}
              >
                <span class="routine-name">{r.name || r.id}</span>
                {live && (
                  <span class="routine-state">
                    {running?.done ? (running.why ?? "done") : (running?.state ?? "…")}
                  </span>
                )}
              </button>
            );
          })}
        </div>
      )}

      {/* One button, two truthful names. `stopRoutine` halts the machine AND
          puts the rover back in teleop, so it is still the control you want
          once a routine has finished — but calling it "Stop" then would be
          offering to stop something that already stopped, and the operator
          would be left wondering what is still running. */}
      {running && (
        <button
          type="button"
          class="btn ghost danger routine-stop"
          onClick={() => rid && stopRoutine(rid)}
        >
          {running.done ? "Back to teleop" : "Stop routine"}
        </button>
      )}

      {list.length === 0 && (
        <>
          <p class="hint">
            {!rid
              ? "No rover selected."
              : fetch.pending
              ? `Asking ${rid} for its routines…`
              : fetch.stalled
              // Distinguished from "none programmed", because they look
              // identical on screen and mean opposite things: one is a rover
              // that answered, the other is a rover that never spoke.
              ? `No answer from ${rid} — its routines never arrived.`
              : "None yet. Build one in Routines and it appears here, and " +
                "answers to its name out loud."}
          </p>
          {rid && fetch.stalled && (
            <button type="button" class="btn ghost small" onClick={fetch.retry}>
              Ask again
            </button>
          )}
        </>
      )}
    </div>
  );
}
