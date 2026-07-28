// Routines tab: draw a state machine and watch the robot run it.
//
// Draft-then-Save like the Hardware tab, with one difference the operator can
// feel: routines take effect the moment they land, because they are data the
// engine reads rather than hardware a constructor owns.
//
// The live state is highlighted from telemetry as it runs, which is what turns
// this from an editor into something you can debug a match with.

import { useEffect, useState } from "preact/hooks";
import { conn, robotDocuments, robots, selectedRobot } from "../../net/ws.ts";
import { refreshLayout } from "../../state/hardware.ts";
import { configTarget, targetRobot } from "../../state/settings.ts";
import {
  addRoutine,
  current,
  discardRoutines,
  duplicateRoutine,
  editing,
  fireEvent,
  liveState,
  problems,
  refreshRoutines,
  removeRoutine,
  routines,
  routinesAccepted,
  routinesDirty,
  routinesResult,
  runRoutine,
  saveRoutines,
  selectedEdge,
  selectedState,
  setRoutineField,
  stopRoutine,
} from "../../state/routines.ts";
import { Inspector } from "./Inspector.tsx";
import { RoutineCanvas } from "./RoutineCanvas.tsx";

function RunBar() {
  const robot = selectedRobot.value;
  const running = robot?.mode === "routine";
  const status = robot?.routine;
  const [event, setEvent] = useState("go");

  return (
    <div class="run-bar">
      {running
        ? (
          <button type="button" class="btn small danger" onClick={stopRoutine}>
            Stop
          </button>
        )
        : (
          <button
            type="button"
            class="btn small primary"
            disabled={!current.value}
            onClick={runRoutine}
          >
            Run
          </button>
        )}

      <span class="run-status">
        {!running
          ? "Not running."
          : status?.state
          ? <>In <b>{status.state}</b>{status.drive ? ` · ${status.drive}` : ""}</>
          : `Finished${status?.why ? ` — ${status.why}` : ""}.`}
      </span>

      <div class="run-event">
        <input
          class="field-input tiny"
          type="text"
          value={event}
          onInput={(e) => setEvent((e.target as HTMLInputElement).value)}
        />
        <button
          type="button"
          class="btn ghost small"
          disabled={!running || !event.trim()}
          onClick={() => fireEvent(event.trim())}
          title="Satisfies a 'when I press a button' transition in the state the robot is in right now."
        >
          Send event
        </button>
      </div>
    </div>
  );
}

export function RoutinesPage() {
  const rid = targetRobot.value;
  const documents = rid ? robotDocuments.value[rid] : undefined;
  const routine = current.value;
  const result = routinesResult.value;
  const found = problems.value;

  useEffect(() => {
    if (rid && !documents?.routines_rev && conn.value === "live") refreshRoutines();
  }, [rid, conn.value]);

  // The layout too, even though this tab doesn't edit it: the action pickers
  // offer this robot's mechanisms and their presets by name, and without it
  // every "which mechanism?" menu is empty for no visible reason.
  useEffect(() => {
    if (rid && !documents?.layout_rev && conn.value === "live") refreshLayout();
  }, [rid, conn.value]);

  useEffect(() => {
    if (result?.ok && routinesDirty.value) routinesAccepted();
  }, [documents?.routines_rev]);

  // Switching routine or robot must drop the selection: a node id from the old
  // one would leave the inspector showing a state that isn't on the canvas.
  useEffect(() => {
    selectedState.value = null;
    selectedEdge.value = null;
  }, [routine?.id, rid]);

  if (!rid) {
    return <p class="hint pad">No robot selected — pick one on the driving view first.</p>;
  }

  const all = routines.value.routines;
  const live = liveState.value;
  const byState = new Map<string, string[]>();
  const general: string[] = [];
  for (const problem of found) {
    if (!problem.state) general.push(problem.message);
    else byState.set(problem.state, [...(byState.get(problem.state) ?? []), problem.message]);
  }

  return (
    <>
      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Robot</span>
          <select
            class="field-select"
            value={rid}
            onChange={(e) => configTarget.value = (e.target as HTMLSelectElement).value}
          >
            {robots.value.map((r) => (
              <option key={r.robot_id} value={r.robot_id}>{r.robot_id}</option>
            ))}
          </select>
        </div>
        <div class="settings-bar-group">
          {routinesDirty.value && (
            <button type="button" class="btn ghost small" onClick={discardRoutines}>
              Discard
            </button>
          )}
          <button type="button" class="btn ghost small" onClick={refreshRoutines}>
            Refresh
          </button>
          <button
            type="button"
            class="btn small primary"
            disabled={!routinesDirty.value}
            onClick={saveRoutines}
          >
            {routinesDirty.value ? "Save to robot" : "Saved"}
          </button>
        </div>
      </div>

      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Routine</span>
          <select
            class="field-select"
            value={routine?.id ?? ""}
            onChange={(e) => editing.value = (e.target as HTMLSelectElement).value}
          >
            {all.length === 0 && <option value="">— none —</option>}
            {all.map((r) => (
              <option key={r.id} value={r.id}>{r.name || r.id}</option>
            ))}
          </select>
        </div>
        <div class="settings-bar-group">
          <button type="button" class="btn ghost small" onClick={addRoutine}>New</button>
          <button
            type="button"
            class="btn ghost small"
            disabled={!routine}
            onClick={duplicateRoutine}
          >
            Duplicate
          </button>
          <button
            type="button"
            class="btn ghost small danger"
            disabled={!routine}
            onClick={() => routine && removeRoutine(routine.id)}
          >
            Delete
          </button>
        </div>
      </div>

      {result && !result.ok && (
        <p class="banner error">
          The robot refused these routines:<br />{result.errors.join(" · ")}
        </p>
      )}
      {general.length > 0 && <p class="banner warn">{general.join(" · ")}</p>}

      {!routine
        ? (
          <p class="hint pad">
            {documents?.routines_rev
              ? "No routines yet. “New” gives you a two-state skeleton to build on."
              : `Fetching ${rid}'s routines…`}
          </p>
        )
        : (
          <>
            <RunBar />

            <section class="group open">
              <div class="group-body routine-meta">
                <label class="arg">
                  <span>name</span>
                  <input
                    class="field-input"
                    type="text"
                    value={routine.name ?? ""}
                    onInput={(e) =>
                      setRoutineField("name", (e.target as HTMLInputElement).value)}
                  />
                </label>
                <label class="arg">
                  <span>starts at</span>
                  <select
                    class="field-select tiny"
                    value={routine.start}
                    onChange={(e) =>
                      setRoutineField("start", (e.target as HTMLSelectElement).value)}
                  >
                    {(routine?.states ?? []).map((s) => (
                      <option key={s.id} value={s.id}>{s.id}</option>
                    ))}
                  </select>
                </label>
                <label class="arg">
                  <span>when it ends</span>
                  <select
                    class="field-select tiny"
                    value={routine.on_end ?? "stop"}
                    onChange={(e) =>
                      setRoutineField(
                        "on_end",
                        (e.target as HTMLSelectElement).value as "stop" | "restart",
                      )}
                  >
                    <option value="stop">stop</option>
                    <option value="restart">start over</option>
                  </select>
                </label>
                <label class="arg">
                  <span>on e-stop</span>
                  <select
                    class="field-select tiny"
                    value={routine.on_estop ?? "abort"}
                    onChange={(e) =>
                      setRoutineField(
                        "on_estop",
                        (e.target as HTMLSelectElement).value as "abort" | "hold",
                      )}
                    title="Abort resets to the start — clearing an e-stop should not resume a sequence whose idea of where the robot is went stale."
                  >
                    <option value="abort">abort</option>
                    <option value="hold">hold</option>
                  </select>
                </label>
                <label class="arg">
                  <span>time limit (s)</span>
                  <input
                    class="field-input tiny"
                    type="number"
                    min={0}
                    max={3600}
                    step={1}
                    value={routine.timeout ?? 0}
                    onInput={(e) =>
                      setRoutineField("timeout",
                        Number((e.target as HTMLInputElement).value))}
                    title="0 = none. Every state still has its own limit."
                  />
                </label>
              </div>
            </section>

            <div class="routine-body">
              <div class="routine-canvas-pane">
                <RoutineCanvas
                  routine={routine}
                  live={live}
                  problemStates={new Set(byState.keys())}
                />
              </div>

              <Inspector routine={routine} live={live} problems={byState} />
            </div>
          </>
        )}
    </>
  );
}
