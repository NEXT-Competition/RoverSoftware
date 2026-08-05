// Code tab: write Python, send it to a rover, watch it run.
//
// The sibling of the Routines tab, not its replacement. A graph is the better
// shape for a sequence of states you want to see at a glance; code is the
// better shape for anything with arithmetic in it — a loop that scales
// throttle by measured range is a program, and drawing it as one is a picture
// of a program rather than a program.
//
// Draft-then-Save like Routines and Hardware, with the same rule: nothing
// reaches the rover until Save, and the rover keeps running what it has until
// then. The difference an operator feels is the verdict — the robot COMPILES
// every script when it lands, so a missing colon comes back as "line 12" and
// the editor puts a marker on line 12, rather than saving cleanly and dying at
// the field the first time somebody presses Run.

import { useEffect, useRef, useState } from "preact/hooks";
import { robotDocuments, robots, selectedRobot } from "../../net/ws.ts";
import { useRadioFetch } from "../../state/fetch.ts";
import { refreshLayout } from "../../state/hardware.ts";
import { configTarget, targetRobot } from "../../state/settings.ts";
import {
  addScript,
  clearConsole,
  console_,
  current,
  discardScripts,
  duplicateScript,
  editing,
  errorLine,
  exportScript,
  exportScripts,
  importScripts,
  liveScript,
  refreshScripts,
  removeScript,
  runScript,
  saveScripts,
  scripts,
  scriptsAccepted,
  scriptsDirty,
  scriptsResult,
  setScriptCode,
  setScriptName,
  stopScript,
} from "../../state/scripts.ts";
import { TEMPLATES } from "../../scripts/templates.ts";
import { Waiting } from "../settings/Waiting.tsx";
import { ApiReference } from "./ApiReference.tsx";
import { CodeEditor } from "./CodeEditor.tsx";
import { Console } from "./Console.tsx";

/** Run / Stop, and what the rover is doing about it.
 *
 * Same shape as the Routines tab's bar, deliberately: the two authoring
 * surfaces should not need to be learned twice. */
function RunBar({ onRun }: { onRun: () => void }) {
  const robot = selectedRobot.value;
  const live = liveScript.value;
  const running = !!live?.run;
  const dirty = scriptsDirty.value;

  return (
    <div class="run-bar">
      {running
        ? (
          <button type="button" class="btn small danger" onClick={() => stopScript()}>
            Stop
          </button>
        )
        : (
          <button
            type="button"
            class="btn small primary"
            disabled={!current.value || dirty}
            title={dirty
              ? "Save it to the robot first — Run starts what the ROVER is carrying"
              : "Run on the rover (⌘/Ctrl + Enter)"}
            onClick={onRun}
          >
            Run
          </button>
        )}

      <span class="run-status">
        {!robot
          ? "No rover selected."
          : running
          ? <>Running <b>{live?.id}</b> · {live?.t?.toFixed(1)}s{live?.drive
            ? ` · driving with ${live.drive}`
            : ""}</>
          : live?.why
          ? <>Last run: {live.why}.</>
          : "Not running."}
      </span>

      {/* The one thing that is genuinely easy to get wrong: pressing Run on a
          draft that has not been sent runs the OLD code, and the operator
          watches the bug they just fixed happen again. Saying so beats a
          Run button that quietly does the wrong thing. */}
      {dirty && (
        <span class="run-hint">Unsaved — Run uses what the rover already has.</span>
      )}
    </div>
  );
}

function TemplatePicker() {
  return (
    <div class="template-picker">
      <p class="hint pad">
        Nothing written yet. Start from one of these — every one runs as-is on a
        bare chassis, checking for a camera or a mechanism before reaching for
        one.
      </p>
      <div class="template-grid">
        {TEMPLATES.map((template) => (
          <button
            key={template.key}
            type="button"
            class="template-card"
            onClick={() => addScript(template.name, template.code)}
          >
            <span class="template-title">{template.name}</span>
            <span class="template-blurb">{template.blurb}</span>
          </button>
        ))}
        <button
          type="button"
          class="template-card blank"
          onClick={() => addScript("New script", "# rover.forward(0.3, seconds=1)\n")}
        >
          <span class="template-title">Empty</span>
          <span class="template-blurb">Start from nothing.</span>
        </button>
      </div>
    </div>
  );
}

export function CodePage() {
  const rid = targetRobot.value;
  const documents = rid ? robotDocuments.value[rid] : undefined;
  const script = current.value;
  const result = scriptsResult.value;
  const failed = errorLine.value;
  const [importError, setImportError] = useState<string | null>(null);
  const editorRef = useRef<HTMLDivElement>(null);

  const fetch = useRadioFetch(
    rid && `${rid}:scripts`,
    !!documents?.scripts_rev,
    refreshScripts,
  );

  // The layout too, even though this tab does not edit it: `rover.mechanisms`
  // is the operator's own names, and a reference panel that cannot say what
  // this rover has is missing the half a script actually needs.
  useRadioFetch(rid && `${rid}:layout`, !!documents?.layout_rev, refreshLayout);

  useEffect(() => {
    if (result?.ok && scriptsDirty.value) scriptsAccepted();
  }, [documents?.scripts_rev]);

  async function onImport(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    setImportError(importScripts(file.name, await file.text()));
    // Clear it so picking the SAME file again still fires a change event —
    // otherwise a failed import cannot be retried without switching files.
    input.value = "";
  }

  /** Insert a signature from the reference panel at the caret. */
  function insert(snippet: string) {
    const area = editorRef.current?.querySelector("textarea");
    if (!area || !script) return;
    const at = area.selectionStart;
    const end = area.selectionEnd;
    setScriptCode(script.id, script.code.slice(0, at) + snippet + script.code.slice(end));
    // Put the caret after what we inserted, once the re-render has landed.
    requestAnimationFrame(() => {
      area.focus();
      area.selectionStart = area.selectionEnd = at + snippet.length;
    });
  }

  if (!rid) {
    return <p class="hint pad">No robot selected — pick one on the driving view first.</p>;
  }

  const all = scripts.value.scripts;

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
          {scriptsDirty.value && (
            <button type="button" class="btn ghost small" onClick={discardScripts}>
              Discard
            </button>
          )}
          <button type="button" class="btn ghost small" onClick={refreshScripts}>
            Refresh
          </button>
          <button
            type="button"
            class="btn small primary"
            disabled={!scriptsDirty.value}
            onClick={saveScripts}
          >
            {scriptsDirty.value ? "Save to robot" : "Saved"}
          </button>
        </div>
      </div>

      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Script</span>
          <select
            class="field-select"
            value={script?.id ?? ""}
            onChange={(e) => editing.value = (e.target as HTMLSelectElement).value}
          >
            {all.length === 0 && <option value="">— none —</option>}
            {all.map((s) => (
              <option key={s.id} value={s.id}>{s.name || s.id}</option>
            ))}
          </select>
        </div>
        <div class="settings-bar-group">
          <button
            type="button"
            class="btn ghost small"
            onClick={() => addScript("New script", "# rover.forward(0.3, seconds=1)\n")}
          >
            New
          </button>
          <button
            type="button"
            class="btn ghost small"
            disabled={!script}
            onClick={duplicateScript}
          >
            Duplicate
          </button>
          <button
            type="button"
            class="btn ghost small danger"
            disabled={!script}
            onClick={() => script && removeScript(script.id)}
          >
            Delete
          </button>
        </div>
        {/* Saving puts scripts on the ROBOT, which is what makes them survive a
            power cycle. These are the other half a competition needs: a file to
            put in git, hand to another team, or restore onto a fresh SD card.
            A single script goes out as .py so it opens in any editor. */}
        <div class="settings-bar-group">
          <button
            type="button"
            class="btn ghost small"
            disabled={!script}
            onClick={exportScript}
          >
            Export .py
          </button>
          <button
            type="button"
            class="btn ghost small"
            disabled={all.length === 0}
            onClick={exportScripts}
          >
            Export all
          </button>
          <label class="btn ghost small file-btn">
            Import
            <input type="file" accept=".py,.json,text/x-python,application/json" onChange={onImport} />
          </label>
        </div>
      </div>

      {importError && <p class="banner error">{importError}</p>}
      {result && !result.ok && (
        <p class="banner error">
          The robot refused this:<br />{result.errors.join(" · ")}
        </p>
      )}
      {result?.ok && (result.warnings?.length ?? 0) > 0 && (
        <p class="banner warn">{result.warnings.join(" · ")}</p>
      )}

      {!script
        ? (
          documents?.scripts_rev
            ? <TemplatePicker />
            : <Waiting what="scripts" robot={rid} fetch={fetch} />
        )
        : (
          <>
            <RunBar onRun={runScript} />

            <div class="code-body">
              <div class="code-main">
                <label class="arg code-name">
                  <span>name</span>
                  <input
                    class="field-input"
                    type="text"
                    value={script.name ?? ""}
                    onInput={(e) =>
                      setScriptName(script.id, (e.target as HTMLInputElement).value)}
                  />
                  <span class="code-id tape">{script.id}</span>
                </label>

                <div ref={editorRef}>
                  <CodeEditor
                    code={script.code}
                    onChange={(code) => setScriptCode(script.id, code)}
                    onRun={runScript}
                    errorLine={failed?.id === script.id ? failed.line : null}
                  />
                </div>

                <Console
                  console={console_.value}
                  status={liveScript.value}
                  onClear={clearConsole}
                />
              </div>

              <ApiReference onInsert={insert} />
            </div>
          </>
        )}
    </>
  );
}
