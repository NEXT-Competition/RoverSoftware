// Controller tab: remap the gamepad, and see what it's actually reporting.
//
// The premise is that an axis or button index is not something anyone knows.
// The same pad enumerates differently across macOS, Linux, USB and Bluetooth,
// so the honest interface is "press the control you want" — which needs the
// raw, unmapped sample the server publishes (controller_input.py::state).
// That stream is subscribed to only while this tab is mounted; it is useless
// anywhere else and would be dead weight on a driving client's socket.
//
// The live readout under the mapping is not decoration: it applies the current
// dead zone, gains and inversion to the raw axes, so an operator can confirm a
// binding is right without driving the robot to find out.

import { useEffect, useRef, useState } from "preact/hooks";
import { conn, gamepad, robotDocuments, settingsResult } from "../../net/ws.ts";
import {
  AXIS_FIELDS,
  BUTTON_FIELDS,
  FEEL_FIELDS,
  ROUTINE_BIND_SLOTS,
  UNBOUND,
} from "../../settings/schema.ts";
import {
  commit,
  configTarget,
  fieldValue,
  watchGamepad,
} from "../../state/settings.ts";
import { requestRoutines, routinesOn } from "../../state/routines.ts";
import { useRadioFetch } from "../../state/fetch.ts";
import { SettingField } from "./Field.tsx";

/** Movement (from an axis's value when listening started) that counts as
 *  "this is the one". Well above stick noise, below a deliberate nudge. */
const AXIS_DETECT_DELTA = 0.5;

function num(path: string, fallback: number): number {
  const v = fieldValue(path);
  return typeof v === "number" ? v : fallback;
}

function text(path: string): string {
  const v = fieldValue(path);
  return typeof v === "string" ? v : "";
}

/** Apply the current mapping to a raw sample, exactly as the server does. */
function derive(axes: number[]) {
  const dz = num("controller.deadzone", 0.08);
  const rest = num("controller.trigger_rest", -1);
  const span = 1 - rest;
  const trigger = (raw: number) =>
    span <= 0 ? 0 : Math.max(0, Math.min(1, (raw - rest) / span));
  const deadzone = (v: number) => (Math.abs(v) < dz ? 0 : v);
  const clamp1 = (v: number) => (v < -1 ? -1 : v > 1 ? 1 : v);

  const r2 = trigger(axes[num("controller.axis_r2", 5)] ?? rest);
  const l2 = trigger(axes[num("controller.axis_l2", 4)] ?? rest);
  const steerRaw = axes[num("controller.axis_steer", 2)] ?? 0;
  const invert = fieldValue("controller.invert_steer") === true;
  return {
    throttle: clamp1(deadzone(r2 - l2) * num("controller.throttle_gain", 1)),
    steer: clamp1(
      deadzone(steerRaw) * num("controller.steer_gain", 1) * (invert ? -1 : 1),
    ),
  };
}

/** Signed -1..1 bar, filling out from the centre. */
function SignedBar({ value }: { value: number }) {
  const pct = Math.min(Math.abs(value), 1) * 50;
  const geom = value >= 0
    ? `left:50%;width:${pct}%`
    : `left:${50 - pct}%;width:${pct}%`;
  return (
    <div class="axis-track">
      <div
        class="axis-fill"
        style={`${geom};background:${value >= 0 ? "var(--accent)" : "var(--warn)"}`}
      />
    </div>
  );
}

export function ControllerSettings() {
  const pad = gamepad.value;
  const connState = conn.value;
  const result = settingsResult.value;
  const rejected = result?.rejected ?? {};
  // Which binding is waiting for a press, and the axis baseline to compare a
  // move against. Null when nothing is listening.
  const [listening, setListening] = useState<string | null>(null);
  const baseline = useRef<number[]>([]);
  const prevButtons = useRef<boolean[]>([]);

  // The routines a slot can be pointed at. Read off the ROBOT's saved document
  // rather than the editor's draft, for the same reason the driving view's
  // buttons are (RoutineControls.tsx): a binding must name something the rover
  // is actually carrying, not something half-written on another tab.
  const rid = configTarget.value;
  const routinesFetch = useRadioFetch(
    rid && `${rid}:routines`,
    !!(rid && robotDocuments.value[rid]?.routines_rev),
    () => rid && requestRoutines(rid),
  );
  const routines = routinesOn(rid);

  // Subscribe to raw frames only while this tab is on screen. Re-sent when the
  // socket comes back, since a subscription is per-connection and a reconnect
  // would otherwise leave the mapping editor looking at a frozen pad.
  useEffect(() => {
    if (connState !== "live") return;
    watchGamepad(true);
    return () => watchGamepad(false);
  }, [connState]);

  // Capture the next press / the next axis to move, and bind it.
  useEffect(() => {
    if (!pad) return;
    const path = listening;
    if (!path) {
      prevButtons.current = pad.buttons;
      return;
    }
    if (path.startsWith("controller.axis_")) {
      const base = baseline.current;
      for (let idx = 0; idx < pad.axes.length; idx++) {
        const from = base[idx] ?? pad.axes[idx];
        if (Math.abs(pad.axes[idx] - from) >= AXIS_DETECT_DELTA) {
          commit(path, idx);
          setListening(null);
          break;
        }
      }
    } else {
      for (let idx = 0; idx < pad.buttons.length; idx++) {
        if (pad.buttons[idx] && !prevButtons.current[idx]) {
          commit(path, idx);
          setListening(null);
          break;
        }
      }
    }
    prevButtons.current = pad.buttons;
  }, [pad]);

  function listen(path: string) {
    if (listening === path) {
      setListening(null);
      return;
    }
    baseline.current = pad?.axes ?? [];
    prevButtons.current = pad?.buttons ?? [];
    setListening(path);
  }

  const axes = pad?.axes ?? [];
  const drive = derive(axes);
  const boundAxes = new Map<number, string>(
    AXIS_FIELDS.map((f) => [num(f.path, -1), f.label]),
  );
  const boundButtons = new Map<number, string>();
  for (const btn of BUTTON_FIELDS) {
    const idx = num(btn.path, UNBOUND);
    if (idx >= 0) boundButtons.set(idx, btn.label);
  }
  // A slot with a button but no routine is deliberately NOT listed as bound:
  // the server drops it too, and showing it as taken would explain a dead
  // button as a working one.
  for (const slot of ROUTINE_BIND_SLOTS) {
    const idx = num(slot.button, UNBOUND);
    const id = text(slot.routine);
    if (idx >= 0 && id) boundButtons.set(idx, `Routine: ${id}`);
  }

  return (
    <>
      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class={`pill ${pad?.connected ? "ok" : "bad"}`}>
            <span class="led" />
            {pad?.connected ? pad.name ?? "gamepad" : "no gamepad on the base station"}
          </span>
        </div>
      </div>
      <p class="hint pad">
        These bindings are the mapping the base station itself drives with: it
        reads the pad plugged into it and sends straight out over the radio. A
        controller connected to this tablet does nothing — plug it into the base
        station instead.
      </p>

      {!pad?.connected
        ? (
          <p class="banner warn">
            Plug a controller into the base station to bind it. The saved
            mapping is still editable by hand below.
          </p>
        )
        : (
          <section class="group open">
            <div class="group-head static">
              <span class="group-title">Live input</span>
            </div>
            <div class="group-body">
              <div class="live-drive">
                <div class="live-row">
                  <span class="k">throttle</span>
                  <SignedBar value={drive.throttle} />
                  <span class="v">{drive.throttle.toFixed(2)}</span>
                </div>
                <div class="live-row">
                  <span class="k">steer</span>
                  <SignedBar value={drive.steer} />
                  <span class="v">{drive.steer.toFixed(2)}</span>
                </div>
              </div>
              <p class="group-blurb">
                What the mapping above produces right now — dead zone, gains and
                inversion included. This is the command the robot would get.
              </p>

              <div class="axis-list">
                {axes.map((value, idx) => (
                  <div class={`axis-row${boundAxes.has(idx) ? " bound" : ""}`} key={idx}>
                    <span class="axis-idx">{idx}</span>
                    <SignedBar value={value} />
                    <span class="axis-val">{value.toFixed(2)}</span>
                    <span class="axis-name">{boundAxes.get(idx) ?? ""}</span>
                  </div>
                ))}
              </div>

              <div class="button-grid">
                {(pad.buttons ?? []).map((down, idx) => (
                  <span
                    key={idx}
                    class={`btn-chip${down ? " down" : ""}${
                      boundButtons.has(idx) ? " bound" : ""
                    }`}
                    title={boundButtons.get(idx) ?? "unbound"}
                  >
                    {idx}
                  </span>
                ))}
              </div>
            </div>
          </section>
        )}

      <section class="group open">
        <div class="group-head static">
          <span class="group-title">Buttons</span>
        </div>
        <div class="group-body">
          {BUTTON_FIELDS.map((btn) => {
            const idx = num(btn.path, UNBOUND);
            const waiting = listening === btn.path;
            return (
              <div class={`bind-row${waiting ? " listening" : ""}`} key={btn.path}>
                <span class="field-label">{btn.label}</span>
                <span class="bind-value">
                  {waiting
                    ? "press a button…"
                    : idx < 0
                    ? "unbound"
                    : `button ${idx}`}
                </span>
                <button
                  type="button"
                  class={`btn small${waiting ? " active" : ""}`}
                  disabled={!pad?.connected && !waiting}
                  onClick={() => listen(btn.path)}
                >
                  {waiting ? "Cancel" : "Bind"}
                </button>
                <button
                  type="button"
                  class="btn ghost small"
                  disabled={idx < 0}
                  onClick={() => commit(btn.path, UNBOUND)}
                >
                  Clear
                </button>
              </div>
            );
          })}
        </div>
      </section>

      <section class="group open">
        <div class="group-head static">
          <span class="group-title">Routines</span>
        </div>
        <div class="group-body">
          <p class="group-blurb">
            Put a routine on a button. Pick which one, bind a button, and
            pressing it runs that routine on whichever rover is selected —
            exactly as tapping it in the driving view does. The mode buttons
            above are how you get back: switching to teleop ends a running
            routine.
          </p>

          {ROUTINE_BIND_SLOTS.map((slot) => {
            const idx = num(slot.button, UNBOUND);
            const id = text(slot.routine);
            const waiting = listening === slot.button;
            // An id the rover no longer lists still gets an option of its own,
            // so a routine that was renamed — or a rover that is simply off —
            // shows the binding it has instead of silently reading as "none".
            const missing = !!id && !routines.some((r) => r.id === id);
            return (
              <div class={`bind-row${waiting ? " listening" : ""}`} key={slot.button}>
                <select
                  class="field-select bind-routine"
                  value={id}
                  onChange={(e) =>
                    commit(slot.routine, (e.target as HTMLSelectElement).value)}
                >
                  <option value="">— no routine —</option>
                  {routines.map((r) => (
                    <option key={r.id} value={r.id}>{r.name || r.id}</option>
                  ))}
                  {missing && <option value={id}>{id} (not on this rover)</option>}
                </select>
                <span class="bind-value">
                  {waiting
                    ? "press a button…"
                    : idx < 0
                    ? "unbound"
                    : !id
                    // Both halves are needed, and saying so beats letting an
                    // operator press a bound-looking button and get nothing.
                    ? `button ${idx} — pick a routine`
                    : `button ${idx}`}
                </span>
                <button
                  type="button"
                  class={`btn small${waiting ? " active" : ""}`}
                  disabled={!pad?.connected && !waiting}
                  onClick={() => listen(slot.button)}
                >
                  {waiting ? "Cancel" : "Bind"}
                </button>
                <button
                  type="button"
                  class="btn ghost small"
                  disabled={idx < 0 && !id}
                  onClick={() => {
                    commit(slot.button, UNBOUND);
                    commit(slot.routine, "");
                  }}
                >
                  Clear
                </button>
              </div>
            );
          })}

          {routines.length === 0 && (
            <p class="hint">
              {!rid
                ? "No rover selected, so there are no routines to choose from. The bindings themselves belong to this base station and are kept either way."
                : routinesFetch.pending
                ? `Asking ${rid} for its routines…`
                : routinesFetch.stalled
                ? `No answer from ${rid} — its routines never arrived.`
                : `${rid} has no routines yet. Build one in the Routines tab and it appears here.`}
            </p>
          )}
          {rid && routines.length === 0 && routinesFetch.stalled && (
            <button type="button" class="btn ghost small" onClick={routinesFetch.retry}>
              Ask again
            </button>
          )}
        </div>
      </section>

      <section class="group open">
        <div class="group-head static">
          <span class="group-title">Axes</span>
        </div>
        <div class="group-body">
          {AXIS_FIELDS.map((field) => {
            const waiting = listening === field.path;
            return (
              <div class={`bind-row${waiting ? " listening" : ""}`} key={field.path}>
                <span class="field-label">{field.label}</span>
                <span class="bind-value">
                  {waiting ? "move it…" : `axis ${num(field.path, -1)}`}
                </span>
                <button
                  type="button"
                  class={`btn small${waiting ? " active" : ""}`}
                  disabled={!pad?.connected && !waiting}
                  onClick={() => listen(field.path)}
                >
                  {waiting ? "Cancel" : "Detect"}
                </button>
              </div>
            );
          })}
          <p class="group-blurb">
            Detect binds whichever axis you move first. Triggers rest at one end
            of their travel, so pull and release the one you want.
          </p>
        </div>
      </section>

      <section class="group open">
        <div class="group-head static">
          <span class="group-title">Feel</span>
        </div>
        <div class="group-body">
          {FEEL_FIELDS.map((field) => (
            <SettingField
              key={field.path}
              field={field}
              rejected={rejected[field.path]}
            />
          ))}
        </div>
      </section>
    </>
  );
}
