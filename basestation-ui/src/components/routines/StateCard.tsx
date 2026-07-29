// One state: what drives, what happens, and what makes it move on.
//
// The card is the state machine's box. It highlights while the robot is
// actually in it, which is the difference between an editor and a debugger.

import type { RoutineStateSpec } from "../../net/types.ts";
import { DRIVE_MODES } from "../../routines/vocab.ts";
import {
  addAction,
  addTransition,
  removeAction,
  removeTransition,
  setAction,
  setDrive,
  setDriveValue,
  setStateField,
  setTransition,
  type Slot,
} from "../../state/routines.ts";
import { ActionRow } from "./ActionRow.tsx";
import { TransitionRow } from "./TransitionRow.tsx";

const SLOTS: { key: Slot; label: string; hint: string }[] = [
  { key: "on_enter", label: "When entered", hint: "Runs once." },
  {
    key: "on_tick",
    label: "While here",
    hint: "Runs every tick, so keep it to things that are safe to repeat.",
  },
  {
    key: "on_exit",
    label: "When leaving",
    hint: "Runs however the state is left — including a timeout, an abort or an e-stop. The right place to disarm something.",
  },
];

export function StateCard(
  { state, states, isStart, isLive, problems, highlightTransition = null }: {
    state: RoutineStateSpec;
    states: string[];
    isStart: boolean;
    isLive: boolean;
    problems: string[];
    /** Index of the transition selected on the canvas, so drawing a wire and
     *  then setting its condition is one continuous gesture. */
    highlightTransition?: number | null;
  },
) {
  const mode = state.drive?.mode ?? "stop";

  return (
    <section
      id={`state-${state.id}`}
      class={`state-card${isLive ? " live" : ""}${state.terminal ? " terminal" : ""}`}
    >
      <header class="state-head">
        <input
          class="field-input state-id"
          type="text"
          value={state.id}
          onInput={(e) =>
            setStateField(state.id, "id", (e.target as HTMLInputElement).value)}
        />
        {isStart && <span class="pill">start</span>}
        {isLive && <span class="pill live">running</span>}
        <label class="arg">
          <span>final</span>
          <input
            type="checkbox"
            checked={!!state.terminal}
            onChange={(e) =>
              setStateField(state.id, "terminal", (e.target as HTMLInputElement).checked)}
            title="Reaching this state ends the routine. It also loses its output handle on the canvas."
          />
        </label>
      </header>

      {problems.length > 0 && (
        <p class="banner error small">{problems.join(" · ")}</p>
      )}

      <div class="state-drive">
        <label class="arg">
          <span>drives with</span>
          <select
            class="field-select tiny"
            value={mode}
            onChange={(e) => setDrive(state.id, (e.target as HTMLSelectElement).value)}
          >
            {DRIVE_MODES.map((m) => (
              <option key={m.value} value={m.value}>{m.label}</option>
            ))}
          </select>
        </label>

        {mode === "manual" && (
          <>
            <label class="arg">
              <span>throttle</span>
              <input
                class="field-input tiny"
                type="number"
                min={-1}
                max={1}
                step={0.05}
                value={state.drive?.throttle ?? 0}
                onInput={(e) =>
                  setDriveValue(state.id, "throttle",
                    Number((e.target as HTMLInputElement).value))}
              />
            </label>
            <label class="arg">
              <span>steer</span>
              <input
                class="field-input tiny"
                type="number"
                min={-1}
                max={1}
                step={0.05}
                value={state.drive?.steer ?? 0}
                onInput={(e) =>
                  setDriveValue(state.id, "steer",
                    Number((e.target as HTMLInputElement).value))}
              />
            </label>
          </>
        )}

        <label class="arg">
          <span>time limit (s)</span>
          <input
            class="field-input tiny"
            type="number"
            min={0}
            max={600}
            step={1}
            placeholder="default"
            value={state.timeout ?? ""}
            onInput={(e) => {
              const raw = (e.target as HTMLInputElement).value;
              setStateField(state.id, "timeout", raw === "" ? undefined : Number(raw));
            }}
            title="Blank inherits the robot's default. Reaching it ends the routine — if a state overran, the next one's assumptions probably don't hold either."
          />
        </label>
      </div>

      {SLOTS.map((slot) => {
        const actions = state[slot.key] ?? [];
        if (actions.length === 0 && slot.key === "on_tick") {
          // Keep the card readable: the two slots people reach for are entry
          // and exit, so an empty "while here" is a button, not a heading.
          return (
            <button
              key={slot.key}
              type="button"
              class="btn ghost small subtle"
              onClick={() => addAction(state.id, slot.key, { do: "mech_stop", mech: "" })}
            >
              + action while here
            </button>
          );
        }
        return (
          <div key={slot.key} class="slot">
            <div class="slot-head">
              <span class="doc-label">{slot.label}</span>
              <span class="doc-hint">{slot.hint}</span>
            </div>
            {actions.map((action, index) => (
              <ActionRow
                key={index}
                action={action}
                states={states}
                onChange={(next) => setAction(state.id, slot.key, index, next)}
                onRemove={() => removeAction(state.id, slot.key, index)}
              />
            ))}
            <button
              type="button"
              class="btn ghost small subtle"
              onClick={() =>
                addAction(state.id, slot.key, { do: "mech_stop", mech: "" })}
            >
              + action
            </button>
          </div>
        );
      })}

      <div class="slot">
        <div class="slot-head">
          <span class="doc-label">Then</span>
          <span class="doc-hint">
            Checked in order, first match wins — and at most one per tick.
          </span>
        </div>
        {(state.transitions ?? []).map((transition, index) => (
          <TransitionRow
            key={index}
            transition={transition}
            states={states}
            index={index}
            highlighted={index === highlightTransition}
            onChange={(next) => setTransition(state.id, index, next)}
            onRemove={() => removeTransition(state.id, index)}
          />
        ))}
        {!state.terminal && (
          <button
            type="button"
            class="btn ghost small subtle"
            onClick={() => addTransition(state.id)}
          >
            + transition
          </button>
        )}
      </div>
    </section>
  );
}
