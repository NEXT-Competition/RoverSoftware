// One mechanism: an intake, an arm, a second launcher.
//
// Three shapes, and the editor shows only the fields the chosen one has:
//   power     hold a value; named presets an operator or a routine asks for
//   pulse     a timed swing-hold-return cycle, the launcher generalized
//   sequence  an ordered queue: servo first, then one motor, then another

import { useState } from "preact/hooks";
import type { MechanismSpec, SequenceStepSpec, WaitSpec } from "../../net/types.ts";
import {
  addMechanismActuator,
  addPreset,
  addStep,
  moveStep,
  removeMechanism,
  removeMechanismActuator,
  removePreset,
  removeStep,
  setMechanismField,
  setPreset,
  setStepField,
  setStepValue,
  setStepWait,
} from "../../state/hardware.ts";
import { ActuatorCard } from "./ActuatorCard.tsx";
import { JogControl } from "./JogControl.tsx";
import { NumberField, SelectField, TextField, ToggleField } from "./fields.tsx";

const KINDS = [
  { value: "power" as const, label: "Power (holds a value)" },
  { value: "pulse" as const, label: "Pulse (timed cycle)" },
  { value: "sequence" as const, label: "Sequence (one after another)" },
];

const WAIT_KINDS = [
  { value: "", label: "nothing — just the time above" },
  { value: "rpm", label: "a motor reaching a speed" },
  { value: "mech_ready", label: "another mechanism being ready" },
];

const ON_TIMEOUT = [
  { value: "abort" as const, label: "stop the whole sequence" },
  { value: "advance" as const, label: "carry on anyway" },
];

function StepRow(
  { mech, step, index, total }: {
    mech: MechanismSpec;
    step: SequenceStepSpec;
    index: number;
    total: number;
  },
) {
  const wait = step.wait_for;
  // Only actuators that actually measure a speed can be waited on — the robot
  // refuses a speed gate on an actuator with no encoder pins, so offering one
  // here would be offering a choice that cannot be saved.
  const measured = mech.actuators.filter(
    (a) => (a.encoder_a ?? -1) !== -1 && (a.encoder_b ?? -1) !== -1,
  );

  const setWaitField = (key: keyof WaitSpec, value: string | number) =>
    setStepWait(mech.name, index, { ...(wait ?? { kind: "rpm" }), [key]: value } as WaitSpec);

  return (
    <div class="step-row">
      <div class="step-head">
        <span class="step-index">{index + 1}</span>
        <input
          class="field-input"
          type="text"
          placeholder={`step ${index + 1}`}
          value={step.name ?? ""}
          onInput={(e) =>
            setStepField(mech.name, index, "name", (e.target as HTMLInputElement).value)}
        />
        <button
          type="button"
          class="btn ghost small"
          disabled={index === 0}
          title="Move earlier"
          onClick={() => moveStep(mech.name, index, -1)}
        >
          ↑
        </button>
        <button
          type="button"
          class="btn ghost small"
          disabled={index === total - 1}
          title="Move later"
          onClick={() => moveStep(mech.name, index, 1)}
        >
          ↓
        </button>
        <button
          type="button"
          class="btn ghost small danger"
          onClick={() => removeStep(mech.name, index)}
        >
          Remove
        </button>
      </div>

      <div class="doc-label">
        Move
        <span class="doc-hint">
          Blank leaves an actuator exactly as it was, so a flywheel started in
          an earlier step keeps spinning through this one. A servo is in
          degrees; a motor is throttle, −1 to 1.
        </span>
      </div>
      {/* A checkbox per actuator rather than a bare number, because "not
          mentioned" and "set to 0" mean opposite things here — leave the
          flywheel alone, versus stop it — and a grid of numbers cannot show
          the difference. */}
      <div class="step-values">
        {mech.actuators.map((actuator) => {
          const servo = actuator.kind === "servo";
          const held = actuator.name in step.values;
          return (
            <div class="step-value" key={actuator.name}>
              <ToggleField
                label={`${actuator.name}${servo ? " (°)" : ""}`}
                value={held}
                onChange={(on) =>
                  setStepValue(mech.name, index, actuator.name, on ? 0 : null)}
              />
              {held
                ? (
                  <NumberField
                    label=""
                    value={step.values[actuator.name] ?? 0}
                    min={servo ? -90 : -1}
                    max={servo ? 90 : 1}
                    step={servo ? 0.5 : 0.05}
                    onChange={(v) => setStepValue(mech.name, index, actuator.name, v)}
                  />
                )
                : <span class="doc-hint">unchanged</span>}
            </div>
          );
        })}
      </div>
      <ToggleField
        label="Stop everything this step doesn't name"
        value={step.clear === true}
        onChange={(v) => setStepField(mech.name, index, "clear", v)}
      />


      <div class="doc-grid tight">
        <NumberField
          label="Hold for at least"
          unit="s"
          value={step.seconds ?? 0}
          min={0}
          max={60}
          step={0.05}
          onChange={(v) => setStepField(mech.name, index, "seconds", v)}
          hint="A floor, not a duration: the step also waits for the condition below."
        />
        <SelectField
          label="Then wait for"
          value={wait?.kind ?? ""}
          options={WAIT_KINDS}
          onChange={(v) =>
            setStepWait(
              mech.name,
              index,
              v === "" ? null : ({ kind: v } as WaitSpec),
            )}
        />
      </div>

      {wait?.kind === "rpm" && (
        <div class="doc-grid tight">
          <SelectField
            label="Motor"
            value={wait.actuator ?? ""}
            options={measured.map((a) => ({ value: a.name, label: a.name }))}
            onChange={(v) => setWaitField("actuator", v)}
            hint={measured.length === 0
              ? "No actuator here has encoder pins set — a speed cannot be waited on until one does."
              : undefined}
          />
          <NumberField
            label="At least"
            unit="rpm"
            value={wait.at_least ?? 0}
            min={0}
            max={60000}
            step={50}
            onChange={(v) => setWaitField("at_least", v)}
            hint="0 = no floor. Direction is ignored, so a reversed flywheel still counts."
          />
          <NumberField
            label="At most"
            unit="rpm"
            value={wait.at_most ?? 0}
            min={0}
            max={60000}
            step={50}
            onChange={(v) => setWaitField("at_most", v)}
            hint="0 = no ceiling."
          />
        </div>
      )}

      {wait?.kind === "mech_ready" && (
        <TextField
          label="Mechanism"
          value={wait.mech ?? ""}
          onChange={(v) => setWaitField("mech", v)}
          hint="Its name, as a routine would say it. “shooter” is the built-in launcher."
        />
      )}

      {wait && (
        <div class="doc-grid tight">
          <NumberField
            label="Give up after"
            unit="s"
            value={step.timeout ?? 0}
            min={0}
            max={60}
            step={0.5}
            onChange={(v) => setStepField(mech.name, index, "timeout", v)}
            hint="0 = use the mechanism's step timeout."
          />
          <SelectField
            label="If it never happens"
            value={step.on_timeout ?? "abort"}
            options={ON_TIMEOUT}
            onChange={(v) =>
              setStepField(mech.name, index, "on_timeout", v as "abort" | "advance")}
          />
        </div>
      )}
    </div>
  );
}

function SequenceEditor({ mech }: { mech: MechanismSpec }) {
  const list = mech.steps ?? [];
  return (
    <div class="sequence-editor">
      <div class="doc-label">
        Steps
        <span class="doc-hint">
          Run in order, one at a time, off the control loop — so nothing here
          blocks driving. Each step moves what it names, waits, and hands on to
          the next.
        </span>
      </div>

      {list.length === 0 && (
        <p class="doc-hint">
          No steps yet, so activating this does nothing.
        </p>
      )}

      {list.map((step, index) => (
        <StepRow
          key={index}
          mech={mech}
          step={step}
          index={index}
          total={list.length}
        />
      ))}

      <button type="button" class="btn ghost small" onClick={() => addStep(mech.name)}>
        + Step
      </button>
    </div>
  );
}

function PresetEditor({ mech }: { mech: MechanismSpec }) {
  const [name, setName] = useState("");
  const presets = mech.presets ?? {};
  return (
    <div class="preset-editor">
      <div class="doc-label">
        Presets
        <span class="doc-hint">
          Named states a routine asks for by name, so it reads “intake → in”
          rather than a column of numbers.
        </span>
      </div>

      {Object.keys(presets).length === 0 && (
        <p class="doc-hint">No presets yet.</p>
      )}

      {Object.entries(presets).map(([preset, values]) => (
        <div key={preset} class="preset-row">
          <div class="preset-head">
            <span class="preset-name">{preset}</span>
            <button
              type="button"
              class="btn ghost small danger"
              onClick={() => removePreset(mech.name, preset)}
            >
              Remove
            </button>
          </div>
          <div class="doc-grid tight">
            {mech.actuators.map((actuator) => (
              <NumberField
                key={actuator.name}
                label={actuator.name}
                value={values[actuator.name] ?? 0}
                min={-1}
                max={1}
                step={0.05}
                onChange={(v) => setPreset(mech.name, preset, actuator.name, v)}
              />
            ))}
          </div>
        </div>
      ))}

      <div class="preset-add">
        <input
          class="field-input"
          type="text"
          placeholder="new preset name"
          value={name}
          onInput={(e) => setName((e.target as HTMLInputElement).value)}
        />
        <button
          type="button"
          class="btn ghost small"
          disabled={!name.trim()}
          onClick={() => {
            addPreset(mech.name, name.trim());
            setName("");
          }}
        >
          Add preset
        </button>
      </div>
    </div>
  );
}

export function MechanismCard({ mech }: { mech: MechanismSpec }) {
  const set = <K extends keyof MechanismSpec>(key: K, value: MechanismSpec[K]) =>
    setMechanismField(mech.name, key, value);

  return (
    <section class="mech-card">
      <div class="mech-head">
        <TextField
          label="Name"
          value={mech.name}
          onChange={(v) => set("name", v)}
          hint="How a routine refers to it. “shooter” is reserved for the built-in launcher."
        />
        <SelectField
          label="Type"
          value={mech.kind}
          options={KINDS}
          onChange={(v) => set("kind", v)}
        />
        <button
          type="button"
          class="btn ghost small danger"
          onClick={() => removeMechanism(mech.name)}
        >
          Remove
        </button>
      </div>

      <ToggleField
        label="Enabled"
        value={mech.enabled !== false}
        onChange={(v) => set("enabled", v)}
      />

      {mech.kind === "sequence"
        ? (
          <>
            <div class="doc-grid">
              <NumberField
                label="Rest angle"
                unit="°"
                value={mech.rest_angle ?? -30}
                min={-90}
                max={90}
                step={0.5}
                onChange={(v) => set("rest_angle", v)}
                hint="Where a servo on this mechanism parks between runs, and where an e-stop puts it."
              />
              <NumberField
                label="Step timeout"
                unit="s"
                value={mech.step_timeout ?? 5}
                min={0.1}
                max={60}
                step={0.5}
                onChange={(v) => set("step_timeout", v)}
                hint="The backstop for a step waiting on something that never happens."
              />
              <NumberField
                label="Cooldown"
                unit="s"
                value={mech.cooldown ?? 0}
                min={0}
                max={60}
                step={0.1}
                onChange={(v) => set("cooldown", v)}
              />
              <NumberField
                label="Magazine"
                value={mech.max_activations ?? 0}
                min={0}
                max={999}
                onChange={(v) => set("max_activations", Math.round(v))}
                hint="0 = unlimited."
              />
            </div>
            <ToggleField
              label="Loop"
              value={mech.loop === true}
              onChange={(v) => set("loop", v)}
            />
            <SequenceEditor mech={mech} />
          </>
        )
        : mech.kind === "pulse"
        ? (
          <div class="doc-grid">
            <NumberField
              label="Rest angle"
              unit="°"
              value={mech.rest_angle ?? -30}
              min={-90}
              max={90}
              step={0.5}
              onChange={(v) => set("rest_angle", v)}
              hint="Home, and where an e-stop parks it."
            />
            <NumberField
              label="Active angle"
              unit="°"
              value={mech.active_angle ?? 30}
              min={-90}
              max={90}
              step={0.5}
              onChange={(v) => set("active_angle", v)}
            />
            <NumberField
              label="Active time"
              unit="s"
              value={mech.active_seconds ?? 0.35}
              min={0.05}
              max={5}
              step={0.05}
              onChange={(v) => set("active_seconds", v)}
              hint="Too short and it never arrives; too long and it stalls against the stop."
            />
            <NumberField
              label="Recover time"
              unit="s"
              value={mech.recover_seconds ?? 0.35}
              min={0.05}
              max={5}
              step={0.05}
              onChange={(v) => set("recover_seconds", v)}
            />
            <NumberField
              label="Cooldown"
              unit="s"
              value={mech.cooldown ?? 0}
              min={0}
              max={60}
              step={0.1}
              onChange={(v) => set("cooldown", v)}
            />
            <NumberField
              label="Magazine"
              value={mech.max_activations ?? 0}
              min={0}
              max={999}
              onChange={(v) => set("max_activations", Math.round(v))}
              hint="0 = unlimited."
            />
          </div>
        )
        : (
          <>
            <NumberField
              label="Auto-stop"
              unit="s"
              value={mech.auto_stop_seconds ?? 0}
              min={0}
              max={300}
              step={0.5}
              onChange={(v) => set("auto_stop_seconds", v)}
              hint="Stop this long after being told to run. 0 = run until stopped."
            />
            <PresetEditor mech={mech} />
          </>
        )}

      <JogControl mech={mech.name} />

      <div class="actuator-list">
        {mech.actuators.map((actuator) => (
          <ActuatorCard
            key={actuator.name}
            actuator={actuator}
            mech={mech.name}
            onRemove={() => removeMechanismActuator(mech.name, actuator.name)}
          />
        ))}
        <button
          type="button"
          class="btn ghost small"
          onClick={() => addMechanismActuator(mech.name)}
        >
          + Motor
        </button>
      </div>
    </section>
  );
}
