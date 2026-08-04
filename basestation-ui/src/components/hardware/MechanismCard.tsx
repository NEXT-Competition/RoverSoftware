// One mechanism: an intake, an arm, a second launcher.
//
// Two shapes, and the editor shows only the fields the chosen one has:
//   power  hold a value; named presets an operator or a routine asks for
//   pulse  a timed swing-hold-return cycle, the launcher generalized

import { useState } from "preact/hooks";
import type { MechanismSpec } from "../../net/types.ts";
import {
  addMechanismActuator,
  addPreset,
  removeMechanism,
  removeMechanismActuator,
  removePreset,
  setMechanismField,
  setPreset,
} from "../../state/hardware.ts";
import { ActuatorCard } from "./ActuatorCard.tsx";
import { JogControl } from "./JogControl.tsx";
import { NumberField, SelectField, TextField, ToggleField } from "./fields.tsx";

const KINDS = [
  { value: "power" as const, label: "Power (holds a value)" },
  { value: "pulse" as const, label: "Pulse (timed cycle)" },
];

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

      {mech.kind === "pulse"
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
            <NumberField
              label="Spin-up ramp"
              unit="/s"
              value={mech.slew_rate ?? 0}
              min={0}
              max={20}
              step={0.1}
              onChange={(v) => set("slew_rate", v)}
              hint="Limits how fast it winds UP. A flywheel told to go from stop to full in one step asks its ESC for a current it cannot give, and the ESC cuts out. 0 = go straight there. Stopping is never ramped."
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
