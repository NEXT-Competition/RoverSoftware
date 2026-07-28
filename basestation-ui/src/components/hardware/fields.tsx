// Small controls for editing a document in place.
//
// Deliberately not settings/Field.tsx. That component's whole job is the round
// trip — stage on drag, commit on release, show what the robot clamped it to.
// These edit a local draft that goes nowhere until Save, so they are plain
// inputs with no server in the loop.

import type { JSX } from "preact";

export function TextField(
  { label, value, onChange, placeholder, hint, invalid }: {
    label: string;
    value: string;
    onChange: (value: string) => void;
    placeholder?: string;
    hint?: string;
    invalid?: boolean;
  },
) {
  return (
    <label class={`doc-field${invalid ? " invalid" : ""}`}>
      <span class="doc-label">{label}</span>
      <input
        class="field-input"
        type="text"
        value={value}
        placeholder={placeholder}
        onInput={(e) => onChange((e.target as HTMLInputElement).value)}
      />
      {hint && <span class="doc-hint">{hint}</span>}
    </label>
  );
}

export function NumberField(
  { label, value, onChange, min, max, step = 1, unit, hint, invalid }: {
    label: string;
    value: number;
    onChange: (value: number) => void;
    min?: number;
    max?: number;
    step?: number;
    unit?: string;
    hint?: string;
    invalid?: boolean;
  },
) {
  return (
    <label class={`doc-field${invalid ? " invalid" : ""}`}>
      <span class="doc-label">
        {label}
        {unit && <span class="doc-unit">{unit}</span>}
      </span>
      <input
        class="field-input"
        type="number"
        value={value}
        min={min}
        max={max}
        step={step}
        onInput={(e) => {
          const parsed = Number((e.target as HTMLInputElement).value);
          if (!Number.isNaN(parsed)) onChange(parsed);
        }}
      />
      {hint && <span class="doc-hint">{hint}</span>}
    </label>
  );
}

export function SelectField<T extends string>(
  { label, value, options, onChange, hint }: {
    label: string;
    value: T;
    options: { value: T; label: string }[];
    onChange: (value: T) => void;
    hint?: string;
  },
) {
  return (
    <label class="doc-field">
      <span class="doc-label">{label}</span>
      <select
        class="field-select"
        value={value}
        onChange={(e) => onChange((e.target as HTMLSelectElement).value as T)}
      >
        {options.map((o) => <option key={o.value} value={o.value}>{o.label}</option>)}
      </select>
      {hint && <span class="doc-hint">{hint}</span>}
    </label>
  );
}

export function ToggleField(
  { label, value, onChange, hint }: {
    label: string;
    value: boolean;
    onChange: (value: boolean) => void;
    hint?: string;
  },
) {
  return (
    <label class="doc-field row">
      <input
        type="checkbox"
        checked={value}
        onChange={(e) => onChange((e.target as HTMLInputElement).checked)}
      />
      <span class="doc-label">{label}</span>
      {hint && <span class="doc-hint">{hint}</span>}
    </label>
  );
}

/** A row of toggles for picking which actuators fill a role. */
export function ChipPicker(
  { label, options, selected, onToggle, hint }: {
    label: string;
    options: string[];
    selected: string[];
    onToggle: (name: string) => void;
    hint?: string;
  },
): JSX.Element {
  return (
    <div class="doc-field">
      <span class="doc-label">{label}</span>
      <div class="chip-row">
        {options.length === 0
          ? <span class="doc-hint">No actuators yet.</span>
          : options.map((name) => (
            <button
              key={name}
              type="button"
              class={`chip${selected.includes(name) ? " on" : ""}`}
              onClick={() => onToggle(name)}
            >
              {name}
            </button>
          ))}
      </div>
      {hint && <span class="doc-hint">{hint}</span>}
    </div>
  );
}
