// One editable setting. Every control on the settings page is this component.
//
// Numbers get BOTH a slider and a number box, deliberately. The console is
// driven on a touchscreen with gloves on, where a slider is the only usable
// control; but "Kp = 0.42" is a value you want to type exactly, and a slider
// alone can't express that. They edit the same value.
//
// When each control commits differs, and that difference is the whole point:
//   slider   staged while dragging, sent on release  (one radio frame, not 60)
//   number   sent on Enter or blur                   (not on every keystroke)
//   toggle   sent immediately                        (there is no "mid-drag")
//   enum     sent immediately
//   text     sent on Enter or blur
//
// The value shown is the server's, unless a local edit is outstanding — see
// state/settings.ts for why that distinction has to exist.

import type { JSX } from "preact";
import { useEffect, useRef, useState } from "preact/hooks";
import type { Field } from "../../settings/schema.ts";
import { formatValue } from "../../settings/schema.ts";
import {
  commit,
  commitStaged,
  fieldValue,
  isConfirmed,
  isDirty,
  revert,
  stage,
} from "../../state/settings.ts";

interface Props {
  field: Field;
  /** Why the server refused this value, if it did. */
  rejected?: string;
  /** True when the value is stored but only applies after a restart. */
  restartPending?: boolean;
}

function clampToField(field: Field, n: number): number {
  const lo = field.min ?? -Infinity;
  const hi = field.max ?? Infinity;
  return n < lo ? lo : n > hi ? hi : n;
}

/** Typed entry for a numeric field. Lives in the label row. */
function NumberBox({ field }: { field: Field }) {
  const value = fieldValue(field.path);
  const num = typeof value === "number" ? value : Number(value ?? 0);
  // Uncontrolled while being typed into, so entering "0.0" doesn't get
  // reformatted to "0" under the cursor after every keystroke.
  const [typing, setTyping] = useState<string | null>(null);

  function commitTyped(raw: string) {
    setTyping(null);
    const parsed = Number(raw);
    if (raw.trim() === "" || Number.isNaN(parsed)) {
      revert(field.path); // unparseable -> fall back to what the server has
      return;
    }
    commit(field.path, clampToField(field, parsed));
  }

  return (
    <input
      class="field-num"
      type="number"
      inputMode="decimal"
      min={field.min}
      max={field.max}
      step={field.step ?? 1}
      value={typing ?? formatValue(field, num)}
      onInput={(e) => setTyping((e.target as HTMLInputElement).value)}
      onBlur={(e) => commitTyped((e.target as HTMLInputElement).value)}
      onKeyDown={(e: JSX.TargetedKeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") (e.target as HTMLInputElement).blur();
        if (e.key === "Escape") {
          setTyping(null);
          revert(field.path);
          (e.target as HTMLInputElement).blur();
        }
      }}
    />
  );
}

/** Full-width slider for a numeric field. Sits on its own row under the label,
 *  because sharing a row leaves neither the label nor the travel enough space. */
function RangeInput({ field }: { field: Field }) {
  const value = fieldValue(field.path);
  const num = typeof value === "number" ? value : Number(value ?? 0);
  return (
    <input
      class="field-range"
      type="range"
      min={field.min ?? 0}
      max={field.max ?? 1}
      step={field.step ?? 1}
      value={num}
      // Staged on input, sent on change: a range input fires `change` once, on
      // release, which is one radio frame per adjustment rather than one per
      // pixel of travel.
      onInput={(e) => stage(field.path, Number((e.target as HTMLInputElement).value))}
      onChange={() => commitStaged(field.path)}
    />
  );
}

function BoolField({ field }: { field: Field }) {
  const on = fieldValue(field.path) === true;
  return (
    <button
      type="button"
      role="switch"
      aria-checked={on}
      class={`toggle${on ? " on" : ""}`}
      onClick={() => commit(field.path, !on)}
    >
      <span class="knob" />
    </button>
  );
}

function EnumField({ field }: { field: Field }) {
  const value = String(fieldValue(field.path) ?? "");
  const choices = field.choices ?? [];
  // Segmented control while the options fit on one row; a select past that.
  if (choices.length <= 3) {
    return (
      <div class="segmented">
        {choices.map((choice) => (
          <button
            key={choice}
            type="button"
            class={value === choice ? "on" : ""}
            onClick={() => commit(field.path, choice)}
          >
            {choice}
          </button>
        ))}
      </div>
    );
  }
  return (
    <select
      class="field-select"
      value={value}
      onChange={(e) => commit(field.path, (e.target as HTMLSelectElement).value)}
    >
      {choices.map((choice) => <option key={choice} value={choice}>{choice}</option>)}
    </select>
  );
}

function TextField({ field }: { field: Field }) {
  const value = String(fieldValue(field.path) ?? "");
  const ref = useRef<HTMLInputElement>(null);
  // Keep the box in step with the server unless the operator is typing in it.
  useEffect(() => {
    const el = ref.current;
    if (el && document.activeElement !== el) el.value = value;
  }, [value]);
  return (
    <input
      ref={ref}
      class="field-text"
      type="text"
      spellcheck={false}
      autocomplete="off"
      value={value}
      onBlur={(e) => commit(field.path, (e.target as HTMLInputElement).value)}
      onKeyDown={(e: JSX.TargetedKeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") (e.target as HTMLInputElement).blur();
      }}
    />
  );
}

export function SettingField({ field, rejected, restartPending }: Props) {
  const numeric = field.kind === "float" || field.kind === "int";
  const missing = fieldValue(field.path) === undefined;
  const cls = [
    "field",
    numeric ? "numeric" : "inline",
    isDirty(field.path) ? "dirty" : "",
    isConfirmed(field.path) ? "confirmed" : "",
    rejected ? "rejected" : "",
    missing ? "unknown" : "",
  ].filter(Boolean).join(" ");

  return (
    <div class={cls}>
      <div class="field-head">
        <span class="field-label">
          {field.label}
          {field.unit && <span class="field-unit">{field.unit}</span>}
          {field.live === false && (
            <span
              class="badge"
              title="Stored now, but the service only picks it up on restart"
            >
              restart
            </span>
          )}
          {restartPending && <span class="badge warn">pending restart</span>}
        </span>
        {field.kind === "bool" && <BoolField field={field} />}
        {field.kind === "enum" && <EnumField field={field} />}
        {field.kind === "text" && <TextField field={field} />}
        {numeric && <NumberBox field={field} />}
      </div>
      {numeric && <RangeInput field={field} />}
      {rejected
        ? <p class="field-help error">Refused: {rejected}</p>
        : field.help && <p class="field-help">{field.help}</p>}
    </div>
  );
}
