// One motor or servo: what it is called, where it is plugged in, how it maps.

import type { ActuatorSpec } from "../../net/types.ts";
import { channelConflicts, setActuatorField } from "../../state/hardware.ts";
import { NumberField, SelectField, TextField, ToggleField } from "./fields.tsx";

const KINDS = [
  { value: "esc" as const, label: "ESC (spins)" },
  { value: "servo" as const, label: "Servo (positions)" },
];

export function ActuatorCard(
  { actuator, mech, onRemove }: {
    actuator: ActuatorSpec;
    /** The owning mechanism's name, or null for a drivetrain actuator. */
    mech: string | null;
    onRemove: () => void;
  },
) {
  const set = <K extends keyof ActuatorSpec>(key: K, value: ActuatorSpec[K]) =>
    setActuatorField(mech, actuator.name, key, value);
  const clash = channelConflicts.value.has(actuator.channel);

  return (
    <div class="actuator-card">
      <div class="actuator-head">
        <TextField
          label="Name"
          value={actuator.name}
          onChange={(v) => set("name", v)}
          hint="Lower case, no spaces. Also its tuning path and how a routine refers to it."
        />
        <button type="button" class="btn ghost small danger" onClick={onRemove}>
          Remove
        </button>
      </div>

      <div class="doc-grid">
        <SelectField
          label="Type"
          value={actuator.kind}
          options={KINDS}
          onChange={(v) => set("kind", v)}
          hint="An ESC is held at neutral at boot so it arms; a servo just parks."
        />
        <NumberField
          label="PWM channel"
          value={actuator.channel}
          min={0}
          max={15}
          onChange={(v) => set("channel", Math.round(v))}
          invalid={clash}
          hint={clash
            ? "Another actuator already claims this channel — two on one move together and neither answers its own commands."
            : "Fusion HAT channel, 0-15."}
        />
        <ToggleField
          label="Inverted"
          value={!!actuator.inverted}
          onChange={(v) => set("inverted", v)}
          hint="For a motor mounted facing the other way."
        />
        <NumberField
          label="Neutral angle"
          unit="°"
          value={actuator.neutral_angle ?? 5}
          min={-90}
          max={90}
          step={0.5}
          onChange={(v) => set("neutral_angle", v)}
          hint="Where this ESC holds stop. Find it with tools/esc_calibrate.py."
        />
        <NumberField
          label="Forward endpoint"
          unit="°"
          value={actuator.max_angle ?? 20}
          min={-90}
          max={90}
          step={0.5}
          onChange={(v) => set("max_angle", v)}
        />
        <NumberField
          label="Reverse endpoint"
          unit="°"
          value={actuator.min_angle ?? -20}
          min={-90}
          max={90}
          step={0.5}
          onChange={(v) => set("min_angle", v)}
          hint="The throw is symmetric about neutral, so whichever endpoint is closer to it sets the range."
        />
      </div>
    </div>
  );
}
