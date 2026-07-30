// One motor or servo: what it is called, where it is plugged in, how it maps.

import type { ActuatorSpec } from "../../net/types.ts";
import {
  channelConflicts,
  encoderPinConflicts,
  setActuatorField,
  toggleEncoder,
} from "../../state/hardware.ts";
import { NumberField, SelectField, TextField, ToggleField } from "./fields.tsx";

const KINDS = [
  { value: "esc" as const, label: "ESC (spins)" },
  { value: "servo" as const, label: "Servo (positions)" },
];

/** No encoder. Not 0, which is a real BCM pin — matches robot/config.py. */
const NO_PIN = -1;

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

  const pinA = actuator.encoder_a ?? NO_PIN;
  const pinB = actuator.encoder_b ?? NO_PIN;
  const pins = encoderPinConflicts.value;
  // Setting one pin and not the other is refused by the robot, so say so here
  // rather than at Save: half a quadrature encoder decodes nothing.
  const halfWired = (pinA === NO_PIN) !== (pinB === NO_PIN);
  // The whole encoder block is collapsed away until somebody opens it. Most
  // builds have no encoders, and four more fields on every actuator card would
  // make the common case pay for the rare one.
  const hasEncoder = pinA !== NO_PIN || pinB !== NO_PIN || !!actuator.encoder_cpr;

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

      <div class="actuator-encoder">
        <div class="encoder-head">
          <div class="doc-label">
            Encoder
            <span class="doc-hint">
              Measures how fast this wheel really turns, so the two sides can be
              held together. Optional — without one the motor runs open-loop.
            </span>
          </div>
          <button
            type="button"
            class={`btn ghost small${hasEncoder ? " danger" : ""}`}
            onClick={() => toggleEncoder(mech, actuator.name)}
          >
            {hasEncoder ? "Remove" : "Add"}
          </button>
        </div>

        {hasEncoder && (
          <div class="doc-grid tight">
            <NumberField
              label="A pin"
              value={pinA}
              min={NO_PIN}
              max={27}
              onChange={(v) => set("encoder_a", Math.round(v))}
              invalid={pins.has(pinA) || halfWired}
              hint={pins.has(pinA)
                ? "Another actuator already reads this pin — both would count the same edges."
                : "A Fusion HAT DIGITAL pin (numbered as BCM GPIO), not the PWM channel above."}
            />
            <NumberField
              label="B pin"
              value={pinB}
              min={NO_PIN}
              max={27}
              onChange={(v) => set("encoder_b", Math.round(v))}
              invalid={pins.has(pinB) || halfWired}
              hint={halfWired
                ? "Set both pins or neither — one channel decodes nothing."
                : "The second quadrature channel."}
            />
            <NumberField
              label="Counts per rev"
              value={actuator.encoder_cpr ?? 0}
              min={0}
              max={100000}
              step={1}
              onChange={(v) => set("encoder_cpr", v)}
              hint="Of the WHEEL, gearbox included. Measure it: tools/encoder_monitor.py, turn the wheel once."
            />
            <ToggleField
              label="Count inverted"
              value={!!actuator.encoder_invert}
              onChange={(v) => set("encoder_invert", v)}
              hint="So forward reads positive. Separate from Inverted — that mirrors the motor, this the sensor."
            />
          </div>
        )}
      </div>
    </div>
  );
}
