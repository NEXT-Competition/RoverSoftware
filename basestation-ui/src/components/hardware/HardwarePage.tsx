// Hardware tab: declare what this robot is built from.
//
// Draft-then-Save, unlike the tuning page next door. A layout is a document,
// not a scalar — half of one is a robot with one drive motor — so the whole
// thing is edited locally and sent in one go.
//
// A saved layout does NOT take effect until the robot restarts, and the page
// says so. Actuators are owned by constructors: rebuilding them mid-loop with
// the drivetrain armed is how an ESC ends up holding an undefined pulse.

import { useEffect } from "preact/hooks";
import { robotDocuments, robots, selectedRobot } from "../../net/ws.ts";
import {
  addDriveActuator,
  addMechanism,
  channelConflicts,
  discardLayout,
  jogUnlocked,
  layout,
  layoutAccepted,
  layoutDirty,
  layoutResult,
  refreshLayout,
  removeDriveActuator,
  saveLayout,
  setDriveField,
  setDriveKind,
  setSteerActuator,
  toggleRole,
} from "../../state/hardware.ts";
import { useRadioFetch } from "../../state/fetch.ts";
import { configTarget, targetRobot } from "../../state/settings.ts";
import { Waiting } from "../settings/Waiting.tsx";
import { ActuatorCard } from "./ActuatorCard.tsx";
import { MechanismCard } from "./MechanismCard.tsx";
import { ChipPicker, NumberField, SelectField, ToggleField } from "./fields.tsx";

const DRIVE_KINDS = [
  { value: "tank" as const, label: "Tank — left/right tracks" },
  { value: "servo_steer" as const, label: "Motor + steering servo" },
  { value: "single" as const, label: "Single motor, no steering" },
  { value: "none" as const, label: "No drivetrain" },
];

function DrivetrainCard() {
  const doc = layout.value;
  if (!doc) return null;
  const drive = doc.drive;
  const names = drive.actuators.map((a) => a.name);
  const robot = selectedRobot.value;
  const alignMode = robot?.mode === "object_align" || robot?.mode === "shooter_align";

  return (
    <section class="group open">
      <div class="group-body">
        <SelectField
          label="Drivetrain"
          value={drive.kind}
          options={DRIVE_KINDS}
          onChange={setDriveKind}
        />

        {drive.kind === "tank" && (
          <>
            <ChipPicker
              label="Left side"
              options={names}
              selected={drive.roles.left}
              onToggle={(n) => toggleRole("left", n)}
              hint="Every motor on this side gets the same track speed."
            />
            <ChipPicker
              label="Right side"
              options={names}
              selected={drive.roles.right}
              onToggle={(n) => toggleRole("right", n)}
            />
          </>
        )}

        {(drive.kind === "servo_steer" || drive.kind === "single") && (
          <ChipPicker
            label="Drive motors"
            options={names}
            selected={drive.roles.throttle}
            onToggle={(n) => toggleRole("throttle", n)}
          />
        )}

        {drive.kind === "servo_steer" && (
          <>
            <SelectField
              label="Steering servo"
              value={drive.roles.steer}
              options={[{ value: "", label: "— pick one —" }].concat(
                names.map((n) => ({ value: n, label: n })),
              )}
              onChange={setSteerActuator}
            />
            <NumberField
              label="Steering gain"
              value={drive.steer_gain ?? 1}
              min={0}
              max={2}
              step={0.05}
              onChange={(v) => setDriveField("steer_gain", v)}
              hint="How much of the servo's throw a full steer command uses."
            />
            <NumberField
              label="Pivot creep"
              value={drive.min_pivot_throttle ?? 0.15}
              min={0}
              max={1}
              step={0.01}
              onChange={(v) => setDriveField("min_pivot_throttle", v)}
              hint="A steered chassis cannot turn on the spot, but the autonomy modes ask it to. This creeps forward so the steering bites. 0 disables it."
            />
            {alignMode && (
              <p class="banner warn">
                This robot is in an align mode. A steered chassis cannot pivot in
                place, so alignment relies on the pivot creep above — leave it
                above zero or the robot will steer at its target without moving.
              </p>
            )}
          </>
        )}

        <div class="doc-grid">
          <NumberField
            label="Slew rate"
            unit="/s"
            value={drive.slew_rate ?? 4}
            min={0}
            max={20}
            step={0.1}
            onChange={(v) => setDriveField("slew_rate", v)}
            hint="Max throttle change per second. 0 disables limiting."
          />
          <NumberField
            label="Arm hold"
            unit="s"
            value={drive.arm_seconds ?? 2}
            min={0}
            max={10}
            step={0.1}
            onChange={(v) => setDriveField("arm_seconds", v)}
            hint="Hold neutral this long at boot so the ESCs arm."
          />
        </div>

        <div class="actuator-list">
          {drive.actuators.map((actuator) => (
            <ActuatorCard
              key={actuator.name}
              actuator={actuator}
              mech={null}
              onRemove={() => removeDriveActuator(actuator.name)}
            />
          ))}
          <button type="button" class="btn ghost small" onClick={addDriveActuator}>
            + Motor
          </button>
        </div>
      </div>
    </section>
  );
}

export function HardwarePage() {
  const rid = targetRobot.value;
  const documents = rid ? robotDocuments.value[rid] : undefined;
  const doc = layout.value;
  const result = layoutResult.value;
  const clashes = channelConflicts.value;

  // Ask per robot, like the config next door: this crosses the radio, so it is
  // requested rather than polled — and re-requested if the answer doesn't come
  // back, because a document that loses a fragment loses all of it.
  const fetch = useRadioFetch(
    rid && `${rid}:layout`,
    !!documents?.layout_rev,
    refreshLayout,
  );

  // Drop the draft once the robot has accepted it, so the form starts showing
  // the robot's own copy again — including anything it clamped on the way in.
  useEffect(() => {
    if (result?.ok && layoutDirty.value) layoutAccepted();
  }, [documents?.layout_rev]);

  if (!rid) {
    return <p class="hint pad">No robot selected — pick one on the driving view first.</p>;
  }

  return (
    <>
      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Robot</span>
          <select
            class="field-select"
            value={rid}
            onChange={(e) =>
              configTarget.value = (e.target as HTMLSelectElement).value}
          >
            {robots.value.map((r) => (
              <option key={r.robot_id} value={r.robot_id}>{r.robot_id}</option>
            ))}
          </select>
        </div>
        <div class="settings-bar-group">
          {layoutDirty.value && (
            <button type="button" class="btn ghost small" onClick={discardLayout}>
              Discard
            </button>
          )}
          <button type="button" class="btn ghost small" onClick={refreshLayout}>
            Refresh
          </button>
          <button
            type="button"
            class="btn small primary"
            disabled={!layoutDirty.value || clashes.size > 0}
            onClick={saveLayout}
          >
            {layoutDirty.value ? "Save to robot" : "Saved"}
          </button>
        </div>
      </div>

      {clashes.size > 0 && (
        <p class="banner error">
          {clashes.size === 1 ? "Channel" : "Channels"}{" "}
          {[...clashes].join(", ")} claimed twice. Two actuators on one PWM
          channel move together and neither answers its own commands.
        </p>
      )}
      {result && !result.ok && (
        <p class="banner error">
          The robot refused this layout:
          <br />
          {result.errors.join(" · ")}
        </p>
      )}
      {result?.ok && result.restart_required && !layoutDirty.value && (
        <p class="banner warn">
          Saved. It takes effect when the robot's service restarts — actuators
          are built at start-up, and rebuilding them mid-drive would leave an
          ESC at an undefined pulse.
        </p>
      )}
      {result?.warnings?.length ? (
        <p class="banner warn">{result.warnings.join(" · ")}</p>
      ) : null}

      {!doc
        ? (
          <Waiting
            what="layout"
            robot={rid}
            fetch={fetch}
            note={
              <p class="hint">
                A build from before layouts existed answers nothing here. It is
                a two-motor tank drive on channels 0 and 1.
              </p>
            }
          />
        )
        : (
          <>
            <DrivetrainCard />

            <section class="group open">
              <div class="group-body">
                <div class="doc-label">
                  Mechanisms
                  <span class="doc-hint">
                    Everything that moves but does not drive: intakes, arms,
                    launchers.
                  </span>
                </div>
                {doc.mechanisms.map((mech) => (
                  <MechanismCard key={mech.name} mech={mech} />
                ))}
                <div class="chip-row">
                  <button
                    type="button"
                    class="btn ghost small"
                    onClick={() => addMechanism("power")}
                  >
                    + Powered mechanism
                  </button>
                  <button
                    type="button"
                    class="btn ghost small"
                    onClick={() => addMechanism("pulse")}
                  >
                    + Pulse mechanism
                  </button>
                </div>
              </div>
            </section>

            <section class="group open">
              <div class="group-body">
                <ToggleField
                  label="Enable bench testing"
                  value={jogUnlocked.value}
                  onChange={(v) => jogUnlocked.value = v}
                  hint="Unlocks the Test buttons above. Put the robot on blocks first — the wheels are not the only thing that moves."
                />
                <p class="doc-hint">
                  The robot refuses a test unless it is in teleop with no e-stop
                  latched, and stops on its own if the link drops.
                </p>
              </div>
            </section>
          </>
        )}
    </>
  );
}
