import { selected, selectedRobot, send } from "../net/ws.ts";

/**
 * Arm / disarm / manual fire for the shooter_align mode.
 *
 * Only rendered when the selected robot reports a `shooter` block, which the
 * robot emits only while shooter_align is the active mode — so the fire button
 * cannot be reached from teleop, and the panel disappears the moment the robot
 * leaves the mode rather than lingering with a stale arm indicator.
 *
 * Every rule about *when* it is safe to shoot (dwell, cooldown, magazine, the
 * arm latch itself) lives on the robot. This is a remote control, not a second
 * opinion: it shows what the robot reports and forwards button presses.
 */
export function ShooterControls() {
  const rid = selected.value;
  const s = selectedRobot.value?.shooter;
  if (!rid || !s) return null;

  const cooling = s.cool > 0;
  return (
    <div>
      <div class="section-title">
        <span class="eyebrow">Shooter</span>
        <span class="eyebrow" style={`color:var(--${s.armed ? "danger" : "muted"})`}>
          {s.armed ? "ARMED" : "safe"}
        </span>
      </div>
      <div class="btn-row" style="margin-top:8px">
        <button
          class={`btn${s.armed ? " danger active" : ""}`}
          onClick={() =>
            send({ action: s.armed ? "disarm_shooter" : "arm_shooter", robot_id: rid })}
        >
          {s.armed ? "Disarm" : "Arm"}
        </button>
        <button
          class="btn"
          disabled={!s.armed || cooling}
          onClick={() => send({ action: "fire", robot_id: rid })}
        >
          {cooling ? `${s.cool.toFixed(1)}s` : "Fire"}
        </button>
      </div>
      <p class="hint">
        {!s.armed
          ? "Disarmed. Arming is dropped on e-stop and on leaving this mode."
          : s.ready
          ? "On target — holding still, firing when the aim settles."
          : `Armed, searching. ${s.shots} fired.`}
      </p>
    </div>
  );
}
