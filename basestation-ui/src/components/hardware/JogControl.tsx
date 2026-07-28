// Hold-to-run bench test for one mechanism.
//
// This is a new way to make hardware move, so it carries the same discipline as
// the thing it resembles. The robot refuses a jog unless it is in teleop with
// no e-stop latched, and every jog expires after 0.4 s — so this control
// REPEATS while held and simply stops if the link drops mid-press. Releasing,
// leaving the page, or the tab losing focus all send a zero, mirroring the
// safety stops the drive pad already does on blur and visibility change.

import { useEffect, useRef } from "preact/hooks";
import { jog, jogUnlocked } from "../../state/hardware.ts";

const REPEAT_MS = 150; // comfortably inside the robot's 0.4 s jog expiry

export function JogControl(
  { mech, actuator, power = 0.3 }: {
    mech: string;
    actuator?: string;
    power?: number;
  },
) {
  const timer = useRef<ReturnType<typeof setInterval>>();

  const stop = () => {
    clearInterval(timer.current);
    timer.current = undefined;
    jog(mech, 0, actuator);
  };

  const start = (direction: 1 | -1) => {
    if (!jogUnlocked.value) return;
    clearInterval(timer.current);
    jog(mech, direction * power, actuator);
    timer.current = setInterval(() => jog(mech, direction * power, actuator), REPEAT_MS);
  };

  // A jog that outlives the component, the tab, or the window is a motor
  // nobody is watching. Same rule as the drive pad's blur handler.
  useEffect(() => {
    const halt = () => timer.current !== undefined && stop();
    globalThis.addEventListener("blur", halt);
    document.addEventListener("visibilitychange", halt);
    return () => {
      globalThis.removeEventListener("blur", halt);
      document.removeEventListener("visibilitychange", halt);
      if (timer.current !== undefined) stop();
    };
  }, [mech, actuator]);

  const disabled = !jogUnlocked.value;
  return (
    <div class="jog-row">
      <span class="doc-label">Test</span>
      {([-1, 1] as const).map((direction) => (
        <button
          key={direction}
          type="button"
          class="btn small jog"
          disabled={disabled}
          onPointerDown={() => start(direction)}
          onPointerUp={stop}
          onPointerLeave={stop}
          onPointerCancel={stop}
        >
          {direction > 0 ? "Forward ▸" : "◂ Reverse"}
        </button>
      ))}
      {disabled && <span class="doc-hint">Unlock testing above to use this.</span>}
    </div>
  );
}
