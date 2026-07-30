import { selected, selectedRobot, send } from "../net/ws.ts";

// Always-visible safety control. Fires against the selected robot; queued
// through the WS proxy so it lands even mid-reconnect.
export function EstopBar() {
  const rid = selected.value;
  const robot = selectedRobot.value;
  const armed = robot?.estop ?? false;

  return (
    <div class="estop-dock">
      {armed && (
        <button
          type="button"
          class="btn estop-clear"
          disabled={!rid}
          onClick={() => rid && send({ action: "clear_estop", robot_id: rid })}
        >
          Clear
        </button>
      )}
      {/* The face stays the same two words in every state — an operator
          reaching for this is not reading it. The latch is said in the
          accessible name instead, because on screen it is carried by the pulse
          and by the Clear button appearing, and neither of those is anything a
          screen reader can report. */}
      <button
        type="button"
        class={`estop${armed ? " armed" : ""}`}
        disabled={!rid}
        aria-label={armed
          ? `Emergency stop, latched on ${rid}. Press to stop again, or use Clear to release.`
          : rid
          ? `Emergency stop ${rid}`
          : "Emergency stop"}
        onClick={() => rid && send({ action: "estop", robot_id: rid })}
      >
        E‑STOP
      </button>
    </div>
  );
}
