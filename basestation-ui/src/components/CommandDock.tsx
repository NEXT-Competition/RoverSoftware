// The commander's dock: what every rover is doing, and where an order goes.
//
// This is the SHELL for the voice/LLM layer, built ahead of it deliberately —
// the hard part of commanding a fleet by voice is not the model, it is having
// one place that answers "what is each rover doing right now" without selecting
// them one at a time. That readout is useful today, on its own, and it is the
// context an order would be issued against tomorrow.
//
// --- What it does NOT do ---
// There is no model behind the input. It says so, plainly, and the field is
// disabled rather than accepting text it would silently drop. An input that
// looks live and does nothing is worse than no input: at a competition somebody
// will talk into it while a rover is moving. Everything else on this dock is
// real telemetry off the fleet frame — no placeholder rows, no invented status.
//
// The vocabulary an order will use is already on screen: rover ids from the
// fleet, place names from the map (state/places.ts), and mode names from
// ModeControls. That is the actual reason places were worth building as a named
// table — "send rover2 to bucket A" has to resolve against something.

import { robots, selected, selectRobot } from "../net/ws.ts";
import { placeList } from "../state/places.ts";
import type { Robot } from "../net/types.ts";

/** What this rover is doing, in the operator's words rather than the wire's. */
function activity(robot: Robot): { text: string; tone: string } {
  if (robot.estop) return { text: "E-STOP LATCHED", tone: "bad" };
  if (!robot.online) return { text: "no telemetry", tone: "bad" };
  if (robot.mode === "routine") {
    const state = robot.routine?.state;
    return state
      ? { text: state, tone: "run" }
      : { text: "routine — no state", tone: "warn" };
  }
  if (robot.mode === "waypoint") return { text: "driving route", tone: "run" };
  if (robot.mode === "object_align") return { text: "aligning on target", tone: "run" };
  if (robot.mode === "shooter_align") return { text: "aligning to shoot", tone: "run" };
  return { text: "teleop", tone: "idle" };
}

function MicIcon() {
  return (
    <svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true">
      <rect x="9" y="3" width="6" height="10.5" rx="3" fill="currentColor" />
      <path
        d="M5.5 11.5a6.5 6.5 0 0 0 13 0M12 18v3"
        stroke="currentColor"
        stroke-width="1.8"
        stroke-linecap="round"
        fill="none"
      />
    </svg>
  );
}

export function CommandDock() {
  const fleet = robots.value;
  const sel = selected.value;
  const places = placeList.value;

  if (!fleet.length) return null;

  return (
    <div class="cmd-dock">
      <div class="cmd-order">
        <span class="cmd-mic" aria-hidden="true">
          <MicIcon />
        </span>
        <input
          class="cmd-input"
          type="text"
          disabled
          // Short enough to survive the dock's width at 1280 — a placeholder
          // that ellipsizes mid-example teaches nothing.
          placeholder={places.length
            ? `e.g. “send ${fleet[0].robot_id} to ${places[0].name}”`
            : "e.g. “send rover1 to bucket A” — save a place first"}
          aria-label="Fleet order (not yet available)"
        />
        {/* The honest label. This dock looks like somewhere you can talk, so it
            has to say plainly that nothing is listening yet. */}
        <span class="cmd-state" title="No command model is connected yet">
          not wired up
        </span>
      </div>

      {/* The readout that earns the dock its space today. Every rover, what it
          is doing, without selecting them one at a time. */}
      <ul class="cmd-fleet">
        {fleet.map((robot) => {
          const act = activity(robot);
          return (
            <li key={robot.robot_id}>
              <button
                class={`cmd-rover${robot.robot_id === sel ? " sel" : ""}`}
                onClick={() => selectRobot(robot.robot_id)}
              >
                <span class={`cmd-led ${act.tone}`} aria-hidden="true" />
                <span class="cmd-name">{robot.robot_id}</span>
                <span class={`cmd-act ${act.tone}`}>{act.text}</span>
              </button>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
