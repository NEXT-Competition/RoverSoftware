// The commander's dock: what every rover is doing, and where an order goes.
//
// This began as the SHELL for the voice/LLM layer, built ahead of it
// deliberately — the hard part of commanding a fleet by voice is not the model,
// it is having one place that answers "what is each rover doing right now"
// without selecting them one at a time. That readout is the context an order is
// issued against, and it was worth having on its own.
//
// The model is here now (basestation/command/), so the input is live. What it
// does NOT do is grow into the command screen: an order given by voice needs
// the transcript, the parsed intent and the approval card beside it, and this
// strip has room for none of those. Typing here sends; talking happens one tap
// away, on the view this dock's mic button opens.
//
// The old comment on this file argued that an input which looks live and does
// nothing is worse than no input, because at a competition somebody will talk
// into it while a rover is moving. That rule still holds and is why the mic
// button navigates rather than pretending to listen — the microphone lives on
// the command view, where what it heard can actually be shown.
//
// The vocabulary an order uses is already on screen: rover ids from the fleet,
// place names from the map (state/places.ts), and mode names from ModeControls.
// That is the actual reason places were worth building as a named table —
// "send rover2 to bucket A" has to resolve against something.

import { useState } from "preact/hooks";
import { robots, selected, selectRobot, send } from "../net/ws.ts";
import { placeList } from "../state/places.ts";
import { lastOutcome, outcomeTone, pending } from "../state/command.ts";
import { showView } from "../state/view.ts";
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
  const [text, setText] = useState("");
  const outcome = lastOutcome.value;
  const waiting = pending.value.length;

  if (!fleet.length) return null;

  const submit = (e: Event) => {
    e.preventDefault();
    const value = text.trim();
    if (!value) return;
    send({ action: "command_text", text: value, source: "text" });
    setText("");
  };

  return (
    <div class="cmd-dock">
      <form class="cmd-order" onSubmit={submit}>
        {/* Opens the command view rather than recording here. The microphone
            belongs where the transcript can be shown — see the file header. */}
        <button
          type="button"
          class="cmd-mic"
          title="Open the command screen and talk"
          aria-label="Open the command screen and talk"
          onClick={() => showView("command")}
        >
          <MicIcon />
        </button>
        <input
          class="cmd-input"
          type="text"
          value={text}
          // Short enough to survive the dock's width at 1280 — a placeholder
          // that ellipsizes mid-example teaches nothing.
          placeholder={places.length
            ? `e.g. “send ${fleet[0].robot_id} to ${places[0].name}”`
            : `e.g. “${fleet[0].robot_id} align to the bucket”`}
          aria-label="Fleet order"
          onInput={(e) => setText((e.target as HTMLInputElement).value)}
        />
        {/* What came of the last order, in the operator's words. A command
            given from here has to report back here — the command view may not
            be open, and an order that vanished silently is one nobody trusts. */}
        {waiting > 0
          ? (
            <button
              type="button"
              class="cmd-state plan"
              title="A command is waiting for your approval"
              onClick={() => showView("command")}
            >
              {waiting} to approve
            </button>
          )
          : outcome
          ? (
            <span class={`cmd-state ${outcomeTone(outcome.status)}`} title={outcome.say}>
              {outcome.say}
            </span>
          )
          : <span class="cmd-state">ready</span>}
      </form>

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
