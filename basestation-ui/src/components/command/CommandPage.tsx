// The command screen: hold the key, say the order, watch what it became.
//
// A full view rather than a bigger dock, because voice needs three things the
// dock's strip has no room for and cannot do without:
//
//   1. **What was heard.** The single most common failure is the microphone,
//      not the model. Showing the transcript separately from the intent is what
//      lets an operator tell "it misheard me" from "it misunderstood me" in one
//      glance, which decides whether they repeat themselves or rephrase.
//   2. **What it will do, before it does it.** Every gated order shows as a
//      card with the planned command spelled out. Nothing dangerous happens
//      because a 4B model was confident.
//   3. **A log.** With voice, an AI on MCP, and two tablets all able to command
//      the same fleet, "why did rover2 just turn" has to have an answer.
//
// The layout is a two-column plot table at desk width and a single stack on the
// kiosk. The fleet strip stays visible in both, because an order is always
// issued *against* what the rovers are currently doing.

import { useEffect, useRef, useState } from "preact/hooks";
import { robots, selected, selectRobot, send, videoRobots } from "../../net/ws.ts";
import { sendAudio } from "../../net/ws.ts";
import {
  beginUtterance,
  endUtterance,
  micError,
  micLevel,
  micState,
  setAudioSink,
  startMic,
  stopMic,
} from "../../net/audio.ts";
import {
  cameraFocus,
  commandEnabled,
  heard,
  heardConf,
  historyNewestFirst,
  modelName,
  modelNote,
  modelOk,
  outcomeTone,
  pending,
  stt,
  sttError,
  thinking,
  vocabulary,
} from "../../state/command.ts";
import { showView } from "../../state/view.ts";
import type { CommandOutcome, PendingCommand } from "../../net/types.ts";

/** Hold-to-talk key. Space is the obvious one and it is not otherwise bound on
 *  this view (the drive pad is not mounted here). */
const PTT_KEY = "Space";

function MicGlyph({ size = 26 }: { size?: number }) {
  return (
    <svg viewBox="0 0 24 24" width={size} height={size} aria-hidden="true">
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

function BackGlyph() {
  return (
    <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true">
      <path
        d="M15 5 8 12l7 7"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
        fill="none"
      />
    </svg>
  );
}

/** Where the operator talks. The whole left column exists to serve this button. */
function Talk() {
  const state = micState.value;
  const level = micLevel.value;
  const recording = state === "recording";
  const ready = state === "ready" || recording;

  // Held state lives in a ref as well as the signal: the keyup listener is
  // registered once, and a stale closure over `recording` would strand the
  // recorder on if the key went up during a re-render.
  const held = useRef(false);

  const down = () => {
    if (held.current) return;
    if (!beginUtterance()) return;
    held.current = true;
    send({ action: "mic", on: true });
  };

  const up = () => {
    if (!held.current) return;
    held.current = false;
    endUtterance();
    send({ action: "mic", on: false });
  };

  useEffect(() => {
    setAudioSink(sendAudio);
    void startMic();
    const onKeyDown = (e: KeyboardEvent) => {
      // Not while typing an order into the text field beside it.
      if (e.code !== PTT_KEY || e.repeat) return;
      const el = document.activeElement;
      if (el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement) return;
      e.preventDefault();
      down();
    };
    const onKeyUp = (e: KeyboardEvent) => {
      if (e.code !== PTT_KEY) return;
      e.preventDefault();
      up();
    };
    // Releasing outside the window (alt-tab mid-sentence) must still end the
    // utterance, or the buffer stays open until the socket's cap trims it.
    globalThis.addEventListener("keydown", onKeyDown);
    globalThis.addEventListener("keyup", onKeyUp);
    globalThis.addEventListener("blur", up);
    return () => {
      up();
      globalThis.removeEventListener("keydown", onKeyDown);
      globalThis.removeEventListener("keyup", onKeyUp);
      globalThis.removeEventListener("blur", up);
      setAudioSink(null);
      // The stream is released when the view unmounts rather than held for the
      // session: a base station with a hot microphone it isn't using is not
      // something to leave running next to a team's conversation.
      stopMic();
    };
  }, []);

  const label = recording
    ? "Listening — release to send"
    : ready
    ? "Hold to talk"
    : state === "starting"
    ? "Opening the microphone…"
    : state === "denied"
    ? "Microphone blocked"
    : state === "error"
    ? "No microphone"
    : "Microphone off";

  return (
    <div class="talk">
      <button
        type="button"
        class={`talk-btn${recording ? " live" : ""}`}
        disabled={!ready}
        aria-pressed={recording}
        aria-label={label}
        onPointerDown={(e) => {
          // Capture, so a thumb that slides off the button still delivers the
          // release here. Without it, dragging off strands the recorder on.
          (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
          down();
        }}
        onPointerUp={up}
        onPointerCancel={up}
        onContextMenu={(e) => e.preventDefault()}
      >
        {/* The ring is the input level, not a decoration: a muted OS mic and a
            quiet room look identical until something moves. */}
        <span
          class="talk-ring"
          style={`transform:scale(${recording ? 1 + Math.min(level, 1) * 0.35 : 1})`}
          aria-hidden="true"
        />
        <MicGlyph size={34} />
      </button>
      <div class="talk-label">{label}</div>
      <div class="talk-hint">
        {ready
          ? <>hold <kbd>Space</kbd> or the button</>
          // "starting" is usually the browser's permission prompt sitting
          // there waiting to be answered — invisible on a kiosk if it opened
          // behind the window. Saying so beats a spinner that reads as a hang,
          // which is exactly what this looked like the first time it ran.
          : state === "starting"
          ? "allow microphone access if your browser asks"
          : micError.value ?? " "}
      </div>
      {state === "denied" && (
        <button type="button" class="btn ghost" onClick={() => void startMic()}>
          Retry microphone
        </button>
      )}
    </div>
  );
}

/** Type an order. Always available — it is the fallback when the mic is blocked,
 *  when Whisper isn't installed, and when the pit is too loud to talk in. */
function TypeOrder() {
  const [text, setText] = useState("");
  const submit = (e: Event) => {
    e.preventDefault();
    const value = text.trim();
    if (!value) return;
    send({ action: "command_text", text: value, source: "text" });
    setText("");
  };
  const vocab = vocabulary.value;
  const example = vocab.places.length && vocab.robots.length
    ? `send ${vocab.robots[0]} to ${vocab.places[0].name}`
    : vocab.robots.length
    ? `${vocab.robots[0]} align to the bucket`
    : "stop";

  return (
    <form class="order-form" onSubmit={submit}>
      <input
        class="order-input"
        type="text"
        value={text}
        placeholder={`e.g. “${example}”`}
        aria-label="Type an order"
        onInput={(e) => setText((e.target as HTMLInputElement).value)}
      />
      <button type="submit" class="btn" disabled={!text.trim()}>Send</button>
    </form>
  );
}

/** A gated command waiting for a human. This is the safety mechanism made
 *  visible, so it is the loudest thing on the screen after the e-stop. */
function ConfirmCard({ item }: { item: PendingCommand }) {
  return (
    <div class="confirm-card">
      <div class="confirm-head">
        <span class="eyebrow">Needs your approval</span>
        <span class="confirm-src mono">
          {item.source} · {Math.max(0, Math.round(item.expires_in))}s
        </span>
      </div>
      <div class="confirm-say">{item.say}</div>
      <div class="btn-row">
        <button
          type="button"
          class="btn danger"
          onClick={() => send({ action: "command_confirm", id: item.id })}
        >
          Do it
        </button>
        <button
          type="button"
          class="btn ghost"
          onClick={() => send({ action: "command_cancel", id: item.id })}
        >
          Cancel
        </button>
      </div>
    </div>
  );
}

/** What the model heard, while it is still deciding what it meant. */
function Heard() {
  const text = heard.value;
  const conf = heardConf.value;
  const failed = sttError.value;
  // Nothing was transcribed. This never reaches the order log — no order was
  // ever recognised — so if it isn't said here it isn't said anywhere, and
  // holding the button then watching nothing happen is how an operator decides
  // the feature is broken.
  if (!text && failed) {
    return (
      <div class="heard failed">
        <span class="eyebrow">Not heard</span>
        <span class="heard-text">{failed}</span>
      </div>
    );
  }
  if (!text) return null;
  return (
    <div class={`heard${thinking.value ? " thinking" : ""}`}>
      <span class="eyebrow">Heard</span>
      <span class="heard-text">“{text}”</span>
      {conf != null && (
        <span class="heard-conf mono" title="Speech recogniser confidence">
          {Math.round(conf * 100)}%
        </span>
      )}
      {thinking.value && <span class="heard-dots" aria-label="thinking" />}
    </div>
  );
}

/** One row of the log. Shows the sentence, what it became, and who said it. */
function LogRow({ row }: { row: CommandOutcome }) {
  const tone = outcomeTone(row.status);
  return (
    <li class={`log-row ${tone}`}>
      <span class={`log-led ${tone}`} aria-hidden="true" />
      <div class="log-body">
        {row.transcript && <div class="log-said">“{row.transcript}”</div>}
        <div class="log-say">{row.say}</div>
        <div class="log-meta mono">
          {row.source}
          {row.intent ? ` · ${row.intent}` : ""}
          {row.route ? ` · ${row.route}` : ""}
          {row.ms != null ? ` · ${row.ms}ms` : ""}
        </div>
      </div>
    </li>
  );
}

/** Every rover and what it is doing — an order is always given against this. */
function FleetStrip() {
  const fleet = robots.value;
  const sel = selected.value;
  if (!fleet.length) {
    return <div class="cmd-empty">No rovers are reporting telemetry.</div>;
  }
  return (
    <ul class="cmd-fleet wide">
      {fleet.map((robot) => {
        let text = "teleop";
        let tone = "idle";
        if (robot.estop) [text, tone] = ["E-STOP LATCHED", "bad"];
        else if (!robot.online) [text, tone] = ["no telemetry", "bad"];
        else if (robot.mode === "routine") {
          [text, tone] = [robot.routine?.state ?? "routine", "run"];
        } else if (robot.mode === "waypoint") [text, tone] = ["driving route", "run"];
        else if (robot.mode === "object_align") {
          [text, tone] = [
            robot.vision?.label ? `aligning on ${robot.vision.label}` : "aligning",
            "run",
          ];
        } else if (robot.mode === "shooter_align") [text, tone] = ["aiming", "run"];
        return (
          <li key={robot.robot_id}>
            <button
              class={`cmd-rover${robot.robot_id === sel ? " sel" : ""}`}
              onClick={() => selectRobot(robot.robot_id)}
            >
              <span class={`cmd-led ${tone}`} aria-hidden="true" />
              <span class="cmd-name">{robot.robot_id}</span>
              <span class={`cmd-act ${tone}`}>{text}</span>
            </button>
          </li>
        );
      })}
    </ul>
  );
}

/** The camera, here as well as on the map view — "pull up the FPV" is one of
 *  the orders this screen exists to serve, and switching views to see the
 *  result would defeat it. */
function CommandFPV() {
  const id = cameraFocus.value ?? selected.value;
  const live = id != null && videoRobots.value.includes(id);
  return (
    <div class="cmd-fpv">
      <div class="section-title">
        <span class="eyebrow">Camera</span>
        <span class="eyebrow" style={`color:${live ? "var(--ok)" : "var(--faint)"}`}>
          {id ?? "—"} · {live ? "live" : "no feed"}
        </span>
      </div>
      <div class="fpv-box">
        {live
          ? <img key={id} class="fpv-img" src={`/video/${id}.mjpg`} alt={`${id} camera`} />
          : <div class="fpv-empty">{id ? "no video from this rover" : "select a rover"}</div>}
      </div>
    </div>
  );
}

/** What is actually listening, said plainly. The original CommandDock's comment
 *  argued that an input which looks live but does nothing is worse than none;
 *  the same rule applies to a model that is not running, so this never claims
 *  more than it has. */
function Readiness() {
  const speech = stt.value;
  const model = modelOk.value;
  return (
    <div class="ready-row">
      <span class={`pill ${speech.available ? "ok" : "warn"}`} title={speech.error ?? ""}>
        <span class={`led ${speech.available ? "ok" : "warn"}`} />
        {speech.available ? `speech · ${speech.model}` : "speech unavailable"}
      </span>
      <span class={`pill ${model ? "ok" : "warn"}`} title={modelNote.value}>
        <span class={`led ${model ? "ok" : "warn"}`} />
        {model ? `model · ${modelName.value}` : "no model"}
      </span>
      {!model && (
        <button
          type="button"
          class="btn ghost small"
          onClick={() => send({ action: "command_check" })}
        >
          Re-check
        </button>
      )}
      <button
        type="button"
        class={`btn ghost small${commandEnabled.value ? " active" : ""}`}
        title="Turn the language model off. Stop, rover names, modes and the camera still work."
        onClick={() => send({ action: "command_enable", on: !commandEnabled.value })}
      >
        {commandEnabled.value ? "Model on" : "Model off"}
      </button>
    </div>
  );
}

/** The words that currently mean something. Not a help page — a live readout:
 *  these are exactly the names the recogniser will accept this second. */
function Sayable() {
  const vocab = vocabulary.value;
  const routines = vocab.routines[vocab.selected ?? ""] ?? [];
  return (
    <div class="sayable">
      <div class="section-title">
        <span class="eyebrow">You can say</span>
      </div>
      <dl class="say-list">
        <dt>rovers</dt>
        <dd class="mono">{vocab.robots.join(" · ") || "—"}</dd>
        <dt>places</dt>
        <dd class="mono">{vocab.places.map((p) => p.name).join(" · ") || "none saved"}</dd>
        <dt>modes</dt>
        <dd class="mono">{vocab.modes.map((m) => m.replace(/_/g, " ")).join(" · ")}</dd>
        {vocab.labels.length > 0 && (
          <>
            <dt>objects</dt>
            <dd class="mono">{vocab.labels.join(" · ")}</dd>
          </>
        )}
        {/* Scoped to the rover that would be commanded, because that is the
            list the recogniser will actually match against — a routine loaded
            on rover2 is not sayable while rover1 is selected. */}
        {routines.length > 0 && (
          <>
            <dt>routines</dt>
            <dd class="mono">{routines.map((r) => r.name).join(" · ")}</dd>
          </>
        )}
      </dl>
      <p class="say-note">
        “stop” always works, even with no model running — it never goes through one.
      </p>
    </div>
  );
}

export function CommandPage() {
  const queue = pending.value;
  const log = historyNewestFirst.value;

  return (
    <div class="command-page">
      <header class="cmd-top panel">
        <button
          type="button"
          class="icon-btn"
          aria-label="Back to the map"
          title="Back to the map"
          onClick={() => showView("ops")}
        >
          <BackGlyph />
        </button>
        <span class="name">
          Command
          <small>speak or type an order</small>
        </span>
        <Readiness />
      </header>

      <div class="cmd-grid">
        <section class="cmd-col-main panel">
          <Talk />
          <TypeOrder />
          <Heard />

          {/* Directly under the input, not in a corner: this is the one thing
              on the screen that is waiting for the operator. */}
          {queue.length > 0 && (
            <div class="confirm-stack">
              {queue.map((item) => <ConfirmCard key={item.id} item={item} />)}
            </div>
          )}

          <div class="section-title" style="margin-top:4px">
            <span class="eyebrow">Fleet</span>
            <span class="eyebrow">{selected.value ?? "—"}</span>
          </div>
          <FleetStrip />
        </section>

        <section class="cmd-col-side">
          <div class="panel cmd-side-panel">
            <CommandFPV />
          </div>
          <div class="panel cmd-side-panel">
            <Sayable />
          </div>
        </section>
      </div>

      <section class="cmd-log panel">
        <div class="section-title">
          <span class="eyebrow">Orders</span>
          <span class="eyebrow">{log.length ? `${log.length}` : "none yet"}</span>
        </div>
        {log.length === 0
          ? (
            <div class="cmd-empty">
              Nothing has been commanded yet. Hold <kbd>Space</kbd> and say a
              rover’s name.
            </div>
          )
          : (
            <ul class="log-list">
              {log.map((row, i) => <LogRow key={`${row.at}-${i}`} row={row} />)}
            </ul>
          )}
      </section>
    </div>
  );
}
