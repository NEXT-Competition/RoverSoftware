// The command layer's client state: what was heard, what it became, and what
// is waiting for somebody to approve it.
//
// Mirrors basestation/command/executor.py. The base station owns all of it —
// this file holds no logic about what a command *means*, only about how to
// render what came back. That split is the point: a second implementation of
// "is this rover allowed to fire" living in the browser would be a second
// implementation to get wrong, and the browser is the half that can be stale.
//
// --- Actions that never leave the base station ---
// An outcome's `actions` may contain entries with a `ui` key. Those are the
// parts of an order that move *this screen* — showing a camera, switching
// views — and they are here rather than in the executor because the base
// station has no screen. "Pull up rover2's camera" is one sentence that both
// selects the rover over the radio and points the operator at its feed;
// applyUiActions is the half that does the pointing.

import { batch, computed, signal } from "@preact/signals";
import type {
  CommandAction,
  CommandMessage,
  CommandOutcome,
  CommandVocabulary,
  PendingCommand,
  SttStatus,
} from "../net/types.ts";
import { selected } from "../net/ws.ts";

const EMPTY_VOCAB: CommandVocabulary = {
  robots: [],
  online: [],
  selected: null,
  places: [],
  routines: {},
  labels: [],
  modes: [],
};

/** Is the language model switched on? The keyword fast path runs regardless. */
export const commandEnabled = signal(true);
/** Is a model server actually answering? */
export const modelOk = signal(false);
export const modelName = signal<string | null>(null);
/** The loaded model's id, or the reason it couldn't be reached. Shown verbatim:
 *  "start LM Studio and load a model" is an instruction, not an error code. */
export const modelNote = signal("checking…");
export const modelUrl = signal("");

export const pending = signal<PendingCommand[]>([]);
export const history = signal<CommandOutcome[]>([]);
export const vocabulary = signal<CommandVocabulary>(EMPTY_VOCAB);
export const stt = signal<SttStatus>({
  available: false,
  model: "",
  error: null,
  sample_rate: 16000,
});

/** The last transcript, shown while the model is still classifying it. Cleared
 *  when the outcome lands, because by then the history row says it better. */
export const heard = signal<string | null>(null);
export const heardConf = signal<number | null>(null);
/** True between a transcript arriving and its outcome — the "thinking" state.
 *  Worth its own signal: without it, a slow model is indistinguishable from a
 *  command that was silently dropped. */
export const thinking = signal(false);

/** Why the last utterance produced no transcript.
 *
 *  Its own signal because it is not an *order's* outcome — nothing was ever
 *  recognised, so it never reaches the log. Without it, holding the button and
 *  speaking with no speech model installed does visibly nothing at all, which
 *  is the single worst thing a voice interface can do. */
export const sttError = signal<string | null>(null);

/** Which robot's camera an order asked to see, if any. Consumed by the command
 *  view's feed panel; null means "whichever rover is selected". */
export const cameraFocus = signal<string | null>(null);

/** Set by app.tsx so a spoken "open settings" can actually switch views. Wired
 *  as a callback rather than importing state/view.ts directly, because view.ts
 *  imports from net/input.ts and the cycle breaks Vite's module graph. */
let viewSwitcher: ((view: string) => void) | null = null;
export function setViewSwitcher(fn: (view: string) => void): void {
  viewSwitcher = fn;
}

/** The most recent outcome, whatever it was. */
export const lastOutcome = computed<CommandOutcome | null>(() => {
  const rows = history.value;
  return rows.length ? rows[rows.length - 1] : null;
});

/** Newest first, which is the order a log is read in. */
export const historyNewestFirst = computed(() => [...history.value].reverse());

/** True when anything at all is available to command with — a model, or just
 *  the keyword path. Only false while the very first frame is in flight. */
export const commandReady = computed(() => vocabulary.value.robots.length > 0);

export function applyCommandFrame(msg: CommandMessage): void {
  batch(() => {
    commandEnabled.value = msg.enabled;
    modelOk.value = msg.model_ok;
    modelName.value = msg.model ?? null;
    modelNote.value = msg.model_note ?? "";
    modelUrl.value = msg.base_url ?? "";
    pending.value = msg.pending ?? [];
    history.value = msg.history ?? [];
    vocabulary.value = msg.vocabulary ?? EMPTY_VOCAB;
    stt.value = msg.stt ?? stt.value;

    // A transcript with no outcome yet: the mic worked, the model is thinking.
    if (typeof msg.heard === "string") {
      heard.value = msg.heard || null;
      heardConf.value = msg.heard_conf ?? null;
      thinking.value = Boolean(msg.heard);
      sttError.value = null;
      // An empty transcript is not an error — it is a button held in silence,
      // or Whisper declining to guess at a quarter-second of noise. Say so
      // rather than leaving the operator wondering if the mic is dead.
      if (!msg.heard) sttError.value = "I didn't catch anything — hold the button while you speak.";
    }
    if (msg.stt_error) {
      heard.value = null;
      thinking.value = false;
      sttError.value = msg.stt_error;
    }
    if (msg.outcome) {
      thinking.value = false;
      applyUiActions(msg.outcome);
    }
  });
}

/**
 * Carry out the parts of an order that only move this screen.
 *
 * Deliberately narrow: it reads a whitelist of `ui` keys and ignores anything
 * else. The executor already refuses to plan an action outside its registry,
 * so this is belt and braces — but this is the one place server-supplied data
 * turns into navigation, and a switch statement is cheap.
 */
function applyUiActions(outcome: CommandOutcome): void {
  if (outcome.status !== "done") return;
  for (const step of outcome.actions ?? []) {
    const kind = (step as CommandAction).ui;
    if (kind === "select" && typeof step.robot_id === "string") {
      // The dashboard owns its selection locally so a tap feels instant, and
      // adopts the server's only on the first frame (net/ws.ts). So a spoken
      // "rover three" has to move it here, or the base station is calling one
      // rover while the screen highlights another — and the sticks follow the
      // base station.
      selected.value = step.robot_id;
    } else if (kind === "camera") {
      cameraFocus.value = (step.robot_id as string) ?? selected.value;
    } else if (kind === "view" && typeof step.view === "string") {
      viewSwitcher?.(step.view);
    }
  }
}

/** How an outcome should be coloured. Follows DESIGN.md's rule strictly:
 *  amber is *a plan* (something drawn, not yet done), which is exactly what a
 *  command waiting on a human is — never "wrong". Red is refused or failed. */
export function outcomeTone(status: CommandOutcome["status"]): string {
  if (status === "done") return "ok";
  if (status === "pending") return "plan";
  if (status === "unheard") return "quiet";
  return "bad";
}
