// Asking a robot for something, over a link that can lose the answer.
//
// The config, the layout, the routines and the field descriptors are all
// FETCHED rather than pushed: they are kilobytes on a radio shared with
// telemetry, so the page asks once when it opens instead of polling. "Once" was
// the problem. The reply crosses a 57600-baud link in numbered fragments and
// nothing is applied until every fragment lands (robot/comms/doc_transfer.py),
// so a single fragment lost to a congested radio doesn't degrade the answer —
// it deletes it. The tab then sat on "Fetching…" forever, with no retry, no
// error, and nothing to press.
//
// So: ask, wait, ask again a couple of times, and if the robot still hasn't
// answered SAY SO and offer the button. Three attempts over eight seconds is
// well inside the patience of someone who just opened a tab, and bounded
// because a robot that is genuinely off does not get more reachable by being
// asked thirty times.

import { useEffect, useState } from "preact/hooks";
import { conn } from "../net/ws.ts";

/** How long to wait for an answer before asking again. Comfortably longer than
 *  a document takes to cross the radio (~0.7 s for a config snapshot) plus the
 *  robot's own 5 s reassembly timeout would allow for a stalled transfer. */
const RETRY_MS = 4000;

/** Total asks, including the first. */
const MAX_ATTEMPTS = 3;

export interface RadioFetch {
  /** Asked, still waiting, more attempts to come. */
  pending: boolean;
  /** Asked as often as we're going to. Nothing came back. */
  stalled: boolean;
  /** How many times we've asked so far. */
  attempts: number;
  /** Start over — for the button the stalled state offers. */
  retry: () => void;
}

/**
 * Request `key`'s document until it arrives, or until we give up on it.
 *
 * `key` identifies what is being fetched for whom (`"rover1:layout"`), so
 * switching robots starts a fresh count rather than inheriting the last one's
 * exhausted attempts. `have` is how the caller says the answer landed; passing
 * a revision counter's truthiness is usually right, because a robot that has
 * never been given a layout still answers with the one it is running.
 */
export function useRadioFetch(
  key: string | null,
  have: boolean,
  request: () => void,
): RadioFetch {
  const [attempts, setAttempts] = useState(0);
  // Read through the signal so a socket that drops and comes back re-arms the
  // whole thing: the robot never heard the first ask.
  const live = conn.value === "live";

  useEffect(() => {
    setAttempts(0);
  }, [key, live]);

  useEffect(() => {
    if (!key || have || !live || attempts >= MAX_ATTEMPTS) return;
    // One pass through this effect is one attempt. Bumping the counter on the
    // timer is what re-runs it; the answer arriving flips `have` and the
    // cleanup cancels the pending retry before it can spend airtime.
    request();
    const timer = setTimeout(() => setAttempts((n) => n + 1), RETRY_MS);
    return () => clearTimeout(timer);
  }, [key, have, live, attempts]);

  return {
    // Nothing is in flight while the socket is down — the ask never left the
    // building — so that reads as stalled, not as waiting.
    pending: !have && live && attempts < MAX_ATTEMPTS,
    stalled: !have && (!live || attempts >= MAX_ATTEMPTS),
    attempts,
    retry: () => setAttempts(0),
  };
}
