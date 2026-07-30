// The empty state of a tab whose contents come off the radio.
//
// Shared by Tuning, Hardware and Routines because they fail the same way and
// should say so the same way. The old copy ("Fetching rover1's layout… if it
// stays empty, the robot is offline or…") was a guess printed once and then
// left on screen forever, which is indistinguishable from a page that is still
// working. This one knows whether it is still trying, and when it has stopped
// it says that plainly and gives you the button.

import type { RadioFetch } from "../../state/fetch.ts";

export function Waiting(
  { what, robot, fetch, note }: {
    /** What is being fetched, as it reads mid-sentence: "layout", "routines". */
    what: string;
    robot: string;
    fetch: RadioFetch;
    /** Anything specific to this tab worth saying once we've given up. */
    note?: preact.ComponentChildren;
  },
) {
  if (fetch.pending) {
    return (
      <p class="hint pad">
        Asking {robot} for its {what}…
        {/* `attempts` counts the ones already made, so the one in flight is
            the next number up. Only shown once there has been a retry —
            "attempt 1" on a first load would read as a warning. */}
        {fetch.attempts >= 1 && ` (attempt ${fetch.attempts + 1})`}
        <br />
        This crosses the radio, so it takes a moment.
      </p>
    );
  }

  return (
    <div class="waiting-stalled">
      <p class="banner warn">
        No answer from {robot} — its {what} never arrived.
        {" "}Usually that means the robot is off, out of radio range, or running
        a build from before this tab existed.
      </p>
      {note}
      <button type="button" class="btn small primary" onClick={fetch.retry}>
        Ask again
      </button>
    </div>
  );
}
