import { selected, videoRobots } from "../net/ws.ts";

// First-person camera feed for the selected robot. The bridge serves an endless
// MJPEG stream at /video/{id}.mjpg, which an <img> renders frame by frame — no
// player, no decode code. We only mount the <img> when the fleet snapshot says
// this robot is actually streaming, so we never show a broken-image icon; when
// the feed goes stale the bridge drops it from `video` and we fall back to the
// placeholder within a few seconds.
export function FPV() {
  const id = selected.value;
  const live = id != null && videoRobots.value.includes(id);

  return (
    <section class="rail-section">
      <div class="section-title" style="margin-bottom:10px">
        <span class="eyebrow">Camera</span>
        <span
          class="eyebrow"
          style={`color:${live ? "var(--ok)" : "var(--faint)"}`}
        >
          {live ? "live" : "no feed"}
        </span>
      </div>
      <div class="fpv-box">
        {live
          ? (
            // key by id so switching robots remounts the <img> and starts a
            // fresh stream rather than reusing the previous connection.
            <img
              key={id}
              class="fpv-img"
              src={`/video/${id}.mjpg`}
              alt={`${id} camera`}
            />
          )
          : (
            <div class="fpv-empty">
              {id ? "waiting for video…" : "select a robot"}
            </div>
          )}
      </div>
    </section>
  );
}
