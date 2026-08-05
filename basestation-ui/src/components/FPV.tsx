import { selected, videoRobots } from "../net/ws.ts";

// First-person camera feed for the selected robot. The bridge serves an endless
// MJPEG stream at /video/{id}.mjpg, which an <img> renders frame by frame — no
// player, no decode code.
//
// Mounting the <img> is not just how the feed is DISPLAYED, it is how the feed
// is REQUESTED: the bridge counts open streams per robot and tells that rover to
// switch its camera on, then off again when the last one closes
// (basestation/app.py::push_fpv_demand). Every rover used to stream whether or
// not anyone was looking, which on a shared field network meant the unwatched
// ones were crowding out the rover actually being driven.
//
// So this mounts on SELECTION rather than on `videoRobots`. Waiting for the
// robot to appear in `video` would deadlock the demand it is meant to create:
// no <img>, no viewer, no camera, and therefore never any video. `videoRobots`
// is now only what the live/no-feed label reads — frames actually arriving.
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
        {id
          ? (
            <>
              {/* key by id so switching robots tears the old stream down and
                  opens a fresh one — which is also what drops the previous
                  rover's viewer count to zero and stops its camera. Kept
                  mounted but hidden until frames arrive, so the request stays
                  open (that is the demand) without showing a broken image. */}
              <img
                key={id}
                class="fpv-img"
                src={`/video/${id}.mjpg`}
                alt={`${id} camera`}
                style={live ? undefined : "display:none"}
              />
              {live ? null : <div class="fpv-empty">waiting for video…</div>}
            </>
          )
          : <div class="fpv-empty">select a robot</div>}
      </div>
    </section>
  );
}
