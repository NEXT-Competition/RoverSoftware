import { useEffect, useState } from "preact/hooks";
import { selected, videoRobots } from "../net/ws.ts";

// Where the scope overlay's state lives between sessions. Not in the settings
// file with everything else: these lines are one operator's aiming reference on
// one screen, not a property of the fleet, and pushing them through the settings
// frame would sync them onto every other browser watching the same rovers.
const FPV_STORAGE_KEY = "rs.fpv.guide_state";

type GuideState = {
  showGuides: boolean;
  lockSliders: boolean;
  h1: number;
  h2: number;
  v1: number;
  v2: number;
};

const GUIDE_DEFAULTS: GuideState = {
  showGuides: false,
  lockSliders: false,
  h1: 25,
  h2: 75,
  v1: 25,
  v2: 75,
};

// Read field by field rather than trusting the parse: this is JSON from a
// browser store that some earlier version wrote, so a missing or wrong-typed
// key is expected rather than exceptional. A bad value falls back to its
// default instead of putting NaN into a `top:` and losing the whole overlay.
function loadGuideState(): GuideState {
  try {
    const raw = localStorage.getItem(FPV_STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      if (parsed && typeof parsed === "object") {
        return {
          showGuides: parsed.showGuides === true,
          lockSliders: parsed.lockSliders === true,
          h1: typeof parsed.h1 === "number" ? parsed.h1 : GUIDE_DEFAULTS.h1,
          h2: typeof parsed.h2 === "number" ? parsed.h2 : GUIDE_DEFAULTS.h2,
          v1: typeof parsed.v1 === "number" ? parsed.v1 : GUIDE_DEFAULTS.v1,
          v2: typeof parsed.v2 === "number" ? parsed.v2 : GUIDE_DEFAULTS.v2,
        };
      }
    }
  } catch {
    // Storage can be unavailable outright (private mode, a blocked origin). A
    // console with no saved guide lines is fine; one that fails to render
    // because it could not read them is not.
  }
  return { ...GUIDE_DEFAULTS };
}

function saveGuideState(state: GuideState): void {
  try {
    localStorage.setItem(FPV_STORAGE_KEY, JSON.stringify(state));
  } catch {
    /* see loadGuideState: never worth failing the view over */
  }
}

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
  // One object, loaded lazily. Six separate pieces of state would each want
  // their own initialiser, and localStorage would then be read on every render
  // of a component that re-renders on every fleet frame.
  const [guides, setGuides] = useState<GuideState>(loadGuideState);
  const { showGuides, lockSliders, h1, h2, v1, v2 } = guides;
  const set = (patch: Partial<GuideState>) =>
    setGuides((prev) => ({ ...prev, ...patch }));

  useEffect(() => {
    saveGuideState(guides);
  }, [guides]);

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

        {/* Aiming reference, drawn OVER the feed rather than into it: the
            stream is an <img> of somebody else's JPEG, and the lines have to
            stay put while the frames behind them change. */}
        {showGuides && (
          <>
            <div
              class="fpv-guide-line fpv-guide-horizontal"
              style={`top:${h1}%`}
            />
            <div
              class="fpv-guide-line fpv-guide-horizontal"
              style={`top:${h2}%`}
            />
            <div
              class="fpv-guide-line fpv-guide-vertical"
              style={`left:${v1}%`}
            />
            <div
              class="fpv-guide-line fpv-guide-vertical"
              style={`left:${v2}%`}
            />
          </>
        )}
      </div>

      <div class="fpv-guide-panel">
        <div class="fpv-guide-switches">
          <label class="fpv-scope-switch">
            <input
              type="checkbox"
              checked={showGuides}
              onChange={(event) =>
                set({ showGuides: (event.target as HTMLInputElement).checked })}
            />
            <span class="fpv-scope-track" />
            <span class="fpv-scope-label">Scope</span>
          </label>
          {/* Reads as "Sliders", stores as "locked", so an operator who has
              placed the lines can put the controls away and not knock one with
              a thumb mid-match. */}
          <label class="fpv-scope-switch">
            <input
              type="checkbox"
              checked={!lockSliders}
              onChange={(event) =>
                set({
                  lockSliders: !(event.target as HTMLInputElement).checked,
                })}
              disabled={!showGuides}
            />
            <span class="fpv-scope-track" />
            <span class="fpv-scope-label">Sliders</span>
          </label>
        </div>

        {showGuides && !lockSliders && (
          <div class="fpv-guide-controls">
            <div class="fpv-guide-row">
              <label for="fpv-h1">H1</label>
              <input
                id="fpv-h1"
                class="fpv-guide-slider"
                type="range"
                min="0"
                max="100"
                value={h1}
                onInput={(event) =>
                  set({ h1: Number((event.target as HTMLInputElement).value) })}
              />
            </div>
            <div class="fpv-guide-row">
              <label for="fpv-h2">H2</label>
              <input
                id="fpv-h2"
                class="fpv-guide-slider"
                type="range"
                min="0"
                max="100"
                value={h2}
                onInput={(event) =>
                  set({ h2: Number((event.target as HTMLInputElement).value) })}
              />
            </div>
            <div class="fpv-guide-row">
              <label for="fpv-v1">V1</label>
              <input
                id="fpv-v1"
                class="fpv-guide-slider"
                type="range"
                min="0"
                max="100"
                value={v1}
                onInput={(event) =>
                  set({ v1: Number((event.target as HTMLInputElement).value) })}
              />
            </div>
            <div class="fpv-guide-row">
              <label for="fpv-v2">V2</label>
              <input
                id="fpv-v2"
                class="fpv-guide-slider"
                type="range"
                min="0"
                max="100"
                value={v2}
                onInput={(event) =>
                  set({ v2: Number((event.target as HTMLInputElement).value) })}
              />
            </div>
          </div>
        )}
      </div>
    </section>
  );
}
