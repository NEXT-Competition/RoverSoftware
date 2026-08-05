import { useEffect, useState } from "preact/hooks";
import { selected, videoRobots } from "../net/ws.ts";

const FPV_STORAGE_KEY = "rs.fpv.guide_state";

type GuideState = {
  showGuides: boolean;
  lockSliders: boolean;
  h1: number;
  h2: number;
  v1: number;
  v2: number;
};

function loadGuideState(): GuideState {
  try {
    const raw = localStorage.getItem(FPV_STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      if (parsed && typeof parsed === "object") {
        return {
          showGuides: parsed.showGuides === true,
          lockSliders: parsed.lockSliders === true,
          h1: typeof parsed.h1 === "number" ? parsed.h1 : 25,
          h2: typeof parsed.h2 === "number" ? parsed.h2 : 75,
          v1: typeof parsed.v1 === "number" ? parsed.v1 : 25,
          v2: typeof parsed.v2 === "number" ? parsed.v2 : 75,
        };
      }
    }
  } catch {
    /* ignore storage failures */
  }
  return { showGuides: false, lockSliders: false, h1: 25, h2: 75, v1: 25, v2: 75 };
}

function saveGuideState(state: GuideState): void {
  try {
    localStorage.setItem(FPV_STORAGE_KEY, JSON.stringify(state));
  } catch {
    /* ignore storage failures */
  }
}

// First-person camera feed for the selected robot. The bridge serves an endless
// MJPEG stream at /video/{id}.mjpg, which an <img> renders frame by frame — no
// player, no decode code. We only mount the <img> when the fleet snapshot says
// this robot is actually streaming, so we never show a broken-image icon; when
// the feed goes stale the bridge drops it from `video` and we fall back to the
// placeholder within a few seconds.
export function FPV() {
  const id = selected.value;
  const live = id != null && videoRobots.value.includes(id);
  const saved = loadGuideState();
  const [showGuides, setShowGuides] = useState(saved.showGuides);
  const [lockSliders, setLockSliders] = useState(saved.lockSliders);
  const [h1, setH1] = useState(saved.h1);
  const [h2, setH2] = useState(saved.h2);
  const [v1, setV1] = useState(saved.v1);
  const [v2, setV2] = useState(saved.v2);

  useEffect(() => {
    saveGuideState({ showGuides, lockSliders, h1, h2, v1, v2 });
  }, [showGuides, lockSliders, h1, h2, v1, v2]);

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
              onChange={(event) => setShowGuides((event.target as HTMLInputElement).checked)}
            />
            <span class="fpv-scope-track" />
            <span class="fpv-scope-label">Scope</span>
          </label>
          <label class="fpv-scope-switch">
            <input
              type="checkbox"
              checked={!lockSliders}
              onChange={(event) => setLockSliders(!(event.target as HTMLInputElement).checked)}
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
                onInput={(event) => setH1(Number((event.target as HTMLInputElement).value))}
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
                onInput={(event) => setH2(Number((event.target as HTMLInputElement).value))}
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
                onInput={(event) => setV1(Number((event.target as HTMLInputElement).value))}
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
                onInput={(event) => setV2(Number((event.target as HTMLInputElement).value))}
              />
            </div>
          </div>
        )}
      </div>
    </section>
  );
}
