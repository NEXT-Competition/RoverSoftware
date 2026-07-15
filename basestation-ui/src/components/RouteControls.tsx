import { selected } from "../net/ws.ts";
import {
  clearRoute,
  routeMode,
  routePts,
  sendRoute,
  toggleRouteMode,
} from "../state/route.ts";

export function RouteControls() {
  const rid = selected.value;
  const adding = routeMode.value;
  const count = routePts.value.length;

  return (
    <div>
      <div class="section-title">
        <span class="eyebrow">Route</span>
        {count > 0 && <span class="eyebrow" style="color:var(--warn)">{count} wp</span>}
      </div>
      <div class="btn-row" style="margin-top:8px">
        <button
          class={`btn${adding ? " active" : ""}`}
          disabled={!rid}
          onClick={toggleRouteMode}
        >
          {adding ? "Adding…" : "Add waypoints"}
        </button>
        <button
          class="btn"
          disabled={!rid || count === 0}
          onClick={sendRoute}
        >
          Send
        </button>
        <button class="btn ghost" disabled={count === 0} onClick={clearRoute}>
          Clear
        </button>
      </div>
      <p class="hint">
        {adding
          ? "Tap the map to drop waypoints, then Send."
          : "Toggle “Add waypoints”, tap the map, then Send."}
      </p>
    </div>
  );
}
