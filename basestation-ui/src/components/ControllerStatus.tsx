import { controller } from "../net/ws.ts";

// The gamepad, as the base station sees it. There is only one reader now — the
// pygame thread on the base station (basestation/controller_input.py) — so this
// is the whole truth about whether a controller can drive, rather than one of
// two possible sources. A pad plugged into the tablet showing the dashboard is
// not a control surface any more; see net/input.ts for why.
export function ControllerStatus() {
  const server = controller.value;
  return (
    <div class="controller-status">
      <span>🎮</span>
      <span>
        {server.connected ? server.name ?? "controller" : "no controller"}
      </span>
    </div>
  );
}
