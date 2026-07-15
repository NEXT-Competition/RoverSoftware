import { controller } from "../net/ws.ts";
import { browserGamepad } from "../net/gamepad.ts";

// Show whichever gamepad is live — the server-side pygame reader (reported over
// /ws) or one the browser itself exposes.
export function ControllerStatus() {
  const server = controller.value;
  const local = browserGamepad.value;
  const active = server.connected || local.connected;
  const name = server.connected ? server.name : local.name;
  return (
    <div class="controller-status">
      <span>🎮</span>
      <span>{active ? name ?? "controller" : "no controller"}</span>
    </div>
  );
}
