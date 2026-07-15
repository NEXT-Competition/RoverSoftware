import { render } from "preact";

// Bundled locally (no CDN) so the native binary and offline Pi kiosk render
// identically. Variable fonts: Space Grotesk for UI, JetBrains Mono for numbers.
import "@fontsource-variable/space-grotesk";
import "@fontsource-variable/jetbrains-mono";
import "leaflet/dist/leaflet.css";
import "./styles/theme.css";

import { App } from "./app.tsx";
import { connect } from "./net/ws.ts";
import { releaseDrive, startInputLoop } from "./net/input.ts";

connect();
startInputLoop();

// Safety: if the tab is backgrounded or loses focus mid-drive, command a stop.
addEventListener("visibilitychange", () => {
  if (document.visibilityState === "hidden") releaseDrive();
});
addEventListener("blur", releaseDrive);

render(<App />, document.getElementById("app")!);
