import { render } from "preact";

// Bundled locally (no CDN) so the native binary and offline Pi kiosk render
// identically. Variable fonts: Space Grotesk for UI, JetBrains Mono for numbers.
import "@fontsource-variable/archivo";
import "@fontsource-variable/jetbrains-mono";
import "leaflet/dist/leaflet.css";
import "./styles/theme.css";

import { effect } from "@preact/signals";

import { App } from "./app.tsx";
import { baseSettings, connect, robotConfigs } from "./net/ws.ts";
import { releaseDrive, startInputLoop } from "./net/input.ts";
import { acknowledge } from "./state/settings.ts";

connect();
startInputLoop();

// A settings frame is the server answering an edit. Clear the local copies of
// whatever we sent so the fields show what the server actually stored (it
// clamps), while edits still in progress are left alone. Wired here rather than
// inside ws.ts to keep the socket layer unaware of the editing state above it.
effect(() => {
  baseSettings.value;
  robotConfigs.value;
  acknowledge();
});

// Safety: if the tab is backgrounded or loses focus mid-drive, command a stop.
addEventListener("visibilitychange", () => {
  if (document.visibilityState === "hidden") releaseDrive();
});
addEventListener("blur", releaseDrive);

render(<App />, document.getElementById("app")!);
