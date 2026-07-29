// Which light the console is being read in.
//
// Not a cosmetic preference. The dark rendition is right in a pit or indoors,
// and wrong on the 7" panel outdoors — sunlight on glass beats any dark UI, and
// this is the surface an operator uses while looking mostly at a rover. So the
// two renditions are a FIELD CONTROL, sit one tap away, and the choice is
// remembered per device: the kiosk that boots outdoors stays in daylight, and
// the laptop in the pit stays dark, without either operator setting it twice.
//
// Kept in localStorage rather than in base-station settings on purpose. This is
// a property of the screen you are standing at, not of the fleet — pushing it
// through the bridge would make the tablet in the sun follow the laptop indoors.

import { signal } from "@preact/signals";

export type Rendition = "console" | "daylight";

const KEY = "rs.rendition";

function initial(): Rendition {
  try {
    const saved = localStorage.getItem(KEY);
    if (saved === "console" || saved === "daylight") return saved;
  } catch {
    // Private mode, or a kiosk with storage disabled. Fall through to the
    // ambient guess rather than failing to render.
  }
  // The OS is the only evidence available on first run, and a Pi kiosk that
  // reports a light preference is usually one someone set up for daylight.
  try {
    if (globalThis.matchMedia?.("(prefers-color-scheme: light)").matches) {
      return "daylight";
    }
  } catch { /* ignore */ }
  return "console";
}

export const rendition = signal<Rendition>(initial());

/** Push the current rendition onto <html>, which is what the tokens key off. */
function paint(value: Rendition): void {
  const root = document.documentElement;
  if (value === "daylight") root.setAttribute("data-theme", "daylight");
  else root.removeAttribute("data-theme");
  // Keeps the browser's own chrome (scrollbars, form controls, the address bar
  // on a tablet) on the same side as the UI.
  root.style.colorScheme = value === "daylight" ? "light" : "dark";
}

export function setRendition(value: Rendition): void {
  rendition.value = value;
  paint(value);
  try {
    localStorage.setItem(KEY, value);
  } catch { /* a kiosk with storage disabled still gets the change */ }
}

export function toggleRendition(): void {
  setRendition(rendition.value === "daylight" ? "console" : "daylight");
}

/** Called once at start-up, before the first paint, so the app never flashes
 *  the wrong rendition at an operator who chose the other one. */
export function initRendition(): void {
  paint(rendition.value);
}
