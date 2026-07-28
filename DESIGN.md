---
name: RoverSoftware Base Station
description: An OLED-dark glass cockpit — machined instruments floating over live terrain, where color only ever means state.
colors:
  bg: "#07090d"
  map-void: "#0a0d12"
  well: "#0c0f15"
  panel: "rgba(19, 24, 33, 0.72)"
  panel-solid: "#131821"
  panel-2: "rgba(31, 38, 48, 0.7)"
  hairline: "rgba(255, 255, 255, 0.08)"
  hairline-strong: "rgba(255, 255, 255, 0.14)"
  text: "#e9edf4"
  muted: "#8b95a5"
  faint: "#5c6675"
  accent: "#4c9eff"
  accent-deep: "#2f7fe0"
  accent-lift: "#7cc0ff"
  on-accent: "#06131f"
  ok: "#35c46b"
  warn: "#f0a53a"
  danger: "#ff4d5a"
typography:
  headline:
    fontFamily: "Space Grotesk Variable, system-ui, sans-serif"
    fontSize: "17px"
    fontWeight: 700
    lineHeight: 1
    letterSpacing: "0.14em"
  title:
    fontFamily: "Space Grotesk Variable, system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 600
    lineHeight: 1.4
    letterSpacing: "0.02em"
  body:
    fontFamily: "Space Grotesk Variable, system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 400
    lineHeight: 1.4
    letterSpacing: "normal"
  label:
    fontFamily: "Space Grotesk Variable, system-ui, sans-serif"
    fontSize: "10px"
    fontWeight: 600
    lineHeight: 1.2
    letterSpacing: "0.22em"
  readout:
    fontFamily: "JetBrains Mono Variable, ui-monospace, monospace"
    fontSize: "14px"
    fontWeight: 400
    lineHeight: 1.4
    letterSpacing: "normal"
  micro:
    fontFamily: "JetBrains Mono Variable, ui-monospace, monospace"
    fontSize: "11.5px"
    fontWeight: 400
    lineHeight: 1.3
    letterSpacing: "0.02em"
rounded:
  panel: "20px"
  control: "13px"
  input: "10px"
  inner: "8px"
  pill: "999px"
spacing:
  hair: "3px"
  xs: "6px"
  sm: "9px"
  md: "14px"
  lg: "16px"
components:
  panel-glass:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.text}"
    rounded: "{rounded.panel}"
    padding: "14px"
  button:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.text}"
    rounded: "{rounded.control}"
    padding: "0 15px"
    height: "52px"
    typography: "{typography.title}"
  button-active:
    backgroundColor: "{colors.accent}"
    textColor: "{colors.on-accent}"
    rounded: "{rounded.control}"
    padding: "0 15px"
    height: "52px"
  button-danger:
    backgroundColor: "rgba(255, 77, 90, 0.1)"
    textColor: "{colors.danger}"
    rounded: "{rounded.control}"
    padding: "0 15px"
    height: "52px"
  button-small:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.text}"
    rounded: "{rounded.control}"
    padding: "0 13px"
    height: "38px"
  estop:
    backgroundColor: "{colors.danger}"
    textColor: "#ffffff"
    rounded: "16px"
    padding: "0 26px"
    height: "60px"
    typography: "{typography.headline}"
  pill:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "6px 13px"
  input-number:
    backgroundColor: "{colors.well}"
    textColor: "{colors.text}"
    rounded: "{rounded.input}"
    padding: "0 10px"
    height: "38px"
    width: "92px"
    typography: "{typography.readout}"
  toggle-on:
    backgroundColor: "{colors.accent}"
    textColor: "#ffffff"
    rounded: "{rounded.pill}"
    width: "56px"
    height: "32px"
  chip:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "5px 11px"
  chip-on:
    backgroundColor: "{colors.accent}"
    textColor: "#ffffff"
    rounded: "{rounded.pill}"
    padding: "5px 11px"
  robot-card:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.text}"
    rounded: "{rounded.control}"
    padding: "12px 13px"
  robot-card-selected:
    backgroundColor: "rgba(76, 158, 255, 0.1)"
    textColor: "{colors.text}"
    rounded: "{rounded.control}"
    padding: "12px 13px"
---

# Design System: RoverSoftware Base Station

## Overview

**Creative North Star: "The Glass Cockpit"**

You are looking through a canopy at ground you are driving across, and the
instruments are laid over it in machined glass. That is the whole system in one
image. The map is never a panel in the layout — it is the world, full-bleed
behind everything, and every control is a lens you read the world through rather
than a card that replaces it. Panels blur and saturate what is behind them
(`blur(20px) saturate(1.3)`), so the terrain stays legible through the glass and
the operator never loses the connection between the numbers and the ground.

The surfaces are OLED-dark on purpose — near-black (#07090d) so an unlit pixel is
genuinely off, and so the only things emitting light on the screen are the things
that are actually happening. Chrome is quiet to the point of near-invisibility:
hairlines at 8% white, labels at 22% letter-spacing in a grey you have to look
for. What is loud is state. A live robot glows green. An armed e-stop breathes.
A rejected setting paints its own left edge red. The interface holds still and
lets the machines be the moving part.

Controls are tactile and consequential. Every press compresses (`scale(0.96)`),
every transition uses a cubic-bezier with real mass and never a linear ease,
and the drive nub tracks the finger with no transition at all while it is held —
because a joystick that eases toward your thumb is a joystick that lies about
what the robot is doing. Nothing here is decorative motion. Every animation is a
machine acknowledging an instruction it is about to carry out.

**Key Characteristics:**
- Full-bleed live map; all UI floats above it in translucent glass
- Near-black OLED base with a rationed, signal-only palette
- Two type voices: Space Grotesk for words, JetBrains Mono for anything a robot measured
- No display type — the largest thing on screen is the word E-STOP at 17px
- Raised glass for information, recessed wells for input
- 52px minimum tap target everywhere, on every screen size
- Motion with mass; nothing linear, nothing decorative

## Colors

A near-monochrome instrument field with four signal colors that are only ever
allowed to mean something.

### Primary
- **Beacon Blue** (`{colors.accent}`): selection and liveness. The selected robot's
  card border and tint, the active mode button, the live state node in a routine
  graph, a toggle that is on, a focused input's border, and the heading arrow of
  every robot on the map. If something is blue, it is either the thing you picked
  or the thing happening right now.
- **Deep Beacon** (`{colors.accent-deep}`) and **Beacon Lift** (`{colors.accent-lift}`):
  the two ends of the gradient on physical-feeling blue objects — the drive pad
  nub and the range-slider thumb — lit from above so they read as a raised cap.
- **Ink on Beacon** (`{colors.on-accent}`): the near-black used for text on a filled
  blue surface. Never pure black, never white.

### Secondary
- **Live Green** (`{colors.ok}`): a thing is alive and reporting. Online dots, filled
  IMU calibration pips, a confirmed setting's gutter, a terminal state in a
  routine. Always paired with a glow when it means *live* rather than *passed*.
- **Caution Amber** (`{colors.warn}`): pending or restart-gated. An uncommitted field
  edit, a `restart` badge, a waypoint tooltip, a link-degraded banner. Amber is
  "not yet," never "wrong."
- **Abort Red** (`{colors.danger}`): stopped, refused, offline, invalid. The e-stop,
  a latched robot, a rejected value, a dangling transition wire. Red is the only
  color permitted to be a large filled area, and only on the e-stop.

### Neutral
- **Canopy Black** (`{colors.bg}`): the base behind everything and the theme-color of
  the installed app. **Map Void** (`{colors.map-void}`) sits a hair above it as the
  map's own background before tiles load.
- **Instrument Glass** (`{colors.panel}`): the translucent fill of every floating
  panel. **Glass Solid** (`{colors.panel-solid}`) is its opaque twin, used only where
  transparency would be unreadable (Leaflet controls, SVG label plates).
- **Raised Glass** (`{colors.panel-2}`): one step up — buttons, pills, robot cards,
  actuator cards, the canvas toolbar. This is "an object sitting on the panel."
- **Inset Well** (`{colors.well}`): one step *down*, and opaque. Every input, track,
  tab strip, and segmented control. This is "a slot cut into the panel."
- **Panel White** (`{colors.text}`), **Readout Grey** (`{colors.muted}`), **Etched Grey**
  (`{colors.faint}`): the three ink levels — value, label, and engraving.
- **Hairline** / **Hairline Strong** (`{colors.hairline}` / `{colors.hairline-strong}`):
  8% and 14% white. Every border in the system is one of these two. There are no
  colored borders except on signal states.

### Named Rules
**The Signal-Only Rule.** Green, amber, red and blue mean state and nothing else.
No color is ever chosen because a surface looked plain. Audit test: point at any
colored pixel and name the machine condition it reports — if you can't, it's
wrong.

**The One Blue Rule.** Beacon Blue marks exactly one subject per view: the
selected robot, or the state the machine is in right now. Two blue things
competing in the same panel means the hierarchy has failed.

**The Big-Red-Is-Reserved Rule.** Exactly one element in the entire system is a
large saturated fill: the E-STOP. Nothing else may take a solid red background,
because the operator must be able to find it by color alone, at a glance, in a
hurry.

## Typography

**Body / UI Font:** Space Grotesk Variable (with `system-ui`, `-apple-system`, `Segoe UI`, `Noto Sans`, `DejaVu Sans`)
**Readout / Mono Font:** JetBrains Mono Variable (with `ui-monospace`, `SFMono-Regular`, `DejaVu Sans Mono`)
**Display Font:** none — deliberately.

**Character:** Space Grotesk's slightly mechanical geometry reads as engineered
rather than corporate, and it holds up at the small sizes a dense console
demands. JetBrains Mono does the actual work: every quantity a robot reported is
tabular and fixed-width, so a value that changes doesn't shift the layout under
your thumb while you're driving. Both are bundled locally — there is no network
in the field.

### Hierarchy
- **Headline** (700, 17px, `0.14em`, uppercase): the E-STOP label. This is the only
  headline in the system, and it is the largest type anywhere on screen.
- **Title** (600, 16px, `0.02em`): the brand lockup, robot names, group headers
  (15px), button labels (15px). The workhorse for anything clickable or nameable.
- **Body** (400, 16px, 1.4): base size. Help text drops to 12px with 1.5 line-height
  and a `62ch` measure.
- **Label** (600, 10px, `0.22em`, uppercase): eyebrows and section markers, in Etched
  Grey. Widely tracked so they read as engraving on a panel rather than as text.
  The brand's "BASE STATION" sub-line is the same idea at 9px / `0.24em`.
- **Readout** (mono, 14px): telemetry values, coordinates, config numbers, node
  ids on the routine canvas.
- **Micro** (mono, 11.5px): axis rows, map tags, bind values, edge labels, bar
  captions. The smallest legible unit in the system.

### Named Rules
**The No-Display Rule.** Nothing exceeds 17px. This is a console, not a page —
there is no hero, no headline, and no title that announces where you are. If a
new surface seems to need 32px type, it needs a better layout instead.

**The Mono-For-Measured Rule.** If a robot reported it, measured it, or a
validator clamped it, it is JetBrains Mono. If a human wrote it, it is Space
Grotesk. Mode names, help text, and button labels are prose. Battery percentage,
lat/lon, PID gains, and channel numbers are readouts. Never mix within one value.

**The Widely-Tracked-Small Rule.** Type below 11px is always uppercase and tracked
to at least `0.1em`. Small tight lowercase is unreadable at arm's length on a 7"
panel in daylight.

## Layout

**The HUD is a three-row grid over a full-bleed map.** `#app` is a positioned
container holding the Leaflet map at `inset: 0` and a `.hud` grid layered above it
at `z-index: 1000` — above every Leaflet pane (max ~800) so tiles and markers can
never paint over a control. The grid is
`minmax(320px, 360px) 1fr` × `auto 1fr auto`, with named areas `top` / `rail` /
`dock`: a topbar pinned top-left, a full-height rail down the left, and a dock
anchored bottom-right that carries the drive pad.

**The overlay is transparent to pointers.** `.hud` sets `pointer-events: none` and
each child re-enables it, so the gaps between panels are live map. This is why the
layout uses gaps rather than a background: the gutters are how you pan the terrain
without dismissing the instruments.

**Padding is safe-area aware everywhere.** Every edge inset is
`calc(14px + var(--sa-*))`, with the four `env(safe-area-inset-*)` values captured
as `--sa-t/r/b/l` at `:root`. The iPad home indicator and notch never sit under a
control.

**Spacing rhythm.** The system runs on a loose odd-numbered scale rather than a
strict 4pt grid — 3 / 6 / 9 / 14 / 16 are the reused steps, with 9px between
sibling controls, 14px between panels and as standard panel padding, and 13px of
vertical breathing inside a rail section. Follow the existing steps; do not
introduce a parallel 4/8/12/16 scale alongside it.

**Forms get a measure, canvases get the room.** `.settings-col` is capped at 720px
and centered, because these are forms with help text and a 1600px line of prose on
a kiosk is unreadable. The one documented exception is `.settings-col.wide`
(1600px) for the routine node graph, where a narrow column means you spend the
session panning.

### Responsive
- **≤900px:** the routine editor stacks — graph above, inspector below at full width.
- **≤820px or portrait:** the rail becomes a bottom sheet (`max-height: 48dvh`) with a
  drag handle and a `translateY(calc(100% - 54px))` collapsed state; the dock floats
  above it.
- **≤760px:** the canvas hint text is dropped rather than truncated.
- **≤700px:** the settings header wraps to two rows, with an `::after` spacer
  reserving the e-stop's corner so the tab strip still gets full width.
- **≤540px tall (the 7" 800×480 kiosk):** everything compacts — HUD gap and padding
  to 9px, drive pad to 150px, e-stop to 52px, robot cards to 9px/11px. This
  breakpoint is keyed on *height*, not width, because the kiosk's problem is
  vertical room.

### Named Rules
**The Gap-Is-The-Map Rule.** The HUD's gutters are pass-through touch targets for
the terrain. Never fill a gap with a background, and never add a full-bleed
container behind the HUD grid.

**The Stop-Is-Never-Covered Rule.** The e-stop dock sits at `z-index: 1001`; the
settings sheet at `1000`. Any new full-screen surface goes *under* the e-stop, and
any layout that would push it off-screen is wrong. Changing a drive limit must
never put the stop button out of reach.

**The Thumb-Floor Rule.** `--tap: 52px` is the minimum height for any primary
control, and it does not shrink on the kiosk breakpoint. Controls that look
smaller (the 44px gear) still carry a full tap target.

## Elevation & Depth

**Hybrid, and the hybrid is the identity.** Depth is expressed two ways at once:
translucent glass that *floats* above the terrain, and opaque wells that *sink*
into the panel. Everything in the system is one or the other.

Floating surfaces get the full instrument treatment: a translucent fill, a 1px
hairline, `backdrop-filter: blur(20px) saturate(1.3)`, a 1px inset white highlight
along the top edge (`--inset-hi`) that reads as a machined bevel catching light,
and a long soft drop shadow. The saturation boost is deliberate — it keeps
satellite terrain colorful through the glass instead of washing it grey.

Recessed surfaces do the opposite: a flat opaque `#0c0f15` fill, darker than any
panel, with a hairline border and no shadow. Inputs, sliders, progress tracks, tab
strips, segmented controls and button chips are all wells.

### Shadow Vocabulary
- **Panel float** (`box-shadow: inset 0 1px 0 rgba(255,255,255,0.06), 0 18px 50px -24px rgba(0,0,0,0.8)`): every floating glass panel.
- **Pad float** (`inset 0 1px 0 rgba(255,255,255,0.06), 0 20px 50px -20px rgba(0,0,0,0.85)`): the drive pad, slightly heavier — it is the object closest to the operator.
- **Sheet lift** (`inset 0 1px 0 rgba(255,255,255,0.06), 0 -14px 40px -20px rgba(0,0,0,0.85)`): the bottom-sheet rail in portrait, shadow cast upward.
- **Cap** (`0 8px 20px -6px rgba(0,0,0,0.7), inset 0 1px 2px rgba(255,255,255,0.4)`): the drive nub and slider thumb — a raised physical cap, lit from above.
- **Alarm** (`0 10px 30px -8px rgba(255,60,76,0.6)`): the e-stop's red bloom, animating to `-4px / 0.95` on the 1.1s armed pulse.
- **LED glow** (`0 0 8px currentColor`, `0 0 9px var(--ok)`): a lit indicator. Small, colored, and only on something live.
- **Status gutter** (`inset 2px 0 0 <signal>`): a 2px colored bar on a field's left edge — amber dirty, green confirmed, red rejected, blue listening.

### Named Rules
**The See-Through Rule.** Every floating panel is glass. A solid opaque panel over
the map is a bug — the operator must always see the ground through the
instruments. `--panel-solid` exists only for surfaces that are not over the map
(Leaflet's own controls, SVG label plates behind graph text).

**The Float-Or-Sink Rule.** There is no flat surface in this system. A surface
either floats (translucent + blur + bevel + drop shadow) or sinks (opaque
`#0c0f15` + hairline). If you are adding a surface and it is neither, decide
which: does it present information, or does it receive input?

**The Glow-Is-Alive Rule.** A colored `box-shadow` means a live state — a lit LED,
a filled calibration pip, an armed e-stop. Never apply a glow to a static surface
for atmosphere.

## Shapes

**A nested radius scale, each step smaller as you go inward.** 20px on floating
panels, 13px on controls sitting on them, 10px on inputs sitting inside those, and
8–9px on the smallest inner elements (tab pills, segmented buttons, chips). The
e-stop is the one deliberate exception at 16px — between panel and control,
because it belongs to neither.

Fully round (`999px`) is reserved for two silhouettes: **status pills**, which read
as lozenge-shaped indicator lamps, and **chips**, which read as removable tags. A
rectangle with a large radius is a container; a lozenge is a state.

Circles carry the physical controls: the drive pad is a 190px circle (150px on the
kiosk) with a dashed inner ring at 16% inset and a 1px crosshair inset 12% from each
edge — a gunsight, not a decoration, since it marks the zero the nub returns to.
The 74px nub, the 26px slider thumb, the 24px toggle knob and the 8–10px LEDs are
all circles. **Round means it moves or it lights up.**

The routine canvas keeps the same vocabulary in SVG: rounded node bodies filled
with Raised Glass and stroked in Hairline Strong, `crosshair`-cursor circular link
handles on each node's right edge, and Bézier wires at 1.75px that thicken to 2.5px
and turn Beacon Blue when chosen. A dangling wire is red and dashed.

### Named Rules
**The Nested-Radius Rule.** A child's radius is always smaller than its parent's.
20 → 13 → 10 → 8. Never place a 20px-radius element inside a 13px one.

**The Round-Means-Live Rule.** Circles are reserved for things that move (nub,
thumb, knob) or things that light up (LEDs, pips, badges). A circular static
decoration does not exist here.

## Components

### Buttons
- **Shape:** softly squared (13px radius), 52px minimum height, 15px semibold label.
- **Default:** Raised Glass fill, hairline border, inset top bevel, Panel White text.
- **Active / selected:** Beacon Blue fill and border with near-black `#06131f` text —
  the strongest inversion in the system, used for the current mode and on-state
  segmented options.
- **Ghost:** transparent fill, hairline retained. For secondary actions inside a panel.
- **Danger:** Abort Red text on a 10% red wash with a 40% red border; on press the
  fill goes fully red with white text — the action commits visually before it commits.
- **Subtle:** Readout Grey text with a *dashed* border. Reserved for "add another one
  of these" affordances on the document editors.
- **Small:** 38px height, 13px label, for toolbars and inline actions.
- **Press:** `scale(0.96)` over 0.25s on `--ease`. Not a hover system — hover exists
  (icon buttons brighten) but nothing depends on it.

### E-STOP (signature component)
The single most important object on screen. A 60px-tall (52px on the kiosk) red
vertical gradient (`#ff5b67 → #e0313e`), 16px radius, white 700-weight 17px label
tracked to `0.14em`, sitting in its own absolutely-positioned dock in the top-right
corner at `z-index: 1001` — above the settings sheet, above everything. When a
robot is latched it takes `.armed` and breathes on a 1.1s infinite pulse that
widens and brightens its red bloom. It is the only element in the system permitted
a large saturated fill, and it is never scrolled, covered, or collapsed.

### Drive pad (signature component)
A 190px circular glass instrument with a blue radial bloom at 50%/38%, a dashed
guide ring, a crosshair marking zero, and a 74px gradient nub. The nub eases with
`--ease-out` at 0.18s when releasing to center, and has `transition: none` while
`.active` — the thumb's position is reported with zero lag, because any easing here
is the interface lying about the robot's throttle. Disabled state is `opacity: 0.4`
with pointer events off. `touch-action: none` so the browser never steals the drag.

### Cards / Containers
- **Panel:** 20px radius, Instrument Glass, hairline border, blur + saturate, panel
  float shadow, 14px padding.
- **Robot card:** 13px radius on Raised Glass with a **1.5px transparent border that
  becomes Beacon Blue when selected**, plus a 10% blue tint. The border is present
  but invisible at rest so selection never reflows the list by a pixel.
- **State / actuator / mechanism cards:** 13px radius on Raised Glass, 12px/14px
  padding. State cards add a 3px colored left edge — Hairline Strong at rest, Beacon
  Blue when live (plus a 1px inset blue ring), Live Green when terminal.

### Inputs / Fields
- **Style:** Inset Well fill, hairline border, 10px radius, 38px height, mono 14px,
  right-aligned for numbers (92px fixed width, spinners stripped — useless at
  touch sizes and they steal width).
- **Focus:** border becomes Beacon Blue, outline removed. Nothing else moves.
- **State gutters:** a 2px inset bar on the left edge with 10px of added left padding —
  amber for a pending edit, green for a confirmed one (fading over 0.6s), red for a
  value the robot refused. All three occupy the same gutter so a column of fields
  reads as one status channel.
- **Unknown:** `opacity: 0.5` — a value the server has never reported is shown, but
  visibly not real.
- **Toggle:** 56×32 well that fills Beacon Blue when on; a 24px knob translates 24px
  and brightens to pure white.
- **Segmented:** a well containing 8px-radius buttons; the on-state takes Beacon Blue
  with `#06131f` text.
- **Range:** a 5px well track with a 26px gradient cap thumb, on a 34px-tall
  transparent hit area so the tap target survives the thin track.

### Navigation
There is no nav in the operating view — the map *is* the view, and settings is a
full-screen sheet reached from one gear in the topbar. Inside settings, a **tab
strip** is a well containing 10px-radius transparent buttons; the active tab takes
Raised Glass, Panel White text and the inset bevel, so the selected tab reads as
raised out of the slot. The sheet enters on a 0.32s `--ease-out` fade-and-rise from
10px.

### Status indicators
- **Pill:** 999px lozenge on Raised Glass, 12px semibold, optionally containing an 8px
  LED that glows in `currentColor`. `.quiet` variant switches to mono 11.5px in
  Readout Grey and lets the LED alone carry the color.
- **Dot:** a 10px circle — Live Green with a 9px glow when online, Abort Red flat when
  offline, Etched Grey when unknown. The glow is the tell: offline is *not* lit.
- **Pips:** three 5×11px rounded bars for IMU calibration 0–3. A level, not a
  measurement — what matters is "enough to trust the heading yet."
- **Bars:** 7px well tracks with a signal-colored fill that animates `left` and `width`
  on a 0.12s *linear* transition. Linear is correct here and nowhere else: this is a
  live measurement, and easing it would misreport the machine.
- **Badge:** 9px uppercase tracked `0.14em`, hairline-strong border, 5px radius. The
  `restart` badge takes amber.

### Routine canvas (signature component)
An SVG node graph on a dotted grid (`--hairline-strong` circles). Nodes are rounded
Raised Glass rectangles with a mono 13px id and a 10px Etched Grey subtitle; the
stroke encodes role — Deep Beacon for start, Live Green for terminal, Abort Red for
a problem, Beacon Blue at 2px when selected. **The live node is filled, not
outlined**, because across a field you want a block of color, not a 2px border, and
its text inverts to white. Wires carry a 20px-wide transparent hit stroke behind
the 1.75px visible line — a 2px wire is unhittable with a fingertip. Edge labels
sit on Glass Solid plates so they stay readable over a wire. The toolbar is a bar
*above* the canvas, never floating over it, because a floating toolbar puts
invisible dead zones exactly where auto-layout drops nodes.

## Do's and Don'ts

### Do:
- **Do** float every panel on glass (`rgba(19,24,33,0.72)` + `blur(20px) saturate(1.3)`
  + a 1px `rgba(255,255,255,0.06)` top bevel). The terrain shows through the
  instruments — that is the system.
- **Do** sink anything that receives input into an opaque `#0c0f15` well with a
  hairline border. Float or sink; never flat.
- **Do** set every number a robot reported in JetBrains Mono, and every word a human
  wrote in Space Grotesk.
- **Do** keep every primary control at or above `--tap: 52px`, including on the 7"
  kiosk breakpoint.
- **Do** put a control's state on its left-edge gutter (`inset 2px 0 0 <signal>`) so a
  column of fields reads as one status channel.
- **Do** use `--ease` (`cubic-bezier(0.32,0.72,0,1)`) for state changes and `--ease-out`
  (`cubic-bezier(0.16,1,0.3,1)`) for things settling into place.
- **Do** use `linear` transitions *only* for live measurements — telemetry bars, axis
  fills — where easing would misreport the machine.
- **Do** nest radii inward: 20 → 13 → 10 → 8.
- **Do** honor `prefers-reduced-motion` — the system already collapses all durations
  to 0.001ms globally.
- **Do** bundle every font and tile asset locally. There is no network in the field.

### Don't:
- **Don't** put a solid opaque panel over the map.
- **Don't** use a signal color decoratively. If you can't name the machine condition a
  colored pixel reports, remove the color.
- **Don't** give anything other than the E-STOP a large saturated red fill.
- **Don't** let any element cover, scroll, or collapse the e-stop dock — it lives above
  every full-screen surface at `z-index: 1001`.
- **Don't** introduce display type. Nothing in this system exceeds 17px, and the
  largest type is the word E-STOP.
- **Don't** ease the drive nub while it is being held. Zero lag under the thumb.
- **Don't** add a parallel 4/8/12/16 spacing scale — the system runs on 3/6/9/14/16.
- **Don't** rely on hover for anything. The primary device is a touchscreen with no
  pointer.
- **Don't** fill the HUD's gutters with a background; they are how the operator
  touches the map.
- **Don't** let this read as a consumer smart-home app (rounded pastel cards, friendly
  illustrations, cheerful empty states), an enterprise SaaS dashboard (KPI tiles, a
  nav sidebar, grey-on-white charts), or a gamer/RGB aesthetic (neon for excitement
  rather than signal). All three are confirmed rejections.
