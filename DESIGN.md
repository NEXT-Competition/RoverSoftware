---
name: RoverSoftware Base Station
description: A pit board over live terrain — paper-white cards, poster-scale numerals you read at a glance in daylight, and one fluorescent tape colour that marks the rover you are calling.
colors:
  bg: "#d9dbd4"
  map-void: "#cfd2ca"
  paper: "#fbfbf8"
  panel: "rgba(251, 251, 248, 0.82)"
  panel-solid: "#fbfbf8"
  panel-2: "rgba(255, 255, 255, 0.74)"
  well: "#e7e8e1"
  hairline: "rgba(20, 22, 15, 0.10)"
  hairline-strong: "rgba(20, 22, 15, 0.20)"
  ink: "#14160f"
  text: "#14160f"
  muted: "#5b6053"
  faint: "#767b6e"
  accent: "#ccf24a"
  accent-deep: "#b2d92f"
  accent-lift: "#ddf87e"
  on-accent: "#14160f"
  ok: "#0f7a4a"
  warn: "#b0600a"
  danger: "#d4192a"
typography:
  display:
    fontFamily: "Archivo Variable, system-ui, sans-serif"
    fontSize: "56px"
    fontWeight: 800
    lineHeight: 0.88
    letterSpacing: "-0.045em"
  headline:
    fontFamily: "Archivo Variable, system-ui, sans-serif"
    fontSize: "17px"
    fontWeight: 800
    lineHeight: 1
    letterSpacing: "0.10em"
  title:
    fontFamily: "Archivo Variable, system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 650
    lineHeight: 1.3
    letterSpacing: "-0.01em"
  body:
    fontFamily: "Archivo Variable, system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 400
    lineHeight: 1.45
    letterSpacing: "normal"
  label:
    fontFamily: "Archivo Variable, system-ui, sans-serif"
    fontSize: "10px"
    fontWeight: 700
    lineHeight: 1.2
    letterSpacing: "0.18em"
  readout:
    fontFamily: "JetBrains Mono Variable, ui-monospace, monospace"
    fontSize: "14px"
    fontWeight: 500
    lineHeight: 1.4
    letterSpacing: "-0.01em"
  micro:
    fontFamily: "JetBrains Mono Variable, ui-monospace, monospace"
    fontSize: "11.5px"
    fontWeight: 500
    lineHeight: 1.3
    letterSpacing: "0.02em"
rounded:
  panel: "26px"
  control: "16px"
  input: "12px"
  inner: "9px"
  pill: "999px"
  chamfer: "13px"
spacing:
  hair: "3px"
  xs: "6px"
  sm: "9px"
  md: "14px"
  lg: "16px"
components:
  panel-paper:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.ink}"
    rounded: "{rounded.panel}"
    padding: "14px"
  nav-cluster:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink}"
    rounded: "{rounded.pill}"
    padding: "5px"
  button:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink}"
    rounded: "{rounded.control}"
    padding: "0 16px"
    height: "52px"
    typography: "{typography.title}"
  button-active:
    backgroundColor: "{colors.ink}"
    textColor: "{colors.paper}"
    rounded: "{rounded.control}"
    padding: "0 16px"
    height: "52px"
  button-accent:
    backgroundColor: "{colors.accent}"
    textColor: "{colors.on-accent}"
    rounded: "{rounded.control}"
    padding: "0 16px"
    height: "52px"
  button-danger:
    backgroundColor: "rgba(212, 25, 42, 0.09)"
    textColor: "{colors.danger}"
    rounded: "{rounded.control}"
    padding: "0 16px"
    height: "52px"
  button-small:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink}"
    rounded: "{rounded.control}"
    padding: "0 14px"
    height: "38px"
  estop:
    backgroundColor: "{colors.danger}"
    textColor: "#ffffff"
    rounded: "18px"
    padding: "0 26px"
    height: "60px"
    typography: "{typography.headline}"
  pill:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "6px 14px"
  input-number:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    rounded: "{rounded.input}"
    padding: "0 11px"
    height: "38px"
    width: "92px"
    typography: "{typography.readout}"
  toggle-on:
    backgroundColor: "{colors.ink}"
    textColor: "{colors.paper}"
    rounded: "{rounded.pill}"
    width: "56px"
    height: "32px"
  chip:
    backgroundColor: "{colors.well}"
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "5px 12px"
  chip-on:
    backgroundColor: "{colors.ink}"
    textColor: "{colors.paper}"
    rounded: "{rounded.pill}"
    padding: "5px 12px"
  robot-card:
    backgroundColor: "{colors.panel-2}"
    textColor: "{colors.ink}"
    rounded: "{rounded.control}"
    padding: "12px 14px"
  robot-card-selected:
    backgroundColor: "{colors.accent}"
    textColor: "{colors.on-accent}"
    rounded: "{rounded.control}"
    padding: "12px 14px"
---

# Design System: RoverSoftware Base Station

## Overview

**Creative North Star: "The Pit Board"**

A pit board is held over a wall at a circuit so a driver going past at speed can
read it in one look: white card, black numerals set far larger than any document
would justify, and a strip of fluorescent tape marking which car the numbers are
about. Nothing on it is decorative, and every choice exists to survive distance,
motion and sunlight.

That is this console. The map is still the world — full-bleed, live, behind
everything — but the instruments over it are **paper**, not glass: warm white
cards that hold their contrast when the 7" panel is in direct sun, which is where
this product actually gets used. The numbers on those cards are set at poster
scale, because the operator reads them standing over a bench or holding a tablet
at arm's length, not leaning into a monitor.

The fluorescent lime is the tape. It marks **the rover you are calling** — the
selected robot's card, the mode it is in, the nub under your thumb — and it does
that job alone. Everything the machine reports about its own condition speaks in
marshal's flags instead: deep green, burnt amber, flag red. Fluorescent is for
*which one*; flag colour is for *how it is*. An operator never has to work out
whether a colour is talking about their selection or the robot's health, because
the two vocabularies never overlap.

**Key Characteristics:**
- Full-bleed live map; all UI floats above it on paper-white cards
- Poster-scale numerals for what an operator actually reads at distance
- One fluorescent accent for selection, three flag colours for state
- Two type voices: Archivo for words, JetBrains Mono for anything a robot measured
- Generous radii (26px panels) and pill-shaped clusters of controls
- 52px minimum tap target everywhere, on every screen size
- Motion with mass; nothing linear, nothing decorative

## Colors

A paper field with one fluorescent marker and three flag signals.

### Primary
- **Course Lime** (`{colors.accent}`): the tape. Selection and engagement, and
  nothing else — the selected robot's card, the active mode, the drive nub, a
  toggle that is on. If something is lime, it is the thing you picked or the thing
  you are commanding.
- **Deep Lime** (`{colors.accent-deep}`) and **Lime Lift** (`{colors.accent-lift}`):
  the two ends of the gradient on physical-feeling lime objects — the drive-pad nub
  and the range-slider thumb — lit from above so they read as a raised cap.
- **Ink** (`{colors.ink}`): near-black with a green cast so it sits with the lime
  rather than against it. Body text, and the fill of any control that is simply
  *on* without being the operator's current subject (an active tab, a chip).

### Secondary — the flags
- **Flag Green** (`{colors.ok}`): a thing is alive and reporting. Deliberately deep
  and blue-shifted, so it can never be mistaken for the fluorescent.
- **Flag Amber** (`{colors.warn}`): pending or restart-gated. Amber is "not yet,"
  never "wrong."
- **Flag Red** (`{colors.danger}`): stopped, refused, offline, invalid. The only
  colour permitted to be a large saturated fill, and only on the e-stop.

### Neutral
- **Field Paper** (`{colors.paper}`) and **Paper Glass** (`{colors.panel}`): the warm
  white of every floating card, opaque and translucent. Warm, not blue-white — a
  cool white on a sunlit panel reads as glare.
- **Raised Paper** (`{colors.panel-2}`): one step up — buttons, pills, robot cards,
  toolbars. "An object sitting on the card."
- **Cut Well** (`{colors.well}`): one step *down*, and opaque. Every input, track, tab
  strip and segmented control. "A slot cut into the card."
- **Ink** / **Pencil** (`{colors.muted}`) / **Faint Pencil** (`{colors.faint}`): the
  three ink levels — value, label, engraving.
- **Hairline** / **Hairline Strong**: 10% and 20% ink. Every border in the system is
  one of these two.

### Named Rules
**The Two-Vocabularies Rule.** Lime means *which one*. Green, amber and red mean
*how it is*. The two never trade jobs, and no state is ever fluorescent. Audit
test: point at any coloured pixel and say either "that is the one I picked" or name
the machine condition — if neither sentence fits, the colour is wrong.

**The Lime-Is-A-Surface Rule.** Course Lime is only ever a fill with ink on top. It
is never text, never a border on its own, never a glow. Lime type on paper is
illegible at exactly the distance this console is read from.

**The Big-Red-Is-Reserved Rule.** Exactly one element in the system is a large
saturated fill: the E-STOP. Nothing else may take a solid red background, because
the operator must find it by colour alone, at a glance, in a hurry.

## Typography

**Display / UI Font:** Archivo Variable (with `system-ui`, `-apple-system`, `Segoe UI`, `Noto Sans`, `DejaVu Sans`)
**Readout / Mono Font:** JetBrains Mono Variable (with `ui-monospace`, `SFMono-Regular`, `DejaVu Sans Mono`)

**Character:** Archivo is a grotesk drawn for high-performance signage and small
print — it holds a dense console at 13px and turns genuinely powerful at 56px
without changing family, which is the whole reason it can carry both the pit-board
numerals and the settings form. JetBrains Mono does the measuring: every quantity a
robot reported is tabular, so a value that changes doesn't shift the layout under
your thumb while you're driving. Both are bundled locally — there is no network in
the field.

### Hierarchy
- **Display** (800, 56px, `-0.045em`, `0.88`): the pit-board numerals. Reserved for
  the values an operator reads at distance — the selected rover's speed, its
  battery, the leg it is on. Mono variant for anything measured.
- **Headline** (800, 17px, `0.10em`, uppercase): the E-STOP label, and section
  banners on full-screen surfaces.
- **Title** (650, 16px): robot names, group headers, button labels. The workhorse for
  anything clickable or nameable.
- **Body** (400, 16px, 1.45): base size. Help text drops to 12.5px with a `62ch`
  measure.
- **Label** (700, 10px, `0.18em`, uppercase): eyebrows and section markers, in Faint
  Pencil. Engraving on a panel rather than text.
- **Readout** (mono, 14px): telemetry values, coordinates, config numbers.
- **Micro** (mono, 11.5px): axis rows, map tags, bind values, edge labels.

### Named Rules
**The Pit-Board Rule.** Display scale is earned by *reading distance*, never by
importance. A number an operator glances at from across a bench is display; a number
they only read while already looking at the panel is a readout. At most **three**
display numerals are on screen at once — a pit board with ten numbers is a
spreadsheet, and nobody reads a spreadsheet at speed.

**The Mono-For-Measured Rule.** If a robot reported it, measured it, or a validator
clamped it, it is JetBrains Mono. If a human wrote it, it is Archivo. Mode names,
help text and button labels are prose. Battery, lat/lon, PID gains and channel
numbers are readouts. Never mix within one value.

**The Widely-Tracked-Small Rule.** Type below 11px is always uppercase and tracked to
at least `0.1em`. Small tight lowercase is unreadable at arm's length on a 7" panel
in daylight.

## Layout

**The HUD is a three-row grid over a full-bleed map.** `#app` holds the Leaflet map at
`inset: 0` and a `.hud` grid layered above it at `z-index: 1000` — above every Leaflet
pane (max ~800) so tiles and markers can never paint over a control. The grid is
`minmax(320px, 360px) 1fr` × `auto 1fr auto`, with named areas `top` / `rail` / `dock`.

**The topbar is a pill cluster.** Brand lockup, connection state and the settings gear
sit inside one `999px` container on Raised Paper — a single object with internal
divisions, not a row of separate buttons. Controls that belong together share a capsule.

**The overlay is transparent to pointers.** `.hud` sets `pointer-events: none` and each
child re-enables it, so the gaps between panels are live map. The gutters are how you
pan the terrain without dismissing the instruments.

**Padding is safe-area aware everywhere.** Every edge inset is `calc(14px + var(--sa-*))`,
with the four `env(safe-area-inset-*)` values captured as `--sa-t/r/b/l` at `:root`.

**Spacing rhythm.** A loose odd-numbered scale rather than a strict 4pt grid — 3 / 6 /
9 / 14 / 16, with 9px between sibling controls and 14px between panels. Do not introduce
a parallel 4/8/12/16 scale alongside it.

### Responsive
- **≤900px:** the routine editor stacks — graph above, inspector below.
- **≤820px or portrait:** the rail becomes a bottom sheet (`max-height: 48dvh`) with a
  drag handle; the dock floats above it.
- **≤760px:** the canvas hint text is dropped rather than truncated.
- **≤700px:** the settings header wraps to two rows.
- **≤540px tall (the 7" 800×480 kiosk):** everything compacts — HUD gap and padding to
  9px, drive pad to 150px, e-stop to 52px, **display numerals to 34px**. This breakpoint
  is keyed on *height*, not width, because the kiosk's problem is vertical room.

### Named Rules
**The Gap-Is-The-Map Rule.** The HUD's gutters are pass-through touch targets for the
terrain. Never fill a gap with a background.

**The Stop-Is-Never-Covered Rule.** The e-stop dock sits at `z-index: 1001`; the settings
sheet at `1000`. Any new full-screen surface goes *under* the e-stop.

**The Thumb-Floor Rule.** `--tap: 52px` is the minimum height for any primary control,
and it does not shrink on the kiosk breakpoint.

```
"top   top   top "
"rail  .     .   "
"rail  cmd   dock"
```

**Cards lift; slots sink.** A floating surface is warm translucent paper with a
hairline, `backdrop-filter: blur(20px) saturate(1.15)`, and a soft offset drop shadow in
ink rather than black — a black shadow on a light ground reads as dirt. A recessed
surface is flat opaque Cut Well with a hairline and no shadow.

The saturation boost is lower than a dark system needs: paper over satellite terrain
already reads as high contrast, and over-saturating turns the map into noise behind the
numbers.

### Shadow Vocabulary
- **Card lift** (`0 14px 34px -20px rgba(20,22,15,0.36)`): every floating paper panel.
- **Pad lift** (`0 18px 40px -18px rgba(20,22,15,0.42)`): the drive pad — the object
  closest to the operator.
- **Sheet lift** (`0 -14px 40px -22px rgba(20,22,15,0.34)`): the bottom-sheet rail in
  portrait, shadow cast upward.
- **Cap** (`0 6px 14px -4px rgba(20,22,15,0.4), inset 0 1px 2px rgba(255,255,255,0.6)`):
  the drive nub and slider thumb — a raised physical cap, lit from above.
- **Alarm** (`0 10px 30px -8px rgba(212,25,42,0.5)`): the e-stop's bloom, animating on
  the 1.1s armed pulse.
- **Flag ring** (`0 0 0 3px rgba(<flag>, 0.18)`): a lit indicator wears a soft ring
  rather than a glow — a glow on paper looks like a printing error.

### Named Rules
**The Paper-Not-Glare Rule.** Panel white is warm (`#fbfbf8`), never `#ffffff`. Pure
white next to a sunlit satellite tile is the thing that makes a field screen unreadable.

**The Float-Or-Sink Rule.** There is no flat surface in this system. A surface either
lifts (translucent paper + blur + offset shadow) or sinks (opaque Cut Well + hairline).
Does it present information, or does it receive input?

**The No-Glow Rule.** Nothing glows. This is printed matter in daylight; a coloured halo
is the one device that would break the material. Liveness is carried by a flag-coloured
dot with a soft ring, and by motion.

Below 820px or in portrait the rail becomes a bottom sheet, and the command dock
is **removed rather than shrunk** — it is a fleet-wide readout, and that layout
belongs to the panel where the operator is standing next to one rover.

**A nested radius scale, each step smaller as you go inward.** 26px on floating panels,
16px on controls sitting on them, 12px on inputs inside those, and 9px on the smallest
inner elements. The e-stop is the one deliberate exception at 18px.

Fully round (`999px`) carries two silhouettes: **control clusters** — the topbar nav,
segmented groups, anything that reads as one object with internal divisions — and
**status pills and chips**. A rectangle with a large radius is a container; a lozenge is
either a state or a group.

Circles carry the physical controls: the drive pad is a 190px circle (150px on the
kiosk) with a dashed inner ring and a crosshair marking the zero the nub returns to. The
74px nub, the 26px slider thumb, the 24px toggle knob and the 8–10px flag dots are all
circles. **Round means it moves or it reports.**

### Named Rules
**The Nested-Radius Rule.** A child's radius is always smaller than its parent's.
26 → 16 → 12 → 9. Never place a 26px-radius element inside a 16px one.

**The Cluster-Is-A-Capsule Rule.** Controls that act on the same subject live inside one
`999px` capsule on Raised Paper, with the active member filled. Separate capsules mean
separate subjects.

## Components

### Buttons
- **Shape:** 16px radius, 52px minimum height, 15px 650-weight label.
- **Default:** Raised Paper fill, hairline border, Ink text.
- **Active / selected:** **Ink** fill with paper text — the strongest inversion in the
  system, for a control that is simply on.
- **Accent:** Course Lime fill with ink text. Reserved for the operator's current
  subject — the selected robot, the engaged mode.
- **Ghost:** transparent fill, hairline retained. Secondary actions inside a panel.
- **Danger:** Flag Red text on a 9% red wash; on press the fill goes fully red with
  white text — the action commits visually before it commits.
- **Small:** 38px height, 13px label, for toolbars and inline actions.
- **Press:** `scale(0.96)` over 0.25s on `--ease`. Not a hover system.

### E-STOP (signature component)
The single most important object on screen. A 60px-tall (52px on the kiosk) red vertical
gradient, 18px radius, white 800-weight 17px label tracked to `0.10em`, in its own dock
at `z-index: 1001` — above the settings sheet, above everything. When a robot is latched
it takes `.armed` and breathes on a 1.1s pulse. It is the only element permitted a large
saturated fill, and it is never scrolled, covered or collapsed. On paper it is *more*
prominent than it was on black, which is correct.

### Pit-board readout (signature component)
A display numeral with a tracked micro-label above it, set in mono when the robot
measured it. 56px at 800 weight, `-0.045em`, dropping to 34px on the kiosk breakpoint.
Three at most on screen. This is the component that carries the whole identity: it is
why the panel is paper and why the type is Archivo.

### Drive pad (signature component)
A 190px circular paper instrument with a lime radial bloom at 50%/38%, a dashed guide
ring, a crosshair marking zero, and a 74px gradient lime nub. The nub eases with
`--ease-out` at 0.18s when releasing to center, and has `transition: none` while
`.active` — the thumb's position is reported with zero lag, because any easing here is
the interface lying about the robot's throttle.

### Cards / Containers
- **Panel:** 26px radius, Paper Glass, hairline, blur + saturate, card lift, 14px padding.
- **Robot card:** 16px radius on Raised Paper; the selected one takes a **full Course
  Lime fill** with ink text, rather than a tinted border. On a pit board the car you are
  calling is marked, not outlined.
- **State / actuator cards:** 16px radius on Raised Paper. State is carried by a
  flag-coloured dot and a tinted fill, never by a thick coloured edge.

### Inputs / Fields
- **Style:** Cut Well fill, hairline, 12px radius, 38px height, mono 14px, right-aligned
  for numbers (92px fixed width, spinners stripped).
- **Focus:** border becomes Ink, outline removed. Nothing else moves.
- **State:** a pending, confirmed or rejected field tints its own **fill** with an 8%
  flag wash and shows a flag dot at its right edge. A column of fields reads as one
  status channel without a single coloured bar.
- **Unknown:** `opacity: 0.5` — a value the server has never reported is shown, but
  visibly not real.
- **Toggle:** 56×32 well that fills Ink when on; a 24px paper knob translates 24px.
- **Segmented:** a `999px` well capsule containing pill buttons; the on-state takes Ink.
- **Range:** a 5px well track with a 26px lime cap thumb on a 34px hit area.

### Navigation
There is no nav in the operating view — the map *is* the view, and settings is a
full-screen sheet reached from one gear in the topbar capsule. Inside settings, a **tab
strip** is a `999px` well capsule containing pill buttons; the active tab takes Ink with
paper text.

### Status indicators
- **Pill:** 999px lozenge on Raised Paper, 12px 650-weight, optionally containing an 8px
  flag dot. `.quiet` switches to mono 11.5px in Pencil.
- **Dot:** a 10px circle — Flag Green with a soft ring when online, Flag Red flat when
  offline, Faint Pencil when unknown. The ring is the tell.
- **Pips:** three 5×11px rounded bars for IMU calibration 0–3.
- **Bars:** 7px well tracks with a flag-coloured fill on a 0.12s *linear* transition.
  Linear is correct here and nowhere else: this is a live measurement.
- **Badge:** 9px uppercase tracked `0.14em`, hairline-strong border, 6px radius. The
  `restart` badge takes amber.

### Routine canvas (signature component)
An SVG node graph on a dotted grid. Nodes are rounded Raised Paper rectangles with a
mono 13px id; the stroke encodes role — Ink for start, Flag Green for terminal, Flag Red
for a problem, Ink at 2px when selected. **The live node is filled Course Lime**, because
across a bench you want a block of colour, not a 2px border. Wires carry a 20px
transparent hit stroke behind the 1.75px visible line.

## Do's and Don'ts

### Do:
- **Do** float every panel on warm paper (`rgba(251,251,248,0.82)` + `blur(20px)
  saturate(1.15)` + an offset ink shadow). The terrain shows through the card.
- **Do** sink anything that receives input into an opaque `#e7e8e1` well.
- **Do** set every number a robot reported in JetBrains Mono, and every word a human
  wrote in Archivo.
- **Do** reserve Course Lime for the operator's current subject, and flag colours for
  the machine's condition.
- **Do** keep every primary control at or above `--tap: 52px`, including on the kiosk.
- **Do** group controls acting on one subject inside a single `999px` capsule.
- **Do** cap the screen at three display numerals.
- **Do** use `--ease` for state changes and `--ease-out` for things settling.
- **Do** use `linear` transitions *only* for live measurements.
- **Do** nest radii inward: 26 → 16 → 12 → 9.
- **Do** honor `prefers-reduced-motion`.
- **Do** bundle every font and tile asset locally. There is no network in the field.

### Don't:
- **Don't** use pure `#ffffff` for a surface. Warm paper only — white is glare.
- **Don't** let a flag colour and Course Lime appear in the same role. Fluorescent is
  *which one*; flag colour is *how it is*.
- **Don't** set Course Lime as type, a lone border, or a glow. It is a fill.
- **Don't** give anything other than the E-STOP a large saturated red fill.
- **Don't** add a glow, halo or neon edge anywhere. This is printed matter in daylight.
- **Don't** carry state on a thick coloured left border. State tints the fill and shows a
  flag dot.
- **Don't** put more than three display numerals on one screen.
- **Don't** let any element cover, scroll or collapse the e-stop dock.
- **Don't** rely on hover. The primary device is a touchscreen with no pointer.
- **Don't** fill the HUD's gutters with a background.
- **Don't** add a parallel 4/8/12/16 spacing scale — the system runs on 3/6/9/14/16.
- **Don't** let this read as a consumer smart-home app (pastel cards, friendly
  illustrations), an enterprise SaaS dashboard (KPI tiles, nav sidebar, grey-on-white
  charts), or a gamer/RGB aesthetic. All three are confirmed rejections.
