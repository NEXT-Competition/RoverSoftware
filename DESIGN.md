---
name: RoverSoftware Base Station
description: A mission-control plot table — the terrain is the only lit thing in the room, the instruments are machined dark plates pulled back around it, and one signal teal marks the rover you are calling. Re-renders as paper at full contrast when the panel goes outdoors.
colors:
  bg: "#07090a"
  paper: "#0f1416"
  panel: "rgba(15, 20, 22, 0.88)"
  panel-solid: "#0f1416"
  panel-2: "rgba(30, 38, 41, 0.72)"
  well: "#171e20"
  hairline: "rgba(150, 180, 184, 0.14)"
  hairline-strong: "rgba(150, 180, 184, 0.30)"
  ink: "#e9f1f1"
  text: "#e9f1f1"
  muted: "#9aacae"
  faint: "#7d9094"
  accent: "#2ee6c5"
  accent-deep: "#17c8a8"
  accent-lift: "#86f6e0"
  on-accent: "#04211c"
  data: "#f0a53a"
  chart-1: "#3987e5"
  chart-2: "#d95926"
  chart-3: "#199e70"
  ok: "#35d68a"
  warn: "#f5b43f"
  danger: "#ff4256"
  on-danger: "#1a0206"
  on-ok: "#04211c"
  map-label-bg: "rgba(6, 10, 11, 0.72)"
  map-label-ink: "#f2f7f7"
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
  panel: "14px"
  control: "10px"
  input: "8px"
  inner: "6px"
  pill: "999px"
  chamfer: "13px"
spacing:
  hair: "3px"
  xs: "6px"
  sm: "9px"
  md: "14px"
  lg: "16px"
components:
  panel-plate:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.ink}"
    rounded: "{rounded.panel}"
    padding: "14px"
  faceplate:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.ink}"
    rounded: "{rounded.panel}"
    chamfer: "{rounded.chamfer}"
    padding: "7px 7px 7px 15px"
  readout-field:
    backgroundColor: "{colors.well}"
    textColor: "{colors.ink}"
    rounded: "{rounded.input}"
    padding: "0 11px"
    height: "38px"
    width: "92px"
    typography: "{typography.readout}"
  toggle-on:
    backgroundColor: "{colors.accent}"
    textColor: "{colors.on-accent}"
    rounded: "{rounded.pill}"
    width: "56px"
    height: "32px"
  chip:
    backgroundColor: "{colors.well}"
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "5px 12px"
  chip-on:
    backgroundColor: "{colors.accent}"
    textColor: "{colors.on-accent}"
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
  command-dock:
    backgroundColor: "{colors.panel}"
    textColor: "{colors.ink}"
    rounded: "{rounded.panel}"
    chamfer: "{rounded.chamfer}"
    padding: "10px 12px"
  place-pin:
    backgroundColor: "{colors.accent-deep}"
    textColor: "{colors.on-accent}"
    rounded: "{rounded.pill}"
    width: "26px"
    height: "26px"
---

# Design System: RoverSoftware Base Station

## Overview

**Creative North Star: "The Plot Table"**

A plot table is the lit surface at the centre of a control room. The terrain is
the only thing on it that glows; everything else — the consoles ranged around it,
the faceplates, the readouts — is machined, dark, and deliberately recessive, so
nothing competes with the one thing everybody in the room is looking at.

That is this console. The map is full-bleed and live, and it *is* the lit object:
the tile layer is pulled down just far enough that the plates over it read, and
no further. The instruments above it are **milled dark plates** — a 1px lit top
edge, tight radii, an offset shadow — not sheets of glass floating on a blurred
gradient. Every number on them is a **measurement**, set in mono with tabular
figures, because a column of readings that jitters as digits change is a column
nobody can read.

The signal teal is the tape. It marks **the rover you are calling** — the
selected robot's card, the mode it is in, the nub under your thumb — and does
that job alone. Amber is reserved for **a plan**: routes, pending waypoints, a
leg you have drawn but not yet driven. Everything the machine reports about its
own condition speaks in flags: green, amber, red. Teal is *which one*; amber is
*what you asked for*; flag colour is *how it is*. The three vocabularies never
overlap, so an operator never has to work out which question a colour is
answering.

**Two renditions, one identity.** Dark is the default, because the console is
read indoors and in a pit. But the primary panel is a 7" screen used **outdoors
in direct sun**, where a dark ground is exactly wrong — sunlight on glass beats
any dark UI. `[data-theme="daylight"]` re-renders the same identity on paper at
full contrast, and its toggle sits in the top bar rather than behind Settings,
because it is a response to where the operator is standing rather than a
preference they set once. Same accent, same geometry, same grammar; only the
light changes. The choice is remembered per device — the kiosk that lives
outdoors and the laptop in the pit each keep their own.

**Key Characteristics:**
- Full-bleed live map as the lit object; tiles dimmed, markers never
- Machined plates: 14px panels, a 1px lit top edge, an offset shadow
- One chamfered corner as the signature — top cluster and command dock only
- One signal teal for selection, amber for plans, three flags for state
- Poster-scale numerals for what an operator reads at distance
- Two type voices: Archivo for words, JetBrains Mono for anything measured
- 52px minimum tap target everywhere, at every size, in both renditions
- Motion with mass; nothing linear, nothing decorative

## Colors

A graphite field with one signal marker, one plan colour, and three flag signals.

### Primary
- **Signal Teal** (`{colors.accent}`): the tape. Selection and engagement, and
  nothing else — the selected robot's card, the active mode, the drive nub, a
  toggle that is on. If something is teal, it is the thing you picked or the
  thing you are commanding.
- **Deep Teal** (`{colors.accent-deep}`) / **Teal Lift** (`{colors.accent-lift}`):
  the two ends of the gradient on physical-feeling teal objects — the drive-pad
  nub, the slider thumb, a saved place's pin — lit from above so they read as a
  raised cap.
- **Ink** (`{colors.ink}`): near-white with a cyan cast, so it sits *with* the
  teal rather than against it. Body text and every readout.

### The plan
- **Plan Amber** (`{colors.data}`): a route drawn but not yet driven, a pending
  waypoint, a leg on the map. Amber here means **drawn**, never **wrong** —
  which is why it is a separate token from Flag Amber even where the hues are
  close.

### Secondary — the flags
- **Flag Green** (`{colors.ok}`): alive and reporting.
- **Flag Amber** (`{colors.warn}`): pending or restart-gated. "Not yet," never
  "wrong."
- **Flag Red** (`{colors.danger}`): stopped, refused, offline, invalid. The only
  colour permitted a large saturated fill, and only on the e-stop.

### Chart series
`{colors.chart-1}` / `{colors.chart-2}` / `{colors.chart-3}`, assigned in that
fixed order and never cycled — today the P, I and D contributions on the PID
tuning graphs. They are deliberately none of the above: the accent means "the
rover you are calling", Plan Amber means "drawn", and the flags mean robot
state. A data series that borrowed any of them would claim a meaning it does not
have.

Both renditions are stepped for their own surface rather than lightened from
each other, and the set is validated for colour-vision deficiency across **every
pair**, not just neighbouring ones, because three lines share one plot. Adding a
fourth slot means re-validating, not picking a fourth nice colour. Everything a
chart can say structurally it says structurally: a reference line is dashed and
muted, a total is ink, and a difference between two lines is the shaded gap
between them — hue is spent only on identity that has nowhere else to live.

### Ink on fills
`{colors.on-accent}`, `{colors.on-danger}` and `{colors.on-ok}` exist because the
two renditions invert what belongs on a fill: a bright flag on a dark ground
carries **dark** ink, and its deep daylight twin carries **white**. A fill never
has to guess what sits on it.

### Map labels do not theme
`{colors.map-label-bg}` / `{colors.map-label-ink}` are fixed across both
renditions. A marker label sits on satellite imagery, which is bright either way
— a label that inverted with the ink would be dark-on-dark half the time.

## Typography

Two voices, and the split is strict.

- **Archivo** for anything a person wrote: labels, buttons, prose, headings.
- **JetBrains Mono** for anything a robot measured: coordinates, throttle, link
  age, counts, ids. With `tabular-nums` wherever the value updates live, so a
  changing digit never shifts the ones beside it.

Mono is never a costume for "technical"; it earns its place only on data.
`{typography.display}` belongs to the readouts an operator reads at arm's length,
never to a brand wordmark.

## Layout

A CSS grid over a full-bleed map, transparent to pointers except where a plate
sits:

```
"top   top   top "
"rail  .     .   "
"rail  cmd   dock"
```

- **top** — the chamfered cluster: brand, fleet health, link state, rendition
  toggle, settings.
- **rail** — the instrument stack, 320–360px, with its own scroll.
- **cmd** — the command dock: every rover's current activity, and where a spoken
  order will land.
- **dock** — the drive pad, bottom-right, under the thumb.
- The e-stop lives outside the view switch entirely, at `z-index: 1001`.

Below 820px or in portrait the rail becomes a bottom sheet, and the command dock
is **removed rather than shrunk** — it is a fleet-wide readout, and that layout
belongs to the panel where the operator is standing next to one rover.

## Elevation & Depth

On a dark ground the **lit top edge does the work the shadow does on paper**.
Every plate carries `--inset-hi` (a 1px highlight along its top) plus an offset,
blurred shadow. A zero-offset coloured halo is decoration and is not used.

`--lift-card` for plates, `--lift-pad` for the drive pad, `--lift-cap` for the
small physical caps — nubs, pins, thumbs.

## Shapes

Machined, not moulded: `14 / 10 / 8 / 6`, plus a 999px pill for chips and status
capsules.

**The chamfer is the signature.** A 13px cut across one corner turns a plate into
a faceplate. It appears on the **top cluster** and the **command dock** — the two
pieces that frame the console — and nowhere else. A notch on every panel would be
a texture rather than a mark.

## Components

Exact values live in the frontmatter. The rules that are not obvious from them:

### E-STOP (signature component)
The single most important object on screen, and **the one control that does not
theme**: same red, same place, same weight in every rendition and every light. A
60px-tall (52px on the kiosk) red vertical gradient in its own dock at
`z-index: 1001` — above the settings sheet, above everything. When a robot is
latched it takes `.armed` and breathes on a 1.1s pulse. Never scrolled, covered
or collapsed. An operator reaching for it is not reading the screen.

### Plot readout (signature component)
A display numeral with a tracked micro-label above it, in mono when the robot
measured it. 56px at 800 weight, dropping to 34px at the kiosk breakpoint. Three
at most on screen.

### Drive pad (signature component)
A 190px circular instrument with a teal radial bloom, a dashed guide ring, a
crosshair marking zero, and a 74px gradient teal nub. The nub eases home on
release, and has `transition: none` while `.active` — any easing there is the
interface lying about the robot's throttle.

### Command dock
Carries a disabled input and an honest "not wired up" badge. It looks like
somewhere you can talk, so it has to say plainly that nothing is listening yet:
an input that silently drops what is typed into it is the thing somebody talks at
while a rover is moving. Everything else on it is real telemetry — no placeholder
rows, no invented status.

### Places and routes
A **place pin** is teal and round; a **route waypoint** is amber and numbered.
A saved place is a fact about the field that outlives the match; a route point is
this run's plan. They must never share a colour.

### robot-card-selected
A full teal fill, and the loudest thing in the rail on purpose: "which rover am I
about to command" is the question the rail exists to answer.

## Do's and Don'ts

**Do**
- Add every new token to **both** renditions in the same edit.
- Use `rgba(var(--*-rgb), a)` for alpha ramps, never a literal rendition colour.
- Spell out state in words beside any coloured dot — safety state never rides on
  colour alone.
- Keep tap targets at 52px in both renditions and at every breakpoint.

**Don't**
- Don't put teal on anything that is not the operator's current selection.
- Don't use amber for an error, or red for anything but stopped/refused/invalid.
- Don't add a chamfer to a third surface.
- Don't dim the map further to make a plate legible — fix the plate.
- Don't pick dark or light by category. It is picked here from the use scene, and
  that is why there are two renditions rather than one.
