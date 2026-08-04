# 5 · Describe your hardware

*Settings → Hardware. What the robot HAS, written from the dashboard.*

Nothing about your build is compiled in. The **layout document** says which
drivetrain you have, how many motors and servos are fitted, and what the rest of
them are grouped into. Change it from the tab, save it to the robot, restart,
and the robot is a different machine.

![The Hardware tab: a robot picker with Refresh and Saved buttons, a Drivetrain selector reading Tank — left/right tracks, chip pickers for the left and right sides, slew rate and arm hold fields, and the first motor card.](../img/hardware.webp)

The tab is a draft you edit locally and send in one go — unlike the tuning page
next door, which applies each field as you change it. A layout is a **tree**, and
half a tree is a robot with one drive motor.

## 1 · Pick the drivetrain

| Kind | What it is | What you then assign |
|---|---|---|
| `tank` | Two sides, skid steer | Which motors are on the *left* side and which on the *right*. Every motor on a side gets that side's track speed, so a six-wheeler is three names on each list. |
| `servo_steer` | Drive motor plus a steering servo | The drive motors, the steering servo, a *steering gain* and a *pivot creep*. |
| `single` | One motor, no steering | The drive motors only. |
| `none` | No drivetrain at all | Nothing — for a fixed platform that is all mechanism. |

> **A steered chassis cannot pivot on the spot**
>
> …and object align and waypoint mode both ask it to. **Pivot creep** is what
> makes that survivable: it creeps forward so the steering bites. Set to zero,
> the rover will steer at its target without ever moving.

## 2 · Add a motor

Press **+ Motor** at the bottom of the drivetrain card. You get a fresh actuator
card with the fields below; fill them in and it exists.

![An actuator card for a motor named left: type ESC (spins), PWM channel 0, an Inverted checkbox, neutral angle 5, forward endpoint 20 and reverse endpoint -20, each with explanatory help text.](../img/hardware-motor.webp)

*An existing motor. Every field carries the sentence you would otherwise have
had to find in the source.*

![A newly added motor card with a default name, ready to be renamed and given a channel.](../img/hardware-motor-new.webp)

*A motor added just now. It starts on a free channel and a safe neutral, so an
unfinished card cannot arm anything.*

**Name**
: Lower case, no spaces. It is also this actuator's tuning path and how a
routine refers to it, so `intake_motor` is worth more than `m3`.

**Type**
: **ESC (spins)** is held at neutral at boot so it arms; **Servo (holds a
position)** simply parks. Pick wrong and an ESC will refuse to arm.

**PWM channel**
: The Fusion HAT channel, `0–15`. Claiming a channel twice is *refused* —
silently making two motors move together is not a thing this page will do to
you.

**Inverted**
: For a motor mounted facing the other way. On a tank chassis the right side is
almost always mirrored, so this is almost always ticked on one side.

**Neutral angle**
: Where *this* ESC holds stop. Find it with `tools/esc_calibrate.py`; do not
assume it is zero.

**Forward / reverse endpoint**
: The ends of the throw. The throw is **symmetric about neutral**, so whichever
endpoint is closer to neutral sets the usable range — which is what keeps a
normal and a mirrored motor starting together and matching speed even when
neutral is not centred.

> **Order matters**
>
> Add the motor here first, then assign it to a side in the drivetrain card
> above — the *Left side* and *Right side* pickers list the actuators that
> exist, so a motor you have not created yet cannot be assigned.

### The encoder, if this motor has one

Press **Add** in the Encoder row at the bottom of the card. Optional, and off on
every motor until you do — a motor without one simply runs open-loop, exactly as
the rover always has.

Why bother: everything else on this page commands a *throttle*, and a throttle
is a wish rather than a speed. Two motors handed the same pulse turn at
different rates — different ESCs, different gearbox friction, weight off centre,
one track on grass — so the rover drives a slow arc while every number in the
dashboard reports a straight line. An encoder is the only thing that can see
that, and [Wheel speed matching](tuning.md#making-both-tracks-turn-together) is
what then corrects it.

**A pin / B pin**
: **BCM GPIO numbers on the Pi header — not the PWM channel above.** They are
different buses, and mixing them up is the first mistake everyone makes here; it
presents as an encoder that counts nothing at all. Set both or neither: one
channel of a quadrature encoder decodes nothing, and the robot refuses a layout
that sets only one. Claiming a pin twice is refused for the same reason a PWM
channel is, and a worse one — two actuators reading one pin count the same edges,
so a rover with one genuinely dragging track would report both wheels at exactly
the same speed.

**Counts per rev**
: Counts per revolution **of the wheel**, gearbox included. *Measure this, do not
derive it* — the number on the encoder is per revolution of the motor, this
decoder counts four edges per cycle, and the printed gear ratio is frequently
not the real one. Run `python tools/encoder_monitor.py --pins 17,27`, zero it,
turn the wheel exactly one full turn by hand, and read the count.

**Count inverted**
: Tick it if a wheel counts *down* when driving forward. Separate from
**Inverted** above: that one mirrors the motor, this one mirrors the sensor, and
a mirrored track motor usually needs both.

The pins are read through `fusion_hat`, the same library that drives the motors,
so there is no separate GPIO package. It does need `rpi-lgpio` in place of the
stock `RPi.GPIO`, which cannot arm interrupts on a current kernel — one command,
`just encoder-gpio`, and [Wiring and calibration](../reference/wiring.md) has the
why.

## 3 · Group the rest into mechanisms

Anything that is not drive is a **mechanism**: an intake, an arm, a launcher. A
mechanism owns one or more actuators and comes in three kinds.

**Powered**
: Holds a value until told otherwise — an intake that spins, an arm that holds
an angle. Give it named **presets** like `in`, `out` and `stow`, so a routine
asks for *"intake → in"* rather than a column of numbers. *Auto-stop* ends the
motion after N seconds; `0` runs until stopped.

**Pulse**
: Owns a cycle: rest angle, active angle, active time, recover time, cooldown
and a magazine count. Asking it to fire repeatedly still yields one activation
per cycle, which is what makes a launcher safe to wire to a condition that stays
true.

**Sequence**
: A queue of steps run **one after another**. This is the kind for a shooter
whose actuators cannot all move at once — spin the flywheel, *then* push the
ball in, *then* run the belt. It is covered in full below.

![A mechanism card named intake of type Power, with an Enabled checkbox, an auto-stop field, two named presets on and off each mapping intake_motor to a value, an Add preset row, and Test controls reading Reverse and Forward.](../img/hardware-mechanism.webp)

A powered mechanism with two presets and its own actuator. The **Test** row jogs
it from the bench — and is refused unless the robot is in teleop with no e-stop
latched, because a bench test that runs while a routine owns the motors is how a
hand ends up in an intake.

## 3a · Sequence a mechanism whose parts move in turn

Both of the kinds above write every actuator at the same instant. A launcher
with a feeder servo, a flywheel and a belt on it needs the opposite: an order.
Press **+ Sequence mechanism**, add the actuators, then add a step per stage.

Each step has three parts.

**What it moves.**
: Tick an actuator to include it, and give it a value. The units are the
actuator's own — **degrees** for a servo, **throttle** (−1 to 1) for a motor —
so a step reads the way the build does.

: An actuator you *do not* tick is **left exactly as it was**. This is the whole
point, and the one thing to get straight: the flywheel started in step 1 keeps
spinning through step 2, which is what lets the feeder push a ball into a wheel
that is already up to speed. (Presets work the other way round, zeroing what
they do not name. If you want that here, tick *"stop everything this step
doesn't name"*.)

**How long it holds.**
: *Hold for at least* is a **floor, not a duration**. The step ends when that
time has passed *and* its condition is satisfied. Leave the condition off and
the floor is all there is, which is an ordinary timed sequence.

**How fast it gets there.**
: *Ramp over* is acceleration, and it answers a different question from the one
above: not how long the step holds, but how long the actuators take to **arrive**.
Leave it at 0 and the step writes its values instantly, which is what every step
did before this field existed. Set it, and each actuator the step names is walked
from where it actually was to its target over that many seconds — a flywheel
eased up to speed instead of slammed there, a feeder arm that arrives at the ball
instead of hitting it.

: It is per-step because a shooter wants both in one cycle: a long, gentle
spin-up, then a feeder that must move *now*, before the wheel bleeds speed.

: **Deceleration is a ramp downward** — a step whose target is lower than the
current value, which the same linear walk handles in either direction. A shooter
that should wind down rather than cut out ends with `flywheel 0` over 1 s. Note
that this is the *only* way to get a soft stop: e-stop, a mode change and
shutdown all park instantly and always will, because a mechanism that eased
itself down over a second is one that ignores the button for a second.

: The ramp counts as part of the hold: a step cannot hand over while its own
actuators are still travelling, so the effective floor is whichever of *hold for
at least* and *ramp over* is longer.

**What else it waits for.**
: *"A motor reaching a speed"* is the one a shooter wants. Timing a spin-up
works at one battery charge and fires early at every other; waiting for the
encoder to actually read 3 000 rpm works at all of them. It needs encoder pins
on that actuator — the robot refuses the layout otherwise, rather than shipping
a gate that can never open.

: *"Another mechanism being ready"* waits for a different mechanism to go idle,
which is how two mechanisms hand off to each other.

A gate that never opens would leave the flywheel at full throttle forever, so
every gated step has a ceiling — its own *give up after*, or the mechanism's
**step timeout**. Reaching it **stops the whole sequence** by default and parks
everything, because carrying on is exactly the jam the gate was written to
prevent. Switch it to *carry on anyway* only when the wait is an optimisation
rather than a safety condition.

The finished thing for the three-actuator launcher:

| # | Step | Moves | Ramp | Holds ≥ | Waits for |
|---|------|-------|------|---------|-----------|
| 1 | spin up | flywheel `1.0` | 1.2 s | 0.2 s | flywheel ≥ 3000 rpm |
| 2 | feed | feeder `40°` | 0.35 s | 0.25 s | — |
| 3 | advance | belt `0.8` | 0.2 s | 0.6 s | — |
| 4 | spin down | flywheel `0` | 1.0 s | — | — |

Start it from a routine with **start a sequence**. It returns immediately and
advances itself off the control loop, so nothing here ever blocks driving —
asking again while it runs does nothing, so it is safe on a button that stays
held. To wait for it to finish, transition on **mechanism is ready**, which stays
false for exactly as long as it is running.

> **A sequence cannot go straight onto a gamepad button**
>
> The **mechanism preset** button slots bind a *preset*, and only `power`
> mechanisms have presets — a sequence has steps. Binding one to a preset slot
> gets you `preset refused: 'launcher' is not a powered mechanism` on the
> robot's console and nothing else.
>
> To fire one from the pad, put it in a one-state routine (**start a sequence**)
> and bind *that* to a **routine** button. Give the state `drive: teleop` and you
> keep stick control while it fires. See [Routines](routines.md).

> **Every way of stopping parks it**
>
> E-stop, a mode change and shutdown all park a half-finished queue: servos to
> the rest angle, motors to stop. A sequence never resumes from the middle — the
> next activation starts at step 1.

## 4 · Save it

Press **Save to robot**. The layout is sliced into numbered fragments, and
*nothing is applied until every fragment arrives* — then the robot validates the
whole document, stores it, and echoes the stored copy back, because the
validator clamps and what was saved is not always what was sent. Errors come
back naming the state or field that was wrong.

> **A saved layout takes effect on the next start**
>
> Actuators are built by constructors; rebuilding them mid-loop with the
> drivetrain armed is how an ESC ends up holding an undefined pulse. Press
> **Restart robot** at the top of this tab — it asks twice, then the rover
> parks its motors, drops off the fleet for a few seconds and comes back
> running the new layout. `just restart`, `sudo systemctl restart
> roversoftware-robot` and a power cycle all still work, and are what you need
> if the rover was started by hand rather than as a service: it refuses to stop
> when nothing would start it again.

Before you trust any of it, calibrate the ESCs —
[Wiring and calibration](../reference/wiring.md) is the wheels-off-the-ground
procedure.
