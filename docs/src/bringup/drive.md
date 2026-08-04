# 3 · Drive a rover

*A gamepad on the base station, or the two touch joysticks in the browser.*

1. Tap a rover in the fleet list. Everything that follows applies to it.
2. Make sure it is in **Teleop**. The mode grid is under *Control* in the rail.
3. Drive it: hold the on-screen pads, or use a gamepad — **left stick for
   throttle, right stick for steering**.

**Plug the gamepad into the base station, not into the tablet.** It is read
there by the bridge process itself and its commands go straight out over the
radio. A pad connected to whatever machine is showing the dashboard does
nothing: the browser has no part in the physical-controller path, so a laggy
socket, a reconnect or a backgrounded tab cannot come between a stick moving and
a rover moving.

Both input paths are two-stick, and they match on purpose: left joystick is
throttle only, right joystick is steering only, so each can be held and trimmed
independently. (Prefer the old trigger throttle? Clear the throttle axis in
**Settings → Controller** and the gamepad goes back to <kbd>R2</kbd> forward /
<kbd>L2</kbd> reverse, steering untouched.)
Both input methods produce a throttle and steer in `-1 … +1` and are rate-limited
to the bridge's `drive_hz`, which is one shared airtime budget. That is why a
physical pad and a tablet joystick feel identical, and why two operators cannot
flood the radio between them.

> **Failsafe**
>
> Teleop stops the robot if drive commands stop arriving (`command_timeout`).
> Walking out of radio range parks the rover rather than leaving it at its last
> throttle. E-stop overrides every mode until it is explicitly cleared.

## If the gamepad's buttons are wrong

They will be. Axis and button indices describe a *driver*, not a controller —
the same pad enumerates differently on macOS and Linux, over USB and over
Bluetooth. **Settings → Controller** remaps it by *pressing the control you
want*, with a live view of every axis and button and the throttle/steer the
current mapping produces.

![The Controller settings tab, showing gamepad mapping fields, dead zone, trigger rest value, throttle and steer authority, and steering inversion.](../img/settings-controller.webp)

Also here: the throttle and steer axes, dead zone, trigger rest value (a trigger
that idles at `-1` instead of `0` is normal), throttle and steer authority, and
throttle/steering inversion. This replaces editing constants and restarting a
service.

Sticks report **up as negative**, which is what *Invert throttle* is for — it
defaults on, and a rover that reverses when you push forward is that switch.

## The buttons a gamepad also carries

Beyond driving, the mapped pad can e-stop, clear, switch mode, arm or fire the
launcher, and work the mechanisms this build has. All of it is remappable on the
same page, and all of it goes through the same whitelist a spoken order does.

Mechanism controls come in two shapes, and the difference is deliberate:

| Shape | Controls | Why |
|---|---|---|
| Press once | intake *in*, dumper, shooter | They run for long stretches; holding a button through a match is not a thing anyone wants. |
| Run while held | intake *spit*, feeder, agitator | They clear a jam or feed a shot, so letting go is the stop — and only a held control can carry an auto-stop dead-man, because only a held control is being refreshed. |

The agitator is bound to a **hat** (the D-pad) rather than a button: one hat
reports a direction pair, so *up* runs it and *down* runs it backwards off a
single binding. `controller.hat_agitator` and `controller.hat_agitator_rev` both
name the hat's index — usually `0` — and the directions are fixed. That pair is
not in the Controller tab's bind-by-pressing flow, which only detects axes and
buttons; set them in `~/.config/roversoftware/basestation.json` if hat 0 is not
the one your pad reports.

A binding that names a mechanism this rover does not have is refused with a line
in its log and moves nothing, so one mapping can serve a fleet whose rovers are
not identically equipped.

A press-once control is sent as a bare message so the robot toggles from the
state it is actually in; a held one is sent as an explicit on/off and re-sent
several times a second, so a lost release frame costs a fraction of a second of
extra running rather than a mechanism nobody can stop.
