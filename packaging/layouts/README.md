# Fleet layouts

One document per rover, saying what that build HAS — see `robot/layout.py`.

| File | Rover | Channel 2 | Extra mechanisms |
|---|---|---|---|
| `east.json` | east.local | `dumper`, runs at -30 | — |
| `shooter.json` | rover 2 | `dumper` (labelled *Flywheel*), runs at -35 | `feeder` ch4, `agitator` ch5 |

Rover 2's right track runs faster than its left, so it carries a drivetrain
**trim**: `speed_scale_forward=0.75`, `speed_scale_reverse=0.8` on `right`. The
two directions are separate numbers because the mismatch is worse going forward
than backing up — trim each against the way the rover actually tracks in that
direction. Scale the FASTER side *down*; the slower one is already being asked
for everything it has.

Both share the drivetrain (ch0 left, ch1 right) and an `intake` on ch3. Every
motor is neutral at **5**, the Fusion HAT angle that stops it fully and the
pulse that keeps an ESC armed.

Edit `build_layouts.py`, then:

```
just layouts                                    # regenerate + validate
just push-layout packaging/layouts/east.json    # deploy to $ROBOT_HOST
just host=bot2.local push-layout packaging/layouts/shooter.json
```

The `.json` files are generated — change the script, not them.

## Two things that will bite

**Channel 2 is either a mechanism or the built-in shooter, never both.**
`layout.apply` reserves the shooter's channel when `RS_SHOOTER_ENABLED=1`, and a
mechanism on it is a validation **error**.

That is not a soft failure, whatever the message says. The error text reads
*"'dumper' is disabled"*, but an error makes the whole document invalid, and
`apply` installs nothing when validation fails — so the rover boots on the
compiled-in two-motor defaults with **no intake, no feeder, no agitator and no
drivetrain trim**. It drives, and it veers.

So the two rovers differ here, and each is internally consistent:

| | `east.json` | `shooter.json` |
|---|---|---|
| ch2 | `dumper` mechanism | *(empty)* |
| `RS_SHOOTER_ENABLED` | `0` | `1` |
| what drives ch2 | the mechanism, at a fixed throttle | the built-in shooter, holding an RPM |

Only the built-in shooter can hold a *speed* — the closed-loop controller lives
in `robot/drive/shooter.py`, and a mechanism can only hold a throttle. A dumper
has nothing to hold a speed against, which is why east keeps the mechanism.

After any change here, check `just logs` for the `layout:` line.

**Angles are reached through a symmetric throw.** The usable throw is the
*narrower* side of neutral, so `min_angle`/`max_angle` are what decide whether a
preset can reach the angle you want — pushing a preset past ±1.0 only clamps.
With neutral 5, endpoints -30/+40 give a throw of 35 and ±1.0 lands exactly on
both. Widening one endpoint alone does nothing; widen the side you need and
check the other still reaches.

**An endpoint past ±45 is a dead motor, not a faster one.** The HAT maps
-90…+90 onto a 500–2500 µs pulse, because that is a *servo's* range. An ESC
listens to 1000–2000 µs only and reads anything outside it as a lost signal: it
cuts the motor and re-arms — the beep — partway through the throw. That is what
`shooter.json`'s flywheel did at -50 (944 µs): spun up over the ramp, then died
about a second in with the ESC's startup tune. So ±45 is the whole of what is
available, and with neutral 5 the widest legal throw is `min(45-5, 5+45)` = 40.
`robot/layout.py` warns about any actuator that breaks this, and
`tests/test_fleet_layouts.py` checks every preset on both rovers.

## Why the channel-2 mechanism is called `dumper` on both

The gamepad binding addresses a mechanism by NAME, and the mapping is one per
base station rather than one per rover. Sharing the name is what makes the same
button work the channel-2 motor on whichever rover is selected. `label` is what
the dashboard shows, and that is where the two builds differ. Rename it on both
rovers together, or the button stops reaching one of them.
