# Fleet layouts

One document per rover, saying what that build HAS — see `robot/layout.py`.

| File | Rover | Channel 2 | Extra mechanisms |
|---|---|---|---|
| `east.json` | east.local | `dumper`, runs at -30 | — |
| `shooter.json` | rover 2 | `dumper` (labelled *Flywheel*), runs at +50 | `feeder` ch4, `agitator` ch5 |

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

**Channel 2 needs the built-in launcher off.** Both rovers put a *mechanism*
there, and `layout.apply` reserves the shooter's channel when
`RS_SHOOTER_ENABLED=1`. With it on, the mechanism loses the tie, is disabled,
and the rover boots fine while that button does nothing. Set
`RS_SHOOTER_ENABLED=0` (`just config`).

**Angles are reached through a symmetric throw.** The usable throw is the
*narrower* side of neutral, so `min_angle`/`max_angle` are what decide whether a
preset can reach the angle you want — pushing a preset past ±1.0 only clamps.
With neutral 5, endpoints -30/+40 give a throw of 35 and ±1.0 lands exactly on
both. Widening one endpoint alone does nothing; widen the side you need and
check the other still reaches.

## Why the channel-2 mechanism is called `dumper` on both

The gamepad binding addresses a mechanism by NAME, and the mapping is one per
base station rather than one per rover. Sharing the name is what makes the same
button work the channel-2 motor on whichever rover is selected. `label` is what
the dashboard shows, and that is where the two builds differ. Rename it on both
rovers together, or the button stops reaching one of them.
