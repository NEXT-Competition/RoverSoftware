# The shape of the system

Two programs. **The robot** runs on a Raspberry Pi with a SunFounder Fusion HAT
and owns everything that moves. **The base station** runs on your laptop or a
second Pi, owns the radio, the gamepad and the map, and serves the touch
dashboard. They talk in newline-delimited JSON over one shared XBee channel,
addressed by robot id — so a whole fleet lives on one radio.

```text
                         XBee (JSON over serial)
 base station  <───────────────────────────────────────►  robot (Pi)
 (Pi / Mac)                                                │
 PS4 controller                                            ▼
 map + telemetry                                    ┌──────────────┐
 voice → local LLM                                  │ XBeeLink     │  reader thread → queue
 MCP → any AI                                       │              │
                                                    └──────┬───────┘
                                                           ▼
                                                    ┌──────────────┐
                                                    │ControlManager│  mode arbitration + e-stop
                                                    └──────┬───────┘
                                          teleop / object_align / waypoint
                                                           │  DriveCommand(left,right)
                                                           ▼
                                                    ┌──────────────┐
                                                    │  TankDrive   │  mixing + slew limit
                                                    └──────┬───────┘
                                                    ESCMotor ch0   ESCMotor ch1
                                                    (servo PWM)    (servo PWM)
```

The load-bearing idea is `ControlManager`. Teleop, object alignment, waypoint
navigation and the routine engine are all just *controllers*, and every one of
them emits the same `DriveCommand(left, right)`. The drive layer never learns
that a new mode exists, which is why a steered chassis reuses the autonomy
unchanged and why the state-machine editor can *delegate* a step to object align
instead of re-implementing it.

## The two halves of the base station

The Python program is the **bridge**: it owns the radio, the gamepad and the
tile cache, and speaks an internal `/ws` + `/tiles` API. The Deno app serves the
UI and reverse-proxies those two paths, so one build runs three ways — a native
desktop window, a LAN server for an iPad, and the Pi's Chromium kiosk.

That split is why the packages come in pairs, and why the base-station `.deb`
installs two systemd services rather than one.

> **Before you go further**
>
> Everything in this book works against the simulator. Read it with
> `./start-basestation.sh` running in a terminal and click along — nothing here
> needs a rover until [step 10](bringup/deploy.md).
