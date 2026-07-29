<div class="hero">

# Describe the robot.<br>Draw what it does.<br><span class="tape">Then drive it.</span>

RoverSoftware is a modular Python stack for a ground robot and the touch base
station that commands a fleet of them. The build is declared from the dashboard
rather than compiled in, the behaviour is a state machine you draw, and all of
it runs against a simulator before you connect a single wire.

</div>

![The RoverSoftware base station: a satellite map filling the screen with three rovers drawn as heading arrows with position trails, a left rail showing throttle, heading and link age above a fleet list, a live camera feed with detection boxes, and a joystick in the bottom-right corner.](img/dashboard.webp)

*The driving view. **Terrain is the lit object** and every panel is pulled back
around it — the rover you are calling is filled teal, anything merely planned is
amber, and the stop button never moves. Every screenshot in this book is the
real app, running against `--sim`.*

<div class="quick-links">

[**Install it →**](install/apt.md) · one `apt install` on a rover \
[**Start with no hardware →**](bringup/simulator.md) · three simulated rovers in one command \
[**Add your motors →**](bringup/hardware.md) · the Hardware tab, field by field \
[**Program a routine →**](bringup/routines.md) · a state machine that runs on the robot

</div>

- **Runs without hardware** — three simulated rovers, `--sim`
- **Radio link** — XBee, newline-delimited JSON
- **Drive command** — one type, fleet-wide
- **Programming** — no Python required
- **Works offline** — tiles, speech and the language model all run locally

## Who this is for

If you are **driving** a rover at an event, read the bring-up steps in order and
stop after step 4. If you are **building** one, steps 5 through 7 are the whole
job. If you are **maintaining the code**, the reference section is where the
contracts live.

Nothing here needs a robot until step 9. Start the simulator and click along:

```bash
./start-basestation.sh
```
