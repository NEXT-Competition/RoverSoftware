# 7 · Tune it in the field

*Settings → Tuning. Over the radio, live, and saved on the robot.*

Everything the dashboard is allowed to change is on this tab: PID gains for
object alignment and waypoint heading hold, drive limits and per-motor ESC
calibration, loop and telemetry rates, vision thresholds, shooter geometry and
firing policy, GPS/IMU and FPV.

![The Tuning tab with a group expanded, showing numeric fields with units, ranges and one-line explanations.](../img/settings-tuning-open.webp)

Fetched when you open the tab rather than streamed: the full set is about
2.4 KB on a link shared with telemetry. On a robot running its own layout, the
per-motor groups are the ones **it** declared.

## Three rules worth knowing

**Clamped, not refused**
: Ask for a gain of 500 and you get the maximum, echoed back — so the field shows
what the robot is actually doing, not what you typed. A browser cannot reach an
arbitrary attribute: the tuning whitelist in `robot/tuning.py` decides which
paths exist and clamps every value.

**Badged "restart"**
: Serial ports, PWM channels and enable flags are stored immediately but only
take effect on the next start. The badge tells you which is which, so you are
never left wondering whether a change took.

**Saved on the robot**
: Robot settings persist on the robot, base-station settings on the base
station. Field tuning survives the next power cycle, and a rover carries its own
calibration to the next event.

## The base station's own settings

![The Base station settings tab with fields for the radio drive rate, dashboard refresh rate, video frame rate, basemap URL and trail length.](../img/settings-base.webp)

Radio airtime budget (`drive_hz`), dashboard refresh, video frame rate, basemap
URL and trail length. Point the basemap at a local tile server here for offline
field use.

## A tuning order that works

1. **Drive limits first.** Slew rate and max throttle, until hand-driving feels
   controllable. Everything downstream inherits this.
2. **Then heading hold.** Waypoint mode with two points and nothing else
   running; raise `kp` until it oscillates, then back off a third.
3. **Then alignment.** Object align against a stationary target. The standoff
   distance is geometry, not a gain — measure it.
4. **Firing policy last**, and only with the launcher pointed somewhere safe.

Changing gains while a routine is running is legal and sometimes the fastest way
to find a value. It is also how you discover that the rover was mid-approach.
