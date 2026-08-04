#!/usr/bin/env python3
"""Generate this fleet's layout documents.

A layout says what a build HAS (see robot/layout.py). It normally lives only on
the robot, at /var/lib/roversoftware/layout.json, and is authored in the base
station's Hardware tab. These are checked in as well because a fleet of more
than one rover needs the documents to be diffable, reviewable and re-deployable
without a rover in front of you — and because the angles below are the record
of how each machine is actually wired.

Built by SCRIPT rather than hand-written JSON for the same reason
layout.default_doc() is: the document is generated from the same dataclasses the
robot validates against, so it cannot drift into a shape the robot would refuse.
Every document produced here is run through the real validator before it is
written.

    python3 packaging/layouts/build_layouts.py      # regenerate, then commit
    just push-layout packaging/layouts/east.json    # deploy one to a rover
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from robot import layout                                       # noqa: E402
from robot.config import (                                     # noqa: E402
    MechanismConfig,
    MotorConfig,
    RobotConfig,
)

HERE = os.path.dirname(os.path.abspath(__file__))

# --- the shared electrical facts --------------------------------------------
# Fusion HAT literal angles. 5 is neutral (a full stop) for EVERY motor on both
# rovers, and it is also what keeps an ESC armed. The usable throw is symmetric
# about neutral, so whichever endpoint sits closer to 5 sets it: with -30 and
# +40 the throw is 35 either way, and +-1.0 lands exactly on the two endpoints.
NEUTRAL = 5.0


def motor(channel, name, label, lo=-30.0, hi=40.0, scale=1.0):
    return MotorConfig(channel=channel, name=name, label=label, kind="esc",
                       neutral_angle=NEUTRAL, min_angle=lo, max_angle=hi,
                       speed_scale=scale)


def power(name, label, actuators, presets, auto_stop=0.0, slew=0.0):
    return MechanismConfig(name=name, label=label, kind="power",
                           actuators=actuators, presets=presets,
                           auto_stop_seconds=auto_stop, slew_rate=slew)


# --- east: intake + dumper ---------------------------------------------------

def east() -> dict:
    """Rover 1 (east.local). No launcher; channel 2 is a dumper."""
    cfg = RobotConfig()
    cfg.mechanisms = {
        "intake": power(
            "intake", "Intake",
            {"roller": motor(3, "roller", "Roller")},
            # +1.0 -> +40 takes in, -1.0 -> -30 spits.
            {"in": {"roller": 1.0}, "out": {"roller": -1.0}},
            # Dead-man for the HELD spit only; the toggled "in" is exempt.
            auto_stop=0.5,
        ),
        "dumper": power(
            "dumper", "Dumper",
            {"motor": motor(2, "motor", "Dumper")},
            # One preset, because the control is a bool: pressed = run at -30,
            # pressed again = stop. A toggle is refreshed by nothing, so it
            # carries no dead-man.
            {"run": {"motor": -1.0}},
        ),
    }
    return layout.to_doc(cfg)


# --- shooter: intake + flywheel + feeder + agitator ---------------------------

def shooter() -> dict:
    """Rover 2. Same wiring as east plus a feeder and an agitator.

    Channel 2 carries a flywheel here rather than a dumper, but it keeps the
    mechanism NAME `dumper`: the gamepad binding addresses a mechanism by name,
    the mapping is one per base station rather than one per rover, and the point
    of the shared name is that the same button works the channel-2 motor on
    whichever rover is selected. `label` is what the dashboard shows, and that
    is where the two builds differ.
    """
    cfg = RobotConfig()
    # This rover's right track runs faster than its left, so trim the FASTER
    # side down until it tracks straight. There is no way to speed the slower
    # one up — it is already being asked for everything it has.
    cfg.drive.actuators["right"].speed_scale = 0.9
    cfg.mechanisms = {
        "intake": power(
            "intake", "Intake",
            {"roller": motor(3, "roller", "Roller")},
            # Mirror of east: this one takes in at -30 and spits at +40.
            {"in": {"roller": -1.0}, "out": {"roller": 1.0}},
            auto_stop=0.5,
        ),
        "dumper": power(
            "dumper", "Flywheel",
            # Runs at -50: reaching it needs a throw of 55, so the endpoints are
            # -50/+60 — pushing the preset past -1.0 would only clamp.
            {"motor": motor(2, "motor", "Flywheel", lo=-50.0, hi=60.0)},
            {"run": {"motor": -1.0}},
            # Wind up over ~1 s instead of in one step. A flywheel is the one
            # load here with real inertia, and a step from neutral to full asks
            # its ESC for a current it cannot deliver — the wheel spins up and
            # the ESC's own protection cuts it dead. Stopping is not ramped.
            slew=1.0,
        ),
        "feeder": power(
            "feeder", "Feeder",
            # Runs at +50: throw of 45, so endpoints -40/+50.
            # The actuator names here and below match what this rover already
            # had, because an actuator name IS its tuning path
            # (mech.feeder.feeder.*) — renaming one orphans anything tuned
            # against it for no gain.
            {"feeder": motor(4, "feeder", "Feeder", lo=-40.0, hi=50.0)},
            {"run": {"feeder": 1.0}},
            # RUN WHILE HELD, so this one wants the dead-man: the pad
            # re-announces a held control every 0.25 s, and if the robot stops
            # hearing it the feeder stops instead of running on alone.
            auto_stop=0.5,
        ),
        "agitator": power(
            "agitator", "Agitator",
            # Neutral is 5, so +-90 is not reachable both ways: the throw is the
            # narrower side. -80/+90 gives 85 each way, which is full speed in
            # both directions and all this needs — it only has to stir.
            {"agitator": motor(5, "agitator", "Agitator", lo=-80.0, hi=90.0)},
            {"run": {"agitator": 1.0}, "reverse": {"agitator": -1.0}},
            auto_stop=0.5,  # both directions are held (D-pad up / down)
        ),
    }
    return layout.to_doc(cfg)


def main() -> int:
    failed = False
    for name, build in (("east", east), ("shooter", shooter)):
        doc = build()
        # The real validator, with no reserved channels: neither rover enables
        # the built-in launcher, which is what frees channel 2 for a mechanism.
        result = layout.validate(doc, {})
        blob = json.dumps(doc, indent=2, sort_keys=True) + "\n"
        status = "ok" if result.ok else "REFUSED"
        print(f"{name}.json: {status}  {len(blob)}/{layout.MAX_DOC_BYTES} bytes")
        for e in result.errors:
            print(f"  error: {e}")
        for w in result.warnings:
            print(f"  warning: {w}")
        if not result.ok or len(blob) > layout.MAX_DOC_BYTES:
            failed = True
            continue
        for m in doc["mechanisms"]:
            chans = ", ".join(f"ch{a['channel']}" for a in m["actuators"])
            print(f"  {m['name']:<9} {chans:<6} presets={sorted(m['presets'])}")
        with open(os.path.join(HERE, f"{name}.json"), "w") as fh:
            fh.write(blob)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
