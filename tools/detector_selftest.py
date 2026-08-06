#!/usr/bin/env python3
"""Standalone detector self-test — verify vision in isolation, either backend.

Runs a one-shot DIAGNOSTIC of the camera + model by itself, independent of the
rest of the robot stack, and prints a PASS/FAIL summary. Run it first when
bringing up vision or chasing a "the model doesn't work on the rover" problem.

    python tools/detector_selftest.py                      # auto-detect backend
    python tools/detector_selftest.py --backend imx500     # AI Camera, on-sensor
    python tools/detector_selftest.py --model /var/lib/roversoftware/model.eim
    python tools/detector_selftest.py --label cone --frames 50
    python tools/detector_selftest.py --save /tmp/frame.jpg

Which backend it tests follows the same rule the rover uses
(`sensors/imx500.resolve_backend`), so this cannot drift from what actually
runs. Checks, in both cases:

  1. The runtime is installed (edge_impulse_linux / picamera2 + IMX500).
  2. The model file exists (and, for the .eim, is EXECUTABLE — it's a binary EI
     runs; the .rpk is data the sensor loads, so it needs no chmod).
  3. The model loads; prints labels and input size — warning loudly on Edge
     Impulse FOMO, which cannot report object size (so no approach/standoff).
  4. The camera opens; prints which backend won.
  5. N live frames: per-frame ms, achieved fps, and every detection as the
     controller would see it (label, conf, error_x, size).

--- This is also the standoff calibration procedure ---
`VisionConfig.standoff_size` is "how big the box looks when we're close enough",
which is not a number anyone can guess. Park the rover at the distance you want
it to stop, run this, and read off the printed `size`. That's your
RS_VISION_STANDOFF. No tape measure, no camera intrinsics. (Recalibrate it if
you switch backends — the two normalize `size` against different rectangles.)

--- And the METRIC RANGE calibration (separate, and optional) ---
Standoff above needs none of this — it stops on the raw `size` ratio. Metres are
for telemetry and for a standoff said in metres, and they do need a tape measure
once. The model is one constant (`distance * size = k`, see
robot/control/rangefinder.py), so the calibration is one measured PAIR and two
runs:

    # 1. CALIBRATE: bucket centred, tape-measured 3.00 m away
    python tools/detector_selftest.py --distance 3.00
    #    -> prints RS_VISION_RANGE_AT_M / RS_VISION_RANGE_SIZE to paste

    # 2. VERIFY at a distance you did NOT calibrate at (e.g. 1.5 m)
    python tools/detector_selftest.py --range-at-m 3.00 --range-size 0.121
    #    -> prints `range=` per frame; a few percent off is right

The object's real height is folded into `k` and never asked for, which is also
why the pair is per TARGET: a cone and a bucket at the same distance give
different boxes. A rover with an ultrasonic fitted learns the same constant on
its own, per label, and this is the fallback for labels it has not yet seen from
a measurable distance.

Both runs work off the MEDIAN box over the run, not one frame — the box jitters.
Same backend caveat as standoff: a pair measured on one backend is wrong on the
other (edge_impulse normalises `size` against a ~50 deg crop, imx500 against the
full ~66 deg frame — roughly 28% apart).

--- And it's how you verify framing and colour order ---
`--save` writes a frame. On Edge Impulse it's the model's own CROPPED input, and
two things need checking by eye: the colours must look right (if red and blue
are swapped, the frames are RGB where Edge Impulse expects BGR — the model will
still return confident garbage, so nothing else will tell you), and your target
must be inside the crop (EI discards ~25% of the width at 640x480). On the
IMX500 it's the full frame with the sensor's boxes drawn on it — there is no
crop, so what you're checking is that the boxes land on the objects.

Off-hardware (neither runtime installed) it prints a clear note and exits 0, so
it's safe to run on a dev laptop.
"""

import argparse
import os
import sys
import time

# Make the repo root (parent of tools/) importable, matching the other tools.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import RobotConfig  # noqa: E402
from robot.control.rangefinder import Rangefinder  # noqa: E402
from robot.sensors.camera import describe, draw_boxes, encode_jpeg, open_source  # noqa: E402
from robot.sensors.imx500 import (Decoder, resolve_backend, select_box,  # noqa: E402
                                  to_detection)

try:
    from edge_impulse_linux.image import ImageImpulseRunner
    _IMPORT_ERROR = None
except Exception as _e:
    # edge_impulse_linux itself is pure Python, but its image module imports cv2
    # (OpenCV) and numpy at load time. Keep the real error so a missing cv2 on
    # the Pi doesn't masquerade as "edge_impulse_linux not installed".
    ImageImpulseRunner = None
    _IMPORT_ERROR = _e

_FOMO_TYPE = "constrained_object_detection"


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


def _rangefinder(args) -> Rangefinder:
    """The same estimator the robot runs, standing on the pair passed in.

    Built here rather than reimplementing `k / size` so that what this tool
    prints and what the rover reports can never drift apart — including the
    refusal to answer at all when the pair is not set.
    """
    return Rangefinder(ref_distance_m=args.range_at_m, ref_size=args.range_size)


def _range_report(sizes, clipped, args, rf: Rangefinder) -> None:
    """Turn the frames just captured into a calibration pair, or check one.

    The box jitters several percent frame to frame, so this works off the MEDIAN
    of the run — never a single reading. Median, not mean: one frame where the
    model boxes half a bucket is an outlier, not evidence.
    """
    if clipped:
        print(f"\n  {clipped} frame(s) had the target CLIPPED at the frame edge "
              "and were excluded")
        print("  (a clipped box is short, so it reads as further away). Back off "
              "or re-aim if")
        print("  that is most of the run.")
    if not sizes:
        if clipped:
            print("  No usable frames — nothing to calibrate from.")
        return
    med = _median(sizes)
    print(f"\n  median size over {len(sizes)} frames: {med:.3f} "
          f"(min {min(sizes):.3f} / max {max(sizes):.3f})")
    if args.distance > 0.0:
        print(f"\n  CALIBRATION — target at a tape-measured {args.distance:.2f} m."
              " Paste into /etc/roversoftware/robot.env:")
        print(f"      RS_VISION_RANGE_AT_M={args.distance:g}")
        print(f"      RS_VISION_RANGE_SIZE={med:.3f}")
        print("  This pair is for THIS target and THIS backend — the object's")
        print("  height is folded into it. Then VERIFY: move to a distance you")
        print("  did NOT calibrate at, re-run with --range-at-m/--range-size (no")
        print("  --distance), and check the printed range. A few percent off is")
        print("  right; tens of percent means something upstream is wrong.")
        return
    est = rf.distance_m(med)
    if est is not None:
        print(f"  range at the median box: {est:.2f} m")
    else:
        print("  Pass --distance <metres> to calibrate metric range, or")
        print("  --range-at-m/--range-size to check an existing pair.")


def imx500_selftest(cfg, args) -> int:
    """The AI Camera path: same five checks, no CPU inference anywhere.

    Drives `sensors/imx500.Decoder` — the exact decode the rover runs — so a PASS
    here means the rover will see what this prints, coordinates included.
    """
    print("=== IMX500 (Raspberry Pi AI Camera) detector self-test ===\n")

    # 1. Runtime present?
    try:
        from picamera2 import Picamera2  # noqa: F401
        from picamera2.devices import IMX500  # noqa: F401
    except Exception as e:
        print(f"picamera2 with IMX500 support not importable ({e}) — nothing to test here.")
        print("This is expected on a dev laptop. On the Pi:")
        print("  sudo apt install python3-picamera2 imx500-all")
        return 0
    print("[1/5] PASS  picamera2 + IMX500 device support installed")

    # 2. Network file present? No chmod check: unlike the .eim this is data the
    #    sensor loads, not a binary the Pi executes.
    path = os.path.expanduser(cfg.vision.imx500_model)
    if not os.path.exists(path):
        print(f"[2/5] FAIL  no network at {path}")
        print("      Install the Sony model zoo:  sudo apt install imx500-all")
        print("      (they land in /usr/share/imx500-models/)")
        return 1
    print(f"[2/5] PASS  network {path} ({os.path.getsize(path) / 1e6:.1f} MB)")

    # 3. Is the camera actually attached? Checked before opening so the failure
    #    is one clear line instead of a libcamera stack trace.
    from robot.sensors.camera import imx500_present
    if not imx500_present():
        print("[3/5] FAIL  no IMX500 camera found on the CSI bus")
        print("      Check the ribbon (contacts toward the board), then:  rpicam-hello --list-cameras")
        return 1
    print("[3/5] PASS  AI Camera detected")

    # 4. Open it — this is where the network crosses to the sensor.
    print("[4/5] opening the camera and uploading the network to the sensor")
    print("      (cold start takes tens of seconds — this is normal)...")
    t0 = time.monotonic()
    source = open_source(cfg.camera, cfg.vision)
    if source is None or getattr(source, "imx500", None) is None:
        print(f"[4/5] FAIL  could not open the AI Camera with {os.path.basename(path)}")
        if source is not None:
            source.close()
        return 1
    decoder = Decoder(source, cfg.vision)
    print(f"[4/5] PASS  {describe(source)} in {time.monotonic() - t0:.1f}s")
    print(f"            {decoder.describe()}")
    task = getattr(decoder.intrinsics, "task", "?")
    if task != "object detection":
        print(f"            WARNING: task is {task!r}, not object detection — this "
              "network will not produce boxes.")
    if args.label and decoder.labels and args.label not in decoder.labels:
        print(f"            WARNING: --label {args.label!r} is not in this network's "
              f"labels — nothing will ever match. First few: {decoder.labels[:8]}")

    # 5. Live frames. No inference timing to report — the sensor did that before
    #    the frame arrived; what's measured here is the Pi-side decode.
    print(f"\n[5/5] reading {args.frames} frames...\n")
    times, seen, last, sizes, clipped = [], 0, None, [], 0
    rf = _rangefinder(args)
    try:
        for i in range(args.frames):
            frame, metadata = source.read_with_metadata()
            if frame is None:
                print(f"  frame {i:3d}  (no frame)")
                continue
            h, w = frame.shape[0], frame.shape[1]
            t1 = time.monotonic()
            boxes = decoder.parse(metadata, w, h)
            ms = (time.monotonic() - t1) * 1e3
            times.append(ms)
            last = (frame, boxes)
            if not boxes:
                print(f"  frame {i:3d}  {ms:6.2f} ms   (no target)")
                continue
            seen += 1
            best = select_box(boxes, cfg.vision.select, w)
            for b in boxes:
                d = to_detection(b, w, h, 0.0)
                mark = "*" if b is best else " "
                # Decoder.parse CLIPS boxes to the frame, so a target running off
                # the top or bottom reports a short height and therefore reads as
                # further away than it is. Poison for a calibration median — drop
                # those frames and say so rather than averaging the error in.
                cut = b[1] <= 0 or (b[1] + b[3]) >= h - 1
                # Only the box object_align would steer on feeds calibration — a
                # second bucket further away must not drag the median either.
                if b is best and d.size and not cut:
                    sizes.append(d.size)
                elif b is best and cut:
                    clipped += 1
                rng = rf.distance_m(d.size)
                print(f"  frame {i:3d}  {ms:6.2f} ms {mark} {d.label:<12} "
                      f"conf={d.confidence:.2f}  error_x={d.error_x:+.3f}  "
                      f"size={d.size:.3f}"
                      + (f"  range={rng:.2f} m" if rng is not None else "")
                      + ("  [CLIPPED at the frame edge]" if cut else ""))
    finally:
        source.close()

    print()
    if times:
        avg = sum(times) / len(times)
        print(f"  Pi-side decode: avg {avg:.2f} ms (inference itself ran on the sensor)")
    print(f"  frames with a target: {seen}/{len(times)}")
    print("  '*' marks the box object_align would steer on "
          f"(select={cfg.vision.select}).")
    _range_report(sizes, clipped, args, rf)

    if args.save and last is not None:
        frame, boxes = last
        jpeg = encode_jpeg(draw_boxes(frame, [(*b[:4], b[4], b[5], True) for b in boxes]), 85)
        if jpeg:
            with open(args.save, "wb") as f:
                f.write(jpeg)
            print(f"\n  wrote the last frame + boxes to {args.save}")
            print("  Check by eye: do the boxes land on the objects, and are the")
            print("  colours right (red/blue not swapped)?")
        else:
            print(f"\n  could not encode {args.save} (install python3-opencv or Pillow)")

    print()
    if seen:
        print("PASS — on-sensor detection is working. To calibrate standoff: park at")
        print("your stop distance and put the printed `size` in RS_VISION_STANDOFF.")
        print("Also set RS_VISION_HFOV to the camera's REAL horizontal FOV (~66 for")
        print("the stock lens) — there is no Edge Impulse crop on this path.")
        return 0
    print("No targets detected. If something should be in frame, check: the label")
    print("filter, --conf, and whether this network was trained on that object")
    print(f"(labels: {decoder.labels[:12]}).")
    return 1


def main() -> int:
    cfg = RobotConfig()
    p = argparse.ArgumentParser(description="Object detector self-test")
    p.add_argument("--backend", default=os.environ.get("RS_VISION_BACKEND", cfg.vision.backend),
                   choices=["auto", "edge_impulse", "imx500"],
                   help="which detection backend to test (default: the rover's own rule)")
    p.add_argument("--model", default=os.environ.get("RS_VISION_MODEL", cfg.vision.model_path))
    p.add_argument("--imx500-model", dest="imx500_model",
                   default=os.environ.get("RS_VISION_IMX500_MODEL", cfg.vision.imx500_model),
                   help="the .rpk network uploaded to the AI Camera's sensor")
    # A CUSTOM YOLO export (tools/imx500_export_yolo.py) carries no embedded
    # labels, so without this every box comes back "0"/"1"/"2" and --label never
    # matches. Zoo networks embed theirs and need nothing here.
    p.add_argument("--imx500-labels", dest="imx500_labels",
                   default=os.environ.get("RS_VISION_IMX500_LABELS", cfg.vision.imx500_labels),
                   help="labels.txt for a custom .rpk export (empty = embedded)")
    p.add_argument("--label", default=os.environ.get("RS_VISION_LABEL", cfg.vision.target_label),
                   help="only report this label ('' = all)")
    p.add_argument("--device", default=os.environ.get("RS_CAMERA_DEVICE", cfg.camera.device))
    p.add_argument("--conf", type=float, default=cfg.vision.min_confidence)
    p.add_argument("--frames", type=int, default=20, help="how many frames to classify")
    p.add_argument("--save", default=None, metavar="OUT.jpg",
                   help="write a frame here (check colour + framing)")
    # Metric range. Defaults come from the env so a robot.env that is already
    # calibrated makes this print real metres with no extra flags.
    p.add_argument("--distance", type=float, default=0.0, metavar="M",
                   help="CALIBRATE: tape-measured distance to the target right "
                        "now; prints the range pair to put in robot.env")
    p.add_argument("--range-at-m", dest="range_at_m", type=float, metavar="M",
                   default=float(os.environ.get("RS_VISION_RANGE_AT_M",
                                                cfg.vision.range_at_m)),
                   help="VERIFY: the distance half of an existing calibration "
                        "pair; prints range per frame so you can check it at a "
                        "distance you did not calibrate at")
    p.add_argument("--range-size", dest="range_size", type=float, metavar="S",
                   default=float(os.environ.get("RS_VISION_RANGE_SIZE",
                                                cfg.vision.range_size)),
                   help="VERIFY: the box-height half of that pair")
    args = p.parse_args()

    cfg.vision.backend = args.backend
    cfg.vision.model_path = args.model
    cfg.vision.imx500_model = args.imx500_model
    cfg.vision.imx500_labels = args.imx500_labels
    cfg.vision.target_label = args.label
    cfg.vision.min_confidence = args.conf
    cfg.camera.device = args.device
    # Same decision the rover makes — and it points cfg.camera.device at the
    # matching capture backend, so the test can't accidentally pass on one
    # camera while the rover opens another.
    backend = resolve_backend(cfg.vision, cfg.camera)
    if backend == "imx500":
        return imx500_selftest(cfg, args)

    print("=== Edge Impulse detector self-test ===\n")

    # 1. Library present?
    if ImageImpulseRunner is None:
        missing = getattr(_IMPORT_ERROR, "name", None)
        if isinstance(_IMPORT_ERROR, ModuleNotFoundError) and missing and missing != "edge_impulse_linux":
            # edge_impulse_linux is installed, but a dependency of it isn't —
            # usually OpenCV, which the Pi lacks by default. This is a real FAIL
            # on the Pi, not the "nothing to test on a laptop" case below.
            hint = ("sudo apt install python3-opencv" if missing == "cv2"
                    else f"pip install {missing}")
            print(f"[1/5] FAIL  edge_impulse_linux is installed, but its dependency "
                  f"'{missing}' is missing.")
            print(f"            Install it:  {hint}")
            return 1
        print(f"edge_impulse_linux not importable ({_IMPORT_ERROR}) — nothing to test here.")
        print("This is expected on a dev laptop (it's Linux-only and needs a")
        print("compiled .eim). On the Pi:  pip install edge_impulse_linux")
        return 0
    print("[1/5] PASS  edge_impulse_linux installed")

    # 2. Model file present and executable?
    path = os.path.expanduser(args.model)
    if not os.path.exists(path):
        print(f"[2/5] FAIL  no model at {path}")
        print("      Download one:  edge-impulse-linux-runner --download model.eim")
        return 1
    if not os.access(path, os.X_OK):
        print(f"[2/5] FAIL  {path} is not executable")
        print(f"      The .eim is a binary Edge Impulse runs.  Fix:  chmod +x {path}")
        return 1
    size_mb = os.path.getsize(path) / 1e6
    print(f"[2/5] PASS  model {path} ({size_mb:.1f} MB, executable)")

    # 3. Model init.
    runner = ImageImpulseRunner(path)
    try:
        info = runner.init()
    except Exception as e:
        print(f"[3/5] FAIL  could not init the model: {e}")
        return 1
    mp = info.get("model_parameters", {}) or {}
    labels = list(mp.get("labels", []) or [])
    mw = int(mp.get("image_input_width", 0) or 0)
    mh = int(mp.get("image_input_height", 0) or 0)
    mtype = mp.get("model_type", "?")
    fomo = mtype == _FOMO_TYPE
    proj = info.get("project", {})
    print(f"[3/5] PASS  '{proj.get('owner','?')}/{proj.get('name','?')}' "
          f"input={mw}x{mh} type={mtype}")
    print(f"            labels: {labels}")
    if args.label and args.label not in labels:
        print(f"            WARNING: --label {args.label!r} is not in this model's "
              f"labels — nothing will ever match.")
    if fomo:
        print()
        print("            *** FOMO MODEL — object size is NOT available. ***")
        print("            FOMO reports centroids with fixed cell-sized boxes, so")
        print("            object_align can turn to face the target but can NOT")
        print("            approach or stop at a standoff. Export a YOLO-style")
        print("            (object_detection) model if you need approach.")
        print()

    # 4. Camera.
    cfg.camera.device = args.device
    source = open_source(cfg.camera)
    if source is None:
        print("[4/5] FAIL  no camera available")
        print("      Install one:  sudo apt install python3-picamera2   (Pi Camera)")
        print("                    pip install opencv-python            (USB webcam)")
        runner.stop()
        return 1
    print(f"[4/5] PASS  camera open via {describe(source)} "
          f"({cfg.camera.width}x{cfg.camera.height})")

    # 5. Live inference.
    print(f"[5/5] classifying {args.frames} frames...\n")
    times, seen, last_cropped, sizes, clipped = [], 0, None, [], 0
    rf = _rangefinder(args)
    try:
        for i in range(args.frames):
            frame = source.read()
            if frame is None:
                print(f"  frame {i:3d}  (no frame)")
                continue
            t0 = time.monotonic()
            features, cropped = runner.get_features_from_image(frame)
            res = runner.classify(features)
            ms = (time.monotonic() - t0) * 1e3
            times.append(ms)
            last_cropped = cropped

            boxes = [b for b in (res.get("result", {}) or {}).get("bounding_boxes", []) or []
                     if b.get("value", 0.0) >= args.conf
                     and (not args.label or b.get("label") == args.label)]
            if not boxes:
                print(f"  frame {i:3d}  {ms:6.1f} ms   (no target)")
                continue
            seen += 1
            # Largest box only — matches the default select=largest that
            # object_align steers on, and keeps a background bucket out of it.
            big = None if fomo else max(boxes, key=lambda b: b["height"])
            # A target running off the top or bottom of EI's crop reports a short
            # height and so reads as further away. Never calibrate on one: drop
            # the frame and say how many went.
            cut = big is not None and (
                big["y"] <= 0 or (big["y"] + big["height"]) >= mh - 1)
            if cut:
                clipped += 1
            elif big is not None:
                sizes.append(big["height"] / mh)
            for b in boxes:
                cx = b["x"] + b["width"] / 2.0
                ex = _clamp(2.0 * cx / mw - 1.0, -1.0, 1.0)
                size = None if fomo else b["height"] / mh
                sz = "n/a (FOMO)" if fomo else f"{size:.3f}"
                rng = rf.distance_m(size)
                print(f"  frame {i:3d}  {ms:6.1f} ms   {b['label']:<12} "
                      f"conf={b['value']:.2f}  error_x={ex:+.3f}  size={sz}"
                      + (f"  range={rng:.2f} m" if rng is not None else "")
                      + ("  [CLIPPED at the crop edge]" if cut and b is big else ""))
    finally:
        source.close()
        runner.stop()

    print()
    if times:
        avg = sum(times) / len(times)
        print(f"  inference: avg {avg:.1f} ms  ->  {1000.0 / avg:.1f} fps "
              f"(min {min(times):.0f} / max {max(times):.0f})")
    print(f"  frames with a target: {seen}/{len(times)}")
    _range_report(sizes, clipped, args, rf)

    if args.save and last_cropped is not None:
        try:
            import cv2
            cv2.imwrite(args.save, last_cropped)
            print(f"\n  wrote the model's cropped input to {args.save}")
            print("  Check it by eye: are the colours right (red/blue not swapped),")
            print("  and is your target inside the crop?")
        except Exception as e:
            print(f"\n  could not write {args.save}: {e}")

    print()
    if seen:
        sz_hint = ("size is unavailable on FOMO — approach/standoff is off"
                   if fomo else
                   "park at your stop distance and put the printed size in RS_VISION_STANDOFF")
        print(f"PASS — detector is working. To calibrate standoff: {sz_hint}.")
        return 0
    print("No targets detected. If something should be in frame, check: the label")
    print("filter, --conf, the colour order (--save), and whether the target is")
    print("inside Edge Impulse's center-crop.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
