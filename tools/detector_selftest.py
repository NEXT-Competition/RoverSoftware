#!/usr/bin/env python3
"""Standalone Edge Impulse detector self-test — verify vision in isolation.

Runs a one-shot DIAGNOSTIC of the camera + model by itself, independent of the
rest of the robot stack, and prints a PASS/FAIL summary. Run it first when
bringing up vision or chasing a "the model doesn't work on the rover" problem.

    python tools/detector_selftest.py
    python tools/detector_selftest.py --model /var/lib/roversoftware/model.eim
    python tools/detector_selftest.py --label cone --frames 50
    python tools/detector_selftest.py --save /tmp/frame.jpg

Checks:
  1. edge_impulse_linux installed.
  2. Model file exists and is EXECUTABLE (the .eim is a binary EI runs).
  3. Model initializes; prints labels, input size, and model_type — warning
     loudly on FOMO, which cannot report object size (so no approach/standoff).
  4. Camera opens; prints which backend won.
  5. N frames of live inference: per-frame ms, achieved fps, and every detection
     as the controller would see it (label, conf, error_x, size).

--- This is also the standoff calibration procedure ---
`VisionConfig.standoff_size` is "how big the box looks when we're close enough",
which is not a number anyone can guess. Park the rover at the distance you want
it to stop, run this, and read off the printed `size`. That's your
RS_VISION_STANDOFF. No tape measure, no camera intrinsics.

--- And it's how you verify colour order ---
`--save` writes the model's own CROPPED input frame. Two things to check by eye:
the colours must look right (if red and blue are swapped, the frames are RGB
where Edge Impulse expects BGR — the model will still return confident garbage,
so nothing else will tell you), and your target must actually be inside the
frame (EI center-crops, discarding ~25% of the width at 640x480).

Off-hardware (no edge_impulse_linux) it prints a clear note and exits 0, so it's
safe to run on a dev laptop.
"""

import argparse
import os
import sys
import time

# Make the repo root (parent of tools/) importable, matching the other tools.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import RobotConfig  # noqa: E402
from robot.sensors.camera import describe, open_source  # noqa: E402

try:
    from edge_impulse_linux.image import ImageImpulseRunner
except Exception:
    ImageImpulseRunner = None

_FOMO_TYPE = "constrained_object_detection"


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def main() -> int:
    cfg = RobotConfig()
    p = argparse.ArgumentParser(description="Edge Impulse detector self-test")
    p.add_argument("--model", default=os.environ.get("RS_VISION_MODEL", cfg.vision.model_path))
    p.add_argument("--label", default=os.environ.get("RS_VISION_LABEL", cfg.vision.target_label),
                   help="only report this label ('' = all)")
    p.add_argument("--device", default=os.environ.get("RS_CAMERA_DEVICE", cfg.camera.device))
    p.add_argument("--conf", type=float, default=cfg.vision.min_confidence)
    p.add_argument("--frames", type=int, default=20, help="how many frames to classify")
    p.add_argument("--save", default=None, metavar="OUT.jpg",
                   help="write the model's cropped input frame here (check colour + framing)")
    args = p.parse_args()

    print("=== Edge Impulse detector self-test ===\n")

    # 1. Library present?
    if ImageImpulseRunner is None:
        print("edge_impulse_linux not installed — nothing to test here.")
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
    times, seen, last_cropped = [], 0, None
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
            for b in boxes:
                cx = b["x"] + b["width"] / 2.0
                ex = _clamp(2.0 * cx / mw - 1.0, -1.0, 1.0)
                sz = "n/a (FOMO)" if fomo else f"{b['height'] / mh:.3f}"
                print(f"  frame {i:3d}  {ms:6.1f} ms   {b['label']:<12} "
                      f"conf={b['value']:.2f}  error_x={ex:+.3f}  size={sz}")
    finally:
        source.close()
        runner.stop()

    print()
    if times:
        avg = sum(times) / len(times)
        print(f"  inference: avg {avg:.1f} ms  ->  {1000.0 / avg:.1f} fps "
              f"(min {min(times):.0f} / max {max(times):.0f})")
    print(f"  frames with a target: {seen}/{len(times)}")

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
