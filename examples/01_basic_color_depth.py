#!/usr/bin/env python3
"""Color + depth viewer — the base the other examples build on.

Color (Left), Color (Right — only in a mode that splits L|R) and Depth each
open in their own window; titles
show fps, hover shows RGB / distance. Depth is drawn false-color over the
configured range; the Colorizer line below takes mode="grayscale" for a gray
ramp instead. --filters enables the depth post-process chain (spatial /
temporal / hole-filling). Wire drops are reported
whenever the counters rise; --frame-meta additionally prints one frame's
number, hardware timestamp, host capture time and age once a second — the
fields to reach for when synchronizing against other sensors, and for
measuring latency. The full camera model (K / D / R / P) is printed at
startup.

Building on this base: 02 / 03 add a point cloud, 04 saves to disk, 05 tunes
the camera live, 06 runs several cameras at once, and viewer.py drives all
of it from a menu. Press q / ESC to quit.
Requires: pip install opencv-python.
"""
import argparse
import time

import cv2
import numpy as np
from example_helpers import Panels, add_device_args, print_device_info

import pyeys3d as ey


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_device_args(p)
    p.add_argument("--frame-meta", action="store_true", dest="frame_meta",
                   help="Once a second, print one frame's number and both its "
                        "timestamps, plus how old it is — the fields an "
                        "application aligns against other sensors.")
    args = p.parse_args()

    with ey.Pipeline() as pipeline:
        cfg = ey.Config().enable_device(args.model, mode_id=args.mode,
                                        serial_number=args.serial,
                                        usb_port=args.usb_port)
        cfg.set_ir_value(args.ir_value)
        cfg.set_depth_range(*args.depth_range)
        if args.filters:
            cfg.with_filters(ey.SpatialFilter(), ey.TemporalFilter(),
                             ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND))
        pipeline.start(cfg)
        tag = print_device_info(pipeline)
        # False-color depth over the configured range; pass
        # mode="grayscale" for a gray ramp instead.
        colorizer = ey.Colorizer(pipeline)
        panels = Panels()
        last_drops = pipeline.frames_dropped

        # The full stored model; see docs/api.md before using K or D.
        intr = pipeline.intrinsics
        if intr is not None:
            with np.printoptions(precision=4, suppress=True):
                print(f"  K  {intr.K.reshape(3, 3)}")
                print(f"  D  {intr.D}")
                print(f"  R  {intr.R.reshape(3, 3)}")
                print(f"  P  {intr.P.reshape(3, 4)}")

        t_tick = time.time()
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
            if frames is None:
                continue
            color = frames.get_color_frame()
            right = frames.get_right_color_frame()
            depth = frames.get_depth_frame()

            # Panels take the raw frame for hover, bgr to display.
            if color is not None:
                panels.show(f"{tag} Color (Left)", "color",
                            color.get_data(), color.get_data_bgr())
            if right is not None:
                panels.show(f"{tag} Color (Right)", "color",
                            right.get_data(), right.get_data_bgr())
            if depth is not None:
                dmm = depth.get_data()       # (H, W) uint16, 1 unit = 1 mm
                panels.show(f"{tag} Depth", "depth", dmm,
                            colorizer.colorize_bgr(dmm))

            now = time.time()
            if now - t_tick >= 1.0:
                t_tick = now
                panels.tick(pipeline.fps)
                # Wire drops point at USB bandwidth or scheduling.
                drops = pipeline.frames_dropped
                if drops != last_drops:
                    print(f"warning: {tag} wire drops "
                          f"color={drops.color} depth={drops.depth}")
                    last_drops = drops
                f = color if color is not None else depth
                if args.frame_meta and f is not None:
                    print(f"frame_number={f.frame_number}  "
                          f"hw_timestamp_us={f.hw_timestamp_us}  "
                          f"timestamp={f.timestamp:.6f}  "
                          f"age={now - f.timestamp:+.3f} s")
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
