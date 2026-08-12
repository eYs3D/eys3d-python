#!/usr/bin/env python3
"""The smallest complete pyeys3d program — the README quick start, runnable.

Auto-detects the connected camera, opens its signature mode (each model's
default, listed in the README), and shows color + depth while computing the
point cloud each frame. Expects a single connected camera — with several,
name one with Config().enable_device("G100P") below, or use the numbered
examples and their --model flag. Copy this file into your project as the
starting point; it has no dependency on the other examples.
Requires: pip install opencv-python.
"""
import shutil
import sys

import cv2

import pyeys3d as ey


def main() -> int:
    with ey.Pipeline() as pipeline:
        pipeline.start(ey.Config())        # auto-detect + signature mode

        dev = pipeline.device_info
        intr = pipeline.intrinsics          # None on an uncalibrated unit
        print(f"Opened {dev.model}[{dev.usb_port}]  serial {dev.serial_number}"
              f"  firmware {dev.firmware_version}", flush=True)
        if intr is not None:
            print(f"  fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} "
                  f"cy={intr.cy:.1f}  baseline {intr.baseline_mm:.2f} mm",
                  flush=True)
        else:
            print("  (uncalibrated unit: no intrinsics / point cloud)",
                  flush=True)

        colorizer = ey.Colorizer(pipeline)     # depth -> rgb8 colormap
        # Reprojection needs the stored calibration, so an uncalibrated unit
        # streams color and depth without the cloud.
        pc = ey.PointCloud(pipeline) if intr is not None else None
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
            if frames is None:
                continue
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if color is None or depth is None:
                continue
            dmm = depth.get_data()             # (H, W) uint16, 1 unit = 1 mm
            verts = pc.calculate(depth)[0] if pc else ()   # (N, 3) float32 m

            center_mm = int(dmm[dmm.shape[0] // 2, dmm.shape[1] // 2])
            center = f"{center_mm:5d} mm" if center_mm else " no data"
            nearest = f"{'no points':>23}"
            if len(verts):
                x, y, z = (int(v * 1000) for v in verts[verts[:, 2].argmin()])
                nearest = f"X{x:+5d} Y{y:+5d} Z{z:5d} mm"
            line = (f"center {center}   nearest {nearest}   "
                    f"depth #{depth.frame_number} t={depth.timestamp:.3f}")
            # Clip to the terminal width and pad, keeping the \r status one row.
            cols = shutil.get_terminal_size().columns - 1
            print(f"\r{line[:cols]:<{cols}}", end="", flush=True)

            cv2.imshow("color", color.get_data_bgr())
            cv2.imshow("depth", colorizer.colorize_bgr(dmm))
    cv2.destroyAllWindows()
    print()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
    except RuntimeError as e:
        # Most often "2 cameras match": this example takes no options, so
        # the message naming them is the whole answer, not a traceback.
        print(e, file=sys.stderr)
        raise SystemExit(1) from None
