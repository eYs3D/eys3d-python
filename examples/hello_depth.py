#!/usr/bin/env python3
"""The first program to run: prints the distance at the image center.

No window and no packages beyond pyeys3d itself — this proves the camera,
the driver, and the depth path work before anything else is installed.
Auto-detects the connected camera and opens its signature mode, then
prints the center-pixel distance as it updates. Ctrl-C to stop.

Expects a single connected camera, and takes no options: with several,
name one with Config().enable_device("G100P") below, or use the numbered
examples and their --model / --serial flags.

The first frame lands a few seconds after start(): that delay is a
firmware property and differs per module and per video mode.
"""
import sys

import pyeys3d as ey


def main() -> int:
    with ey.Pipeline() as pipeline:
        pipeline.start(ey.Config())        # auto-detect + signature mode
        dev = pipeline.device_info
        print(f"Opened {dev.model}[{dev.usb_port}]  serial {dev.serial_number}"
              f"  firmware {dev.firmware_version}", flush=True)
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if frames is None:
                continue
            depth = frames.get_depth_frame()
            if depth is None:                  # a color-only video mode
                continue
            dmm = depth.get_data()
            center_mm = int(dmm[dmm.shape[0] // 2, dmm.shape[1] // 2])
            center = f"{center_mm:5d} mm" if center_mm else " no data"
            print(f"\rcenter distance: {center}", end="", flush=True)
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
