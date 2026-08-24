#!/usr/bin/env python3
"""01 for every connected camera at once — one process per camera.

A process each keeps the cameras independent: one wedging takes nothing
else down, and their opens run in parallel because the SDK's per-process
bookkeeping is not shared between them. The cost is on Windows, which
slows a process whose windows are not in front, so the cameras you are not
looking at run at a reduced rate. viewer.py takes the other side of that
trade — every camera in one process, so none of them is ever a background
workload, at the price of opening them one at a time.

With no arguments the parent enumerates the connected cameras and spawns
one child per device, each opening a color | depth window with the frame
rate in the title.

--model / --serial / --usb-port narrow which cameras to open; --mode,
--ir-value, --depth-range and --filters are forwarded to every spawned
camera. Each child is pinned by the serial the parent read, so a hot-plug
during the run cannot swap one child onto another camera; a unit that
reports no serial is pinned by model alone. Press q / ESC in a window to
close that camera; when every window is closed the parent exits.
Requires: pip install opencv-python.
"""
import argparse
import subprocess
import sys
import time

from example_helpers import add_device_args, print_device_info

import pyeys3d as ey


def stream_one(args) -> int:
    """Child: one camera, bound by serial, with the forwarded settings."""
    # Imported in the child only: the parent spawns processes and opens
    # no window.
    import cv2
    import numpy as np

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
        title = f"{tag} Color | Depth"
        last_drops = pipeline.frames_dropped

        t_tick = time.time()
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
            if frames is None:
                continue
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()

            # One window per camera. np.hstack needs a common height, so
            # the taller stream is scaled down to the shorter one.
            panels = []
            if color is not None:
                panels.append(color.get_data_bgr())
            if depth is not None:
                dmm = depth.get_data()       # (H, W) uint16, 1 unit = 1 mm
                panels.append(colorizer.colorize_bgr(dmm))
            if not panels:
                continue
            h = min(p.shape[0] for p in panels)
            panels = [p if p.shape[0] == h else
                      cv2.resize(p, (p.shape[1] * h // p.shape[0], h))
                      for p in panels]
            cv2.imshow(title, np.hstack(panels))   # setWindowTitle below
                                                   # needs the window

            now = time.time()
            if now - t_tick >= 1.0:
                t_tick = now
                fps = pipeline.fps
                cv2.setWindowTitle(
                    title,
                    f"{title}   |   color {fps.color:.1f} / "
                    f"depth {fps.depth:.1f} fps")
                # Wire drops point at USB bandwidth or scheduling.
                drops = pipeline.frames_dropped
                if drops != last_drops:
                    print(f"warning: {tag} wire drops "
                          f"color={drops.color} depth={drops.depth}")
                    last_drops = drops
    cv2.destroyAllWindows()
    return 0


def _child_argv(dev, args):
    """The command line for one child: the camera's exact identity plus the
    parent's forwarded Config settings (each re-parsed identically)."""
    argv = [sys.executable, __file__, "--child",
            "--model", dev.model, "--serial", dev.serial_number]
    # Serial and usb_port are AND-ed by Pipeline, so passing both pins the
    # exact unit. It is the serial that a module may not report, and two of
    # those on one model would leave every child equally ambiguous.
    if dev.usb_port:
        argv += ["--usb-port", dev.usb_port]
    if args.mode is not None:
        argv += ["--mode", str(args.mode)]
    if args.ir_value != -1:
        argv += ["--ir-value", str(args.ir_value)]
    if args.depth_range != [-1, -1]:
        argv += ["--depth-range", *(str(v) for v in args.depth_range)]
    if args.filters:
        argv += ["--filters"]
    return argv


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False)   # --child must not be reachable by abbreviation
    add_device_args(p)
    p.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()

    if args.child:
        return stream_one(args)

    devices = ey.Context().query_devices()
    # Only spawn for models this driver has a mode catalog for; other
    # eYs3D hardware on the bus is reported and left alone.
    supported = [d for d in devices if d.model in ey.supported_models()]
    for dev in devices:
        if dev.model not in ey.supported_models():
            print(f"skipping unsupported device: pid 0x{dev.pid:04x}  "
                  f"serial {dev.serial_number}", flush=True)
    # --model / --serial / --usb-port narrow which cameras to open.
    if args.model:
        supported = [d for d in supported if d.model == args.model]
    if args.serial:
        supported = [d for d in supported if args.serial in d.serial_number]
    if args.usb_port:
        supported = [d for d in supported if d.usb_port == args.usb_port]
    if not supported:
        print("no supported eYs3D camera connected", file=sys.stderr)
        return 1
    print(f"spawning one process per camera ({len(supported)} found):",
          flush=True)
    children = []
    try:
        for dev in supported:
            print(f"  {dev.model}  serial {dev.serial_number}  "
                  f"usb_port {dev.usb_port or '-'}", flush=True)
            children.append(subprocess.Popen(_child_argv(dev, args)))
        codes = [child.wait() for child in children]
    except KeyboardInterrupt:
        codes = []
    finally:
        # Every child holds a camera open, so none may outlive the parent —
        # a spawn that raises half way through has to take the ones already
        # running with it. terminate() only asks; wait() is what confirms,
        # and kill() is for a child wedged inside the SDK.
        for child in children:
            if child.poll() is None:
                child.terminate()
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    # A child that never opened its camera is the common failure (a mode the
    # link cannot carry), and it is invisible unless the parent reports it.
    return max(codes) if codes else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
