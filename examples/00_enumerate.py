#!/usr/bin/env python3
"""List connected eYs3D cameras and their video-mode catalogs.

The full mode table of each connected model prints first, with the
signature mode of each USB link marked (that is the one start() opens
when no mode_id is given); the device summary (model, serial number,
firmware, USB port) prints last so it stays visible next to the prompt.

Usage:
    Linux:    python3 examples/00_enumerate.py
    Windows:  python examples\\00_enumerate.py
"""
import pyeys3d as ey


def main() -> int:
    print(f"pyeys3d version: {ey.__version__}")
    print()

    ctx = ey.Context()
    devs = ctx.query_devices()
    if not devs:
        print("No eYs3D devices detected.")
        print()
        print("If a camera is connected, check:")
        print("  - the device is visible to the OS (Linux: lsusb shows")
        print("    '3438:01xx'; Windows: it appears in Device Manager)")
        print("  - on Linux, the user has access to /dev/video*")
        print("  - the cable is a data cable, not charge-only, and the port")
        print("    supplies bus power")
        return 1

    for model in sorted({d.model for d in devs if d.model != "unknown"}):
        info = ey.load_model(model)
        catalog = info.modes
        # The mode start() picks when none is named, per USB link. Marked
        # because it is the one a caller gets without asking, and it differs
        # between a USB 2 and a USB 3 link on the same camera.
        signature = set(info.signature_mode.values())
        print(f"{model} modes ({len(catalog)}):")
        for mid, mode in sorted(catalog.items()):
            fmt = "YUYV" if mode.color.fmt == 0 else "MJPEG"
            mark = "*" if mid in signature else " "
            print(f" {mark}{mid:3d}: USB{mode.usb}  {mode.name}  color={fmt}")
        sig = ", ".join(f"USB{u} -> {m}"
                        for u, m in sorted(info.signature_mode.items()))
        print(f"  (* signature mode: {sig})")
        print()

    # The identifiers below pin a camera:
    # Config.enable_device(serial_number=... / usb_port=...).
    print(f"Found {len(devs)} device(s):")
    for d in devs:
        print(f"  model={d.model!s:8s} "
              f"pid=0x{d.pid:04x} "
              f"serial='{d.serial_number}' "
              f"usb_port='{d.usb_port}' "
              f"link={d.usb_speed} "
              f"firmware='{d.firmware_version}'")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
