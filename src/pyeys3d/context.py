"""Device enumeration — maps the native USB scan onto cataloged models.

The native scan reports raw USB descriptors (product id, serial, port); the
model name comes from the Python catalog, so recognising a new camera needs
only its YAML, never a native rebuild.
"""
from __future__ import annotations

import time
from collections import namedtuple
from typing import List

from ._pyeys3d_native import Context as _NativeContext
from .modes import model_for_pid

# A connected camera as reported by enumeration. firmware_version can be
# empty when the camera is held by another process.
DeviceInfo = namedtuple(
    "DeviceInfo",
    ["model", "serial_number", "usb_port", "pid",
     "usb_port_type", "usb_speed", "firmware_version"],
    defaults=[0, "unknown", ""])

_USB_SPEED = {2: "USB2.0", 3: "USB3.0"}


def _usb_speed(port_type: int) -> str:
    return _USB_SPEED.get(port_type, "unknown")


class Context:
    """Enumerate connected eYs3D cameras. Lightweight — construct one
    whenever a fresh scan is needed."""

    def __init__(self) -> None:
        self._native = _NativeContext()

    def query_devices(self) -> List[DeviceInfo]:
        """A DeviceInfo per connected camera; model is taken from the
        catalog ('unknown' for an unrecognized product id).

        Concurrent enumeration (e.g. one process per camera) can occasionally
        hand back a descriptor string with a stray non-UTF-8 byte; re-reading
        returns clean data, so the scan is retried up to five times, after
        which the UnicodeDecodeError propagates.

        The list is point-in-time — call again after a hot-plug.
        """
        for attempt in range(5):
            try:
                return [
                    DeviceInfo(
                        model=model_for_pid(d.pid) or "unknown",
                        serial_number=d.serial_number,
                        usb_port=d.usb_port,
                        usb_port_type=d.usb_port_type,
                        usb_speed=_usb_speed(d.usb_port_type),
                        pid=d.pid,
                        firmware_version=d.firmware_version,
                    )
                    for d in self._native.query_devices()
                ]
            except UnicodeDecodeError:
                if attempt == 4:
                    raise
                time.sleep(0.05)          # let the concurrent scan clear
        raise AssertionError("unreachable: the loop returns or re-raises")


__all__ = ["Context", "DeviceInfo"]
