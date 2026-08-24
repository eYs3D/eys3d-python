"""Argument, reporting and display plumbing shared by the examples.

What belongs here: the scaffolding an example needs to be a program —
command-line options, console encoding, printing what was opened, putting
a frame in a window. What never belongs here: pyeys3d API usage. Each
example teaches one subject and has to show that subject in full, so the
calls a reader came to copy stay in the example even when several examples
make the same ones. Duplication between examples is the price of each one
being liftable on its own.

Where the same thing is done twice, it is written the same way twice —
same name, same body, same comments, character for character. A diff
between two examples then shows only what the later one adds, and a fix
lands on every copy. Where two examples genuinely need different
implementations (02's point-cloud window drives itself from the main loop,
viewer.py's runs on its own thread), they stay separate rather than being
merged into something that serves neither.

- add_device_args(parser): the common device options — --model / --mode /
  --serial / --usb-port / --ir-value / --depth-range / --filters.
- print_device_info(pipeline): print the opened device's model / serial /
  firmware / calibration; returns its model[usb_port] tag.
- Panels: OpenCV windows with an fps + hover readout in each title bar.
"""
from __future__ import annotations

import argparse
import sys


def _use_utf8_console() -> None:
    """Emit UTF-8 whatever the console code page is, so --help text and status
    lines (which use non-ASCII punctuation) never raise UnicodeEncodeError on a
    legacy Windows code page (cp1252 / cp950 / cp437). Runs on import, before
    any example parses args, so it covers the --help path too."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass


_use_utf8_console()


def add_device_args(parser: argparse.ArgumentParser
                    ) -> argparse.ArgumentParser:
    """Add the standard options the numbered examples 01-06 accept: which
    camera, which video mode, and the settings applied at start().

    How depth is colored and where point colors come from are not options:
    each example picks the obvious default and shows it in one line, which
    is the line to change.
    """
    parser.add_argument("--model", default=None,
                        help="Camera model string passed to Config.enable_device() "
                             "(default: auto-detect the connected camera).")
    parser.add_argument("--mode", type=int, default=None, metavar="MODE_ID",
                        help="mode_id passed to Config.enable_device() "
                             "(default: the model's signature mode; "
                             "use 00_enumerate.py to list available ids).")
    parser.add_argument("--serial", default="",
                        help="serial_number passed to Config.enable_device() "
                             "(substring match; default: any).")
    parser.add_argument("--usb-port", default="", dest="usb_port",
                        help="usb_port passed to Config.enable_device() "
                             "— bind by USB path, e.g. 2-1.3 (default: any).")
    parser.add_argument("--ir-value", type=int, default=-1, metavar="LEVEL",
                        dest="ir_value",
                        help="Value passed to Config.set_ir_value() "
                             "(default: -1 = per-model default; 0 = off).")
    parser.add_argument("--depth-range", type=int, nargs=2, default=[-1, -1],
                        metavar=("NEAR_MM", "FAR_MM"), dest="depth_range",
                        help="The pair passed to Config.set_depth_range() "
                             "(default: -1 -1 = per-model default, "
                             "e.g. G100+ 250 1900, R77 200 1500).")
    parser.add_argument("--filters", action="store_true",
                        help="Pass SpatialFilter + TemporalFilter + "
                             "HoleFillingFilter(FARTHEST_AROUND) to "
                             "Config.with_filters() (default: off).")
    return parser


def print_device_info(pipeline) -> str:
    """Print what was opened — model, serial, USB link, firmware and the
    stored intrinsics — and return its model[usb_port] window tag."""
    dev = pipeline.device_info
    tag = f"{dev.model}[{dev.usb_port}]"
    print(f"Opened {tag}", flush=True)
    print(f"  serial    : {dev.serial_number}", flush=True)
    print(f"  link      : {dev.usb_speed}", flush=True)
    print(f"  firmware  : {dev.firmware_version}", flush=True)
    intr = pipeline.intrinsics
    if intr is None:
        print("  intrinsics: none (device not calibrated)", flush=True)
    else:
        print(f"  intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
              f"cx={intr.cx:.1f} cy={intr.cy:.1f}  @ {intr.width}x{intr.height}"
              f"  baseline {intr.baseline_mm:.2f} mm", flush=True)
    return tag


class Panels:
    """OpenCV windows with an fps + hover readout in each title bar.

    Per frame, call show(name, kind, src, bgr) per window (kind is "color"
    or "depth"; src is the raw frame the hover reads, bgr is displayed).
    Once a second, call tick(pipeline.fps). resizable=True creates the
    windows as WINDOW_NORMAL, which a mode whose raster is larger than the
    screen needs; reset() destroys them so they come back at the next
    mode's native size. on_click fires when any of these windows is
    clicked, which is how a program showing several cameras at once can
    tell which one the user means.
    """

    def __init__(self, resizable: bool = False, on_click=None) -> None:
        import cv2
        self._cv2 = cv2
        self._resizable = resizable
        self._on_click = on_click
        self._fps = None     # pyeys3d StreamFps, or None before the first tick
        self._panels = {}    # name -> {"kind", "src", "hover"}

    def _set_title(self, name: str) -> None:
        st = self._panels[name]
        title = name
        if self._fps is not None:
            rate = (self._fps.depth if st["kind"] == "depth"
                    else self._fps.color)
            title += f"   |   {rate:.1f} fps"
        if st["hover"]:
            title += f"   |   {st['hover']}"
        self._cv2.setWindowTitle(name, title)

    def _callback(self, name: str):
        cv2 = self._cv2
        def on_mouse(event, x, y, flags, _):
            if event == cv2.EVENT_LBUTTONDOWN and self._on_click is not None:
                self._on_click()
            if event != cv2.EVENT_MOUSEMOVE:
                return
            st = self._panels.get(name)
            src = st["src"] if st else None
            # OpenCV reports image-pixel coordinates even on a
            # resized window, so the readout needs no scaling.
            if src is None or not (0 <= y < src.shape[0] and 0 <= x < src.shape[1]):
                return
            if st["kind"] == "depth":
                mm = int(src[y, x])
                st["hover"] = (f"({x},{y})  {mm} mm  ({mm / 1000:.3f} m)"
                               if mm > 0 else f"({x},{y})  no data")
            else:
                r, g, b = (int(v) for v in src[y, x])
                st["hover"] = f"({x},{y})  RGB=({r},{g},{b})"
            self._set_title(name)

        return on_mouse

    def show(self, name: str, kind: str, src, bgr) -> None:
        cv2 = self._cv2
        if name not in self._panels:
            # GUI_NORMAL drops the Qt toolbar / padding when OpenCV is built
            # with Qt; AUTOSIZE keeps the window at the image size, NORMAL
            # lets the user resize it.
            cv2.namedWindow(name, (cv2.WINDOW_NORMAL if self._resizable
                                   else cv2.WINDOW_AUTOSIZE)
                            | cv2.WINDOW_GUI_NORMAL)
            if self._resizable:
                cv2.resizeWindow(name, bgr.shape[1], bgr.shape[0])
            cv2.setMouseCallback(name, self._callback(name))
            self._panels[name] = {"kind": kind, "src": None, "hover": ""}
        self._panels[name]["src"] = src
        cv2.imshow(name, bgr)

    def tick(self, fps) -> None:
        """Refresh every title's rate; takes Pipeline.fps."""
        self._fps = fps
        for name in self._panels:
            self._set_title(name)

    def reset(self) -> None:
        """Destroy the windows so they recreate at the next mode's native
        resolution, and drop the ones a new mode no longer produces."""
        for name in self._panels:
            self._cv2.destroyWindow(name)
        self._panels.clear()
        self._fps = None
