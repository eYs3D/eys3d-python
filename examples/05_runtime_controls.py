#!/usr/bin/env python3
"""01 + live camera-control tuning in a separate control window.

Every control set through Config applies once at start(); this example shows
the runtime counterparts on Pipeline, adjusted while streaming. The color and
depth windows stay clean (as in 01); the controls live in their own window so
they never cover the image. Keys (focus the Controls window):

    Up / Down      select a control
    - / +          adjust the selected control (also Left / Right); AE / AWB
                   flip on/off, Power Line cycles through its modes
    d              restore defaults (IR to the model default, AE / AWB to auto)
    x              hardware-reset the camera (drops the link; it reconnects)
    q / ESC        quit

White Balance stages its value and commits 0.3 s after the last keypress,
so holding the key sweeps the range on one device write instead of one per
repeat; the menu shows it as pending until then. Auto Exposure / Auto White
Balance grey out their manual value while on.
Values are read back from the device after every change and survive a USB
drop: the hot-plug watchdog re-applies the latest state on reconnect. Press
x to see it — hardware_reset() re-enumerates the camera like a replug, and
the Link line tracks it going down and back up.
Requires: pip install opencv-python.
"""
import argparse
import time

import cv2
import numpy as np
from example_helpers import Panels, add_device_args, print_device_info

import pyeys3d as ey

_PLF_NAMES = {1: "50Hz", 2: "60Hz"}

# cv2.waitKeyEx arrow-key codes (X11 / Windows) and the adjust keys.
_UP = {65362, 2490368}
_DOWN = {65364, 2621440}
_DEC = {65361, 2424832, 45, 95}      # Left / '-' / '_'
_INC = {65363, 2555904, 43, 61}      # Right / '+' / '='

# Control indices within make_control_rows()'s list.
_IR, _AE, _EXP, _AWB, _WB, _PLF = range(6)


def norm_key(k):
    """Drop the modifier-state bits OpenCV's GTK backend packs into the
    high half, so NumLock or CapsLock does not stop a key matching.

    Windows extended keys (the arrows) carry their code in the high half
    and leave the low half zero, so a zero low half means the whole value
    is the key. Testing the low half for the keysym range instead would
    cover the arrows and miss every ASCII key."""
    low = k & 0xFFFF
    return low if k > 0 and low else k


def make_control_rows(pipeline, e_step):
    """(label, read value, adjust by direction) per live camera control.
    Each read/write is a device control transfer, so they run on demand,
    not per frame; White Balance has no adjust fn — it commits once the
    key stops moving."""
    def rng(r):
        return f" ({r.min}~{r.max})" if r else ""

    def ir(d):
        pipeline.set_ir_value(max(0, (pipeline.get_ir_value() or 0) + d))

    def exposure(d):
        pipeline.set_exposure((pipeline.get_exposure() or 0) + d * e_step)

    def power_line(d):
        # Only 50 Hz (1) and 60 Hz (2) are supported, not UVC's off / auto,
        # so +/- toggles between them.
        pipeline.set_power_line_frequency(
            1 if pipeline.get_power_line_frequency() == 2 else 2)

    return [
        (f"IR Intensity{rng(pipeline.get_ir_range())}",
         pipeline.get_ir_value, ir),
        ("Auto Exposure", pipeline.get_auto_exposure,
         lambda d: pipeline.set_auto_exposure(not pipeline.get_auto_exposure())),
        (f"Exposure{rng(pipeline.get_exposure_range())}",
         pipeline.get_exposure, exposure),
        ("Auto White Balance", pipeline.get_auto_white_balance,
         lambda d: pipeline.set_auto_white_balance(
             not pipeline.get_auto_white_balance())),
        (f"White Balance{rng(pipeline.get_white_balance_range())}",
         pipeline.get_white_balance, None),
        ("Power Line",
         lambda: _PLF_NAMES.get(pipeline.get_power_line_frequency(),
                                pipeline.get_power_line_frequency()),
         power_line),
    ]


def enabled(n, vals):
    """Exposure is auto while AE is on; White Balance is auto while AWB is.
    A control the model does not have reads back None (white balance on the
    monochrome G62 / R77), which disables it too."""
    if vals[n] == "None":
        return False
    if n == _EXP:
        return vals[_AE] != "True"
    if n == _WB:
        return vals[_AWB] != "True"
    return True


def render_controls(cv2, win, rows, vals, sel, pipeline):
    """Draw the control menu in its own window; the selected row is
    highlighted and a disabled (auto) control is dimmed."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
    line_h, pad = 28, 12
    lines = []
    for n, (label, _, _) in enumerate(rows):
        val = vals[n] if enabled(n, vals) else "auto"
        lines.append((f"{label}: {val}", n == sel, enabled(n, vals)))
    lines.append((f"Link: {'up' if pipeline.is_connected else 'down'}"
                  f"  (reconnects {pipeline.reconnect_count})", False, True))
    lines.append(("Up/Dn select   -/+ adjust   [d] defaults   [x] reset   "
                  "[q] quit", False, True))

    def tw(s):
        return cv2.getTextSize(s, font, scale, thick)[0][0]

    width = max(tw(t) for t, _, _ in lines) + 2 * pad
    img = np.full((pad * 2 + line_h * len(lines), width, 3), 32, np.uint8)
    for n, (text, selected, on) in enumerate(lines):
        top = pad + line_h * n
        if selected:
            cv2.rectangle(img, (4, top + 2), (width - 4, top + line_h - 2),
                          (70, 55, 30), -1)
            fg = (90, 220, 255)
        else:
            fg = (232, 232, 232) if on else (110, 110, 110)
        cv2.putText(img, text, (pad, top + line_h - 8), font, scale, fg, thick,
                    cv2.LINE_AA)
    cv2.imshow(win, img)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_device_args(p)
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
        controls_win = f"{tag} Controls"
        cv2.namedWindow(controls_win,
                        cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)

        er = pipeline.get_exposure_range()
        wr = pipeline.get_white_balance_range()
        e_step = er.step if er and er.step > 0 else 1
        wb_step = wr.step if wr and wr.step > 0 else 100

        rows = make_control_rows(pipeline, e_step)
        wb_pending = None                  # target while the key repeats
        wb_last_key = 0.0
        vals = [str(read()) for _, read, _ in rows]
        sel = 0
        seen_reconnects = pipeline.reconnect_count
        t_tick = time.time()

        def move(step):
            nonlocal sel
            for _ in range(len(rows)):     # skip disabled (auto) controls
                sel = (sel + step) % len(rows)
                if enabled(sel, vals):
                    return

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            k = norm_key(cv2.waitKeyEx(1))
            if k in (ord('q'), 27):
                break

            # A reconnect re-applies the kept values; re-read what the camera
            # actually holds. WB commits once the key repeats stop — one write
            # for the whole sweep instead of one per repeat.
            if pipeline.reconnect_count != seen_reconnects:
                seen_reconnects = pipeline.reconnect_count
                vals = [str(read()) for _, read, _ in rows]
            if wb_pending is not None and time.monotonic() - wb_last_key > 0.3:
                try:
                    pipeline.set_white_balance(wb_pending)
                except (ValueError, RuntimeError) as e:
                    print(e)
                wb_pending = None
                vals = [str(read()) for _, read, _ in rows]

            render_controls(cv2, controls_win, rows, vals, sel, pipeline)
            if frames is not None:
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

            if k == -1:
                continue
            adjusted = False
            try:
                if k in _UP:
                    move(-1)
                elif k in _DOWN:
                    move(+1)
                elif k in _DEC or k in _INC:
                    d = -1 if k in _DEC else +1
                    if sel == _WB:
                        base = (wb_pending if wb_pending is not None
                                else (pipeline.get_white_balance() or 0))
                        wb_pending = max(0, base + d * wb_step)
                        wb_last_key = time.monotonic()
                        vals[sel] = f"{wb_pending} (pending)"
                    else:
                        rows[sel][2](d)
                        adjusted = True
                elif k == ord('d'):
                    # eSPDI has no reset call, so the defaults are written
                    # directly; a control the model lacks (e.g. no white
                    # balance on a mono unit) is skipped, not fatal.
                    wb_pending = None
                    for restore in (lambda: pipeline.set_ir_value(-1),
                                    lambda: pipeline.set_auto_exposure(True),
                                    lambda: pipeline.set_auto_white_balance(
                                        True)):
                        try:
                            restore()
                        except (ValueError, RuntimeError):
                            pass
                    adjusted = True
                elif k == ord('x'):
                    # On reconnect the loop above re-reads the kept values.
                    print(f"{tag}: hardware reset - the camera drops off "
                          f"the bus and reconnects")
                    pipeline.hardware_reset()
            except (ValueError, RuntimeError) as e:
                print(e)                   # out-of-range or device-rejected
                adjusted = True
            if adjusted:
                vals = [str(read()) for _, read, _ in rows]
                if not enabled(sel, vals):    # AE/AWB just greyed this row
                    move(+1)
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
