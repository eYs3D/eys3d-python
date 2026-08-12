#!/usr/bin/env python3
"""01 + saving to disk: single snapshots or a recorded clip.

Everything lands under one output directory (default ./capture):

    capture/snapshots/<timestamp>_color.png          press s to save
    capture/snapshots/<timestamp>_depth.png          raw 16-bit, 1 mm/unit
    capture/snapshots/<timestamp>_depth_preview.png  the viewable rendering
    capture/snapshots/<timestamp>_cloud.ply          MeshLab / Open3D
    capture/clips/<timestamp>/                       one recorded clip
        color/000000.jpg  depth/000000.png           paired frame sets
        metadata.jsonl                               index + calibration

Snapshot (default): a live viewer opens; press s to save a set, q /
ESC quits. --snapshot saves one set and exits with no window and no
keypress, after giving auto-exposure time to settle — the frame it
writes is the first properly exposed one, not the first to arrive.

Clip: --record 10 records a ten-second clip, then exits. The first
metadata.jsonl line carries the device identity, depth range, and
color intrinsics (fx fy cx cy, baseline); each following line indexes
one frame set with frame numbers, hardware timestamps, host-clock
capture times and file names. --play <clip dir> replays it at the
recorded pace. The format is deliberately plain — every file opens
with standard tools, and swapping in your own storage means editing
save_frame_set() only.

Depth PNGs are uint16 with one millimeter per unit — most image tools
display them dark; the *_depth_preview.png alongside a snapshot is the
viewable rendering. Load the values back with cv2.imread(path,
cv2.IMREAD_UNCHANGED).
Requires: pip install opencv-python.
"""
import argparse
import json
import os
import sys
import time

import cv2
from example_helpers import Panels, add_device_args, print_device_info

import pyeys3d as ey


def save_ply(path, verts, colors=None) -> None:
    """Write an ASCII PLY (xyz in meters, optional uint8 rgb)."""
    with open(path, "w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(verts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            f.write("property uchar red\nproperty uchar green\n"
                    "property uchar blue\n")
        f.write("end_header\n")
        if colors is not None:
            for (x, y, z), (r, g, b) in zip(verts, colors):
                f.write(f"{x:.5f} {y:.5f} {z:.5f} {r} {g} {b}\n")
        else:
            for x, y, z in verts:
                f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def file_stamp() -> str:
    """Timestamp shared by snapshot files and clip directories; the
    millisecond suffix keeps rapid saves from colliding."""
    return (time.strftime("%Y%m%d-%H%M%S")
            + f"-{int(time.time() * 1000) % 1000:03d}")


def imwrite(path, img) -> bool:
    """cv2.imwrite reports failure by returning False, never by raising: a
    full disk, a codec that rejects the extension, or — on Windows — any
    path that is not representable in the system code page. Reported here
    so a caller never names a file that was not written."""
    ok = cv2.imwrite(path, img)
    if not ok:
        print(f"could not write {path}", file=sys.stderr)
    return ok


def save_snapshot(out_dir, color, depth, verts, colors,
                  depth_rgb=None) -> list:
    """Save one snapshot set; returns the list of files written."""
    out_dir = os.path.join(out_dir, "snapshots")
    os.makedirs(out_dir, exist_ok=True)
    stamp = file_stamp()
    written = []
    if color is not None:
        path = os.path.join(out_dir, f"{stamp}_color.png")
        if imwrite(path, color.get_data_bgr()):
            written.append(path)
    if depth is not None:
        path = os.path.join(out_dir, f"{stamp}_depth.png")
        if imwrite(path, depth.get_data()):            # uint16, 1 unit = 1 mm
            written.append(path)
    if depth_rgb is not None:
        path = os.path.join(out_dir, f"{stamp}_depth_preview.png")
        if imwrite(path, depth_rgb[:, :, ::-1].copy()):
            written.append(path)
    if verts is not None and len(verts):
        path = os.path.join(out_dir, f"{stamp}_cloud.ply")
        save_ply(path, verts, colors)
        written.append(path)
    return written


def save_frame_set(clip_dir, index, color, depth) -> dict:
    """Write one clip frame set; returns its metadata.jsonl entry. A frame
    that could not be written is left out of the entry rather than indexed,
    so playback never opens a file that is not there."""
    entry = {"index": index}
    if color is not None:
        name = f"color/{index:06d}.jpg"
        if imwrite(os.path.join(clip_dir, name), color.get_data_bgr()):
            entry.update(color_file=name,
                         color_frame_number=color.frame_number,
                         color_hw_us=color.hw_timestamp_us,
                         color_time=color.timestamp)
    if depth is not None:
        name = f"depth/{index:06d}.png"                # uint16, 1 mm per unit
        if imwrite(os.path.join(clip_dir, name), depth.get_data()):
            entry.update(depth_file=name,
                         depth_frame_number=depth.frame_number,
                         depth_hw_us=depth.hw_timestamp_us,
                         depth_time=depth.timestamp)
    return entry


def record(args) -> int:
    # One clip = one timestamped directory, named like the snapshots,
    # with the color / depth pairs in sibling subdirectories.
    clip_dir = os.path.join(args.out, "clips", file_stamp())
    os.makedirs(os.path.join(clip_dir, "color"), exist_ok=True)
    os.makedirs(os.path.join(clip_dir, "depth"), exist_ok=True)
    with ey.Pipeline() as pipeline, \
         open(os.path.join(clip_dir, "metadata.jsonl"), "w",
              encoding="ascii") as meta:
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
        # First line: what produced the clip — identity, depth range and
        # intrinsics — so playback (or any later consumer) needs nothing
        # but the directory.
        header = {
            "model": pipeline.device_info.model,
            "serial_number": pipeline.device_info.serial_number,
            "depth_min_mm": pipeline.depth_near_mm,
            "depth_max_mm": pipeline.depth_far_mm,
        }
        intr = pipeline.intrinsics
        if intr is not None:
            header["intrinsics"] = {
                "width": intr.width, "height": intr.height,
                "fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy,
                "baseline_mm": intr.baseline_mm,
            }
        meta.write(json.dumps(header) + "\n")
        index = 0
        # The clock starts on the first frame, not on start(): the first one
        # lands seconds later (a firmware property, and longer on a cold
        # start than the whole clip on a short --record), which would
        # otherwise come straight out of the recorded seconds.
        t_end = None
        while t_end is None or time.time() < t_end:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if frames is None:
                continue
            if t_end is None:
                t_end = time.time() + args.record
            entry = save_frame_set(clip_dir, index,
                                   frames.get_color_frame(),
                                   frames.get_depth_frame())
            meta.write(json.dumps(entry) + "\n")
            index += 1
        drops = pipeline.frames_dropped
    print(f"recorded {index} frame sets to {clip_dir}/")
    if drops.color or drops.depth:
        print(f"warning: {tag} wire drops "
              f"color={drops.color} depth={drops.depth}")
    return 0


def play(args) -> int:
    meta_path = os.path.join(args.play, "metadata.jsonl")
    with open(meta_path, encoding="ascii") as f:
        entries = [json.loads(line) for line in f]
    header = entries.pop(0) if entries and "model" in entries[0] else {}
    if not entries:
        print(f"{meta_path} has no frames", file=sys.stderr)
        return 1
    print(f"playing {len(entries)} frame sets"
          + (f" from {header['model']}" if header else ""))
    # The clip's own depth range, so playback colors match the recording.
    colorizer = ey.Colorizer(min_mm=header.get("depth_min_mm", 0),
                             max_mm=header.get("depth_max_mm", 5000))
    t0_wall = time.time()
    t0_rec = entries[0].get("color_time") or entries[0].get("depth_time")
    for e in entries:
        t_rec = e.get("color_time") or e.get("depth_time")
        delay = (t_rec - t0_rec) - (time.time() - t0_wall)
        if delay > 0:
            time.sleep(delay)
        if "color_file" in e:
            img = cv2.imread(os.path.join(args.play, e["color_file"]))
            if img is None:                    # truncated or deleted frame
                print(f"skipping unreadable {e['color_file']}",
                      file=sys.stderr)
            else:
                cv2.imshow("Playback Color", img)
        if "depth_file" in e:
            dmm = cv2.imread(os.path.join(args.play, e["depth_file"]),
                             cv2.IMREAD_UNCHANGED)
            if dmm is None:
                print(f"skipping unreadable {e['depth_file']}",
                      file=sys.stderr)
            else:
                cv2.imshow("Playback Depth", colorizer.colorize_bgr(dmm))
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break
    cv2.destroyAllWindows()
    return 0


def snapshot_mode(args) -> int:
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
        try:
            pc = ey.PointCloud(pipeline)
        except RuntimeError as e:
            pc = None                      # uncalibrated unit: images still save
            print(f"note: {e} No PLY will be written.")

        def grab_and_save(frames):
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            verts = colors = depth_rgb = None
            if depth is not None:
                dmm = depth.get_data()       # (H, W) uint16, 1 unit = 1 mm
                depth_rgb = colorizer.colorize(dmm)
                if pc is not None:
                    # Points take the left color frame's rgb; a depth-only
                    # mode has none, so they take the depth colormap.
                    if color is not None:
                        verts, colors = pc.calculate(depth, color)
                    else:
                        verts, _ = pc.calculate(depth)
                        colors = depth_rgb.reshape(-1, 3)[dmm.flatten() != 0]
            for path in save_snapshot(args.out, color, depth, verts, colors,
                                      depth_rgb):
                print("saved", path)
            if verts is not None and len(verts):
                x, y, z = (int(v * 1000) for v in verts[verts[:, 2].argmin()])
                print(f"  {len(verts)} points, nearest "
                      f"X{x:+5d} Y{y:+5d} Z{z:5d} mm")

        if args.snapshot:
            frames = None
            t0 = time.time()
            while frames is None and time.time() - t0 < 10:
                frames = pipeline.wait_for_frames(timeout_ms=1500)
            if frames is None:
                print("no frames within 10 s; nothing saved", file=sys.stderr)
                return 1
            for _ in range(20):   # let AE settle before the capture frame
                pipeline.wait_for_frames(timeout_ms=200)
            frames = pipeline.wait_for_frames(timeout_ms=1500)
            if frames is None:
                print("the camera stopped delivering while auto-exposure "
                      "settled; nothing saved", file=sys.stderr)
                return 1
            grab_and_save(frames)
            return 0

        print("press s to save a snapshot, q / ESC to quit")
        t_tick = time.time()
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
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

            if k == ord('s'):
                grab_and_save(frames)
    cv2.destroyAllWindows()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_device_args(p)
    p.add_argument("--out", default="capture", metavar="DIR",
                   help="Output directory (default: ./capture). Snapshots "
                        "land there; each clip gets its own timestamped "
                        "subdirectory.")
    p.add_argument("--snapshot", action="store_true",
                   help="Save one snapshot set and exit, with no window and "
                        "no keypress — auto-exposure is given time to settle "
                        "first, so the saved frame is exposed properly.")
    p.add_argument("--record", type=float, default=None, metavar="SECONDS",
                   help="Record a clip this many seconds long and exit, "
                        "e.g. --record 10 for a ten-second clip.")
    p.add_argument("--play", default=None, metavar="DIR",
                   help="Play back a recorded clip directory.")
    args = p.parse_args()
    if args.play and (args.record or args.snapshot):
        p.error("--play replays an existing clip; it cannot be combined "
                "with --record or --snapshot")
    if args.play:
        return play(args)
    os.makedirs(args.out, exist_ok=True)
    if args.record:
        return record(args)
    return snapshot_mode(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
