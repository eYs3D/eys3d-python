#!/usr/bin/env python3
"""01 + a live 3D point cloud rendered with Open3D.

The CloudViewer class below is the display layer — it shows the numpy →
Open3D conversion for projects already using Open3D; everything outside it —
argument parsing, configuration and the frame loop — is line-for-line
identical to examples/02_pointcloud.py, which renders the
same cloud with pyglet/OpenGL instead. Drag rotates, scroll zooms, R resets
the view, Q / ESC quits.

Points take their color from the left color frame, and from the depth
colormap in a depth-only mode — one visible branch, and the place to
change what the cloud is colored by.
Requires: pip install open3d opencv-python.
"""
import argparse
import shutil
import sys
import time

import cv2
import numpy as np
import open3d as o3d
from example_helpers import Panels, add_device_args, print_device_info

import pyeys3d as ey


class CloudViewer:
    """Point-cloud display layer (Open3D visualizer).

    The interface is shared with the pyglet/OpenGL version in examples/02:
      update(verts, colors)  push the newest cloud (float32 metres, optical)
      status(tag, fps, n, nearest)   rate, point count, closest point
      pump() -> bool         draw + process window events; False once closed
      close()
    """

    def __init__(self, title):
        self._vis = o3d.visualization.VisualizerWithKeyCallback()
        self._vis.create_window(title, width=960, height=540)
        self._vis.get_render_option().point_size = 1.0
        self._cloud = o3d.geometry.PointCloud()
        self._added = False
        self._closing = False
        self._home_view = None      # captured after the first cloud lands
        for k in (ord('Q'), 256):   # Q / Escape
            self._vis.register_key_callback(
                k, lambda v: (setattr(self, "_closing", True), False)[1])
        self._vis.register_key_callback(ord('R'), lambda v: (
            v.set_view_status(self._home_view) if self._home_view else None,
            False)[1])

    def update(self, verts, colors):
        # Optical (X right, Y down, Z forward) -> Open3D view: negate Y, Z.
        self._cloud.points = o3d.utility.Vector3dVector(
            verts.astype(np.float64) * (1.0, -1.0, -1.0))
        if colors is not None:
            self._cloud.colors = o3d.utility.Vector3dVector(
                colors.astype(np.float64) / 255.0)
        if not self._added:
            self._vis.add_geometry(self._cloud)
            self._added = True
        else:
            self._vis.update_geometry(self._cloud)

    def status(self, tag, fps, npts, nearest):
        # Open3D cannot retitle its window.
        line = f"{tag} | {fps:5.1f} fps | {npts:>7} pts | nearest {nearest}"
        # Clip to the terminal width and pad, keeping the \r status one row.
        cols = shutil.get_terminal_size().columns - 1
        print(f"\r{line[:cols]:<{cols}}", end="", flush=True)

    def pump(self):
        if self._closing:
            return False
        alive = self._vis.poll_events()
        self._vis.update_renderer()
        if self._added and self._home_view is None:
            self._home_view = self._vis.get_view_status()
        return alive and not self._closing

    def close(self):
        print()
        self._vis.destroy_window()


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
        try:
            pc = ey.PointCloud(pipeline)
        except RuntimeError as e:
            print(e, file=sys.stderr)
            return 1
        viewer = CloudViewer(f"{tag} Point Cloud")

        t_tick, npts = time.time(), 0
        nearest = "no points"
        try:
            while True:
                # Short timeout: the 3D window is pumped every iteration.
                frames = pipeline.wait_for_frames(timeout_ms=30)
                if not viewer.pump():
                    break
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
                    # colorize() not colorize_bgr(): the rgb is reused as
                    # the point color below.
                    depth_rgb = colorizer.colorize(dmm)
                    panels.show(f"{tag} Depth", "depth", dmm,
                                depth_rgb[:, :, ::-1].copy())

                    # Points take the left color frame's rgb; a depth-only
                    # mode has none, so they take the depth colormap.
                    if color is not None:
                        verts, colors = pc.calculate(depth, color)
                    else:
                        verts, _ = pc.calculate(depth)
                        colors = depth_rgb.reshape(-1, 3)[dmm.flatten() != 0]
                    if len(verts):
                        npts = len(verts)
                        x, y, z = (int(v * 1000)
                                   for v in verts[verts[:, 2].argmin()])
                        nearest = f"X{x:+5d} Y{y:+5d} Z{z:5d} mm"
                        viewer.update(verts, colors)

                now = time.time()
                if now - t_tick >= 1.0:
                    t_tick = now
                    fps = pipeline.fps
                    panels.tick(fps)
                    # Wire drops point at USB bandwidth or scheduling.
                    drops = pipeline.frames_dropped
                    if drops != last_drops:
                        print(f"warning: {tag} wire drops "
                              f"color={drops.color} depth={drops.depth}")
                        last_drops = drops
                    viewer.status(tag, fps.depth, npts, nearest)
        finally:
            viewer.close()
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
