#!/usr/bin/env python3
"""01 + a live 3D point cloud rendered with pyglet / OpenGL.

The CloudViewer class below is the display layer; everything outside it —
argument parsing, configuration and the frame loop — is line-for-line
identical to examples/03_pointcloud_open3d.py, which renders
the same cloud with Open3D instead. Left-drag rotates, middle-drag pans,
scroll zooms, R resets the view, Q / ESC quits.

Points take their color from the left color frame, and from the depth
colormap in a depth-only mode — one visible branch, and the place to
change what the cloud is colored by.
Requires: pip install pyglet opencv-python (pyglet >= 2).
"""
import argparse
import ctypes
import math
import sys
import time

import cv2
import numpy as np
import pyglet
from example_helpers import Panels, add_device_args, print_device_info
from pyglet import gl
from pyglet.graphics.shader import Shader, ShaderProgram
from pyglet.math import Mat4, Vec3
from pyglet.window import key

import pyeys3d as ey


class CloudViewer:
    """Point-cloud display layer (pyglet / OpenGL point rendering).

    The interface is shared with the Open3D version in examples/03:
      update(verts, colors)  push the newest cloud (float32 metres, optical)
      status(tag, fps, n, nearest)   rate, point count, closest point
      pump() -> bool         draw + process window events; False once closed
      close()
    """

    _VERTEX_SRC = """#version 330 core
    uniform mat4 mvp;
    in vec3 position;
    in vec3 color;
    out vec3 frag_color;
    void main() {
        gl_Position = mvp * vec4(position, 1.0);
        gl_PointSize = 2.0;
        frag_color = color;
    }
    """

    _FRAGMENT_SRC = """#version 330 core
    in vec3 frag_color;
    out vec4 out_color;
    void main() { out_color = vec4(frag_color, 1.0); }
    """

    def __init__(self, title):
        self._title = title
        self._window = pyglet.window.Window(960, 540, resizable=True,
                                            caption=title)
        self._window.switch_to()
        self._program = ShaderProgram(
            Shader(self._VERTEX_SRC, "vertex"),
            Shader(self._FRAGMENT_SRC, "fragment"))
        # One VAO with two streaming VBOs (xyz float32, rgb uint8).
        self._vao = gl.GLuint()
        gl.glGenVertexArrays(1, ctypes.byref(self._vao))
        gl.glBindVertexArray(self._vao)
        self._vbo_pos = gl.GLuint()
        self._vbo_col = gl.GLuint()
        gl.glGenBuffers(1, ctypes.byref(self._vbo_pos))
        gl.glGenBuffers(1, ctypes.byref(self._vbo_col))
        pos_loc = self._program.attributes["position"]["location"]
        col_loc = self._program.attributes["color"]["location"]
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo_pos)
        gl.glEnableVertexAttribArray(pos_loc)
        gl.glVertexAttribPointer(pos_loc, 3, gl.GL_FLOAT, gl.GL_FALSE, 0, 0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo_col)
        gl.glEnableVertexAttribArray(col_loc)
        gl.glVertexAttribPointer(col_loc, 3, gl.GL_UNSIGNED_BYTE,
                                 gl.GL_TRUE, 0, 0)
        gl.glBindVertexArray(0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_PROGRAM_POINT_SIZE)
        self._closing = False
        self._verts = None
        self._colors = None
        # Orbit view state: yaw/pitch around the cloud centroid.
        self._yaw = self._pitch = self._pan_x = self._pan_y = 0.0
        self._distance = self._home_distance = 1.5
        self._center = None

        @self._window.event
        def on_resize(w, h):
            gl.glViewport(0, 0, max(w, 1), max(h, 1))
            return pyglet.event.EVENT_HANDLED

        @self._window.event
        def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
            if buttons & pyglet.window.mouse.LEFT:
                self._yaw += dx * 0.25
                self._pitch -= dy * 0.25
            elif buttons & pyglet.window.mouse.MIDDLE:
                self._pan_x += dx * 0.001 * self._distance
                self._pan_y += dy * 0.001 * self._distance

        @self._window.event
        def on_mouse_scroll(x, y, sx, sy):
            self._distance = max(0.05, self._distance - sy * 0.08 * self._distance)

        @self._window.event
        def on_key_press(symbol, modifiers):
            if symbol in (key.Q, key.ESCAPE):
                self._closing = True
            elif symbol == key.R:
                self._yaw = self._pitch = self._pan_x = self._pan_y = 0.0
                self._distance = self._home_distance

        @self._window.event
        def on_close():
            self._closing = True
            return True

    def update(self, verts, colors):
        self._verts = verts
        if colors is None and len(verts):
            colors = np.full((len(verts), 3), 204, np.uint8)   # neutral gray
        self._colors = colors
        if self._center is None and len(verts):
            # Pivot the orbit about the first cloud's centroid.
            self._center = verts.mean(axis=0)
            self._home_distance = max(0.5, float(self._center[2]))
            self._distance = self._home_distance

    def status(self, tag, fps, npts, nearest):
        self._window.set_caption(f"{self._title}   |   {fps:.1f} fps   |   "
                                 f"{npts} pts   |   nearest {nearest}")

    def pump(self):
        if self._closing:
            return False
        self._window.switch_to()
        self._window.dispatch_events()
        if self._closing:
            return False
        self._draw()
        self._window.flip()
        return True

    def close(self):
        self._window.close()

    def _draw(self):
        w, h = self._window.get_size()
        gl.glClearColor(0.05, 0.05, 0.05, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        proj = Mat4.perspective_projection(
            w / float(h) if h else 1.0, 0.01, 50.0, fov=60.0)
        view = (Mat4.from_translation(
                    Vec3(self._pan_x, self._pan_y, -self._distance))
                @ Mat4.from_rotation(math.radians(self._pitch), Vec3(1, 0, 0))
                @ Mat4.from_rotation(math.radians(self._yaw), Vec3(0, 1, 0))
                @ Mat4.from_scale(Vec3(1.0, -1.0, -1.0)))  # optical -> view
        if self._center is not None:
            view = view @ Mat4.from_translation(
                Vec3(-float(self._center[0]), -float(self._center[1]),
                     -float(self._center[2])))

        verts, colors = self._verts, self._colors
        if verts is None or not len(verts):
            return
        n = len(verts)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo_pos)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes,
                        verts.ctypes.data_as(ctypes.c_void_p),
                        gl.GL_STREAM_DRAW)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo_col)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, colors.nbytes,
                        colors.ctypes.data_as(ctypes.c_void_p),
                        gl.GL_STREAM_DRAW)
        self._program.use()
        self._program["mvp"] = proj @ view
        gl.glBindVertexArray(self._vao)
        gl.glDrawArrays(gl.GL_POINTS, 0, n)
        gl.glBindVertexArray(0)
        self._program.stop()


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
