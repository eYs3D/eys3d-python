#!/usr/bin/env python3
"""Every camera setting on one screen, changed while the camera runs.

01-06 each show one part of the API in the smallest code that works.
This drives all of it from a menu instead, so a camera can be taken
through its range without editing anything. Everything is reached on
screen; the only option is where files are written.

  - Color (Left / Right when the mode splits L|R) and Depth, hover reads
    the pixel value in the title bar.
  - Live camera controls: IR, auto-exposure, exposure, auto-white-balance,
    white balance, power-line frequency — applied on the streaming device.
  - Video mode, depth clip and depth filters, staged in the menu and
    applied together by Enter; each reopens the stream on the same camera
    (bound by serial / USB port), which is the only way these three can
    change.
  - Snapshot (PNG + 16-bit depth + PLY) and clip recording, for a record
    of a verification run.
  - An optional 3D point cloud in a separate OpenGL window (press p).

Depth is drawn false-color and cloud points take the color frame, both
without asking — 01 and 02 show the one line each that changes. What is
not here: the per-frame metadata 01 prints with --frame-meta, the full
camera model it prints at startup, and 04's clip playback.

Keys:
    ↑ ↓ ← →        move between menu cells
    - / +          adjust the selected value, or toggle it
    Enter          apply staged video-mode / depth-clip / filter changes
    d              restore camera properties to their defaults
    x              hardware-reset the camera (USB re-enumeration)
    p              open / close the 3D point cloud window
    s              save a snapshot set
    r              start / stop recording a clip
    q / ESC        quit

Every connected camera opens in this one process, each with its own window
set titled model[usb_port]. Click a camera's window to send it the keys.
Several cameras stop on a mode picker (no preview) first: pick a mode and
press Enter to start that one, or press q to close it. Choose modes the
shared USB bus can carry together — one camera at its signature mode can
take most of a USB 3 link. One process is deliberate: Windows slows a
process whose windows are not in front, so a viewer per camera would leave
every camera but the focused one running at a reduced rate.

Requires: pip install opencv-python. The 3D window also needs pyglet.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from example_helpers import Panels, print_device_info

import pyeys3d as ey
from pyeys3d import load_catalog, load_model

_PLF_NAMES = {1: "50Hz", 2: "60Hz"}

# cv2.waitKeyEx arrow-key codes (X11 / Windows) and the adjust keys.
_UP = {65362, 2490368}
_DOWN = {65364, 2621440}
_LEFT = {65361, 2424832}
_RIGHT = {65363, 2555904}
_ENTER = {13, 10}
_MINUS = {45, 95}          # '-' / '_' : decrease / toggle
_PLUS = {43, 61}           # '+' / '=' : increase / toggle

# Depth post-process stages, each an independent on/off in the menu.
FILTER_DEFS = [("spatial", "Spatial"),
               ("temporal", "Temporal"),
               ("hole", "Hole-fill")]

# Settings that only take effect when the stream is reopened: the depth
# clip and the filter chain are both fixed at start(). The menu stages a
# copy and Enter applies it, so a sweep of the range costs one reopen.
_DEPTH_STEP_MM = 50
_DEPTH_MAX_MM = 16383        # Z14 depth is 14-bit: 2^14 - 1 mm

# Control indices within make_control_rows()'s list.
_IR, _AE, _EXP, _AWB, _WB, _PLF = range(6)

# Menu layout: display rows of cells, packed two/three per line to save
# height. A cell is (kind, ref): ("mode", None) is the full-width video-mode
# picker; ("ctl", i) is camera control i; ("flt", key) is a filter toggle.
# The cursor is a (row, col) index navigated in 2D by the arrow keys.
_MENU_LAYOUT: list = [
    [("mode", None)],
    [("ctl", _IR), ("ctl", _PLF)],
    [("ctl", _AE), ("ctl", _EXP)],
    [("ctl", _AWB), ("ctl", _WB)],
    [("dep", "near"), ("dep", "far")],
    [("flt", "spatial"), ("flt", "temporal"), ("flt", "hole")],
]


def norm_key(k):
    """Drop the modifier-state bits OpenCV's GTK backend packs into the
    high half, so NumLock or CapsLock does not stop a key matching.

    Windows extended keys (the arrows) carry their code in the high half
    and leave the low half zero, so a zero low half means the whole value
    is the key. Testing the low half for the keysym range instead would
    cover the arrows and miss every ASCII key."""
    low = k & 0xFFFF
    return low if k > 0 and low else k


# --- disk output (snapshots and clips) --------------------------------------

def file_stamp() -> str:
    """Timestamp shared by snapshot files and clip directories; the
    millisecond suffix keeps rapid saves from colliding."""
    return (time.strftime("%Y%m%d-%H%M%S")
            + f"-{int(time.time() * 1000) % 1000:03d}")


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


def _prepare_out_dir(out_dir: str) -> str:
    """Create the capture directory and return it as an absolute path.

    The default is relative, so it follows the working directory: resolving
    it here means every "saved ..." line names a findable file. A directory
    that cannot be created (a read-only working directory) falls back to the
    home directory rather than failing the run.
    """
    resolved = os.path.abspath(out_dir)
    try:
        os.makedirs(resolved, exist_ok=True)
    except OSError as e:
        fallback = os.path.join(os.path.expanduser("~"),
                                os.path.basename(resolved) or "capture")
        print(f"cannot write to {resolved} ({e}); using {fallback}",
              file=sys.stderr)
        resolved = fallback
        os.makedirs(resolved, exist_ok=True)
    print(f"captures -> {resolved}", flush=True)
    return resolved


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


# --- optional 3D point cloud window (pyglet / OpenGL) -----------------------

class CloudViewer:
    """Point-cloud display layer (pyglet / OpenGL point rendering).

    Left-drag orbits, middle-drag pans, scroll zooms, R resets the view.
    The window, its GL context and its event pump live on a dedicated
    thread so they never share the main thread's message pump with the
    OpenCV windows; on the same thread the two block each other. update()
    hands over the newest cloud; pump() reports whether the window is open.
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
        self._lock = threading.Lock()
        self._verts = None
        self._colors = None
        self._pending_caption = None
        self._closing = False
        self._center = None
        self._yaw = self._pitch = self._pan_x = self._pan_y = 0.0
        self._distance = self._home_distance = 1.5
        self._ready = threading.Event()
        self._init_error = None
        self._thread = threading.Thread(target=self._run, name="cloud-viewer",
                                        daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=15.0):
            # Init neither succeeded nor raised: a GL context create that
            # blocks rather than failing, which some virtualized and remote
            # display drivers do. Treated as a failure so the caller is
            # never left waiting.
            self._closing = True
            raise RuntimeError(
                "point-cloud window did not initialize within 15 s "
                "(no usable OpenGL context?)")
        if self._init_error is not None:
            raise self._init_error

    def update(self, verts, colors):
        if colors is None and verts is not None and len(verts):
            colors = np.full((len(verts), 3), 204, np.uint8)
        with self._lock:
            self._verts = verts
            self._colors = colors

    def set_caption(self, text) -> None:
        with self._lock:
            self._pending_caption = text

    def pump(self) -> bool:
        """Whether the window is still open. The window pumps itself on its
        own thread; this only lets the main loop notice it being closed."""
        return self._thread.is_alive() and not self._closing

    def close(self) -> None:
        self._closing = True
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            import pyglet
            from pyglet import gl
            from pyglet.graphics.shader import Shader, ShaderProgram
            from pyglet.window import key
            if sys.platform == "win32":
                # A GUI window's message pump behaves correctly only from an
                # apartment-threaded COM context; claim one for this thread so
                # the OpenCV windows keep the main thread's apartment.
                ctypes.windll.ole32.CoInitializeEx(None, 0x2)
            self._gl = gl
            win = pyglet.window.Window(960, 540, resizable=True,
                                       caption=self._title)
            self._window = win
            win.switch_to()
            self._program = ShaderProgram(
                Shader(self._VERTEX_SRC, "vertex"),
                Shader(self._FRAGMENT_SRC, "fragment"))
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

            @win.event
            def on_resize(w, h):
                gl.glViewport(0, 0, max(w, 1), max(h, 1))
                return pyglet.event.EVENT_HANDLED

            @win.event
            def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
                if buttons & pyglet.window.mouse.LEFT:
                    self._yaw += dx * 0.25
                    self._pitch -= dy * 0.25
                elif buttons & pyglet.window.mouse.MIDDLE:
                    self._pan_x += dx * 0.001 * self._distance
                    self._pan_y += dy * 0.001 * self._distance

            @win.event
            def on_mouse_scroll(x, y, sx, sy):
                self._distance = max(
                    0.05, self._distance - sy * 0.08 * self._distance)

            @win.event
            def on_key_press(symbol, modifiers):
                if symbol in (key.Q, key.ESCAPE):
                    self._closing = True
                elif symbol == key.R:
                    self._yaw = self._pitch = self._pan_x = self._pan_y = 0.0
                    self._distance = self._home_distance

            @win.event
            def on_close():
                self._closing = True
                return True
        except BaseException as e:      # let __init__ re-raise (e.g. no pyglet)
            self._init_error = e
            self._ready.set()
            return
        self._ready.set()
        self._loop()

    def _loop(self) -> None:
        win = self._window
        while not self._closing:
            win.switch_to()
            win.dispatch_events()
            if self._closing:
                break
            with self._lock:
                verts, colors = self._verts, self._colors
                cap, self._pending_caption = self._pending_caption, None
            if cap is not None:
                win.set_caption(cap)
            if self._center is None and verts is not None and len(verts):
                self._center = verts.mean(axis=0)
                self._home_distance = max(0.5, float(self._center[2]))
                self._distance = self._home_distance
            self._draw(verts, colors)
            win.flip()
            time.sleep(0.004)
        try:
            win.close()
        except Exception:
            pass

    def _draw(self, verts, colors) -> None:
        gl = self._gl
        from pyglet.math import Mat4, Vec3
        w, h = self._window.get_size()
        gl.glClearColor(0.05, 0.05, 0.05, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        proj = Mat4.perspective_projection(
            w / float(h) if h else 1.0, 0.01, 50.0, fov=60.0)
        view = (Mat4.from_translation(
                    Vec3(self._pan_x, self._pan_y, -self._distance))
                @ Mat4.from_rotation(math.radians(self._pitch), Vec3(1, 0, 0))
                @ Mat4.from_rotation(math.radians(self._yaw), Vec3(0, 1, 0))
                @ Mat4.from_scale(Vec3(1.0, -1.0, -1.0)))   # optical -> view
        if self._center is not None:
            view = view @ Mat4.from_translation(
                Vec3(-float(self._center[0]), -float(self._center[1]),
                     -float(self._center[2])))

        if verts is None or not len(verts):
            return
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
        gl.glDrawArrays(gl.GL_POINTS, 0, len(verts))
        gl.glBindVertexArray(0)
        self._program.stop()


# --- camera-control menu ----------------------------------------------------

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


class Session:
    """The per-open stream state: colorizer, point cloud, control menu.
    Rebuilt each time the stream reopens on a new video mode or filter
    setting, since resolution and calibration change with the mode."""

    def __init__(self, pipeline):
        self.pipeline = pipeline
        # Depth is always shown through the range-mapped colormap: it carries
        # more than grayscale, and the point cloud falls back to these same
        # colors when the mode has no color stream.
        self.colorizer = ey.Colorizer(pipeline)
        try:
            self.pc = ey.PointCloud(pipeline)
        except RuntimeError:
            self.pc = None                        # uncalibrated: no point cloud
        er = pipeline.get_exposure_range()
        wr = pipeline.get_white_balance_range()
        self.e_step = er.step if er and er.step > 0 else 1
        self.wb_step = wr.step if wr and wr.step > 0 else 100
        self.rows = make_control_rows(pipeline, self.e_step)
        self.wb_row = next(i for i, (_, _, adj) in enumerate(self.rows)
                           if adj is None)
        self.vals = [str(read()) for _, read, _ in self.rows]

    def read_vals(self):
        self.vals = [str(read()) for _, read, _ in self.rows]


def _copy_stream(stream):
    """A staged copy: the filter flags are nested, so a shallow copy would
    let the pending set edit the applied one."""
    return {"filters": dict(stream["filters"]),
            "near": stream["near"], "far": stream["far"],
            "ir": stream["ir"]}


def open_stream(pipeline, model, dev, mode_id, stream):
    """(Re)open the stream on one camera in a given mode, and return a
    fresh Session. The camera is pinned by serial / USB port so a reopen
    cannot land on a different device. `stream` carries what only start()
    can set: the depth clip, and the post-process stages individually
    (spatial / temporal / hole)."""
    cfg = ey.Config().enable_device(model, mode_id=mode_id,
                                    serial_number=dev.serial_number,
                                    usb_port=dev.usb_port)
    cfg.set_depth_range(stream["near"], stream["far"])
    # Carried explicitly: a fresh Config resolves an unset IR to the model
    # default, so a reopen for a filter change would undo the user's IR.
    if stream["ir"] >= 0:
        cfg.set_ir_value(stream["ir"])
    filters = stream["filters"]
    active = []
    if filters["spatial"]:
        active.append(ey.SpatialFilter())
    if filters["temporal"]:
        active.append(ey.TemporalFilter())
    if filters["hole"]:
        active.append(ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND))
    if active:
        cfg.with_filters(*active)
    pipeline.start(cfg)
    print_device_info(pipeline)
    return Session(pipeline)


# --- main loop --------------------------------------------------------------

class CameraView:
    """One camera: its windows, its menu state and its stream.

    The viewer holds one of these per connected camera and steps them all
    from a single loop. Windows slows a process whose windows are not in
    front, so a process per camera leaves every camera but the focused one
    at a reduced rate; holding them in one process keeps every camera at the
    foreground rate. Each view owns a Controls window naming its camera, and
    keys go to whichever view was clicked last.

    step(key) advances one frame: it drives the mode picker before the first
    open, the status while a (re)open runs off-thread, and the streaming
    loop after that. It returns False once the view has closed itself.
    """

    def __init__(self, dev, out_dir, pick: bool):
        self.dev = dev
        self.model = dev.model
        # Window titles carry model[usb_port] so several cameras are told
        # apart, and the control window says which module it drives.
        self.tag = f"{dev.model}[{dev.usb_port}]"
        self.controls_win = f"{self.tag} Controls"
        self.out_dir = out_dir
        self.catalog = load_catalog(self.model)
        self.usb = dev.usb_port_type
        # Only the modes this USB link can carry; unknown port type shows all.
        self.mode_ids = [m for m in sorted(self.catalog)
                         if self.usb not in (2, 3)
                         or self.catalog[m].usb == self.usb]
        if not self.mode_ids:
            self.mode_ids = sorted(self.catalog)
        self.mode_id = load_model(self.model).signature_for(self.usb)
        if self.mode_id not in self.mode_ids:    # signature mode off this link
            self.mode_ids = [self.mode_id] + self.mode_ids
        # -1 asks for the model's own default; the values the camera
        # reports after start() replace them.
        self.stream = {"filters": {k: False for k, _ in FILTER_DEFS},
                       "near": -1, "far": -1, "ir": -1}
        self.stream_pending = _copy_stream(self.stream)

        self.pipeline = ey.Pipeline()
        self.session = None
        self.panels = Panels(resizable=True, on_click=self.take_focus)
        self.cloud = None                # CloudViewer, created on first p press
        self.rec = None                  # recording state, set between r presses
        self.sel = (0, 0)                # (row, col) cursor over _MENU_LAYOUT
        self.cur_mode = self.mode_id     # the mode the stream is open in
        self.mode_pick = self.mode_ids.index(self.mode_id)
        self.pick_idx = self.mode_pick   # cursor in the pre-stream picker
        self.wb_pending = None
        self.wb_last_key = 0.0
        self.seen_reconnects = 0
        self.npts, self.nearest = 0, "no points"
        # Last frame that arrived, for the keys that act on one.
        self.shown = (None, None, None)
        self.t_tick = time.time()
        self.alive = True
        self.solo = True             # cleared by the loop while others are up
        self.focused = True          # set by the loop; only it takes keys
        self.wants_focus = False
        self.ui = {"img": None}          # last control image (frozen on busy)
        # A mode's cold start blocks start() for several seconds, so every
        # open runs on a worker thread while the loop keeps windows responsive.
        self.switch = {"thread": None, "target": None, "result": None,
                       "error": None}

        cv2.namedWindow(self.controls_win,
                        cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback(self.controls_win, self._on_controls_mouse)
        # Several cameras stop here so the user can fit them to what the
        # shared USB bus carries — one camera at its signature mode can take
        # most of a USB 3 link.
        self.state = "pick" if pick else "opening"
        if not pick:
            self._begin_open(self.mode_id, _copy_stream(self.stream))

    # --- focus ---------------------------------------------------------

    def take_focus(self) -> None:
        """Called from an OpenCV mouse callback on any of this view's
        windows. cv2 cannot report which window has focus, so the last one
        clicked is the one the keys drive."""
        self.wants_focus = True

    def _on_controls_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.take_focus()

    # --- opening -------------------------------------------------------

    def _begin_open(self, mode_id, stream):
        """Start a threaded (re)open. The first open and a mode / depth /
        filter change take the same path: both cold-start the stream."""
        if self.switch["thread"] is not None:
            return
        # IR is a live control, not a staged one, so the staged copy carries
        # whatever it held at the last open. Take the device's current value
        # instead, or the reopen resets the projector to the model default.
        if self.session is not None:
            live_ir = self.pipeline.get_ir_value()
            if live_ir is not None:
                stream["ir"] = live_ir
        self.switch.update(target=mode_id, result=None, error=None)
        self.switch["thread"] = threading.Thread(
            target=self._open_worker, args=(mode_id, stream), daemon=True)
        self.switch["thread"].start()
        self.state = "opening"

    def _open_worker(self, new_mode, new_stream):
        try:
            self.pipeline.stop()
            s = open_stream(self.pipeline, self.model, self.dev, new_mode,
                            new_stream)
            self.switch["result"] = (new_mode, new_stream, s)
        except (ValueError, RuntimeError) as e:
            # Device rejected the mode (e.g. a USB3-only mode on a USB2 link);
            # fall back to the last good open so the view survives. Before the
            # first open that is the signature mode.
            self.switch["error"] = str(e)
            fallback = (self.cur_mode if self.session is not None
                        else load_model(self.model).signature_for(self.usb))
            fb_stream = _copy_stream(self.stream)
            # Give up only when the fallback is what was just refused. A
            # filter or depth-clip change keeps the mode, so the mode alone
            # matching is not a dead end — reopening it with the settings
            # that were working is, and that is the common transient here.
            if fallback == new_mode and fb_stream == new_stream:
                self.switch["result"] = None
                return
            try:
                s = open_stream(self.pipeline, self.model, self.dev, fallback,
                                fb_stream)
                self.switch["result"] = (fallback, fb_stream, s)
            except (ValueError, RuntimeError):
                self.switch["result"] = None

    def _step_opening(self, key) -> None:
        # No camera access while the worker owns the pipeline; keep the
        # windows pumping. The first open does not dim the menu (there is
        # none yet), a reopen dims the frozen one.
        if self.cloud is not None and not self.cloud.pump():
            self.cloud.close()
            self.cloud = None
        _render_status(self.controls_win, self.ui, "Opening",
                       dim=self.session is not None)
        if self.switch["thread"].is_alive():
            if key in (ord('q'), 27):
                self.alive = False
            return
        self.switch["thread"].join()
        self.switch["thread"] = None
        if self.switch["error"]:
            print(f"{self.tag}: cannot open mode {self.switch['target']}: "
                  f"{self.switch['error']}", file=sys.stderr)
        if self.switch["result"] is None:
            # Target and fallback both failed: the pipeline is stopped and
            # cannot be read, so this camera is done.
            print(f"{self.tag}: stream could not be opened", file=sys.stderr)
            self.alive = False
            return
        self.cur_mode, applied, self.session = self.switch["result"]
        self.mode_pick = self.mode_ids.index(self.cur_mode)
        self.stream = applied
        self.stream["near"] = self.pipeline.depth_near_mm
        self.stream["far"] = self.pipeline.depth_far_mm
        # Read back like the clip range: the menu changes IR on the live
        # device, and the next reopen has to carry that value forward
        # rather than let a fresh Config resolve it to the model default.
        self.stream["ir"] = self.pipeline.get_ir_value()
        self.stream_pending = _copy_stream(self.stream)
        self.seen_reconnects = self.pipeline.reconnect_count
        self.last_drops = self.pipeline.frames_dropped
        self.wb_pending = None
        self.sel = _clamp_sel(self.sel, self.session.vals)
        self.panels.reset()          # windows back to native resolution
        # A USB3-only mode opens on a USB2 link but delivers no frames; the
        # picker hides these, so this only fires for a signature-mode fallback.
        if self.usb in (2, 3) and self.catalog[self.cur_mode].usb != self.usb:
            print(f"warning: {self.tag} mode {self.cur_mode} needs USB"
                  f"{self.catalog[self.cur_mode].usb} but the link is "
                  f"{self.dev.usb_speed}; it may deliver no frames",
                  file=sys.stderr)
        self.state = "stream"

    # --- pre-stream mode picker ----------------------------------------

    def _step_pick(self, key) -> None:
        """Mode selection with no preview: show this camera's modes and wait
        for a choice, so several cameras can be opened at rates the shared
        bus carries together."""
        m = self.catalog[self.mode_ids[self.pick_idx]]
        header = (f"Select mode for {self.tag} [{self.dev.usb_speed}]"
                  f"   ({self.pick_idx + 1}/{len(self.mode_ids)})")
        waiting = "Waiting for preview"
        if not self.focused:
            waiting += "   -   click this window to control it"
        items = [_status_item(waiting),
                 ("cells", [(header, True, True)])]
        items += [("info", ln) for ln in _mode_detail_lines(m)]
        items.append(("hint", ("Move   ", "[Enter] Start", "   [q] Close",
                               True)))
        _render_controls(self.controls_win, items, self.ui)
        if key in (ord('q'), 27):
            self.alive = False
        elif key in _UP or key in _MINUS:
            self.pick_idx = (self.pick_idx - 1) % len(self.mode_ids)
        elif key in _DOWN or key in _PLUS:
            self.pick_idx = (self.pick_idx + 1) % len(self.mode_ids)
        elif key in _ENTER:
            self._begin_open(self.mode_ids[self.pick_idx],
                             _copy_stream(self.stream))

    # --- streaming ------------------------------------------------------

    def _stop_recording(self) -> None:
        if self.rec is not None:
            self.rec["meta"].close()
            print(f"recorded {self.rec['index']} frame sets to "
                  f"{self.rec['dir']}/")
            self.rec = None

    def _step_stream(self, key) -> None:
        pipeline, session, tag = self.pipeline, self.session, self.tag
        if key in (ord('q'), 27):
            self.alive = False
            return
        # One camera can afford to block for its frame. Several share this
        # loop, so a wait long enough to matter would stall the others for
        # exactly as long; poll instead and come back next pass.
        if not self.solo:
            timeout_ms = 15
        else:
            timeout_ms = 30 if self.cloud else 1000
        frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
        if self.cloud is not None and not self.cloud.pump():
            self.cloud.close()       # closing via the 3D window's X or q/ESC
            self.cloud = None

        # A reconnect reopens on the kept mode and re-applies controls;
        # re-read what the device now holds.
        if pipeline.reconnect_count != self.seen_reconnects:
            self.seen_reconnects = pipeline.reconnect_count
            session.read_vals()
        if (self.wb_pending is not None
                and time.monotonic() - self.wb_last_key > 0.3):
            try:
                pipeline.set_white_balance(self.wb_pending)
            except (ValueError, RuntimeError) as e:
                print(e)
            self.wb_pending = None
            session.read_vals()

        # Controls render in their own window; the image windows stay clean
        # so the color / depth frames are fully readable.
        state = "Streaming" if pipeline.is_connected else "Reconnecting"
        if not self.focused:
            state += "   -   click this window to control it"
        _render_controls(self.controls_win, [_status_item(state)]
                         + _build_menu(
            session, self.sel, self.dev.usb_speed,
            self.catalog[self.mode_ids[self.mode_pick]],
            self.mode_ids[self.mode_pick] != self.cur_mode,
            self.stream, self.stream_pending, pipeline, self.rec), self.ui)

        if frames is None:
            # No frame this pass, but the key still has to land: [x] exists
            # for a wedged camera, which delivers nothing by definition.
            if key != -1:
                self._handle_key(key, *self.shown)
            return
        color = frames.get_color_frame()
        right = frames.get_right_color_frame()
        depth = frames.get_depth_frame()

        # Panels take the raw frame for hover, bgr to display.
        if color is not None:
            self.panels.show(f"{tag} Color (Left)", "color",
                             color.get_data(), color.get_data_bgr())
        if right is not None:
            self.panels.show(f"{tag} Color (Right)", "color",
                             right.get_data(), right.get_data_bgr())
        depth_rgb = None
        if depth is not None:
            dmm = depth.get_data()       # (H, W) uint16, 1 unit = 1 mm
            # colorize() not colorize_bgr(): the rgb is reused as the point
            # color below.
            depth_rgb = session.colorizer.colorize(dmm)
            self.panels.show(f"{tag} Depth", "depth", dmm,
                             depth_rgb[:, :, ::-1].copy())

        # Compute the cloud only when the 3D window is open; snapshots
        # compute it on demand at the s press.
        if depth is not None and self.cloud is not None:
            verts, colors = _cloud_from(session, depth, color, depth_rgb)
            if verts is not None and len(verts):
                self.npts = len(verts)
                x, y, z = (int(v * 1000) for v in verts[verts[:, 2].argmin()])
                self.nearest = f"X{x:+5d} Y{y:+5d} Z{z:5d} mm"
                self.cloud.update(verts, colors)

        if self.rec is not None:
            entry = save_frame_set(self.rec["dir"], self.rec["index"], color,
                                   depth)
            self.rec["meta"].write(json.dumps(entry) + "\n")
            self.rec["index"] += 1

        now = time.time()
        if now - self.t_tick >= 1.0:
            self.t_tick = now
            fps = pipeline.fps
            self.panels.tick(fps)
            # Wire drops point at USB bandwidth or scheduling.
            drops = pipeline.frames_dropped
            if drops != self.last_drops:
                print(f"warning: {tag} wire drops "
                      f"color={drops.color} depth={drops.depth}")
                self.last_drops = drops
            if self.cloud is not None:
                self.cloud.set_caption(
                    f"{tag} Point Cloud   |   {fps.depth:.1f} fps"
                    f"   |   {self.npts} pts   |   nearest {self.nearest}")

        self.shown = (color, depth, depth_rgb)
        if key != -1:
            self._handle_key(key, color, depth, depth_rgb)

    def _handle_key(self, k, color, depth, depth_rgb) -> None:
        pipeline, session = self.pipeline, self.session
        # Arrows move the 2D cursor; the grid skips disabled (auto) cells.
        if k in _UP:
            self.sel = _move_sel(self.sel, -1, 0, session.vals)
        elif k in _DOWN:
            self.sel = _move_sel(self.sel, +1, 0, session.vals)
        elif k in _LEFT:
            self.sel = _move_sel(self.sel, 0, -1, session.vals)
        elif k in _RIGHT:
            self.sel = _move_sel(self.sel, 0, +1, session.vals)
        elif k in _MINUS or k in _PLUS:
            # -/+ changes the selected cell: adjust a value, cycle the mode
            # candidate, or flip a toggle (AE / AWB / filter).
            d = -1 if k in _MINUS else +1
            kind, ref = _MENU_LAYOUT[self.sel[0]][self.sel[1]]
            if kind == "mode":
                self.mode_pick = (self.mode_pick + d) % len(self.mode_ids)
            elif kind == "dep":
                near, far = (self.stream_pending["near"],
                             self.stream_pending["far"])
                v = (near if ref == "near" else far) + d * _DEPTH_STEP_MM
                # Keep the pair ordered and inside what Z14 depth can carry,
                # so Enter cannot stage a range start() would reject.
                if ref == "near":
                    self.stream_pending["near"] = max(
                        0, min(v, far - _DEPTH_STEP_MM))
                else:
                    self.stream_pending["far"] = max(
                        near + _DEPTH_STEP_MM, min(v, _DEPTH_MAX_MM))
            elif kind == "flt":
                f = self.stream_pending["filters"]
                f[ref] = not f[ref]
            elif ref in (_AE, _AWB):
                _toggle_control(session, ref)
                self.sel = _clamp_sel(self.sel, session.vals)
            elif ref == session.wb_row:
                base = (self.wb_pending if self.wb_pending is not None
                        else (pipeline.get_white_balance() or 0))
                self.wb_pending = max(0, base + d * session.wb_step)
                self.wb_last_key = time.monotonic()
                session.vals[ref] = f"{self.wb_pending} (pending)"
            else:
                try:
                    session.rows[ref][2](d)
                    session.read_vals()
                except (ValueError, RuntimeError) as e:
                    print(e)
        elif k in _ENTER:
            # Enter commits staged mode / clip / filter changes (a reopen).
            # Locked while recording so a clip stays one continuous mode.
            if self.rec is not None:
                print("stop recording (r) before changing mode / filters")
            elif (self.mode_ids[self.mode_pick] != self.cur_mode
                    or self.stream_pending != self.stream):
                self._begin_open(self.mode_ids[self.mode_pick],
                                 _copy_stream(self.stream_pending))
        elif k == ord('p'):
            if session.pc is None:
                # No calibration: the window would open and stay empty.
                print(f"{self.tag}: no point cloud - this unit reports no "
                      f"calibration", file=sys.stderr)
            elif self.cloud is None:
                try:
                    self.cloud = CloudViewer(f"{self.tag} Point Cloud")
                except ImportError:
                    print("3D view needs pyglet: pip install pyglet",
                          file=sys.stderr)
                except Exception as e:
                    # GL 3.3 / shader / context init can fail on a headless /
                    # RDP / weak-GPU host; keep the camera session alive
                    # instead of taking the whole viewer down.
                    self.cloud = None
                    print(f"3D view unavailable: {e}", file=sys.stderr)
            else:
                self.cloud.close()
                self.cloud = None
        elif k == ord('s'):
            sv, sc = None, None
            if depth is not None:
                sv, sc = _cloud_from(session, depth, color, depth_rgb)
            for path in save_snapshot(self.out_dir, color, depth, sv, sc,
                                      depth_rgb):
                print("saved", path)
        elif k == ord('r'):
            if self.rec is None:
                self.rec = _start_recording(self.out_dir, pipeline)
            else:
                self._stop_recording()
        elif k == ord('d'):
            # Restore camera properties to their defaults (IR to the catalog
            # default, AE / AWB to auto). eSPDI has no reset call, so the
            # defaults are written directly; a control the model lacks (e.g.
            # no white balance on a mono unit) is skipped.
            self.wb_pending = None
            for restore in (lambda: pipeline.set_ir_value(-1),
                            lambda: pipeline.set_auto_exposure(True),
                            lambda: pipeline.set_auto_white_balance(True)):
                try:
                    restore()
                except (ValueError, RuntimeError):
                    pass
            session.read_vals()
            self.sel = _clamp_sel(self.sel, session.vals)  # AE/AWB may grey one
        elif k == ord('x'):
            # Locked while recording so a clip is not cut mid-reset.
            if self.rec is not None:
                print("stop recording (r) before a hardware reset")
            else:
                print(f"{self.tag}: hardware reset - the camera drops off "
                      f"the bus and reconnects")
                pipeline.hardware_reset()

    # --- loop entry -----------------------------------------------------

    def step(self, key, focused: bool = True) -> bool:
        """Advance one frame. Only the focused view is given the key; the
        others are stepped with -1 so they keep streaming. Returns False
        once this view has closed itself."""
        self.focused = focused
        if self.state == "pick":
            self._step_pick(key)
        elif self.state == "opening":
            self._step_opening(key)
        else:
            self._step_stream(key)
        return self.alive

    def close(self) -> None:
        # Let an in-flight open finish so it does not leave the device open
        # after the view is gone (the worker owns pipeline during a switch).
        if self.switch["thread"] is not None:
            self.switch["thread"].join()
            self.switch["thread"] = None
        self._stop_recording()
        if self.cloud is not None:
            self.cloud.close()
            self.cloud = None
        self.panels.reset()
        try:
            cv2.destroyWindow(self.controls_win)
        except cv2.error:
            pass
        self.pipeline.stop()


def _run(args) -> int:
    """Open every connected camera in this one process and step them from a
    single loop, so none of them is ever a background workload."""
    supported = set(ey.supported_models())
    devs = [d for d in ey.Context().query_devices() if d.model in supported]
    if not devs:
        print("no supported eYs3D camera connected", file=sys.stderr)
        return 1
    out_dir = _prepare_out_dir(args.out)
    # One camera streams straight away on its signature mode; several stop on
    # the picker first, because they have to share one USB bus.
    pick = len(devs) > 1
    for d in devs:
        print(f"  {d.model}  serial {d.serial_number}  {d.usb_speed}",
              flush=True)
    # Built inside the try: a CameraView starts its camera in its
    # constructor, so one that raises half way along leaves the cameras
    # before it open with nobody to close them.
    views = []
    try:
        for d in devs:
            views.append(CameraView(d, out_dir, pick))
        focused = views[0]
        while views:
            key = norm_key(cv2.waitKeyEx(1))
            for v in views:
                v.solo = len(views) == 1
                if v.wants_focus:
                    v.wants_focus = False
                    focused = v
            for v in list(views):
                if not v.step(key if v is focused else -1, v is focused):
                    v.close()
                    views.remove(v)
                    if v is focused:
                        focused = views[0] if views else None
    finally:
        for v in views:
            v.close()
        cv2.destroyAllWindows()
    return 0


def _cloud_from(session, depth, color, depth_rgb):
    """Vertices + colors for the current depth frame, honoring the color
    source available in this mode."""
    if session.pc is None:
        return None, None
    if color is not None:
        return session.pc.calculate(depth, color)
    verts, _ = session.pc.calculate(depth)
    if verts is None or not len(verts):
        return verts, None
    if depth_rgb is not None:
        dmm = depth.get_data()
        colors = depth_rgb.reshape(-1, 3)[dmm.flatten() != 0]
        # Use the depth-colormap colors only when their validity mask lines up
        # with the reprojected vertex count; a mode/clip mismatch would
        # misalign per-vertex color or over-read the shorter buffer on the GPU.
        if len(colors) == len(verts):
            return verts, colors
    return verts, np.full((len(verts), 3), 128, np.uint8)


def _start_recording(out_dir, pipeline):
    clip_dir = os.path.join(out_dir, "clips", file_stamp())
    os.makedirs(os.path.join(clip_dir, "color"), exist_ok=True)
    os.makedirs(os.path.join(clip_dir, "depth"), exist_ok=True)
    meta = open(os.path.join(clip_dir, "metadata.jsonl"), "w",
                encoding="ascii")
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
    print(f"recording to {clip_dir}/  (press r to stop)")
    return {"dir": clip_dir, "meta": meta, "index": 0}


_FILTER_NAMES = dict(FILTER_DEFS)


def _cell_enabled(cell, vals):
    """False for a control the current state disables: exposure is auto
    while AE is on, white balance is auto while AWB is on. A control the
    model does not have reads back None (white balance on the monochrome
    G62 / R77), which disables it too."""
    kind, ref = cell
    if kind != "ctl":
        return True
    if vals[ref] == "None":
        return False
    if ref == _EXP:
        return vals[_AE] != "True"
    if ref == _WB:
        return vals[_AWB] != "True"
    return True


_DEPTH_NAMES = {"near": "Depth Near", "far": "Depth Far"}


def _cell_text(cell, session, stream, pending, enabled):
    """The label:value string for one menu cell; '*' marks a staged change
    waiting on Enter."""
    kind, ref = cell
    if kind == "ctl":
        label = session.rows[ref][0]
        if ref in (_AE, _AWB):
            val = "ON" if session.vals[ref] == "True" else "OFF"
        elif not enabled:
            val = "auto"
        else:
            val = session.vals[ref]
        return f"{label}: {val}"
    if kind == "dep":
        star = "*" if pending[ref] != stream[ref] else ""
        return f"{_DEPTH_NAMES[ref]}: {pending[ref]} mm{star}"
    on = pending["filters"][ref]
    star = "*" if on != stream["filters"][ref] else ""
    return f"{_FILTER_NAMES[ref]}: {'ON' if on else 'OFF'}{star}"


def _row_cols(r, vals):
    """Navigable column indices of layout row r (skips disabled cells)."""
    return [c for c, cell in enumerate(_MENU_LAYOUT[r])
            if _cell_enabled(cell, vals)]


def _move_sel(sel, dr, dc, vals):
    """Move the (row, col) cursor over the enabled cells: left/right within a
    row, up/down to the nearest column of the next row with an enabled cell."""
    r, c = sel
    if dc:
        cand = [cc for cc in _row_cols(r, vals) if (cc - c) * dc > 0]
        return (r, min(cand, key=lambda cc: abs(cc - c))) if cand else sel
    for _ in range(len(_MENU_LAYOUT)):
        r = (r + dr) % len(_MENU_LAYOUT)
        cols = _row_cols(r, vals)
        if cols:
            return (r, min(cols, key=lambda cc: abs(cc - c)))
    return sel


def _clamp_sel(sel, vals):
    """Keep the cursor on an enabled cell after the grid changes."""
    r, c = sel
    cols = _row_cols(r, vals)
    if cols:
        return (r, c if c in cols else min(cols, key=lambda cc: abs(cc - c)))
    return _move_sel(sel, 1, 0, vals)


def _toggle_control(session, ref):
    """Flip an AE / AWB toggle on the device (their adjust fn ignores its
    argument), then re-read the values."""
    try:
        session.rows[ref][2](0)
        session.read_vals()
    except (ValueError, RuntimeError) as e:
        print(e)


def _mode_detail_lines(m):
    """The picked mode rendered as its catalog (YAML) entry, read-only."""
    lr = "true" if m.color.split_lr else "false"
    intl = "true" if m.interleave else "false"
    return [
        f'  name: "{m.name}"',
        f"  usb: {m.usb}",
        f"  color: {{w: {m.color.width}, h: {m.color.height}, "
        f"fmt: {m.color.fmt}, split_lr: {lr}}}",
        f"  depth: {{w: {m.depth.width}, h: {m.depth.height}, "
        f"dtype: {m.depth.dtype}}}",
        f"  zd_index: {m.zd_index}",
        f"  fps: {m.fps}",
        f"  interleave: {intl}",
    ]


def _status_item(state):
    """The control window's top status bar as a menu item. Only the exact
    text "Streaming" renders idle; everything else, including "Streaming"
    with the click hint appended, renders busy. The Opening state is drawn
    by _render_status while the stream is (re)opening."""
    return ("status", (state, "normal" if state == "Streaming" else "busy"))


def _build_menu(session, sel, usb_speed, pick_mode, pick_pending,
                stream, pending, pipeline, rec):
    """Return a list of render items: ("cells", [(text, selected, enabled),
    ...]) for a packed row, or ("info", text) for a read-only line. `sel`
    is the (row, col) cursor over _MENU_LAYOUT."""
    items = []
    header = (f"Video Mode [{usb_speed}]: #{pick_mode.mode_id}"
              + ("  *" if pick_pending else ""))
    items.append(("cells", [(header, sel == (0, 0), True)]))
    items += [("info", ln) for ln in _mode_detail_lines(pick_mode)]
    # Controls, depth clip and filters, packed two/three per row.
    for r in range(1, len(_MENU_LAYOUT)):
        cells = []
        for c, cell in enumerate(_MENU_LAYOUT[r]):
            en = _cell_enabled(cell, session.vals)
            cells.append((_cell_text(cell, session, stream, pending,
                                     en), sel == (r, c), en))
        items.append(("cells", cells))
    items.append(("info", f"Link: {'up' if pipeline.is_connected else 'down'}"
                          f"  (reconnects {pipeline.reconnect_count})"))
    if rec is not None:
        items.append(("info", f"REC {rec['index']} frames"))
    # First hint line: navigation / editing controls (+ quit). The [Enter]
    # box is highlighted while a mode / filter change is staged.
    staged = pick_pending or pending != stream
    items.append(("hint", ("Move   [-/+] Adjust / Toggle   ",
                           "[Enter] Apply", "   [q] Quit", staged)))
    items.append(("info", "[d] Default properties   [p] Point cloud   "
                          "[s] Snapshot   [r] Record   [x] Reset"))
    return items


_ARROWS_W = 69          # span the drawn up/down/left/right glyphs occupy


def _draw_arrows(img, x0, ytop, line_h, color=(180, 180, 180)):
    """Draw up / down / left / right arrow glyphs (Hershey fonts have no
    arrow characters). Returns the x just past them."""
    cy = ytop + line_h // 2

    def tri(pts):
        cv2.fillConvexPoly(img, np.array(pts, np.int32), color, cv2.LINE_AA)

    cx, step = x0 + 7, 18
    tri([(cx, cy - 6), (cx - 5, cy + 3), (cx + 5, cy + 3)])   # up
    cx += step
    tri([(cx, cy + 6), (cx - 5, cy - 3), (cx + 5, cy - 3)])   # down
    cx += step
    tri([(cx - 6, cy), (cx + 3, cy - 5), (cx + 3, cy + 5)])   # left
    cx += step
    tri([(cx + 6, cy), (cx - 3, cy - 5), (cx - 3, cy + 5)])   # right
    return cx + 8


def _render_controls(win, items, ui=None):
    """Draw the menu in its own window from a list of render items: packed
    selectable cell rows and read-only info lines. Multi-column rows share
    fixed column positions so the columns line up; the selected cell is
    highlighted and the canvas is sized so nothing is clipped."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
    line_h, pad, gap = 30, 14, 64

    def tw(s):
        return cv2.getTextSize(s, font, scale, thick)[0][0]

    # Column grid from the multi-cell rows (controls / filters).
    multi = [pl for k, pl in items if k == "cells" and len(pl) >= 2]
    ncol = max((len(pl) for pl in multi), default=1)
    colw = [0] * ncol
    for pl in multi:
        for j, (t, _, _) in enumerate(pl):
            colw[j] = max(colw[j], tw(t))
    colx = [pad]
    for j in range(1, ncol):
        colx.append(colx[j - 1] + colw[j - 1] + gap)

    widths = [240]
    for kind, payload in items:
        if kind == "cells":
            if len(payload) >= 2:
                widths.append(colx[len(payload) - 1] + colw[len(payload) - 1]
                              + pad)
            else:
                widths.append(2 * pad + tw(payload[0][0]))
        elif kind == "hint":
            pre, enter_tok, post, _hl = payload
            widths.append(pad + tw("[") + _ARROWS_W + tw("]") + 8
                          + tw(pre) + tw(enter_tok) + tw(post) + pad)
        elif kind == "status":
            widths.append(2 * pad + tw("Status: " + payload[0]))
        else:  # info
            widths.append(2 * pad + tw(payload))
    width = max(widths)
    img = np.full((pad * 2 + line_h * len(items), width, 3), 32, np.uint8)

    y = pad
    for kind, payload in items:
        if kind == "status":
            text, style = payload
            bar = (35, 140, 220) if style == "busy" else (55, 55, 55)
            fg = (25, 25, 25) if style == "busy" else (220, 220, 220)
            cv2.rectangle(img, (0, y), (width, y + line_h), bar, -1)
            cv2.putText(img, f"Status: {text}", (pad, y + line_h - 9), font,
                        scale, fg, thick, cv2.LINE_AA)
        elif kind == "info":
            cv2.putText(img, payload, (pad, y + line_h - 9), font, scale,
                        (160, 160, 160), thick, cv2.LINE_AA)
        elif kind == "hint":
            grey, base, x = (160, 160, 160), y + line_h - 9, pad
            pre, enter_tok, post, hl = payload
            cv2.putText(img, "[", (x, base), font, scale, grey, thick,
                        cv2.LINE_AA)
            x = _draw_arrows(img, x + tw("["), y, line_h)
            cv2.putText(img, "]", (x, base), font, scale, grey, thick,
                        cv2.LINE_AA)
            x += tw("]") + 8
            cv2.putText(img, pre, (x, base), font, scale, grey, thick,
                        cv2.LINE_AA)
            x += tw(pre)
            ew = tw(enter_tok)
            if hl:                                 # highlight the Enter box
                cv2.rectangle(img, (x - 4, y + 2), (x + ew + 4, y + line_h - 2),
                              (35, 140, 220), -1)
                cv2.putText(img, enter_tok, (x, base), font, scale, (25, 25, 25),
                            thick, cv2.LINE_AA)
            else:
                cv2.putText(img, enter_tok, (x, base), font, scale, grey, thick,
                            cv2.LINE_AA)
            cv2.putText(img, post, (x + ew, base), font, scale, grey, thick,
                        cv2.LINE_AA)
        else:
            for j, (text, selected, enabled) in enumerate(payload):
                x = colx[j] if len(payload) >= 2 else pad
                w = tw(text)
                if selected:
                    cv2.rectangle(img, (x - 5, y + 1),
                                  (x + w + 5, y + line_h - 3), (70, 55, 30), -1)
                    fg = (90, 220, 255)
                elif not enabled:
                    fg = (110, 110, 110)          # disabled (auto) control
                else:
                    fg = (232, 232, 232)
                cv2.putText(img, text, (x, y + line_h - 9), font, scale, fg,
                            thick, cv2.LINE_AA)
        y += line_h
    if ui is not None:
        ui["img"] = img
    cv2.imshow(win, img)


def _render_status(win, ui, state, dim=False):
    """Overlay a status bar on the last control-window image and show it,
    keeping the rest of the UI in place — used for the Opening state, which
    covers both the first open and a mode / filter reopen (the user only
    needs to know the stream is coming up). `dim` greys the frozen menu while
    a reopen makes it non-interactive; the first open does not dim it."""
    line_h, pad, font = 30, 14, cv2.FONT_HERSHEY_SIMPLEX
    base = ui.get("img") if ui else None
    if base is not None:
        img = (base * 0.35).astype(np.uint8) if dim else base.copy()
    else:
        img = np.full((pad * 2 + line_h, 560, 3), 32, np.uint8)
    w = img.shape[1]
    cv2.rectangle(img, (0, pad), (w, pad + line_h), (35, 140, 220), -1)
    cv2.putText(img, f"Status: {state}", (pad, pad + line_h - 9), font, 0.55,
                (25, 25, 25), 1, cv2.LINE_AA)
    cv2.imshow(win, img)


def main() -> int:
    # example_helpers switched the console to UTF-8 on import; the arrow
    # glyphs in this module's docstring need it before argparse prints
    # --help.
    p = argparse.ArgumentParser(
        prog="viewer.py", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="capture", metavar="DIR",
                   help="Output directory for snapshots and clips "
                        "(default: ./capture, resolved and printed at "
                        "startup).")
    args = p.parse_args()

    try:
        return _run(args)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
