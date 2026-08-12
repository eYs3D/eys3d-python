# Python Driver for eYs3D Stereo Depth Cameras

[![Python](https://img.shields.io/badge/Python-3.8%20%E2%80%93%203.13-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**Language:** [English](README.md) · [日本語](docs/README.ja.md) · [繁體中文](docs/README.zh-TW.md) · [简体中文](docs/README.zh-CN.md)

`pyeys3d` is the official Python driver for eYs3D stereo depth cameras.
It delivers color, depth, and point clouds through a pipeline-based API
(`Pipeline` / `Config` / `FrameSet`) that calls the eSPDI C API directly.
Supports CPython 3.8–3.13 on Linux (x86_64, aarch64) and Windows (x64).

### Supported Devices

| Module | Product code | USB | Status |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | Production |
| **R77** | YX8072 | USB 2.0 | Production |
| **G62** | YX8081 | USB 2.0 | Production |

---

## Features

- YUYV and **MJPEG** color, decoded to `rgb8` (the monochrome modules
  G62 / R77 deliver grayscale, R = G = B)
- Wide **L\|R color split** — both eyes delivered as separate frames in
  the side-by-side stereo modes (`split_lr` in the model's catalog)
- **Depth post-processing filters** — spatial / temporal / hole filling
- **Point-cloud reprojection** — XYZ and XYZRGB, optical convention
- **Device intrinsics** from the on-camera rectify log (K / D / R / P)
- **Hot-plug recovery** — a watchdog reopens the device after a USB drop
- **Camera controls** at start and at runtime — IR intensity, exposure,
  white balance, power-line frequency
- **Per-frame metadata** — serial number, hardware timestamp, host-clock
  capture time, and wire-drop counters
- **Device binding** by serial number or USB topology for multi-camera setups
- **`examples/viewer.py`** — every capability above on one screen,
  changed while the camera runs

## Install

Pre-built wheels are attached to each [GitHub
Release](https://github.com/eYs3D/eys3d-python/releases). Download the wheel
matching your Python version and platform (Linux x86_64 / aarch64 or
Windows x64, CPython 3.8–3.13) and install it — the wheel bundles the
eSPDI runtime. On Linux the camera enumerates as a UVC video device: a
standard desktop grants the logged-in user access automatically, but a
headless or minimal setup may need the user added to the `video` group
(`sudo usermod -aG video $USER`, then re-login) to read `/dev/video*`.
Windows needs two system components most machines already
have:

- the [Visual C++ 2015–2022 Redistributable
  (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) — without it
  `import pyeys3d` fails with *"DLL load failed: The specified module
  could not be found"*
- the system OpenCL runtime, installed by every GPU driver

```bash
pip install pyeys3d-1.0.0-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-1.0.0-cp310-cp310-win_amd64.whl      # Windows
```

From source — needs a C++17 compiler; the build fetches CMake and
Ninja automatically when they are not already installed:

- Linux: GCC (`apt install build-essential`)
- Windows: [Visual Studio 2022 Build
  Tools](https://visualstudio.microsoft.com/visual-studio-build-tools/)
  with the **Desktop development with C++** workload


```bash
git clone https://github.com/eYs3D/eys3d-python.git
cd eys3d-python
pip install .
```

## Quick start

With no arguments, the connected camera is auto-detected and opened in its
signature mode — the model's default mode, defined in its catalog:

```python
import pyeys3d as ey
import cv2

with ey.Pipeline() as pipeline:
    pipeline.start(ey.Config())            # auto-detect + signature mode
    dev = pipeline.device_info
    intr = pipeline.intrinsics             # None if the unit is uncalibrated
    print(f"Opened {dev.model}  serial {dev.serial_number}"
          f"  firmware {dev.firmware_version}")
    if intr is not None:
        print(f"  fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} "
              f"cy={intr.cy:.1f}  baseline {intr.baseline_mm:.2f} mm")
    colorizer = ey.Colorizer(pipeline)     # depth -> rgb8 colormap
    # Reprojection needs the calibration an uncalibrated unit does not carry.
    pc = ey.PointCloud(pipeline) if intr is not None else None
    while True:
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break
        if frames is None:
            continue
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            continue
        dmm = depth.get_data()             # (H, W) uint16, 1 unit = 1 mm
        verts = pc.calculate(depth)[0] if pc else ()   # (N, 3) float32 meters

        center_mm = int(dmm[dmm.shape[0] // 2, dmm.shape[1] // 2])
        center = f"{center_mm:5d} mm" if center_mm else " no data"
        nearest = f"{'no points':>23}"
        if len(verts):
            x, y, z = (int(v * 1000) for v in verts[verts[:, 2].argmin()])
            nearest = f"X{x:+5d} Y{y:+5d} Z{z:5d} mm"
        print(f"\rcenter {center}   nearest {nearest}", end="", flush=True)

        cv2.imshow("color", color.get_data_bgr())
        cv2.imshow("depth", colorizer.colorize_bgr(dmm))
```

## Examples

`examples/` holds a runnable file per subject. `quickstart.py` and
`viewer.py` are where to start; the rest each add one subject to `01`'s
color + depth base:

| File | Teaches |
|---|---|
| `quickstart.py` | The program above, ready to copy |
| `viewer.py` | Every capability, driven from an on-screen menu |
| `hello_depth.py` | The camera works — no window, no extra packages |
| `00_enumerate.py` | What is connected, and every video mode it has |
| `01_basic_color_depth.py` | Color + depth, and the settings applied at `start()` |
| `02_pointcloud.py` / `03_pointcloud_open3d.py` | Depth as a 3D point cloud, in pyglet and in Open3D |
| `04_capture.py` | Snapshots, clip recording and replay |
| `05_runtime_controls.py` | Which settings change while streaming |
| `06_multicam.py` | Several cameras, a process each |

**`viewer.py` is where to start with a camera in hand.** It puts every
setting on one screen and changes them while the camera runs — video mode,
depth clip, filters, IR, exposure, white balance — with snapshots, clip
recording and a 3D cloud, across every connected camera at once. Nothing
has to be passed to reach any of it:

```bash
pip install opencv-python "pyglet>=2"
python examples/viewer.py
```

Per-example flags, key tables and setup notes are in
**[examples/README.md](examples/README.md)**
([browse online](https://github.com/eYs3D/eys3d-python/tree/main/examples)).

The wheel installs the library only — download
`pyeys3d-<version>-examples.zip` from the same Release page, or use this
repository.

## API surface

### `Context`

Enumerate connected cameras. Lightweight — construct one whenever a fresh scan is needed.

```python
ctx = ey.Context()
for dev in ctx.query_devices():
    print(dev)
# DeviceInfo(model='G100P', serial_number='8036259M200025', usb_port='2-2:1.0',
#            pid=385, usb_port_type=3, usb_speed='USB3.0',
#            firmware_version='YX80362-B01-...')
```

### `Config`

Declarative — picks a video mode from the per-model catalog under
`pyeys3d/modes/<MODEL>.yaml`. All arguments to `enable_device` are
optional: omit the model to auto-detect the connected camera, and omit
`mode_id` to use that model's signature mode (per-model list under
Supported video modes below).

```python
ey.Config()                                          # auto-detect + signature mode
ey.Config().enable_device("G100P")                   # this model, signature mode
config = (ey.Config()
          .enable_device("G100P", mode_id=1)         # explicit model + mode
          .set_ir_value(3)                           # model-specific; -1 = default
          .set_auto_exposure(True))
```

Other optional camera controls — each applied at `start()` only when set,
otherwise left untouched:

| Method | Argument |
|---|---|
| `set_auto_exposure(enabled)` | `True` / `False`; if off, set the value with `set_exposure` |
| `set_exposure(value)` | manual exposure, register units (turns auto-exposure off) |
| `set_auto_white_balance(enabled)` | `True` / `False`; if off, set the value with `set_white_balance` |
| `set_white_balance(value)` | manual white balance, register units (turns auto WB off); **color models only** |
| `set_power_line_frequency(mode)` | anti-flicker (sync exposure to the mains so lighting flicker doesn't band the image): `1` 50 Hz / `2` 60 Hz |
| `set_ir_value(level)` | IR projector intensity (model-specific range; 0 = off), or `-1` for the mode-aware default described below |
| `set_depth_range(near_mm, far_mm)` | drop depth outside `[near_mm, far_mm]`; unset = per-model default, `far_mm` at most 16383 (the 14-bit depth limit) |
| `set_depth_quality_registers(source)` | firmware depth-tuning profile: `True` (default) = the bundled per-model profile, `False` = leave the firmware defaults, or a path to a custom profile file |

Controls are held to what the camera supports, so an unsupported request
raises `ValueError` on the host and never reaches the device. Values with a
fixed range are refused by the setter itself; the ones that depend on which
camera and mode were chosen — the monochrome models (G62 / R77) have no
white balance, and a video mode opens only on the USB link it declares (see
Supported video modes) — are refused by `start()`, when the config resolves
against the opened camera.

A few seconds after depth streaming starts (once the stream has
settled), the pipeline applies the model's depth-quality register
profile to the firmware in the background, and re-applies it after a
USB reconnect (the firmware resets on re-enumeration). The bundled
profiles ship at
`pyeys3d/quality/DM_Quality_Cfg/<PART>_DM_Quality_Register_Setting.cfg`
(the firmware part number — G100+ `YX80362`, R77 `YX8072`, G62 `YX8081`,
plus a `DEFAULT`) — one `address,mask,value` hex triple per line; pass a
file path to substitute your own tuning.

The IR projector is the one control that is always written at `start()`.
An explicit `set_ir_value` wins (0 = off in any mode); left unset,
the default follows what the mode needs — the model's cataloged default
when the mode has depth (stereo matching needs the projector) or the
module is monochrome (the G62 / R77 sensors see IR, so it is their scene
illumination and color modes stay black without it), and off for a
color-only mode on a color sensor, keeping the dot pattern out of the
color image.

### `Pipeline`

Owns one open device. `start(config)` → `wait_for_frames(timeout_ms)`
returns a `FrameSet` (or `None` on timeout) → `stop()`. `poll_for_frames()`
is the non-blocking variant: it returns the latest set if a newer one is
ready, else `None`.

After `start()`, `pipeline.device_info` names the opened device (model, serial,
usb_port) and `pipeline.color_profile` / `depth_profile` give each stream's
`StreamProfile(width, height, fps)` (or `None`) before the first frame.

On a USB drop a watchdog reopens the device; meanwhile `wait_for_frames`
returns `None` and `pipeline.is_connected` is `False` (`reconnect_count` counts
the reopens). `pipeline.frames_dropped` reports frames the camera produced
but the host never received, per stream — a rising count points at USB
bandwidth or scheduling trouble.

Recovery covers drops that happen once streaming is up. A drop during
`start()` itself lands inside the SDK's open call, which the driver cannot
interrupt, so `start()` may block until the camera is reattached. Have the
camera attached before calling `start()`.

```python
pipeline = ey.Pipeline()
pipeline.start(config)
try:
    frames = pipeline.wait_for_frames(timeout_ms=1000)
    color = frames.get_color_frame()
    depth = frames.get_depth_frame()
finally:
    pipeline.stop()
```

Or as a context manager:

```python
with ey.Pipeline() as pipeline:
    pipeline.start(config)
    ...
```

### Camera controls at runtime

Every control above except the depth clip and the quality-register profile
also has a runtime counterpart on `Pipeline` with the same name — call it while streaming and the camera
changes immediately, no restart:

```python
pipeline.set_ir_value(4)            # validated against the model's range
pipeline.set_auto_exposure(False)
pipeline.set_exposure(-6)           # switches auto-exposure off first
```

`pipeline.get_*()` reads the current value back from the device (e.g.
`get_exposure()`, `get_ir_value()`), returning `None` when the camera
lacks the control. Values set at runtime survive a USB drop — the hot-plug
watchdog re-applies the latest state on reconnect.

Exposure and white balance are in device register units — exposure may be
negative (the modules use a signed log2 scale, e.g. `-13` ≈ 1/8192 s).
Each query returns a `ControlRange(min, max, step, default)` the runtime
setters validate against: `get_exposure_range()` reports the fixed register
range the modules share, `get_white_balance_range()` what the device
reports, and `get_ir_range()` the model catalog — the IR register accepts
values beyond the model's qualified operating range, so the catalog is
authoritative there.

`set_temporal_filter` requires the temporal filter to have been enabled via
`Config.with_filters(...)` at `start()`; the disparity stream it runs on is
fixed at open time, so it can be retuned but not switched on after the fact.

### Multi-camera

Each `Pipeline` owns one camera. Several can run in one process, and
`06_multicam.py` still gives each camera a process of its own — a pattern
worth copying, since a camera that stops responding then takes nothing else
down with it. On Windows there is a reason to prefer one process: the system slows
processes whose windows are not in front, so a viewer per camera leaves
every camera but the focused one running at a reduced rate.

Within one process the cameras are set up one at a time — `start()`,
`Context.query_devices()` and the driver's own reconnect take one turn
between them, because the SDK keeps its device bookkeeping per process.
Opening from several threads is safe but not faster: the opens queue.
Separate processes do not share that state and open in parallel.

A camera itself opens in one process at a time. A second process asking
for a camera that is already open fails at `start()` with a `RuntimeError`
naming the eSPDI code, and the process holding it keeps streaming
undisturbed, so leaving `viewer.py` running makes the next example fail at
`start()`; closing it releases the camera.

When several cameras are connected, selection must be unambiguous: pin the
unit by serial number (substring match) or USB port (exact match):

```python
config.enable_device("G100P", mode_id=1, usb_port="2-2:1.0")   # Windows: the
# port identity is the device path's instance segment, stable per physical
# port, e.g. usb_port="6&35c4e9&0&0000" — copy it from 00_enumerate.py
# or
config.enable_device("G100P", mode_id=1, serial_number="8036259M200025")
```

An ambiguous start raises an error listing each candidate's model, serial
number, and USB port; `examples/00_enumerate.py` prints the same
identifiers. If both `serial_number` and `usb_port` are given, a camera
must match **both** — pinning "this serial on this port"; if no connected
camera matches both, selection fails with the same listing. To follow one
unit across ports, pin the serial alone.

### `Frame`

| Property | Description |
|---|---|
| `domain` | `FrameDomain.COLOR_RGB8` (color) or `FrameDomain.DEPTH_MM` (depth) |
| `width`, `height` | Frame dimensions in pixels |
| `frame_number` | SDK-reported per-stream sequence number |
| `hw_timestamp_us` | Hardware timestamp in microseconds (USB DMA complete) |
| `timestamp` | Capture time on the host clock (epoch seconds, comparable with `time.time()`) |
| `get_data()` | Color: `(H, W, 3)` uint8 rgb8. Depth: `(H, W)` uint16 millimeters. |

Arrays returned by `get_data()` are zero-copy, read-only views into the
frame; to modify pixels, copy first (`img = frame.get_data().copy()`).

In wide L\|R split modes the right-eye color is available from the same
frame set: `frames.get_right_color_frame()`.

### Intrinsics

`pipeline.intrinsics` gives the camera model the active video mode maps
to, as the device stores it:

```python
intr = pipeline.intrinsics                            # None if uncalibrated
print(intr.width, intr.height, intr.baseline_mm)      # e.g. 1280 720 59.93
print(intr.fx, intr.fy, intr.cx, intr.cy)             # rectified pinhole
print(intr.K, intr.D, intr.R, intr.P)                 # full model
```

Frames are delivered **already rectified**, so `fx`/`fy`/`cx`/`cy` (the
same values as `P`) describe what you receive; `K` and `D` describe the
raw sensor before rectification and must not be re-applied to a delivered
frame. See `docs/api.md` for the full field table.

### Filters and point cloud

Declare the depth post-process chain with `with_filters`. The pipeline runs it
natively and delivers depth already filtered and in millimeters; with no
filters the firmware millimeter fast path is used.

```python
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .with_filters(
              ey.SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0),
              ey.TemporalFilter(alpha=0.4, delta=20, persistence=3),
              ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND)))
pipeline.start(config)

frames = pipeline.wait_for_frames()
depth  = frames.get_depth_frame().get_data()   # (H, W) uint16 mm, filtered

pc = ey.PointCloud(pipeline)
verts, colors = pc.calculate(frames.get_depth_frame(), frames.get_color_frame())
# verts:  (N, 3) float32 meters, optical frame (X right, Y down, Z forward)
# colors: (N, 3) uint8, or None when no color frame is supplied
```

The color frame is optional — pass it for XYZRGB, or omit it
(`pc.calculate(depth)`) for a lighter XYZ-only cloud.

Depth is computed from the rectified left eye, so it already shares that
eye's viewpoint: there is no second sensor to reproject from and no
alignment step before texturing, measuring, or overlaying the two. Where a
mode delivers depth at a smaller raster than color (the scale-down modes,
including the G100+ and R77 USB 2 signature modes), scale the pixel index
by the height ratio — the two frames stay the same view, at two sizes.

The chain order is fixed (spatial → temporal → hole filling) regardless of
argument order. Defaults are shown in parentheses:

| Filter | Parameters |
|---|---|
| `SpatialFilter` | `alpha` smoothing 0–1, 1 = none (0.5); `delta` edge threshold in disparity units (20); `magnitude` 1–5 passes (2); `holes_fill` max run bridged, 0 = off (0) |
| `TemporalFilter` | `alpha` current-frame blend 0–1 (0.4); `delta` gap threshold in disparity units (20); `persistence` frames a value is held over dropouts 0–8 (3) |
| `HoleFillingFilter` | `mode` = `HoleFill.OFF` / `FROM_LEFT` / `FARTHEST_AROUND` (default) / `NEAREST_AROUND` |

### Depth visualization

`Colorizer` maps a depth frame to an rgb8 image (build once, `colorize` each
frame):

```python
colorizer = ey.Colorizer(pipeline)      # colormap range from the depth clip
rgb = colorizer.colorize(frames.get_depth_frame())   # (H, W, 3) uint8 rgb8
```

`ey.Colorizer(pipeline)` takes the range from the depth clip (`min_mm` /
`max_mm` override); holes (depth 0) are black. `mode='grayscale'` renders mono
instead of the default JET color map.

## Supported video modes

The signature mode — what `Config()` opens when no `mode_id` is given:

| Model | Link | Signature mode |
|---|---|---|
| G100+ | USB 3 | `1` — L'+D 1280x720@60 interleave (SDK 30fps) |
| G100+ | USB 2 | `56` — L'+D 1280x720@24 + 640x360 depth interleave (USB 2.0, SDK 12fps) |
| R77 | USB 2 | `2` — L'+D 1280x920@30 + 640x460 depth |
| G62 | USB 2 | `1` — L'+D 640x480@25 |

The names are the catalog's own: `L` / `R` are the raw eyes and `L'` / `R'`
the rectified ones, `D` is depth, and a side-by-side pair reads
`<width>(x2)x<height>`.

Each mode declares the USB link it needs, and opens only on that link —
asking for one the negotiated link cannot carry raises `ValueError`. The
G100+ signature therefore follows the link: mode `1` on USB 3, mode `56`
on USB 2, which trades the 60 fps rate and full-size depth for what the
slower link carries (24 fps, 640x360 depth).

Mode catalogs live under `pyeys3d/modes/`; the
YAML files also ship inside the wheel (`pyeys3d/modes/<MODEL>.yaml`), so
the full mode tables are readable straight from an installed package:

- `pyeys3d/modes/G100P.yaml` — 80 modes (55 USB 3, 25 USB 2)
- `pyeys3d/modes/R77.yaml`  — 9 modes (MJPEG + YUYV, incl. wide L\|R)
- `pyeys3d/modes/G62.yaml`  — 15 modes (MJPEG + YUYV, incl. wide L\|R)

List them programmatically:

```python
from pyeys3d.modes import load_catalog
for mid, mode in sorted(load_catalog("G100P").items()):
    yuyv = "YUYV" if mode.color.fmt == 0 else "MJPEG"
    print(f"  {mid}: {mode.name}  color={yuyv}")
```

## Diagnostics

`PYEYS3D_LOG_LEVEL` controls the native layer's log verbosity: `none` /
`error` / `warn` (default) / `info` / `debug`. Set `PYEYS3D_TIMING=1` to log
per-stage timing (color decode, depth convert + filter) when the pipeline
stops. `PYEYS3D_PC_THREADS` overrides how many workers `PointCloud`
reprojects with (default 4, capped at the core count) — the passes are
memory-bound, so more workers help less than the core count suggests. Native errors carry the eSPDI code name and a hint, e.g.
`APC_OpenDevice2 failed: rc=-27 APC_NOT_SUPPORT_RES (the device rejected
this mode)`. A cataloged mode asked for on the wrong USB link is caught
before that, as a `ValueError` from `start()`.

## Compatibility

Python ≥ 3.8. Linux x86_64 and aarch64 (Jetson), and Windows 10/11 x64.

## Support

Questions and bug reports: <support@eys3d.com>. To make an issue
diagnosable at a glance, include the failing command re-run with
`PYEYS3D_LOG_LEVEL=info`, the `device_info` line printed at startup
(model / serial / firmware), and your OS and Python version. Development
setup and the test / lint gates are in [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Apache-2.0. See `LICENSE`.
