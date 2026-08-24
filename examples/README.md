# pyeys3d Examples

**Language:** [English](README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

Runnable samples for the `pyeys3d` driver. Each file is self-contained,
commented, and teaches one subject: read the one you need, copy it, and
delete what your program does not use.

Two are where to start. `quickstart.py` is the smallest complete program,
to copy into your own project; `viewer.py` drives every capability from an
on-screen menu, which is the fastest way to see what a camera can do. The
rest each add one subject: `hello_depth.py` proves the camera works with no
display at all, `00_enumerate.py` lists what is connected, `01` is the
color + depth base, and `02`–`06` each add one capability to that base.

## Setup

Install the `pyeys3d` wheel matching your Python version and platform
(Linux x86_64 / aarch64 or Windows x64) from the same Release page this
bundle came from:

```bash
pip install pyeys3d-<version>-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-<version>-cp310-cp310-win_amd64.whl      # Windows
```

`hello_depth.py` and `00_enumerate.py` need nothing else. The ones that
draw take these:

```bash
pip install opencv-python           # every example with a window
pip install "pyglet>=2"             # 02_pointcloud.py, viewer.py's 3D window
pip install open3d                  # 03_pointcloud_open3d.py
```

## At a glance

| File | Teaches | Needs |
|---|---|---|
| [`quickstart.py`](#quickstartpy) | The smallest complete program | opencv |
| [`viewer.py`](#viewerpy) | Every capability, driven from a menu | opencv, pyglet |
| [`hello_depth.py`](#hello_depthpy) | The camera works | — |
| [`00_enumerate.py`](#00_enumeratepy) | What is connected, and what modes it has | — |
| [`01_basic_color_depth.py`](#01_basic_color_depthpy) | Color + depth, and the settings applied at start | opencv |
| [`02_pointcloud.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | Depth as a 3D point cloud | opencv, pyglet |
| [`03_pointcloud_open3d.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | The same cloud in Open3D | opencv, open3d |
| [`04_capture.py`](#04_capturepy) | Saving snapshots and clips, and replaying them | opencv |
| [`05_runtime_controls.py`](#05_runtime_controlspy) | Which settings change while streaming | opencv |
| [`06_multicam.py`](#06_multicampy) | Several cameras, a process each | opencv |

## Options `01`–`06` share

Every numbered example takes the same seven, one per API call it makes.
Anything not listed is a default the example sets in one visible line —
change the line, not a flag.

| Flag | Maps to | Default |
|---|---|---|
| `--model MODEL` | `Config.enable_device(model)` | auto-detect |
| `--mode MODE_ID` | `Config.enable_device(mode_id=)` | the model's signature mode |
| `--serial SERIAL` | `Config.enable_device(serial_number=)` | any (substring match) |
| `--usb-port PORT` | `Config.enable_device(usb_port=)` | any (exact match) |
| `--ir-value LEVEL` | `Config.set_ir_value()` | per-model default; `0` is off |
| `--depth-range NEAR_MM FAR_MM` | `Config.set_depth_range()` | per-model default |
| `--filters` | `Config.with_filters(Spatial, Temporal, HoleFilling)` | off |

`hello_depth.py`, `quickstart.py` and `00_enumerate.py` take none: the
first two expect a single camera, and the third opens nothing. `viewer.py`
takes only `--out`, because everything else is on screen. Each example
adds its own subject's flags on top — listed with it below, and in full
under `--help`.

`--serial` and `--usb-port` are how you pick one camera out of several.
With more than one connected and neither given, `Config` refuses to guess
rather than opening the wrong one.

## Start here

### `quickstart.py`

```bash
python quickstart.py
```

The project README's quick start, runnable as-is: device identity and
intrinsics at startup, then color + depth windows with the center
distance, the cloud's nearest point, and each frame's number and
timestamp on one status line. Copy this file into your project as the
starting point — it imports nothing from the other examples.

Expects a single camera and takes no flags.

### `viewer.py`

```bash
python viewer.py
python viewer.py --out /tmp/captures
```

Every camera setting on one screen, changed while the camera runs — the
fastest way to see what a camera does, and to check one end to end without
editing anything. Each camera gets its Color / Depth windows plus a
Controls window carrying the menu.

| Key | Action |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> | Move between menu cells |
| <kbd>-</kbd> / <kbd>+</kbd> | Adjust the selected value, or toggle it |
| <kbd>Enter</kbd> | Apply the staged video mode / depth clip / filter changes (reopens the stream) |
| <kbd>p</kbd> | Open / close the 3D point-cloud window |
| <kbd>s</kbd> | Save a snapshot set |
| <kbd>r</kbd> | Start / stop recording a clip |
| <kbd>d</kbd> | Restore the camera properties to their defaults |
| <kbd>x</kbd> | Hardware-reset the camera |
| <kbd>q</kbd> / <kbd>ESC</kbd> | Close the focused camera |

`--out DIR` is the only option, and only because the screen has nowhere to
put it; it is resolved and printed at startup (`captures -> ...`), so every
`saved` line names a file you can find.

The menu carries the video mode, IR, power-line frequency, auto/manual
exposure, auto/manual white balance, the depth clip and the three depth
filters. Video mode, depth clip and filters are marked `*` while staged
and applied together by <kbd>Enter</kbd>, because all three are fixed at
`start()` and changing them reopens the stream — the camera goes away for
a few seconds each time. Everything else applies as you press the key.
<kbd>Enter</kbd> and <kbd>x</kbd> are held off while a clip records.

With more than one camera connected, each stops on a mode picker instead
of streaming — the Controls window is up, the preview is not. Pick a mode
and press <kbd>Enter</kbd> to start that one, or <kbd>q</kbd> to close it.
Choose with the shared USB bus in mind: the cameras divide one host
controller's bandwidth, and a signature mode can take most of a USB 3 link
on its own. Click a camera's window to send it the keys.

What the viewer does not do, and the numbered examples do: per-frame
metadata (`01 --frame-meta`), the full camera model at startup (`01`), and
clip playback (`04 --play`).

## The rest, one subject each

### `hello_depth.py`

```bash
python hello_depth.py
```

No window, no packages beyond `pyeys3d`. Prints the distance at the image
center as it updates, so a failure here is the camera, the driver or the
cable — never the display code. Expects a single camera and takes no flags;
with several connected, name one in the `enable_device()` call in the file.

### `00_enumerate.py`

```bash
python 00_enumerate.py
```

Prints the full video-mode catalog of each connected model — id, the USB
link the mode needs, resolution and frame rate, and a `*` on each link's
signature mode (the one `start()` opens when no `mode_id` is given) — then
a summary line per camera. Run it before `--mode` to see what ids exist.

### `01_basic_color_depth.py`

```bash
python 01_basic_color_depth.py
python 01_basic_color_depth.py --mode 3 --filters --frame-meta
```

The base the others build on: Color (Left), Color (Right) where the mode
splits L|R, and Depth each open in their own window, hover reads RGB or the
distance under the cursor, and the full stored camera model (K / D / R / P)
prints at startup.

| Flag | Effect |
|---|---|
| `--frame-meta` | Once a second, print one frame's number, its hardware and host timestamps, and its age — the fields to align against other sensors, and to measure latency |

### `02_pointcloud.py` / `03_pointcloud_open3d.py`

```bash
python 02_pointcloud.py
python 03_pointcloud_open3d.py
```

01 plus a live 3D point cloud, rebuilt every frame from the depth image and
the stored intrinsics. The two files are the same program with a different
display layer — pyglet/OpenGL in `02`, Open3D in `03`. In the cloud window:
drag to orbit, middle-drag to pan, scroll to zoom, <kbd>R</kbd> resets the
view, <kbd>Q</kbd> / <kbd>ESC</kbd> closes it.

### `04_capture.py`

```bash
python 04_capture.py                      # window; press s to save a set
python 04_capture.py --snapshot           # save one set and exit, no window
python 04_capture.py --record 10          # record ten seconds and exit
python 04_capture.py --play capture/clips/20260101-120000-000
```

Saving snapshots and clips, and replaying them. Everything lands under
`--out` (default `./capture/`):

```
capture/snapshots/<stamp>_color.png            press s, or --snapshot
capture/snapshots/<stamp>_depth.png            raw 16-bit, 1 mm per unit
capture/snapshots/<stamp>_depth_preview.png    the viewable rendering
capture/snapshots/<stamp>_cloud.ply            MeshLab / Open3D
capture/clips/<stamp>/                         one recorded clip
    color/000000.jpg  depth/000000.png         paired frame sets
    metadata.jsonl                             index + calibration
```

| Flag | Effect |
|---|---|
| `--out DIR` | Where snapshots and clips are written (default `capture`) |
| `--snapshot` | Save one set and exit, with no window and no keypress. Auto-exposure is given time to settle first, so the frame written is the first properly exposed one — not the first to arrive |
| `--record SECONDS` | Record a clip that long, then exit |
| `--play DIR` | Replay a clip at its recorded pace |

`metadata.jsonl`'s first line carries the device, depth range and
intrinsics; each line after indexes one frame set. Depth PNGs are uint16
with one millimeter per unit, so most viewers show them almost black; the
`_depth_preview.png` beside each is the viewable rendering. Read the values
back with `cv2.imread(path, cv2.IMREAD_UNCHANGED)`.

### `05_runtime_controls.py`

```bash
python 05_runtime_controls.py
```

Which settings can change while the camera streams. Everything set through
`Config` applies once at `start()`; this shows the `Pipeline` counterparts
— IR, auto/manual exposure, auto/manual white balance (color models only)
and power-line frequency — in an arrow-key menu in its own window.

| Key | Action |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> | Select a control |
| <kbd>-</kbd> / <kbd>+</kbd> (or <kbd>←</kbd> <kbd>→</kbd>) | Adjust it; AE / AWB flip on and off |
| <kbd>d</kbd> | Restore defaults — IR to the model default, AE / AWB to auto |
| <kbd>x</kbd> | Hardware-reset the camera, re-enumerating it over USB |
| <kbd>q</kbd> / <kbd>ESC</kbd> | Quit |

Values are read back from the device after every change and survive a USB
drop: the hot-plug watchdog re-applies the latest state on reconnect. Press
<kbd>x</kbd> to see it — the Link line tracks the camera going down and
coming back.

### `06_multicam.py`

```bash
python 06_multicam.py
python 06_multicam.py --model G100P --mode 3
```

01 for every connected camera at once, one process per camera. The parent
enumerates, spawns a child per device pinned to the serial it read, and
waits; each child opens one `color | depth` window with its frame rate in
the title. `--model` / `--serial` / `--usb-port` narrow which cameras to
open, and the remaining shared flags are forwarded to every child.

## Several cameras: a process each, or one process

`06_multicam.py` and `viewer.py` show the two arrangements, and the choice
is a trade, not a preference:

- **A process per camera** (`06`) keeps them independent — one camera
  wedging takes nothing else down — and their opens run in parallel,
  because the SDK's per-process bookkeeping is not shared across
  processes. On Windows the cameras whose windows are not in front run at
  a reduced rate: the system slows background processes.
- **One process** (`viewer.py`) has no such split — whichever window is
  clicked, every camera stays foreground — but the cameras must be set up
  one at a time, so N cameras cost N opens back to back.

Cameras opened from separate threads of one process are serialised by the
driver for the same reason. Nothing is required of the caller.

## How these files are organised

`example_helpers.py` holds the scaffolding an example needs to be a
program — the shared flags, the console encoding, the device summary
printed at startup, and the OpenCV windows — and no `pyeys3d` calls at
all. Every API call a reader came for stays in the example that teaches
it, even where that repeats another file.

Where two examples do the same thing, they do it in the same characters,
so a diff between them shows only what the later one adds. Where they
genuinely differ — `02`'s cloud window drives itself from the main loop,
`viewer.py`'s runs on its own thread — they stay apart rather than being
merged into something that serves neither.

## Notes

- **Windows**: the wheel needs the Visual C++ 2015–2022 Redistributable
  (x64) and a system OpenCL runtime (installed by every GPU driver).
- Full API reference: [`docs/api.md`](../docs/api.md) — in the repo, and
  bundled inside this examples archive.
