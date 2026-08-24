# pyeys3d API Reference

Classes are listed in the order you use them: enumerate → configure →
stream → read frames → visualize → compute.

---

## Conventions

These hold everywhere in the API; the per-call docs do not repeat them.

| Subject | Rule |
|---|---|
| Depth values | `uint16`, **1 unit = 1 millimeter**. `0` means no data — never a valid distance |
| 3-D points | `float32` **meters** |
| Coordinate frame | Optical: **X right, Y down, Z forward**, origin at the left lens |
| Color pixels | `uint8` **RGB**; `get_data_bgr()` returns a BGR copy for OpenCV |
| Image origin | Top-left, `(u, v)` = (column, row) — the NumPy `[v, u]` order |
| Rectification | Color and depth are delivered **already rectified** |
| Timestamps | `timestamp` is seconds on the host clock, same epoch as `time.time()` |
| Distances in configs | Millimeters (`set_depth_range`, `depth_near_mm`, `baseline_mm`) |

## Lifecycle

Nearly everything the pipeline reports comes from the opened device, so it
only exists between `start()` and `stop()`.

| State | What is available |
|---|---|
| Before `start()` | `Context.query_devices()` only. Every `Pipeline` property is `None`, `0`, or `False` |
| After `start()` | Device identity, stream profiles, intrinsics, and the camera controls. No frame has necessarily arrived yet |
| First frame | Lands some seconds after `start()`. The delay is a firmware property and varies per module and per video mode |
| After a USB drop | `is_connected` goes `False` and `wait_for_frames()` returns `None` while the watchdog reopens the device. Settings are re-applied on recovery; `reconnect_count` increments. Covers drops after streaming is up — see [`start()`](#startconfig-config--none) for a drop during the open itself |
| After `stop()` | Back to the "before `start()`" state. Frames already handed out stay valid — they own their pixel buffers |

`Pipeline` is a context manager; `with` guarantees `stop()` even on an
exception.

## Errors

| Exception | Raised by | Cause |
|---|---|---|
| `RuntimeError` | `Pipeline.start()` | No camera matched, the selection was ambiguous, or the device could not be opened |
| `RuntimeError` | `PointCloud(pipeline)` | The pipeline is not started, or the unit carries no calibration |
| `RuntimeError` | `Colorizer(pipeline)` | The pipeline is not started |
| `RuntimeError` | `Pipeline.set_*` | The device rejected the write |
| `ValueError` | `Pipeline.set_*` / `Config.set_*` | The value is outside the accepted range, or the control does not apply to this camera. Exposure and power-line frequency have a fixed range that both sides check; the IR and white-balance limits come from the model or the device, so `Config` accepts what only `start()` can refuse |
| `ValueError` | `Config.enable_device()` | Unknown model string or mode id |
| `ValueError` | `SpatialFilter` / `TemporalFilter` / `HoleFillingFilter` | A parameter outside the range in the Filters section |
| `ValueError` | `Pipeline.start()` | The config cannot resolve against the opened camera: a mode asked for on the wrong USB link, white balance on a monochrome model, an IR level or depth range outside the model's limits |

Not every camera implements every control, and a device may refuse a
value that is nominally in range — the power-line mode is the common
case. Both surface as `RuntimeError` from the setter, so a menu that
offers all values should catch it and move on rather than assume the
write succeeded.

## Threading

| Operation | Rule |
|---|---|
| `start()` / `stop()` / `wait_for_frames()` / `poll_for_frames()` | One thread only — the thread that owns the pipeline |
| Camera controls (`set_*` / `get_*`) | Safe from any thread while streaming, e.g. a UI thread adjusting exposure |
| Properties | Safe to read from any thread, including while another stops the pipeline: a reader racing `stop()` returns the value it would after `stop()`, never an error |
| `Frame` objects | Own their buffers; safe to hand to another thread and outlive the next `wait_for_frames()` |
| Multiple cameras | One `Pipeline` each; several may run in one process, or one process each to isolate them. A given camera opens in one process at a time |

## Logging

The native layer writes to stderr. `PYEYS3D_LOG_LEVEL` selects the
verbosity:

| Value | Shows |
|---|---|
| `none` | Nothing |
| `error` | Failures only |
| `warn` | **Default.** Failures plus conditions that degrade output — a missing calibration, a stream that never starts, an unmet firmware ordering constraint |
| `info` | Adds open/close and per-stream setup detail |
| `debug` | Adds per-frame detail |

Leave it at `warn`: that is the level at which the driver reports a
camera that is streaming but not usable. Ask for `debug` output when
reporting a problem.

Two more variables affect the native layer:

| Variable | Effect |
|---|---|
| `PYEYS3D_TIMING=1` | Logs per-stage timing (color decode, depth convert + filter) when the pipeline stops |
| `PYEYS3D_PC_THREADS=N` | Workers `PointCloud` reprojects with, capped at the core count (default 4). Both passes are memory-bound, so more workers help less than the core count suggests |

---

## Context

Enumerate connected cameras without opening a stream.

```python
ctx = ey.Context()
devices = ctx.query_devices()   # list[DeviceInfo]
```

### `Context()`

No parameters.

### `query_devices() → list[DeviceInfo]`

Returns every eYs3D device on the USB bus; one whose PID this release does
not carry a catalog for reports `model="unknown"` and cannot be opened. The
list is point-in-time — call again after a hot-plug.

### `DeviceInfo` fields

| Field | Type | Description |
|---|---|---|
| `model` | `str` | Model string, e.g. `"G100P"` |
| `serial_number` | `str` | Unit serial number |
| `firmware_version` | `str` | Firmware build identifier |
| `usb_port` | `str` | USB topology path, e.g. `"2-1.3"` |
| `usb_port_type` | `int` | Negotiated USB port type: `2` = USB 2.0, `3` = USB 3.0 (`0` = unknown) |
| `usb_speed` | `str` | Human-readable form of the above: `"USB2.0"`, `"USB3.0"`, or `"unknown"` |
| `pid` | `int` | USB product ID |

A camera's video-mode catalog is split by USB port type: a USB 2.0 link
cannot open USB 3.0-only modes. With no `mode_id`, `start()` picks the
model's signature mode for the negotiated link. `00_enumerate.py` lists the
catalog; each mode's required link is shown in the listing.

---

## Config

Declare what to open and how, before calling `Pipeline.start()`. All
setters return `self` for chaining.

```python
cfg = (ey.Config()
       .enable_device("G100P", mode_id=1)
       .set_ir_value(3)
       .set_depth_range(300, 1500)
       .with_filters(ey.SpatialFilter(), ey.TemporalFilter()))
```

### `Config()`

No parameters.

### `enable_device(model=None, mode_id=None, *, serial_number="", usb_port="") → Config`

Select which camera to open and which video mode to use.

| Parameter | Type | Default | Behavior when omitted |
|---|---|---|---|
| `model` | `str \| None` | `None` | Auto-detect the single connected camera; raises if zero or more than one |
| `mode_id` | `int \| None` | `None` | Use the model's signature mode for the negotiated USB link |
| `serial_number` | `str` | `""` | No serial filter (substring match) |
| `usb_port` | `str` | `""` | No USB-path filter (exact match) |

Given both `serial_number` and `usb_port`, only a camera matching both is
accepted — pin the serial alone to follow a unit across ports. Selection
must resolve to exactly one camera.

Use `ey.supported_models()` to list supported model strings.
Use `ey.load_catalog(model)` to list all mode ids and their stream specs;
each `ModeDescriptor` carries `usb`, the link that mode needs. For the fuller
picture, `ey.load_model(model)` returns a `ModelInfo` — the same mode catalog
plus the model's IR range, product id, and per-link signature modes.

### `set_ir_value(value: int) → Config`

IR projector intensity at start.

| Value | Meaning |
|---|---|
| `-1` (default) | Per-model default: on for depth/mono modes, off for color-only modes |
| `0` | Off |
| `1..N` | Model-specific maximum (G62: 96, G100+/R77: 6) |

### `set_auto_exposure(enabled: bool) → Config`

Enable (`True`) or disable (`False`) auto-exposure at start.
Default: device power-on state (auto on).

### `set_exposure(value: int) → Config`

Set a fixed exposure; implicitly disables auto-exposure.

The unit is a signed log2 register the modules share (`-13` to `3`). Read
it back with `Pipeline.get_exposure_range()` after `start()`.

### `set_auto_white_balance(enabled: bool) → Config`

Enable or disable auto white balance at start.
Default: device power-on state (AWB on).

### `set_white_balance(value: int) → Config`

Set a fixed white-balance value; implicitly disables AWB.
Typical range: 2800–6500 K (device-specific). Read with
`Pipeline.get_white_balance_range()` after `start()`.

The monochrome models (G62, R77) have no white balance: setting it, or
`set_auto_white_balance()`, raises `ValueError` when the config resolves.

### `set_power_line_frequency(mode: int) → Config`

Anti-flicker: sync exposure to the mains frequency so artificial lighting
does not band the image. eYs3D cameras support 50/60 Hz only.

| Value | Meaning |
|---|---|
| `1` | 50 Hz |
| `2` | 60 Hz |

(UVC also defines `0` = Disabled and `3` = Auto, but these cameras reject
them, so the API accepts only `1` / `2`.)

### `with_filters(*filters) → Config`

Attach depth post-process filters. They run in the capture thread in a
fixed order — spatial, then temporal, then hole filling — whatever order
they are passed in; one filter of each type is kept. See the **Filters**
section for available types.

Without filters, depth is delivered as raw firmware millimeters.
With a `SpatialFilter` or `TemporalFilter`, the pipeline switches to
11-bit disparity internally and converts back to mm after filtering.

### `set_depth_range(near_mm: int = -1, far_mm: int = -1) → Config`

Hard-clip the depth range. Pixels outside `[near_mm, far_mm]` are
set to 0 (no-data) before the frame leaves the capture thread.

| Value | Meaning |
|---|---|
| `-1` (default) | Per-model default (G100+: 250–1900 mm, R77: 200–1500 mm, G62: 100–1500 mm) |
| `≥ 0` | Use this value |

`near_mm` must stay below `far_mm`, and `far_mm` at most 16383 — the
largest distance the 14-bit depth format carries. A lone `near_mm` at or
above the model's default far clip is refused when the config resolves.

The same range is read back via `Pipeline.depth_near_mm` /
`Pipeline.depth_far_mm` and is used by `Colorizer(pipeline)` as the
display range.

### `set_depth_quality_registers(source: bool | str = True) → Config`

Apply firmware depth-quality tuning registers after the depth stream
settles (a few seconds in; re-applied after reconnects).

| Value | Meaning |
|---|---|
| `True` (default) | Apply the model's bundled profile |
| `False` | Leave firmware defaults untouched |
| `"path/to/file"` | Apply a custom profile (one `address,mask,value` hex triple per line) |

---

## Pipeline

Opens the device, manages the capture thread, and delivers frames.
Use as a context manager to guarantee clean teardown.

```python
with ey.Pipeline() as pipeline:
    pipeline.start(cfg)
    frames = pipeline.wait_for_frames(timeout_ms=1000)
```

### `Pipeline()`

No parameters.

### `start(config: Config) → None`

Open the device and start streaming. Raises `RuntimeError` if no matching
device is found or the device cannot be opened.

The camera must stay attached for the duration of the call. Unplugging it
mid-open lands inside the SDK's own open call, which the driver cannot
interrupt or time out, so `start()` may block until the camera is
reattached. Hot-plug recovery covers drops after this call returns.

### `stop() → None`

Stop streaming and close the device. Called automatically on context
manager exit.

### `wait_for_frames(timeout_ms: int = 1000) → FrameSet | None`

Block until the next frame set arrives, or until `timeout_ms` elapses.
Returns `None` on timeout (e.g. a momentary USB hiccup). Continues to
deliver frames after the device auto-reconnects; check
`reconnect_count` to detect recoveries.

### `poll_for_frames() → FrameSet | None`

Non-blocking variant — returns `None` immediately if no frame is ready.

### Properties (read-only after `start()`)

| Property | Type | Description |
|---|---|---|
| `device_info` | `OpenedDevice` | Opened device identity — see [OpenedDevice fields](#openeddevice-fields) |
| `intrinsics` | `Intrinsics \| None` | The camera model stored for the active video mode — see [Intrinsics](#intrinsics) |
| `color_profile` | `StreamProfile \| None` | Active color stream spec (`.width`, `.height`, `.fps`) |
| `depth_profile` | `StreamProfile \| None` | Active depth stream spec |
| `fps` | `StreamFps` | Measured delivery rate per stream (`.color`, `.depth`) — see [fps](#fps) |
| `depth_near_mm` | `int` | Near clip applied by the engine (from `set_depth_range` or model default) |
| `depth_far_mm` | `int` | Far clip applied by the engine |
| `is_streaming` | `bool` | True between `start()` and `stop()` |
| `is_connected` | `bool` | False while the watchdog is recovering from a USB drop |
| `reconnect_count` | `int` | Number of USB-drop recoveries since `start()` |
| `frames_dropped` | `DroppedFrames` | Wire drops per stream (`.color`, `.depth`); gaps between consumer reads are not counted |
| `quality_registers` | `QualityRegisters` | Depth-quality register write outcome (`.applied`, `.failed`, `.pending`); reads `0 / 0 / False` before it runs, on a depthless mode, or with the profile disabled |

### `OpenedDevice` fields

What `device_info` returns: the identity of the camera this pipeline
opened. Distinct from `DeviceInfo`, which `Context.query_devices()`
returns *before* anything is opened.

| Field | Type | Description |
|---|---|---|
| `model` | `str` | Model string, e.g. `"G100P"` |
| `serial_number` | `str` | Unit serial; pass to `Config.enable_device(serial_number=)` to pin this camera |
| `usb_port` | `str` | USB path, e.g. `"2-1.3"`; pass to `Config.enable_device(usb_port=)` to pin this port |
| `usb_port_type` | `int` | Negotiated USB port type: `2` = USB 2.0, `3` = USB 3.0 (`0` = unknown) |
| `usb_speed` | `str` | Human-readable form of the above: `"USB2.0"`, `"USB3.0"`, or `"unknown"` |
| `pid` | `int` | USB product id |
| `firmware_version` | `str` | Firmware string — quote it in any support report |

The optical calibration is not part of this structure: identity is known
at enumeration, intrinsics only after `start()`.

### `fps`

`StreamFps(color, depth)` — the rate at which frames actually reach the
host, in Hz, measured where the engine publishes them.

```python
rate = pipeline.fps
print(f"{rate.depth:.1f} of {pipeline.depth_profile.fps} fps nominal")
```

Because it is measured at delivery rather than where you read, it does
not drop when your own loop is slow. That is what makes it a diagnostic:

- **`fps` matches the nominal rate, your loop is slower** — the time is
  going into your code, not the camera.
- **`fps` is below the nominal rate** — the camera or the USB link is not
  keeping up; check `frames_dropped`, the exposure time, and cabling.

An **interleaved** video mode alternates color and depth on the wire, so
each stream lands at half the mode's rate; the mode name says so
(e.g. `"L'+D 1280x720@60 interleave (SDK 30fps)"`), and
`StreamProfile.fps` reports the mode's rate, not the per-stream one.

A stream silent for more than a second reads `0.0`, so a stalled stream
never reports its last healthy rate. There is no measurement until the
second frame of a stream arrives.

### Runtime control setters (callable while streaming)

All raise `ValueError` on out-of-range input and `RuntimeError` if the
device rejects the command.

#### `set_ir_value(value: int) → None`

-1 restores the model default. See `get_ir_range()` for the valid range.

#### `set_auto_exposure(enabled: bool) → None`

#### `set_exposure(value: int) → None`

Validated against `get_exposure_range()`. Implicitly turns AE off.

#### `set_auto_white_balance(enabled: bool) → None`

#### `set_white_balance(value: int) → None`

Validated against `get_white_balance_range()`. Implicitly turns AWB off.
Both raise `ValueError` on the monochrome models (G62, R77), which have
no white balance.

#### `set_power_line_frequency(mode: int) → None`

1 = 50 Hz, 2 = 60 Hz (anti-flicker). eYs3D cameras do not support UVC's
0 = disabled or 3 = auto, so only 1 / 2 are accepted; raises otherwise.

#### `set_temporal_filter(*, alpha=None, delta=None, persistence=None) → None`

Retune the temporal filter mid-stream. Requires that a `TemporalFilter`
was passed to `Config.with_filters()`. Parameters left `None` keep their
current value.

### Runtime control getters

All return `None` before `start()`. Each range has its own source: the IR
range comes from the model catalog, the exposure range is the fixed
register range the modules share, and the white-balance range is what the
device reports — so it alone can also be `None` while streaming, when the
camera does not implement the control.

| Method | Return type | Description |
|---|---|---|
| `get_ir_value()` | `int \| None` | Current IR projector level |
| `get_ir_range()` | `ControlRange \| None` | Valid IR range (`.min`, `.max`, `.step`, `.default`) |
| `get_auto_exposure()` | `bool \| None` | AE state |
| `get_exposure()` | `int \| None` | Current exposure register value |
| `get_exposure_range()` | `ControlRange \| None` | Valid exposure range |
| `get_auto_white_balance()` | `bool \| None` | AWB state |
| `get_white_balance()` | `int \| None` | Current WB value |
| `get_white_balance_range()` | `ControlRange \| None` | Valid WB range |
| `get_power_line_frequency()` | `int \| None` | Current anti-flicker mode |

### Device operations

#### `hardware_reset() → None`

Reset the camera over USB — a firmware-triggered detach and
re-enumeration, equivalent to a physical replug. The device drops off the
bus and comes back; the watchdog rebinds it by port identity and streaming
resumes on its own, so `is_connected` goes `False` for the reconnect and
`reconnect_count` then increments. Recovers an unresponsive camera
without physical access.

The firmware acknowledges the triggering writes unreliably (the link is
already going away), so this reports nothing and never raises on a missing
acknowledge. Requires a started pipeline.

---

## Intrinsics

`Pipeline.intrinsics` — the camera model the active video mode maps to,
passed through as the device stores it. `None` before `start()` or on a
unit with no calibration.

**Most applications never read this.** Frames are delivered rectified and
`PointCloud.calculate()` reprojects for you, with no intrinsics argument.
Reach for it to project your own 3-D points onto the image, to deproject a
few pixels without building a whole cloud, or to hand the model to another
library.

| Field | Type | Description |
|---|---|---|
| `width`, `height` | `int` | The size the model is expressed at |
| `fx`, `fy` | `float` | Focal length in pixels |
| `cx`, `cy` | `float` | Principal point in pixels |
| `K` | `(9,) float64` | 3x3 camera matrix, row-major |
| `D` | `(8,) float64` | Distortion: `k1, k2, p1, p2, k3, k4, k5, k6` |
| `R` | `(9,) float64` | 3x3 rectification rotation, row-major |
| `P` | `(12,) float64` | 3x4 rectified projection, row-major |
| `baseline_mm` | `float` | Distance between the two lens centers |

`fx`, `fy`, `cx`, `cy` are `P[0]`, `P[5]`, `P[2]`, `P[6]` — the same
values, named for convenience.

### Using the values

**The images you receive are already rectified.** That makes `fx` / `fy` /
`cx` / `cy` (equivalently `P`) the model that describes them. `K` and `D`
describe the raw sensor *before* rectification: do not apply `D` to a
delivered frame — the distortion is already removed, and undistorting a
second time bends a straight image. They are here because the device
stores them, for calibration work and for tools that want a full model.

`D` is the **rational polynomial** model — `k1, k2, p1, p2, k3, k4, k5,
k6`. That is eight coefficients, not the five of the plumb-bob model most
libraries default to; passing them to a function expecting five silently
fits the wrong model. Nothing in this driver applies `D`, and rectified
delivery means an application does not need to either.

**The values are expressed at `width` x `height`, which is not
necessarily your stream's size.** The device stores several calibrations
and each video mode selects the one for its resolution, so `fx` differs
between modes and usually already matches what you receive. The exception
is a scale-down mode, which delivers depth smaller than the calibration it
selected. `PointCloud` rescales internally, so nothing is needed for the
normal path; only a caller deprojecting by hand has to scale, by the
height ratio:

```python
intr = pipeline.intrinsics
prof = pipeline.depth_profile
r  = prof.height / intr.height          # 1.0 unless the mode scales down
fx, fy = intr.fx * r, intr.fy * r
cx, cy = intr.cx * r, intr.cy * r

dmm = depth.get_data()
z = dmm[v, u] / 1000.0                  # millimeters -> meters
x = (u - cx) * z / fx
y = (v - cy) * z / fy                   # optical: X right, Y down, Z forward
```

Projecting a 3-D point back onto the image is the same relation inverted,
with the ratio for the stream you are drawing on.

### When the unit has no calibration

`intrinsics` is `None`. The native layer warns at
`PYEYS3D_LOG_LEVEL=warn` (the default) with the consequences:

- color is **not rectified**
- depth still streams, but its **quality is degraded** — expect broken,
  patchy values rather than a blank image
- `PointCloud(pipeline)` raises `RuntimeError`; there is no point cloud

---

## FrameSet

Returned by `Pipeline.wait_for_frames()`. Each accessor returns `None`
when the corresponding stream is inactive in the current mode.

### `get_color_frame() → Frame | None`

Left (primary) color frame. Always use this for point-cloud coloring —
depth is aligned to the left camera.

### `get_right_color_frame() → Frame | None`

Right color frame. Present in stereo-color modes (e.g. G100+). Not
available in depth-only modes.

### `get_depth_frame() → Frame | None`

Depth frame as `uint16` mm after clipping and optional filtering.

---

## Frame

Wraps one captured image buffer.

### Properties

| Property | Type | Description |
|---|---|---|
| `domain` | `FrameDomain` | `COLOR_RGB8` or `DEPTH_MM` |
| `width` | `int` | Image width in pixels |
| `height` | `int` | Image height in pixels |
| `frame_number` | `int` | Device-side counter for **this stream** — see the step rule below |
| `hw_timestamp_us` | `int` | Capture timestamp on the device clock, microseconds |
| `timestamp` | `float` | Host-clock capture time (seconds, same epoch as `time.time()`) |

`timestamp` is `hw_timestamp_us` mapped onto the host clock, so it is
comparable with `time.time()` and with other sensors. It always increases.

Two rules for `frame_number`:

- **It steps by 2 in an interleaved mode**, where the two streams share one
  device sequence and take alternate numbers. That is not a drop — read
  `frames_dropped` for real losses.
- **Color and depth count separately**, so their numbers never match. Pair
  frames by `timestamp`, not by `frame_number`. The left and right color
  frames of a split mode do share one number, being one exposure.

### `get_data() → np.ndarray`

Returns the image as a NumPy array:
- Color: `(H, W, 3) uint8` in **RGB** order
- Depth: `(H, W) uint16` in **millimeters**

The array is a **read-only view** onto the frame's own buffer, valid for as
long as the `Frame` is alive. Copy it (`np.array(frame.get_data())`) to edit
the pixels or to keep them past the frame.

### `get_data_bgr() → np.ndarray`

Color frames only: a fresh, **writable** `(H, W, 3) uint8` array in **BGR**
order, ready for `cv2.imshow()` or AI model inference without a separate
`cvtColor` call. Depth frames pass through unchanged, and so keep
`get_data()`'s read-only view.

---

## Colorizer

Maps a depth frame to an RGB or grayscale image for display.
Requires `pip install opencv-python`.

```python
colorizer = ey.Colorizer(pipeline)            # build once after start()
rgb   = colorizer.colorize(depth)             # (H, W, 3) uint8 RGB
bgr   = colorizer.colorize_bgr(depth)         # (H, W, 3) uint8 BGR for cv2
```

### `Colorizer(pipeline=None, *, mode="color", min_mm=None, max_mm=None)`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `pipeline` | `Pipeline \| None` | `None` | Take `min_mm`/`max_mm` from the pipeline's depth clip range. Requires the pipeline to be started. |
| `mode` | `"color" \| "grayscale"` | `"color"` | `"color"` = JET false-color (blue→red); `"grayscale"` = linear gray ramp (R=G=B) |
| `min_mm` | `int \| None` | `None` | Explicit near range; overrides pipeline value |
| `max_mm` | `int \| None` | `None` | Explicit far range; overrides pipeline value |

With no range at all (no pipeline, no explicit values), the scale runs
from 0 to the frame's own maximum — colors are stable within a frame but
flicker between frames as the scene changes.

Depth value 0 (no-data) always renders black regardless of range.

### `colorize(depth) → np.ndarray`

`depth` may be a `Frame` or a `(H, W) uint16` ndarray.
Returns `(H, W, 3) uint8` **RGB**.

### `colorize_bgr(depth) → np.ndarray`

Same as `colorize()` but returns `(H, W, 3) uint8` **BGR** for direct
`cv2.imshow()` use.

---

## PointCloud

Reprojects depth pixels to 3-D metric coordinates using the stereo
calibration. Requires `Pipeline.intrinsics` to be present; on an
uncalibrated unit it is `None` and the constructor raises `RuntimeError`.

```python
pc = ey.PointCloud(pipeline)

# geometry only
verts, _    = pc.calculate(depth_frame)

# geometry + color from camera
verts, cols = pc.calculate(depth_frame, color_frame)
```

### `PointCloud(pipeline: Pipeline)`

Raises `RuntimeError` if the pipeline is not started, or if the unit
carries no calibration (the same condition that makes `intrinsics`
`None`).

### `calculate(depth: Frame, color: Frame | None = None) → (np.ndarray, np.ndarray | None)`

Reproject valid depth pixels to 3-D.

| Parameter | Default | Description |
|---|---|---|
| `depth` | required | A `DEPTH_MM` frame from `get_depth_frame()` |
| `color` | `None` | A `COLOR_RGB8` frame for per-point coloring. Any resolution — the engine scales with nearest-neighbour to match the depth grid |

Returns:
- `verts`: `(N, 3) float32` — XYZ in meters, **optical convention** (X right, Y down, Z forward). N = number of valid (non-zero) depth pixels, compacted — no padding.
- `colors`: `(N, 3) uint8` RGB per vertex when `color` is provided; `None` otherwise.

The compaction order matches NumPy raster scan
(`depth.get_data().flatten()`), so

```python
valid_mask = depth.get_data().flatten() != 0
```

selects exactly the same N elements, in the same order, from any 2-D
array registered to the depth grid — which is how the examples colour a
cloud from the depth colormap:

```python
verts, _ = pc.calculate(depth)
colors = colorizer.colorize(dmm).reshape(-1, 3)[dmm.flatten() != 0]
```

This holds because the delivered depth frame carries no non-depth pixels:
row 0 opens with a device serial watermark, and the engine zeroes it
before delivery so it can never enter the mask.

---

## Filters

Passed to `Config.with_filters()` before `start()`. The chain runs in
order in the capture thread; output is always `uint16` mm.

### `SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0)`

Edge-preserving spatial IIR over the disparity image.

| Parameter | Range | Default | Description |
|---|---|---|---|
| `alpha` | 0..1 | 0.5 | Smoothing strength; 1 = no smoothing |
| `delta` | 1..4095 | 20 | Edge threshold in disparity units; pixels above this gap are kept |
| `magnitude` | 1..5 | 2 | Number of four-direction passes |
| `holes_fill` | ≥ 0 | 0 | Max hole run bridged per pass; 0 = smoothing only |

### `TemporalFilter(alpha=0.4, delta=20, persistence=3)`

Temporal IIR across frames in the disparity domain.

| Parameter | Range | Default | Description |
|---|---|---|---|
| `alpha` | 0..1 | 0.4 | Current-frame blend weight |
| `delta` | 0..4095 | 20 | Per-pixel disparity gap above which blending is skipped |
| `persistence` | 0..8 | 3 | Frames a last-valid value is held over dropouts |

Can be retuned while streaming via `Pipeline.set_temporal_filter()`.

Every range above is enforced when the filter is constructed, so a value
outside it raises `ValueError` there rather than reaching the kernels,
which clamp what they are handed. `Pipeline.set_temporal_filter()` holds
callers to the same ranges.

### `HoleFillingFilter(mode=HoleFill.FARTHEST_AROUND)`

Fill zero-depth holes in the mm image (runs after spatial/temporal).

| `mode` | Value | Description |
|---|---|---|
| `HoleFill.OFF` | 0 | No fill |
| `HoleFill.FROM_LEFT` | 1 | Carry last valid value rightward |
| `HoleFill.FARTHEST_AROUND` | 2 | Farthest valid neighbour around the hole |
| `HoleFill.NEAREST_AROUND` | 3 | Nearest valid neighbour around the hole |

---

## Opening several cameras

Cameras are set up one at a time within a process: `Pipeline.start()`,
`Context.query_devices()` and the driver's own reconnect take one turn
between them, because the SDK's device bookkeeping is per process rather
than per handle. Opening from several threads is therefore safe but not
faster — the opens queue. Separate processes do not share that state and
open in parallel, which is the arrangement `examples/06_multicam.py` uses.

Overlapping opens fail the flash reads behind `APC_GetSerialNumber` and
the rectify log, which leaves a camera streaming with `intrinsics` set to
`None`. The driver serialises them; callers need do nothing.

Every example is described in
[`examples/README.md`](../examples/README.md), including `viewer.py`,
which drives this whole API from an on-screen menu.
