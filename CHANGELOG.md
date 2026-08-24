# Changelog

All notable changes to **pyeys3d** are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/), and the project adheres to
[Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-08-24

The initial release — a direct-eSPDI Python wrapper for eYs3D stereo depth
cameras, with a pipeline-based public API.

### Added

- `Pipeline`, `Config`, `Frame`, `FrameSet`, and `Context` — the capture API,
  plus `Context.query_devices()` enumeration.
- Support for **G100+ / R77 / G62** over USB on **Linux (x86_64, aarch64)
  and Windows 10/11 (x64)**, with per-model video-mode catalogs under
  `pyeys3d/modes/`. One shared capture engine over per-OS backends: V4L2
  pull on Linux, the SDK callback push on Windows.
- Video-mode catalogs are **USB-port-type aware**: each mode declares the
  link it needs (USB 2.0 / 3.0), `start()` picks the model's signature mode
  for the negotiated link, and `DeviceInfo` / `device_info` report
  `usb_port_type` / `usb_speed`.
- **`examples/viewer.py`** — every camera setting on one screen, changed
  while the camera runs: live color / depth with hover readout, camera
  controls, video mode / depth clip / filter switching, snapshot and clip
  capture, and an optional 3D point cloud window. Every connected camera
  runs in the one process, each with its own window set; with more than
  one, each stops on a mode picker before streaming so they can be fitted
  to the shared USB bus.
- Examples take one option per API call they make: `--model` / `--mode` /
  `--serial` / `--usb-port` / `--ir-value` / `--depth-range` / `--filters`,
  the same seven in `01` through `06`. How depth is colored and where cloud
  points take their color are defaults each example sets in one visible
  line, not options.
- Cameras are **set up one at a time within a process**. `Pipeline.start()`,
  `Context.query_devices()` and the driver's reconnect take one turn between
  them: the SDK's device bookkeeping is per process, and overlapping opens
  leave cameras streaming with no intrinsics. Separate processes are
  unaffected. `Context.query_devices()` also releases the GIL.
- Color in YUYV and MJPEG, decoded to `rgb8`; the monochrome modules
  (G62 / R77) deliver grayscale (R = G = B). Wide **L|R color split** in the
  side-by-side stereo modes (`split_lr` in the catalog) via
  `get_right_color_frame()`.
- **Depth** in millimeters, with an optional post-process chain — spatial,
  temporal, and hole filling — declared through `Config.with_filters(...)`.
- **Firmware depth-quality register profiles** bundled per model, applied in
  the background once depth streaming starts and settles, and re-applied
  after a reconnect; `Config.set_depth_quality_registers()` disables them or
  substitutes a custom profile file.
- **Point-cloud reprojection** (`PointCloud`) producing XYZ or XYZRGB in the
  optical frame — a two-pass OpenMP kernel with NEON row counting on
  aarch64, delivering a compacted cloud.
- **Per-frame metadata** — per-stream serial number, hardware timestamp, and
  a host-clock capture time from a fitted hardware-to-host clock model —
  plus wire-drop counters (`Pipeline.frames_dropped`).
- **Camera model** — `Pipeline.intrinsics` returns `Intrinsics(width,
  height, fx, fy, cx, cy, K, D, R, P, baseline_mm)` for the active video
  mode, passed through as the device stores it. The depth clip range reads
  back as `Pipeline.depth_near_mm` / `depth_far_mm`.
- **Measured delivery rate** — `Pipeline.fps` reports the rate each stream
  actually reaches the host, sampled where the engine publishes frames, so
  it is independent of how fast the application reads. Against the mode's
  nominal `StreamProfile.fps` it separates a capture shortfall from an
  application that cannot keep up.
- Device controls — IR-projector level (`ir_value`, with a mode-aware
  default),
  auto/manual exposure, auto/manual white balance, and power-line
  frequency — settable declaratively at `start()` via `Config` **and at
  runtime** via the matching `Pipeline.set_*` methods (with
  `Pipeline.set_temporal_filter(...)` to retune the running filter chain).
  Runtime values survive a hot-plug reconnect.
- Control range queries returning `ControlRange(min, max, step, default)`:
  `get_exposure_range()` reports the modules' fixed signed log2 register
  range, `get_white_balance_range()` what the device reports, and
  `get_ir_range()` the model catalog. The runtime setters validate against
  them.
- Controls are held to what the cameras support, so an unsupported request
  fails on the host rather than at the device. A fixed range is refused by
  the setter (power-line frequency takes 50 or 60 Hz, the depth clip stays
  within the 14-bit range); what depends on the camera and mode chosen is
  refused by `start()` (the monochrome modules have no white balance, and a
  video mode opens only on the USB link it declares).
- Device binding by USB topology (exact) or serial number (substring). When
  both are given they are **AND-ed** — the unit must match the serial *and*
  sit on that port — so a saved profile carrying a now-stale `usb_port`
  alongside a serial will not open; pin the serial alone to follow a camera
  across ports. Ambiguous selection fails with the candidate list instead of
  silently picking a camera.
- **Hot-plug recovery** — a watchdog reopens the device after a USB drop,
  re-binding by USB port, then serial number, then PID; a pinned serial is
  re-verified so a different same-model unit on the same port is never bound
  in the original's place.
- `Pipeline.hardware_reset()` — a firmware-triggered USB detach and
  re-enumeration, equivalent to a physical replug, so an unresponsive camera
  can be recovered without physical access. The watchdog reconnects it on its
  own (`reconnect_count` increments).
- Native errors name the eSPDI code (generated from the SDK header at build
  time) with a hint for the common cases, instead of a bare `rc=` number.
  Conditions that leave the camera streaming but unusable — a missing
  calibration, a depth stream that never starts, an unmet firmware
  ordering constraint — are reported at the default `PYEYS3D_LOG_LEVEL`.
- `Colorizer.colorize_bgr()` and `Frame.get_data_bgr()` deliver OpenCV
  channel order directly, with no separate `cvtColor` pass.
- A full API reference in `docs/api.md`, covering the conventions (units,
  coordinate frame, channel order), object lifecycle, exceptions, the
  threading rules, and logging.
- Pre-built wheels for Linux x86_64 / aarch64 and Windows x64
  (CPython 3.8–3.13) bundling the eSPDI runtime, plus type stubs
  (`py.typed`), and a standalone examples bundle attached to each Release.
  The Linux wheels bundle the GCC runtime with its symbols localized, so the
  binding coexists in one process with a second, independently linked
  libstdc++ / OpenCV (a plain `import cv2`) without symbols from the two
  clashing.
- Examples with a four-language README in the bundle: `hello_depth` (zero
  dependencies), device enumeration, a self-contained quick start, an
  OpenCV color/depth viewer, twin point-cloud viewers (pyglet/OpenGL and
  Open3D — identical except the display layer), snapshot / record /
  playback (`04_capture`), live runtime-control adjustment, and
  process-per-camera multi-camera streaming.
