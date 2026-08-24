"""Pipeline orchestrator — owns one open device and delivers frame sets.

Typical use:

    import pyeys3d as ey

    pipeline = ey.Pipeline()
    config = ey.Config().enable_device("G100P", mode_id=1)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if frames is None:
                continue
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            # color.get_data() -> (H, W, 3) uint8 rgb8
            # depth.get_data() -> (H, W) uint16 mm
            ...
    finally:
        pipeline.stop()
"""

from __future__ import annotations

from collections import namedtuple
from typing import Optional

from ._pyeys3d_native import (
    EXPOSURE_UNSET,
    CaptureEngine,
    Frame,
    FrameDomain,
    OpenConfig,
)
from .config import _EXPOSURE_MAX, _EXPOSURE_MIN, Config, ResolvedConfig
from .context import Context, _usb_speed
from .filters import TemporalFilter
from .modes import ModelInfo, load_model, model_for_pid, pid_for_model

# The device a Pipeline actually opened (resolved from auto-detect, or from
# the model / serial_number / usb_port constraints on the Config).
OpenedDevice = namedtuple(
    "OpenedDevice",
    ["model", "serial_number", "usb_port", "usb_port_type", "usb_speed",
     "pid", "firmware_version"])

# Bounds of a camera control (register units); each accessor documents
# whether they come from the device, the model catalog, or the module spec.
ControlRange = namedtuple("ControlRange", ["min", "max", "step", "default"])

# A stream's output spec, known right after start(). fps is the mode's
# nominal rate; Pipeline.fps is what arrives.
StreamProfile = namedtuple("StreamProfile", ["width", "height", "fps"])

# Camera-produced frames the host never received, per stream.
DroppedFrames = namedtuple("DroppedFrames", ["color", "depth"])

# Outcome of the depth-quality register profile.
QualityRegisters = namedtuple("QualityRegisters",
                              ["applied", "failed", "pending"])

# Rate at which frames actually reach the host, per stream (Hz).
StreamFps = namedtuple("StreamFps", ["color", "depth"])

# The stored camera model for the active video mode. width/height are the
# size fx/fy/cx/cy are expressed at, not the stream size. K (9,), D (8,:
# k1 k2 p1 p2 k3 k4 k5 k6), R (9,) and P (12,) are flat — reshape before use.
Intrinsics = namedtuple(
    "Intrinsics",
    ["width", "height", "fx", "fy", "cx", "cy",
     "K", "D", "R", "P", "baseline_mm"])


def _build_open_config(resolved: ResolvedConfig, device) -> OpenConfig:
    """Map a ResolvedConfig + selected device onto the native OpenConfig.

    Kept pure so the resolved→native field mapping is unit-testable
    without a camera."""
    oc = OpenConfig()
    # Pin to the resolved device so selection is deterministic even when
    # several cameras are connected and only a model (or nothing) was set.
    oc.usb_port = device.usb_port
    oc.serial_number = device.serial_number
    oc.expected_pid = pid_for_model(resolved.model)
    oc.is_mono = resolved.mono

    m = resolved.mode
    oc.color_w = m.color.width
    oc.color_h = m.color.height
    oc.color_fmt = m.color.fmt
    oc.color_split_lr = m.color.split_lr
    oc.depth_w = m.depth.width
    oc.depth_h = m.depth.height
    oc.depth_dtype = m.depth.dtype
    oc.zd_index = m.zd_index
    oc.fps = m.fps
    oc.interleave = m.interleave
    oc.ir_value = resolved.ir_value
    # Tri-state: leave the camera untouched unless the user set it.
    if resolved.enable_auto_exposure is not None:
        oc.auto_exposure = 1 if resolved.enable_auto_exposure else 0
    oc.exposure_time = (EXPOSURE_UNSET if resolved.exposure_time is None
                        else resolved.exposure_time)
    if resolved.enable_auto_white_balance is not None:
        oc.auto_white_balance = 1 if resolved.enable_auto_white_balance else 0
    oc.white_balance = resolved.white_balance
    oc.power_line_frequency = resolved.power_line_frequency

    if resolved.spatial_filter is not None:
        s = resolved.spatial_filter
        oc.filter_spatial = True
        oc.spatial_alpha = float(s.alpha)
        oc.spatial_delta = int(s.delta)
        oc.spatial_magnitude = int(s.magnitude)
        oc.spatial_holes_fill = int(s.holes_fill)
    if resolved.temporal_filter is not None:
        t = resolved.temporal_filter
        oc.filter_temporal = True
        oc.temporal_alpha = float(t.alpha)
        oc.temporal_delta = int(t.delta)
        oc.temporal_persistence = int(t.persistence)
    if resolved.hole_filling_filter is not None:
        oc.filter_hole = True
        oc.hole_mode = int(resolved.hole_filling_filter.mode)

    oc.depth_near_mm = resolved.depth_near_mm
    oc.depth_far_mm = resolved.depth_far_mm
    oc.quality_regs = [list(t) for t in resolved.quality_registers]
    return oc


class FrameSet:
    """A matched color + depth set from one wait_for_frames() call.

    In wide L|R split modes the right color frame is also available via
    get_right_color_frame(); it shares the left frame's timestamp.
    """

    def __init__(self, color: Optional[Frame], depth: Optional[Frame],
                 right_color: Optional[Frame] = None) -> None:
        self._color = color
        self._depth = depth
        self._right = right_color

    def get_color_frame(self) -> Optional[Frame]:
        """The left / primary color frame, or None if this set has no color."""
        return self._color

    def get_right_color_frame(self) -> Optional[Frame]:
        """The right-eye color frame in wide L|R split modes, else None."""
        return self._right

    def get_depth_frame(self) -> Optional[Frame]:
        """The depth frame, or None if this set has no depth stream."""
        return self._depth

    def __repr__(self) -> str:
        c = self._color.frame_number if self._color else None
        d = self._depth.frame_number if self._depth else None
        return f"FrameSet(color_frame_number={c}, depth_frame_number={d})"


class Pipeline:
    """Top-level orchestrator.

    Wraps a single CaptureEngine instance. start(), stop(), and the frame
    accessors (wait_for_frames / poll_for_frames) must be called from one
    thread. The camera-control setters and getters (set_* / get_*) may be
    called from another thread while streaming — e.g. a UI thread adjusting
    exposure while a capture thread collects frames. They are not
    synchronized against stop(): a setter racing teardown raises
    RuntimeError rather than acting on a stopped pipeline, and a reader
    racing it returns the same value it would after stop(). Both take one
    snapshot of the engine and work from it, so neither can see it become
    None between checking and using it.
    """

    def __init__(self) -> None:
        self._engine: Optional[CaptureEngine] = None
        self._streaming = False
        self._color_profile: Optional[StreamProfile] = None
        self._depth_profile: Optional[StreamProfile] = None
        self._model_info: Optional[ModelInfo] = None   # of the opened camera
        self._temporal_spec: Optional[TemporalFilter] = None
        self._usb_port_type = 0                 # negotiated link, from select
        # Bumped per start(); PointCloud records it to detect a restart.
        self._start_generation = 0

    @staticmethod
    def _select_device(config: Config):
        """Pick the device to open from those connected.

        The model filters by camera type; serial_number (substring) and
        usb_port (exact) identify one unit and are AND-ed when both are
        given (this unit on this port). Selection must resolve to exactly
        one camera — anything ambiguous raises with the candidate list so
        the caller can pin one."""
        devices = [d for d in Context().query_devices()
                   if model_for_pid(d.pid) is not None]
        if not devices:
            raise RuntimeError("No eYs3D camera connected.")

        def _summary(ds):
            return "".join(f"\n  {d.model}  sn='{d.serial_number}'  "
                           f"usb_port='{d.usb_port}'" for d in ds)

        cands = devices
        if config.model:
            cands = [d for d in cands if d.model == config.model]
            if not cands:
                raise RuntimeError(
                    f"No {config.model} connected. "
                    f"Connected:{_summary(devices)}")

        if config.serial_number or config.usb_port:
            cands = [d for d in cands
                     if (not config.serial_number
                         or config.serial_number in d.serial_number)
                     and (not config.usb_port
                          or d.usb_port == config.usb_port)]
            if not cands:
                wanted = " and ".join(
                    s for s in (config.serial_number
                                and f"serial_number~{config.serial_number!r}",
                                config.usb_port
                                and f"usb_port={config.usb_port!r}") if s)
                raise RuntimeError(
                    f"No camera matches {wanted}. "
                    f"Connected:{_summary(devices)}")

        if len(cands) > 1:
            raise RuntimeError(
                f"{len(cands)} cameras match; pin one with "
                f"Config.enable_device(serial_number=...) or usb_port=...:"
                f"{_summary(cands)}")
        return cands[0]

    def start(self, config: Config) -> None:
        """Open the device and begin streaming.

        With an unconfigured Config (or one without a model), the connected
        camera is auto-detected and opened in its signature mode.

        Raises RuntimeError if the pipeline is already started, if no camera
        matches, if several do, or if the device cannot be opened; ValueError
        if the Config does not resolve against the model's catalog. A failed
        start() leaves the pipeline not-started, so a retry is safe. Returns
        once the streams are open — the first frame lands seconds later, a
        firmware property that varies per module and per video mode.

        The camera must stay attached for the duration of this call: a drop
        mid-open lands inside the SDK's own open call, which cannot be
        interrupted. Hot-plug recovery covers drops after it returns."""
        if self._streaming:
            raise RuntimeError(
                "Pipeline already started; call stop() before starting again")
        device = self._select_device(config)
        self._usb_port_type = device.usb_port_type
        resolved = config._resolve(device_model=model_for_pid(device.pid),
                                   usb_type=device.usb_port_type)
        oc = _build_open_config(resolved, device)
        m = resolved.mode

        # Per-stream output dimensions, knowable before the first frame. Wide
        # L|R modes output half the wire width per eye; a stream absent from
        # the mode (color-only / depth-only) resolves to None. The width
        # must be a multiple of 4 to split — half of it lands on a YUYV
        # macropixel boundary — matching the native ingest, which leaves an
        # odd-half width unsplit.
        split = m.color.split_lr and (m.color.width % 4 == 0)
        cw = m.color.width // 2 if split else m.color.width
        color_profile = (StreamProfile(cw, m.color.height, m.fps)
                         if m.color.width > 0 else None)
        depth_profile = (StreamProfile(m.depth.width, m.depth.height, m.fps)
                         if m.depth.width > 0 else None)

        # Closed by hand rather than left to the destructor: the traceback
        # keeps this frame, and so the engine, alive well past the except
        # block, and a retry would then reopen a device the failed attempt
        # still holds.
        engine = CaptureEngine()
        try:
            engine.open(oc)
            engine.start()
        except BaseException:
            engine.close()
            raise
        # Published only once the streams are up, so device_info and the
        # profiles read as not-started after a failed start().
        self._engine = engine
        self._color_profile = color_profile
        self._depth_profile = depth_profile
        # Kept for the runtime setters: the IR range to validate against and
        # the temporal spec to merge partial retunes into.
        self._model_info = load_model(resolved.model)
        self._temporal_spec = resolved.temporal_filter
        self._start_generation += 1
        self._streaming = True

    @property
    def device_info(self) -> Optional[OpenedDevice]:
        """The device this pipeline opened — model, serial_number, usb_port,
        usb_port_type / usb_speed (the negotiated USB 2.0 / 3.0 link), pid,
        firmware_version. Useful after an auto-detect start() to learn which
        camera was picked. None before start(). The camera model is separate,
        on Pipeline.intrinsics."""
        engine = self._engine
        if engine is None:
            return None
        pid = engine.pid
        return OpenedDevice(
            model=model_for_pid(pid) or "unknown",
            serial_number=engine.serial_number,
            usb_port=engine.usb_port,
            usb_port_type=self._usb_port_type,
            usb_speed=_usb_speed(self._usb_port_type),
            pid=pid,
            firmware_version=engine.firmware_version,
        )

    @property
    def color_profile(self) -> Optional[StreamProfile]:
        """The color stream's output profile (width, height, fps), or None if
        the active mode has no color stream. Available right after start().
        fps is the mode's nominal rate; an interleaved mode delivers half of
        it per stream — Pipeline.fps measures what actually arrives."""
        return self._color_profile

    @property
    def depth_profile(self) -> Optional[StreamProfile]:
        """The depth stream's output profile (width, height, fps), or None if
        the active mode has no depth stream. Available right after start().
        fps is the mode's nominal rate; an interleaved mode delivers half of
        it per stream — Pipeline.fps measures what actually arrives."""
        return self._depth_profile

    @property
    def intrinsics(self) -> Optional[Intrinsics]:
        """The camera model the active video mode maps to, as the device
        stores it. None before start(), or on an uncalibrated unit.

        Frames arrive rectified, so fx / fy / cx / cy describe them; K and
        D describe the raw sensor. Values are expressed at width x height,
        which a scale-down mode's depth does not match — PointCloud
        rescales internally, hand deprojection does not. See docs/api.md.
        """
        engine = self._engine
        native = engine.intrinsics if engine is not None else None
        if native is None or not native.valid:
            return None
        return Intrinsics(
            width=native.width, height=native.height,
            fx=native.fx, fy=native.fy, cx=native.cx, cy=native.cy,
            K=native.K, D=native.D, R=native.R, P=native.P,
            baseline_mm=native.baseline_mm)

    @property
    def depth_near_mm(self) -> int:
        """Near clip applied to the depth stream (mm)."""
        engine = self._engine
        return engine.depth_near_mm if engine is not None else 0

    @property
    def depth_far_mm(self) -> int:
        """Far clip applied to the depth stream (mm)."""
        engine = self._engine
        return engine.depth_far_mm if engine is not None else 0

    # --- camera controls: read the current value back from the camera ---
    # Each value is read from the camera on demand; the matching
    # Config.set_* writes it at open time.

    def get_auto_exposure(self) -> Optional[bool]:
        """True if auto-exposure is on, False if manual. None before start()
        or when the camera does not report the control."""
        engine = self._engine
        return engine.get_auto_exposure() if engine else None

    def get_exposure(self) -> Optional[int]:
        """Current exposure in camera register units. None before start() or
        when the camera does not report the control."""
        engine = self._engine
        return engine.get_exposure() if engine else None

    def get_auto_white_balance(self) -> Optional[bool]:
        """True if auto white balance is on, False if manual. None before
        start(), or on a monochrome model (G62 / R77), which has none."""
        engine = self._engine
        return engine.get_auto_white_balance() if engine else None

    def get_white_balance(self) -> Optional[int]:
        """Current white-balance value. None before start(), or on a
        monochrome model (G62 / R77), which has none."""
        engine = self._engine
        return engine.get_white_balance() if engine else None

    def get_power_line_frequency(self) -> Optional[int]:
        """Current anti-flicker mode (1 = 50 Hz, 2 = 60 Hz). None before
        start() or when the camera does not report the control."""
        engine = self._engine
        return engine.get_power_line_frequency() if engine else None

    def get_ir_value(self) -> Optional[int]:
        """Current IR projector intensity. None before start() or when the
        camera does not report the control."""
        engine = self._engine
        return engine.get_ir_value() if engine else None

    def get_ir_range(self) -> Optional[ControlRange]:
        """The valid IR projector range for the opened model, from its
        catalog (min, max, step, default). The catalog value is the
        model's qualified operating range; the register itself accepts
        values beyond it. None before start()."""
        info = self._model_info
        if info is None:
            return None
        ir = info.ir
        return ControlRange(ir.min, ir.max, 1, ir.default)

    def get_exposure_range(self) -> Optional[ControlRange]:
        """The (min, max, step, default) for the manual exposure value
        (register units), or None before start().

        The eYs3D modules share one fixed exposure register range on a
        signed log2 scale; some firmware misreports its UVC descriptor as
        an absolute-time range, so the fixed range is authoritative rather
        than whatever the device publishes.

        `default` repeats the minimum: the modules power on in
        auto-exposure and publish no manual default, and the minimum is the
        darkest setting the module has."""
        if self._engine is None:
            return None
        return ControlRange(_EXPOSURE_MIN, _EXPOSURE_MAX, 1, _EXPOSURE_MIN)

    def get_white_balance_range(self) -> Optional[ControlRange]:
        """The device-reported (min, max, step, default) for the manual
        white-balance value, or None if unavailable."""
        engine = self._engine
        r = engine.get_white_balance_range() if engine else None
        return ControlRange(*r) if r is not None else None

    # --- camera controls: adjust while streaming ---
    # Runtime counterparts of the Config.set_* methods: each writes the
    # camera immediately and survives a hot-plug reconnect. Raises
    # RuntimeError before start() or when the camera rejects the write.

    def _running_engine(self) -> CaptureEngine:
        # Snapshot once: a concurrent stop() nulling self._engine between the
        # check and the return would otherwise hand the caller None and turn
        # its .set_*() into AttributeError instead of the documented
        # RuntimeError.
        engine = self._engine
        if engine is None:
            raise RuntimeError("Pipeline not started")
        return engine

    def set_ir_value(self, value: int) -> None:
        """Set the IR projector intensity now. 0 turns the projector off;
        the range in between is model-specific (from the model's catalog,
        see get_ir_range()).

        -1 restores what start() would have chosen for the open mode: the
        catalog default where IR is the illumination — any depth mode, and
        any mode on a monochrome model — and off for a color-only mode on
        a color model, which keeps the dot pattern out of the image.

        Raises ValueError outside that range, RuntimeError before start() or
        when the camera rejects the write."""
        engine = self._running_engine()
        info = self._model_info
        if info is None:
            raise RuntimeError(
                "The opened camera has no catalog entry, so its IR range "
                "cannot be checked")
        ir = info.ir
        if value == -1:
            # The depth profile is None exactly when the mode carries no
            # depth.
            value = (ir.default
                     if self._depth_profile is not None or info.mono
                     else 0)
        if not (ir.min <= value <= ir.max):
            raise ValueError(
                f"ir_value {value} out of range [{ir.min}, {ir.max}] "
                f"for {info.model}")
        if not engine.set_ir_value(value):
            raise RuntimeError(f"The camera rejected ir_value={value}")

    def set_auto_exposure(self, enabled: bool) -> None:
        """Switch auto-exposure on (True) or off (False) now."""
        if not self._running_engine().set_auto_exposure(bool(enabled)):
            raise RuntimeError("The camera rejected the auto-exposure switch")

    def set_exposure(self, value: int) -> None:
        """Set a manual exposure value now; switches auto-exposure off first.

        The value is in camera register units and may be negative (the
        modules use a signed log2 scale); it is validated against the
        fixed exposure range — see get_exposure_range()."""
        engine = self._running_engine()
        r = self.get_exposure_range()
        if r is not None and not (r.min <= value <= r.max):
            raise ValueError(
                f"exposure {value} outside the exposure range "
                f"[{r.min}, {r.max}] (step {r.step})")
        if not engine.set_exposure(value):
            raise RuntimeError(f"The camera rejected exposure={value}")

    def set_auto_white_balance(self, enabled: bool) -> None:
        """Switch auto white balance on (True) or off (False) now. Not
        available on the monochrome models (G62 / R77): no color, no white
        balance."""
        engine = self._running_engine()
        info = self._model_info
        if info is not None and info.mono:
            raise ValueError(
                f"{info.model} is a monochrome camera; it has no white balance")
        if not engine.set_auto_white_balance(bool(enabled)):
            raise RuntimeError("The camera rejected the auto-white-balance switch")

    def set_white_balance(self, value: int) -> None:
        """Set a manual white-balance value now; switches auto white balance
        off first. Validated against the device-reported range — see
        get_white_balance_range(). Not available on the monochrome models
        (G62 / R77): no color, no white balance."""
        engine = self._running_engine()
        info = self._model_info
        if info is not None and info.mono:
            raise ValueError(
                f"{info.model} is a monochrome camera; it has no white balance")
        r = engine.get_white_balance_range()
        if r is not None and not (r[0] <= value <= r[1]):
            raise ValueError(
                f"white_balance {value} outside the device range "
                f"[{r[0]}, {r[1]}] (step {r[2]})")
        if value < 0:
            raise ValueError(f"white_balance must be >= 0, got {value}")
        if not engine.set_white_balance(value):
            raise RuntimeError(f"The camera rejected white_balance={value}")

    def set_power_line_frequency(self, mode: int) -> None:
        """Set the anti-flicker mode now: 1 = 50 Hz, 2 = 60 Hz (the values
        eYs3D cameras support; UVC's 0 = disabled and 3 = auto are not)."""
        if mode not in (1, 2):
            raise ValueError(
                f"power_line_frequency must be 1 (50 Hz) or 2 (60 Hz), got {mode}")
        if not self._running_engine().set_power_line_frequency(mode):
            raise RuntimeError(f"The camera rejected power_line_frequency={mode}")

    def set_temporal_filter(self, *, alpha: Optional[float] = None,
                            delta: Optional[int] = None,
                            persistence: Optional[int] = None) -> None:
        """Retune the temporal filter while streaming; parameters left None
        keep their current value. alpha is 0..1, delta 0..4095 and
        persistence 0..8 — the same bounds as TemporalFilter; anything else
        raises ValueError. Only available when the
        temporal filter was enabled at start() (the disparity stream is fixed
        at open time), and raises RuntimeError otherwise."""
        engine = self._running_engine()
        t = self._temporal_spec
        if t is None:
            raise RuntimeError(
                "The temporal filter was not enabled at start(); enable it via "
                "Config.with_filters(pyeys3d.TemporalFilter()) and restart")
        new = TemporalFilter(
            alpha=t.alpha if alpha is None else float(alpha),
            delta=t.delta if delta is None else int(delta),
            persistence=t.persistence if persistence is None else int(persistence),
        )
        # delta's ceiling is load-bearing: the kernels promote it to Q4 and
        # compare it in a uint16 lane.
        engine.set_temporal_params(new.alpha, new.delta, new.persistence)
        self._temporal_spec = new

    def hardware_reset(self) -> None:
        """Reset the camera over USB — a firmware-triggered detach and
        re-enumeration, equivalent to a physical replug. The device drops off
        the bus and comes back; the watchdog rebinds it by port identity and
        streaming resumes on its own, so `is_connected` goes False for the
        reconnect and `reconnect_count` then increments.

        Recovers an unresponsive camera without physical access. The firmware
        acknowledges the triggering writes unreliably (the link is already
        going away), so this reports nothing and never raises on a missing
        acknowledge. Requires a started pipeline."""
        self._running_engine().reset_usb()

    @property
    def is_connected(self) -> bool:
        """False while the watchdog is recovering from a USB drop."""
        engine = self._engine
        return engine.is_connected if engine is not None else False

    @property
    def reconnect_count(self) -> int:
        """How many times the device has been reopened after a USB drop."""
        engine = self._engine
        return engine.reconnect_count if engine is not None else 0

    @property
    def fps(self) -> StreamFps:
        """Rate at which frames reach the host, per stream (Hz).

        Measured at publish, so it is unaffected by how fast the caller
        reads: compare against depth_profile.fps to tell a capture
        shortfall from a slow application. Reads 0 after a second of
        silence.
        """
        engine = self._engine
        if engine is None:
            return StreamFps(0.0, 0.0)
        return StreamFps(engine.color_fps, engine.depth_fps)

    @property
    def quality_registers(self) -> QualityRegisters:
        """How the depth-quality register profile went: registers applied,
        registers the firmware would not hold, and whether the write is
        still in progress. The write starts a few seconds after depth
        does and reads 0 / 0 / True until it finishes. Before start(), on a
        mode without depth, and with set_depth_quality_registers(False), all
        three read 0 / 0 / False."""
        engine = self._engine
        if engine is None:
            return QualityRegisters(0, 0, False)
        return QualityRegisters(engine.quality_regs_ok,
                                engine.quality_regs_failed,
                                engine.quality_regs_pending)

    @property
    def frames_dropped(self) -> DroppedFrames:
        """Frames the camera produced but the host never received, per
        stream (detected as serial-number gaps). Consumer-side skips are
        not counted: wait_for_frames() always delivers the newest frame,
        so a slow consumer simply sees frame_number advance by more
        than one."""
        engine = self._engine
        if engine is None:
            return DroppedFrames(0, 0)
        return DroppedFrames(engine.dropped_color_frames,
                             engine.dropped_depth_frames)

    def stop(self) -> None:
        """Close the camera and release the device. Safe to call twice, and
        after a start() that raised."""
        # Detach before closing so two threads reaching stop() cannot both
        # call close(), which would join the same watchdog twice.
        engine, self._engine = self._engine, None
        if engine is not None:
            engine.close()
        self._streaming = False
        self._color_profile = None
        self._depth_profile = None
        self._model_info = None
        self._temporal_spec = None
        self._usb_port_type = 0

    def wait_for_frames(self, timeout_ms: int = 1000) -> Optional[FrameSet]:
        """Block for the next matched frame set, or None on timeout. Either
        half is None in a mode that carries only one stream. Raises
        RuntimeError before start()."""
        engine = self._engine
        if not self._streaming or engine is None:
            raise RuntimeError("Pipeline not started")
        result = engine.wait_for_frames(timeout_ms)
        if result is None:
            return None
        color, depth, right = result
        return FrameSet(color, depth, right)

    def poll_for_frames(self) -> Optional[FrameSet]:
        """Non-blocking: return the latest matched set if a newer one is
        ready, else None. Raises RuntimeError before start()."""
        engine = self._engine
        if not self._streaming or engine is None:
            raise RuntimeError("Pipeline not started")
        result = engine.wait_for_frames(0)
        if result is None:
            return None
        color, depth, right = result
        return FrameSet(color, depth, right)

    @property
    def is_streaming(self) -> bool:
        """True between start() and stop(). Stays True while the watchdog
        reconnects — is_connected is the one that goes False."""
        return self._streaming

    def __enter__(self) -> "Pipeline":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def __del__(self) -> None:
        # Routed through stop() so the close happens in a call that drops
        # the GIL for the second it takes; the native destructor does not,
        # and would stall every other Python thread at an arbitrary
        # collection point. Interpreter shutdown can already have taken what
        # this needs, and nothing can be reported from here.
        try:
            self.stop()
        except Exception:      # noqa: BLE001
            pass


__all__ = [
    "ControlRange",
    "DroppedFrames",
    "Frame",
    "FrameDomain",
    "FrameSet",
    "Intrinsics",
    "OpenedDevice",
    "Pipeline",
    "QualityRegisters",
    "StreamFps",
    "StreamProfile",
]
