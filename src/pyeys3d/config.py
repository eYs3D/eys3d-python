"""Declarative pipeline configuration.

The declarative form keeps validation on the Python side; the native
CaptureEngine receives a resolved descriptor at `Pipeline.start()` time.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from typing import Optional, Tuple, Union

from .filters import HoleFillingFilter, SpatialFilter, TemporalFilter
from .modes import ModeDescriptor, load_catalog, load_model, supported_models
from .quality import RegisterTriple, load_profile, parse_profile

# The eYs3D modules share one fixed manual-exposure register range on a
# signed log2 scale (UVC). It does not vary by model, so it is a constant
# rather than a per-model catalog value. Pipeline imports it for the
# runtime setter and for get_exposure_range(), so the value a caller is
# held to is the same before start() and while streaming.
_EXPOSURE_MIN, _EXPOSURE_MAX = -13, 3


@dataclass
class ResolvedConfig:
    """The validated, frozen configuration handed to CaptureEngine. IR, depth
    clip and the mono flag are already resolved to concrete per-model values;
    the engine carries no model knowledge of its own."""
    model: str
    mode: ModeDescriptor
    mono: bool = False                     # monochrome sensor -> luma decode
    ir_value: int = 0                      # resolved (model default applied)
    enable_auto_exposure: Optional[bool] = None
    exposure_time: Optional[int] = None    # None = leave as-is (units signed)
    enable_auto_white_balance: Optional[bool] = None
    white_balance: int = -1                # -1 = leave as-is
    power_line_frequency: int = -1         # -1 = leave as-is
    spatial_filter: Optional[SpatialFilter] = None
    temporal_filter: Optional[TemporalFilter] = None
    hole_filling_filter: Optional[HoleFillingFilter] = None
    depth_near_mm: int = 0                 # resolved (model default applied)
    depth_far_mm: int = 0
    # Firmware depth-quality register profile as (address, mask, value)
    # triples; empty when disabled or the mode has no depth stream.
    quality_registers: Tuple[RegisterTriple, ...] = ()


class Config:
    """Declarative configuration object.

    With no arguments, Pipeline.start() auto-detects the connected camera
    and uses its signature mode (the model's default mode):

        pipeline.start(pyeys3d.Config())

    Or name a model (signature mode is implied when mode_id is omitted):

        config = pyeys3d.Config().enable_device("G100P")

    Or pin a specific device and mode for multi-camera setups:

        config = pyeys3d.Config().enable_device("G100P", mode_id=1,
                                                usb_port="2-2:1.0")
    """

    def __init__(self) -> None:
        self._model: Optional[str] = None
        self._mode_id: Optional[int] = None
        self._serial_number: str = ""
        self._usb_port: str = ""
        self._ir_value: int = -1
        self._ae: Optional[bool] = None
        self._exposure: Optional[int] = None
        self._awb: Optional[bool] = None
        self._wb: int = -1
        self._power_line_freq: int = -1
        self._spatial: Optional[SpatialFilter] = None
        self._temporal: Optional[TemporalFilter] = None
        self._hole: Optional[HoleFillingFilter] = None
        self._depth_min_mm: int = -1
        self._depth_max_mm: int = -1
        # True = bundled per-model profile, False = leave firmware
        # defaults, str = path to a custom profile file.
        self._quality_registers: Union[bool, str] = True

    # ---------------- device selection ----------------

    def enable_device(
        self,
        model: Optional[str] = None,
        mode_id: Optional[int] = None,
        *,
        serial_number: str = "",
        usb_port: str = "",
    ) -> "Config":
        """Select the camera and video mode.

        Everything is optional. `model` omitted (None) lets Pipeline.start()
        auto-detect the connected camera; `mode_id` omitted uses that model's
        signature mode. `serial_number` (substring match) and `usb_port`
        (exact match) pin a specific unit when several are connected; given
        both, only a camera matching both is accepted, and selection fails
        with the candidate list if none does.
        """
        if model is not None and model not in supported_models():
            raise ValueError(
                f"Unsupported model {model!r}. Supported: {supported_models()}"
            )
        if model is not None and mode_id is not None:
            catalog = load_catalog(model)
            if mode_id not in catalog:
                valid = sorted(catalog.keys())
                raise ValueError(
                    f"mode_id {mode_id} not in {model} catalog. Valid ids: {valid}"
                )
        self._model = model
        self._mode_id = mode_id
        self._serial_number = serial_number
        self._usb_port = usb_port
        return self

    # ---------------- camera controls ----------------
    # Each control is written to the camera at start() only when set here;
    # left unset, the camera keeps its own value. After start(), the current
    # value is read back with the matching Pipeline.get_* accessor.

    def set_ir_value(self, value: int) -> "Config":
        """Set IR projector intensity; 0 turns the projector off. The valid
        range is model-specific (listed in the model's YAML, e.g. G62 0-96,
        G100+/R77 0-6) and is checked against the model at start().

        -1 (default) picks per mode: the model's cataloged default when the
        mode has depth (stereo matching needs the projector) or the module is
        monochrome (G62 / R77 sensors need IR as illumination even for
        color), and off for a color-only mode on a color sensor (keeps the
        dot pattern out of the color image)."""
        if value < -1:
            raise ValueError(f"ir_value must be -1 or >= 0, got {value}")
        self._ir_value = value
        return self

    def set_auto_exposure(self, enabled: bool) -> "Config":
        """Turn auto-exposure on (True) or off (False). With it off, set the
        manual value via set_exposure()."""
        self._ae = enabled
        return self

    def set_exposure(self, value: int) -> "Config":
        """Set a manual exposure value; this turns auto-exposure off.

        The value is in camera register units and may be negative — the
        modules use a signed log2 scale (e.g. -13 ≈ 1/8192 s) over one
        fixed range, reported after start() by
        Pipeline.get_exposure_range()."""
        value = int(value)
        if not (_EXPOSURE_MIN <= value <= _EXPOSURE_MAX):
            raise ValueError(
                f"exposure {value} outside the exposure range "
                f"[{_EXPOSURE_MIN}, {_EXPOSURE_MAX}]")
        self._exposure = value
        return self

    def set_auto_white_balance(self, enabled: bool) -> "Config":
        """Turn auto white balance on (True) or off (False). With it off, set
        the manual value via set_white_balance()."""
        self._awb = enabled
        return self

    def set_white_balance(self, value: int) -> "Config":
        """Set a manual white-balance value (camera register units); this turns
        auto white balance off (manual mode)."""
        if value < 0:
            raise ValueError(f"white_balance must be >= 0, got {value}")
        self._wb = value
        return self

    def set_power_line_frequency(self, mode: int) -> "Config":
        """Anti-flicker: sync exposure to the mains so lighting flicker does
        not band the image. 1 = 50 Hz, 2 = 60 Hz. (UVC also defines 0 =
        disabled and 3 = auto, but eYs3D cameras do not support them.)"""
        if mode not in (1, 2):
            raise ValueError(
                f"power_line_frequency must be 1 (50 Hz) or 2 (60 Hz), got {mode}")
        self._power_line_freq = mode
        return self

    def with_filters(self, *filters: object) -> "Config":
        """Attach the depth post-process chain.

        Accepts any of SpatialFilter / TemporalFilter / HoleFillingFilter;
        a second filter of the same kind replaces the first, and any other
        type raises TypeError. The pipeline runs them natively on the
        depth thread and always delivers depth in millimeters; the
        disparity stream and ZD conversion are handled internally. With no
        filters the firmware millimeter fast path is used.

            cfg.with_filters(ey.SpatialFilter(), ey.TemporalFilter(),
                             ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND))

        The chain order is fixed (spatial -> temporal -> hole filling)
        regardless of argument order.
        """
        for f in filters:
            if isinstance(f, SpatialFilter):
                self._spatial = f
            elif isinstance(f, TemporalFilter):
                self._temporal = f
            elif isinstance(f, HoleFillingFilter):
                self._hole = f
            else:
                raise TypeError(
                    f"with_filters accepts SpatialFilter / TemporalFilter / "
                    f"HoleFillingFilter, got {type(f).__name__}"
                )
        return self

    def set_depth_range(self, near_mm: int = -1, far_mm: int = -1) -> "Config":
        """Override the near/far depth range in millimeters. -1 keeps the
        per-model default (G100+ 250-1900, R77 200-1500, G62 100-1500)."""
        if near_mm < -1 or far_mm < -1:
            raise ValueError(
                f"depth range must be -1 (default) or >= 0 mm, got "
                f"({near_mm}, {far_mm})")
        if near_mm >= 0 and far_mm >= 0 and near_mm >= far_mm:
            raise ValueError(
                f"depth range needs near_mm < far_mm, got "
                f"({near_mm}, {far_mm})")
        if far_mm > 16383:            # Z14 depth is 14-bit: 2^14 - 1 mm (~16.4 m)
            raise ValueError(
                f"far_mm {far_mm} exceeds the 16383 mm depth data limit (Z14)")
        self._depth_min_mm = near_mm
        self._depth_max_mm = far_mm
        return self

    def set_depth_quality_registers(self, source: Union[bool, str] = True) -> "Config":
        """Firmware depth-quality register profile, applied in the
        background once depth streaming starts and settles (a few
        seconds in), and re-applied after a USB reconnect.

        True (default) applies the model's bundled profile (under
        `pyeys3d/quality/DM_Quality_Cfg/`); False leaves the firmware
        defaults untouched; a path applies a custom profile file (one
        `address,mask,value` hex triple per line, `#` comments). Ignored
        by modes without a depth stream. A path that cannot be read or
        parsed raises ValueError from Pipeline.start(), not from here."""
        if not isinstance(source, (bool, str)):
            raise ValueError(
                f"source must be True, False, or a file path, got {source!r}")
        self._quality_registers = source
        return self

    # ---------------- read accessors (used by Pipeline auto-detection) ----

    @property
    def model(self) -> Optional[str]:
        return self._model

    @property
    def serial_number(self) -> str:
        return self._serial_number

    @property
    def usb_port(self) -> str:
        return self._usb_port

    # ---------------- resolution ----------------

    def _resolve(self, device_model: Optional[str] = None,
                 usb_type: Optional[int] = None) -> ResolvedConfig:
        """Validate and freeze. Called by `Pipeline.start()` internally.

        `device_model` supplies the model auto-detected from the connected
        camera when enable_device was called without one. `usb_type` is the
        connected link's USB port type (2 or 3); when no mode_id was given
        the mode falls back to the model's signature mode for that link.
        """
        model = self._model or device_model
        if model is None:
            raise RuntimeError(
                "No model set and none detected. Connect a camera, or call "
                "Config.enable_device(model=...)."
            )
        info = load_model(model)
        mode_id = self._mode_id if self._mode_id is not None \
            else info.signature_for(usb_type)
        if mode_id not in info.modes:
            valid = sorted(info.modes.keys())
            raise ValueError(
                f"mode_id {mode_id} not in {model} catalog. Valid ids: {valid}"
            )
        if info.mono and (self._wb >= 0 or self._awb is not None):
            raise ValueError(
                f"{model} is a monochrome camera; it has no white balance. "
                f"Drop set_white_balance() / set_auto_white_balance()."
            )
        mode = info.modes[mode_id]
        # Each mode is defined for one USB link: a USB 3 link cannot open a
        # USB 2 mode, nor the reverse. Fail here with a clear message rather
        # than a device-level rejection at open.
        #
        # 0 is a device that reported a port type nothing could make sense
        # of — a hub, or a failed descriptor read — as distinct from None,
        # which is a caller resolving with no device in hand. Only the
        # first is worth saying anything about: the mode chosen for an
        # unknown link (signature_for falls back to the USB 3 entry) may be
        # one the link cannot carry, and the camera then opens and delivers
        # nothing, which reads like dead hardware.
        if usb_type == 0:
            warnings.warn(
                f"{model}: the USB link type did not resolve, so mode "
                f"{mode_id} ({mode.name}) cannot be checked against it. It "
                f"needs a USB {mode.usb} link; on anything else the camera "
                f"opens but delivers no frames.",
                RuntimeWarning, stacklevel=3)
        if usb_type in (2, 3) and mode.usb != usb_type:
            if self._mode_id is not None:
                raise ValueError(
                    f"mode {mode_id} ({mode.name}) needs a USB {mode.usb} "
                    f"link, but the camera negotiated USB {usb_type}. Pick a "
                    f"USB {usb_type} mode, or omit mode_id for the signature "
                    f"mode."
                )
            # No mode was asked for, so this one came from the catalog's
            # own signature entry: naming mode_id in the message would point
            # at something the caller did not set. The catalog has no mode
            # for this link.
            links = sorted({m.usb for m in info.modes.values()})
            raise ValueError(
                f"{model} has no video mode for a USB {usb_type} link; its "
                f"catalog defines "
                f"{' and '.join('USB ' + str(u) for u in links)} modes only. "
                f"The camera reports a USB {usb_type} port; on a USB "
                f"{links[0]}-only module that number usually describes the "
                f"host port rather than the link it negotiated. Try another "
                f"port or cable."
            )
        # Unset IR and depth clip fall back to the model catalog here, so
        # the native engine carries no model knowledge. Unset IR means the
        # model default wherever IR is the illumination (a depth mode, or a
        # mono sensor) and off for color-only on a color sensor, which
        # keeps the dot pattern out of the image.
        if self._ir_value >= 0:
            ir = self._ir_value
            if not (info.ir.min <= ir <= info.ir.max):
                raise ValueError(
                    f"ir_value {ir} out of range [{info.ir.min}, "
                    f"{info.ir.max}] for {model}"
                )
        elif mode.has_depth or info.mono:
            ir = info.ir.default
        else:
            ir = 0
        depth_min = self._depth_min_mm if self._depth_min_mm >= 0 \
            else info.depth_near_mm
        depth_max = self._depth_max_mm if self._depth_max_mm >= 0 \
            else info.depth_far_mm
        # Re-check after the model defaults fill the unset side: a lone
        # minimum above the model's default far clip would zero every pixel.
        if depth_min >= depth_max:
            raise ValueError(
                f"depth range resolves to ({depth_min}, {depth_max}) mm for "
                f"{model}; near_mm must stay below far_mm")
        # The filter chain runs on the depth stream, so a mode without
        # depth has nothing to filter. Refused here rather than dropped
        # natively, so set_temporal_filter() cannot later report a filter
        # the caller asked for as one that was never enabled.
        if not mode.has_depth:
            asked = [n for n, f in (("SpatialFilter", self._spatial),
                                    ("TemporalFilter", self._temporal),
                                    ("HoleFillingFilter", self._hole)) if f]
            if asked:
                raise ValueError(
                    f"mode {mode_id} ({mode.name}) opens no depth stream, so "
                    f"{' and '.join(asked)} cannot run. Drop the filters, or "
                    f"pick a mode with depth.")
        quality_regs: Tuple[RegisterTriple, ...] = ()
        if mode.has_depth and self._quality_registers is not False:
            if self._quality_registers is True:
                quality_regs = load_profile(model)
            else:
                try:
                    with open(self._quality_registers, encoding="utf-8") as f:
                        text = f.read()
                except OSError as e:
                    raise ValueError(
                        f"custom quality profile {self._quality_registers!r} "
                        f"could not be read: {e}") from e
                quality_regs = parse_profile(text, self._quality_registers)
        return ResolvedConfig(
            model=model,
            mode=mode,
            mono=info.mono,
            ir_value=ir,
            enable_auto_exposure=self._ae,
            exposure_time=self._exposure,
            enable_auto_white_balance=self._awb,
            white_balance=self._wb,
            power_line_frequency=self._power_line_freq,
            spatial_filter=self._spatial,
            temporal_filter=self._temporal,
            hole_filling_filter=self._hole,
            depth_near_mm=depth_min,
            depth_far_mm=depth_max,
            quality_registers=quality_regs,
        )


__all__ = ["Config", "ResolvedConfig"]
