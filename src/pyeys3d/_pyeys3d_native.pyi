"""Type stubs for the pyeys3d native extension (eSPDI capture engine).

Hand-maintained so IDEs and type checkers see the compiled API. The
user-facing wrappers (Config, Pipeline, filters, PointCloud) live in the
pure-Python layer and carry their own annotations.
"""
from __future__ import annotations

from enum import IntEnum
from typing import List, Optional, Tuple

import numpy as np

__version__: str

# Sentinel for OpenConfig.exposure_time ("unset"); exposure register units
# are signed, so -1 is a legal value and cannot mean unset.
EXPOSURE_UNSET: int


class FrameDomain(IntEnum):
    COLOR_RGB8: int
    DEPTH_MM: int
    DISPARITY_D11: int
    DISPARITY_Q4: int


class DeviceInfo:
    index: int
    pid: int
    vid: int
    serial_number: str
    device_node: str
    usb_port: str
    usb_port_type: int
    firmware_version: str
    def __repr__(self) -> str: ...


class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    @property
    def K(self) -> np.ndarray: ...   # 3x3 camera matrix (9,)
    @property
    def D(self) -> np.ndarray: ...   # rational polynomial (8,): k1 k2 p1 p2 k3 k4 k5 k6
    @property
    def R(self) -> np.ndarray: ...   # 3x3 rectification (9,)
    @property
    def P(self) -> np.ndarray: ...   # 3x4 rectified projection (12,)
    baseline_mm: float
    valid: bool


class Frame:
    domain: FrameDomain
    width: int
    height: int
    bytes_per_pixel: int
    frame_number: int
    hw_timestamp_us: int
    @property
    def timestamp(self) -> float: ...
    def get_data(self) -> np.ndarray:
        """Color: (H, W, 3) uint8 rgb8. Depth: (H, W) uint16 millimeters.

        A read-only view onto the frame's buffer, valid while the Frame is
        alive. Copy it to edit the pixels or to keep them past the frame."""
        ...
    # get_data_bgr is not declared here: the native class does not have
    # it. pyeys3d/__init__.py attaches it to this type on import, so it
    # exists on a Frame obtained through the package and not on one taken
    # from this module directly.


class Context:
    def __init__(self) -> None: ...
    def query_devices(self) -> List[DeviceInfo]: ...


class OpenConfig:
    """Resolved open descriptor (internal; built by Pipeline.start)."""
    def __init__(self) -> None: ...
    usb_port: str
    serial_number: str
    expected_pid: int
    color_w: int
    color_h: int
    color_fmt: int
    color_split_lr: bool
    is_mono: bool
    depth_w: int
    depth_h: int
    depth_dtype: int
    zd_index: int
    fps: int
    interleave: bool
    ir_value: int
    auto_exposure: int
    exposure_time: int
    auto_white_balance: int
    white_balance: int
    power_line_frequency: int
    filter_spatial: bool
    spatial_alpha: float
    spatial_delta: int
    spatial_magnitude: int
    spatial_holes_fill: int
    filter_temporal: bool
    temporal_alpha: float
    temporal_delta: int
    temporal_persistence: int
    filter_hole: bool
    hole_mode: int
    depth_near_mm: int
    depth_far_mm: int
    quality_regs: List[List[int]]


class CaptureEngine:
    """Native capture engine (internal; owned by Pipeline)."""
    def __init__(self) -> None: ...
    def open(self, cfg: OpenConfig) -> None: ...
    def start(self) -> None: ...
    def close(self) -> None: ...
    def wait_for_frames(
        self, timeout_ms: int = ...
    ) -> Optional[Tuple[Optional[Frame], Optional[Frame], Optional[Frame]]]:
        """(color, depth, right) or None on timeout. Colour is None in a
        depth-only mode, depth is None in a color-only mode, and right is
        None unless the mode splits L|R."""
        ...
    @property
    def split_color(self) -> bool: ...
    @property
    def pid(self) -> int: ...
    @property
    def serial_number(self) -> str: ...
    @property
    def usb_port(self) -> str: ...
    @property
    def firmware_version(self) -> str: ...
    def get_auto_exposure(self) -> Optional[bool]: ...
    def get_exposure(self) -> Optional[int]: ...
    def get_auto_white_balance(self) -> Optional[bool]: ...
    def get_white_balance(self) -> Optional[int]: ...
    def get_power_line_frequency(self) -> Optional[int]: ...
    def get_exposure_range(self) -> Optional[Tuple[int, int, int, int]]: ...
    def get_white_balance_range(self) -> Optional[Tuple[int, int, int, int]]: ...
    def set_ir_value(self, value: int) -> bool: ...
    def get_ir_value(self) -> Optional[int]: ...
    def set_auto_exposure(self, on: bool) -> bool: ...
    def set_exposure(self, value: int) -> bool: ...
    def set_auto_white_balance(self, on: bool) -> bool: ...
    def set_white_balance(self, value: int) -> bool: ...
    def set_power_line_frequency(self, mode: int) -> bool: ...
    def set_temporal_params(
        self, alpha: float, delta: int, persistence: int
    ) -> None: ...
    def reset_usb(self) -> bool: ...
    @property
    def intrinsics(self) -> Intrinsics: ...
    @property
    def depth_near_mm(self) -> int: ...
    @property
    def depth_far_mm(self) -> int: ...
    @property
    def is_open(self) -> bool: ...
    @property
    def dropped_color_frames(self) -> int: ...
    @property
    def dropped_depth_frames(self) -> int: ...
    @property
    def color_fps(self) -> float: ...
    @property
    def depth_fps(self) -> float: ...
    @property
    def is_streaming(self) -> bool: ...
    @property
    def is_connected(self) -> bool: ...
    @property
    def reconnect_count(self) -> int: ...
    @property
    def quality_regs_ok(self) -> int: ...
    @property
    def quality_regs_failed(self) -> int: ...
    @property
    def quality_regs_pending(self) -> bool: ...


class PointCloud:
    """Native reprojector (use the pyeys3d.PointCloud Python wrapper)."""
    def __init__(self, engine: CaptureEngine) -> None: ...
    def calculate(
        self, depth: Frame, color: Optional[Frame] = ...
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]: ...
