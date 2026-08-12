"""pyeys3d — direct-eSPDI Python wrapper for eYs3D stereo depth cameras.

Supports G100+, R77, and G62 in YUYV and MJPEG color modes, with a full
post-process filter chain (spatial / temporal / hole-filling) and
point-cloud reprojection through a `Pipeline` / `Config` / `Frame` API.

Examples live in the `examples/` directory of the source tree.

Quick start:

    import pyeys3d as ey

    pipeline = ey.Pipeline()
    config = ey.Config().enable_device("G100P", mode_id=1)
    pipeline.start(config)

    while True:
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        if frames is None:
            continue
        color = frames.get_color_frame().get_data()    # (H, W, 3) uint8 rgb8
        depth = frames.get_depth_frame().get_data()    # (H, W) uint16 mm
        ...
"""

import os as _os

if _os.name == "nt":
    # The wheel ships eSPDI_DM.dll next to the binding; Windows does not
    # search the importing module's directory for dependent DLLs, so
    # register it explicitly before the native import. add_dll_directory is
    # Windows-only, so a Linux mypy run cannot see it; the guard makes the
    # call safe and the ignore keeps the cross-platform check quiet.
    _os.add_dll_directory(  # type: ignore[attr-defined]
        _os.path.dirname(_os.path.abspath(__file__)))

from ._pyeys3d_native import (
    Frame,
    FrameDomain,
    __version__,
)


def _frame_get_data_bgr(self):
    """Color: a fresh, writable (H, W, 3) uint8 bgr8 copy of get_data().

    Depth frames are returned unchanged, which means they keep get_data()'s
    read-only view onto the frame buffer — copy before editing those."""
    d = self.get_data()
    return d[:, :, ::-1].copy() if d.ndim == 3 else d

Frame.get_data_bgr = _frame_get_data_bgr   # type: ignore[attr-defined]
del _frame_get_data_bgr
from .config import Config
from .context import Context, DeviceInfo
from .filters import (
    HoleFill,
    HoleFillingFilter,
    SpatialFilter,
    TemporalFilter,
)
from .modes import (
    ModeDescriptor,
    ModelInfo,
    load_catalog,
    load_model,
    supported_models,
)
from .pipeline import (
    ControlRange,
    DroppedFrames,
    FrameSet,
    Intrinsics,
    OpenedDevice,
    Pipeline,
    QualityRegisters,
    StreamFps,
    StreamProfile,
)
from .point_cloud import PointCloud
from .viz import Colorizer

__all__ = [
    "__version__",
    # core
    "Config",
    "Context",
    "DeviceInfo",
    "Frame",
    "FrameDomain",
    "FrameSet",
    "Pipeline",
    # pipeline value types
    "ControlRange",
    "DroppedFrames",
    "Intrinsics",
    "OpenedDevice",
    "QualityRegisters",
    "StreamFps",
    "StreamProfile",
    # filters + point cloud
    "SpatialFilter",
    "TemporalFilter",
    "HoleFillingFilter",
    "HoleFill",
    "PointCloud",
    "Colorizer",
    # mode catalog
    "ModeDescriptor",
    "ModelInfo",
    "load_catalog",
    "load_model",
    "supported_models",
]
