"""Point-cloud reprojection.

    pc = ey.PointCloud(pipeline)
    verts, colors = pc.calculate(depth_frame, frames.get_color_frame())
    # verts:  (N, 3) float32 meters, optical frame (X right, Y down, Z fwd)
    # colors: (N, 3) uint8, or None for an XYZ-only cloud
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional, Tuple

import numpy as np

from ._pyeys3d_native import Frame
from ._pyeys3d_native import PointCloud as _NativePointCloud

if TYPE_CHECKING:                       # a type here, not a dependency
    from .pipeline import Pipeline


class PointCloud:
    """Reproject a depth frame to a point cloud.

    Construct from a started Pipeline (it reads the device calibration):

        pc = ey.PointCloud(pipeline)
        verts, colors = pc.calculate(depth_frame, color_frame)

    Pass the color frame for an XYZRGB cloud, or omit it for an XYZ-only
    cloud (skips per-vertex color sampling, lighter on CPU and memory).
    `verts` is (N, 3) float32 meters in the optical frame (X right, Y down,
    Z forward); `colors` is (N, 3) uint8 or None.

    Raises RuntimeError if the pipeline is not started, or if the unit
    reports no calibration — an uncalibrated camera still delivers depth.

    Tied to the start() it was built against: the calibration is per video
    mode, so calculate() refuses after the pipeline has been restarted.
    Build a new one each time.
    """

    def __init__(self, pipeline: "Pipeline") -> None:
        # Accept a Pipeline (preferred) or the underlying engine directly.
        engine: Any = getattr(pipeline, "_engine", pipeline)
        if engine is None:
            raise RuntimeError(
                "PointCloud needs a started Pipeline; call pipeline.start() first."
            )
        # Only a Pipeline maps an uncalibrated unit to None; a bare engine
        # is passed straight through. Refused here rather than letting the
        # native reprojector fault on absent calibration.
        if hasattr(pipeline, "intrinsics") and pipeline.intrinsics is None:
            raise RuntimeError(
                "PointCloud needs the device calibration, but this camera "
                "reports none (uncalibrated unit)."
            )
        # The native object copies the calibration at construction, so
        # record the open it belongs to.
        self._pipeline = (pipeline if hasattr(pipeline, "_start_generation")
                          else None)
        self._generation = getattr(pipeline, "_start_generation", 0)
        self._native = _NativePointCloud(engine)

    def calculate(self, depth: Frame, color: Optional[Frame] = None
                  ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Reproject one depth frame. Returns (verts, colors).

        `depth` must be a millimeter depth frame; a disparity or color frame
        raises RuntimeError. N is the number of non-zero depth pixels, so it
        varies frame to frame and is not H*W — an all-hole frame returns an
        empty (0, 3) array rather than raising. `colors` is None unless a
        color frame is passed. Both arrays are freshly allocated and
        writable on every call, so they outlive the frames they came from.
        """
        if (self._pipeline is not None
                and self._pipeline._start_generation != self._generation):
            raise RuntimeError(
                "This PointCloud belongs to an earlier start(): the pipeline "
                "has been restarted since, and the calibration is per video "
                "mode, so reprojecting with this one would be silently wrong. "
                "Build a new PointCloud after every start().")
        return self._native.calculate(depth, color)


__all__ = ["PointCloud"]
