"""Depth visualization helpers.

`Colorizer` maps a depth frame to an rgb8 image for display, using OpenCV
(a fast C++ scale + colormap).
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Union

import numpy as np

from ._pyeys3d_native import Frame

if TYPE_CHECKING:                       # a type here, not a dependency
    from .pipeline import Pipeline


class Colorizer:
    """Reusable converter that maps a depth frame to an rgb8 image.

        colorizer = ey.Colorizer(pipeline)        # build once
        rgb = colorizer.colorize(frames.get_depth_frame())   # (H, W, 3) uint8

    Depth is mapped over a fixed ``[min_mm, max_mm]`` range so colors stay
    stable frame to frame, and holes (depth == 0) render black. Passing the
    pipeline takes the range from its depth clip (``depth_near_mm`` /
    ``depth_far_mm``); ``min_mm`` / ``max_mm`` override it. With no range at
    all the scale runs from 0 to the frame's own maximum, which flickers as
    the scene changes. ``mode='grayscale'`` renders a gray ramp instead of
    the default JET color map.

    Output of ``colorize()`` is rgb8; use ``colorize_bgr()`` to get bgr8
    directly for ``cv2.imshow``. Needs OpenCV (``pip install opencv-python``).

    Raises ValueError for an unknown ``mode``, RuntimeError when handed a
    pipeline that is not started, and ImportError when OpenCV is absent.
    """

    def __init__(self, pipeline: "Optional[Pipeline]" = None, *,
                 mode: str = "color",
                 min_mm: Optional[int] = None, max_mm: Optional[int] = None) -> None:
        if mode not in ("color", "grayscale"):
            raise ValueError(f"mode must be 'color' or 'grayscale', got {mode!r}")
        if pipeline is not None:
            if min_mm is None:
                min_mm = pipeline.depth_near_mm
            if max_mm is None:
                max_mm = pipeline.depth_far_mm
            if pipeline.depth_far_mm == 0:
                # Checked against the pipeline, not against max_mm: an
                # explicit max_mm would otherwise hide a pipeline that is
                # not started, and min_mm would silently come back 0.
                raise RuntimeError(
                    "Colorizer(pipeline) needs a started pipeline; call "
                    "pipeline.start() first, or construct Colorizer without "
                    "a pipeline and pass min_mm/max_mm.")
        self.mode = mode
        self.min_mm = int(min_mm) if min_mm is not None else 0
        self.max_mm = int(max_mm) if max_mm is not None else 0
        # An inverted range is a mistake, not a request for autoscaling,
        # which is what colorize() would otherwise quietly fall back to.
        if self.max_mm and self.max_mm <= self.min_mm:
            raise ValueError(
                f"max_mm ({self.max_mm}) must be above min_mm "
                f"({self.min_mm})")
        try:
            import cv2
        except ImportError as exc:
            raise ImportError(
                "Colorizer needs OpenCV: pip install opencv-python") from exc
        self._cv2 = cv2
        # JET as an RGB-ordered 256-LUT so applyColorMap emits rgb8 directly.
        jet = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1),
                                cv2.COLORMAP_JET)            # (256, 1, 3) BGR
        self._lut = cv2.cvtColor(jet, cv2.COLOR_BGR2RGB)

    def colorize(self, depth: Union[Frame, np.ndarray]) -> np.ndarray:
        """Map a depth frame, or a (H, W) uint16 millimeter array, to a
        fresh (H, W, 3) uint8 rgb8 image. Holes (depth == 0) render black.
        With no configured range the frame's own maximum sets the top of the
        scale, which flickers as the scene changes. The result is writable
        and independent of the frame's buffer."""
        d = depth.get_data() if isinstance(depth, Frame) else depth
        lo = self.min_mm
        hi = self.max_mm if self.max_mm > lo else int(d.max())
        if hi <= lo:
            hi = lo + 1
        scale = 255.0 / (hi - lo)
        # Clamped first: convertScaleAbs takes the absolute value, so a
        # sample below lo would fold back into the scale and paint a near
        # object the color of a far one.
        u8 = self._cv2.convertScaleAbs(np.maximum(d, lo),
                                       alpha=scale, beta=-lo * scale)
        if self.mode == "grayscale":
            rgb = self._cv2.cvtColor(u8, self._cv2.COLOR_GRAY2RGB)
        else:
            rgb = self._cv2.applyColorMap(u8, self._lut)
        rgb[d == 0] = 0
        return rgb

    def colorize_bgr(self, depth: Union[Frame, np.ndarray]) -> np.ndarray:
        """Like colorize() but returns bgr8 for direct cv2.imshow use."""
        rgb = self.colorize(depth)
        return rgb[:, :, ::-1].copy()


__all__ = ["Colorizer"]
