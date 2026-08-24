"""Post-process filter specs.

The depth filter chain runs natively inside the pipeline's capture thread,
so it is declared up front rather than applied frame-by-frame in Python.
Filter specs are built up front and handed to the Config:

    spatial  = ey.SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0)
    temporal = ey.TemporalFilter(alpha=0.4, delta=20, persistence=3)
    holes    = ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND)

    config = (ey.Config()
              .enable_device("G100P")
              .with_filters(spatial, temporal, holes))

    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    depth  = frames.get_depth_frame().get_data()   # (H, W) uint16 mm, filtered

A spatial or temporal filter operates in the disparity domain; the pipeline
opens the device in 11-bit disparity mode for it and converts back to
millimeters internally. Hole filling operates in the millimeter domain.
With no filters the pipeline uses the firmware millimeter fast path.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class HoleFill(IntEnum):
    """Hole-filling strategy for HoleFillingFilter."""
    OFF = 0
    FROM_LEFT = 1          # carry the last valid value rightward
    FARTHEST_AROUND = 2    # farthest valid neighbour around the hole
    NEAREST_AROUND = 3     # nearest valid neighbour around the hole


def _check_range(name: str, value, low, high) -> None:
    """Refuse a value the native chain would silently clamp instead.

    The kernels clamp rather than reject, so an out-of-range setting would
    reach the device reinterpreted; the filter specs and the matching
    Pipeline.set_* hold callers to the same bounds.
    """
    if high is None:
        if value < low:
            raise ValueError(f"{name} must be >= {low}, got {value}")
        return
    if not (low <= value <= high):
        raise ValueError(f"{name} must be between {low} and {high}, "
                         f"got {value}")


@dataclass
class SpatialFilter:
    """Edge-preserving spatial IIR over the disparity image.

    alpha       smoothing strength 0..1 (1 = no smoothing).
    delta       edge threshold in disparity units; gaps above it are kept.
    magnitude   number of four-direction passes (1..5).
    holes_fill  max run length bridged from the last valid neighbour
                during a pass (0 = smoothing only).
    """
    alpha: float = 0.5
    delta: int = 20
    magnitude: int = 2
    holes_fill: int = 0

    def __post_init__(self) -> None:
        _check_range("alpha", self.alpha, 0.0, 1.0)
        _check_range("delta", self.delta, 1, 4095)
        _check_range("magnitude", self.magnitude, 1, 5)
        _check_range("holes_fill", self.holes_fill, 0, None)


@dataclass
class TemporalFilter:
    """Temporal IIR across frames in the disparity domain.

    alpha        blend weight of the current frame 0..1.
    delta        per-pixel gap (disparity units) above which the blend is
                 skipped (treated as a moving edge).
    persistence  how long a last-valid value is held over dropouts (0..8).
    """
    alpha: float = 0.4
    delta: int = 20
    persistence: int = 3

    def __post_init__(self) -> None:
        _check_range("alpha", self.alpha, 0.0, 1.0)
        _check_range("delta", self.delta, 0, 4095)
        _check_range("persistence", self.persistence, 0, 8)


@dataclass
class HoleFillingFilter:
    """Fill zero-depth holes in the millimeter image.

    mode  a HoleFill value (OFF / FROM_LEFT / FARTHEST_AROUND / NEAREST_AROUND).
          Plain ints 1/2/3 are also accepted.
    """
    mode: HoleFill = HoleFill.FARTHEST_AROUND

    def __post_init__(self) -> None:
        _check_range("mode", int(self.mode), 0, 3)


__all__ = [
    "HoleFill",
    "HoleFillingFilter",
    "SpatialFilter",
    "TemporalFilter",
]
