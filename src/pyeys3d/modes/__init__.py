"""Per-model camera catalog.

Each YAML in this package is the single source of truth for one model: its
USB PID, sensor type (mono / color), IR range and default, default depth
clip, signature mode, and the full video-mode table. Adding a new camera is
a matter of dropping in a new YAML — no native rebuild. The Python loader
normalizes each file into a `ModelInfo` (with its `ModeDescriptor` table)
that `pyeys3d.Config` and `Pipeline` resolve against.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from importlib import resources
from typing import Dict, Optional, Tuple

import yaml


@dataclass(frozen=True)
class StreamSpec:
    width: int
    height: int


@dataclass(frozen=True)
class ColorSpec(StreamSpec):
    fmt: int          # 0 = YUYV, 1 = MJPEG
    split_lr: bool    # True when one wire frame packs L|R side-by-side


@dataclass(frozen=True)
class DepthSpec(StreamSpec):
    dtype: int        # eSPDI APC_DEPTH_DATA_* code


@dataclass(frozen=True)
class ModeDescriptor:
    mode_id: int
    name: str
    color: ColorSpec
    depth: DepthSpec
    zd_index: int
    fps: int
    interleave: bool
    usb: int          # USB port type the mode requires (2 or 3)

    @property
    def has_color(self) -> bool:
        return self.color.width > 0 and self.color.height > 0

    @property
    def has_depth(self) -> bool:
        return self.depth.width > 0 and self.depth.height > 0


@dataclass(frozen=True)
class IrRange:
    min: int
    max: int
    default: int


@dataclass(frozen=True)
class ModelInfo:
    """Everything model-specific, loaded from the model's YAML."""
    model: str
    pid: int
    mono: bool                 # monochrome sensor -> luma-only decode path
    ir: IrRange
    depth_near_mm: int         # default depth range when the user sets none
    depth_far_mm: int
    signature_mode: Dict[int, int]   # USB port type -> default mode
    modes: Dict[int, ModeDescriptor]

    def signature_for(self, usb_type: Optional[int]) -> int:
        """Default mode for a USB port type (2 or 3). Falls back to the
        USB 3 entry, else any entry, when the type is unknown or absent."""
        if usb_type in self.signature_mode:
            return self.signature_mode[usb_type]
        if 3 in self.signature_mode:
            return self.signature_mode[3]
        return next(iter(self.signature_mode.values()))


_SUPPORTED_MODELS = ("G100P", "R77", "G62")


def _parse_mode(mid: int, entry: dict) -> ModeDescriptor:
    return ModeDescriptor(
        mode_id=mid,
        name=entry["name"],
        color=ColorSpec(
            width=int(entry["color"]["w"]),
            height=int(entry["color"]["h"]),
            fmt=int(entry["color"]["fmt"]),
            split_lr=bool(entry["color"]["split_lr"]),
        ),
        depth=DepthSpec(
            width=int(entry["depth"]["w"]),
            height=int(entry["depth"]["h"]),
            dtype=int(entry["depth"]["dtype"]),
        ),
        zd_index=int(entry["zd_index"]),
        fps=int(entry["fps"]),
        interleave=bool(entry["interleave"]),
        usb=int(entry["usb"]),
    )


def _read_model_yaml(name: str) -> str:
    # importlib.resources.files() arrived in Python 3.9; fall back to the
    # older read_text() API on 3.8 (present there, deprecated only later).
    if hasattr(resources, "files"):
        return resources.files(__name__).joinpath(name).read_text(
            encoding="utf-8")
    return resources.read_text(__name__, name, encoding="utf-8")


@lru_cache(maxsize=None)
def load_model(model: str) -> ModelInfo:
    """Return the full `ModelInfo` (camera parameters + mode table)."""
    if model not in _SUPPORTED_MODELS:
        raise ValueError(
            f"Unsupported model {model!r}. Supported: {_SUPPORTED_MODELS}"
        )
    raw = yaml.safe_load(_read_model_yaml(f"{model}.yaml"))
    if raw.get("model") != model:
        raise RuntimeError(
            f"{model}.yaml declares model={raw.get('model')!r}; expected {model!r}"
        )
    modes = {int(mid): _parse_mode(int(mid), entry)
             for mid, entry in raw.get("modes", {}).items()}
    rng = raw["depth_range_mm"]
    return ModelInfo(
        model=model,
        pid=int(raw["pid"]),
        mono=bool(raw["mono"]),
        ir=IrRange(min=int(raw["ir"]["min"]), max=int(raw["ir"]["max"]),
                   default=int(raw["ir"]["default"])),
        depth_near_mm=int(rng["near"]),
        depth_far_mm=int(rng["far"]),
        signature_mode={int(k): int(v)
                        for k, v in raw["signature_mode"].items()},
        modes=modes,
    )


def load_catalog(model: str) -> Dict[int, ModeDescriptor]:
    """Return {mode_id: ModeDescriptor} for the named model, keyed by the
    mode id the camera itself uses. Raises ValueError for an unknown
    model."""
    return load_model(model).modes


def model_for_pid(pid: int) -> Optional[str]:
    """Map a USB product id to a model name, or None if unrecognized."""
    for m in _SUPPORTED_MODELS:
        if load_model(m).pid == pid:
            return m
    return None


def pid_for_model(model: str) -> int:
    """The USB product id declared by the model's YAML."""
    return load_model(model).pid


def supported_models() -> Tuple[str, ...]:
    """The model names this build has a mode catalog for. Any other eYs3D
    unit on the bus enumerates as "unknown" and cannot be opened."""
    return _SUPPORTED_MODELS


__all__ = [
    "ColorSpec",
    "DepthSpec",
    "IrRange",
    "ModeDescriptor",
    "ModelInfo",
    "StreamSpec",
    "load_catalog",
    "load_model",
    "model_for_pid",
    "pid_for_model",
    "supported_models",
]
