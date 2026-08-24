"""Firmware depth-quality register profiles.

Profiles live in :mod:`pyeys3d.quality.DM_Quality_Cfg`, one file per
firmware part number (`<PART>_DM_Quality_Register_Setting.cfg`);
`load_profile` maps a camera model onto its part number, falling back to
the DEFAULT profile for models without a dedicated one. Each file holds
one register per line as `address,mask,value` hex
triples — a read-modify-write on the masked bits of the firmware's
2-byte-address / 1-byte-value register space. `#` and `;` start comments.

The pipeline applies the resolved profile in the background once the
stream settles (a few seconds after depth streaming starts) — the
firmware accepts these writes only while the depth pipeline is running —
and re-applies it after a USB reconnect, since the firmware power-cycles
register state on re-enumeration.
"""
from __future__ import annotations

from functools import lru_cache
from importlib import resources
from typing import Optional, Tuple

RegisterTriple = Tuple[int, int, int]  # (address, mask, value)

# Resource subpackage and the shared filename suffix the profiles carry.
_CFG_PKG = __name__ + ".DM_Quality_Cfg"
_CFG_SUFFIX = "_DM_Quality_Register_Setting.cfg"

# Camera model -> firmware part number; a model absent here resolves to
# DEFAULT.
_MODEL_PART = {
    "G100P": "YX80362",
    "R77": "YX8072",
    "G62": "YX8081",
}


def parse_profile(text: str, origin: str) -> Tuple[RegisterTriple, ...]:
    """Parse profile text into (address, mask, value) triples.

    Raises ValueError naming `origin` and the line on malformed input, so
    a typo in a user-supplied profile fails at start() rather than as a
    skipped register."""
    triples = []
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line[0] in "#;":
            continue
        parts = [p.strip() for p in line.split(",")]
        try:
            if len(parts) != 3:
                raise ValueError
            addr, mask, value = (int(p, 16) for p in parts)
        except ValueError:
            raise ValueError(
                f"{origin}:{lineno}: expected 'address,mask,value' hex "
                f"triple, got {raw!r}") from None
        if not (0 <= addr <= 0xFFFF and 0 <= mask <= 0xFF
                and 0 <= value <= 0xFF):
            raise ValueError(
                f"{origin}:{lineno}: value out of range in {raw!r} "
                f"(address 16-bit, mask/value 8-bit)")
        triples.append((addr, mask, value))
    return tuple(triples)


def _read_cfg(name: str) -> str:
    # importlib.resources.files() arrived in Python 3.9; fall back to the
    # older read_text() API on 3.8 (present there, deprecated only later).
    if hasattr(resources, "files"):
        return resources.files(_CFG_PKG).joinpath(name).read_text(
            encoding="utf-8")
    return resources.read_text(_CFG_PKG, name, encoding="utf-8")


@lru_cache(maxsize=None)
def load_profile(model: str) -> Tuple[RegisterTriple, ...]:
    """Return the bundled profile for `model`, falling back to DEFAULT."""
    parts: Tuple[Optional[str], ...] = (_MODEL_PART.get(model), "DEFAULT")
    for part in parts:
        if part is None:
            continue
        name = part + _CFG_SUFFIX
        try:
            text = _read_cfg(name)
        except FileNotFoundError:
            continue
        return parse_profile(text, name)
    raise FileNotFoundError(
        f"no bundled depth-quality profile for {model!r} and the DEFAULT "
        f"profile is missing")


__all__ = ["RegisterTriple", "load_profile", "parse_profile"]
