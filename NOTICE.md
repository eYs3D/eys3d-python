NOTICE
======

`pyeys3d` is © 2026 eYs3D Microelectronics Corp., released under
Apache-2.0 (see `LICENSE`).

## Python dependencies

Declared in `pyproject.toml`:

| Component | License |
|---|---|
| `numpy` | BSD-3-Clause |
| `pyyaml` | MIT |
| `pybind11` (build-time) | BSD-3-Clause |
| `scikit-build-core` (build-time) | Apache-2.0 |

## Bundled binaries

The eSPDI runtime — `eSPDI/libeSPDI_*.so` (Linux) and
`eSPDI/win_x64/eSPDI_DM.dll` (Windows) — is © eYs3D Microelectronics
Corp.

Linux wheels additionally bundle the Khronos OpenCL ICD loader
(`libOpenCL.so.1`), © The Khronos Group Inc., licensed under
Apache-2.0.

Linux wheels also bundle the GNU OpenMP runtime (`libgomp.so.1`), © Free
Software Foundation, Inc., licensed under GPL-3.0-or-later **with the GCC
Runtime Library Exception 3.1** — the exception under which it may be
distributed with a program that does not carry the GPL. The same wheels
link libstdc++ and libgcc statically, under the same exception. The
Windows wheel bundles neither: it uses the Visual C++ runtime the host
installs (see the README's install notes).
