// SIMD kernels with compile-time arch dispatch.
//
// aarch64 builds select the NEON intrinsic path; every other target uses
// the portable scalar implementation. Both ABIs mandate their respective
// 128-bit SIMD extensions (NEON on ARMv8, SSE2 on x86_64), so no runtime
// feature detection is needed.

#pragma once

#include <cstddef>
#include <cstdint>

namespace pyeys3d::simd {

// YUYV (4:2:2 packed: Y0 U Y1 V) → rgb8 (R G B R G B ...).
//
// BT.601 limited-range coefficients in 8.8 fixed point. The aarch64 NEON
// kernel processes 32 pixels per iteration (vld4q_u8 + vst3q_u8); the
// scalar kernel auto-vectorises to SSE2/AVX2 under GCC -O3 on x86_64.
// Both kernels produce identical output bytes.
//
// `src` is `w * h * 2` bytes (YUYV); `dst` is `w * h * 3` bytes (rgb8).
// `w` must be even (YUYV invariant); `h` ≥ 1. OpenMP parallelises across
// rows when available.
void yuyv_to_rgb8(const uint8_t* src, uint8_t* dst, int w, int h);

// Split-aware variant for wide-color L|R modes (G100+ 2560x720 wide
// YUYV, modes 22 / 25 / 26). The source raster is `(2 * half_w) * h * 2`
// bytes; each row's first `half_w` YUYV pairs are converted to
// `dst_left`, the next `half_w` pairs to `dst_right`. Both output
// buffers are `half_w * h * 3` bytes.
//
// Output bytes are identical to `yuyv_to_rgb8` followed by an in-order
// row slice.
void yuyv_to_rgb8_split(const uint8_t* src,
                        uint8_t* dst_left, uint8_t* dst_right,
                        int half_w, int h);

// ---------------------------------------------------------------------------
//   Monochrome fast path
// ---------------------------------------------------------------------------
// The G62 / R77 sensors are monochrome; their color stream carries luma
// only. Decoding straight to a single gray plane (MJPEG via TJPF_GRAY, or
// the Y bytes of YUYV) skips chroma upsampling and the YCbCr->RGB matrix,
// then a cheap replicate yields an exact-gray rgb8 frame (R == G == B).

// Extract the Y plane from a YUYV raster: gray[i] = src[2*i]. `src` is
// `w * h * 2` bytes, `gray` is `w * h` bytes.
void yuyv_extract_y(const uint8_t* src, uint8_t* gray, int w, int h);

// Replicate a `w * h` gray plane to rgb8 (`w * h * 3`), R = G = B = gray.
void gray_to_rgb8(const uint8_t* gray, uint8_t* dst, int w, int h);

// Split-aware replicate for wide L|R mono modes. `gray` is the wide plane
// (`(2 * half_w) * h` bytes); each row's first `half_w` grays go to
// `dst_left`, the next `half_w` to `dst_right` (each `half_w * h * 3`).
void gray_to_rgb8_split(const uint8_t* gray,
                        uint8_t* dst_left, uint8_t* dst_right,
                        int half_w, int h);

// ---------------------------------------------------------------------------
//   Point-cloud support
// ---------------------------------------------------------------------------

// Count the pixels in a depth row whose Z14-masked value is nonzero —
// the sizing pass of the two-pass point-cloud reprojection. NEON on
// aarch64 (16 pixels per iteration, batched accumulation to stay within
// the uint16 lane range); scalar elsewhere.
uint32_t count_valid_depth(const uint16_t* row, int w);

}  // namespace pyeys3d::simd
