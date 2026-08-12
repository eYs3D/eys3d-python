#include "point_cloud.hpp"

#include "simd_kernels.hpp"

#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <thread>
#include <vector>

namespace pyeys3d {

namespace {

constexpr int kSerialSkipPixels = 8;   // 16-byte SN watermark in depth row 0
// Depth is Z14: the top two bits are firmware status, not millimetres.
constexpr uint16_t kDepthMask = 0x3FFF;

const uint16_t* as_u16(const Frame& f) {
    return reinterpret_cast<const uint16_t*>(f.data.data());
}

// Worker count for the two reprojection passes. Both are memory-bound
// rather than compute-bound, so the curve flattens early: measured against
// a live camera, four workers reach 2.4x at 640x480 and 1.5x at 1280x720,
// while going wider adds nothing to the color path and loses ground at
// 640x480 past six. PYEYS3D_PC_THREADS overrides it for a workload that
// measures otherwise, clamped to the hardware.
int pc_threads() {
    // Read once, as the other knobs are: a live getenv would race any
    // setenv on another thread, and this runs on every frame.
    static const int n = [] {
        const int hw = std::max(
            1, static_cast<int>(std::thread::hardware_concurrency()));
        if (const char* e = std::getenv("PYEYS3D_PC_THREADS")) {
            const int v = std::atoi(e);
            if (v > 0) return std::min(v, hw);
        }
        return std::min(4, hw);
    }();
    return n;
}

}  // namespace

// ===========================================================================
//   PointCloud
// ===========================================================================

PointCloud::PointCloud(const CaptureEngine& engine) : calib_(engine.calibration()) {
    if (!calib_.valid) {
        throw std::runtime_error(
            "PointCloud needs this camera's calibration, which the pipeline "
            "does not have: the unit is either uncalibrated or its "
            "calibration could not be read when the device was opened. The "
            "engine's log line from open() says which. Depth frames are "
            "unaffected.");
    }
}

PointCloudResult PointCloud::calculate(const Frame& depth, const Frame* color) {
    if (depth.domain != FrameDomain::DEPTH_MM) {
        throw std::runtime_error("PointCloud.calculate expects a DEPTH_MM depth frame.");
    }
    const int W = depth.width, H = depth.height;

    // Rectified intrinsics scaled to the depth resolution.
    const double ratio = (calib_.out_img_height > 0)
        ? static_cast<double>(H) / calib_.out_img_height : 1.0;
    const float fx = static_cast<float>(calib_.left.P[0] * ratio);
    const float fy = static_cast<float>(calib_.left.P[5] * ratio);
    const float cx = static_cast<float>(calib_.left.P[2] * ratio);
    const float cy = static_cast<float>(calib_.left.P[6] * ratio);

    PointCloudResult res;
    if (fx == 0.0f || fy == 0.0f) {
        return res;  // can't reproject
    }
    const float inv_fx = 1.0f / fx;
    const float inv_fy = 1.0f / fy;

    const bool want_rgb = color != nullptr
        && color->domain == FrameDomain::COLOR_RGB8
        && color->width > 0 && color->height > 0;

    const uint16_t* d = as_u16(depth);
    constexpr float kMmToM = 1.0f / 1000.0f;

    const int cw = want_rgb ? color->width : 0;
    const int ch = want_rgb ? color->height : 0;
    const uint8_t* cdata = want_rgb ? color->data.data() : nullptr;

    // thread_local working storage: capacity persists across calls and
    // concurrent calls on one PointCloud stay safe.
    thread_local std::vector<float> u_lut, v_lut;
    thread_local std::vector<int32_t> cu_off;
    thread_local std::vector<uint32_t> row_count, row_off;
    thread_local std::vector<float> xyz;
    thread_local std::vector<uint8_t> rgb;

    // The lookup tables are cheap to build (microseconds), so rebuild
    // them every call rather than track intrinsics/dimension changes.
    u_lut.resize(static_cast<size_t>(W));
    v_lut.resize(static_cast<size_t>(H));
    for (int u = 0; u < W; ++u) u_lut[u] = (static_cast<float>(u) - cx) * inv_fx;
    for (int v = 0; v < H; ++v) v_lut[v] = (static_cast<float>(v) - cy) * inv_fy;
    if (want_rgb) {
        cu_off.resize(static_cast<size_t>(W));
        for (int u = 0; u < W; ++u)
            cu_off[u] = ((cw == W) ? u : (u * cw) / W) * 3;
    }

    // Two passes over the raster. Pass 1 counts the valid pixels per
    // row; the prefix sum assigns every row a contiguous slot in the
    // compacted output, which is what lets pass 2 write in parallel
    // with no per-point append bookkeeping.
    row_count.resize(static_cast<size_t>(H));
    row_off.resize(static_cast<size_t>(H));

    // Hoist raw pointers out of the thread_local vectors before entering
    // the parallel regions: a thread_local name evaluated inside the
    // region resolves to each worker's own (empty) instance, not the
    // calling thread's buffers.
    uint32_t* counts = row_count.data();
#ifdef _OPENMP
    #pragma omp parallel for schedule(static) num_threads(pc_threads())
#endif
    for (int v = 0; v < H; ++v) {
        const uint16_t* row = d + static_cast<size_t>(v) * W;
        const int u_start = (v == 0) ? kSerialSkipPixels : 0;
        counts[v] = simd::count_valid_depth(row + u_start, W - u_start);
    }
    uint32_t total = 0;
    for (int v = 0; v < H; ++v) {
        row_off[v] = total;
        total += row_count[v];
    }

    res.count = total;
    xyz.resize(static_cast<size_t>(total) * 3);
    if (want_rgb) {
        rgb.resize(static_cast<size_t>(total) * 3);
    }

    const uint32_t* offs = row_off.data();
    const float* ul = u_lut.data();
    const float* vl = v_lut.data();
    const int32_t* cu = want_rgb ? cu_off.data() : nullptr;
    float* out_xyz = xyz.data();
    uint8_t* out_rgb = want_rgb ? rgb.data() : nullptr;
#ifdef _OPENMP
    #pragma omp parallel for schedule(static) num_threads(pc_threads())
#endif
    for (int v = 0; v < H; ++v) {
        const uint16_t* row = d + static_cast<size_t>(v) * W;
        const float y_unit = vl[v];
        float* dst = out_xyz + static_cast<size_t>(offs[v]) * 3;
        uint8_t* cdst = want_rgb
            ? out_rgb + static_cast<size_t>(offs[v]) * 3 : nullptr;
        const uint8_t* crow = want_rgb
            ? cdata + static_cast<size_t>((ch == H) ? v : (v * ch) / H) * cw * 3
            : nullptr;
        const int u_start = (v == 0) ? kSerialSkipPixels : 0;
        for (int u = u_start; u < W; ++u) {
            const uint16_t z_mm = row[u] & kDepthMask;
            if (z_mm == 0) continue;
            const float z_m = static_cast<float>(z_mm) * kMmToM;
            // Optical convention: X right, Y down, Z forward.
            dst[0] = ul[u] * z_m;
            dst[1] = y_unit * z_m;
            dst[2] = z_m;
            dst += 3;
            if (want_rgb) {
                // Nearest-neighbour sample into the color frame.
                const uint8_t* px = crow + cu[u];
                cdst[0] = px[0];
                cdst[1] = px[1];
                cdst[2] = px[2];
                cdst += 3;
            }
        }
    }

    res.xyz = xyz.data();
    if (want_rgb) {
        res.rgb = rgb.data();
        res.has_rgb = true;
    }
    return res;
}

}  // namespace pyeys3d
