// Point-cloud reprojection for pyeys3d.
//
// The depth post-process chain (spatial / temporal / hole-filling) runs
// natively inside CaptureEngine on the depth thread, so by the time a
// frame reaches Python it is already DEPTH_MM. PointCloud is the only
// post-process object the user constructs directly, because it needs the
// device calibration.

#pragma once

#include <cstdint>
#include <vector>

#include "capture_engine.hpp"

namespace pyeys3d {

// ---------------------------------------------------------------------------
//   PointCloud — reproject DEPTH_MM to vertices (optical convention)
// ---------------------------------------------------------------------------
//
// Output is XYZ float32 meters in the optical frame (X right, Y down,
// Z forward). When a color frame is supplied the per-vertex RGB is
// sampled and returned too.
// Views into per-thread scratch owned by calculate(); valid until the
// calling thread's next calculate() call. The binding layer copies them
// into the arrays it hands to Python before returning.
struct PointCloudResult {
    const float*   xyz = nullptr;   // N*3 floats, meters
    const uint8_t* rgb = nullptr;   // N*3 bytes (null when no color)
    uint32_t       count = 0;
    bool           has_rgb = false;
};

class PointCloud {
public:
    explicit PointCloud(const CaptureEngine& engine);
    // depth must be DEPTH_MM. color is optional rgb8; when given and its
    // dimensions are compatible the result carries per-vertex RGB.
    PointCloudResult calculate(const Frame& depth, const Frame* color);

private:
    Calibration calib_;
};

}  // namespace pyeys3d
