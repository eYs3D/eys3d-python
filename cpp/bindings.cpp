// pyeys3d — direct-eSPDI Python wrapper for eYs3D stereo depth cameras.
//
// Thin pybind11 layer over the C++ capture engine and point cloud.
// SDK calls live in capture_engine.cpp and the per-OS backends; the
// user-facing Python API lives in src/pyeys3d/.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <cstring>

#include "capture_engine.hpp"
#include "point_cloud.hpp"

namespace py = pybind11;

// PYEYS3D_VERSION is injected by CMake from the pyproject.toml version.
#ifndef PYEYS3D_VERSION
#define PYEYS3D_VERSION "0.0.0+unknown"
#endif

namespace {

// Frame data → numpy ndarray, zero-copy. Color is (H, W, 3) uint8; every
// uint16 domain (depth mm, disparity D11 / Q4) is (H, W) uint16. The array
// is a view onto the Frame's buffer; `base` (the Python Frame object) is
// kept alive for as long as the array exists, so the buffer stays valid.
// The Frame returned by wait_for_frames already owns a private copy of the
// rolling capture buffer, so no further copy is needed here.
py::array frame_to_ndarray(const pyeys3d::Frame& f, py::handle base) {
    if (f.data.empty()) return py::array();
    void* ptr = const_cast<uint8_t*>(f.data.data());
    const auto h = static_cast<py::ssize_t>(f.height);
    const auto w = static_cast<py::ssize_t>(f.width);
    py::array arr;
    if (f.domain == pyeys3d::FrameDomain::COLOR_RGB8) {
        arr = py::array_t<uint8_t>(
            {h, w, py::ssize_t{3}},
            {w * 3, py::ssize_t{3}, py::ssize_t{1}},
            static_cast<uint8_t*>(ptr), base);
    } else {
        arr = py::array_t<uint16_t>(
            {h, w},
            {w * 2, py::ssize_t{2}},
            reinterpret_cast<uint16_t*>(ptr), base);
    }
    // Read-only: the array shares the Frame's buffer, so writing through it
    // would mutate the frame (and any sibling view). Callers that need to
    // edit pixels should copy first, e.g. np.array(frame.get_data()).
    py::detail::array_proxy(arr.ptr())->flags &=
        ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
    return arr;
}

// PointCloudResult → (vertices, colors) numpy tuple. The result views
// the calling thread's scratch, so the copy below must happen before
// this thread's next calculate().
py::object pc_result_to_py(const pyeys3d::PointCloudResult& r) {
    py::array_t<float> verts({static_cast<py::ssize_t>(r.count), py::ssize_t{3}});
    if (r.count) std::memcpy(verts.mutable_data(), r.xyz,
                             static_cast<size_t>(r.count) * 3 * sizeof(float));
    if (r.has_rgb) {
        py::array_t<uint8_t> cols({static_cast<py::ssize_t>(r.count), py::ssize_t{3}});
        if (r.count) std::memcpy(cols.mutable_data(), r.rgb,
                                 static_cast<size_t>(r.count) * 3);
        return py::make_tuple(verts, cols);
    }
    return py::make_tuple(verts, py::none());
}

}  // namespace

PYBIND11_MODULE(_pyeys3d_native, m) {
    m.doc() = "pyeys3d native module: eSPDI capture engine, filters, point cloud.";
    m.attr("__version__") = PYEYS3D_VERSION;
    // Sentinel for OpenConfig.exposure_time (exposure register units are
    // signed, so "unset" cannot be -1).
    m.attr("EXPOSURE_UNSET") = pyeys3d::OpenConfig::kExposureUnset;

    // ----- Enums -----
    py::enum_<pyeys3d::FrameDomain>(m, "FrameDomain")
        .value("COLOR_RGB8",    pyeys3d::FrameDomain::COLOR_RGB8)
        .value("DEPTH_MM",      pyeys3d::FrameDomain::DEPTH_MM)
        .value("DISPARITY_D11", pyeys3d::FrameDomain::DISPARITY_D11)
        .value("DISPARITY_Q4",  pyeys3d::FrameDomain::DISPARITY_Q4);

    // ----- DeviceInfo -----
    py::class_<pyeys3d::DeviceInfo>(m, "DeviceInfo")
        .def_readonly("index", &pyeys3d::DeviceInfo::index)
        .def_readonly("pid", &pyeys3d::DeviceInfo::pid)
        .def_readonly("vid", &pyeys3d::DeviceInfo::vid)
        .def_readonly("serial_number", &pyeys3d::DeviceInfo::serial_number)
        .def_readonly("device_node", &pyeys3d::DeviceInfo::device_node)
        .def_readonly("usb_port", &pyeys3d::DeviceInfo::usb_port)
        .def_readonly("usb_port_type", &pyeys3d::DeviceInfo::usb_port_type)
        .def_readonly("firmware_version", &pyeys3d::DeviceInfo::firmware_version)
        .def("__repr__", [](const pyeys3d::DeviceInfo& d) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%04x", d.pid);
            return std::string("DeviceInfo(index=") + std::to_string(d.index)
                 + ", pid=0x" + buf
                 + ", sn='" + d.serial_number + "'"
                 + ", usb_port='" + d.usb_port + "')";
        });

    // ----- Calibration (intrinsics) -----
    py::class_<pyeys3d::Intrinsics>(m, "Intrinsics")
        .def_readonly("width", &pyeys3d::Intrinsics::width)
        .def_readonly("height", &pyeys3d::Intrinsics::height)
        .def_readonly("fx", &pyeys3d::Intrinsics::fx)
        .def_readonly("fy", &pyeys3d::Intrinsics::fy)
        .def_readonly("cx", &pyeys3d::Intrinsics::cx)
        .def_readonly("cy", &pyeys3d::Intrinsics::cy)
        .def_property_readonly("K", [](const pyeys3d::Intrinsics& i) {
            return py::array_t<double>(9, i.K.data()); })
        .def_property_readonly("D", [](const pyeys3d::Intrinsics& i) {
            return py::array_t<double>(8, i.D.data()); })
        .def_property_readonly("R", [](const pyeys3d::Intrinsics& i) {
            return py::array_t<double>(9, i.R.data()); })
        .def_property_readonly("P", [](const pyeys3d::Intrinsics& i) {
            return py::array_t<double>(12, i.P.data()); })
        .def_readonly("baseline_mm", &pyeys3d::Intrinsics::baseline_mm)
        .def_readonly("valid", &pyeys3d::Intrinsics::valid);

    // ----- Frame -----
    py::class_<pyeys3d::Frame>(m, "Frame")
        .def_readonly("domain", &pyeys3d::Frame::domain)
        .def_readonly("width", &pyeys3d::Frame::width)
        .def_readonly("height", &pyeys3d::Frame::height)
        .def_readonly("bytes_per_pixel", &pyeys3d::Frame::bytes_per_pixel)
        .def_readonly("frame_number", &pyeys3d::Frame::frame_number)
        .def_readonly("hw_timestamp_us", &pyeys3d::Frame::hw_timestamp_us)
        // Host-clock time of capture (epoch seconds, float — directly
        // comparable with time.time()), mapped from the hardware
        // timestamp via the engine's fitted clock model.
        .def_property_readonly("timestamp", [](const pyeys3d::Frame& f) {
            return static_cast<double>(f.host_timestamp_ns) / 1e9;
        })
        .def("get_data", [](py::object self) {
            return frame_to_ndarray(self.cast<const pyeys3d::Frame&>(), self);
        });

    // ----- OpenConfig -----
    py::class_<pyeys3d::OpenConfig>(m, "OpenConfig")
        .def(py::init<>())
        .def_readwrite("usb_port", &pyeys3d::OpenConfig::usb_port)
        .def_readwrite("serial_number", &pyeys3d::OpenConfig::serial_number)
        .def_readwrite("expected_pid", &pyeys3d::OpenConfig::expected_pid)
        .def_readwrite("color_w", &pyeys3d::OpenConfig::color_w)
        .def_readwrite("color_h", &pyeys3d::OpenConfig::color_h)
        .def_readwrite("color_fmt", &pyeys3d::OpenConfig::color_fmt)
        .def_readwrite("color_split_lr", &pyeys3d::OpenConfig::color_split_lr)
        .def_readwrite("is_mono", &pyeys3d::OpenConfig::is_mono)
        .def_readwrite("depth_w", &pyeys3d::OpenConfig::depth_w)
        .def_readwrite("depth_h", &pyeys3d::OpenConfig::depth_h)
        .def_readwrite("depth_dtype", &pyeys3d::OpenConfig::depth_dtype)
        .def_readwrite("zd_index", &pyeys3d::OpenConfig::zd_index)
        .def_readwrite("fps", &pyeys3d::OpenConfig::fps)
        .def_readwrite("interleave", &pyeys3d::OpenConfig::interleave)
        .def_readwrite("ir_value", &pyeys3d::OpenConfig::ir_value)
        .def_readwrite("auto_exposure", &pyeys3d::OpenConfig::auto_exposure)
        .def_readwrite("exposure_time", &pyeys3d::OpenConfig::exposure_time)
        .def_readwrite("auto_white_balance", &pyeys3d::OpenConfig::auto_white_balance)
        .def_readwrite("white_balance", &pyeys3d::OpenConfig::white_balance)
        .def_readwrite("power_line_frequency", &pyeys3d::OpenConfig::power_line_frequency)
        .def_readwrite("filter_spatial", &pyeys3d::OpenConfig::filter_spatial)
        .def_readwrite("spatial_alpha", &pyeys3d::OpenConfig::spatial_alpha)
        .def_readwrite("spatial_delta", &pyeys3d::OpenConfig::spatial_delta)
        .def_readwrite("spatial_magnitude", &pyeys3d::OpenConfig::spatial_magnitude)
        .def_readwrite("spatial_holes_fill", &pyeys3d::OpenConfig::spatial_holes_fill)
        .def_readwrite("filter_temporal", &pyeys3d::OpenConfig::filter_temporal)
        .def_readwrite("temporal_alpha", &pyeys3d::OpenConfig::temporal_alpha)
        .def_readwrite("temporal_delta", &pyeys3d::OpenConfig::temporal_delta)
        .def_readwrite("temporal_persistence", &pyeys3d::OpenConfig::temporal_persistence)
        .def_readwrite("filter_hole", &pyeys3d::OpenConfig::filter_hole)
        .def_readwrite("hole_mode", &pyeys3d::OpenConfig::hole_mode)
        .def_readwrite("depth_near_mm", &pyeys3d::OpenConfig::depth_near_mm)
        .def_readwrite("depth_far_mm", &pyeys3d::OpenConfig::depth_far_mm)
        .def_readwrite("quality_regs", &pyeys3d::OpenConfig::quality_regs);

    // ----- Context -----
    py::class_<pyeys3d::Context>(m, "Context")
        // APC_Init walks the USB bus, and the destructor releases it; both
        // are the same blocking class as query_devices below.
        .def(py::init<>(), py::call_guard<py::gil_scoped_release>())
        // Reads every camera's descriptors and waits its turn behind any
        // open in this process; both take long enough that holding the GIL
        // would stall every other thread.
        .def("query_devices", &pyeys3d::Context::query_devices,
             py::call_guard<py::gil_scoped_release>());

    // ----- CaptureEngine -----
    py::class_<pyeys3d::CaptureEngine>(m, "CaptureEngine")
        .def(py::init<>())
        .def("open", &pyeys3d::CaptureEngine::open, py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("start", &pyeys3d::CaptureEngine::start,
             py::call_guard<py::gil_scoped_release>())
        .def("close", &pyeys3d::CaptureEngine::close,
             py::call_guard<py::gil_scoped_release>())
        .def("wait_for_frames",
             [](pyeys3d::CaptureEngine& self, int timeout_ms) -> py::object {
                 pyeys3d::Frame c, d, r;
                 bool ok, has_right = false;
                 {
                     py::gil_scoped_release release;
                     ok = self.wait_for_frames(timeout_ms, c, d, r, has_right);
                 }
                 if (!ok) return py::none();
                 // A stream absent from the mode (e.g. depth-only or
                 // color-only) leaves its frame unfilled (empty data);
                 // surface it as None so get_*_frame() honors its
                 // Optional contract.
                 py::object color = c.data.empty() ? py::none() : py::cast(std::move(c));
                 py::object depth = d.data.empty() ? py::none() : py::cast(std::move(d));
                 py::object right = has_right ? py::cast(std::move(r)) : py::none();
                 return py::make_tuple(color, depth, right);
             },
             py::arg("timeout_ms") = 1000)
        .def_property_readonly("split_color", &pyeys3d::CaptureEngine::split_color)
        .def_property_readonly("pid", &pyeys3d::CaptureEngine::pid)
        .def_property_readonly("serial_number", &pyeys3d::CaptureEngine::serial_number)
        .def_property_readonly("usb_port", &pyeys3d::CaptureEngine::usb_port)
        .def_property_readonly("firmware_version", &pyeys3d::CaptureEngine::firmware_version)
        // Camera-control reads. Semantic accessors (not raw property ids):
        // the id spaces differ between the Linux and Windows SDKs, so the
        // mapping lives in the native per-OS backend. Each returns None
        // when the device is closed or lacks the control.
        .def("get_auto_exposure", &pyeys3d::CaptureEngine::get_auto_exposure,
             py::call_guard<py::gil_scoped_release>())
        .def("get_exposure", &pyeys3d::CaptureEngine::get_exposure,
             py::call_guard<py::gil_scoped_release>())
        .def("get_auto_white_balance",
             &pyeys3d::CaptureEngine::get_auto_white_balance,
             py::call_guard<py::gil_scoped_release>())
        .def("get_white_balance", &pyeys3d::CaptureEngine::get_white_balance,
             py::call_guard<py::gil_scoped_release>())
        .def("get_power_line_frequency",
             &pyeys3d::CaptureEngine::get_power_line_frequency,
             py::call_guard<py::gil_scoped_release>())
        .def("get_exposure_range", &pyeys3d::CaptureEngine::get_exposure_range,
             py::call_guard<py::gil_scoped_release>())
        .def("get_white_balance_range",
             &pyeys3d::CaptureEngine::get_white_balance_range,
             py::call_guard<py::gil_scoped_release>())
        // Runtime camera controls: write the device now and persist the value
        // for hot-plug reopens. Each returns False if the device rejects it.
        .def("set_ir_value", &pyeys3d::CaptureEngine::set_ir_value,
             py::arg("value"), py::call_guard<py::gil_scoped_release>())
        .def("get_ir_value", &pyeys3d::CaptureEngine::get_ir_value,
             py::call_guard<py::gil_scoped_release>())
        .def("set_auto_exposure", &pyeys3d::CaptureEngine::set_auto_exposure,
             py::arg("on"), py::call_guard<py::gil_scoped_release>())
        .def("set_exposure", &pyeys3d::CaptureEngine::set_exposure,
             py::arg("value"), py::call_guard<py::gil_scoped_release>())
        .def("set_auto_white_balance",
             &pyeys3d::CaptureEngine::set_auto_white_balance,
             py::arg("on"), py::call_guard<py::gil_scoped_release>())
        .def("set_white_balance", &pyeys3d::CaptureEngine::set_white_balance,
             py::arg("value"), py::call_guard<py::gil_scoped_release>())
        .def("set_power_line_frequency",
             &pyeys3d::CaptureEngine::set_power_line_frequency,
             py::arg("mode"), py::call_guard<py::gil_scoped_release>())
        // Retunes the running temporal stage; throws when the chain was
        // opened without it, since there is nothing to retune.
        .def("set_temporal_params", &pyeys3d::CaptureEngine::set_temporal_params,
             py::arg("alpha"), py::arg("delta"), py::arg("persistence"),
             py::call_guard<py::gil_scoped_release>())
        // Triggers the firmware's USB detach; False when not streaming.
        .def("reset_usb", &pyeys3d::CaptureEngine::reset_usb,
             py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("intrinsics", &pyeys3d::CaptureEngine::intrinsics)
        .def_property_readonly("depth_near_mm", &pyeys3d::CaptureEngine::depth_near_mm)
        .def_property_readonly("depth_far_mm", &pyeys3d::CaptureEngine::depth_far_mm)
        .def_property_readonly("is_open", &pyeys3d::CaptureEngine::is_open)
        .def_property_readonly("is_streaming", &pyeys3d::CaptureEngine::is_streaming)
        .def_property_readonly("is_connected", &pyeys3d::CaptureEngine::is_connected)
        .def_property_readonly("reconnect_count", &pyeys3d::CaptureEngine::reconnect_count)
        .def_property_readonly("dropped_color_frames",
                               &pyeys3d::CaptureEngine::dropped_color_frames)
        .def_property_readonly("dropped_depth_frames",
                               &pyeys3d::CaptureEngine::dropped_depth_frames)
        .def_property_readonly("color_fps", &pyeys3d::CaptureEngine::color_fps)
        .def_property_readonly("depth_fps", &pyeys3d::CaptureEngine::depth_fps)
        .def_property_readonly("quality_regs_ok",
                               &pyeys3d::CaptureEngine::quality_regs_ok)
        .def_property_readonly("quality_regs_failed",
                               &pyeys3d::CaptureEngine::quality_regs_failed)
        .def_property_readonly("quality_regs_pending",
                               &pyeys3d::CaptureEngine::quality_regs_pending);

    // ----- Point cloud -----
    // The depth filter chain runs natively in CaptureEngine; PointCloud is
    // the only post-process object constructed on the Python side.
    py::class_<pyeys3d::PointCloud>(m, "PointCloud")
        .def(py::init<const pyeys3d::CaptureEngine&>(), py::arg("engine"))
        .def("calculate",
             [](pyeys3d::PointCloud& self, const pyeys3d::Frame& depth,
                const pyeys3d::Frame* color) {
                 // The result views the calling thread's scratch, so
                 // concurrent calculate() calls on one PointCloud (possible
                 // because the GIL is released here) stay safe: each thread
                 // fills and then copies out its own buffers.
                 pyeys3d::PointCloudResult r;
                 {
                     py::gil_scoped_release release;
                     r = self.calculate(depth, color);
                 }
                 return pc_result_to_py(r);
             },
             py::arg("depth"), py::arg("color") = nullptr,
             "Reproject a DEPTH_MM frame to (vertices, colors). vertices is "
             "(N, 3) float32 meters (optical: X right, Y down, Z forward). "
             "colors is (N, 3) uint8 when a color frame is given, else None.");
}
