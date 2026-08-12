#include "capture_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "apc_error.hpp"
#include "capture_internal.hpp"
#include "device_setup_lock.hpp"
#include "eSPDI.h"
#include "eSPDI_def.h"
#include "log.hpp"
#include "platform.hpp"
#include "simd_kernels.hpp"

namespace pyeys3d {

namespace {

// A stream silent for this long, while streaming is wanted, is a
// disconnect. Wall time, not error counts: non-blocking V4L2 reports
// "no frame yet" constantly during normal operation.
constexpr int64_t kDisconnectNs = 3'000'000'000LL;

// How far into the future the liveness stamps are pushed when a stream is
// (re)started. It has to outlast kDisconnectNs plus the cold-start delay
// before the first frame, which is firmware-dependent and runs to several
// seconds; otherwise the watchdog reads its own start-up as a disconnect
// and reopens a device that was only still waking up.
constexpr int64_t kLivenessGraceNs = 7'000'000'000LL;

// APC_Init3's bMonitorUSBEvent is false. The SDK's USB-event window belongs
// to the thread that opened the handle and reports nothing unless that thread
// pumps a message loop, which this engine never does; releasing the handle
// from another thread — the watchdog rebuilding a device — then tears the
// window down across threads and terminates the process. Disconnects come
// from stream liveness here.
int apc_init_handle(void** handle) {
#ifdef _WIN32
    return APC_Init3(handle, false, false, false);
#else
    return APC_Init(handle, false);
#endif
}

}  // namespace

// ---------------------------------------------------------------------------
//   Context
// ---------------------------------------------------------------------------

Context::Context() {
    SignalHandlerGuard sig_guard;
    const int rc = apc_init_handle(&handle_);
    if (rc != APC_OK || handle_ == nullptr) {
        if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
        throw std::runtime_error(
            "APC_Init failed while opening the camera: " + apc_strerror(rc));
    }
}

Context::~Context() {
    if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
}

namespace {

// Serial reads share the camera's control pipe with any other process
// configuring the same device and can fail transiently while it does.
// Retry briefly rather than report an empty serial that would break
// serial_number binding (kSerialReadAttempts is 1 on Linux, where a
// failed read means the device is held by another process).
std::string read_serial_retry(void* handle, DEVSELINFO& sel) {
    for (int attempt = 0; attempt < kSerialReadAttempts; ++attempt) {
        if (attempt > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        unsigned char buf[256] = {0};
        int len = 0;
        if (APC_GetSerialNumber(handle, &sel, buf, sizeof(buf), &len) == APC_OK
            && len > 0) {
            std::string s = utf16le_to_ascii(buf, len, sizeof(buf));
            if (!s.empty()) return s;
        }
    }
    return {};
}

// APC_GetDeviceInfo shares the same control pipe and can fail transiently
// while another process configures the device; a failed read leaves info
// zero-initialized, emptying usb_port and so breaking usb_port binding (and
// letting two same-serial units collide). Retry on the same budget as the
// serial read — single-shot on Linux, where the failure is persistent.
bool read_device_info_retry(void* handle, DEVSELINFO& sel,
                            DEVINFORMATION& info) {
    for (int attempt = 0; attempt < kSerialReadAttempts; ++attempt) {
        if (attempt > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        info = DEVINFORMATION{};   // discard any partial write from a failed try
        if (APC_GetDeviceInfo(handle, &sel, &info) == APC_OK)
            return true;
    }
    return false;
}

}  // namespace

std::vector<DeviceInfo> Context::query_devices() {
    std::vector<DeviceInfo> out;
    if (!handle_) return out;
    // Reading the device list while another thread opens a camera returns
    // indices that no longer hold by the time they are used.
    SetupTurn setup("enumerate devices");
    const int count = APC_GetDeviceNumber(handle_);
    if (count <= 0) return out;
    out.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        DEVSELINFO sel{i};
        DEVINFORMATION info{};
        DeviceInfo dev;
        dev.index = i;
        dev.serial_number = read_serial_retry(handle_, sel);
        if (read_device_info_retry(handle_, sel, info)) {
            if (info.strDevName) dev.device_node.assign(info.strDevName);
            dev.pid = info.wPID;
            dev.vid = info.wVID;
        }
        dev.usb_port = resolve_usb_port(info);
        {
            // Readable from the descriptor without opening the device;
            // gates which video modes the link can carry (a USB2 link
            // cannot open USB3-only modes).
            USB_PORT_TYPE pt = USB_PORT_TYPE_UNKNOW;
            if (APC_GetDevicePortType(handle_, &sel, &pt) == APC_OK) {
                dev.usb_port_type = static_cast<int>(pt);
            }
        }
        {
            // Readable without opening the device; leave empty if the
            // camera is busy (held by another process).
            char fw_buf[256] = {0};
            int fw_len = 0;
            if (APC_GetFwVersion(handle_, &sel, fw_buf, sizeof(fw_buf) - 1,
                                 &fw_len) == APC_OK && fw_len > 0) {
                dev.firmware_version = fw_buf;
            }
        }
        out.push_back(std::move(dev));
    }
    return out;
}

// ---------------------------------------------------------------------------
//   CaptureEngine
// ---------------------------------------------------------------------------

CaptureEngine::CaptureEngine() : log_tag_(kTag) {}
CaptureEngine::~CaptureEngine() {
    close();
    // Last, and in this order: refuse any callback that has not started,
    // then wait for one that has. A handle the USB self-reset abandoned can
    // still be delivering, and an ingest already past the flag would write
    // into members this destructor is about to destroy.
    cb_gate_->alive.store(false, std::memory_order_release);
    drain_callbacks();
}

void CaptureEngine::load_calibration() {
    constexpr int kMaxAttempts = 4;
    constexpr int kBackoffMs   = 150;
    DEVSELINFO sel{devsel_index_};
    eSPCtrl_RectLogData log{};
    int rc = APC_OK;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        rc = APC_GetRectifyMatLogData(handle_, &sel, &log, cfg_.zd_index);
        if (rc == APC_OK) break;
        if (rc != APC_READFLASHFAIL) break;
        PYEYS3D_WARN(tag(),
            "APC_GetRectifyMatLogData(index=%d) %s (flash read), retry %d/%d",
            cfg_.zd_index, apc_strerror(rc).c_str(), attempt + 1, kMaxAttempts);
        std::this_thread::sleep_for(std::chrono::milliseconds(kBackoffMs));
    }
    if (rc != APC_OK) {
        PYEYS3D_WARN(tag(),
            "APC_GetRectifyMatLogData(index=%d) %s ; intrinsics unavailable",
            cfg_.zd_index, apc_strerror(rc).c_str());
        return;
    }
    // An uncalibrated unit returns an all-zero rectify log with APC_OK;
    // zero intrinsics reproject everything to the origin, so report the
    // calibration as unavailable instead.
    if (log.NewCamMat1[0] <= 0.0) {
        PYEYS3D_WARN(tag(),
            "Calibration parameters are missing (rectify log %d is empty; "
            "device not calibrated). Streaming continues, but color is not "
            "rectified, depth quality is degraded, and the point cloud is "
            "unavailable.", cfg_.zd_index);
        return;
    }
    // Built aside and published in one assignment: a reader taking
    // calib_mtx_ then sees either the whole previous calibration or the
    // whole new one, never a mix of the two.
    Calibration c{};
    c.width  = cfg_.color_w > 0 ? cfg_.color_w : cfg_.depth_w;
    c.height = cfg_.color_h > 0 ? cfg_.color_h : cfg_.depth_h;
    for (int i = 0; i < 9; ++i)  c.left.K[i]  = log.CamMat1[i];
    for (int i = 0; i < 8; ++i)  c.left.D[i]  = log.CamDist1[i];
    for (int i = 0; i < 9; ++i)  c.left.R[i]  = log.LRotaMat[i];
    for (int i = 0; i < 12; ++i) c.left.P[i]  = log.NewCamMat1[i];
    for (int i = 0; i < 9; ++i)  c.right.K[i] = log.CamMat2[i];
    for (int i = 0; i < 8; ++i)  c.right.D[i] = log.CamDist2[i];
    for (int i = 0; i < 9; ++i)  c.right.R[i] = log.RRotaMat[i];
    for (int i = 0; i < 12; ++i) c.right.P[i] = log.NewCamMat2[i];
    c.baseline_mm   = std::abs(static_cast<double>(log.TranMat[0]));
    c.out_img_width  = log.OutImgWidth;
    c.out_img_height = log.OutImgHeight;
    c.valid = true;
    {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        calib_ = c;
    }
    PYEYS3D_INFO(tag(),
        "Rectify loaded (index=%d): L fx=%.2f cx=%.2f / baseline=%.2f mm",
        cfg_.zd_index, c.left.K[0], c.left.K[2], c.baseline_mm);
}

Intrinsics CaptureEngine::intrinsics() const {
    Intrinsics in;
    std::lock_guard<std::mutex> lk(calib_mtx_);
    if (!calib_.valid) return in;
    // The log sizes the L|R pair; cx / cy are per lens, so is this.
    in.width  = calib_.out_img_width / 2;
    in.height = calib_.out_img_height;
    in.K = calib_.left.K;
    in.D = calib_.left.D;
    in.R = calib_.left.R;
    in.P = calib_.left.P;
    in.fx = in.P[0];  in.fy = in.P[5];
    in.cx = in.P[2];  in.cy = in.P[6];
    in.baseline_mm = calib_.baseline_mm;
    in.valid = true;
    return in;
}

void CaptureEngine::load_zd_table_if_needed() {
    if (!chain_disparity_) return;
    DEVSELINFO sel{devsel_index_};
    if (!pyeys3d::load_zd_table(handle_, &sel, cfg_.zd_index, zd_table_)) {
        // Release the handle, and the device too if this ran on the
        // reconnect path, where the streams are already up.
        if (device_open_) APC_CloseDevice(handle_, &sel);
        APC_Release(&handle_);
        handle_ = nullptr;
        device_open_ = false;
        throw std::runtime_error(
            "The spatial and temporal filters need this camera's "
            "disparity-to-millimetre table, which could not be read from it "
            "(zd_index=" + std::to_string(cfg_.zd_index) + "). Start without "
            "those filters to stream the firmware's own depth output.");
    }
}

// Swap the current SDK handle for a fresh one whose first-selected
// device is `index`, verifying the device list did not shift in between
// (by serial when known, by PID otherwise). The identity read doubles
// as the handle's first selection — the ordering that makes
// multi-camera work on Windows (see os_fresh_handle_required).
bool CaptureEngine::open_fresh_handle_for(int index, uint16_t expected_pid,
                                          const std::string& expected_serial) {
    if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
    {
        SignalHandlerGuard sig_guard;
        if (apc_init_handle(&handle_) != APC_OK || handle_ == nullptr) {
            if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
            return false;
        }
    }
    if (APC_GetDeviceNumber(handle_) <= index) {
        APC_Release(&handle_); handle_ = nullptr;
        return false;
    }
    DEVSELINFO sel{index};
    if (!expected_serial.empty()) {
        if (read_serial_retry(handle_, sel) != expected_serial) {
            APC_Release(&handle_); handle_ = nullptr;
            return false;
        }
    }
    DEVINFORMATION info{};
    if (APC_GetDeviceInfo(handle_, &sel, &info) != APC_OK
        || (expected_pid != 0 && info.wPID != expected_pid)) {
        APC_Release(&handle_); handle_ = nullptr;
        return false;
    }
    return true;
}

void CaptureEngine::open(const OpenConfig& cfg) {
    if (device_open_) throw std::runtime_error("This pipeline already has a camera open");
    // Held across the whole open: enumerating to resolve the caller's pin,
    // reading calibration off the camera and opening the streams all share
    // the SDK's per-process state.
    SetupTurn setup("open " + (cfg.serial_number.empty()
                               ? std::string("a camera")
                               : "camera " + cfg.serial_number));
    cfg_ = cfg;

    {
        SignalHandlerGuard sig_guard;
        const int rc = apc_init_handle(&handle_);
        if (rc != APC_OK || handle_ == nullptr) {
            if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
            throw std::runtime_error(
                "APC_Init failed while listing devices: " + apc_strerror(rc));
        }
    }

    const int count = APC_GetDeviceNumber(handle_);
    if (count <= 0) {
        APC_Release(&handle_); handle_ = nullptr;
        throw std::runtime_error("No eYs3D camera enumerated: "
                                 + apc_strerror(APC_NoDevice));
    }

    struct Enumerated {
        int index;
        uint16_t pid;
        std::string serial;
        std::string usb_port;
    };
    std::vector<Enumerated> devs;
    devs.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        DEVSELINFO sel{i};
        DEVINFORMATION info{};
        Enumerated d{i, 0, {}, {}};
        d.serial = read_serial_retry(handle_, sel);
        // This is the path that consults the usb_port pin.
        if (read_device_info_retry(handle_, sel, info)) {
            d.pid = info.wPID;
        }
        d.usb_port = resolve_usb_port(info);
        devs.push_back(std::move(d));
    }

    auto summarize = [](const std::vector<Enumerated>& ds) {
        std::string s;
        for (const auto& d : ds) {
            char pid_hex[8];
            std::snprintf(pid_hex, sizeof(pid_hex), "%04x", d.pid);
            s += "\n  pid=0x" + std::string(pid_hex)
               + "  sn='" + d.serial + "'  usb_port='" + d.usb_port + "'";
        }
        return s;
    };

    // Identity filters: serial_number is a substring match, usb_port an
    // exact match; when both are given they are OR-ed. Two filters
    // pointing at different devices leave more than one candidate and
    // the open fails below rather than silently picking either.
    const bool have_usb = !cfg.usb_port.empty();
    const bool have_sn  = !cfg.serial_number.empty();
    std::vector<Enumerated> cands;
    for (const auto& d : devs) {
        const bool usb_match = have_usb && d.usb_port == cfg.usb_port;
        const bool sn_match  = have_sn
            && d.serial.find(cfg.serial_number) != std::string::npos;
        if (!have_usb && !have_sn) cands.push_back(d);
        else if (usb_match || sn_match) cands.push_back(d);
    }
    if (cands.empty()) {
        std::string msg = "No device matches";
        if (have_sn)  msg += " serial_number~'" + cfg.serial_number + "'";
        if (have_usb) msg += std::string(have_sn ? " /" : "")
                           + " usb_port='" + cfg.usb_port + "'";
        msg += ". Enumerated:" + summarize(devs);
        APC_Release(&handle_); handle_ = nullptr;
        throw std::runtime_error(msg);
    }
    // The PID acts as a model check, not an identity filter.
    if (cfg.expected_pid != 0) {
        std::vector<Enumerated> pid_ok;
        for (const auto& d : cands)
            if (d.pid == cfg.expected_pid) pid_ok.push_back(d);
        if (pid_ok.empty()) {
            char want[8];
            std::snprintf(want, sizeof(want), "%04x", cfg.expected_pid);
            const std::string msg =
                "Matched device(s) are not the expected model (pid 0x"
                + std::string(want) + "). Matched:" + summarize(cands);
            APC_Release(&handle_); handle_ = nullptr;
            throw std::runtime_error(msg);
        }
        cands.swap(pid_ok);
    }
    if (cands.size() > 1) {
        const std::string msg =
            "Device selection is ambiguous ("
            + std::to_string(cands.size()) + " devices match); pin one with "
            "serial_number= or usb_port= :" + summarize(cands);
        APC_Release(&handle_); handle_ = nullptr;
        throw std::runtime_error(msg);
    }

    const Enumerated& chosen_dev = cands.front();
    const int chosen = chosen_dev.index;
    pid_ = chosen_dev.pid;
    {
        std::lock_guard<std::mutex> ilk(identity_mtx_);
        serial_number_ = chosen_dev.serial;
        usb_port_ = chosen_dev.usb_port;
    }
    mono_ = cfg.is_mono;
    // Name the camera in every line from here on: a process driving two of
    // them otherwise logs two indistinguishable streams. Serial is what a
    // user can match against the device list; usb_port covers modules
    // shipped without one.
    log_tag_ = std::string(kTag) + "["
               + (chosen_dev.serial.empty() ? chosen_dev.usb_port
                                            : chosen_dev.serial)
               + "]";

    // The identity reads above already walked every index on this
    // handle. Where the SDK binds stream metadata to a handle's first
    // selection (Windows), re-init so the chosen device is the first —
    // and only — index this handle touches.
    if (os_fresh_handle_required()
        && !open_fresh_handle_for(chosen, chosen_dev.pid, chosen_dev.serial)) {
        throw std::runtime_error(
            "Device list changed while opening (index " + std::to_string(chosen)
            + " no longer matches the selected camera); retry start()");
    }
    devsel_index_ = chosen;
    shared_sel_.index = chosen;
    DEVSELINFO sel{chosen};

    // Read FW version after selection, before the mode setup. Besides
    // logging, this read is required for correct color/depth stream
    // separation on non-interleave devices.
    {
        char fw_buf[256] = {0};
        int fw_len = 0;
        const int fw_rc = APC_GetFwVersion(handle_, &sel, fw_buf, sizeof(fw_buf) - 1, &fw_len);
        if (fw_rc == APC_OK && fw_len > 0) {
            std::lock_guard<std::mutex> ilk(identity_mtx_);
            firmware_version_ = fw_buf;
            PYEYS3D_INFO(tag(), "FW version: %s", firmware_version_.c_str());
        }
    }

    // Depth clip range — already resolved to per-model values by the caller.
    depth_min_mm_ = cfg.depth_near_mm;
    depth_max_mm_ = cfg.depth_far_mm;

    // Resolve the native post-process chain. A disparity-domain filter
    // (spatial / temporal) requires raw D11 from the device (dtype + 2)
    // and the ZD table; hole filling runs in the mm domain either way.
    const bool depth_present = cfg.depth_w > 0 && cfg.depth_h > 0;
    spatial_on_  = cfg.filter_spatial  && depth_present;
    temporal_on_ = cfg.filter_temporal && depth_present;
    hole_on_     = cfg.filter_hole     && depth_present;
    chain_disparity_ = spatial_on_ || temporal_on_;
    if ((cfg.filter_spatial || cfg.filter_temporal || cfg.filter_hole) && !depth_present) {
        PYEYS3D_WARN(tag(), "Depth filters were requested but this mode opens "
                     "no depth stream; the spatial, temporal and "
                     "hole-filling stages are off");
    }

    spatial_params_.alpha_q8   = static_cast<int>(std::lround(
        std::clamp(cfg.spatial_alpha, 0.0, 1.0) * 256.0));
    // Delta is promoted to Q4 before the kernels compare it, so it is held
    // under 4096 here to stay inside the uint16 lanes it is compared in.
    spatial_params_.delta_q4   = std::clamp(cfg.spatial_delta, 1, 4095) << 4;
    spatial_params_.magnitude  = std::clamp(cfg.spatial_magnitude, 1, 5);
    spatial_params_.holes_fill = std::max(0, cfg.spatial_holes_fill);
    temporal_params_.alpha_q8    = static_cast<int>(std::lround(
        std::clamp(cfg.temporal_alpha, 0.0, 1.0) * 256.0));
    temporal_params_.delta       = std::clamp(cfg.temporal_delta, 0, 4095);
    temporal_params_.persistence = std::clamp(cfg.temporal_persistence, 0, 8);
    hole_mode_ = static_cast<pyeys3d::HoleFillMode>(std::clamp(cfg.hole_mode, 0, 3));

    effective_depth_dtype_ = chain_disparity_
        ? cfg.depth_dtype + 2
        : cfg.depth_dtype;

    // MJPEG modes need a JPEG decoder; fail at open rather than letting
    // wait_for_frames() time out forever with no color.
    if (cfg.color_fmt == 1 && cfg.color_w > 0) {
        std::string decoder_err;
        if (!os_init_decoder(decoder_err)) {
            APC_Release(&handle_); handle_ = nullptr;
            throw std::runtime_error(decoder_err);
        }
    }

    // Read the camera's flash before opening the streams. Both reads want
    // only the handle and the mode's zd_index, and on a device that is
    // removed mid-open a flash read against an opened handle does not
    // return at all, which strands start() with no way back. Against an
    // un-opened one it fails and the error reaches the caller.
    load_calibration();
    load_zd_table_if_needed();

    std::string err;
    if (!os_open_streams(err)) {
        APC_Release(&handle_); handle_ = nullptr;
        throw std::runtime_error(err);
    }
    device_open_ = true;

    apply_camera_controls(sel);
}

void CaptureEngine::start() {
    if (!device_open_) throw std::runtime_error("No camera is open; call open() first");
    if (streaming_.load(std::memory_order_acquire)) return;
    // Buffers must be sized before streaming_ is set: the Windows fetch
    // layer's SDK callback gates on streaming_ and ingests immediately.
    prepare_buffers();
    got_any_frame_.store(false, std::memory_order_relaxed);
    // Arm the depth-quality worker before streaming_ so the first depth
    // frame (which can arrive immediately on the callback backends)
    // cannot miss the flag.
    quality_ok_.store(0, std::memory_order_relaxed);
    quality_failed_.store(0, std::memory_order_relaxed);
    quality_pending_.store(!cfg_.quality_regs.empty(), std::memory_order_release);
    streaming_.store(true, std::memory_order_release);
    os_start_fetch();
    // Seeded after fetch is up: os_start_fetch() can block up to the
    // color-first cap (Linux) waiting for the first color frame, so seeding
    // before it would leave the depth stamp already stale by the time the
    // watchdog first checks.
    const int64_t seed = steady_now_ns() + kLivenessGraceNs;
    last_color_ns_.store(seed, std::memory_order_relaxed);
    last_depth_ns_.store(seed, std::memory_order_relaxed);
    watchdog_thread_ = std::thread(&CaptureEngine::watchdog_loop, this);
}

void CaptureEngine::close() {
    streaming_.store(false, std::memory_order_release);
    cv_.notify_all();
    // The watchdog goes first, and alone: during a reconnect it stops and
    // restarts the fetch loops and joins the quality worker itself, so
    // reaching for either while it still runs puts two threads on one
    // std::thread — undefined, and fatal on the way out of a destructor.
    // It leaves within its 500 ms tick, or once the reopen it is inside
    // returns. Everything below then owns these threads exclusively.
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
    os_stop_fetch();
    // After os_stop_fetch no ingest can launch the quality worker, and a
    // running worker bails on !streaming_; join before taking control_mtx_
    // (the worker locks it per register).
    quality_pending_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> qlk(quality_thread_mtx_);
        if (quality_thread_.joinable()) quality_thread_.join();
    }
    std::lock_guard<std::timed_mutex> lk(control_mtx_);   // vs concurrent prop access
    // The same teardown the watchdog uses, so the callback context this
    // open allocated is freed on this path too.
    os_teardown_device();
    os_release_decoder();
}

// Write the user-set camera controls to the device. Only touches what was
// explicitly set (-1 / tri-state fields leave the camera at its current
// state). A manual exposure or white-balance value forces the matching auto
// mode off first, since the camera ignores manual writes while auto is on.
void CaptureEngine::apply_camera_controls(DEVSELINFO& sel) {
    const bool want_manual_exposure =
        cfg_.exposure_time != OpenConfig::kExposureUnset;
    if (cfg_.auto_exposure >= 0 || want_manual_exposure) {
        const bool ae_auto = (cfg_.auto_exposure == 1) && !want_manual_exposure;
        if (!os_write_auto_exposure(sel, ae_auto)) {
            PYEYS3D_WARN(tag(), "auto-exposure write (%s) failed",
                         ae_auto ? "auto" : "manual");
        }
    }
    if (want_manual_exposure && !os_write_exposure_value(sel, cfg_.exposure_time)) {
        PYEYS3D_WARN(tag(), "exposure write (%d) failed", cfg_.exposure_time);
    }
    const bool want_manual_wb = cfg_.white_balance >= 0;
    if (cfg_.auto_white_balance >= 0 || want_manual_wb) {
        const bool awb_on = (cfg_.auto_white_balance == 1) && !want_manual_wb;
        if (!os_write_auto_white_balance(sel, awb_on)) {
            PYEYS3D_WARN(tag(), "auto-white-balance write (%s) failed",
                         awb_on ? "auto" : "manual");
        }
    }
    if (want_manual_wb && !os_write_white_balance_value(sel, cfg_.white_balance)) {
        PYEYS3D_WARN(tag(), "white-balance write (%d) failed", cfg_.white_balance);
    }
    if (cfg_.power_line_frequency >= 0
        && !os_write_power_line_frequency(sel, cfg_.power_line_frequency)) {
        PYEYS3D_WARN(tag(), "power-line-frequency write (%d) failed",
                     cfg_.power_line_frequency);
    }
}

// Close and re-open the bound device, re-applying every mode knob. Runs
// only from the watchdog while the capture loops are parked on
// reconnecting_. Returns true once streaming can resume.
bool CaptureEngine::reopen_device() {
    // A reconnect re-enumerates and re-opens exactly as open() does, so it
    // takes the same turn — a replug during another camera's open would
    // otherwise corrupt both. On the watchdog thread, so a timeout here is
    // one failed retry: it comes round again on the next tick.
    SetupTurn setup("reopen " + (serial_number_.empty()
                                 ? std::string("a camera")
                                 : "camera " + serial_number_),
                    [this] {
                        return !streaming_.load(std::memory_order_acquire);
                    });
    // Hold the control lock for the whole rebuild: a concurrent prop read
    // or runtime setter must never see the handle mid-teardown.
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    // Force the color stream to re-establish before depth resumes reading
    // (same ordering requirement as start()); the depth loop gates on this.
    color_started_.store(false, std::memory_order_release);
    // Drop the temporal IIR history so the first frames after a reconnect
    // are not blended against pre-disconnect state.
    temporal_state_.reset();
    // The device clock restarts on a replug; rebuild the hw -> host
    // clock model from scratch.
    {
        std::lock_guard<std::mutex> flk(mtx_);
        reset_time_model();
    }
    os_teardown_device();

    {
        SignalHandlerGuard sig_guard;
        if (apc_init_handle(&handle_) != APC_OK || handle_ == nullptr) {
            if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
            return false;
        }
    }
    // Re-bind to the same physical camera: exact USB port first, then
    // serial number (replug on a different socket), then PID.
    const int count = APC_GetDeviceNumber(handle_);
    int by_usb = -1, by_serial = -1, by_pid = -1;
    std::string serial_usb_port;   // port of the serial-matched device
    for (int i = 0; i < count; ++i) {
        DEVSELINFO s{i};
        DEVINFORMATION info{};
        uint16_t pid = 0;
        if (APC_GetDeviceInfo(handle_, &s, &info) == APC_OK) {
            pid = info.wPID;
        }
        const std::string up = resolve_usb_port(info);
        // Read this candidate's serial when one is pinned: it confirms a
        // port hit is the pinned unit, and locates that unit if it moved.
        std::string sn;
        bool sn_read = false;
        if (!serial_number_.empty()) {
            unsigned char sn_buf[256] = {0};
            int sn_len = 0;
            if (APC_GetSerialNumber(handle_, &s, sn_buf, sizeof(sn_buf),
                                    &sn_len) == APC_OK && sn_len > 0) {
                sn = utf16le_to_ascii(sn_buf, sn_len, sizeof(sn_buf));
                sn_read = true;
            }
        }
        // Exact USB port. Linux does not re-verify the serial on a fresh
        // handle (see os_fresh_handle_required), so require the pinned serial
        // to match here too -- else a different same-model unit swapped onto
        // this port would bind in its place. An unreadable serial falls
        // through to the port match, so a flaky read does not strand the
        // reconnect of the pinned camera.
        if (by_usb < 0 && !usb_port_.empty() && up == usb_port_
            && (serial_number_.empty() || !sn_read || sn == serial_number_)) {
            by_usb = i;
            break;
        }
        if (by_serial < 0 && !serial_number_.empty() && sn_read
            && sn == serial_number_) {
            by_serial = i;
            serial_usb_port = up;
        }
        if (by_pid < 0 && pid == pid_) by_pid = i;
    }
    // Bind by stable identity first (exact USB port, then serial). Fall back
    // to first-PID-match only when neither a serial nor a USB port was ever
    // pinned -- a single-camera auto-open, with no other same-model unit to
    // confuse it with. With a pinned identity that no present device matches,
    // the bound camera is absent: stay disconnected and let the watchdog
    // retry, rather than silently binding a different same-model unit.
    int chosen = by_usb >= 0 ? by_usb : by_serial;
    if (chosen < 0 && serial_number_.empty() && usb_port_.empty())
        chosen = by_pid;
    if (chosen < 0) { APC_Release(&handle_); handle_ = nullptr; return false; }
    if (chosen == by_serial && by_usb < 0 && serial_usb_port != usb_port_) {
        PYEYS3D_INFO(tag(), "camera moved from usb_port '%s' to '%s'",
                     usb_port_.c_str(), serial_usb_port.c_str());
        std::lock_guard<std::mutex> ilk(identity_mtx_);
        usb_port_ = serial_usb_port;
    }
    // The scan above selected every index on this handle; re-init so the
    // chosen device is the handle's first selection where required
    // (same ordering as open()).
    if (os_fresh_handle_required()
        && !open_fresh_handle_for(chosen, pid_, serial_number_)) {
        return false;
    }
    devsel_index_ = chosen;
    // The fetch loops and the quality worker read shared_sel_; they are
    // parked on reconnecting_ while this runs, and the replug may have
    // shifted the device index.
    shared_sel_.index = chosen;
    DEVSELINFO s{chosen};

    // Reload calibration and the ZD table so intrinsics and the disparity->mm
    // lookup match the device now bound (identity is confirmed above), not the
    // pre-disconnect copy. Ahead of the stream open for the same reason as
    // open(): a camera that drops again here would otherwise leave the
    // watchdog inside a flash read that never returns, and close() waits on
    // this thread. A ZD failure throws; catch it so the watchdog retries
    // instead of the thread unwinding.
    try {
        load_calibration();
        load_zd_table_if_needed();
    } catch (const std::exception& e) {
        PYEYS3D_WARN(tag(), "reopen: calibration/ZD reload failed: %s", e.what());
        if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
        device_open_ = false;
        return false;
    }

    // A hard failure here (wrong dtype / interleave corrupts both
    // streams) aborts the attempt; the watchdog retries next tick.
    std::string err;
    if (!os_open_streams(err)) {
        PYEYS3D_WARN(tag(), "reopen: stream open failed: %s", err.c_str());
        APC_Release(&handle_); handle_ = nullptr;
        return false;
    }
    device_open_ = true;

    // Re-apply the user-set controls; runtime setters update cfg_, so this
    // restores the values as last set, not as start() saw them.
    apply_camera_controls(s);
    return true;
}

// Semantic control reads: lock, then dispatch to the per-OS primitive
// that knows the platform's property-id mapping.
std::optional<bool> CaptureEngine::get_auto_exposure() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    return os_read_auto_exposure(sel);
}

std::optional<int> CaptureEngine::get_exposure() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    return os_read_exposure_value(sel);
}

std::optional<bool> CaptureEngine::get_auto_white_balance() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    return os_read_auto_white_balance(sel);
}

std::optional<int> CaptureEngine::get_white_balance() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    return os_read_white_balance_value(sel);
}

std::optional<int> CaptureEngine::get_power_line_frequency() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    return os_read_power_line_frequency(sel);
}

// ---------------------------------------------------------------------------
//   Runtime camera controls
// ---------------------------------------------------------------------------
// Each setter writes the device under the control lock and mirrors the value
// into cfg_, so a watchdog reopen re-applies the state as last set.

bool CaptureEngine::set_ir_value(int value) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_ || value < 0) return false;
    DEVSELINFO sel{devsel_index_};
    if (APC_SetCurrentIRValue(handle_, &sel,
                              static_cast<unsigned short>(value)) != APC_OK)
        return false;
    cfg_.ir_value = value;
    return true;
}

std::optional<int> CaptureEngine::get_ir_value() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    unsigned short value = 0;
    if (APC_GetCurrentIRValue(handle_, &sel, &value) != APC_OK)
        return std::nullopt;
    return static_cast<int>(value);
}

bool CaptureEngine::set_auto_exposure(bool on) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return false;
    DEVSELINFO sel{devsel_index_};
    if (!os_write_auto_exposure(sel, on)) return false;
    cfg_.auto_exposure = on ? 1 : 0;
    if (on) cfg_.exposure_time = OpenConfig::kExposureUnset;
    return true;
}

bool CaptureEngine::set_exposure(int value) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    // No sign check: exposure register units are signed on some modules.
    if (!handle_ || !device_open_ || value == OpenConfig::kExposureUnset)
        return false;
    DEVSELINFO sel{devsel_index_};
    // The camera ignores exposure writes while AE is automatic.
    os_write_auto_exposure(sel, false);
    if (!os_write_exposure_value(sel, value)) return false;
    cfg_.auto_exposure = 0;
    cfg_.exposure_time = value;
    return true;
}

bool CaptureEngine::set_auto_white_balance(bool on) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return false;
    DEVSELINFO sel{devsel_index_};
    if (!os_write_auto_white_balance(sel, on)) return false;
    cfg_.auto_white_balance = on ? 1 : 0;
    if (on) cfg_.white_balance = -1;
    return true;
}

bool CaptureEngine::set_white_balance(int value) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_ || value < 0) return false;
    DEVSELINFO sel{devsel_index_};
    os_write_auto_white_balance(sel, false);
    if (!os_write_white_balance_value(sel, value)) return false;
    cfg_.auto_white_balance = 0;
    cfg_.white_balance = value;
    return true;
}

bool CaptureEngine::set_power_line_frequency(int mode) {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_ || mode < 0 || mode > 3) return false;
    DEVSELINFO sel{devsel_index_};
    if (!os_write_power_line_frequency(sel, mode)) return false;
    cfg_.power_line_frequency = mode;
    return true;
}

bool CaptureEngine::reset_usb() {
    // The self-reset detaches the USB link and relies on the watchdog to
    // rebind the device. The watchdog only runs while streaming, so before
    // start() nothing would recover it -- refuse rather than strand the
    // device half-open with no path back.
    if (!streaming_.load(std::memory_order_acquire)) return false;
    return os_reset_usb();
}

void CaptureEngine::set_temporal_params(double alpha, int delta, int persistence) {
    if (!temporal_on_) {
        throw std::runtime_error(
            "The temporal filter was not enabled at start(); the disparity data "
            "type is fixed at open, so enable it via Config.with_filters() "
            "and restart to retune it at runtime");
    }
    std::lock_guard<std::mutex> lk(temporal_mtx_);
    temporal_params_.alpha_q8 = static_cast<int>(std::lround(
        std::clamp(alpha, 0.0, 1.0) * 256.0));
    temporal_params_.delta = std::clamp(delta, 0, 4095);
    temporal_params_.persistence = std::clamp(persistence, 0, 8);
    cfg_.temporal_alpha = alpha;
    cfg_.temporal_delta = delta;
    cfg_.temporal_persistence = persistence;
}

void CaptureEngine::watchdog_loop() {
    while (streaming_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!streaming_.load(std::memory_order_acquire)) break;
        // Disconnect when any wanted stream has gone silent: a stalled
        // depth stream while color keeps flowing (a known firmware failure
        // mode) must still trigger recovery, or wait_for_frames() -- which
        // needs both -- would return None forever with no reopen.
        const int64_t now = steady_now_ns();
        int64_t since = 0;
        if (cfg_.color_w > 0 && cfg_.color_h > 0)
            since = std::max(since,
                now - last_color_ns_.load(std::memory_order_relaxed));
        if (cfg_.depth_w > 0 && cfg_.depth_h > 0)
            since = std::max(since,
                now - last_depth_ns_.load(std::memory_order_relaxed));
        if (since < kDisconnectNs) continue;

        connected_.store(false, std::memory_order_release);
        reconnecting_.store(true, std::memory_order_release);
        // Give the capture loops a moment to observe reconnecting_ and park.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // The quality worker also observes reconnecting_ and returns at
        // the next register boundary; collect it before the rebuild so
        // no register write can interleave with the reopen.
        {
            std::lock_guard<std::mutex> qlk(quality_thread_mtx_);
            if (quality_thread_.joinable()) quality_thread_.join();
        }

        if (!got_any_frame_ && os_park_when_never_streamed()) {
            // A reopen would repeat the same failed negotiation. The
            // app sees wait_for_frames() -> None with is_connected False.
            PYEYS3D_ERROR(tag(),
                "Opened but no frame arrived; the device cannot stream "
                "this mode as negotiated. Pipeline stays disconnected.");
            reconnecting_.store(false, std::memory_order_release);
            break;
        }
        PYEYS3D_WARN(tag(), "Device appears disconnected; attempting reopen");
        // Signal fetch loops to exit rather than park, then join them.
        // os_start_fetch() will restart them with the correct color-first
        // sequencing, matching the initial start() initialization path.
        reinit_threads_.store(true, std::memory_order_release);
        os_stop_fetch();
        reinit_threads_.store(false, std::memory_order_relaxed);
        // Give the device time to settle between teardown and rebuild.
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        if (!streaming_.load(std::memory_order_acquire)) {
            // Leaving the flag set would make close()'s os_stop_fetch()
            // take the reconnect shortcut and skip the interleave reset.
            reconnecting_.store(false, std::memory_order_release);
            break;
        }
        // reopen_device() reports failure by returning false, but taking
        // the process's setup turn can throw. Nothing above this frame
        // catches: an exception leaving a thread function is a terminate,
        // so a stuck sibling camera would take the whole process down
        // rather than cost this one a retry.
        bool ok = false;
        try {
            ok = reopen_device();
        } catch (const std::exception& e) {
            PYEYS3D_WARN(tag(), "reopen aborted: %s", e.what());
        }
        reconnecting_.store(false, std::memory_order_release);
        if (ok) {
            connected_.store(true, std::memory_order_release);
            reconnect_count_.fetch_add(1, std::memory_order_relaxed);
            // The firmware power-cycles register state on USB
            // re-enumeration; re-arm the depth-quality profile.
            quality_pending_.store(!cfg_.quality_regs.empty(),
                                   std::memory_order_release);
            os_start_fetch();
            // Seed the grace AFTER fetch is up (os_start_fetch() can block on
            // the color-first wait); seeding before would leave the depth
            // stamp stale by the first post-reopen watchdog check.
            const int64_t reseed = steady_now_ns() + kLivenessGraceNs;
            last_color_ns_.store(reseed, std::memory_order_relaxed);
            last_depth_ns_.store(reseed, std::memory_order_relaxed);
            PYEYS3D_INFO(tag(), "Device reopened successfully");
        } else {
            // Reopen failed: grace the stamps so a truly-gone device is not
            // hammered with a reopen attempt every watchdog tick, then retry.
            const int64_t reseed = steady_now_ns() + kLivenessGraceNs;
            last_color_ns_.store(reseed, std::memory_order_relaxed);
            last_depth_ns_.store(reseed, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

// Apply the depth-quality register profile. Runs on its own thread,
// launched by the first depth frame of a session; each register is a
// read-modify-write on the masked bits followed by a read-back verify,
// retried with pacing because the firmware applies control writes
// asynchronously to streaming. control_mtx_ is held per attempt, not
// across the profile, so runtime setters interleave freely.
void CaptureEngine::quality_worker() {
    // Control traffic during stream warmup stalls frame delivery; hold
    // the profile until the stream settles. Sliced so teardown and
    // reconnect stay responsive (a reopen re-arms the profile).
    constexpr int kWarmupSlices = 30;   // x 100 ms = 3 s
    for (int i = 0; i < kWarmupSlices; ++i) {
        if (!streaming_.load(std::memory_order_acquire)
            || reconnecting_.load(std::memory_order_acquire))
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    constexpr int kMaxRetry = 100;
    const int flags = FG_Address_2Byte | FG_Value_1Byte;
    int ok = 0, failed = 0;
    bool first = true;
    for (const auto& reg : cfg_.quality_regs) {
        // Space the writes so streaming interleaves with the control
        // traffic.
        if (!first)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        first = false;
        const auto addr = static_cast<unsigned short>(reg[0]);
        const int mask = reg[1];
        const int data = reg[2];
        bool done = false;
        unsigned short target = 0;
        for (int attempt = 0; attempt < kMaxRetry && !done; ++attempt) {
            if (attempt > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            // Bail between attempts on teardown or reconnect; the flag
            // re-arms after a reopen and the whole profile re-applies.
            if (!streaming_.load(std::memory_order_acquire)
                || reconnecting_.load(std::memory_order_acquire))
                return;
            std::lock_guard<std::timed_mutex> lk(control_mtx_);
            if (!device_open_ || !handle_) return;
            if (attempt == 0) {
                unsigned short existing = 0;
                if (APC_GetHWRegister(handle_, &shared_sel_, addr,
                                      &existing, flags) != APC_OK)
                    break;
                target = static_cast<unsigned short>(
                    (existing & ~mask) | data);
            }
            APC_SetHWRegister(handle_, &shared_sel_, addr, target, flags);
            unsigned short verify = 0;
            APC_GetHWRegister(handle_, &shared_sel_, addr, &verify, flags);
            done = (verify == target);
        }
        if (done) {
            ++ok;
        } else {
            ++failed;
            PYEYS3D_WARN(tag(), "depth-quality register 0x%04x did not hold "
                         "its value after %d attempts; it keeps the firmware "
                         "default and depth may differ from the profile",
                         addr, kMaxRetry);
        }
        quality_ok_.store(ok, std::memory_order_relaxed);
        quality_failed_.store(failed, std::memory_order_relaxed);
    }
    if (failed == 0) {
        PYEYS3D_INFO(tag(), "depth-quality profile applied (%d registers)", ok);
    } else {
        PYEYS3D_WARN(tag(), "depth-quality profile: %d applied, %d failed",
                     ok, failed);
    }
}

// Size the ingest working state for the opened mode and pre-publish
// same-sized frames, so every publish is a buffer swap rather than a
// deep copy. Runs once per start(); a watchdog reopen keeps the same
// mode, so the buffers stay valid across reconnects.
void CaptureEngine::prepare_buffers() {
    timing_ = std::getenv("PYEYS3D_TIMING") != nullptr;
    decode_fails_ = 0; last_decode_warn_ns_ = 0;
    short_color_frames_ = 0; last_short_color_warn_ns_ = 0;
    short_depth_frames_ = 0; last_short_depth_warn_ns_ = 0;
    dec_sum_ns_ = 0; dec_max_ns_ = 0; dec_cnt_ = 0;
    flt_sum_ns_ = 0; flt_max_ns_ = 0; flt_cnt_ = 0;
    reset_time_model();
    last_mapped_ns_ = 0;
    last_color_ingest_sn_ = -1;
    last_depth_ingest_sn_ = -1;
    color_dropped_.store(0, std::memory_order_relaxed);
    depth_dropped_.store(0, std::memory_order_relaxed);

    if (cfg_.color_w > 0 && cfg_.color_h > 0) {
        const int w = cfg_.color_w;
        const int h = cfg_.color_h;
        const bool split = cfg_.color_split_lr && (w % 4 == 0);
        const int out_w = split ? w / 2 : w;   // per-eye width in split modes

        // For MJPEG split, the wide frame is decoded into this intermediate
        // then sliced row-by-row into the two half-width outputs.
        if (split && cfg_.color_fmt == 1)
            wide_rgb_.resize(static_cast<size_t>(w) * h * 3);
        // Monochrome modules (G62 / R77): decode to a single gray plane and
        // replicate to rgb8, skipping chroma upsampling + the YCbCr->RGB
        // matrix.
        if (mono_) gray_.resize(static_cast<size_t>(w) * h);

        const size_t out_bytes = static_cast<size_t>(out_w) * h * 3;
        color_left_.domain = FrameDomain::COLOR_RGB8;
        color_left_.width = out_w; color_left_.height = h;
        color_left_.bytes_per_pixel = 3;
        color_left_.data.assign(out_bytes, 0);
        if (split) {
            color_right_.domain = FrameDomain::COLOR_RGB8;
            color_right_.width = out_w; color_right_.height = h;
            color_right_.bytes_per_pixel = 3;
            color_right_.data.assign(out_bytes, 0);
        }
        std::lock_guard<std::mutex> lk(mtx_);
        latest_color_ = color_left_;
        if (split) latest_color_right_ = color_right_;
    }

    if (cfg_.depth_w > 0 && cfg_.depth_h > 0) {
        // The pipeline always delivers DEPTH_MM; the raw read is D11 when a
        // disparity-domain filter is active, otherwise Z14 mm.
        depth_staging_.domain = FrameDomain::DEPTH_MM;
        depth_staging_.width = cfg_.depth_w;
        depth_staging_.height = cfg_.depth_h;
        depth_staging_.bytes_per_pixel = 2;
        depth_staging_.data.assign(
            depth_buffer_bytes(cfg_.depth_w, cfg_.depth_h), 0);
        std::lock_guard<std::mutex> lk(mtx_);
        latest_depth_ = depth_staging_;
    }
}

namespace {
constexpr int64_t kTimeSampleIntervalNs = 500'000'000LL;   // one sample / 500 ms
constexpr size_t  kTimeMaxSamples = 240;                   // ~2 min window
constexpr size_t  kTimeMinSamplesForSlope = 10;            // ~5 s to trust a fit
// Crystal tolerance headroom; a fit outside this is jitter, not drift.
constexpr double  kTimeMaxSlopeError = 5e-4;               // ±500 ppm
// A clock restart lands near zero — a backward step of the whole device
// uptime; stall-drained frames step back only by the stall length.
constexpr int64_t kTimeRestartJumpUs = 30'000'000LL;
// A frame delivered long after capture pairs an old hw stamp with a fresh
// host clock; keep such pairs out of the fit. Only a disagreement that
// persists across whole frames signals a host clock step.
constexpr double  kTimeSampleOutlierNs = 200'000'000.0;
constexpr int     kTimeOutlierStreakReset = 8;
// Monotonic clamp step. Must survive the double-seconds view of the
// timestamp (~0.25 us granularity at the current epoch) while staying
// far below a frame period.
constexpr int64_t kTimeMonotonicStepNs = 1'000LL;          // 1 us
}  // namespace

void CaptureEngine::reset_time_model() {
    time_samples_.clear();
    time_sample_pos_ = 0;
    last_time_sample_ns_ = 0;
    max_hw_us_ = 0;
    time_slope_ns_per_us_ = 1000.0;
    time_outlier_streak_ = 0;
}

// Least squares over the sample window, line through the means. With
// fewer samples than a trustworthy fit needs, keep the nominal slope
// and anchor on the first sample.
void CaptureEngine::refit_time_model() {
    const size_t n = time_samples_.size();
    if (n < kTimeMinSamplesForSlope) {
        time_base_hw_us_ = static_cast<double>(time_samples_.front().hw_us);
        time_base_host_ns_ = static_cast<double>(time_samples_.front().host_ns);
        time_slope_ns_per_us_ = 1000.0;
        return;
    }
    double mean_hw = 0.0, mean_host = 0.0;
    for (const auto& s : time_samples_) {
        mean_hw += static_cast<double>(s.hw_us);
        mean_host += static_cast<double>(s.host_ns);
    }
    mean_hw /= static_cast<double>(n);
    mean_host /= static_cast<double>(n);
    double sxx = 0.0, sxy = 0.0;
    for (const auto& s : time_samples_) {
        const double dx = static_cast<double>(s.hw_us) - mean_hw;
        sxx += dx * dx;
        sxy += dx * (static_cast<double>(s.host_ns) - mean_host);
    }
    if (sxx <= 0.0) return;
    time_slope_ns_per_us_ = std::clamp(sxy / sxx,
                                       1000.0 * (1.0 - kTimeMaxSlopeError),
                                       1000.0 * (1.0 + kTimeMaxSlopeError));
    time_base_hw_us_ = mean_hw;
    time_base_host_ns_ = mean_host;
}

// EWMA over publish intervals; reads 0 past kFpsStaleSec of silence.
namespace {
constexpr double kFpsEwmaAlpha = 0.1;
constexpr double kFpsStaleSec  = 1.0;
}  // namespace

void CaptureEngine::note_publish(std::atomic<double>& interval_s,
                                 std::atomic<int64_t>& last_ns) {
    const int64_t now  = steady_now_ns();
    const int64_t prev = last_ns.exchange(now, std::memory_order_relaxed);
    if (prev == 0) return;              // first frame: no interval yet
    const double dt = static_cast<double>(now - prev) * 1e-9;
    if (dt <= 0.0) return;
    const double old = interval_s.load(std::memory_order_relaxed);
    interval_s.store(old > 0.0 ? old + kFpsEwmaAlpha * (dt - old) : dt,
                     std::memory_order_relaxed);
}

double CaptureEngine::fps_now(const std::atomic<double>& interval_s,
                              const std::atomic<int64_t>& last_ns) {
    const int64_t last = last_ns.load(std::memory_order_relaxed);
    if (last == 0) return 0.0;
    const double age = static_cast<double>(steady_now_ns() - last) * 1e-9;
    if (age > kFpsStaleSec) return 0.0;
    const double dt = interval_s.load(std::memory_order_relaxed);
    return dt > 0.0 ? 1.0 / dt : 0.0;
}

int64_t CaptureEngine::host_time_from_hw(uint64_t hw_us) {
    const int64_t hw = static_cast<int64_t>(hw_us);
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // No hardware timestamp on this frame: report the arrival time and
    // keep the zero out of the clock model.
    if (hw == 0) {
        const int64_t t = std::max(now_ns, last_mapped_ns_ + kTimeMonotonicStepNs);
        last_mapped_ns_ = t;
        return t;
    }

    // A restart-sized backward jump means the device clock restarted
    // (replug); smaller steps (inter-stream skew, stall-drained frames)
    // still belong to the current clock.
    if (!time_samples_.empty() && hw < max_hw_us_ - kTimeRestartJumpUs) {
        reset_time_model();
    }
    if (hw > max_hw_us_) max_hw_us_ = hw;

    if (time_samples_.empty()
        || now_ns - last_time_sample_ns_ >= kTimeSampleIntervalNs) {
        const double predicted = time_base_host_ns_
            + time_slope_ns_per_us_
                  * (static_cast<double>(hw) - time_base_hw_us_);
        if (!time_samples_.empty()
            && std::abs(static_cast<double>(now_ns) - predicted)
                   > kTimeSampleOutlierNs) {
            // Stale delivery: map through the model but keep the sample
            // out of the fit. When every frame disagrees, the host clock
            // stepped — rebuild the model on the current pair.
            if (++time_outlier_streak_ >= kTimeOutlierStreakReset) {
                reset_time_model();
                time_outlier_streak_ = 0;
                time_samples_.push_back({hw, now_ns});
                last_time_sample_ns_ = now_ns;
                refit_time_model();
            }
        } else {
            time_outlier_streak_ = 0;
            if (time_samples_.size() < kTimeMaxSamples) {
                time_samples_.push_back({hw, now_ns});
            } else {
                time_samples_[time_sample_pos_] = {hw, now_ns};
                time_sample_pos_ = (time_sample_pos_ + 1) % kTimeMaxSamples;
            }
            last_time_sample_ns_ = now_ns;
            refit_time_model();
        }
    }

    int64_t mapped = static_cast<int64_t>(
        time_base_host_ns_
        + time_slope_ns_per_us_ * (static_cast<double>(hw) - time_base_hw_us_));
    // Refit shifts and stale deliveries can land a mapped time at or
    // behind one already delivered; hold it a step ahead instead.
    if (mapped < last_mapped_ns_ + kTimeMonotonicStepNs)
        mapped = last_mapped_ns_ + kTimeMonotonicStepNs;
    last_mapped_ns_ = mapped;
    return mapped;
}

// Count camera-produced frames the host never saw, as gaps in the
// stream's serial numbers (sn_step_ is the per-OS expected increment).
// A serial below the last seen one means the device restarted counting
// (reconnect) — re-baseline without counting.
void CaptureEngine::note_stream_serial(int serial, int& last_sn,
                                       std::atomic<uint64_t>& dropped) {
    if (last_sn >= 0 && serial > last_sn + sn_step_) {
        dropped.fetch_add(
            static_cast<uint64_t>(serial - last_sn - sn_step_) / sn_step_,
            std::memory_order_relaxed);
    }
    last_sn = serial;
}

// Decode one fetched color buffer into rgb8 and publish it. Pure
// convert + publish: liveness stamping and interleave parity are the
// fetch layer's job. Corrupt frames are dropped with a rate-limited
// warning (at most one a second).
void CaptureEngine::ingest_color(const uint8_t* buf, size_t got, int serial,
                                 int64_t tv_sec, int64_t tv_usec) {
    got_any_frame_.store(true, std::memory_order_relaxed);
    // Count wire gaps before any decode: a frame that arrives corrupt
    // was still received (decode failures have their own counter).
    note_stream_serial(serial, last_color_ingest_sn_, color_dropped_);
    const int w = cfg_.color_w;
    const int h = cfg_.color_h;
    const bool split = cfg_.color_split_lr && (w % 4 == 0);
    const int out_w = split ? w / 2 : w;
    Frame& left = color_left_;
    Frame& right = color_right_;

    auto note_decode_fail = [&]() {
        ++decode_fails_;
        const int64_t now = steady_now_ns();
        if (now - last_decode_warn_ns_ >= 1'000'000'000LL) {
            PYEYS3D_WARN(tag(), "MJPEG decode failed (%llu dropped so far)",
                         static_cast<unsigned long long>(decode_fails_));
            last_decode_warn_ns_ = now;
        }
    };

    // The SDK can deliver either a JPEG stream or raw YUYV for the
    // same catalog "MJPEG" mode depending on firmware / negotiation,
    // so detect the JPEG SOI marker per frame rather than trusting the
    // catalog flag. No SOI -> treat the buffer as YUYV.
    const bool is_jpeg = (cfg_.color_fmt == 1) && got >= 2
                         && buf[0] == 0xFF && buf[1] == 0xD8;

    // The YUYV kernels read exactly w*h*2 bytes; the length is the
    // producer's word (an SDK callback's imgSize, or a short V4L2 read),
    // so a frame that falls short must be dropped rather than converted
    // past the end of it. JPEG carries its own length and is left to the
    // decoder.
    if (!is_jpeg && got < static_cast<size_t>(w) * h * 2) {
        ++short_color_frames_;
        const int64_t now = steady_now_ns();
        if (now - last_short_color_warn_ns_ >= 1'000'000'000LL) {
            PYEYS3D_WARN(tag(),
                "color frame short: %zu of %zu bytes "
                "(%llu color frames dropped so far)",
                got, static_cast<size_t>(w) * h * 2,
                static_cast<unsigned long long>(short_color_frames_));
            last_short_color_warn_ns_ = now;
        }
        return;
    }

    const int64_t dec_t0 = timing_ ? steady_now_ns() : 0;
    if (mono_) {
        // Luma-only fast path: build a w*h gray plane, then replicate.
        if (is_jpeg) {
            if (!os_decode_jpeg(buf, got, gray_.data(), w, h, /*gray=*/true)) {
                note_decode_fail();
                return;
            }
        } else {
            pyeys3d::simd::yuyv_extract_y(buf, gray_.data(), w, h);
        }
        if (!split) {
            pyeys3d::simd::gray_to_rgb8(gray_.data(), left.data.data(), w, h);
        } else {
            pyeys3d::simd::gray_to_rgb8_split(
                gray_.data(), left.data.data(), right.data.data(), out_w, h);
        }
    } else if (!split) {
        if (is_jpeg) {
            if (!os_decode_jpeg(buf, got, left.data.data(), w, h,
                                /*gray=*/false)) {
                note_decode_fail();
                return;
            }
        } else {
            pyeys3d::simd::yuyv_to_rgb8(buf, left.data.data(), w, h);
        }
    } else if (!is_jpeg) {
        // YUYV split: one pass writes both halves, no wide intermediate.
        pyeys3d::simd::yuyv_to_rgb8_split(
            buf, left.data.data(), right.data.data(), out_w, h);
    } else {
        // MJPEG split: decode wide, then slice each row in half.
        if (!os_decode_jpeg(buf, got, wide_rgb_.data(), w, h,
                            /*gray=*/false)) {
            note_decode_fail();
            return;
        }
        const size_t row_half = static_cast<size_t>(out_w) * 3;
        const size_t wide_row = static_cast<size_t>(w) * 3;
        for (int r = 0; r < h; ++r) {
            const uint8_t* src = wide_rgb_.data() + static_cast<size_t>(r) * wide_row;
            std::memcpy(left.data.data()  + static_cast<size_t>(r) * row_half,
                        src, row_half);
            std::memcpy(right.data.data() + static_cast<size_t>(r) * row_half,
                        src + row_half, row_half);
        }
    }
    if (timing_) {
        const int64_t e = steady_now_ns() - dec_t0;
        dec_sum_ns_ += e; dec_cnt_++; if (e > dec_max_ns_) dec_max_ns_ = e;
    }

    const uint64_t ts =
        static_cast<uint64_t>(tv_sec) * 1'000'000ULL + static_cast<uint64_t>(tv_usec);
    left.frame_number = serial; left.hw_timestamp_us = ts;
    if (split) { right.frame_number = serial; right.hw_timestamp_us = ts; }
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const int64_t host_ns = host_time_from_hw(ts);
        left.host_timestamp_ns = host_ns;
        if (split) right.host_timestamp_ns = host_ns;
        // Swap the freshly decoded frame in; `left`/`right` take the
        // previous (identically sized) published buffers to fill next.
        std::swap(latest_color_, left);
        if (split) std::swap(latest_color_right_, right);
        color_fresh_ = true;
        note_publish(color_interval_, color_publish_ns_);
    }
    cv_.notify_one();
}

// Convert one fetched depth buffer to clipped DEPTH_MM and publish it.
// The Linux loop fetches straight into depth_staging_, so the copy below
// is skipped; a foreign buffer (the Windows callback hands the SDK's own)
// is copied into staging first.
void CaptureEngine::ingest_depth(const uint8_t* buf, size_t got, int serial,
                                 int64_t tv_sec, int64_t tv_usec) {
    got_any_frame_.store(true, std::memory_order_relaxed);
    // First depth frame of the session: the depth pipeline is confirmed
    // running, so the firmware will accept the depth-quality registers.
    // The previous session's worker is always collected (close/watchdog)
    // before the flag re-arms, so the join here returns immediately.
    if (quality_pending_.exchange(false, std::memory_order_acq_rel)) {
        std::lock_guard<std::mutex> qlk(quality_thread_mtx_);
        // Re-checked under the lock: a close() that has already passed its
        // own join would otherwise get a worker started behind it.
        if (streaming_.load(std::memory_order_acquire)) {
            if (quality_thread_.joinable()) quality_thread_.join();
            quality_thread_ = std::thread(&CaptureEngine::quality_worker, this);
        }
    }
    note_stream_serial(serial, last_depth_ingest_sn_, depth_dropped_);
    Frame& staging = depth_staging_;
    // staging alternates with the published buffer on every swap, so a
    // frame that fills only part of it leaves the tail holding pixels from
    // two frames ago — which would then be clipped, filtered and delivered
    // under this frame's timestamp. Drop it instead; the counters already
    // record the gap as a wire drop.
    if (got < staging.data.size()) {
        ++short_depth_frames_;
        const int64_t now = steady_now_ns();
        if (now - last_short_depth_warn_ns_ >= 1'000'000'000LL) {
            PYEYS3D_WARN(tag(),
                "depth frame short: %zu of %zu bytes "
                "(%llu depth frames dropped so far)",
                got, staging.data.size(),
                static_cast<unsigned long long>(short_depth_frames_));
            last_short_depth_warn_ns_ = now;
        }
        return;
    }
    if (buf != staging.data.data()) {
        std::memcpy(staging.data.data(), buf, staging.data.size());
    }

    const int w = staging.width;
    const int h = staging.height;
    const uint16_t z_min = static_cast<uint16_t>(std::clamp(depth_min_mm_, 0, 65535));
    const uint16_t z_max = static_cast<uint16_t>(std::clamp(depth_max_mm_, 0, 65535));
    // Depth row 0 carries a 16-byte serial-number watermark on both the
    // disparity and Z14 paths: the firmware writes the frame's serial into
    // the low bit of each of those bytes, which is where the SDK reads it
    // back from. Eight uint16 pixels cover them. They are not depth, so
    // they are zeroed before anything reads them as such — on the
    // disparity path that must happen before filtering, which would smear
    // them across the image.
    constexpr int kSerialSkipPixels = 8;

    uint16_t* pix = reinterpret_cast<uint16_t*>(staging.data.data());
    // Pixel-independent conversions below; OMP parallelises across
    // pixels (signed index for OpenMP). w*h fits int for every mode.
    const int n = w * h;

    const int64_t flt_t0 = timing_ ? steady_now_ns() : 0;
    if (chain_disparity_) {
        // Raw D11 -> Q4, mask the SN watermark, run the disparity-domain
        // filters, then ZD-lookup back to clipped millimeters in place.
        filt_work_.resize(static_cast<size_t>(n));
        uint16_t* q4 = filt_work_.data();
        pyeys3d::disparity_promote_to_q4(pix, q4, w, h);
        std::fill_n(q4, kSerialSkipPixels, uint16_t{0});
        if (spatial_on_) {
            pyeys3d::spatial_filter_q4(q4, w, h, spatial_params_);
        }
        if (temporal_on_) {
            pyeys3d::TemporalFilterParams tp;
            {   // snapshot: set_temporal_params can retune mid-stream
                std::lock_guard<std::mutex> tlk(temporal_mtx_);
                tp = temporal_params_;
            }
            // Disparity units -> Q4. delta is held under 4096 where it
            // is set, so the promoted value stays inside the uint16 the
            // kernel compares in; above that the NEON lanes would see 0
            // and stop blending while the scalar tail kept going.
            tp.delta <<= 4;
            temporal_state_.resize(w, h);
            pyeys3d::temporal_filter_apply(q4, w, h, temporal_state_, tp);
        }
#ifdef _OPENMP
        #pragma omp parallel for schedule(static)
#endif
        for (int i = 0; i < n; ++i) {
            const uint16_t d_q4 = q4[i];
            if (d_q4 == 0) { pix[i] = 0; continue; }
            // Hold the table's millimetres to the same 14 bits the Z14
            // path below carries: every downstream consumer masks depth
            // with 0x3FFF, so a wider value would read as a status bit.
            const uint16_t z = static_cast<uint16_t>(
                std::clamp(pyeys3d::zd_lookup_q4(zd_table_, d_q4), 0, 0x3FFF));
            pix[i] = (z < z_min || z > z_max) ? 0 : z;
        }
    } else {
        // Z14 firmware mm: strip the status bits and apply the clip.
#ifdef _OPENMP
        #pragma omp parallel for schedule(static)
#endif
        for (int i = 0; i < n; ++i) {
            const uint16_t z = pix[i] & 0x3FFF;
            pix[i] = (z < z_min || z > z_max) ? 0 : z;
        }
    }
    // Clear the watermark before hole filling as well, not only after:
    // the filter reads a frozen copy and carries valid neighbours into
    // holes, so a serial byte that survived the clip would be copied to
    // coordinates the trailing fill_n does not cover, and reach the caller
    // as depth. The disparity path above already clears it pre-filter, and
    // hole_fill_z's left_skip cannot serve here — the around modes, which
    // every example and the viewer default to, ignore it.
    std::fill_n(pix, kSerialSkipPixels, uint16_t{0});
    if (hole_on_) {
        pyeys3d::hole_fill_z(pix, w, h, hole_mode_, 0, hole_scratch_);
    }
    // Again after: those first pixels are holes to the filter, which may
    // have filled them from a neighbour. Row 0 carries an SN watermark,
    // not depth, and dropping it is what makes a caller's `depth != 0`
    // mask match the point cloud.
    std::fill_n(pix, kSerialSkipPixels, uint16_t{0});
    if (timing_) {
        const int64_t e = steady_now_ns() - flt_t0;
        flt_sum_ns_ += e; flt_cnt_++; if (e > flt_max_ns_) flt_max_ns_ = e;
    }

    staging.domain = FrameDomain::DEPTH_MM;
    staging.frame_number = serial;
    staging.hw_timestamp_us =
        static_cast<uint64_t>(tv_sec) * 1'000'000ULL + static_cast<uint64_t>(tv_usec);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        staging.host_timestamp_ns = host_time_from_hw(staging.hw_timestamp_us);
        // Swap the converted frame in; `staging` takes the previous
        // (identically sized) published buffer to fill next.
        std::swap(latest_depth_, staging);
        depth_fresh_ = true;
        note_publish(depth_interval_, depth_publish_ns_);
    }
    cv_.notify_one();
}

bool CaptureEngine::wait_for_frames(int timeout_ms, Frame& out_color, Frame& out_depth,
                                    Frame& out_color_right, bool& has_right) {
    std::unique_lock<std::mutex> lk(mtx_);
    const bool want_color = cfg_.color_w > 0 && cfg_.color_h > 0;
    const bool want_depth = cfg_.depth_w > 0 && cfg_.depth_h > 0;
    const bool split = cfg_.color_split_lr && (cfg_.color_w % 4 == 0);
    has_right = false;

    auto ready = [&]() {
        bool c_ok = !want_color || (color_fresh_
            && latest_color_.frame_number != last_color_sn_returned_);
        bool d_ok = !want_depth || (depth_fresh_
            && latest_depth_.frame_number != last_depth_sn_returned_);
        return (c_ok && d_ok) || !streaming_.load(std::memory_order_acquire);
    };

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);
    if (!cv_.wait_until(lk, deadline, ready)) return false;
    if (!streaming_.load(std::memory_order_acquire)) return false;

    if (want_color) {
        out_color = latest_color_;
        last_color_sn_returned_ = latest_color_.frame_number;
        color_fresh_ = false;
        if (split) { out_color_right = latest_color_right_; has_right = true; }
    }
    if (want_depth) {
        out_depth = latest_depth_;
        last_depth_sn_returned_ = latest_depth_.frame_number;
        depth_fresh_ = false;
    }
    return true;
}

}  // namespace pyeys3d
