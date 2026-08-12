// Linux backend of CaptureEngine: the V4L2 pull model.
//
// Two fetch threads poll APC_Get{Color,Depth}ImageWithTimestamp and
// hand each buffer to the OS-independent ingest layer. Interleave-mode
// parity filtering (color = even SN, depth = odd SN) happens here.

#include "capture_engine.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "apc_error.hpp"
#include "capture_internal.hpp"
#include "eSPDI.h"
#include "log.hpp"
#include "turbojpeg.h"   // symbols re-exported from libeSPDI; no external link

namespace pyeys3d {

// ---------------------------------------------------------------------------
//   JPEG decode (turbojpeg re-exported by libeSPDI)
// ---------------------------------------------------------------------------

bool CaptureEngine::os_init_decoder(std::string& err) {
    if (!tj_) tj_ = tjInitDecompress();
    if (!tj_) {
        err = "turbojpeg decoder init failed (tjInitDecompress); this "
              "camera's MJPEG modes cannot be decoded, so pick a YUYV "
              "mode at this resolution";
        return false;
    }
    return true;
}

void CaptureEngine::os_release_decoder() {
    if (tj_) { tjDestroy(tj_); tj_ = nullptr; }
}

bool CaptureEngine::os_decode_jpeg(const uint8_t* buf, size_t got, uint8_t* out,
                                   int w, int h, bool gray) {
    if (!tj_) return false;
    return tjDecompress2(tj_, buf, got, out, w, gray ? w : w * 3, h,
                         gray ? TJPF_GRAY : TJPF_RGB, 0) == 0;
}

// The Linux SDK does not bind stream metadata to a handle's first
// selection, so a single handle can enumerate and open directly.
bool CaptureEngine::os_fresh_handle_required() { return false; }

// Inline close + release; on Linux this returns promptly even for a
// removed device.
void CaptureEngine::os_teardown_device() {
    DEVSELINFO sel{devsel_index_};
    if (device_open_) {
        APC_CloseDevice(handle_, &sel);
        device_open_ = false;
    }
    if (handle_) { APC_Release(&handle_); handle_ = nullptr; }
}

// A pipeline that never delivered a frame is usually a transient here
// (UVC bandwidth negotiation); reopening recovers it.
bool CaptureEngine::os_park_when_never_streamed() { return false; }

// Inline USB self-reset: the detach write returns promptly on Linux, so
// the whole sequence runs under the control lock and the watchdog then
// reopens the re-enumerated device. Errors are ignored — the detach
// write's acknowledge may never arrive.
bool CaptureEngine::os_reset_usb() {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return false;
    DEVSELINFO sel{devsel_index_};
    const int flags = FG_Address_2Byte | FG_Value_1Byte;
    for (const auto& r : kResetSeq)
        APC_SetHWRegister(handle_, &sel, r.addr, r.val, flags);
    return true;
}

// Configure and open the streams of the already selected device: depth
// data type and interleave mode, then OpenDevice2. A wrong value for
// either corrupts both streams, so both writes must succeed.
bool CaptureEngine::os_open_streams(std::string& err) {
    DEVSELINFO sel{devsel_index_};

    int rc = APC_SetDepthDataType(
        handle_, &sel, static_cast<unsigned short>(effective_depth_dtype_));
    if (rc != APC_OK) {
        err = "APC_SetDepthDataType(" + std::to_string(effective_depth_dtype_)
            + ") failed: " + apc_strerror(rc);
        return false;
    }
    rc = APC_SetInterleaveMode(handle_, &sel, cfg_.interleave);
    if (rc != APC_OK) {
        err = "APC_SetInterleaveMode failed: " + apc_strerror(rc);
        return false;
    }
    // The host splits the shared interleave stream by SN parity, so each
    // stream sees every other serial number.
    sn_step_ = cfg_.interleave ? 2 : 1;

    // The IR level arrives fully resolved (explicit value or the
    // mode-aware default; 0 = projector off) and is always written,
    // unlike the leave-as-is UVC controls applied after open.
    if (cfg_.ir_value >= 0) {
        const int ir_rc = APC_SetCurrentIRValue(
            handle_, &sel, static_cast<unsigned short>(cfg_.ir_value));
        // A projector that refuses the write leaves depth uniformly poor
        // with nothing to point at; the runtime setter reports the same
        // failure, so opening should not be the quiet one.
        if (ir_rc != APC_OK)
            PYEYS3D_WARN(tag(), "IR level %d rejected at open: %s",
                         cfg_.ir_value, apc_strerror(ir_rc).c_str());
    }

    int fps = cfg_.fps;
    // DEPTH_TRANSFER_CTRL is a rendering switch, not an encoding one: the
    // GRAY and COLORFUL values make the SDK replace the depth payload with
    // a 3-byte-per-pixel colormap and report three times the size, which
    // this engine's uint16 depth views and w*h*2 buffers cannot take.
    // NON_TRANSFER is the only value that delivers the module's own depth.
    // The encoding itself (11 vs 14 bit, rectified vs raw) is FW register
    // 0xF0, set by APC_SetDepthDataType above.
    //
    // bIsOutputRGB24 and phWndNotice appear only in APC_OpenDevice2's
    // signature; cm is read once, to test for IMAGE_USERPTR_MODE. The SN
    // values are inert (eSPDI_def.h marks them "Not used"), and nothing
    // else could implement them: the per-frame serial the interleave split
    // and the drop counters run on is decoded from the frame's own first
    // 16 bytes, so keeping the two streams on one sequence is a firmware
    // property, not an API one. IMAGE_SN_SYNC is what the SDK's own
    // console_tester passes; it is kept here for the same reason, to state
    // the intent, and false is passed for the colour flag because
    // ingest_color converts the raw payload itself.
    rc = APC_OpenDevice2(
        handle_, &sel,
        cfg_.color_w, cfg_.color_h, static_cast<bool>(cfg_.color_fmt),
        cfg_.depth_w, cfg_.depth_h,
        DEPTH_IMG_NON_TRANSFER,
        /*bIsOutputRGB24=*/false,
        /*phWndNotice=*/nullptr,
        &fps,
        IMAGE_SN_SYNC);
    if (rc != APC_OK) {
        err = "APC_OpenDevice2 failed: " + apc_strerror(rc);
        return false;
    }
    return true;
}

void CaptureEngine::os_start_fetch() {
    const bool has_color = cfg_.color_w > 0 && cfg_.color_h > 0;
    const bool has_depth = cfg_.depth_w > 0 && cfg_.depth_h > 0;

    if (has_color)
        color_thread_ = std::thread(&CaptureEngine::color_capture_loop, this);

    // Firmware requires color to be established before depth starts, or
    // the two cross-contaminate the color buffer. Time to first frame is a
    // firmware property and varies per module and mode, so the cap only
    // bounds a stream that never starts.
    if (has_color && has_depth) {
        constexpr auto kColorFirstCap = std::chrono::seconds(15);
        const auto deadline = std::chrono::steady_clock::now() + kColorFirstCap;
        while (!color_started_.load(std::memory_order_acquire)
               && streaming_.load(std::memory_order_acquire)
               && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (streaming_.load(std::memory_order_acquire)) {
            if (!color_started_.load(std::memory_order_acquire)) {
                PYEYS3D_WARN(tag(),
                    "no color frame within %llds; starting depth anyway. The "
                    "firmware's color-before-depth ordering was not met and "
                    "both streams may be corrupted",
                    static_cast<long long>(kColorFirstCap.count()));
            }
            // Small settle margin after the first color frame.
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    if (has_depth)
        depth_thread_ = std::thread(&CaptureEngine::depth_capture_loop, this);
}

void CaptureEngine::os_stop_fetch() {
    if (color_thread_.joinable()) color_thread_.join();
    if (depth_thread_.joinable()) depth_thread_.join();
}

void CaptureEngine::color_capture_loop() {
    DEVSELINFO& sel = shared_sel_;   // shared address across fetch threads
    // The SDK's fetch takes no capacity, so an over-long frame would run
    // past this buffer and only be reported afterwards. A YUYV frame is
    // exactly w*h*2 and cannot; a JPEG has no such bound, and a frame that
    // compresses badly (noise, a wildly overexposed scene) can exceed the
    // raster. Give the compressed modes the uncompressed RGB size, which
    // no JPEG of the same image reaches.
    const size_t color_bytes = static_cast<size_t>(cfg_.color_w)
                             * cfg_.color_h * (cfg_.color_fmt == 1 ? 3 : 2);
    std::vector<uint8_t> raw(color_bytes);

    bool sel_dirty = false;
    while (streaming_.load(std::memory_order_acquire)) {
        // Park while the watchdog tears the device down + reopens it.
        // Exit (return) when the watchdog wants to restart threads fresh
        // via os_start_fetch() rather than resume from a parked state.
        if (reconnecting_.load(std::memory_order_acquire)) {
            if (reinit_threads_.load(std::memory_order_acquire)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sel_dirty = true;
            continue;
        }
        if (sel_dirty) { sel.index = devsel_index_; sel_dirty = false; }

        unsigned long got = 0;
        int serial = 0;
        int64_t tv_sec = 0, tv_usec = 0;
        // The SDK keys the color fetch on the configured depth data type so
        // it can de-interleave color from depth on the shared stream; this
        // is the same effective dtype passed to the depth fetch, not a
        // depth value leaking into the color path.
        const int rc = APC_GetColorImageWithTimestamp(
            handle_, &sel, raw.data(), &got, &serial,
            effective_depth_dtype_, &tv_sec, &tv_usec);
        if (rc != APC_OK || got == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        color_started_.store(true, std::memory_order_release);
        if (cfg_.interleave && (serial % 2) != 0) continue;   // color = even SN

        // Stamp liveness only for a frame actually kept as color: in
        // interleave the skipped (odd) frames belong to depth, and counting
        // them here would keep color's stamp fresh through a color-only
        // stall and hide it from the watchdog.
        last_color_ns_.store(steady_now_ns(), std::memory_order_relaxed);
        ingest_color(raw.data(), got, serial, tv_sec, tv_usec);
    }

    if (timing_ && dec_cnt_) {
        PYEYS3D_INFO(tag(), "color decode: avg %.2f ms / max %.2f ms over %lu frames",
                     dec_sum_ns_ / dec_cnt_ / 1e6, dec_max_ns_ / 1e6,
                     static_cast<unsigned long>(dec_cnt_));
    }
}

void CaptureEngine::depth_capture_loop() {
    DEVSELINFO& sel = shared_sel_;   // shared address across fetch threads
    const bool have_color = cfg_.color_w > 0 && cfg_.color_h > 0;

    // Report a stream that never yields a frame; the threshold clears the
    // several-second cold start.
    constexpr int64_t kDepthSilenceWarnNs = 10'000'000'000LL;
    int64_t empty_since_ns = 0;
    bool empty_warned = false;

    bool sel_dirty = false;
    while (streaming_.load(std::memory_order_acquire)) {
        if (reconnecting_.load(std::memory_order_acquire)) {
            if (reinit_threads_.load(std::memory_order_acquire)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sel_dirty = true;
            continue;
        }
        if (sel_dirty) {
            sel.index = devsel_index_;
            sel_dirty = false;
            // After a reopen, let the color stream re-establish first.
            if (have_color) {
                while (streaming_.load(std::memory_order_acquire)
                       && !reconnecting_.load(std::memory_order_acquire)
                       && !color_started_.load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }

        unsigned long got = 0;
        int serial = 0;
        int64_t tv_sec = 0, tv_usec = 0;
        const int rc = APC_GetDepthImageWithTimestamp(
            handle_, &sel, depth_staging_.data.data(), &got, &serial,
            effective_depth_dtype_, &tv_sec, &tv_usec);
        if (rc != APC_OK || got == 0) {
            const int64_t now_ns = steady_now_ns();
            if (empty_since_ns == 0) empty_since_ns = now_ns;
            if (!empty_warned && now_ns - empty_since_ns > kDepthSilenceWarnNs) {
                empty_warned = true;
                PYEYS3D_WARN(tag(),
                    "no depth frame for %llds: APC_GetDepthImageWithTimestamp "
                    "%s (got=%lu, dtype=%d, %dx%d)",
                    static_cast<long long>(kDepthSilenceWarnNs / 1'000'000'000LL),
                    apc_strerror(rc).c_str(), got, effective_depth_dtype_,
                    cfg_.depth_w, cfg_.depth_h);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        empty_since_ns = 0;
        empty_warned = false;
        if (cfg_.interleave && (serial % 2) != 1) continue;   // depth = odd SN

        // Stamp liveness only for a frame kept as depth (see the color
        // loop): the skipped (even) frames are color's in interleave.
        last_depth_ns_.store(steady_now_ns(), std::memory_order_relaxed);
        ingest_depth(depth_staging_.data.data(), got, serial, tv_sec, tv_usec);
    }

    if (timing_ && flt_cnt_) {
        PYEYS3D_INFO(tag(), "depth convert+filter: avg %.2f ms / max %.2f ms over %lu frames",
                     flt_sum_ns_ / flt_cnt_ / 1e6, flt_max_ns_ / 1e6,
                     static_cast<unsigned long>(flt_cnt_));
    }
}

// ---------------------------------------------------------------------------
//   Camera-control primitives (Linux UVC property ids)
// ---------------------------------------------------------------------------

bool CaptureEngine::os_write_auto_exposure(DEVSELINFO& sel, bool on) {
    const long int mode = on ? AE_MOD_APERTURE_PRIORITY_MODE : AE_MOD_MANUAL_MODE;
    return APC_SetCTPropVal(handle_, &sel,
                            CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, mode) == APC_OK;
}

bool CaptureEngine::os_write_exposure_value(DEVSELINFO& sel, int value) {
    return APC_SetCTPropVal(handle_, &sel,
                            CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL,
                            value) == APC_OK;
}

bool CaptureEngine::os_write_auto_white_balance(DEVSELINFO& sel, bool on) {
    return APC_SetPUPropVal(handle_, &sel,
                            PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL,
                            on ? 1 : 0) == APC_OK;
}

bool CaptureEngine::os_write_white_balance_value(DEVSELINFO& sel, int value) {
    return APC_SetPUPropVal(handle_, &sel,
                            PU_PROPERTY_ID_WHITE_BALANCE_CTRL, value) == APC_OK;
}

bool CaptureEngine::os_write_power_line_frequency(DEVSELINFO& sel, int mode) {
    return APC_SetPUPropVal(handle_, &sel,
                            PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL,
                            mode) == APC_OK;
}

std::optional<bool> CaptureEngine::os_read_auto_exposure(DEVSELINFO& sel) const {
    ApcPropVal v = 0;
    if (APC_GetCTPropVal(handle_, &sel, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL,
                         &v) != APC_OK)
        return std::nullopt;
    return v == AE_MOD_APERTURE_PRIORITY_MODE;
}

std::optional<int> CaptureEngine::os_read_exposure_value(DEVSELINFO& sel) const {
    ApcPropVal v = 0;
    if (APC_GetCTPropVal(handle_, &sel, CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL,
                         &v) != APC_OK)
        return std::nullopt;
    return static_cast<int>(v);
}

std::optional<bool>
CaptureEngine::os_read_auto_white_balance(DEVSELINFO& sel) const {
    ApcPropVal v = 0;
    if (APC_GetPUPropVal(handle_, &sel, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL,
                         &v) != APC_OK)
        return std::nullopt;
    return v != 0;
}

std::optional<int>
CaptureEngine::os_read_white_balance_value(DEVSELINFO& sel) const {
    ApcPropVal v = 0;
    if (APC_GetPUPropVal(handle_, &sel, PU_PROPERTY_ID_WHITE_BALANCE_CTRL,
                         &v) != APC_OK)
        return std::nullopt;
    return static_cast<int>(v);
}

std::optional<int>
CaptureEngine::os_read_power_line_frequency(DEVSELINFO& sel) const {
    ApcPropVal v = 0;
    if (APC_GetPUPropVal(handle_, &sel, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL,
                         &v) != APC_OK)
        return std::nullopt;
    return static_cast<int>(v);
}

// Range queries over the Linux SDK's CT/PU surface. The Windows backend
// has its own pair; both yield nullopt when the device publishes no range
// for the property, and the Python layer then writes unvalidated.
std::optional<std::tuple<int, int, int, int>>
CaptureEngine::get_exposure_range() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    int mx = 0, mn = 0, step = 0, def = 0, flags = 0;
    if (APC_GetCTRangeAndStep(handle_, &sel,
                              CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL,
                              &mx, &mn, &step, &def, &flags) != APC_OK)
        return std::nullopt;
    return std::make_tuple(mn, mx, step, def);
}

std::optional<std::tuple<int, int, int, int>>
CaptureEngine::get_white_balance_range() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    int mx = 0, mn = 0, step = 0, def = 0, flags = 0;
    if (APC_GetPURangeAndStep(handle_, &sel, PU_PROPERTY_ID_WHITE_BALANCE_CTRL,
                              &mx, &mn, &step, &def, &flags) != APC_OK)
        return std::nullopt;
    return std::make_tuple(mn, mx, step, def);
}

}  // namespace pyeys3d
