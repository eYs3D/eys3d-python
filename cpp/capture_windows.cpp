// Windows backend of CaptureEngine: the SDK callback push model.
//
// Frame delivery starts inside APC_OpenDevice: one registered callback
// receives color and depth frames as raw wire data and hands them to
// the OS-independent ingest layer. No fetch threads and no host-side
// parity filtering — the SDK splits the interleave stream itself.

#include "capture_engine.hpp"

#include <atomic>
#include <chrono>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "apc_error.hpp"
#include "capture_internal.hpp"
#include "eSPDI.h"
#include "log.hpp"

namespace pyeys3d {

namespace {

// APC_OpenDevice selects streams by index into the device's resolution
// lists rather than by explicit dimensions; match the catalog mode.
// want_mjpg < 0 skips the format check (depth streams carry no format).
int match_stream(const APC_STREAM_INFO* list, int count,
                 int w, int h, int want_mjpg) {
    for (int i = 0; i < count; ++i) {
        if (list[i].nWidth != w || list[i].nHeight != h) continue;
        if (want_mjpg >= 0
            && (list[i].bFormatMJPG != 0) != (want_mjpg != 0)) continue;
        return i;
    }
    return -1;
}

}  // namespace

// Per-open callback context: the engine plus the generation this open was
// given. Passed to APC_OpenDevice as the callback param so the trampoline can
// reject a callback from a handle the USB self-reset left behind.
struct CallbackCtx {
    // Held by value so it stays valid after the engine is gone, which is
    // the whole point: the callback locks it before touching self and
    // keeps it locked for the whole ingest.
    std::shared_ptr<CallbackGate> gate;
    CaptureEngine* self;
    uint64_t generation;
};

// Configure and open the streams of the already selected device:
// interleave and depth data type first, then the stream-index lookup
// and open. Frame delivery starts inside APC_OpenDevice; the
// callback drops everything until start() raises streaming_ (the ingest
// buffers are sized just before that).
bool CaptureEngine::os_open_streams(std::string& err) {
    DEVSELINFO sel{devsel_index_};

    // APC_EnableInterleave is only implemented on interleave-capable
    // firmware and fails on every other module even for a disable
    // request, so gate each call on APC_IsInterleaveDevice — a
    // non-capable device has nothing to clear.
    if (APC_IsInterleaveDevice(handle_, &sel)) {
        int irc = APC_EnableInterleave(handle_, &sel, cfg_.interleave);
        if (irc != APC_OK && cfg_.interleave) {
            err = "APC_EnableInterleave failed: " + apc_strerror(irc);
            return false;
        }
    } else if (cfg_.interleave) {
        err = "mode requires interleave but the device does not support it";
        return false;
    }
    // The SDK splits the interleave stream itself; each stream's serial
    // numbers then advance by 2 per frame.
    sn_step_ = cfg_.interleave ? 2 : 1;
    int rc = APC_SetDepthDataType(
        handle_, &sel, static_cast<unsigned short>(effective_depth_dtype_));
    if (rc != APC_OK) {
        err = "APC_SetDepthDataType(" + std::to_string(effective_depth_dtype_)
            + ") failed: " + apc_strerror(rc);
        return false;
    }

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

    APC_STREAM_INFO color_list[64] = {};
    APC_STREAM_INFO depth_list[64] = {};
    rc = APC_GetDeviceResolutionList(handle_, &sel, 64, color_list,
                                     64, depth_list);
    if (rc < 0) {
        err = "APC_GetDeviceResolutionList failed: " + apc_strerror(rc);
        return false;
    }
    // The return packs the device's total stream counts, which can exceed
    // what fits in the arrays; only the filled entries are searchable.
    const int n_color = std::min(rc / 256, 64);
    const int n_depth = std::min(rc % 256, 64);

    int color_index = -1;
    if (cfg_.color_w > 0 && cfg_.color_h > 0) {
        color_index = match_stream(color_list, n_color,
                                   cfg_.color_w, cfg_.color_h, cfg_.color_fmt);
        if (color_index < 0) {
            err = "no color stream matches " + std::to_string(cfg_.color_w)
                + "x" + std::to_string(cfg_.color_h)
                + (cfg_.color_fmt ? " MJPEG" : " YUY2")
                + " in the device's resolution list; a USB 2.0 link does not "
                  "publish the USB 3.0-only modes, so check the negotiated "
                  "link speed before the mode";
            return false;
        }
    }
    int depth_index = -1;
    if (cfg_.depth_w > 0 && cfg_.depth_h > 0) {
        depth_index = match_stream(depth_list, n_depth,
                                   cfg_.depth_w, cfg_.depth_h, -1);
        if (depth_index < 0) {
            err = "no depth stream matches " + std::to_string(cfg_.depth_w)
                + "x" + std::to_string(cfg_.depth_h)
                + " in the device's resolution list; a USB 2.0 link does not "
                  "publish the USB 3.0-only modes, so check the negotiated "
                  "link speed before the mode";
            return false;
        }
    }

    // Non-capturing lambda: converts to the SDK's plain function-pointer
    // callback type, and — being defined inside a member function — may
    // touch the engine's private state through the param pointer.
    // The SDK delivers the LONGLONG timestamp straight through to
    // hw_timestamp_us via the (sec, usec) split; the unit is
    // microseconds.
    auto trampoline = [](APCImageType::Value imgType, int /*imgId*/,
                         unsigned char* imgBuf, int imgSize,
                         int /*width*/, int /*height*/, int serialNumber,
                         LONGLONG timestamp, void* param) {
        auto* ctx = static_cast<CallbackCtx*>(param);
        // Held for the whole callback, not just to test the flag: an
        // engine being torn down waits here rather than destroying members
        // an ingest is still writing. A handle the USB self-reset left
        // behind can still deliver after its engine is gone, which is why
        // the gate is owned by the context and not by the engine.
        const std::shared_ptr<CallbackGate> gate = ctx->gate;
        // Tested twice on purpose. The unlocked test is what makes the
        // engine's wait finish: once the flag is down a delivered frame
        // costs nothing and takes no lock, so the waiter is not held off by
        // a handle that keeps delivering. The locked test is the one that
        // is sound — the flag can fall between the two — and the lock is
        // then held for the whole ingest, so a teardown waits here instead
        // of destroying members an ingest is still writing.
        if (!gate->alive.load(std::memory_order_acquire)) return;
        std::shared_lock<std::shared_mutex> gate_lk(gate->mtx);
        if (!gate->alive.load(std::memory_order_acquire)) return;
        auto* self = ctx->self;
        // Reject a callback from a handle left behind by the USB self-reset:
        // a newer open has bumped the generation, so this stale one must not
        // touch engine state or contend with the current handle's callback.
        if (ctx->generation
                != self->callback_generation_.load(std::memory_order_acquire))
            return;
        if (!self->streaming_.load(std::memory_order_acquire)) return;
        if (self->reconnecting_.load(std::memory_order_acquire)) return;
        const int64_t tv_sec  = timestamp / 1'000'000;
        const int64_t tv_usec = timestamp % 1'000'000;
        if (APCImageType::IsImageColor(imgType)) {
            self->last_color_ns_.store(steady_now_ns(), std::memory_order_relaxed);
            self->color_started_.store(true, std::memory_order_release);
            self->ingest_color(imgBuf, static_cast<size_t>(imgSize),
                               serialNumber, tv_sec, tv_usec);
        } else {
            self->last_depth_ns_.store(steady_now_ns(), std::memory_order_relaxed);
            self->ingest_depth(imgBuf, static_cast<size_t>(imgSize),
                               serialNumber, tv_sec, tv_usec);
        }
    };

    // Fresh generation for this open; the context carries it so a left-behind
    // handle's callback is rejected once this (or a later) open bumps it.
    const uint64_t gen =
        callback_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
    auto* ctx = new CallbackCtx{cb_gate_, this, gen};
    cb_ctx_ = ctx;   // freed when this handle is released
    // ApcDIDepthSwitch is a bitmask over where depth comes from, not a
    // format: Depth0 takes it from the colour endpoint's second half,
    // Depth1 from the separate depth endpoint, Depth2 additionally from a
    // slave device. Only Depth1 gives depth_index any meaning, and only a
    // multi-baseline module (whose D0/D1/D2 are different baselines) wants
    // the others. Every non-GUI caller in the SDK passes Depth1. Note the
    // eSPDI header's own \param text for this argument is a paste of the
    // unrelated ctrlMode table and does not describe these bits.
    rc = APC_OpenDevice(handle_, &sel, color_index, depth_index,
                        ApcDIDepthSwitch::Depth1, cfg_.fps,
                        trampoline, ctx);
    if (rc != APC_OK) {
        delete ctx;              // no callback was registered
        cb_ctx_ = nullptr;
        err = "APC_OpenDevice failed: " + apc_strerror(rc);
        return false;
    }
    return true;
}

// The Windows SDK binds a device's stream metadata to the first index a
// handle selects; opening any later index streams nothing while still
// reporting success. open() therefore re-inits so the chosen device is
// the handle's first selection.
bool CaptureEngine::os_fresh_handle_required() { return true; }

// Closes and releases inline: teardown returns promptly whether or not the
// device is still attached, so it can run on the caller's thread.
void CaptureEngine::os_teardown_device() {
    void* h = handle_;
    handle_ = nullptr;
    const bool was_open = device_open_;
    device_open_ = false;
    if (!h) return;

    DEVSELINFO sel{devsel_index_};
    if (was_open) APC_CloseDevice(h, &sel);
    void* releasing = h;
    APC_Release(&releasing);
    // The callback stops with the released handle, so its context is safe
    // to free (delete nullptr is a no-op).
    delete static_cast<CallbackCtx*>(cb_ctx_);
    cb_ctx_ = nullptr;
}

// Reopening a device that never delivered a frame repeats the same
// failed negotiation and is not safe to tear down mid-stream here, so
// stay disconnected instead.
bool CaptureEngine::os_park_when_never_streamed() { return true; }

// USB self-reset. The final detach write blocks here once the link starts
// dropping, so run the config writes under the lock, then hand the handle
// to a detached thread for the detach write and release — clearing handle_
// so the watchdog's reopen rebuilds a fresh handle instead of releasing
// this one from under the detach thread. The handle leaks if the detach
// write never returns, bounded by reset calls.
bool CaptureEngine::os_reset_usb() {
    void* h;
    int idx;
    {
        std::lock_guard<std::timed_mutex> lk(control_mtx_);
        if (!handle_ || !device_open_) return false;
        DEVSELINFO sel{devsel_index_};
        const int flags = FG_Address_2Byte | FG_Value_1Byte;
        for (size_t i = 0; i < kResetDetachIdx; ++i)
            APC_SetHWRegister(handle_, &sel, kResetSeq[i].addr,
                              kResetSeq[i].val, flags);
        h = handle_;
        idx = devsel_index_;
        handle_ = nullptr;
        device_open_ = false;
        // Fence the handle being abandoned: it keeps delivering until the
        // detached thread closes it, and its context carries this open's
        // generation, so without the bump its callbacks stay acceptable
        // until some later open happens to raise the counter.
        callback_generation_.fetch_add(1, std::memory_order_acq_rel);
    }
    const ResetReg detach = kResetSeq[kResetDetachIdx];
    // Detached because the write below can block until the device finishes
    // re-enumerating. Closing before the release is what stops this handle
    // calling back; the context it was opened with is deliberately not
    // freed, since only this thread knows when the callbacks have stopped.
    std::thread([h, idx, detach]() {
        DEVSELINFO sel{idx};
        APC_SetHWRegister(h, &sel, detach.addr, detach.val,
                          FG_Address_2Byte | FG_Value_1Byte);
        APC_CloseDevice(h, &sel);
        void* releasing = h;
        APC_Release(&releasing);
    }).detach();
    return true;
}

// The callback registered at open delivers frames on SDK threads; there
// is nothing to launch or join. Delivery is gated on streaming_, which
// start() raises after sizing the ingest buffers and close() clears
// before APC_CloseDevice.
void CaptureEngine::os_start_fetch() {}

// The interleave setting persists on the device across opens. Every
// open writes it anyway, but clearing it at close keeps the device in
// its default state for other clients. The SDK callback has no
// per-stream teardown, so the timing totals print here.
void CaptureEngine::os_stop_fetch() {
    // Skip the teardown I/O on the reconnect path: a control transfer to
    // the already-removed device can block. The reopen re-applies the
    // interleave setting.
    if (reconnecting_.load(std::memory_order_acquire)) return;
    if (timing_ && dec_cnt_) {
        PYEYS3D_INFO(tag(), "color decode: avg %.2f ms / max %.2f ms over %lu frames",
                     dec_sum_ns_ / dec_cnt_ / 1e6, dec_max_ns_ / 1e6,
                     static_cast<unsigned long>(dec_cnt_));
        dec_cnt_ = 0;   // close() may pass through here twice
    }
    if (timing_ && flt_cnt_) {
        PYEYS3D_INFO(tag(), "depth convert+filter: avg %.2f ms / max %.2f ms over %lu frames",
                     flt_sum_ns_ / flt_cnt_ / 1e6, flt_max_ns_ / 1e6,
                     static_cast<unsigned long>(flt_cnt_));
        flt_cnt_ = 0;
    }
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (handle_ && device_open_) {
        DEVSELINFO sel{devsel_index_};
        if (cfg_.interleave && APC_IsInterleaveDevice(handle_, &sel))
            APC_EnableInterleave(handle_, &sel, false);
    }
}

// ---------------------------------------------------------------------------
//   JPEG decode (SDK converter — the DLL exports no turbojpeg)
// ---------------------------------------------------------------------------

bool CaptureEngine::os_init_decoder(std::string&) {
    return true;   // APC_ColorFormat_to_RGB24 needs only the open handle
}

void CaptureEngine::os_release_decoder() {}

bool CaptureEngine::os_decode_jpeg(const uint8_t* buf, size_t got, uint8_t* out,
                                   int w, int h, bool gray) {
    uint8_t* rgb = out;
    if (gray) {
        // The converter only emits RGB24; decode wide and take one channel
        // (mono modules deliver R=G=B). wide_rgb_ doubles as the scratch —
        // its only other use is on this same (color-ingest) call path.
        wide_rgb_.resize(static_cast<size_t>(w) * h * 3);
        rgb = wide_rgb_.data();
    }
    {
        // The converter runs on the SDK handle, which a teardown releases,
        // so the decode is done under the control lock and can never race
        // it. Bounded, and this is the reason: a teardown holds that lock
        // across APC_CloseDevice, which can wait for the SDK's callback
        // thread to return — this thread. An unbounded wait here would be
        // each waiting for the other. The deadline is far longer than a
        // camera-control write, so it only ever expires on a teardown, and
        // dropping a colour frame while the device closes is the outcome
        // that path wants anyway.
        std::unique_lock<std::timed_mutex> lk(control_mtx_, std::defer_lock);
        if (!lk.try_lock_for(std::chrono::milliseconds(200))) return false;
        if (!handle_ || !device_open_) return false;
        DEVSELINFO sel{devsel_index_};
        if (APC_ColorFormat_to_RGB24(handle_, &sel, rgb,
                                     const_cast<uint8_t*>(buf),
                                     static_cast<int>(got), w, h,
                                     APCImageType::COLOR_MJPG) != APC_OK)
            return false;
    }
    if (gray) {
        const size_t n = static_cast<size_t>(w) * h;
        for (size_t i = 0; i < n; ++i) out[i] = rgb[i * 3];
    }
    return true;
}

// ---------------------------------------------------------------------------
//   Camera-control primitives (DirectShow property surface)
// ---------------------------------------------------------------------------
// The APC_Property{CT,PU}_{GetRange,GetCurrent,SetCurrent} family is the
// SDK's control surface here: each control carries a value plus a caps
// flag, and the flag switches auto/manual (DirectShow
// CameraControlFlags: 1 = auto, 2 = manual).

namespace {
constexpr long kDshowAuto = 1;
constexpr long kDshowManual = 2;
}  // namespace

bool CaptureEngine::os_write_auto_exposure(DEVSELINFO& sel, bool on) {
    // Read-modify-write: keep the current value, change only the
    // auto/manual flag. Without a valid read there is no safe value to
    // write back, so fail rather than write zero.
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyCT_GetCurrent(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return false;
    return APC_PropertyCT_SetCurrent(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                     cur, cur2,
                                     on ? kDshowAuto : kDshowManual, pid_) >= 0;
}

bool CaptureEngine::os_write_exposure_value(DEVSELINFO& sel, int value) {
    return APC_PropertyCT_SetCurrent(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                     value, 0, kDshowManual, pid_) >= 0;
}

bool CaptureEngine::os_write_auto_white_balance(DEVSELINFO& sel, bool on) {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyPU_GetCurrent(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return false;
    return APC_PropertyPU_SetCurrent(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                     cur, cur2,
                                     on ? kDshowAuto : kDshowManual, pid_) >= 0;
}

bool CaptureEngine::os_write_white_balance_value(DEVSELINFO& sel, int value) {
    return APC_PropertyPU_SetCurrent(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                     value, 0, kDshowManual, pid_) >= 0;
}

bool CaptureEngine::os_write_power_line_frequency(DEVSELINFO& sel, int mode) {
    return APC_PropertyPU_SetCurrent(handle_, &sel,
                                     PU_PROPERTY_ID_POWERLINE_FREQUENCY,
                                     mode, 0, kDshowManual, pid_) >= 0;
}

std::optional<bool> CaptureEngine::os_read_auto_exposure(DEVSELINFO& sel) const {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyCT_GetCurrent(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return std::nullopt;
    return (flags & kDshowAuto) != 0;
}

std::optional<int> CaptureEngine::os_read_exposure_value(DEVSELINFO& sel) const {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyCT_GetCurrent(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return std::nullopt;
    return static_cast<int>(cur);
}

std::optional<bool>
CaptureEngine::os_read_auto_white_balance(DEVSELINFO& sel) const {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyPU_GetCurrent(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return std::nullopt;
    return (flags & kDshowAuto) != 0;
}

std::optional<int>
CaptureEngine::os_read_white_balance_value(DEVSELINFO& sel) const {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyPU_GetCurrent(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                  &cur, &cur2, &flags, pid_) < 0)
        return std::nullopt;
    return static_cast<int>(cur);
}

std::optional<int>
CaptureEngine::os_read_power_line_frequency(DEVSELINFO& sel) const {
    long cur = 0, cur2 = 0, flags = 0;
    if (APC_PropertyPU_GetCurrent(handle_, &sel,
                                  PU_PROPERTY_ID_POWERLINE_FREQUENCY,
                                  &cur, &cur2, &flags, pid_) < 0)
        return std::nullopt;
    return static_cast<int>(cur);
}

std::optional<std::tuple<int, int, int, int>>
CaptureEngine::get_exposure_range() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    long mn = 0, mx = 0, step = 0, def = 0, flags = 0;
    if (APC_PropertyCT_GetRange(handle_, &sel, CT_PROPERTY_ID_EXPOSURE,
                                &mn, &mx, &step, &def, &flags, pid_) < 0)
        return std::nullopt;
    return std::make_tuple(static_cast<int>(mn), static_cast<int>(mx),
                           static_cast<int>(step), static_cast<int>(def));
}

std::optional<std::tuple<int, int, int, int>>
CaptureEngine::get_white_balance_range() const {
    std::lock_guard<std::timed_mutex> lk(control_mtx_);
    if (!handle_ || !device_open_) return std::nullopt;
    DEVSELINFO sel{devsel_index_};
    long mn = 0, mx = 0, step = 0, def = 0, flags = 0;
    if (APC_PropertyPU_GetRange(handle_, &sel, PU_PROPERTY_ID_WHITEBALANCE,
                                &mn, &mx, &step, &def, &flags, pid_) < 0)
        return std::nullopt;
    return std::make_tuple(static_cast<int>(mn), static_cast<int>(mx),
                           static_cast<int>(step), static_cast<int>(def));
}

}  // namespace pyeys3d
