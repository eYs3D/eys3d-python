// Capture engine for pyeys3d.
//
// Single concrete capture path over eSPDI for G100+, R77, and G62.

#pragma once

#include <array>
#include <atomic>
#include <climits>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <optional>

#include "eSPDI_def.h"   // DEVSELINFO
#include "zd_lookup.hpp"
#include "spatial_filter.hpp"
#include "temporal_filter.hpp"
#include "hole_filling.hpp"

namespace pyeys3d {

// Shared between an engine and every callback context it hands the SDK, by
// shared_ptr, so a callback that outlives the engine still has something
// valid to test and to hold. See CaptureEngine::cb_gate_.
struct CallbackGate {
    std::shared_mutex mtx;
    std::atomic<bool> alive{true};
};

// ============================================================================
//   Device enumeration
// ============================================================================

struct DeviceInfo {
    int index = -1;
    uint16_t pid = 0;
    uint16_t vid = 0;
    std::string serial_number;
    std::string device_node;
    std::string usb_port;
    std::string firmware_version;
    int usb_port_type = 0;   // eSPDI USB_PORT_TYPE: 2 = USB2.0, 3 = USB3.0
};


class Context {
public:
    Context();
    ~Context();
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    std::vector<DeviceInfo> query_devices();

private:
    void* handle_ = nullptr;
};

// ============================================================================
//   Calibration (rectify log → CameraInfo-style intrinsics)
// ============================================================================

struct LensCalibration {
    std::array<double, 9>  K{};   // 3x3 raw camera matrix
    std::array<double, 8>  D{};   // rational polynomial: k1 k2 p1 p2 k3 k4 k5 k6
    std::array<double, 9>  R{};   // 3x3 rectification rotation
    std::array<double, 12> P{};   // 3x4 rectified projection
};

struct Calibration {
    int width = 0;
    int height = 0;
    LensCalibration left;
    LensCalibration right;
    double baseline_mm = 0.0;
    int out_img_width = 0;        // rectify log OutImgWidth
    int out_img_height = 0;       // rectify log OutImgHeight (for ratio_mat)
    bool valid = false;
};

// The camera model this mode's zd_index maps to, as the device stores it.
// Not resampled to any stream size; the point cloud rescales internally.
struct Intrinsics {
    int width = 0;
    int height = 0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    std::array<double, 9>  K{};
    std::array<double, 8>  D{};
    std::array<double, 9>  R{};
    std::array<double, 12> P{};
    double baseline_mm = 0.0;
    bool valid = false;
};

// ============================================================================
//   Frames
// ============================================================================

// The semantic domain of a frame's pixel data. Frames delivered to Python
// are always COLOR_RGB8 or DEPTH_MM; the DISPARITY_* domains are internal
// intermediates of the native filter chain (raw -> Q4 -> ZD mm) inside
// CaptureEngine and are never handed out.
enum class FrameDomain : int {
    COLOR_RGB8     = 0,   // (H, W, 3) uint8
    DEPTH_MM       = 1,   // (H, W) uint16, millimeters (Z14 masked)
    DISPARITY_D11  = 2,   // internal: raw 11-bit disparity from device
    DISPARITY_Q4   = 3,   // internal: Q4 fixed-point disparity (raw << 4)
};

struct Frame {
    FrameDomain domain = FrameDomain::COLOR_RGB8;
    std::vector<uint8_t> data;
    int width = 0;
    int height = 0;
    int bytes_per_pixel = 0;
    int frame_number = 0;
    uint64_t hw_timestamp_us = 0;
    // The hardware timestamp mapped onto the host realtime clock (epoch
    // ns) via a fitted clock model, so frames can be correlated with
    // other sensors; see CaptureEngine::host_time_from_hw().
    int64_t host_timestamp_ns = 0;
};

// ============================================================================
//   OpenConfig
// ============================================================================

struct OpenConfig {
    // device selection
    std::string usb_port;
    std::string serial_number;
    uint16_t expected_pid = 0;

    // video mode
    int color_w = 0;
    int color_h = 0;
    int color_fmt = 0;          // 0 = YUYV, 1 = MJPEG
    bool color_split_lr = false;
    bool is_mono = false;       // monochrome sensor -> luma-only decode
    int depth_w = 0;
    int depth_h = 0;
    int depth_dtype = 0;
    int zd_index = 0;
    int fps = 30;
    bool interleave = false;

    // Manual exposure uses signed register units (a log2 scale on some
    // modules, so negative values are legal); its unset sentinel therefore
    // sits outside the int range the devices use, not at -1.
    static constexpr int kExposureUnset = INT_MIN;

    // runtime
    int ir_value = -1;
    int auto_exposure = -1;       // -1 = leave as-is, 0 = manual, 1 = auto
    int exposure_time = kExposureUnset;  // set (any int) forces manual AE
    int auto_white_balance = -1;  // -1 = leave as-is, 0 = manual, 1 = auto
    int white_balance = -1;       // -1 = leave as-is; >=0 forces manual AWB
    int power_line_frequency = -1;// -1 = leave as-is; 0/1/2/3

    // Post-process filter chain. The pipeline runs these natively in the
    // depth thread and always delivers DEPTH_MM. A disparity-domain filter
    // (spatial or temporal) opens the device in 11-bit disparity mode
    // (dtype + 2) and loads the ZD table; otherwise the firmware Z14 mm
    // fast path is used with no host conversion. hole_filling runs in the
    // mm domain on either path.
    bool   filter_spatial = false;
    double spatial_alpha = 0.5;
    int    spatial_delta = 20;
    int    spatial_magnitude = 2;
    int    spatial_holes_fill = 0;

    bool   filter_temporal = false;
    double temporal_alpha = 0.4;
    int    temporal_delta = 20;
    int    temporal_persistence = 3;

    bool   filter_hole = false;
    int    hole_mode = 2;

    // Depth clip applied to the delivered mm; the Python layer resolves the
    // per-model defaults before handing the config over.
    int depth_near_mm = 0;
    int depth_far_mm = 0;

    // Firmware depth-quality register profile: {address, mask, value}
    // entries on the 2-byte-address / 1-byte-value firmware register
    // space, resolved by the Python layer (bundled per-model profile or
    // a user file). Applied by a worker thread once the stream settles
    // (the depth pipeline must be running for the writes to take
    // effect) and re-applied after a reconnect, since the firmware
    // power-cycles register state on USB re-enumeration. Empty = leave
    // the firmware defaults.
    std::vector<std::array<int, 3>> quality_regs;
};

// ============================================================================
//   CaptureEngine
// ============================================================================

class CaptureEngine {
public:
    CaptureEngine();
    ~CaptureEngine();
    CaptureEngine(const CaptureEngine&) = delete;
    CaptureEngine& operator=(const CaptureEngine&) = delete;

    void open(const OpenConfig& cfg);
    void start();
    void close();

    // Fills out_color (left) and out_depth. out_color_right is populated
    // only in wide L|R split modes; has_right reports whether it was set.
    bool wait_for_frames(int timeout_ms, Frame& out_color, Frame& out_depth,
                         Frame& out_color_right, bool& has_right);

    bool split_color() const { return cfg_.color_split_lr; }
    bool is_open() const { return device_open_; }
    bool is_streaming() const { return streaming_.load(std::memory_order_acquire); }

    uint16_t pid() const { return pid_; }
    // Copied under identity_mtx_: a replug can move the camera to another
    // socket, and the watchdog rewrites usb_port_ when it does, while a
    // caller reads device_info from its own thread. Its own lock rather
    // than control_mtx_, which the rebuild holds for the whole reopen.
    std::string serial_number() const {
        std::lock_guard<std::mutex> lk(identity_mtx_);
        return serial_number_;
    }
    std::string usb_port() const {
        std::lock_guard<std::mutex> lk(identity_mtx_);
        return usb_port_;
    }
    std::string firmware_version() const {
        std::lock_guard<std::mutex> lk(identity_mtx_);
        return firmware_version_;
    }

    // Read the current camera-control state. Each returns nullopt when
    // the device is closed or the read fails. The property-id spaces
    // differ per OS; the mapping lives in the per-OS backend.
    std::optional<bool> get_auto_exposure() const;
    std::optional<int>  get_exposure() const;
    std::optional<bool> get_auto_white_balance() const;
    std::optional<int>  get_white_balance() const;
    std::optional<int>  get_power_line_frequency() const;

    // The device-reported (min, max, step, default) of the manual exposure /
    // white-balance value; nullopt when the device is closed or the query
    // fails.
    std::optional<std::tuple<int, int, int, int>> get_exposure_range() const;
    std::optional<std::tuple<int, int, int, int>> get_white_balance_range() const;

    // Runtime camera controls. Each writes the device immediately and also
    // updates the stored config, so a hot-plug reopen re-applies the latest
    // values rather than reverting to what start() saw. Returns false when
    // the device is closed or rejects the write.
    bool set_ir_value(int value);
    std::optional<int> get_ir_value() const;
    bool set_auto_exposure(bool on);
    bool set_exposure(int value);           // forces manual AE first
    bool set_auto_white_balance(bool on);
    bool set_white_balance(int value);      // forces manual AWB first
    bool set_power_line_frequency(int mode);

    // Retune the temporal filter while streaming. Only legal when the chain
    // was opened with the temporal stage enabled (the disparity data type is
    // fixed at open time); throws std::runtime_error otherwise.
    void set_temporal_params(double alpha, int delta, int persistence);

    // Reset the camera over USB: a fixed firmware register sequence whose
    // tail writes drop the link, so the host sees a detach + re-enumeration
    // like a physical replug. The watchdog then rebinds by port identity and
    // streaming resumes (reconnect_count increments). The triggering writes
    // are acknowledged unreliably (the link is already going away), so their
    // result is ignored. Returns false only when the device is not open.
    bool reset_usb();

    // By value, under calib_mtx_: the watchdog rewrites the whole
    // struct when it reloads the rectify log after a reconnect, and a
    // reference would let PointCloud read it as it is being replaced.
    Calibration calibration() const {
        std::lock_guard<std::mutex> lk(calib_mtx_);
        return calib_;
    }
    const pyeys3d::ZdTable& zd_table() const { return zd_table_; }
    int depth_near_mm() const { return depth_min_mm_; }
    int depth_far_mm() const { return depth_max_mm_; }

    // Hot-plug recovery stats (read-only, for diagnostics).
    uint64_t reconnect_count() const { return reconnect_count_.load(std::memory_order_relaxed); }
    bool is_connected() const { return connected_.load(std::memory_order_acquire); }

    // Depth-quality register profile status: registers applied / failed
    // so far, and whether the worker has yet to run for this session.
    int quality_regs_ok() const { return quality_ok_.load(std::memory_order_relaxed); }
    int quality_regs_failed() const { return quality_failed_.load(std::memory_order_relaxed); }
    bool quality_regs_pending() const { return quality_pending_.load(std::memory_order_acquire); }

    // Frames the camera produced but the host never ingested (detected
    // as gaps in the per-stream serial numbers, interleave-aware).
    // Consumer-side overwrites are not counted — latest-wins delivery
    // makes those a normal part of operation.
    uint64_t dropped_color_frames() const { return color_dropped_.load(std::memory_order_relaxed); }
    uint64_t dropped_depth_frames() const { return depth_dropped_.load(std::memory_order_relaxed); }

    // The stored camera model for this mode's zd_index.
    Intrinsics intrinsics() const;

    // Rate at which each stream reaches the host, measured at publish so
    // it is independent of the consumer. Reads 0 after a second of silence.
    double color_fps() const { return fps_now(color_interval_, color_publish_ns_); }
    double depth_fps() const { return fps_now(depth_interval_, depth_publish_ns_); }

private:
    void color_capture_loop();
    void depth_capture_loop();
    void watchdog_loop();
    void quality_worker();  // applies cfg_.quality_regs; see quality_thread_
    void load_calibration();
    void load_zd_table_if_needed();
    bool reopen_device();   // close + re-open the same device; returns success
    // Fresh SDK handle whose first-selected index is `index`, verifying
    // the device behind it. Used by open() and reopen_device() where
    // os_fresh_handle_required() says the platform needs it.
    bool open_fresh_handle_for(int index, uint16_t expected_pid,
                               const std::string& expected_serial);
    // Per-OS behavior switches (defined in capture_windows/linux.cpp):
    // whether open() must re-init so the chosen device is the handle's
    // first selection, and whether a pipeline that never produced a
    // frame should stay disconnected rather than reopen.
    static bool os_fresh_handle_required();
    static bool os_park_when_never_streamed();
    // Close + release the current handle for a reopen, clearing handle_
    // and device_open_. Runs inline on both platforms.
    void os_teardown_device();
    // Drive the USB self-reset register sequence. The tail write blocks on
    // Windows once the link drops, so the platforms differ: Windows runs
    // the config writes under control_mtx_ then hands the handle to a
    // detached thread for the detach write; Linux runs the whole sequence
    // inline. Returns false only when the device is not open.
    bool os_reset_usb();
    // Write the user-set camera controls (AE / exposure / AWB / WB / power-
    // line frequency) to the device. Called from open() and reopen_device().
    void apply_camera_controls(DEVSELINFO& sel);

    // OS-independent ingest layer. The per-OS fetch layer (Linux pull
    // loops / Windows SDK callback) stamps last_color_ns_ / last_depth_ns_ /
    // color_started_ and applies any host-side interleave parity skip,
    // then hands each raw buffer to ingest_*: pure decode / convert /
    // filter / publish, no SDK calls.
    void prepare_buffers();   // size the ingest state; called from start()
    void ingest_color(const uint8_t* buf, size_t got, int serial,
                      int64_t tv_sec, int64_t tv_usec);
    void ingest_depth(const uint8_t* buf, size_t got, int serial,
                      int64_t tv_sec, int64_t tv_usec);
    // Map a device hw timestamp (µs) onto the host realtime clock (epoch
    // ns). Anchored on the first frame; re-anchors when the device clock
    // runs backward (device reset / counter wrap). Call with mtx_ held —
    // the anchor is shared by both streams.
    int64_t host_time_from_hw(uint64_t hw_us);
    // Track per-stream serial-number gaps (call from the stream's ingest).
    void note_stream_serial(int serial, int& last_sn,
                            std::atomic<uint64_t>& dropped);

    // Per-OS backend, one implementation per build. os_open_streams()
    // configures and opens the already selected device; on failure it
    // fills `err` and leaves the caller to release the handle.
    // os_start_fetch() arms frame delivery after prepare_buffers();
    // os_stop_fetch() tears it down after streaming_ clears.
    bool os_open_streams(std::string& err);
    void os_start_fetch();
    void os_stop_fetch();

    // JPEG decode primitive: gray=true fills a w*h luma plane,
    // otherwise w*h*3 packed RGB.
    bool os_init_decoder(std::string& err);   // at open(), MJPEG modes only
    void os_release_decoder();                // at close()
    bool os_decode_jpeg(const uint8_t* buf, size_t got, uint8_t* out,
                        int w, int h, bool gray);

    // Per-OS camera-control primitives: raw SDK reads/writes with the
    // platform's property-id mapping. No locking and no cfg_ bookkeeping —
    // the common callers own both. Writers return false when the device
    // rejects the write.
    bool os_write_auto_exposure(DEVSELINFO& sel, bool on);
    bool os_write_exposure_value(DEVSELINFO& sel, int value);
    bool os_write_auto_white_balance(DEVSELINFO& sel, bool on);
    bool os_write_white_balance_value(DEVSELINFO& sel, int value);
    bool os_write_power_line_frequency(DEVSELINFO& sel, int mode);
    std::optional<bool> os_read_auto_exposure(DEVSELINFO& sel) const;
    std::optional<int>  os_read_exposure_value(DEVSELINFO& sel) const;
    std::optional<bool> os_read_auto_white_balance(DEVSELINFO& sel) const;
    std::optional<int>  os_read_white_balance_value(DEVSELINFO& sel) const;
    std::optional<int>  os_read_power_line_frequency(DEVSELINFO& sel) const;

    void* handle_ = nullptr;
    int devsel_index_ = -1;
    // One DEVSELINFO, shared by address between the fetch loops and the
    // quality worker. The SDK keys
    // per-stream V4L2 fd / buffer state on the DEVSELINFO address;
    // separate per-thread copies open independent sessions whose
    // color/depth DQBUF then cross-contaminate.
    DEVSELINFO shared_sel_{};
    std::atomic<bool> device_open_{false};
    uint16_t pid_ = 0;
    bool mono_ = false;   // monochrome sensor (G62 / R77): luma-only color
    // Guards the three identity strings below against a reader copying
    // one while the reconnect path reassigns it. Taken alone by the
    // getters; nested inside control_mtx_ where the rebuild writes.
    mutable std::mutex identity_mtx_;
    // Log prefix for every line this engine writes: "CaptureEngine" until
    // open() picks a device, "CaptureEngine[SN12345678]" after. Set once,
    // before any thread that logs exists, so readers need no lock.
    std::string log_tag_;
    const char* tag() const { return log_tag_.c_str(); }
    std::string serial_number_;
    std::string usb_port_;
    std::string firmware_version_;

    OpenConfig cfg_{};
    int effective_depth_dtype_ = 0;   // after the disparity shift
    int depth_min_mm_ = 0;
    int depth_max_mm_ = 0;
    void* tj_ = nullptr;              // Linux turbojpeg decoder (MJPEG modes;
                                      // unused by the Windows backend)

    // Serialises every control-path use of handle_ (prop reads/writes, the
    // runtime setters, cfg_ control fields) against the watchdog tearing the
    // handle down in reopen_device(). The Linux fetch loops stay off this
    // lock — they park on reconnecting_.
    // timed: the SDK's callback thread takes this for the color
    // conversion, and a teardown holds it across APC_CloseDevice, which
    // can wait for that same thread. The callback bounds its wait.
    mutable std::timed_mutex control_mtx_;
    // Guards temporal_params_ between set_temporal_params() and the per-frame
    // snapshot taken by the depth loop.
    std::mutex temporal_mtx_;

    // Native post-process chain, run inline on the depth thread. When a
    // disparity-domain filter is active the device is opened in D11 and
    // chain_disparity_ drives the raw -> Q4 -> filter -> ZD-mm path;
    // otherwise the firmware delivers Z14 mm directly. Buffers and the
    // temporal IIR history are members so they are reused across frames.
    bool chain_disparity_ = false;    // spatial || temporal => open D11
    bool spatial_on_ = false;
    bool temporal_on_ = false;
    bool hole_on_ = false;
    pyeys3d::SpatialFilterParams  spatial_params_{};
    pyeys3d::TemporalFilterParams temporal_params_{};
    pyeys3d::TemporalState        temporal_state_{};
    pyeys3d::HoleFillMode         hole_mode_ = pyeys3d::HoleFillMode::kOff;
    std::vector<uint16_t> filt_work_;     // raw -> Q4 working buffer
    std::vector<uint16_t> hole_scratch_;  // around-mode scratch

    // Ingest working state, sized by prepare_buffers(). The working frames
    // swap with the published latest_* frames on every publish, so the two
    // sides stay identically sized. depth_staging_ doubles as the Linux
    // fetch target; ingest_depth() copies into it only when handed a
    // foreign buffer (the Windows callback path).
    Frame color_left_;
    Frame color_right_;               // wide L|R split modes only
    Frame depth_staging_;
    std::vector<uint8_t> gray_;       // mono luma plane
    std::vector<uint8_t> wide_rgb_;   // MJPEG split decode intermediate
    uint64_t decode_fails_ = 0;
    int64_t last_decode_warn_ns_ = 0;
    // Frames shorter than the mode's raster, per stream. Kept apart from
    // decode failures because a truncated frame is a wire/negotiation
    // problem, not a codec one, and apart from each other because one
    // stream truncating says nothing about the other.
    uint64_t short_color_frames_ = 0;
    uint64_t short_depth_frames_ = 0;
    int64_t last_short_color_warn_ns_ = 0;
    int64_t last_short_depth_warn_ns_ = 0;
    // hw -> host clock model (guarded by mtx_). Sparse (hw, host) sample
    // pairs — at most one every 500 ms, up to a two-minute window — feed
    // a least-squares line host = base + slope * (hw - base_hw). The
    // long window averages out per-frame transport jitter and tracks
    // crystal drift; until enough samples exist the slope stays nominal
    // (pure anchor behavior).
    struct TimeSample { int64_t hw_us; int64_t host_ns; };
    std::vector<TimeSample> time_samples_;   // ring of kTimeMaxSamples
    size_t time_sample_pos_ = 0;
    int64_t last_time_sample_ns_ = 0;
    int64_t max_hw_us_ = 0;
    double time_slope_ns_per_us_ = 1000.0;
    double time_base_hw_us_ = 0.0;
    double time_base_host_ns_ = 0.0;
    int64_t last_mapped_ns_ = 0;             // monotonic output clamp
    int time_outlier_streak_ = 0;            // consecutive rejected samples
    void reset_time_model();                  // call with mtx_ held
    void refit_time_model();                  // call with mtx_ held

    // Per-stream drop tracking (each last-SN is touched only by its
    // stream's ingest). sn_step_ is the expected serial increment per
    // stream, set by the per-OS backend at open: 2 on Linux in interleave
    // mode (the host splits the shared stream by SN parity), 1 otherwise.
    int sn_step_ = 1;
    int last_color_ingest_sn_ = -1;
    int last_depth_ingest_sn_ = -1;
    std::atomic<uint64_t> color_dropped_{0};
    std::atomic<uint64_t> depth_dropped_{0};

    // Smoothed publish interval, inverted on read to give a mean rate.
    static double fps_now(const std::atomic<double>& interval_s,
                          const std::atomic<int64_t>& last_ns);
    static void note_publish(std::atomic<double>& interval_s,
                             std::atomic<int64_t>& last_ns);
    std::atomic<double> color_interval_{0.0};
    std::atomic<double> depth_interval_{0.0};
    std::atomic<int64_t> color_publish_ns_{0};
    std::atomic<int64_t> depth_publish_ns_{0};
    bool timing_ = false;             // PYEYS3D_TIMING decode/filter stats
    double dec_sum_ns_ = 0; int64_t dec_max_ns_ = 0; uint64_t dec_cnt_ = 0;
    double flt_sum_ns_ = 0; int64_t flt_max_ns_ = 0; uint64_t flt_cnt_ = 0;

    // Guards calib_ against a caller reading intrinsics, or building a
    // PointCloud, while the reconnect path reloads it. Its own lock, not
    // control_mtx_, which the rebuild holds for the whole reopen.
    mutable std::mutex calib_mtx_;
    Calibration calib_{};
    pyeys3d::ZdTable zd_table_{};

    std::atomic<bool> streaming_{false};
    std::thread color_thread_;
    std::thread depth_thread_;
    std::thread watchdog_thread_;

    // Depth-quality register worker. quality_pending_ arms on start()
    // and again after a reopen; the first depth frame launches the
    // worker, which holds off until the stream settles and then walks
    // cfg_.quality_regs under control_mtx_ one register at a time.
    // Joined before a reopen and at close().
    //
    // The frame thread creates it and close() / the watchdog collect it,
    // so every touch of the thread object takes quality_thread_mtx_.
    // quality_pending_ decides who starts one, not who may hold it: on
    // Windows the SDK callback keeps delivering until APC_CloseDevice, so
    // without the lock close() can see the object still empty, skip its
    // join, and leave a joinable thread for the destructor to abort on.
    // The worker itself never takes this lock, so joining under it cannot
    // deadlock.
    std::mutex quality_thread_mtx_;
    std::thread quality_thread_;
    std::atomic<bool> quality_pending_{false};
    std::atomic<int> quality_ok_{0};
    std::atomic<int> quality_failed_{0};

    // Hot-plug recovery. Each capture path stamps its own steady-clock
    // time on every frame; the watchdog reopens the device when any
    // wanted stream has gone silent for the disconnect window -- a stream
    // that wedges while the other keeps flowing is a disconnect too. While
    // reconnecting_ is set, capture loops park instead of calling the SDK
    // with a handle the watchdog is tearing down. Both are seeded at
    // start() so a slow first frame is not a false positive.
    std::atomic<bool> reconnecting_{false};
    std::atomic<bool> connected_{true};
    std::atomic<int64_t> last_color_ns_{0};
    std::atomic<int64_t> last_depth_ns_{0};
    std::atomic<uint64_t> reconnect_count_{0};
    // Windows only: each APC_OpenDevice is given a fresh callback generation,
    // carried in its callback context, so a handle left behind by the USB
    // self-reset — which hands its handle to a detached thread — cannot
    // re-enter ingest against the handle that replaced it: its callback
    // loads a now-stale generation and returns. cb_ctx_ owns the current
    // context, freed once the handle it belongs to is released.
    std::atomic<uint64_t> callback_generation_{0};
    void* cb_ctx_ = nullptr;
    // The barrier between the SDK's callback thread and anything that
    // takes this engine apart. It outlives the engine: the callback context
    // of a handle abandoned by the USB self-reset holds a copy of this
    // pointer, and that handle keeps delivering until the detached thread
    // closes it — the generation guard cannot help there, since it lives in
    // the engine the callback would have to read first.
    //
    // The callback holds `mtx` shared for the whole of its ingest, not just
    // to test a flag on the way in; drain_callbacks() takes it exclusively
    // and so returns only once no callback is inside one.
    std::shared_ptr<CallbackGate> cb_gate_ = std::make_shared<CallbackGate>();

    // Wait until no callback is executing. Two conditions, both required:
    // `alive` must already be false, or a handle that keeps delivering will
    // hold this off indefinitely; and the caller must hold no lock a
    // callback can want (control_mtx_, mtx_, quality_thread_mtx_), or the
    // callback being waited for cannot finish. That rules out close(),
    // where delivery stops only once APC_CloseDevice returns — the wait
    // belongs after the flag is down, in the destructor.
    void drain_callbacks() {
        std::unique_lock<std::shared_mutex> lk(cb_gate_->mtx);
    }
    // False until the first frame after start(): tells the watchdog
    // "never streamed" (mode/negotiation problem — reopening repeats it)
    // apart from "streamed, then stopped" (a real link drop).
    std::atomic<bool> got_any_frame_{false};

    // Set true once the color loop has returned its first frame. start()
    // waits on this before launching the depth loop: the SDK requires the
    // color stream to be established first, otherwise depth DQBUF data
    // cross-contaminates the color buffer on non-interleave devices.
    std::atomic<bool> color_started_{false};
    // Set true by the watchdog before calling os_stop_fetch() during a
    // reconnect. Linux fetch loops exit (return) rather than parking when
    // they see this flag, allowing os_stop_fetch() to join them so that
    // os_start_fetch() can restart them fresh with the correct sequencing.
    std::atomic<bool> reinit_threads_{false};

    std::mutex mtx_;
    std::condition_variable cv_;
    Frame latest_color_;
    Frame latest_color_right_;     // populated only in wide L|R split modes
    Frame latest_depth_;
    int last_color_sn_returned_ = -1;
    int last_depth_sn_returned_ = -1;
    bool color_fresh_ = false;
    bool depth_fresh_ = false;
};

}  // namespace pyeys3d
