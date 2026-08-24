// One camera is set up at a time within a process.
//
// The SDK keeps its device bookkeeping per process, not per handle, so two
// threads that enumerate or open at the same time walk over each other even
// though each holds its own APC_Init handle: the flash reads that back
// APC_GetSerialNumber and APC_GetRectifyMatLogData come back
// APC_READFLASHFAIL and stay failed past their retry budget, and a device
// list read while another device is opening no longer matches the index it
// was read at. The stream itself comes up regardless, so a caller that
// parallelised its opens gets cameras that deliver frames and report no
// intrinsics, with nothing but the log to say so.
//
// Separate processes do not share this — one camera per process opens in
// parallel with no interference, which is what examples/06_multicam.py
// does. The cost of holding this lock is therefore paid only by a process
// driving several cameras itself: their opens queue rather than overlap.
//
// The wait is bounded: an SDK call against a camera unplugged mid-open can
// fail to return while holding this lock, so a deadline turns one lost
// camera into an error the other cameras can act on rather than silence.
#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>

namespace pyeys3d {

inline std::timed_mutex& device_setup_lock() {
    static std::timed_mutex m;
    return m;
}

// How long to wait for the turn. A healthy open holds the lock about six
// seconds, and N cameras coming up together queue for (N-1) of those, so
// the deadline has to clear a full rig — not just one open. It is never
// reached when nothing is stuck, so being generous costs only how long a
// caller waits before being told about a camera that will never finish.
inline constexpr int kSetupTurnTimeoutMs = 120'000;

// Take the process's device-setup turn, or throw naming what to do. `what`
// is the operation for the message, e.g. "open G100P" — the caller knows
// which camera it wanted.
class SetupTurn {
public:
    // The watchdog passes give_up: close() drops streaming_ before joining
    // it, and the join would otherwise inherit however long a sibling
    // camera still had on the clock.
    explicit SetupTurn(const std::string& what,
                       std::function<bool()> give_up = nullptr)
        : lock_(device_setup_lock(), std::defer_lock) {
        constexpr auto kSlice = std::chrono::milliseconds(100);
        auto left = std::chrono::milliseconds(kSetupTurnTimeoutMs);
        while (!lock_.try_lock_for(left < kSlice ? left : kSlice)) {
            if (give_up && give_up()) {
                throw std::runtime_error(
                    "gave up waiting to " + what + ": this pipeline is "
                    "closing");
            }
            left -= (left < kSlice ? left : kSlice);
            if (left.count() > 0) continue;
            throw std::runtime_error(
                "timed out waiting to " + what + ": another camera in this "
                "process has been setting up for " +
                std::to_string(kSetupTurnTimeoutMs / 1000) +
                " s and has not finished. A camera unplugged while it was "
                "opening can leave the SDK call it was in unreturned; that "
                "camera is lost until the process restarts, but the others "
                "are not");
        }
    }

private:
    std::unique_lock<std::timed_mutex> lock_;
};

}  // namespace pyeys3d
