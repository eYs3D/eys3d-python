// Platform seam for pyeys3d.
//
// Everything OS-specific that is not frame delivery lives behind these
// declarations; frame delivery has its own seam (the capture backend).
// One platform_<os>.cpp implements this file per OS.

#pragma once

#include <string>

#include "eSPDI_def.h"   // DEVINFORMATION

namespace pyeys3d {

// Map an enumerated device to a stable bus-location string used for
// device binding: the sysfs USB interface path on Linux (from the
// /dev/videoN node name), the SDK-reported USB node number on Windows.
// Returns an empty string when the lookup fails.
std::string resolve_usb_port(const DEVINFORMATION& info);

// RAII guard around SDK calls that are known to replace process signal
// handlers (APC_Init / open): captures the handlers on construction and
// restores them on destruction. A no-op on platforms where the SDK does
// not touch them.
class SignalHandlerGuard {
public:
    SignalHandlerGuard();
    ~SignalHandlerGuard();
    SignalHandlerGuard(const SignalHandlerGuard&) = delete;
    SignalHandlerGuard& operator=(const SignalHandlerGuard&) = delete;

private:
    void* state_ = nullptr;   // opaque per-platform storage
};

}  // namespace pyeys3d
