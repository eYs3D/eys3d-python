// Human-readable APC (eSPDI) error codes.
//
// apc_error_name() maps a return code to its eSPDI_def.h define name via a
// table generated at build time (see cmake/gen_apc_error_names.py), so it
// tracks the bundled SDK automatically. apc_strerror() adds a short hint for
// the codes users actually hit in the field.

#pragma once

#include <string>

#include "eSPDI_def.h"

namespace pyeys3d {

inline const char* apc_error_name(int rc) {
    switch (rc) {
#include "apc_error_names.inc"
        default: return "unknown";
    }
}

inline std::string apc_strerror(int rc) {
    std::string s = "rc=" + std::to_string(rc) + " " + apc_error_name(rc);
    switch (rc) {
        case APC_NoDevice:
#ifdef _WIN32
            s += " (no camera enumerated; check the USB connection and "
                 "Device Manager)";
#else
            s += " (no camera enumerated; check the USB connection and that "
                 "the user can access /dev/video*)";
#endif
            break;
        case APC_Init_Fail:
#ifdef _WIN32
            s += " (SDK initialization failed; check that eSPDI_DM.dll and "
                 "the system OpenCL runtime are loadable)";
#else
            s += " (SDK initialization failed; check libeSPDI loaded and "
                 "/dev/video* permissions)";
#endif
            break;
        case APC_OPEN_DEVICE_FAIL:
        case APC_NOT_SUPPORT_RES:
            s += " (the device rejected this mode; a USB2 link cannot open "
                 "USB3-only modes; check the negotiated USB speed)";
            break;
        case APC_VIDEO_RENDER_FAIL:
            s += " (the streams would not start; most often the camera is "
                 "already open in another process; one process at a time "
                 "per camera)";
            break;
        // Busy / timeout codes exist only in the Linux SDK's header.
#ifdef APC_DEVICE_BUSY
        case APC_DEVICE_BUSY:
            s += " (device busy; another process may hold the camera)";
            break;
#endif
#ifdef APC_DEVICE_TIMEOUT
        case APC_DEVICE_TIMEOUT:
            s += " (device timed out; try re-plugging the camera)";
            break;
#endif
        case APC_READFLASHFAIL:
            s += " (on-camera flash read failed; usually transient)";
            break;
        default:
            break;
    }
    return s;
}

}  // namespace pyeys3d
