// Shared internals of the CaptureEngine translation units: the common
// engine (capture_engine.cpp) and the per-OS fetch layer
// (capture_linux.cpp / capture_windows.cpp). Not part of the public
// surface; include only from those files.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

#include "eSPDI_def.h"

namespace pyeys3d {

inline constexpr char kTag[] = "CaptureEngine";

// The CT/PU property-value type the eSPDI prop calls take: long int on
// Linux, int on Windows.
#ifdef _WIN32
using ApcPropVal = int;
#else
using ApcPropVal = long int;
#endif

// Serial-read attempts during enumeration: retried on Windows, where a
// concurrently-starting process can hold the control pipe transiently;
// single-shot on Linux, where a failed read is persistent (device held
// by another process) and retrying would stall every enumerate.
#ifdef _WIN32
inline constexpr int kSerialReadAttempts = 5;
#else
inline constexpr int kSerialReadAttempts = 1;
#endif

inline int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline std::string utf16le_to_ascii(const unsigned char* buf, int byte_len,
                                    int buf_cap) {
    const int chars = std::min(byte_len / 2, buf_cap / 2);
    std::string out(static_cast<size_t>(chars), '\0');
    for (int j = 0; j < chars; ++j) out[j] = static_cast<char>(buf[j * 2]);
    // The SDK hands back the raw flash payload, which some units pad
    // with NULs or spaces — trim so serial matching and display never
    // see the padding.
    while (!out.empty() && (out.back() == '\0' || out.back() == ' '))
        out.pop_back();
    return out;
}

// eSP876 USB self-reset register sequence, {address, value}. The leading
// writes reconfigure the link; the final write triggers the detach so the
// host re-enumerates the device as if replugged. Split at kResetDetachIdx:
// the platforms that block on the detach write run everything before it
// under the control lock and the detach write separately.
struct ResetReg { unsigned short addr; unsigned char val; };
inline constexpr ResetReg kResetSeq[] = {
    {0xF069, 0xF3}, {0xF0B0, 0x00}, {0xF0D2, 0x80}, {0xF0D2, 0x00},
    {0xF11A, 0x44}, {0xF11A, 0x40}, {0xF0F0, 0x00}, {0xF0FC, 0x20},
    {0xF0FC, 0x00}, {0xF0E0, 0x00}, {0xF500, 0x40}, {0xF500, 0x00},
    {0xF01E, 0x45},
};
inline constexpr size_t kResetDetachIdx = std::size(kResetSeq) - 1;

// Every depth data type in the model catalogs is 2 bytes per pixel, and the
// depth path is written to that throughout: the Z14 mask, the Q4 promotion,
// every uint16 view. A frame shorter than this is dropped by ingest_depth.
inline size_t depth_buffer_bytes(int w, int h) {
    return static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
}

}  // namespace pyeys3d
