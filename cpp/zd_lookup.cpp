#include "zd_lookup.hpp"

#include <chrono>
#include <thread>

#include "eSPDI.h"
#include "apc_error.hpp"
#include "log.hpp"

// The Windows header names the 4096-byte 11-bit table size without the
// bit-width suffix; both SDKs agree on the value.
#ifndef APC_ZD_TABLE_FILE_SIZE_11_BITS
#define APC_ZD_TABLE_FILE_SIZE_11_BITS APC_ZD_TABLE_FILE_SIZE
#endif

namespace pyeys3d {

namespace {
const char* logger() { return "ZdLookup"; }

constexpr int kMaxAttempts = 4;
constexpr int kBackoffMs   = 150;
// APC_GetZDTable requires an exact buffer size for the configured
// disparity width. All dtypes routed through this loader are 11-bit
// variants (4 / 9 / 20 / 52), so the 11-bit constant applies.
constexpr int kBufBytes    = APC_ZD_TABLE_FILE_SIZE_11_BITS;  // 4096
}  // namespace

bool load_zd_table(void* sdk_handle, void* dev_sel_info,
                   int zd_index, ZdTable& out) {
    out.mm.clear();
    out.valid = false;

    // nDataType is pinned to APC_DEPTH_DATA_11_BITS to match kBufBytes.
    // The firmware does not key the lookup on this field beyond the
    // size consistency check, so the table is independent of the
    // currently configured depth stream.
    ZDTABLEINFO info{};
    info.nIndex    = zd_index;
    info.nDataType = APC_DEPTH_DATA_11_BITS;

    std::vector<uint8_t> buf(kBufBytes);
    int actual_len = 0;
    int rc = APC_OK;

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        rc = APC_GetZDTable(
            sdk_handle,
            static_cast<PDEVSELINFO>(dev_sel_info),
            buf.data(),
            kBufBytes,
            &actual_len,
            &info);
        if (rc == APC_OK) break;
        if (rc != APC_READFLASHFAIL) break;
        PYEYS3D_WARN(logger(),
                    "APC_GetZDTable(zd_index=%d) %s (flash read), "
                    "retry %d/%d after %d ms",
                    zd_index, apc_strerror(rc).c_str(),
                    attempt + 1, kMaxAttempts, kBackoffMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(kBackoffMs));
    }

    if (rc != APC_OK) {
        PYEYS3D_WARN(logger(),
                    "APC_GetZDTable(zd_index=%d) failed: %s",
                    zd_index, apc_strerror(rc).c_str());
        return false;
    }
    // The upper bound is the buffer that was handed over: the length is
    // the SDK's word for how much it wrote, and reading past kBufBytes on
    // its say-so would walk off the vector into whatever follows it.
    if (actual_len < 2 || (actual_len & 1) != 0 || actual_len > kBufBytes) {
        PYEYS3D_WARN(logger(),
                    "APC_GetZDTable returned implausible length %d "
                    "(expected even, 2..%d bytes)",
                    actual_len, kBufBytes);
        return false;
    }

    // Firmware emits big-endian uint16 entries; index 0 is the
    // hole-sentinel slot.
    const int n_entries = actual_len / 2;
    out.mm.resize(static_cast<size_t>(n_entries));
    const uint8_t* p = buf.data();
    for (int i = 0; i < n_entries; ++i) {
        const uint16_t v = (static_cast<uint16_t>(p[0]) << 8)
                         | static_cast<uint16_t>(p[1]);
        out.mm[static_cast<size_t>(i)] = static_cast<int32_t>(v);
        p += 2;
    }
    out.valid = true;

    // Not a min/max: entry 0 is the hole sentinel and Z falls as
    // disparity rises, so this is the far end of the table and the near.
    PYEYS3D_INFO(logger(),
                "ZD table loaded: %d entries, disparity 1 -> %d mm, "
                "disparity %d -> %d mm (index=%d)",
                n_entries,
                n_entries > 1 ? out.mm[1] : 0,
                n_entries - 1, out.mm.back(),
                zd_index);
    return true;
}

int32_t zd_lookup_q4(const ZdTable& tbl, uint16_t d_q4) {
    if (d_q4 == 0 || tbl.mm.empty()) return 0;

    const int n = static_cast<int>(tbl.mm.size());

#ifdef EYS3D_ZD_ROUND_NEAREST
    int d_int = (static_cast<int>(d_q4) + 8) >> 4;
    if (d_int >= n) d_int = n - 1;
    return tbl.mm[static_cast<size_t>(d_int)];
#else
    int d_int = static_cast<int>(d_q4) >> 4;
    const int frac = static_cast<int>(d_q4) & 0xF;
    if (d_int >= n - 1) return tbl.mm.back();
    const int32_t a = tbl.mm[static_cast<size_t>(d_int)];
    const int32_t b = tbl.mm[static_cast<size_t>(d_int) + 1];
    return a + ((b - a) * frac >> 4);
#endif
}

}  // namespace pyeys3d
