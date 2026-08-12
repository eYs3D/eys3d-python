// Windows implementation of the platform seam.

#include "platform.hpp"

namespace pyeys3d {

// DEVINFORMATION.wUsbNode is not populated reliably on Windows — every
// device can report the same number. The device path's instance segment
// ("6&35c4e9&0&0000" in \\?\usb#vid_...&mi_00#6&35c4e9&0&0000#{clsid})
// is what Windows itself uses to tell units apart: unique per attached
// device and stable for the same device on the same physical port.
std::string resolve_usb_port(const DEVINFORMATION& info) {
    if (!info.strDevPath) return {};
    const std::string path(info.strDevPath);
    const size_t a = path.find('#');
    if (a == std::string::npos) return {};
    const size_t b = path.find('#', a + 1);
    if (b == std::string::npos) return {};
    const size_t c = path.find('#', b + 1);
    return path.substr(b + 1,
                       (c == std::string::npos ? path.size() : c) - b - 1);
}

// APC_Init leaves process signal handlers alone on Windows, so there is
// nothing to capture or restore.
SignalHandlerGuard::SignalHandlerGuard() = default;
SignalHandlerGuard::~SignalHandlerGuard() = default;

}  // namespace pyeys3d
