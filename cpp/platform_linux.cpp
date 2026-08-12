// Linux implementation of the platform seam (see platform.hpp).

#include "platform.hpp"

#include <csignal>
#include <filesystem>
#include <regex>
#include <system_error>

namespace pyeys3d {

// /dev/videoN -> the sysfs USB interface (e.g. "2-1:1.0"), the stable
// bus location used for device binding across reboots and re-plugs.
std::string resolve_usb_port(const DEVINFORMATION& info) {
    if (!info.strDevName) return {};
    const std::string device_node(info.strDevName);
    const std::string dev_prefix = "/dev/video";
    if (device_node.compare(0, dev_prefix.size(), dev_prefix) != 0) return {};
    const std::string vname = device_node.substr(5);
    const std::string sysfs_link = "/sys/class/video4linux/" + vname + "/device";
    std::error_code ec;
    auto real = std::filesystem::canonical(sysfs_link, ec);
    if (ec) return {};

    static const std::regex kUsbIfacePattern(R"(^\d+-\d+(?:\.\d+)*:\d+\.\d+$)");
    static const std::regex kUsbDevicePattern(R"(^\d+-\d+(?:\.\d+)*$)");
    for (auto it = real.end(); it != real.begin(); ) {
        --it;
        const std::string s = it->string();
        if (std::regex_match(s, kUsbIfacePattern)) return s;
    }
    for (auto it = real.end(); it != real.begin(); ) {
        --it;
        const std::string s = it->string();
        if (std::regex_match(s, kUsbDevicePattern)) return s;
    }
    return {};
}

namespace {
struct SigState {
    struct sigaction sigint;
    struct sigaction sigterm;
};
}  // namespace

SignalHandlerGuard::SignalHandlerGuard() {
    auto* s = new SigState{};
    sigaction(SIGINT,  nullptr, &s->sigint);
    sigaction(SIGTERM, nullptr, &s->sigterm);
    state_ = s;
}

SignalHandlerGuard::~SignalHandlerGuard() {
    auto* s = static_cast<SigState*>(state_);
    sigaction(SIGINT,  &s->sigint,  nullptr);
    sigaction(SIGTERM, &s->sigterm, nullptr);
    delete s;
}

}  // namespace pyeys3d
