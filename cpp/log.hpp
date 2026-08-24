// Lightweight logging for the pyeys3d native layer.
//
// A Python extension has no logging framework of its own, so these
// macros write to stderr with a severity prefix. Output is opt-out via the
// PYEYS3D_LOG_LEVEL environment variable ("error" / "warn" / "info" /
// "debug" / "none"), defaulting to "warn".

#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace pyeys3d {

enum class LogLevel : int { None = 0, Error = 1, Warn = 2, Info = 3, Debug = 4 };

inline LogLevel current_log_level() {
    static LogLevel cached = [] {
        const char* env = std::getenv("PYEYS3D_LOG_LEVEL");
        if (!env) return LogLevel::Warn;
        if (std::strcmp(env, "none")  == 0) return LogLevel::None;
        if (std::strcmp(env, "error") == 0) return LogLevel::Error;
        if (std::strcmp(env, "warn")  == 0) return LogLevel::Warn;
        if (std::strcmp(env, "info")  == 0) return LogLevel::Info;
        if (std::strcmp(env, "debug") == 0) return LogLevel::Debug;
        return LogLevel::Warn;
    }();
    return cached;
}

inline void log_emit(LogLevel lvl, const char* tag, const char* fmt, ...) {
    if (static_cast<int>(lvl) > static_cast<int>(current_log_level())) return;
    const char* prefix = "";
    switch (lvl) {
        case LogLevel::Error: prefix = "[pyeys3d ERROR]"; break;
        case LogLevel::Warn:  prefix = "[pyeys3d WARN] "; break;
        case LogLevel::Info:  prefix = "[pyeys3d INFO] "; break;
        case LogLevel::Debug: prefix = "[pyeys3d DEBUG]"; break;
        default: return;
    }
    std::fprintf(stderr, "%s %s: ", prefix, tag);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fputc('\n', stderr);
}

}  // namespace pyeys3d

#define PYEYS3D_ERROR(tag, ...) ::pyeys3d::log_emit(::pyeys3d::LogLevel::Error, tag, __VA_ARGS__)
#define PYEYS3D_WARN(tag, ...)  ::pyeys3d::log_emit(::pyeys3d::LogLevel::Warn,  tag, __VA_ARGS__)
#define PYEYS3D_INFO(tag, ...)  ::pyeys3d::log_emit(::pyeys3d::LogLevel::Info,  tag, __VA_ARGS__)
#define PYEYS3D_DEBUG(tag, ...) ::pyeys3d::log_emit(::pyeys3d::LogLevel::Debug, tag, __VA_ARGS__)
