#pragma once
#include <cstdio>
#include <mutex>
#include <string>

namespace franka_rt {

// Minimal stdout logger — libfranka does not pull in spdlog, so we keep this
// dependency-free.  Good enough for the volume of messages we emit (mostly
// init/teardown + errors).
//
//   logger().info("connected to %s", addr);
//   logger().warn("limit clamped");
//   logger().error("control loop exception: %s", e.what());

struct Logger {
    enum Level { kInfo, kWarn, kError };

    template <typename... Args>
    void log(Level lv, const char* fmt, Args... args) {
        std::lock_guard<std::mutex> g(mtx_);
        const char* tag = (lv == kError) ? "[ERR]" : (lv == kWarn) ? "[WRN]" : "[INF]";
        std::fprintf(stderr, "[franka_rt] %s ", tag);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
        std::fprintf(stderr, fmt, args...);
#pragma GCC diagnostic pop
        std::fputc('\n', stderr);
    }
    template <typename... Args>
    void info(const char* fmt, Args... args)  { log(kInfo,  fmt, args...); }
    template <typename... Args>
    void warn(const char* fmt, Args... args)  { log(kWarn,  fmt, args...); }
    template <typename... Args>
    void error(const char* fmt, Args... args) { log(kError, fmt, args...); }

  private:
    std::mutex mtx_;
};

inline Logger& logger() {
    static Logger instance;
    return instance;
}

}  // namespace franka_rt
