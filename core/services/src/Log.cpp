#include "lagrange/services/Log.hpp"

#include <cstdio>
#include <mutex>
#include <utility>

namespace lagrange::services {

namespace {

std::mutex& sinkMutex() {
    static std::mutex mutex;
    return mutex;
}

LogSink& sink() {
    static LogSink current;
    return current;
}

const char* name(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:
            return "debug";
        case LogLevel::Info:
            return "info";
        case LogLevel::Warning:
            return "warning";
        case LogLevel::Error:
            return "error";
    }
    return "info";
}

}  // namespace

void setLogSink(LogSink next) {
    const std::lock_guard<std::mutex> lock(sinkMutex());
    sink() = std::move(next);
}

void log(LogLevel level, std::string_view source, std::string_view text) {
    const std::lock_guard<std::mutex> lock(sinkMutex());
    if (sink()) {
        sink()(level, source, text);
        return;
    }
    std::fprintf(stderr, "[%.*s %s] %.*s\n", static_cast<int>(source.size()), source.data(),
                 name(level), static_cast<int>(text.size()), text.data());
}

}  // namespace lagrange::services
