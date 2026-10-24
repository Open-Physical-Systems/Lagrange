#pragma once

// The log of a run (roadmap 0.2-A12): what modules report goes to one sink, which the server
// forwards to the viewer's console; without one, to stderr.

#include <cstdint>
#include <functional>
#include <string_view>

namespace lagrange::services {

enum class LogLevel : std::uint8_t { Debug, Info, Warning, Error };

/// Receives every log line: the level, who logs ("electrical/acme_drive", "plugin") and the
/// text. Called from the thread that logs; keep it short.
using LogSink = std::function<void(LogLevel, std::string_view source, std::string_view text)>;

/// Installs the sink for the whole process; an empty sink restores stderr.
void setLogSink(LogSink sink);

/// Logs one line through the current sink. Thread-safe.
void log(LogLevel level, std::string_view source, std::string_view text);

}  // namespace lagrange::services
