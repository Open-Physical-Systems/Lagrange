// What the viewer reads of itself from /proc for the Performance workspace's Threads and
// Memory tabs (docs/design/ui/11-performance/SPEC.md). Linux only, like the whole product
// (CLAUDE.md §1a).
#include "lagrange/viewer/ViewerSelf.hpp"

#include <algorithm>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <unistd.h>

namespace lagrange::viewer {

namespace {

std::string readFile(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        return {};
    }
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// The next space-separated field of `text` from `at`, which is moved past it.
std::string_view nextField(std::string_view text, std::size_t& at) {
    while (at < text.size() && text[at] == ' ') {
        ++at;
    }
    const std::size_t start = at;
    while (at < text.size() && text[at] != ' ' && text[at] != '\n') {
        ++at;
    }
    return text.substr(start, at - start);
}

std::uint64_t toUnsigned(std::string_view text) {
    std::uint64_t value = 0;
    std::from_chars(text.data(), text.data() + text.size(), value);
    return value;
}

}  // namespace

std::optional<ThreadSample> parseTaskStat(std::string_view stat) {
    // "tid (name) state ppid ..." — proc(5): utime is field 14 and stime field 15.
    const std::size_t open = stat.find('(');
    const std::size_t close = stat.rfind(')');
    if (open == std::string_view::npos || close == std::string_view::npos || close < open) {
        return std::nullopt;
    }
    ThreadSample sample;
    const std::string_view tid = stat.substr(0, open);
    std::from_chars(tid.data(), tid.data() + tid.size(), sample.tid);
    sample.name = std::string(stat.substr(open + 1, close - open - 1));
    std::size_t at = close + 1;
    const std::string_view state = nextField(stat, at);
    if (state.empty()) {
        return std::nullopt;
    }
    sample.state = state.front();
    // Fields 4 to 13, then utime (14) and stime (15).
    for (int field = 4; field <= 13; ++field) {
        if (nextField(stat, at).empty()) {
            return std::nullopt;
        }
    }
    const std::string_view utime = nextField(stat, at);
    const std::string_view stime = nextField(stat, at);
    if (utime.empty() || stime.empty()) {
        return std::nullopt;
    }
    sample.cpuTicks = toUnsigned(utime) + toUnsigned(stime);
    return sample;
}

std::vector<ThreadSample> readOwnThreads() {
    std::vector<ThreadSample> threads;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task", ec)) {
        if (std::optional<ThreadSample> sample = parseTaskStat(readFile(entry.path() / "stat"))) {
            threads.push_back(std::move(*sample));
        }
    }
    std::ranges::sort(threads, {}, &ThreadSample::tid);
    return threads;
}

double clockTicksPerSecond() {
    const long ticks = sysconf(_SC_CLK_TCK);
    return ticks > 0 ? static_cast<double>(ticks) : 100.0;  // 100: the Linux USER_HZ default
}

ProcessMemory parseProcessStatus(std::string_view status) {
    ProcessMemory memory;
    std::size_t start = 0;
    while (start < status.size()) {
        std::size_t end = status.find('\n', start);
        if (end == std::string_view::npos) {
            end = status.size();
        }
        const std::string_view line = status.substr(start, end - start);
        start = end + 1;
        const std::size_t colon = line.find(':');
        if (colon == std::string_view::npos) {
            continue;
        }
        const std::string_view key = line.substr(0, colon);
        std::uint64_t* target = key == "VmRSS"    ? &memory.residentBytes
                                : key == "VmHWM"  ? &memory.peakBytes
                                : key == "VmSize" ? &memory.virtualBytes
                                                  : nullptr;
        if (target == nullptr) {
            continue;
        }
        std::size_t at = colon + 1;
        while (at < line.size() && (line[at] == ' ' || line[at] == '\t')) {
            ++at;
        }
        std::size_t digits = at;
        while (digits < line.size() && line[digits] >= '0' && line[digits] <= '9') {
            ++digits;
        }
        *target = toUnsigned(line.substr(at, digits - at)) * 1024;  // proc(5): in kB
        memory.known = true;
    }
    return memory;
}

ProcessMemory readOwnMemory() {
    return parseProcessStatus(readFile("/proc/self/status"));
}

}  // namespace lagrange::viewer
