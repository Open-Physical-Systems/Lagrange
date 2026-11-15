#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

/// What the viewer measures of itself for the Performance workspace (design 11-performance:
/// viewer frame, GPU passes, threads, memory). Nothing here comes from a server: the viewer is a
/// separate process, and its numbers are its own (CLAUDE.md §3 invariant 14).
namespace lagrange::viewer {

/// The viewer's frame as the application measured it, filled once a frame by the frame loop.
struct ViewerTimings {
    /// One GPU pass of the viewer's frame, timed with Vulkan timestamp queries.
    struct Pass {
        const char* name{""};     ///< "3D view", "panels (ImGui)", ...
        double meanMs{0.0};       ///< GPU time, mean over the recent frames that ran it
        double maxMs{0.0};        ///< worst of those frames
        std::uint32_t frames{0};  ///< how many of the recent frames ran it
    };
    /// The GPU timestamps work on this device (VkQueueFamilyProperties::timestampValidBits) and
    /// at least one frame has been read back.
    bool gpuTimed{false};
    bool gpuTimestamps{false};  ///< The device can time the viewer's queue at all.
    std::vector<Pass> passes;   ///< In the order the frame records them.
    std::uint32_t window{0};    ///< Frames the means are over.

    /// CPU time of the UI thread per frame (ms, smoothed): building the panels, recording the
    /// command buffer, waiting for the GPU, presenting. `presentMs` is 0 headless.
    double uiMs{0.0};
    double recordMs{0.0};
    double waitMs{0.0};
    double presentMs{0.0};

    /// Device-local memory the viewer uses and may use, from VK_EXT_memory_budget (per process).
    bool vramKnown{false};
    std::uint64_t vramUsedBytes{0};
    std::uint64_t vramBudgetBytes{0};
};

/// One thread of a process, as /proc/<pid>/task/<tid>/stat gives it.
struct ThreadSample {
    int tid{0};
    std::string name;           ///< comm: at most 15 characters
    char state{'?'};            ///< R running, S sleeping, D disk wait, ...
    std::uint64_t cpuTicks{0};  ///< utime + stime, in clock ticks (sysconf(_SC_CLK_TCK))
};

/// Parses one line of /proc/<pid>/task/<tid>/stat. The name is in parentheses and may itself
/// hold spaces and parentheses, so the fields are counted from its last ')'.
[[nodiscard]] std::optional<ThreadSample> parseTaskStat(std::string_view stat);

/// Every thread of this process, by tid; empty where /proc is not readable.
[[nodiscard]] std::vector<ThreadSample> readOwnThreads();

/// Clock ticks per second of ThreadSample::cpuTicks.
[[nodiscard]] double clockTicksPerSecond();

/// This process's memory, from /proc/self/status.
struct ProcessMemory {
    std::uint64_t residentBytes{0};  ///< VmRSS
    std::uint64_t peakBytes{0};      ///< VmHWM: the most resident so far
    std::uint64_t virtualBytes{0};   ///< VmSize
    bool known{false};
};

/// Parses the text of /proc/<pid>/status (the Vm* lines are in kB).
[[nodiscard]] ProcessMemory parseProcessStatus(std::string_view status);
[[nodiscard]] ProcessMemory readOwnMemory();

}  // namespace lagrange::viewer
