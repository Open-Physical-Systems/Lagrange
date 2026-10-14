#pragma once

#include <cstdint>
#include <span>
#include <string_view>

namespace lagrange::viewer {

/// One line of a schedule chart: something that steps at a fixed interval.
struct ScheduleRow {
    const char* label;
    std::string_view domain;  ///< For its colour.
    std::int64_t dtNs;        ///< 0: no step of its own.
    bool onGpu;
};

/// Fills the rest of the current window with a 10 ms window of the schedule: ticks for
/// modules that step often, blocks for those that step a few times, the time of the next step
/// for those beyond the window, and a line at every sync barrier (design: Schedule,
/// Scheduler).
void drawScheduleChart(std::span<const ScheduleRow> schedule, std::int64_t syncNs);

}  // namespace lagrange::viewer
