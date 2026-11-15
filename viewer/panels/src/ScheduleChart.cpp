#include "PanelFormat.hpp"
#include "ScheduleChart.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::int64_t kScheduleWindowNs = 10'000'000;  // design: a 10 ms window

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

}  // namespace

void drawScheduleChart(std::span<const ScheduleRow> schedule, std::int64_t syncNs) {
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    // Every row at least 16 px high above the axis: a short panel (1024 × 640) scrolls rather
    // than drawing the last rows over the axis.
    ImVec2 size = ImGui::GetContentRegionAvail();
    size.y = std::max(size.y, px(16.0F) * static_cast<float>(schedule.size()) + px(30.0F));
    ImGui::Dummy(size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const theme::Fonts& fonts = theme::fonts();
    const float label = px(theme::kLabelSize);
    const float small = px(theme::kCaptionSize);
    // The names' column gives up some of its 110 px in a narrow panel (QA3 F7, 1024 × 640).
    const float left = origin.x + std::clamp(size.x * 0.45F, px(64.0F), px(110.0F));
    const float right = origin.x + size.x - px(12.0F);
    const float span = std::max(right - left, 1.0F);
    const float axisY = origin.y + size.y - px(20.0F);
    const auto toX = [&](std::int64_t ns) {
        return left + static_cast<float>(ns) / static_cast<float>(kScheduleWindowNs) * span;
    };

    // Sync barriers and the time axis.
    if (syncNs > 0 && syncNs <= kScheduleWindowNs) {
        for (std::int64_t t = 0; t <= kScheduleWindowNs; t += syncNs) {
            draw->AddLine(ImVec2(toX(t), origin.y), ImVec2(toX(t), axisY), u32(c::line));
        }
    }
    draw->AddLine(ImVec2(left, axisY), ImVec2(right, axisY), u32(c::lineStrong));
    // Tick labels centred on their ticks; the last one carries the unit and ends at the edge,
    // so that "10" and "ms" never run into each other (L7).
    const int lastMs = static_cast<int>(kScheduleWindowNs / 1'000'000);
    // Every 2 ms when the numbers have room, else every 5, else only the ends (QA3 F7: at
    // 1024 × 640 "0 2 4 6 8 10" ran into each other).
    const float widest = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, "10 ms").x;
    int every = lastMs;
    for (const int candidate : {2, 5}) {
        if (span * static_cast<float>(candidate) / static_cast<float>(lastMs) >=
            widest + px(8.0F)) {
            every = candidate;
            break;
        }
    }
    for (int ms = 0; ms <= lastMs; ms += every) {
        char text[16];
        std::snprintf(text, sizeof(text), ms == lastMs ? "%d ms" : "%d", ms);
        const float width = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
        const float tick = toX(static_cast<std::int64_t>(ms) * 1'000'000);
        float x = tick - width * 0.5F;
        if (ms == 0) {
            x = tick;
        } else if (ms == lastMs) {
            x = std::min(tick - px(6.0F), origin.x + size.x - px(2.0F) - width);
        }
        draw->AddText(fonts.mono, small, ImVec2(x, axisY + px(4.0F)), u32(c::t3), text);
    }

    // One row per module that runs: ticks when it steps often, blocks when it steps a
    // few times in the window, and the time of its next step when that is beyond it.
    const std::size_t rows = schedule.size();
    const float pitch = std::clamp(
        (axisY - origin.y - px(6.0F)) / static_cast<float>(std::max<std::size_t>(rows, 1)),
        px(16.0F), px(30.0F));
    float y = origin.y + px(4.0F);
    for (const ScheduleRow& node : schedule) {
        const ImVec4 color = widgets::domainColor(node.domain);
        const float midY = y + pitch * 0.5F;
        fit::draw(draw, fonts.sans, label, ImVec2(origin.x + px(4.0F), midY - label * 0.5F),
                  u32(c::t2), node.label, left - origin.x - px(10.0F));
        const float barHalf = std::min(pitch * 0.32F, px(8.0F));
        char text[64];
        if (node.dtNs <= 0) {
            std::snprintf(text, sizeof(text), "%s",
                          node.onGpu ? "GPU · once per frame" : "runs on demand");
            fit::draw(draw, fonts.sans, small, ImVec2(left + px(4.0F), midY - small * 0.5F),
                      u32(c::t3), text, right + px(10.0F) - left - px(4.0F));
        } else if (node.dtNs > kScheduleWindowNs) {
            char step[24];
            format::step(node.dtNs, step, sizeof(step));
            draw->AddRectFilled(ImVec2(left, midY - barHalf),
                                ImVec2(left + px(8.0F), midY + barHalf), u32(color), px(2.0F));
            std::snprintf(text, sizeof(text), "next at %s", step);
            fit::draw(draw, fonts.sans, small, ImVec2(left + px(14.0F), midY - small * 0.5F),
                      u32(c::t2), text, right + px(10.0F) - left - px(14.0F));
        } else {
            const float stepWidth =
                static_cast<float>(node.dtNs) / static_cast<float>(kScheduleWindowNs) * span;
            const bool ticks = stepWidth < px(10.0F);
            const float width = ticks ? std::max(px(1.5F), stepWidth * 0.4F)
                                      : std::min(stepWidth * 0.35F, px(14.0F));
            for (std::int64_t t = 0; t < kScheduleWindowNs; t += node.dtNs) {
                const float x = toX(t);
                draw->AddRectFilled(ImVec2(x, midY - barHalf), ImVec2(x + width, midY + barHalf),
                                    u32(color), ticks ? 0.0F : px(2.0F));
            }
        }
        y += pitch;
    }
}

}  // namespace lagrange::viewer
