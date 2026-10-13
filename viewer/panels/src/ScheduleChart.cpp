#include "PanelFormat.hpp"
#include "ScheduleChart.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
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
    const ImVec2 size = ImGui::GetContentRegionAvail();
    ImGui::Dummy(size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const theme::Fonts& fonts = theme::fonts();
    const float label = px(theme::kLabelSize);
    const float small = px(theme::kCaptionSize);
    const float left = origin.x + px(110.0F);
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
    for (int ms = 0; ms <= 10; ms += 2) {
        char text[8];
        std::snprintf(text, sizeof(text), "%d", ms);
        draw->AddText(
            fonts.mono, small,
            ImVec2(toX(static_cast<std::int64_t>(ms) * 1'000'000) - px(3.0F), axisY + px(4.0F)),
            u32(c::t3), text);
    }
    draw->AddText(fonts.mono, small, ImVec2(right - px(14.0F), axisY + px(4.0F)), u32(c::t3),
                  " ms");

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
        draw->AddText(fonts.sans, label, ImVec2(origin.x + px(4.0F), midY - label * 0.5F),
                      u32(c::t2), node.label);
        const float barHalf = std::min(pitch * 0.32F, px(8.0F));
        char text[64];
        if (node.dtNs <= 0) {
            std::snprintf(text, sizeof(text), "%s",
                          node.onGpu ? "GPU · once per frame" : "runs on demand");
            draw->AddText(fonts.sans, small, ImVec2(left + px(4.0F), midY - small * 0.5F),
                          u32(c::t3), text);
        } else if (node.dtNs > kScheduleWindowNs) {
            char step[24];
            format::step(node.dtNs, step, sizeof(step));
            draw->AddRectFilled(ImVec2(left, midY - barHalf),
                                ImVec2(left + px(8.0F), midY + barHalf), u32(color), px(2.0F));
            std::snprintf(text, sizeof(text), "next at %s", step);
            draw->AddText(fonts.sans, small, ImVec2(left + px(14.0F), midY - small * 0.5F),
                          u32(c::t2), text);
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
