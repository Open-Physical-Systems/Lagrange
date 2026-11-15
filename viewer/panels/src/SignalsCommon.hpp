#pragma once

// Small pieces shared by the Signals workspace's panels (SignalsPanel.cpp, SignalsTabs.cpp):
// the tables of Cursors, Measure and Statistics look the same.

#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/ScopeSession.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <cmath>
#include <cstdio>
#include <string>

#include <imgui.h>

namespace lagrange::viewer::signals {

// Resizable: a long signal name can be given more of the width (M4).
inline constexpr ImGuiTableFlags kTableFlags =
    ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp |
    ImGuiTableFlags_NoPadOuterX | ImGuiTableFlags_Resizable;

/// Mono text ending at the right edge of the cell; in a cell too narrow for it, cut with an
/// ellipsis (whole in a tooltip) rather than drawn over the next cell.
inline void rightMono(const ImVec4& color, const char* text) {
    ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    fit::textRight(text);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

/// A number, or "-" for NaN.
inline void number(double value, char* out, std::size_t size, const char* format = "%.4g") {
    if (std::isnan(value)) {
        std::snprintf(out, size, "-");
    } else {
        std::snprintf(out, size, format, value);
    }
}

inline void headerCell(const char* text, bool right) {
    ImGui::TableNextColumn();
    if (right) {
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        widgets::AlignRight(ImGui::CalcTextSize(text).x);
        ImGui::PopFont();
    }
    widgets::Caption("%s", text);
}

/// The first cell of a trace's row: its series swatch and its name, cut in the middle.
inline void traceCell(const ScopeTrace& trace) {
    ImGui::TableNextColumn();
    widgets::Swatch(theme::kSeries[trace.color % ScopeSession::kColors]);
    ImGui::SameLine(0.0F, widgets::px(6.0F));
    // All of the name that fits, cut in the middle; whole in the tooltip.
    const std::string shown = fit::cut(ImGui::GetFont(), ImGui::GetFontSize(), trace.label,
                                       ImGui::GetContentRegionAvail().x, true);
    ImGui::TextUnformatted(shown.c_str());
    ImGui::SetItemTooltip("%s  [%s]", trace.label.c_str(), trace.unit.c_str());
}

}  // namespace lagrange::viewer::signals
