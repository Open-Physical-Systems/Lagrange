// SPDX-License-Identifier: Apache-2.0
// lagrange_theme.hpp — Dear ImGui + ImPlot theme generated from design-tokens.json (v0.1).
//
// Usage (once, after ImGui::CreateContext / ImPlot::CreateContext and font loading):
//   lagrange::theme::apply(ui_scale);
// Fonts are loaded separately, see lagrange/viewer/Fonts.hpp.
//
// This is the viewer's copy of docs/design/ui/shared/lagrange_theme.hpp. The enum and style
// names were checked against the pinned Dear ImGui (v1.92.9b-docking) and ImPlot (v1.0) in
// vendor/imgui_vendor. Keep the tokens in sync with docs/design/ui/shared/design-tokens.json.
#pragma once

#include <cstdint>

#include <imgui.h>
#include <implot.h>

namespace lagrange::theme {

constexpr ImVec4 hex(std::uint32_t rgb, float a = 1.0F) {
    return ImVec4(static_cast<float>((rgb >> 16) & 0xFF) / 255.0F,
                  static_cast<float>((rgb >> 8) & 0xFF) / 255.0F,
                  static_cast<float>(rgb & 0xFF) / 255.0F, a);
}

// ---- tokens (keep in sync with design-tokens.json) ----
namespace c {
inline constexpr ImVec4 bg0 = hex(0x070D17);
inline constexpr ImVec4 bg1 = hex(0x0B1422);
inline constexpr ImVec4 bg2 = hex(0x101B2C);
inline constexpr ImVec4 bg3 = hex(0x16243A);
inline constexpr ImVec4 bg4 = hex(0x1D2E48);
inline constexpr ImVec4 line = hex(0x1B293D);
inline constexpr ImVec4 lineStrong = hex(0x2A3F5C);
inline constexpr ImVec4 t1 = hex(0xE6EBF2);
inline constexpr ImVec4 t2 = hex(0xA9B6C8);
inline constexpr ImVec4 t3 = hex(0x8190A6);
inline constexpr ImVec4 accent = hex(0xE8772E);
inline constexpr ImVec4 accentWeak = hex(0xE8772E, 0.15F);
inline constexpr ImVec4 accentBorder = hex(0xE8772E, 0.45F);
inline constexpr ImVec4 onAccent = hex(0x170A01);
inline constexpr ImVec4 ok = hex(0x3FB97F);
inline constexpr ImVec4 warn = hex(0xE5B33B);
inline constexpr ImVec4 error = hex(0xF04A4A);
inline constexpr ImVec4 info = hex(0x5AA2FF);
}  // namespace c

// Physics-domain identity colors. Index with the Domain enum used by the protocol.
enum class Domain : std::uint8_t {
    RigidBody,
    Electrical,
    Thermal,
    Aerodynamics,
    Weather,
    Emi,
    Radiation,
    Sensors,
    Count
};
inline constexpr ImVec4 kDomainColor[static_cast<int>(Domain::Count)] = {
    hex(0x9AACC6), hex(0xF5C842), hex(0xFF7A6B), hex(0x5CC8E6),
    hex(0x7FA8FF), hex(0xB48CFF), hex(0x9BE15D), hex(0x3FD1A7),
};

// Plot series palette (never the domain colors).
inline constexpr ImVec4 kSeries[8] = {
    hex(0x5AA2FF), hex(0xF5C842), hex(0xFF7A6B), hex(0x3FD1A7),
    hex(0xB48CFF), hex(0x5CC8E6), hex(0xE8772E), hex(0x9BE15D),
};

// Simulated series are drawn solid at this weight (design tokens: seriesRules.sim). ImPlot v1.0
// has no global line weight any more; pass it per item through ImPlotSpec::LineWeight.
inline constexpr float kSeriesLineWeight = 1.3F;

inline void applyStyle(float scale) {
    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();
    s.WindowPadding = ImVec2(8, 8);
    s.FramePadding = ImVec2(6, 3);
    s.ItemSpacing = ImVec2(8, 4);
    s.ItemInnerSpacing = ImVec2(4, 4);
    s.IndentSpacing = 14;
    s.ScrollbarSize = 10;
    s.GrabMinSize = 8;
    s.WindowRounding = 3;
    s.ChildRounding = 3;
    s.FrameRounding = 3;
    s.PopupRounding = 4;
    s.TabRounding = 3;
    s.ScrollbarRounding = 3;
    s.GrabRounding = 3;
    s.WindowBorderSize = 1;
    s.ChildBorderSize = 1;
    s.FrameBorderSize = 1;
    s.PopupBorderSize = 1;
    s.TabBorderSize = 0;
    s.TabBarOverlineSize = 2;
    s.WindowMenuButtonPosition = ImGuiDir_None;
    s.DockingSeparatorSize = 4;  // matches 4 px dock gap

    ImVec4* k = s.Colors;
    k[ImGuiCol_Text] = c::t1;
    k[ImGuiCol_TextDisabled] = c::t3;
    k[ImGuiCol_WindowBg] = c::bg1;
    k[ImGuiCol_ChildBg] = c::bg1;
    k[ImGuiCol_PopupBg] = c::bg2;
    k[ImGuiCol_Border] = c::line;
    k[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
    k[ImGuiCol_FrameBg] = c::bg0;
    k[ImGuiCol_FrameBgHovered] = c::bg3;
    k[ImGuiCol_FrameBgActive] = c::bg4;
    k[ImGuiCol_TitleBg] = c::bg2;
    k[ImGuiCol_TitleBgActive] = c::bg2;
    k[ImGuiCol_TitleBgCollapsed] = c::bg2;
    k[ImGuiCol_MenuBarBg] = c::bg1;
    k[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
    k[ImGuiCol_ScrollbarGrab] = c::bg4;
    k[ImGuiCol_ScrollbarGrabHovered] = c::lineStrong;
    k[ImGuiCol_ScrollbarGrabActive] = c::t3;
    k[ImGuiCol_CheckMark] = c::onAccent;  // drawn on an accent-filled box, see widgets::Checkbox
    k[ImGuiCol_SliderGrab] = c::accent;
    k[ImGuiCol_SliderGrabActive] = c::accent;
    k[ImGuiCol_Button] = c::bg3;
    k[ImGuiCol_ButtonHovered] = c::bg4;
    k[ImGuiCol_ButtonActive] = c::lineStrong;
    k[ImGuiCol_Header] = c::accentWeak;  // selected tree rows / selectables
    k[ImGuiCol_HeaderHovered] = c::bg3;
    k[ImGuiCol_HeaderActive] = c::accentWeak;
    k[ImGuiCol_Separator] = c::line;
    k[ImGuiCol_SeparatorHovered] = c::accentBorder;
    k[ImGuiCol_SeparatorActive] = c::accent;
    k[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
    k[ImGuiCol_ResizeGripHovered] = c::accentBorder;
    k[ImGuiCol_ResizeGripActive] = c::accent;
    k[ImGuiCol_Tab] = c::bg2;
    k[ImGuiCol_TabHovered] = c::bg3;
    k[ImGuiCol_TabSelected] = c::bg1;
    k[ImGuiCol_TabSelectedOverline] = c::accent;
    k[ImGuiCol_TabDimmed] = c::bg2;
    k[ImGuiCol_TabDimmedSelected] = c::bg1;
    k[ImGuiCol_TabDimmedSelectedOverline] = c::lineStrong;
    k[ImGuiCol_DockingPreview] = c::accentBorder;
    k[ImGuiCol_DockingEmptyBg] = c::bg0;
    k[ImGuiCol_PlotLines] = kSeries[0];
    k[ImGuiCol_PlotLinesHovered] = c::accent;
    k[ImGuiCol_PlotHistogram] = kSeries[0];
    k[ImGuiCol_PlotHistogramHovered] = c::accent;
    k[ImGuiCol_TableHeaderBg] = c::bg2;
    k[ImGuiCol_TableBorderStrong] = c::line;
    k[ImGuiCol_TableBorderLight] = c::line;
    k[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
    k[ImGuiCol_TableRowBgAlt] = ImVec4(1, 1, 1, 0.015F);
    k[ImGuiCol_TextSelectedBg] = hex(0xE8772E, 0.30F);
    k[ImGuiCol_DragDropTarget] = c::accent;
    k[ImGuiCol_NavCursor] = c::accent;
    k[ImGuiCol_NavWindowingHighlight] = c::accent;
    k[ImGuiCol_NavWindowingDimBg] = hex(0x000000, 0.45F);
    k[ImGuiCol_ModalWindowDimBg] = hex(0x000000, 0.55F);

    s.ScaleAllSizes(scale);
    s.FontScaleDpi = scale;
}

inline void applyPlotStyle() {
    ImPlotStyle& p = ImPlot::GetStyle();
    p.PlotPadding = ImVec2(8, 6);
    p.LabelPadding = ImVec2(4, 2);
    p.LegendPadding = ImVec2(6, 4);
    p.Colors[ImPlotCol_FrameBg] = c::bg0;
    p.Colors[ImPlotCol_PlotBg] = c::bg0;
    p.Colors[ImPlotCol_PlotBorder] = c::line;
    p.Colors[ImPlotCol_LegendBg] = hex(0x0B1422, 0.88F);
    p.Colors[ImPlotCol_LegendBorder] = c::lineStrong;
    p.Colors[ImPlotCol_LegendText] = c::t2;
    p.Colors[ImPlotCol_TitleText] = c::t1;
    p.Colors[ImPlotCol_InlayText] = c::t2;
    p.Colors[ImPlotCol_AxisText] = c::t3;
    p.Colors[ImPlotCol_AxisGrid] = c::bg3;
    p.Colors[ImPlotCol_AxisTick] = c::lineStrong;
    p.Colors[ImPlotCol_Selection] = c::accent;
    p.Colors[ImPlotCol_Crosshairs] = c::t2;
    static const ImVec4 series[8] = {kSeries[0], kSeries[1], kSeries[2], kSeries[3],
                                     kSeries[4], kSeries[5], kSeries[6], kSeries[7]};
    const ImPlotColormap cmap = ImPlot::AddColormap("Lagrange", series, 8, /*qual=*/true);
    p.Colormap = cmap;
}

inline void apply(float scale = 1.0F) {
    applyStyle(scale);
    applyPlotStyle();
}

}  // namespace lagrange::theme
