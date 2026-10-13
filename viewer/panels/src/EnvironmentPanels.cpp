// Environment workspace: what the environment is, a slice of one of its fields with a probe,
// and the list of fields that can be shown (docs/design/ui/07-environment/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// The "field" colour map of the design tokens (viridis), by fraction 0..1.
ImVec4 viridis(float fraction) {
    static const std::array<ImVec4, 5> kStops{theme::hex(0x440154), theme::hex(0x3B528B),
                                              theme::hex(0x21918C), theme::hex(0x5EC962),
                                              theme::hex(0xFDE725)};
    const float t = std::clamp(fraction, 0.0F, 1.0F) * 4.0F;
    const auto index = std::min(static_cast<std::size_t>(t), std::size_t{3});
    const float blend = t - static_cast<float>(index);
    const ImVec4& a = kStops.at(index);
    const ImVec4& b = kStops.at(index + 1);
    return {a.x + (b.x - a.x) * blend, a.y + (b.y - a.y) * blend, a.z + (b.z - a.z) * blend, 1.0F};
}

/// What the three panels share: which field is shown and how.
struct EnvironmentView {
    std::string field;  // name of the field shown; empty for none
    std::shared_ptr<FieldStream> stream;
    double heightM{0.1};
    float opacity{0.85F};

    /// Chooses the first available field when none is chosen, and keeps the stream open.
    void follow(PanelContext& context) {
        const std::shared_ptr<const EnvironmentInfo> info = context.client.environment();
        if (!info) {
            return;
        }
        const auto available = [&](const std::string& name) {
            return std::ranges::any_of(info->fields, [&](const EnvironmentInfo::Field& f) {
                return f.name == name && f.available;
            });
        };
        if (!field.empty() && !available(field)) {
            field.clear();
            stream.reset();
        }
        if (field.empty()) {
            for (const EnvironmentInfo::Field& candidate : info->fields) {
                if (candidate.available && candidate.components == 1) {
                    field = candidate.name;
                    break;
                }
            }
        }
        if (!field.empty()) {
            stream = context.client.watchField(field, heightM);
        }
    }
};

bool emptyState(const PanelContext& context) {
    if (context.client.environment()) {
        return false;
    }
    widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                               ? "This server does not describe its environment."
                               : "Not connected. The environment appears when a simulation "
                                 "server is reachable.");
    return true;
}

void row(const char* label, const char* unit, const char* format, double value) {
    char text[48];
    std::snprintf(text, sizeof(text), format, value);
    widgets::PropertyRow(label, text, unit);
}

// --- what the environment is --------------------------------------------------------------

class EnvironmentPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Environment"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Environment###EnvironmentState");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        if (!context.client.environment()) {
            ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            emptyState(context);
            ImGui::End();
            return;
        }
        const EnvironmentInfo& info = *context.client.environment();
        ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(6.0F)));
        widgets::Mono(c::t1, "%s", info.layer.empty() ? "no environment layer" : info.layer.c_str());
        if (info.synthetic) {
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill("not modelled by this server", widgets::PillKind::Warn);
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float pad = px(8.0F);
        const auto domainOn = [&](const char* name) {
            const DomainView* view = context.profileSession->find(name);
            return view == nullptr || view->enabled;
        };

        const ImVec4 thermal = widgets::domainColor("thermal");
        if (widgets::SectionHeader("Time and sun", nullptr, true, &thermal)) {
            ImGui::Indent(pad);
            char time[16];
            const int minutes = static_cast<int>(std::lround(info.localTimeHours * 60.0)) % 1440;
            std::snprintf(time, sizeof(time), "%02d:%02d", minutes / 60, minutes % 60);
            widgets::PropertyRow("local time", time, nullptr);
            row("sun elevation", "°", "%.1f", info.sunElevationDeg);
            row("ambient", "°C", "%.1f", info.ambientTemperatureK - 273.15);
            ImGui::Unindent(pad);
        }
        const ImVec4 weather = widgets::domainColor("weather");
        if (widgets::SectionHeader("Weather", domainOn("weather") ? nullptr : "domain off", true,
                                   &weather)) {
            ImGui::Indent(pad);
            row("rain rate", "mm/h", "%.1f", info.rainMmPerH);
            if (info.fogVisibilityM > 0.0) {
                row("fog visibility", "m", "%.0f", info.fogVisibilityM);
            } else {
                widgets::PropertyRow("fog visibility", "no fog", nullptr);
            }
            row("dust", "mg/m³", "%.1f", info.dustMgPerM3);
            // Surface wetness, 0 dry to 1 standing water, as a meter.
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted("surface wetness");
            ImGui::PopStyleColor();
            char wet[16];
            std::snprintf(wet, sizeof(wet), "%.2f", info.surfaceWetness);
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            const float width = ImGui::CalcTextSize(wet).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(px(120.0F) + px(8.0F) + width + pad);
            widgets::Meter(static_cast<float>(info.surfaceWetness), weather, px(120.0F));
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Mono(c::t1, "%s", wet);
            ImGui::Unindent(pad);
        }
        const ImVec4 aero = widgets::domainColor("aerodynamics");
        if (widgets::SectionHeader("Wind", domainOn("aerodynamics") ? nullptr : "domain off", true,
                                   &aero)) {
            ImGui::Indent(pad);
            row("mean", "m/s", "%.1f", info.windSpeed);
            row("direction", "°", "%.0f", info.windDirectionDeg);
            row("gust intensity", nullptr, "%.2f", info.gustIntensity);
            widgets::PropertyRow("field",
                                 info.windField.empty() ? "none — uniform wind"
                                                        : info.windField.c_str(),
                                 nullptr);
            ImGui::Unindent(pad);
        }
        const ImVec4 emi = widgets::domainColor("emi");
        char count[32];
        std::snprintf(count, sizeof(count), "%zu", info.emiSources.size());
        if (widgets::SectionHeader("EMI sources", count, true, &emi)) {
            ImGui::Indent(pad);
            if (info.emiSources.empty()) {
                widgets::Caption("No EMI source in this stage.");
            }
            for (const EnvironmentInfo::EmiSource& source : info.emiSources) {
                // Name, then frequency and current or power; the frequency is a parameter of
                // the site or the device and is shown as given.
                char numbers[96];
                char frequency[32];
                if (source.frequencyHz >= 1e9) {
                    std::snprintf(frequency, sizeof(frequency), "%g GHz", source.frequencyHz / 1e9);
                } else if (source.frequencyHz >= 1e3) {
                    std::snprintf(frequency, sizeof(frequency), "%g kHz", source.frequencyHz / 1e3);
                } else {
                    std::snprintf(frequency, sizeof(frequency), "%g Hz", source.frequencyHz);
                }
                if (source.currentA != 0.0) {
                    std::snprintf(numbers, sizeof(numbers), "%s · %g A", frequency, source.currentA);
                } else if (source.powerW != 0.0) {
                    std::snprintf(numbers, sizeof(numbers), "%s · %g W", frequency, source.powerW);
                } else {
                    std::snprintf(numbers, sizeof(numbers), "%s", frequency);
                }
                ImGui::PushID(source.name.c_str());
                if (ImGui::Selectable("##source", false, ImGuiSelectableFlags_AllowOverlap,
                                      ImVec2(0.0F, px(40.0F))) &&
                    source.prim != kNone) {
                    context.selectedPrim = source.prim;
                }
                ImGui::PopID();
                const ImVec2 min = ImGui::GetItemRectMin();
                const ImVec2 max = ImGui::GetItemRectMax();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                const theme::Fonts& fonts = theme::fonts();
                draw->AddText(fonts.sansMedium, px(theme::kBodySize), ImVec2(min.x, min.y + px(3.0F)),
                              u32(c::t1), source.name.c_str());
                const float mono = px(theme::kMonoSize);
                const float width = fonts.mono->CalcTextSizeA(mono, FLT_MAX, 0.0F, numbers).x;
                draw->AddText(fonts.mono, mono, ImVec2(max.x - width - pad, min.y + px(4.0F)),
                              u32(c::t1), numbers);
                draw->AddText(fonts.mono, px(theme::kCaptionSize),
                              ImVec2(min.x, min.y + px(23.0F)), u32(c::t3), source.detail.c_str());
            }
            ImGui::Unindent(pad);
        }
        const ImVec4 radiation = widgets::domainColor("radiation");
        widgets::SectionHeader("Radiation", domainOn("radiation") ? "no sources" : "domain off",
                               false, &radiation);
        ImGui::End();
    }
};

// --- a slice of the chosen field ----------------------------------------------------------

class FieldSlicePanel final : public Panel {
public:
    explicit FieldSlicePanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Slice 2D"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin("Slice 2D###FieldSlice");
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        view_->follow(context);
        // Header: the height of the slice.
        if (widgets::BeginPanelTools("##slice-tools", px(150.0F))) {
            ImGui::AlignTextToFramePadding();
            widgets::Caption("height");
            ImGui::SameLine(0.0F, px(6.0F));
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            ImGui::SetNextItemWidth(px(90.0F));
            double height = view_->heightM;
            if (ImGui::InputDouble("##height", &height, 0.0, 0.0, "%.2f m") &&
                std::isfinite(height)) {
                view_->heightM = std::clamp(height, -100.0, 1000.0);
            }
            ImGui::PopFont();
            widgets::EndPanelTools();
        }
        if (!context.client.environment() || view_->field.empty()) {
            ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + px(8.0F), ImGui::GetCursorPosY() + px(8.0F)));
            if (!emptyState(context)) {
                widgets::Caption("No field of this run can be shown. The Overlays panel says why.");
            }
            ImGui::End();
            return;
        }
        const std::shared_ptr<const FieldSliceData> slice =
            view_->stream ? view_->stream->latest() : nullptr;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton("##canvas", size);
        const bool hovered = ImGui::IsItemHovered();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
                            u32(theme::hex(0x091120)));
        if (!slice) {
            draw->AddText(theme::fonts().sans, px(theme::kLabelSize),
                          ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), u32(c::t3),
                          "Waiting for the first slice ...");
            ImGui::End();
            return;
        }
        // Fit the slice into the panel, +X to the right and +Y up, as on a map.
        const double spanX = slice->cellM * slice->width;
        const double spanY = slice->cellM * slice->height;
        const float scale = std::min((size.x - px(40.0F)) / static_cast<float>(spanX),
                                     (size.y - px(40.0F)) / static_cast<float>(spanY));
        const double left = slice->originX - slice->cellM * 0.5;
        const double bottom = slice->originY - slice->cellM * 0.5;
        const ImVec2 base(origin.x + (size.x - static_cast<float>(spanX) * scale) * 0.5F,
                          origin.y + (size.y + static_cast<float>(spanY) * scale) * 0.5F);
        const auto toScreen = [&](double x, double y) {
            return ImVec2(base.x + static_cast<float>(x - left) * scale,
                          base.y - static_cast<float>(y - bottom) * scale);
        };
        const float range = std::max(slice->valueMax - slice->valueMin, 1e-12F);
        const float cell = static_cast<float>(slice->cellM) * scale;
        for (std::uint32_t r = 0; r < slice->height; ++r) {
            for (std::uint32_t col = 0; col < slice->width; ++col) {
                const float value = slice->values[(static_cast<std::size_t>(r) * slice->width + col) *
                                                  slice->components];
                ImVec4 color = viridis((value - slice->valueMin) / range);
                color.w = view_->opacity;
                const ImVec2 min(base.x + static_cast<float>(col) * cell,
                                 base.y - static_cast<float>(r + 1) * cell);
                draw->AddRectFilled(min, ImVec2(min.x + cell + 0.5F, min.y + cell + 0.5F),
                                    u32(color));
            }
        }
        draw->AddRect(toScreen(left, bottom + spanY), toScreen(left + spanX, bottom),
                      u32(c::lineStrong));

        // What is in the slice's area: the robots, at their positions.
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        if (context.scene) {
            const std::vector<PrimData>& prims = context.scene->statics->prims;
            for (std::uint32_t i = 0; i < prims.size(); ++i) {
                if (prims[i].parent != kNone) {
                    continue;
                }
                const bool robot = std::ranges::any_of(
                    prims, [i](const PrimData& other) { return other.parent == i; });
                if (!robot) {
                    continue;
                }
                const auto& position = context.scene->transforms[i].position;
                const ImVec2 at = toScreen(position[0], position[1]);
                draw->AddCircleFilled(at, px(4.0F), u32(c::t1));
                draw->AddCircle(at, px(7.0F), u32(c::accent), 0, px(1.5F));
                draw->AddText(fonts.sans, small, ImVec2(at.x + px(10.0F), at.y - small * 0.5F),
                              u32(c::t1), prims[i].name.c_str());
            }
        }

        const EnvironmentInfo& info = *context.client.environment();
        const EnvironmentInfo::Field* field = nullptr;
        for (const EnvironmentInfo::Field& candidate : info.fields) {
            if (candidate.name == view_->field) {
                field = &candidate;
            }
        }
        const char* unit = field != nullptr ? field->unit.c_str() : "";
        const char* label = field != nullptr ? field->label.c_str() : view_->field.c_str();

        // Top left: what the slice is.
        char text[160];
        std::snprintf(text, sizeof(text), "%s  ·  %u × %u cells of %.2f m  ·  z %.2f m", label,
                      slice->width, slice->height, slice->cellM, slice->heightM);
        box(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), text, c::t1);

        // Probe: the value under the cursor (design: crosshair with a tooltip).
        if (hovered) {
            const ImVec2 mouse = ImGui::GetIO().MousePos;
            const double x = left + static_cast<double>(mouse.x - base.x) / scale;
            const double y = bottom + static_cast<double>(base.y - mouse.y) / scale;
            const float value = slice->sample(x, y);
            if (!std::isnan(value)) {
                draw->AddLine(ImVec2(mouse.x - px(8.0F), mouse.y), ImVec2(mouse.x + px(8.0F), mouse.y),
                              u32(c::t1));
                draw->AddLine(ImVec2(mouse.x, mouse.y - px(8.0F)), ImVec2(mouse.x, mouse.y + px(8.0F)),
                              u32(c::t1));
                ImGui::BeginTooltip();
                widgets::Mono(c::t3, "probe · x %.2f  y %.2f  z %.2f m", x, y, slice->heightM);
                widgets::Dot(field != nullptr ? widgets::domainColor(field->domain) : c::t2);
                ImGui::SameLine(0.0F, px(6.0F));
                widgets::Mono(c::t1, "%s  %.3g %s", label, static_cast<double>(value), unit);
                ImGui::EndTooltip();
            }
        }

        // Bottom right: the legend.
        const ImVec2 legendMin(origin.x + size.x - px(250.0F), origin.y + size.y - px(74.0F));
        const ImVec2 legendMax(origin.x + size.x - px(10.0F), origin.y + size.y - px(10.0F));
        draw->AddRectFilled(legendMin, legendMax, u32(theme::hex(0x091120, 0.92F)), px(4.0F));
        draw->AddRect(legendMin, legendMax, u32(c::lineStrong), px(4.0F));
        draw->AddText(fonts.sansSemiBold, px(theme::kLabelSize),
                      ImVec2(legendMin.x + px(10.0F), legendMin.y + px(7.0F)), u32(c::t1), label);
        const float unitWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, unit).x;
        draw->AddText(fonts.mono, small, ImVec2(legendMax.x - unitWidth - px(10.0F), legendMin.y + px(8.0F)),
                      u32(c::t3), unit);
        const float barLeft = legendMin.x + px(10.0F);
        const float barRight = legendMax.x - px(10.0F);
        const float barTop = legendMin.y + px(28.0F);
        constexpr int kSteps = 48;
        for (int i = 0; i < kSteps; ++i) {
            const float a = static_cast<float>(i) / kSteps;
            draw->AddRectFilled(ImVec2(barLeft + (barRight - barLeft) * a, barTop),
                                ImVec2(barLeft + (barRight - barLeft) * (a + 1.0F / kSteps) + 0.5F,
                                       barTop + px(8.0F)),
                                u32(viridis(a)));
        }
        std::snprintf(text, sizeof(text), "%.3g", static_cast<double>(slice->valueMin));
        draw->AddText(fonts.mono, small, ImVec2(barLeft, barTop + px(12.0F)), u32(c::t3), text);
        std::snprintf(text, sizeof(text), "%.3g", static_cast<double>(slice->valueMax));
        const float maxWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
        draw->AddText(fonts.mono, small, ImVec2(barRight - maxWidth, barTop + px(12.0F)), u32(c::t3),
                      text);
        ImGui::End();
    }

private:
    static void box(ImDrawList* draw, const ImVec2& min, const char* text, const ImVec4& color) {
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kMonoSize);
        const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, text).x;
        const ImVec2 max(min.x + width + px(16.0F), min.y + px(26.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
        draw->AddText(font, size, ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(color), text);
    }

    std::shared_ptr<EnvironmentView> view_;
};

// --- which fields can be shown ------------------------------------------------------------

class OverlaysPanel final : public Panel {
public:
    explicit OverlaysPanel(std::shared_ptr<EnvironmentView> view) : view_(std::move(view)) {}
    [[nodiscard]] const char* title() const override { return "Overlays"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Environment; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (emptyState(context)) {
            ImGui::End();
            return;
        }
        view_->follow(context);
        const EnvironmentInfo& info = *context.client.environment();
        // One scalar slice at a time: choosing a field replaces the one shown.
        for (const EnvironmentInfo::Field& field : info.fields) {
            ImGui::PushID(field.name.c_str());
            bool on = view_->field == field.name;
            ImGui::BeginDisabled(!field.available || field.components != 1);
            if (widgets::Checkbox("##show", &on)) {
                view_->field = on ? field.name : std::string{};
                view_->stream.reset();
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::AlignTextToFramePadding();
            widgets::Swatch(widgets::domainColor(field.domain));
            ImGui::SameLine(0.0F, px(8.0F));
            ImGui::PushStyleColor(ImGuiCol_Text, field.available ? c::t1 : c::t3);
            ImGui::TextUnformatted(field.label.c_str());
            ImGui::PopStyleColor();
            // On the right: the unit, or why the field cannot be shown.
            const char* note = field.available ? field.unit.c_str() : field.unavailableReason.c_str();
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            const float width = ImGui::CalcTextSize(note).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(width);
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::TextColored(c::t3, "%s", note);
            ImGui::PopFont();
            ImGui::PopID();
        }
        const std::shared_ptr<const FieldSliceData> slice =
            view_->stream ? view_->stream->latest() : nullptr;
        if (slice && widgets::SectionHeader("Selected layer", nullptr)) {
            ImGui::Indent(px(8.0F));
            char text[64];
            std::snprintf(text, sizeof(text), "%u × %u", slice->width, slice->height);
            widgets::PropertyRow("grid", text, "cells");
            row("cell", "m", "%.2f", slice->cellM);
            std::snprintf(text, sizeof(text), "%.1f × %.1f", slice->cellM * slice->width,
                          slice->cellM * slice->height);
            widgets::PropertyRow("bounds", text, "m");
            row("lowest", nullptr, "%.4g", static_cast<double>(slice->valueMin));
            row("highest", nullptr, "%.4g", static_cast<double>(slice->valueMax));
            std::snprintf(text, sizeof(text), "%llu",
                          static_cast<unsigned long long>(view_->stream->slices()));
            widgets::PropertyRow("slices received", text, nullptr);
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted("opacity");
            ImGui::PopStyleColor();
            ImGui::SameLine(px(8.0F) + px(112.0F));
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::SliderFloat("##opacity", &view_->opacity, 0.1F, 1.0F, "%.2f");
            widgets::PropertyRow("colormap", "viridis", nullptr);
            ImGui::Unindent(px(8.0F));
        }
        if (info.synthetic) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("This server has no environment model. A field shown here is an "
                             "estimate it derives from the recording.");
            ImGui::PopTextWrapPos();
        }
        ImGui::End();
    }

private:
    std::shared_ptr<EnvironmentView> view_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createEnvironmentPanels() {
    auto view = std::make_shared<EnvironmentView>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<EnvironmentPanel>());
    panels.push_back(std::make_unique<FieldSlicePanel>(view));
    panels.push_back(std::make_unique<OverlaysPanel>(view));
    return panels;
}

}  // namespace lagrange::viewer
