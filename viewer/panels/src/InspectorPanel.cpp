// Scene workspace, right panel: the selected prim, its live signals, transform and schema
// attributes (docs/design/ui/02-scene/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::size_t kMaxLiveSignals = 6;
constexpr std::size_t kSparkColumns = 60;
constexpr std::int64_t kSparkWindowNs = 10'000'000'000;  // design: the last 10 s

/// Roll, pitch, yaw in degrees (rotations about X, Y, Z applied in that order) of a unit
/// quaternion given as w, x, y, z.
std::array<double, 3> eulerDegrees(const std::array<float, 4>& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double toDegrees = 180.0 / std::numbers::pi;
    return {std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * toDegrees,
            std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)) * toDegrees,
            std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * toDegrees};
}

class InspectorPanel final : public Panel {
public:
    /// `window` is the ImGui window name. The same inspector appears in several workspaces;
    /// each needs a window of its own, because a window is docked in one place.
    InspectorPanel(const char* window, const char* tools, Workspace workspace)
        : window_(window), tools_(tools), workspace_(workspace) {}

    [[nodiscard]] const char* title() const override { return window_; }
    [[nodiscard]] Workspace workspace() const override { return workspace_; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(window_)) {
            ImGui::End();
            return;
        }
        // The lock keeps the inspector on its prim while the selection moves elsewhere
        // (design: the lock in the panel header).
        const std::size_t count = context.scene ? context.scene->statics->prims.size() : 0;
        if (locked_ && (lockedPrim_ >= count ||
                        (context.scene && context.scene->statics->generation != lockedScene_))) {
            locked_ = false;  // the prim it was locked to is gone
        }
        if (widgets::BeginPanelTools(tools_, px(26.0F))) {
            ImGui::BeginDisabled(!locked_ && context.selectedPrim >= count);
            if (widgets::Button("##lock",
                                locked_ ? widgets::ButtonKind::On : widgets::ButtonKind::Ghost,
                                widgets::Icon::Lock)) {
                locked_ = !locked_;
                lockedPrim_ = context.selectedPrim;
                lockedScene_ = context.scene ? context.scene->statics->generation : 0;
            }
            ImGui::SetItemTooltip("%s", locked_ ? "Unlock: follow the selection again"
                                                : "Lock the inspector to this prim");
            ImGui::EndDisabled();
            widgets::EndPanelTools();
        }
        const std::uint32_t shown = locked_ ? lockedPrim_ : context.selectedPrim;
        const bool valid = context.scene && shown != kNone && shown < count;
        if (!valid) {
            follow(context, kNone, 0);
            widgets::Caption("%s", context.scene ? "Select a prim in the Scene panel."
                                                 : "No scene to inspect yet.");
            ImGui::End();
            return;
        }
        const SceneStatics& statics = *context.scene->statics;
        const std::uint32_t id = shown;
        const PrimData& prim = statics.prims[id];
        const Transform& transform = context.scene->transforms[id];
        follow(context, id, statics.generation);

        header(context, prim, transform);

        // Design 04-electrical: a battery's state as cards, and where it is on its curve.
        const bool battery = workspace_ == Workspace::Electrical &&
                             std::find(prim.apiSchemas.begin(), prim.apiSchemas.end(),
                                       "BatteryAPI_1") != prim.apiSchemas.end();
        if (battery) {
            batteryPanels(context);
        } else if (widgets::SectionHeader("Live", liveMeta(context))) {
            live(context);
        }
        if (widgets::SectionHeader("Transform", "region frame · m · deg")) {
            ImGui::Indent(px(8.0F));
            widgets::Vec3Field("Translate", transform.position.data());
            const std::array<double, 3> euler = eulerDegrees(transform.rotation);
            widgets::Vec3Field("Rotate", euler.data(), "%.1f");
            ImGui::Unindent(px(8.0F));
        }
        attributes(context, statics, prim);
        editsFooter(context, id);
        if (prim.mesh != kNone && widgets::SectionHeader("Geometry", "render mesh")) {
            const MeshData& mesh = statics.meshes[prim.mesh];
            char text[32];
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%zu", mesh.positions.size() / 3);
            widgets::PropertyRow("vertices", text, nullptr);
            std::snprintf(text, sizeof(text), "%zu", mesh.indices.size() / 3);
            widgets::PropertyRow("triangles", text, nullptr);
            if (prim.material != kNone) {
                widgets::PropertyRow("material", statics.materials[prim.material].name.c_str(),
                                     nullptr);
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    /// A signal of the prim on show by the end of its name; NaN when there is none or no sample.
    [[nodiscard]] double liveValue(std::string_view ending) const {
        for (const Live& entry : live_) {
            if (entry.name.ends_with(ending) && entry.buffer) {
                return entry.buffer->latest();
            }
        }
        return NAN;
    }

    /// One of the State cards: a caption, a large number, a line under it.
    static void stateCard(const ImVec2& min, const ImVec2& size, const char* caption,
                          const char* value, const char* below, float meter) {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const theme::Fonts& fonts = theme::fonts();
        const ImVec2 max(min.x + size.x, min.y + size.y);
        const auto u32 = [](const ImVec4& color) { return ImGui::ColorConvertFloat4ToU32(color); };
        draw->AddRectFilled(min, max, u32(theme::c::bg0), px(3.0F));
        draw->AddRect(min, max, u32(theme::c::line), px(3.0F));
        const float x = min.x + px(8.0F);
        draw->AddText(fonts.sans, px(theme::kCaptionSize), ImVec2(x, min.y + px(6.0F)),
                      u32(theme::c::t3), caption);
        draw->AddText(fonts.mono, px(16.0F), ImVec2(x, min.y + px(21.0F)), u32(theme::c::t1),
                      value);
        if (meter >= 0.0F) {
            const ImVec2 a(x, min.y + px(45.0F));
            const ImVec2 b(max.x - px(8.0F), min.y + px(49.0F));
            draw->AddRectFilled(a, b, u32(theme::c::bg4), px(2.0F));
            draw->AddRectFilled(a, ImVec2(a.x + (b.x - a.x) * std::min(meter, 1.0F), b.y),
                                u32(theme::c::ok), px(2.0F));
        } else {
            draw->AddText(fonts.mono, px(theme::kCaptionSize), ImVec2(x, min.y + px(43.0F)),
                          u32(theme::c::t3), below);
        }
    }

    /// The design's State cards and OCV(SoC) plot. What the run does not model, it does not show.
    void batteryPanels(PanelContext& context) {
        const std::shared_ptr<const PrimAttributes> answer = context.client.attributes();
        const auto numbers = [&answer](std::string_view name) -> std::vector<double> {
            if (answer) {
                for (const AttributeInfo& a : answer->attributes) {
                    if (a.name == name) {
                        return a.numbers;
                    }
                }
            }
            return {};
        };
        const std::vector<double> socs = numbers("elec:battery:ocvSoc");
        std::vector<double> volts = numbers("elec:battery:ocvVoltage");
        const std::vector<double> series = numbers("elec:battery:seriesCells");
        const double cells = series.empty() ? 1.0 : series[0];
        for (double& v : volts) {
            v *= cells;
        }
        const double soc = liveValue("/soc");
        const double terminal = liveValue("/terminal_voltage");
        const double amperes = liveValue("/current");
        const double watts = liveValue("/power");
        const double kelvin = liveValue("/temperature");
        double ocv = NAN;
        if (!std::isnan(soc) && socs.size() == volts.size() && !socs.empty()) {
            ocv = volts.front();
            for (std::size_t i = 1; i < socs.size(); ++i) {
                if (soc >= socs[i - 1]) {
                    const double t = (soc - socs[i - 1]) / std::max(socs[i] - socs[i - 1], 1e-9);
                    ocv = volts[i - 1] + std::min(t, 1.0) * (volts[i] - volts[i - 1]);
                }
            }
        }
        if (widgets::SectionHeader("State", "live")) {
            const ImVec2 origin = ImGui::GetCursorScreenPos();
            const float pad = px(8.0F);
            const float gap = px(6.0F);
            const float width = (ImGui::GetContentRegionAvail().x - pad * 2.0F - gap) * 0.5F;
            const float height = px(58.0F);
            char value[48];
            char below[64];
            std::snprintf(value, sizeof(value), "%.1f %%", soc * 100.0);
            stateCard(ImVec2(origin.x + pad, origin.y + pad), ImVec2(width, height),
                      "state of charge", value, "", static_cast<float>(soc));
            std::snprintf(value, sizeof(value), "%.2f V", terminal);
            std::snprintf(below, sizeof(below), "OCV %.2f · sag %.2f", ocv, ocv - terminal);
            stateCard(ImVec2(origin.x + pad + width + gap, origin.y + pad), ImVec2(width, height),
                      "terminal · OCV", value, below, -1.0F);
            std::snprintf(value, sizeof(value), "%.2f A", amperes);
            std::snprintf(below, sizeof(below), "%.0f W %s", std::abs(watts),
                          amperes < 0.0 ? "charge" : "discharge");
            stateCard(ImVec2(origin.x + pad, origin.y + pad + height + gap), ImVec2(width, height),
                      "current · power", value, below, -1.0F);
            if (std::isnan(kelvin)) {
                std::snprintf(value, sizeof(value), "-");
                std::snprintf(below, sizeof(below), "thermal domain off");
            } else {
                std::snprintf(value, sizeof(value), "%.1f °C", kelvin - 273.15);
                below[0] = '\0';
            }
            stateCard(ImVec2(origin.x + pad + width + gap, origin.y + pad + height + gap),
                      ImVec2(width, height), "cell temp", value, below, -1.0F);
            ImGui::Dummy(ImVec2(0.0F, pad * 2.0F + height * 2.0F + gap));
        }
        if (socs.size() >= 2 && socs.size() == volts.size()) {
            char meta[48];
            std::snprintf(meta, sizeof(meta), "%.0fS", cells);
            if (widgets::SectionHeader("OCV(SoC)", meta)) {
                ocvPlot(socs, volts, soc, ocv);
            }
        }
        ImGui::Indent(px(8.0F));
        if (widgets::Button("Plot all battery signals") && context.initialScopeSignals != nullptr) {
            for (const Live& entry : live_) {
                context.initialScopeSignals->push_back(entry.name);
            }
            *context.workspace = Workspace::Signals;
        }
        ImGui::Unindent(px(8.0F));
    }

    static void ocvPlot(const std::vector<double>& socs, const std::vector<double>& volts,
                        double soc, double ocv) {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const theme::Fonts& fonts = theme::fonts();
        const auto u32 = [](const ImVec4& color) { return ImGui::ColorConvertFloat4ToU32(color); };
        const float pad = px(8.0F);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 min(origin.x + pad, origin.y + pad);
        const ImVec2 max(origin.x + ImGui::GetContentRegionAvail().x - pad, min.y + px(96.0F));
        draw->AddRectFilled(min, max, u32(theme::c::bg0));
        draw->AddRect(min, max, u32(theme::c::line));
        const auto [lowest, highest] = std::minmax_element(volts.begin(), volts.end());
        const double span = std::max(*highest - *lowest, 1e-3);
        const float left = min.x + px(34.0F);
        const float right = max.x - px(8.0F);
        const float top = min.y + px(14.0F);
        const float bottom = max.y - px(16.0F);
        const auto at = [&](double x, double v) {
            return ImVec2(left + static_cast<float>(x) * (right - left),
                          bottom - static_cast<float>((v - *lowest) / span) * (bottom - top));
        };
        const float small = px(9.0F);
        char text[48];
        for (const double v : {*lowest, (*lowest + *highest) * 0.5, *highest}) {
            const ImVec2 y = at(0.0, v);
            draw->AddLine(ImVec2(left, y.y), ImVec2(right, y.y), u32(theme::c::line));
            std::snprintf(text, sizeof(text), "%.0f", v);
            draw->AddText(fonts.mono, small, ImVec2(min.x + px(4.0F), y.y - small * 0.5F),
                          u32(theme::c::t3), text);
        }
        draw->AddText(fonts.mono, small, ImVec2(left, bottom + px(3.0F)), u32(theme::c::t3), "0 %");
        draw->AddText(fonts.mono, small, ImVec2(right - px(24.0F), bottom + px(3.0F)),
                      u32(theme::c::t3), "100");
        const ImVec4 line = theme::kDomainColor[static_cast<int>(theme::Domain::Electrical)];
        for (std::size_t i = 1; i < socs.size(); ++i) {
            draw->AddLine(at(socs[i - 1], volts[i - 1]), at(socs[i], volts[i]), u32(line),
                          px(1.5F));
        }
        if (!std::isnan(soc) && !std::isnan(ocv)) {
            const ImVec2 here = at(soc, ocv);
            draw->AddCircleFilled(here, px(3.5F), u32(theme::c::accent));
            std::snprintf(text, sizeof(text), "%.1f %% · %.1f V", soc * 100.0, ocv);
            const float width = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, text).x;
            draw->AddText(fonts.mono, small,
                          ImVec2(std::min(here.x, right - width) - px(8.0F), min.y + px(3.0F)),
                          u32(theme::c::accent), text);
        }
        ImGui::Dummy(ImVec2(0.0F, pad * 2.0F + px(96.0F)));
    }

    struct Live {
        std::string name;   // full signal name
        std::string label;  // its last part, as a row label
        std::string unit;
        std::shared_ptr<ScopeBuffer> buffer;
    };

    /// Keeps the subscriptions and the attribute request in step with the selection. Signals
    /// are streamed only for the prim being inspected.
    void follow(PanelContext& context, std::uint32_t prim, std::uint64_t sceneGeneration) {
        const std::uint64_t catalog = context.catalog ? context.catalog->generation : 0;
        if (prim == prim_ && sceneGeneration == sceneGeneration_ && catalog == catalog_) {
            return;
        }
        for (const Live& entry : live_) {
            context.client.unsubscribe(entry.buffer);
        }
        live_.clear();
        if (prim != prim_) {
            edits_.clear();  // edits are local to the prim they were typed for
        }
        prim_ = prim;
        sceneGeneration_ = sceneGeneration;
        catalog_ = catalog;
        requestId_ = 0;
        if (prim == kNone) {
            return;
        }
        requestId_ = context.client.requestAttributes(prim);
        if (!context.catalog) {
            return;
        }
        for (const SignalInfo& info : context.catalog->signals) {
            if (info.prim != prim || info.components != 1 || live_.size() >= kMaxLiveSignals) {
                continue;
            }
            Live entry;
            entry.buffer = context.client.subscribe(info.name);
            if (!entry.buffer) {
                continue;
            }
            entry.name = info.name;
            entry.label = info.name.substr(info.name.rfind('/') + 1);
            std::ranges::replace(entry.label, '_', ' ');
            entry.unit = info.unit;
            live_.push_back(std::move(entry));
        }
    }

    static void header(PanelContext& context, const PrimData& prim, const Transform& transform) {
        const theme::Fonts& fonts = theme::fonts();
        // Design: a 16 px component mark in the accent before the name.
        ImGui::AlignTextToFramePadding();
        widgets::IconItem(widgets::Icon::Component, 16.0F, c::accent);
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::PushFont(fonts.sansSemiBold, theme::kTitleSize);
        ImGui::TextUnformatted(prim.name.c_str());
        ImGui::PopFont();
        ImGui::SameLine(0.0F, px(8.0F));
        ImGui::TextDisabled("%s", prim.typeName.c_str());
        const float button = widgets::ButtonWidth("Focus");
        ImGui::SameLine();
        widgets::AlignRight(button);
        if (widgets::Button("Focus", widgets::ButtonKind::Ghost)) {
            context.camera.target = transform.position;
        }
        widgets::Mono(c::t3, "%s", prim.path.c_str());
        for (std::size_t i = 0; i < prim.apiSchemas.size(); ++i) {
            if (i != 0) {
                ImGui::SameLine(0.0F, px(6.0F));
            }
            widgets::SchemaChip(prim.apiSchemas[i].c_str());
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
    }

    const char* liveMeta(const PanelContext& context) {
        char time[24];
        format::clock(context.status.simTimeNs, time, sizeof(time));
        std::snprintf(liveMeta_, sizeof(liveMeta_), "sim %s", time);
        return liveMeta_;
    }

    void live(PanelContext& context) {
        ImGui::Indent(px(8.0F));
        if (live_.empty()) {
            widgets::Caption("%s", context.catalog ? "No live signals for this prim."
                                                   : "Waiting for the signal list ...");
            ImGui::Unindent(px(8.0F));
            return;
        }
        const ImVec2 spark(px(120.0F), px(22.0F));
        for (std::size_t i = 0; i < live_.size(); ++i) {
            const Live& entry = live_[i];
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted(entry.label.c_str());
            ImGui::PopStyleColor();

            // Value and unit, right-aligned against the sparkline.
            const double value = entry.buffer->latest();
            char text[32];
            if (std::isnan(value)) {
                std::snprintf(text, sizeof(text), "-");
            } else {
                std::snprintf(text, sizeof(text), "%.4g", value);
            }
            ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
            const float width =
                ImGui::CalcTextSize(text).x + px(6.0F) + ImGui::CalcTextSize(entry.unit.c_str()).x;
            ImGui::PopFont();
            ImGui::SameLine();
            widgets::AlignRight(width + px(10.0F) + spark.x);
            widgets::Mono(c::t1, "%s", text);
            ImGui::SameLine(0.0F, px(6.0F));
            widgets::Mono(c::t3, "%s", entry.unit.c_str());
            ImGui::SameLine(0.0F, px(10.0F));

            const std::int64_t newest = entry.buffer->newestTimeNs();
            if (newest >= 0) {
                entry.buffer->decimate(newest - kSparkWindowNs, newest, columns_);
                for (std::size_t k = 0; k < kSparkColumns; ++k) {
                    low_[k] = columns_[k].min;
                    high_[k] = columns_[k].max;
                }
                widgets::Sparkline(low_.data(), high_.data(), static_cast<int>(kSparkColumns),
                                   spark, theme::kSeries[i % 8]);
            } else {
                ImGui::Dummy(spark);
            }
        }
        if (widgets::Button("Plot in Signals") && context.initialScopeSignals != nullptr) {
            for (const Live& entry : live_) {
                context.initialScopeSignals->push_back(entry.name);
            }
            *context.workspace = Workspace::Signals;
        }
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::Button("Copy signal paths", widgets::ButtonKind::Ghost)) {
            std::string paths;
            for (const Live& entry : live_) {
                paths += entry.name;
                paths += '\n';
            }
            ImGui::SetClipboardText(paths.c_str());
        }
        ImGui::Unindent(px(8.0F));
    }

    void attributes(PanelContext& context, const SceneStatics& statics, const PrimData& prim) {
        const std::shared_ptr<const PrimAttributes> answer = context.client.attributes();
        if (!answer || answer->requestId != requestId_) {
            widgets::Caption("  Loading attributes ...");
            return;
        }
        // One section per attribute namespace, titled with the schema that owns it.
        std::size_t begin = 0;
        const std::vector<AttributeInfo>& all = answer->attributes;
        while (begin < all.size()) {
            const std::string_view group = all[begin].group();
            std::size_t end = begin + 1;
            while (end < all.size() && all[end].group() == group) {
                ++end;
            }
            char title[96];
            sectionTitle(prim, group, title, sizeof(title));
            char meta[32];
            std::snprintf(meta, sizeof(meta), "%zu attributes", end - begin);
            if (widgets::SectionHeader(title, meta)) {
                ImGui::Indent(px(8.0F));
                for (std::size_t i = begin; i < end; ++i) {
                    row(context, statics, all[i]);
                }
                ImGui::Unindent(px(8.0F));
            }
            begin = end;
        }
    }

    static void sectionTitle(const PrimData& prim, std::string_view group, char* out,
                             std::size_t size) {
        const std::string_view domain = widgets::schemaDomain(group);
        for (const std::string& schema : prim.apiSchemas) {
            if (!domain.empty() && widgets::schemaDomain(schema) == domain) {
                std::snprintf(out, size, "%s", schema.c_str());
                return;
            }
        }
        std::snprintf(out, size, "%.*s", static_cast<int>(group.empty() ? 10 : group.size()),
                      group.empty() ? "attributes" : group.data());
    }

    void row(PanelContext& context, const SceneStatics& statics, const AttributeInfo& attribute) {
        const std::size_t colon = attribute.name.find(':');
        const char* label = attribute.name.c_str() + (colon == std::string::npos ? 0 : colon + 1);
        if (attribute.kind == AttributeInfo::Kind::Targets) {
            char text[160] = "-";
            std::uint32_t target = kNone;
            if (!attribute.targets.empty() && attribute.targets.front() < statics.prims.size()) {
                target = attribute.targets.front();
                std::snprintf(text, sizeof(text), "→ %s", statics.prims[target].path.c_str());
            }
            if (widgets::PropertyRow(label, text, nullptr, true, attribute.doc.c_str()) &&
                target != kNone) {
                context.selectedPrim = target;  // follow the relationship
            }
            return;
        }
        if (attribute.editable && attribute.kind == AttributeInfo::Kind::Real) {
            // Design: edits stay local until Apply; an edited field gets the accent border.
            auto it = edits_.find(attribute.name);
            std::array<char, 48> buffer{};
            const std::string& shown = it != edits_.end() ? it->second : attribute.value;
            std::snprintf(buffer.data(), buffer.size(), "%s", shown.c_str());
            const bool edited = it != edits_.end() && it->second != attribute.value;
            if (widgets::PropertyEdit(label, buffer.data(), buffer.size(), attribute.unit.c_str(),
                                      edited, attribute.doc.c_str())) {
                edits_[attribute.name] = buffer.data();
            }
            return;
        }
        widgets::PropertyRow(label, attribute.value.c_str(), attribute.unit.c_str(),
                             attribute.kind == AttributeInfo::Kind::Asset, attribute.doc.c_str());
    }

    /// Design: "N edits", Revert and Apply; Apply sends one set_attribute per edit. The server
    /// applies them to the running simulation from its next step.
    void editsFooter(PanelContext& context, std::uint32_t prim) {
        const std::shared_ptr<const PrimAttributes> answer = context.client.attributes();
        std::size_t count = 0;
        for (const auto& [name, text] : edits_) {
            for (const AttributeInfo& a : answer ? answer->attributes : kNoAttributes) {
                count += a.name == name && a.value != text ? 1 : 0;
            }
        }
        if (count == 0) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        ImGui::Separator();
        ImGui::AlignTextToFramePadding();
        widgets::Caption("%zu %s · applies to the running simulation", count,
                         count == 1 ? "edit" : "edits");
        const float revert = widgets::ButtonWidth("Revert");
        const float apply = widgets::ButtonWidth("Apply");
        ImGui::SameLine();
        widgets::AlignRight(revert + apply + px(6.0F));
        if (widgets::Button("Revert", widgets::ButtonKind::Ghost)) {
            edits_.clear();
        }
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::PrimaryButton("Apply")) {
            for (const auto& [name, text] : edits_) {
                char* end = nullptr;
                const double value = std::strtod(text.c_str(), &end);
                if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
                    continue;  // not a number: left as typed, for the user to correct
                }
                context.client.setAttribute(prim, name, value, false);
            }
            edits_.clear();
            requestId_ = context.client.requestAttributes(prim);  // the values as applied
        }
    }

    const char* window_;
    const char* tools_;
    Workspace workspace_;
    std::vector<Live> live_;
    std::map<std::string, std::string, std::less<>> edits_;  // attribute -> typed text
    static inline const std::vector<AttributeInfo> kNoAttributes;
    bool locked_{false};
    std::uint32_t lockedPrim_{kNone};
    std::uint64_t lockedScene_{0};
    std::uint32_t prim_{kNone};
    std::uint64_t sceneGeneration_{0};
    std::uint64_t catalog_{0};
    std::uint64_t requestId_{0};
    std::array<ScopeBuffer::Column, kSparkColumns> columns_{};
    std::array<float, kSparkColumns> low_{};
    std::array<float, kSparkColumns> high_{};
    char liveMeta_[40]{};
};

}  // namespace

std::unique_ptr<Panel> createInspectorPanel() {
    return std::make_unique<InspectorPanel>("Inspector", "##inspector-tools", Workspace::Scene);
}

std::unique_ptr<Panel> createElectricalInspectorPanel() {
    return std::make_unique<InspectorPanel>("Inspector###InspectorElectrical",
                                            "##inspector-tools-electrical", Workspace::Electrical);
}

}  // namespace lagrange::viewer
