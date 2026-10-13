// Scene workspace, right panel: the selected prim, its live signals, transform and schema
// attributes (docs/design/ui/02-scene/SPEC.md).
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>

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
    [[nodiscard]] const char* title() const override { return "Inspector"; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
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
        if (widgets::BeginPanelTools("##inspector-tools", px(26.0F))) {
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

        if (widgets::SectionHeader("Live", liveMeta(context))) {
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

    static void row(PanelContext& context, const SceneStatics& statics,
                    const AttributeInfo& attribute) {
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
        widgets::PropertyRow(label, attribute.value.c_str(), attribute.unit.c_str(),
                             attribute.kind == AttributeInfo::Kind::Asset, attribute.doc.c_str());
    }

    std::vector<Live> live_;
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
    return std::make_unique<InspectorPanel>();
}

}  // namespace lagrange::viewer
