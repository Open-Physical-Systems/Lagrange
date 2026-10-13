// Scene workspace, left panel: the prim tree with a filter and per-prim domain badges
// (docs/design/ui/02-scene/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdio>
#include <cstring>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr float kRowHeight = 22.0F;  // design tokens: treeRowHeight
constexpr float kIndent = 14.0F;

// Same order as theme::Domain, so that a bit index is a colour index.
constexpr std::array<std::string_view, 8> kDomains{"rigid_body",   "electrical", "thermal",
                                                   "aerodynamics", "weather",    "emi",
                                                   "radiation",    "sensors"};

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

bool containsNoCase(std::string_view text, std::string_view needle) {
    const auto it =
        std::search(text.begin(), text.end(), needle.begin(), needle.end(), [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a)) ==
                   std::tolower(static_cast<unsigned char>(b));
        });
    return it != text.end();
}

class ScenePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Scene"; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        if (!context.scene) {
            widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                       ? "Waiting for the scene ..."
                                       : "Not connected. The scene appears when a simulation "
                                         "server is reachable.");
            ImGui::End();
            return;
        }
        const SceneStatics& statics = *context.scene->statics;
        if (statics.generation != generation_) {
            index(statics);
        }
        toolbar();
        if (dirty_) {
            flatten(statics);
        }

        const float footer = ImGui::GetTextLineHeightWithSpacing() + px(2.0F);
        ImGui::BeginChild("##tree", ImVec2(0.0F, -footer));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        if (rows_.empty()) {
            widgets::Caption("No prim matches the filter.");
        }
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(rows_.size()), px(kRowHeight));
        while (clipper.Step()) {
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                row(statics, rows_[static_cast<std::size_t>(i)], context.selectedPrim);
            }
        }
        ImGui::PopStyleVar();
        ImGui::EndChild();

        widgets::Caption("%zu prims  ·  %d selected  ·  %zu with physics schemas",
                         statics.prims.size(), context.selectedPrim < statics.prims.size() ? 1 : 0,
                         withSchemas_);
        ImGui::End();
    }

private:
    struct Row {
        std::uint32_t prim;
        std::uint16_t depth;
    };

    /// Per-snapshot data: child lists and which domains each prim has schemas of.
    void index(const SceneStatics& statics) {
        const std::size_t count = statics.prims.size();
        children_.assign(count, {});
        badges_.assign(count, 0);
        collapsed_.assign(count, false);
        roots_.clear();
        withSchemas_ = 0;
        for (std::uint32_t i = 0; i < count; ++i) {
            const PrimData& prim = statics.prims[i];
            if (prim.parent == kNone) {
                roots_.push_back(i);
            } else {
                children_[prim.parent].push_back(i);
            }
            for (const std::string& schema : prim.apiSchemas) {
                const std::string_view domain = widgets::schemaDomain(schema);
                for (std::size_t bit = 0; bit < kDomains.size(); ++bit) {
                    if (kDomains[bit] == domain) {
                        badges_[i] = static_cast<std::uint8_t>(badges_[i] | (1U << bit));
                    }
                }
            }
            withSchemas_ += badges_[i] != 0 ? 1U : 0U;
        }
        generation_ = statics.generation;
        dirty_ = true;
    }

    void toolbar() {
        const float button =
            ImGui::CalcTextSize("Schemas").x + ImGui::GetStyle().FramePadding.x * 2.0F + px(6.0F);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - button);
        if (ImGui::InputTextWithHint("##filter", "Filter prims", filter_, sizeof(filter_))) {
            dirty_ = true;
        }
        ImGui::SameLine(0.0F, px(6.0F));
        if (domainMask_ != 0) {
            ImGui::PushStyleColor(ImGuiCol_Border, c::accentBorder);  // a filter is active
        }
        const bool open = ImGui::Button("Schemas");
        if (domainMask_ != 0) {
            ImGui::PopStyleColor();
        }
        if (open) {
            ImGui::OpenPopup("##schemas");
        }
        if (ImGui::BeginPopup("##schemas")) {
            widgets::Caption("Show prims with schemas of");
            for (std::size_t bit = 0; bit < kDomains.size(); ++bit) {
                const std::string_view label = widgets::domainLabel(kDomains[bit]);
                bool on = (domainMask_ & (1U << bit)) != 0;
                widgets::Dot(theme::kDomainColor[bit]);
                ImGui::SameLine(0.0F, px(6.0F));
                char name[32];
                std::snprintf(name, sizeof(name), "%.*s", static_cast<int>(label.size()),
                              label.data());
                if (ImGui::MenuItem(name, nullptr, &on)) {
                    domainMask_ = static_cast<std::uint8_t>(domainMask_ ^ (1U << bit));
                    dirty_ = true;
                }
            }
            if (ImGui::MenuItem("All prims", nullptr, false, domainMask_ != 0)) {
                domainMask_ = 0;
                dirty_ = true;
            }
            ImGui::EndPopup();
        }
    }

    [[nodiscard]] bool matches(const SceneStatics& statics, std::uint32_t id) const {
        if (domainMask_ != 0 && (badges_[id] & domainMask_) == 0) {
            return false;
        }
        return filter_[0] == '\0' || containsNoCase(statics.prims[id].path, filter_);
    }

    /// Fills `keep_`: true for prims that match the filter or have a descendant that does.
    bool mark(const SceneStatics& statics, std::uint32_t id) {
        bool keep = matches(statics, id);
        for (const std::uint32_t child : children_[id]) {
            keep = mark(statics, child) || keep;
        }
        keep_[id] = keep;
        return keep;
    }

    void append(std::uint32_t id, std::uint16_t depth, bool filtering) {
        if (filtering && !keep_[id]) {
            return;
        }
        rows_.push_back({id, depth});
        if (collapsed_[id] && !filtering) {
            return;  // a filter shows every match, whatever was folded
        }
        for (const std::uint32_t child : children_[id]) {
            append(child, static_cast<std::uint16_t>(depth + 1), filtering);
        }
    }

    /// The visible rows, in order. Rebuilt only when the scene, the filter or a fold changed.
    void flatten(const SceneStatics& statics) {
        const bool filtering = filter_[0] != '\0' || domainMask_ != 0;
        rows_.clear();
        if (filtering) {
            keep_.assign(statics.prims.size(), false);
            for (const std::uint32_t root : roots_) {
                mark(statics, root);
            }
        }
        for (const std::uint32_t root : roots_) {
            append(root, 0, filtering);
        }
        dirty_ = false;
    }

    void row(const SceneStatics& statics, const Row& entry, std::uint32_t& selected) {
        const PrimData& prim = statics.prims[entry.prim];
        const bool isSelected = selected == entry.prim;
        const bool hasChildren = !children_[entry.prim].empty();
        const float height = px(kRowHeight);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float caretX = min.x + px(8.0F) + px(kIndent) * static_cast<float>(entry.depth);

        ImGui::PushID(static_cast<int>(entry.prim));
        if (ImGui::Selectable("##prim", isSelected, ImGuiSelectableFlags_None,
                              ImVec2(width, height))) {
            // A click on the caret folds; anywhere else on the row selects.
            if (hasChildren && ImGui::GetIO().MousePos.x < caretX + px(12.0F)) {
                collapsed_[entry.prim] = !collapsed_[entry.prim];
                dirty_ = true;
            } else {
                selected = entry.prim;
            }
        }
        ImGui::PopID();

        ImDrawList* draw = ImGui::GetWindowDrawList();
        const float midY = min.y + height * 0.5F;
        if (isSelected) {
            draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), min.y + height), u32(c::accent));
        }
        if (hasChildren) {
            const float r = px(3.5F);
            const ImVec2 at(caretX + px(4.0F), midY);
            if (collapsed_[entry.prim]) {
                draw->AddTriangleFilled(ImVec2(at.x - r * 0.6F, at.y - r),
                                        ImVec2(at.x - r * 0.6F, at.y + r),
                                        ImVec2(at.x + r * 0.9F, at.y), u32(c::t3));
            } else {
                draw->AddTriangleFilled(ImVec2(at.x - r, at.y - r * 0.6F),
                                        ImVec2(at.x + r, at.y - r * 0.6F),
                                        ImVec2(at.x, at.y + r * 0.9F), u32(c::t3));
            }
        }

        // Badges on the right: one dot per domain the prim has schemas of.
        float right = min.x + width - px(8.0F);
        const float dot = px(3.0F);
        for (std::size_t bit = kDomains.size(); bit-- > 0;) {
            if ((badges_[entry.prim] & (1U << bit)) != 0) {
                draw->AddCircleFilled(ImVec2(right - dot, midY), dot,
                                      u32(theme::kDomainColor[bit]));
                right -= dot * 2.0F + px(3.0F);
            }
        }

        const theme::Fonts& fonts = theme::fonts();
        const float size = px(theme::kBodySize);
        const float textY = midY - size * 0.5F - px(1.0F);
        const float nameX = caretX + px(14.0F);
        draw->PushClipRect(min, ImVec2(std::max(right - px(4.0F), min.x), min.y + height), true);
        const char* name = prim.name.c_str();
        draw->AddText(fonts.sans, size, ImVec2(nameX, textY), u32(prim.visible ? c::t1 : c::t3),
                      name);
        const float nameWidth = fonts.sans->CalcTextSizeA(size, FLT_MAX, 0.0F, name).x;
        const float typeSize = px(theme::kLabelSize);
        draw->AddText(fonts.sans, typeSize,
                      ImVec2(nameX + nameWidth + px(8.0F), midY - typeSize * 0.5F), u32(c::t3),
                      prim.typeName.c_str());
        draw->PopClipRect();
    }

    std::vector<std::vector<std::uint32_t>> children_;
    std::vector<std::uint32_t> roots_;
    std::vector<std::uint8_t> badges_;  // bit i: has a schema of kDomains[i]
    std::vector<bool> collapsed_;
    std::vector<bool> keep_;
    std::vector<Row> rows_;
    std::size_t withSchemas_{0};
    std::uint64_t generation_{0};
    char filter_[96]{};
    std::uint8_t domainMask_{0};
    bool dirty_{true};
};

}  // namespace

std::unique_ptr<Panel> createScenePanel() {
    return std::make_unique<ScenePanel>();
}

}  // namespace lagrange::viewer
