// Scene workspace: the Layers tab beside the scene tree and the History tab beside the
// Inspector (docs/design/ui/02-scene/SPEC.md "Dock layout").
//
// Both show only what the protocol carries (invariant 14). The protocol has no layer stack and
// no undo: Layers lists the files the run is composed of as the scene and the server's log name
// them (stage path, environment layer, robots spawned or added, the session layer once saved),
// and History lists the changes the server's log reports, read-only.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/RunHistory.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <array>
#include <cfloat>
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

/// The run's history, shared by the two tabs; read once per frame.
struct SceneTabsState {
    RunHistory history;
    int frame{-1};

    void follow(const PanelContext& context) {
        if (const int now = ImGui::GetFrameCount(); now != frame) {
            frame = now;
            history.update(*context.client.logs(), context.stats.connects);
        }
    }
};

/// The disconnected and loading states (ui-conventions §4). True when one was drawn.
bool notReady(const PanelContext& context, const char* what) {
    if (context.scene) {
        return false;
    }
    ImGui::PushTextWrapPos(0.0F);
    if (context.connection.state == ConnectionState::Ready) {
        widgets::Caption("Waiting for the scene ...");
    } else {
        widgets::Caption("Not connected. %s appear when a simulation server is reachable.", what);
    }
    ImGui::PopTextWrapPos();
    return true;
}

// --- Layers -------------------------------------------------------------------------------

const char* roleText(LayerRow::Role role) {
    switch (role) {
        case LayerRow::Role::Session:
            return "session";
        case LayerRow::Role::Robot:
            return "robot";
        case LayerRow::Role::Stage:
            return "stage";
        case LayerRow::Role::Environment:
            return "environment";
    }
    return "";
}

class LayersPanel final : public Panel {
public:
    explicit LayersPanel(std::shared_ptr<SceneTabsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "Layers"; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (notReady(context, "The stage's layers")) {
            ImGui::End();
            return;
        }
        const std::shared_ptr<const EnvironmentInfo> environment = context.client.environment();
        const std::string& stagePath = context.scene->statics->stagePath;
        const std::string& environmentLayer = environment ? environment->layer : empty_;
        if (state_->history.generation() != generation_ || stagePath != stage_ ||
            environmentLayer != environment_) {
            generation_ = state_->history.generation();
            stage_ = stagePath;
            environment_ = environmentLayer;
            rows_ = layerRows(stage_, environment_, state_->history);
            selected_ = std::min(selected_, rows_.size() - 1);
        }

        widgets::SectionHeader("Layer stack", "strongest first", true);
        const float height = px(22.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        for (std::size_t i = 0; i < rows_.size(); ++i) {
            const LayerRow& layer = rows_[i];
            widgets::TreeRowSpec spec;
            spec.icon = layer.role == LayerRow::Role::Session ? widgets::Icon::Component
                                                              : widgets::Icon::Stage;
            spec.name = layer.label.c_str();
            spec.strong = layer.role == LayerRow::Role::Stage;
            spec.type = roleText(layer.role);
            spec.selected = i == selected_;
            ImGui::PushID(static_cast<int>(i));
            if (widgets::TreeRow("##layer", spec) != widgets::TreeHit::None) {
                selected_ = i;
            }
            ImGui::PopID();
            if (layer.role == LayerRow::Role::Session) {
                // Where the run's edits go: the Inspector's Apply changes the running model,
                // which the server writes as this layer when asked to save.
                const char* target = "edit target";
                ImFont* font = theme::fonts().mono;
                const float size = px(theme::kCaptionSize);
                const float width = font->CalcTextSizeA(size, FLT_MAX, 0.0F, target).x;
                const ImVec2 max = ImGui::GetItemRectMax();
                ImGui::GetWindowDrawList()->AddText(
                    font, size, ImVec2(max.x - width - px(8.0F), max.y - (height + size) * 0.5F),
                    u32(c::accent), target);
            }
        }
        ImGui::PopStyleVar();

        ImGui::Dummy(ImVec2(0.0F, px(8.0F)));
        if (selected_ < rows_.size()) {
            const LayerRow& layer = rows_[selected_];
            widgets::SectionHeader("Layer", roleText(layer.role), true);
            widgets::PropertyRow("file", layer.path.empty() ? "—" : layer.path.c_str(), nullptr,
                                 false, layer.path.empty() ? nullptr : layer.path.c_str());
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", layer.detail.c_str());
            ImGui::PopTextWrapPos();
            if (!layer.path.empty()) {
                ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
                if (widgets::Button("Copy path")) {
                    ImGui::SetClipboardText(layer.path.c_str());
                }
            }
        }

        // Footer, as the scene tree's: counts, and what this server cannot do yet.
        const float footer = ImGui::GetTextLineHeightWithSpacing() * 2.0F + px(4.0F);
        const float bottom = ImGui::GetWindowContentRegionMax().y;
        if (ImGui::GetCursorPosY() < bottom - footer) {
            ImGui::SetCursorPosY(bottom - footer);
        }
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(
            "%zu layers  ·  mute and edit target: the protocol does not carry the "
            "layer stack yet",
            rows_.size());
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    std::shared_ptr<SceneTabsState> state_;
    std::vector<LayerRow> rows_;
    std::uint64_t generation_{~std::uint64_t{0}};
    std::string stage_;
    std::string environment_;
    std::string empty_;
    std::size_t selected_{0};
};

// --- History ------------------------------------------------------------------------------

enum class Show : int { All = 0, Edits = 1, Robots = 2, Run = 3 };

bool shown(const HistoryItem& item, Show show) {
    using K = HistoryItem::Kind;
    switch (show) {
        case Show::Edits:
            return item.kind == K::Attribute || item.kind == K::SessionSaved;
        case Show::Robots:
            return item.kind == K::Spawn || item.kind == K::Delete;
        case Show::Run:
            return item.kind == K::Profile || item.kind == K::Site || item.kind == K::Reset;
        case Show::All:
            break;
    }
    return true;
}

/// The row's mark: the accent for the edits the Inspector makes, status colours for the rest
/// (never a domain colour: these are not domains).
ImVec4 markOf(HistoryItem::Kind kind) {
    switch (kind) {
        case HistoryItem::Kind::Attribute:
            return c::accent;
        case HistoryItem::Kind::SessionSaved:
            return c::ok;
        case HistoryItem::Kind::Spawn:
            return c::info;
        case HistoryItem::Kind::Delete:
            return c::warn;
        default:
            return c::t3;
    }
}

class HistoryPanel final : public Panel {
public:
    explicit HistoryPanel(std::shared_ptr<SceneTabsState> state) : state_(std::move(state)) {}
    [[nodiscard]] const char* title() const override { return "History"; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        state_->follow(context);
        if (notReady(context, "The run's edits")) {
            ImGui::End();
            return;
        }
        // Which changes: the tab bar is shared with the Inspector, so the filter is in the body.
        static constexpr std::array<const char*, 4> kShow{"All", "Edits", "Robots", "Run"};
        int index = static_cast<int>(show_);
        if (widgets::Segmented("##show", kShow.data(), 4, &index)) {
            show_ = static_cast<Show>(index);
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::vector<HistoryItem>& items = state_->history.items();
        std::size_t count = 0;
        for (const HistoryItem& item : items) {
            count += shown(item, show_) ? 1U : 0U;
        }
        const float footer = ImGui::GetTextLineHeightWithSpacing() * 2.0F + px(4.0F);
        ImGui::BeginChild("##history", ImVec2(0.0F, -footer));
        if (count == 0) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", items.empty()
                                       ? "No changes to this run yet. Edits applied from the "
                                         "Inspector, robots spawned or deleted, profiles applied "
                                         "and site values set appear here."
                                       : "No change of this kind.");
            ImGui::PopTextWrapPos();
        }
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        // Newest first.
        for (std::size_t i = items.size(); i-- > 0;) {
            if (shown(items[i], show_)) {
                row(context, items[i], i);
            }
        }
        ImGui::PopStyleVar();
        ImGui::EndChild();
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption("%zu changes  ·  read-only: the protocol has no undo", count);
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    void row(PanelContext& context, const HistoryItem& item, std::size_t index) {
        const float height = px(44.0F);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const std::uint32_t prim = primOf(context, item);
        const bool selected = prim != kNone && prim == context.selectedPrim;
        ImGui::PushID(static_cast<int>(index));
        if (ImGui::Selectable("##change", selected, ImGuiSelectableFlags_None,
                              ImVec2(width, height)) &&
            prim != kNone) {
            context.selectedPrim = prim;  // the selection is global
        }
        ImGui::PopID();

        ImDrawList* draw = ImGui::GetWindowDrawList();
        const theme::Fonts& fonts = theme::fonts();
        const float label = px(theme::kLabelSize);
        const float mono = px(theme::kMonoSize);
        const float small = px(theme::kCaptionSize);
        const float left = min.x + px(22.0F);
        draw->AddLine(ImVec2(min.x, min.y + height - 1.0F),
                      ImVec2(min.x + width, min.y + height - 1.0F), u32(c::line));
        draw->AddCircleFilled(ImVec2(min.x + px(11.0F), min.y + px(13.0F)), px(3.5F),
                              u32(markOf(item.kind)));

        // Right: when, in simulation time.
        char when[32];
        format::clock(item.simTimeNs, when, sizeof(when));
        const float whenWidth = fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, when).x;
        draw->AddText(fonts.mono, small,
                      ImVec2(min.x + width - whenWidth - px(8.0F), min.y + px(6.0F)), u32(c::t3),
                      when);
        const float room = width - (left - min.x) - whenWidth - px(16.0F);

        // First line: what changed; second line: its value and where.
        std::string first;
        std::string second;
        std::string where;
        switch (item.kind) {
            case HistoryItem::Kind::Attribute:
                first = item.name;
                second =
                    item.before + " → " + item.after + (item.unit.empty() ? "" : " ") + item.unit;
                where = item.target;
                break;
            case HistoryItem::Kind::SessionSaved:
                first = "Session layer saved";
                where = item.target;
                break;
            case HistoryItem::Kind::Spawn:
                first = "Spawned " + item.target;
                second = item.after;
                where = item.detail;
                break;
            case HistoryItem::Kind::Delete:
                first = "Deleted " + item.target;
                break;
            case HistoryItem::Kind::Profile:
                first = "Profile applied";
                second = "the run restarted at t = 0";
                break;
            case HistoryItem::Kind::Site:
                first = "Site · " + item.name;
                second = item.after.empty() ? std::string("set together") : "= " + item.after;
                break;
            case HistoryItem::Kind::Reset:
                first = "Reset";
                second = "the run restarted at t = 0";
                break;
        }
        const bool monoTitle = item.kind == HistoryItem::Kind::Attribute;
        fit::draw(draw, monoTitle ? fonts.mono : fonts.sansMedium, monoTitle ? mono : label,
                  ImVec2(left, min.y + px(5.0F)), u32(c::t1), first, room, monoTitle);
        float x = left;
        const float y = min.y + px(24.0F);
        if (!second.empty()) {
            fit::draw(draw, fonts.mono, small, ImVec2(x, y), u32(c::t2), second,
                      width - (x - min.x) - px(8.0F));
            x += fonts.mono->CalcTextSizeA(small, FLT_MAX, 0.0F, second.c_str()).x + px(10.0F);
        }
        if (!where.empty() && x < min.x + width - px(40.0F)) {
            fit::draw(draw, fonts.mono, small, ImVec2(x, y), u32(c::t3), where,
                      min.x + width - x - px(8.0F), true);
        }
    }

    /// The prim an attribute edit changed, by its path in the scene; kNone for anything else.
    static std::uint32_t primOf(const PanelContext& context, const HistoryItem& item) {
        if (item.kind != HistoryItem::Kind::Attribute || !context.scene) {
            return kNone;
        }
        const std::vector<PrimData>& prims = context.scene->statics->prims;
        for (std::uint32_t i = 0; i < prims.size(); ++i) {
            if (prims[i].path == item.target) {
                return i;
            }
        }
        return kNone;
    }

    std::shared_ptr<SceneTabsState> state_;
    Show show_{Show::All};
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createSceneTabsPanels() {
    auto state = std::make_shared<SceneTabsState>();
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<LayersPanel>(state));
    panels.push_back(std::make_unique<HistoryPanel>(state));
    return panels;
}

}  // namespace lagrange::viewer
