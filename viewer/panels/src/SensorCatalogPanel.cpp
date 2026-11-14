// The Builder's Sensor catalog (docs/design/ui/12-builder/SPEC.md "Sensor catalog"). The data
// is the catalog file (RFC 0016 §5) as SensorCatalog::load returned it, read at start; nothing
// comes from the server.
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cstdio>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using builder::CatalogEntry;
using widgets::px;

/// "range_and_contact" → "Range and contact"; the built-in types' group is "Built-in types".
std::string groupTitle(const CatalogEntry& entry) {
    if (entry.builtinType) {
        return "Built-in types";
    }
    std::string title = entry.group.empty() ? "Other" : entry.group;
    std::ranges::replace(title, '_', ' ');
    title[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(title[0])));
    return title;
}

bool matches(const CatalogEntry& entry, std::string_view filter) {
    const auto has = [&](std::string_view text) {
        return std::ranges::search(text, filter, [](char a, char b) {
                   return std::tolower(static_cast<unsigned char>(a)) ==
                          std::tolower(static_cast<unsigned char>(b));
               }).begin() != text.end();
    };
    return filter.empty() || has(entry.name) || has(entry.summary);
}

class CatalogPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return title_.c_str(); }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        BuilderSession& session = *context.builder;
        const auto entries = session.catalog.entries();
        title_ = "Sensor catalog · " + std::to_string(entries.size()) + "###BuilderCatalog";
        if (!ImGui::Begin(title_.c_str())) {
            ImGui::End();
            return;
        }
        std::size_t overlay = 0;
        for (const CatalogEntry& entry : entries) {
            overlay += entry.overlayOnly ? 1 : 0;
        }
        // Search, and All | URDF | Overlay · N.
        char overlayLabel[24];
        std::snprintf(overlayLabel, sizeof(overlayLabel), "Overlay · %zu", overlay);
        const char* scopes[] = {"All", "URDF", overlayLabel};
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        const float segWidth = ImGui::CalcTextSize("AllURDF").x +
                               ImGui::CalcTextSize(overlayLabel).x + px(16.0F) * 3.0F;
        ImGui::PopFont();
        (void)widgets::SearchField("##filter", "Filter sensors", filter_.data(), filter_.size(),
                                   ImGui::GetContentRegionAvail().x - segWidth - px(6.0F));
        ImGui::SameLine(0.0F, px(6.0F));
        (void)widgets::Segmented("##scope", scopes, 3, &scope_);

        ImGui::BeginChild("##entries");
        if (!session.catalogError.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Pill("catalog not read", widgets::PillKind::Error);
            ImGui::PushTextWrapPos(0.0F);
            widgets::Mono(c::t2, "%s", session.catalogError.c_str());
            ImGui::PopTextWrapPos();
        } else if (entries.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "No sensor catalog is installed on this machine "
                "(share/lagrange_viewer/catalog): sensors take a free key and value "
                "form below.");
            ImGui::PopTextWrapPos();
        }
        const std::string target = targetLink(session);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        std::string group;
        bool open = true;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const CatalogEntry& entry = entries[i];
            if ((scope_ == 1 && entry.overlayOnly) || (scope_ == 2 && !entry.overlayOnly) ||
                !matches(entry, filter_.data())) {
                continue;
            }
            const std::string title = groupTitle(entry);
            if (title != group) {
                group = title;
                std::size_t count = 0;
                for (const CatalogEntry& other : entries) {
                    count += groupTitle(other) == title ? 1 : 0;
                }
                const std::string meta = std::to_string(count);
                open = widgets::SectionHeader(title.c_str(), meta.c_str());
            }
            if (open) {
                row(session, entry, target);
            }
        }
        ImGui::PopStyleVar();
        otherKind(session, target);
        ImGui::EndChild();
        ImGui::End();
    }

private:
    void row(BuilderSession& session, const CatalogEntry& entry, const std::string& target) {
        // Design (.cat .tr): kind name in mono (118 px), the summary in t3 with an ellipsis,
        // and on the right "in use N" or, on hover, Add.
        const theme::Fonts& fonts = theme::fonts();
        const float height = px(22.0F);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        ImGui::PushID(entry.name.c_str());
        ImGui::SetNextItemAllowOverlap();  // the Add button sits on the row
        ImGui::InvisibleButton("##row", ImVec2(width, height));
        const bool hovered =
            ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) &&
            ImGui::IsMouseHoveringRect(min, ImVec2(min.x + width, min.y + height));
        if (editable(session) && ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload(kDragCatalog, entry.name.c_str(), entry.name.size() + 1);
            widgets::Caption("a %s sensor", entry.name.c_str());
            ImGui::EndDragDropSource();
        }
        if (!entry.doc.empty() || !entry.summary.empty()) {
            ImGui::SetItemTooltip("%s\n%s", entry.summary.c_str(), entry.doc.c_str());
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        if (hovered) {
            draw->AddRectFilled(min, ImVec2(min.x + width, min.y + height),
                                ImGui::ColorConvertFloat4ToU32(c::bg3), px(2.0F));
        }
        const float midY = min.y + height * 0.5F;
        const float mono = px(theme::kMonoSize);
        const float x = min.x + px(8.0F);
        draw->PushClipRect(min, ImVec2(x + px(118.0F) - px(4.0F), min.y + height), true);
        draw->AddText(fonts.mono, mono, ImVec2(x, midY - mono * 0.5F - px(1.0F)),
                      ImGui::ColorConvertFloat4ToU32(c::t1), entry.name.c_str());
        draw->PopClipRect();

        long inUse =
            std::ranges::count_if(session.document().sensors, [&](const builder::Sensor& s) {
                return s.type == entry.name ||
                       std::ranges::find(entry.urdfTypes, s.type) != entry.urdfTypes.end();
            });
        float right = min.x + width - px(6.0F);
        bool add = false;
        if (hovered && editable(session) && !target.empty()) {
            const float button = widgets::ButtonWidth("Add");
            ImGui::SetCursorScreenPos(ImVec2(right - button, min.y));
            add = widgets::Button("Add");
            ImGui::SetItemTooltip("Add a %s on %s", entry.name.c_str(), target.c_str());
            right -= button + px(6.0F);
        } else if (inUse > 0) {
            char pill[24];
            std::snprintf(pill, sizeof(pill), "in use %ld", inUse);
            const float pillWidth = widgets::PillWidth(pill);
            ImGui::SetCursorScreenPos(ImVec2(right - pillWidth, midY - px(9.0F)));
            widgets::Pill(pill, widgets::PillKind::Neutral);
            right -= pillWidth + px(6.0F);
        }
        const float caption = px(theme::kCaptionSize);
        const float summaryX = x + px(118.0F);
        draw->PushClipRect(ImVec2(summaryX, min.y),
                           ImVec2(std::max(right, summaryX), min.y + height), true);
        draw->AddText(fonts.sans, caption, ImVec2(summaryX, midY - caption * 0.5F - px(1.0F)),
                      ImGui::ColorConvertFloat4ToU32(c::t3), entry.summary.c_str());
        draw->PopClipRect();
        ImGui::SetCursorScreenPos(ImVec2(min.x, min.y + height));
        ImGui::Dummy(ImVec2(0.0F, 0.0F));
        ImGui::PopID();
        if (add) {
            addSensorOn(session, target, entry.name);
        }
    }

    /// "Other kind…": a sensor of a kind the catalog does not list (a plugin's own); its
    /// parameters are a free key and value form in Properties.
    void otherKind(BuilderSession& session, const std::string& target) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        if (!widgets::SectionHeader("Other kind…", "free form",
                                    session.catalog.entries().empty())) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        const float button = widgets::ButtonWidth("Add");
        (void)widgets::CommitField("##kind", &otherKind_, nullptr, false,
                                   ImGui::GetContentRegionAvail().x - button - px(6.0F), false);
        ImGui::SetItemTooltip("The sensor's type or kind as the URDF names it, e.g. tof");
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(!editable(session) || target.empty() || otherKind_.empty() ||
                             !builder::validName(otherKind_));
        if (widgets::Button("Add")) {
            addSensorOn(session, target, otherKind_);
        }
        ImGui::EndDisabled();
    }

    std::string title_ = "Sensor catalog###BuilderCatalog";
    std::array<char, 96> filter_{};
    int scope_{0};
    std::string otherKind_;
};

}  // namespace

std::unique_ptr<Panel> createCatalogPanel() {
    return std::make_unique<CatalogPanel>();
}

}  // namespace lagrange::viewer::builder_ui
