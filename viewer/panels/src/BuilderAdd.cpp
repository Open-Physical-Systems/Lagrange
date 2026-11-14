// The Builder's Add and Templates tabs (docs/design/ui/12-builder/SPEC.md "Add").
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Materials.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <array>
#include <cstdio>
#include <string>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using widgets::px;

class AddPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kAdd; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        const std::string target = targetLink(session);
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        // Design: "a link on <base_link>, by a joint" (11 px, the link in mono t1).
        widgets::Caption("a link on");
        ImGui::SameLine(0.0F, px(4.0F));
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        ImGui::TextColored(target.empty() ? c::t3 : c::t1, "%s",
                           target.empty() ? "no link" : target.c_str());
        ImGui::PopFont();
        ImGui::SameLine(0.0F, px(2.0F));
        widgets::Caption(", by a joint");

        ImGui::BeginDisabled(!editable(session) || target.empty());
        tile(session, target, "Box", widgets::Icon::Box, builder::Shape::Kind::Box);
        ImGui::SameLine(0.0F, px(6.0F));
        tile(session, target, "Cylinder", widgets::Icon::Cylinder, builder::Shape::Kind::Cylinder);
        ImGui::SameLine(0.0F, px(6.0F));
        tile(session, target, "Sphere", widgets::Icon::Sphere, builder::Shape::Kind::Sphere);
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(true);
        (void)widgets::ShapeTile("mesh", widgets::Icon::Mesh, "Mesh…");
        ImGui::EndDisabled();
        ImGui::SetItemTooltip("Mesh links (STL, OBJ, DAE, glTF) come with RFC 0016 phase 2");

        defaults();
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(
            "Or drag a tile onto a link in the tree or the viewport. A sensor comes "
            "from the catalog below the view.");
        ImGui::PopTextWrapPos();
        ImGui::End();
    }

private:
    static void tile(BuilderSession& session, const std::string& target, const char* label,
                     widgets::Icon icon, builder::Shape::Kind kind) {
        if (widgets::ShapeTile(label, icon, label)) {
            addLinkOn(session, target, kind);
        }
        ImGui::SetItemTooltip("Add a %s link on %s", label, target.c_str());
        if (ImGui::BeginDragDropSource()) {
            const int value = static_cast<int>(kind);
            ImGui::SetDragDropPayload(kDragShape, &value, sizeof(value));
            widgets::Caption("a %s link", label);
            ImGui::EndDragDropSource();
        }
    }

    /// `joint` and `material`: the defaults for the next tile (design: a 70 px label grid).
    void defaults() {
        AddDefaults& defaults = addDefaults();
        const float label = px(70.0F);
        const auto rowLabel = [&](const char* text) {
            const float start = ImGui::GetCursorPosX();
            ImGui::AlignTextToFramePadding();
            ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
            ImGui::TextUnformatted(text);
            ImGui::PopStyleColor();
            ImGui::SameLine(start + label);
        };
        rowLabel("joint");
        if (widgets::DropdownField("##joint",
                                   std::string(builder::toString(defaults.joint)).c_str(), nullptr,
                                   ImGui::GetContentRegionAvail().x)) {
            ImGui::OpenPopup("##joint-types");
        }
        if (ImGui::BeginPopup("##joint-types")) {
            for (const builder::Joint::Type type :
                 {builder::Joint::Type::Fixed, builder::Joint::Type::Revolute,
                  builder::Joint::Type::Continuous, builder::Joint::Type::Prismatic}) {
                if (ImGui::Selectable(std::string(builder::toString(type)).c_str(),
                                      type == defaults.joint)) {
                    defaults.joint = type;
                }
            }
            ImGui::EndPopup();
        }

        rowLabel("material");
        const auto presets = builder::densityPresets();
        std::string name = "custom";
        char unit[48] = "none";
        if (defaults.preset >= 0 && static_cast<std::size_t>(defaults.preset) < presets.size()) {
            const builder::DensityPreset& preset =
                presets[static_cast<std::size_t>(defaults.preset)];
            name = std::string(preset.name);
            std::snprintf(unit, sizeof(unit), "%s kg/m³",
                          builder::formatNumber(preset.density).c_str());
        } else if (defaults.density > 0.0) {
            std::snprintf(unit, sizeof(unit), "%s kg/m³",
                          builder::formatNumber(defaults.density).c_str());
        }
        if (widgets::DropdownField("##material", name.c_str(), unit,
                                   ImGui::GetContentRegionAvail().x)) {
            ImGui::OpenPopup("##materials");
        }
        if (ImGui::BeginPopup("##materials")) {
            if (presets.empty()) {
                widgets::Caption("No density presets yet (RFC 0016 WP-A): type a density");
            }
            for (std::size_t i = 0; i < presets.size(); ++i) {
                char item[96];
                std::snprintf(item, sizeof(item), "%.*s  %s kg/m³",
                              static_cast<int>(presets[i].name.size()), presets[i].name.data(),
                              builder::formatNumber(presets[i].density).c_str());
                if (ImGui::Selectable(item, defaults.preset == static_cast<int>(i))) {
                    defaults.preset = static_cast<int>(i);
                }
                if (!presets[i].source.empty()) {
                    ImGui::SetItemTooltip("%.*s", static_cast<int>(presets[i].source.size()),
                                          presets[i].source.data());
                }
            }
            if (ImGui::Selectable("custom", defaults.preset < 0)) {
                defaults.preset = -1;
            }
            ImGui::EndPopup();
        }
        if (defaults.preset < 0) {
            // "custom" opens a number field (SPEC "Add").
            rowLabel("density");
            std::string text =
                defaults.density > 0.0 ? builder::formatNumber(defaults.density) : "";
            if (widgets::CommitField("##density", &text, "kg/m³", false,
                                     ImGui::GetContentRegionAvail().x)) {
                const auto value = builder::parseNumber(text);
                defaults.density = value && *value > 0.0 ? *value : 0.0;
            }
        }
    }
};

/// RFC 0016 open question 11: a few starting robots, each a builder document. The files are
/// WP-B's (assets/robots/templates/<name>.urdf); BuilderHost opens them (templateRequested).
struct Template {
    const char* name;
    const char* title;
    const char* summary;
};
constexpr std::array<Template, 3> kTemplates{{
    {"diff_drive", "Differential drive base", "a box base, two driven wheels, a caster"},
    {"arm_3dof", "3-joint arm", "a fixed base and three revolute joints"},
    {"pan_tilt", "Pan-tilt head", "two revolute joints carrying a camera"},
}};

class TemplatesPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kTemplates; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        ImGui::BeginDisabled(!editable(session));
        const float width = ImGui::GetContentRegionAvail().x;
        for (const Template& entry : kTemplates) {
            if (widgets::OptionCard(entry.name, entry.title, entry.summary, false,
                                    ImVec2(width, px(50.0F)))) {
                session.templateRequested = true;
                session.templateName = entry.name;
            }
            ImGui::SetItemTooltip("Open %s.urdf as a new document", entry.name);
        }
        ImGui::EndDisabled();
        ImGui::End();
    }
};

}  // namespace

std::unique_ptr<Panel> createAddPanel() {
    return std::make_unique<AddPanel>();
}

std::unique_ptr<Panel> createTemplatesPanel() {
    return std::make_unique<TemplatesPanel>();
}

}  // namespace lagrange::viewer::builder_ui
