// The Builder workspace's panels (RFC 0016, docs/design/ui/12-builder/SPEC.md).
//
// The workspace opens with or without a server (R1); createBuilderPanels() makes one panel per
// window of the layout. The window IDs are builder_windows' (R5).
#include "BuilderCommon.hpp"
#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Materials.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

std::unique_ptr<Panel> createBuilderViewPanel();  // BuilderView.cpp (WP-D)

BuilderSession::BuilderSession() : history(builder::newDocument("robot"), "New robot") {}

int BuilderSession::Problems::errors() const {
    int count = 0;
    for (const std::vector<builder::Problem>* list : {&builder, &loader, &check}) {
        count += static_cast<int>(std::ranges::count_if(
            *list, [](const builder::Problem& p) { return p.level == builder::Level::Error; }));
    }
    return count;
}

namespace builder_ui {

std::string targetLink(const BuilderSession& session) {
    const Document& document = session.document();
    const auto& selection = session.selection;
    if (selection.kind == Kind::Link && builder::findLink(document, selection.name) != nullptr) {
        return selection.name;
    }
    if (selection.kind == Kind::Joint) {
        if (const builder::Joint* joint = builder::findJoint(document, selection.name)) {
            if (builder::findLink(document, joint->child) != nullptr) {
                return joint->child;
            }
        }
    }
    if (selection.kind == Kind::Sensor) {
        if (const builder::Sensor* sensor = builder::findSensor(document, selection.name)) {
            if (builder::findLink(document, sensor->link) != nullptr) {
                return sensor->link;
            }
        }
    }
    for (const builder::TreeItem& item : builder::treeItems(document)) {
        if (item.kind == builder::TreeItem::Kind::Link) {
            return item.name;  // the first root
        }
    }
    return {};
}

const char* shortType(builder::Joint::Type type) {
    switch (type) {
        case builder::Joint::Type::Revolute:
            return "rev";
        case builder::Joint::Type::Continuous:
            return "cont";
        case builder::Joint::Type::Prismatic:
            return "prism";
        case builder::Joint::Type::Fixed:
            return "fixed";
    }
    return "fixed";
}

const char* shortShape(builder::Shape::Kind kind) {
    switch (kind) {
        case builder::Shape::Kind::Box:
            return "box";
        case builder::Shape::Kind::Cylinder:
            return "cyl";
        case builder::Shape::Kind::Sphere:
            return "sphere";
        case builder::Shape::Kind::Mesh:
            return "mesh";
        case builder::Shape::Kind::Capsule:
            return "capsule";
    }
    return "box";
}

std::string errorsAbout(const BuilderSession& session, std::string_view item) {
    std::string text;
    for (const std::vector<builder::Problem>* list :
         {&session.problems.builder, &session.problems.loader, &session.problems.check}) {
        for (const builder::Problem& problem : *list) {
            if (problem.level == builder::Level::Error && problem.item == item) {
                if (!text.empty()) {
                    text += '\n';
                }
                text += problem.message;
            }
        }
    }
    return text;
}

bool selectByName(BuilderSession& session, std::string_view item) {
    const Document& document = session.document();
    if (item.empty()) {
        select(session, Kind::Robot, {});
        return true;
    }
    if (builder::findLink(document, item) != nullptr) {
        select(session, Kind::Link, std::string(item));
    } else if (builder::findJoint(document, item) != nullptr) {
        select(session, Kind::Joint, std::string(item));
    } else if (builder::findSensor(document, item) != nullptr) {
        select(session, Kind::Sensor, std::string(item));
    } else if (std::ranges::any_of(document.overlay.entries, [&](const builder::OverlayEntry& e) {
                   return builder::entryPrim(e) == item;
               })) {
        // A battery or bus of the overlay (RFC 0016 phase 3): its form is the robot's Power.
        select(session, Kind::Robot, {});
    } else {
        return false;
    }
    return true;
}

AddDefaults& addDefaults() {
    static AddDefaults defaults;
    return defaults;
}

double addDensity() {
    const AddDefaults& defaults = addDefaults();
    const auto presets = builder::densityPresets();
    if (defaults.preset >= 0 && static_cast<std::size_t>(defaults.preset) < presets.size()) {
        return presets[static_cast<std::size_t>(defaults.preset)].density;
    }
    return defaults.density;
}

void addLinkOn(BuilderSession& session, const std::string& parent, builder::Shape::Kind kind) {
    Document next = session.document();
    const builder::AddedLink added =
        builder::addLink(next, parent, kind, addDefaults().joint, addDensity());
    if (added.link.empty()) {
        return;
    }
    commit(session, std::move(next), "Add link " + added.link);
    select(session, Kind::Link, added.link);
}

void addSensorOn(BuilderSession& session, const std::string& link, const std::string& type) {
    Document next = session.document();
    const std::string name = builder::addSensor(next, link, type);
    if (name.empty()) {
        return;
    }
    commit(session, std::move(next), "Add sensor " + name + " on " + link);
    select(session, Kind::Sensor, name);
}

void drawPrompt(BuilderSession& session) {
    namespace c = theme::c;
    using Prompt = BuilderSession::Prompt;
    using widgets::px;
    Prompt& prompt = session.prompt;
    constexpr const char* kPopup = "##builder-prompt";
    if (prompt.kind == Prompt::Kind::None || prompt.choice != Prompt::Choice::None) {
        return;
    }
    if (!ImGui::IsPopupOpen(kPopup)) {
        ImGui::OpenPopup(kPopup);
    }
    // A sheet as the command palette draws one (bg1, a lineStrong border), 440 px wide, in the
    // middle of the window, modal: nothing else takes a click or a key until answered.
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float width = std::min(px(440.0F), viewport->WorkSize.x - px(32.0F));
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                                   viewport->WorkPos.y + viewport->WorkSize.y * 0.4F),
                            ImGuiCond_Always, ImVec2(0.5F, 0.5F));
    ImGui::SetNextWindowSize(ImVec2(width, 0.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(16.0F), px(14.0F)));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, c::bg1);
    ImGui::PushStyleColor(ImGuiCol_Border, c::lineStrong);
    const bool shown =
        ImGui::BeginPopupModal(kPopup, nullptr,
                               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings);
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar();
    if (!shown) {
        return;
    }
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushFont(fonts.sansSemiBold, theme::kTitleSize);
    ImGui::TextUnformatted(prompt.title.c_str());
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    ImGui::PushTextWrapPos(0.0F);
    ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
    ImGui::TextUnformatted(prompt.message.c_str());
    ImGui::PopStyleColor();
    ImGui::PopTextWrapPos();
    ImGui::Dummy(ImVec2(0.0F, px(10.0F)));

    // Buttons on the right, the accent one last: Cancel · Discard · Save.
    const float gap = px(8.0F);
    float row = widgets::ButtonWidth(prompt.primary.c_str()) +
                widgets::ButtonWidth(prompt.cancel.c_str()) + gap;
    if (!prompt.secondary.empty()) {
        row += widgets::ButtonWidth(prompt.secondary.c_str()) + gap;
    }
    widgets::AlignRight(row);
    Prompt::Choice choice = Prompt::Choice::None;
    if (widgets::Button(prompt.cancel.c_str(), widgets::ButtonKind::Ghost) ||
        (ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !ImGui::GetIO().WantTextInput)) {
        choice = Prompt::Choice::Cancel;
    }
    if (!prompt.secondary.empty()) {
        ImGui::SameLine(0.0F, gap);
        if (widgets::Button(prompt.secondary.c_str())) {
            choice = Prompt::Choice::Secondary;
        }
    }
    ImGui::SameLine(0.0F, gap);
    ImGui::BeginDisabled(!prompt.primaryEnabled);
    if (widgets::PrimaryButton(prompt.primary.c_str())) {
        choice = Prompt::Choice::Primary;
    }
    ImGui::EndDisabled();
    if (!prompt.primaryEnabled && !prompt.primaryHint.empty()) {
        ImGui::SetItemTooltip("%s", prompt.primaryHint.c_str());
    }
    if (choice != Prompt::Choice::None) {
        prompt.choice = choice;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

}  // namespace builder_ui

std::vector<std::unique_ptr<Panel>> createBuilderPanels() {
    namespace w = builder_windows;
    namespace ui = builder_ui;
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(ui::createRobotPanel());
    panels.push_back(ui::createFilesPanel());
    panels.push_back(ui::createAddPanel());
    panels.push_back(ui::createTemplatesPanel());
    panels.push_back(createBuilderViewPanel());
    panels.push_back(ui::createJointsPanel());
    panels.push_back(ui::createPosesPanel());
    panels.push_back(ui::createCatalogPanel());
    panels.push_back(ui::createProblemsPanel());
    panels.push_back(ui::createPropertiesPanel());
    panels.push_back(ui::createHistoryPanel());
    panels.push_back(ui::createDrivePanel());
    panels.push_back(ui::createTestConsolePanel());
    return panels;
}

}  // namespace lagrange::viewer
