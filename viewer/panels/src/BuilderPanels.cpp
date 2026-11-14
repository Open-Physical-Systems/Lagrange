// The Builder workspace's panels (RFC 0016, docs/design/ui/12-builder/SPEC.md).
//
// WP0 lands the layout with a placeholder in every window, so the workspace opens (with or
// without a server, R1) and each later package replaces one placeholder line in
// createBuilderPanels() with its own panel. The window IDs are builder_windows' (R5).
#include "BuilderCommon.hpp"
#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Materials.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

/// A window of the Builder layout whose panel is not built yet: what it will hold, and which
/// work package of docs/plan/builder-phase1-plan.md builds it.
class PlaceholderPanel final : public Panel {
public:
    PlaceholderPanel(const char* id, const char* what, const char* package)
        : id_(id), what_(what), package_(package) {}
    [[nodiscard]] const char* title() const override { return id_; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }
    void draw(PanelContext& /*context*/) override {
        if (ImGui::Begin(id_)) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", what_);
            ImGui::PopTextWrapPos();
            ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
            widgets::Mono(c::t3, "not built yet · RFC 0016 %s", package_);
        }
        ImGui::End();
    }

private:
    const char* id_;
    const char* what_;
    const char* package_;
};

std::unique_ptr<Panel> placeholder(const char* id, const char* what, const char* package) {
    return std::make_unique<PlaceholderPanel>(id, what, package);
}

}  // namespace

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
    return panels;
}

}  // namespace lagrange::viewer
