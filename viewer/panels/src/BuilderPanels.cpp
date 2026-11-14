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
    panels.push_back(ui::createDrivePanel());
    panels.push_back(ui::createTestConsolePanel());
    return panels;
}

}  // namespace lagrange::viewer
