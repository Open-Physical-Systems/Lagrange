// The Builder's Robot tree and Files tab (docs/design/ui/12-builder/SPEC.md "Robot (tree)").
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <set>
#include <string>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using widgets::px;

bool contains(std::string_view text, std::string_view needle) {
    if (needle.empty()) {
        return true;
    }
    const auto lower = [](char ch) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    };
    return std::ranges::search(text, needle, [&](char a, char b) {
               return lower(a) == lower(b);
           }).begin() != text.end();
}

class RobotPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kRobot; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        const Document& document = session.document();
        // Design (.ph .tools): Collapse all.
        if (widgets::BeginPanelTools("##robot-tools", px(22.0F))) {
            if (widgets::Button("##collapse", widgets::ButtonKind::Ghost,
                                widgets::Icon::ChevronUp)) {
                for (const builder::Link& link : document.links) {
                    folded_.insert(link.name);
                }
            }
            ImGui::SetItemTooltip("Collapse all");
            widgets::EndPanelTools();
        }

        // Search and Add (design: .bar).
        const float addWidth = widgets::ButtonWidth("Add", widgets::Icon::Plus);
        (void)widgets::SearchField("##filter", "Filter links, joints, sensors", filter_.data(),
                                   filter_.size(),
                                   ImGui::GetContentRegionAvail().x - addWidth - px(6.0F));
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::BeginDisabled(!editable(session));
        if (widgets::Button("Add", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-menu");
        }
        ImGui::EndDisabled();
        addMenu(session);
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));

        const float footer = px(26.0F);
        ImGui::BeginChild("##tree", ImVec2(0.0F, -footer));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        rows(session);
        ImGui::PopStyleVar();
        ImGui::EndChild();

        // Footer (design: a 26 px bar with a rule on top).
        const ImVec2 at = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(at.x - ImGui::GetStyle().WindowPadding.x, at.y),
            ImVec2(at.x + ImGui::GetContentRegionAvail().x + ImGui::GetStyle().WindowPadding.x,
                   at.y),
            ImGui::ColorConvertFloat4ToU32(c::line));
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::Caption("%zu links · %zu joints · %zu sensors · %d selected",
                         document.links.size(), document.joints.size(), document.sensors.size(),
                         session.selection.kind == Kind::None ? 0 : 1);
        ImGui::End();
    }

private:
    void addMenu(BuilderSession& session) {
        if (!ImGui::BeginPopup("##add-menu")) {
            return;
        }
        const std::string target = targetLink(session);
        widgets::Caption("on %s", target.empty() ? "no link" : target.c_str());
        if (ImGui::BeginMenu("Link")) {
            if (ImGui::MenuItem("Box", nullptr, false, !target.empty())) {
                addLinkOn(session, target, builder::Shape::Kind::Box);
            }
            if (ImGui::MenuItem("Cylinder", nullptr, false, !target.empty())) {
                addLinkOn(session, target, builder::Shape::Kind::Cylinder);
            }
            if (ImGui::MenuItem("Sphere", nullptr, false, !target.empty())) {
                addLinkOn(session, target, builder::Shape::Kind::Sphere);
            }
            ImGui::MenuItem("Mesh…", nullptr, false, false);
            ImGui::SetItemTooltip("Mesh links come with RFC 0016 phase 2");
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Sensor…", nullptr, false, !target.empty())) {
            // The catalog, with the selected link as the target (SPEC "Robot (tree)").
            ImGui::SetWindowFocus(builder_windows::kCatalog);
        }
        ImGui::MenuItem("Joint between…", nullptr, false, false);
        ImGui::SetItemTooltip("Drag a link's row onto another link to move it there");
        ImGui::EndPopup();
    }

    void rows(BuilderSession& session) {
        const Document& document = session.document();
        const std::vector<builder::TreeItem> items = builder::treeItems(document);
        const std::string_view filter(filter_.data());
        const ImVec4 linkBadge = rigidBodyColor();
        const ImVec4 sensorBadge = sensorsColor();
        int hiddenBelow = -1;  // depth of a folded row: rows deeper than it are hidden
        for (std::size_t i = 0; i < items.size(); ++i) {
            const builder::TreeItem& item = items[i];
            if (filter.empty() && hiddenBelow >= 0) {
                if (item.depth > hiddenBelow) {
                    continue;
                }
                hiddenBelow = -1;
            }
            if (!filter.empty() && item.kind != builder::TreeItem::Kind::Robot &&
                !contains(item.name, filter) && !contains(item.joint, filter)) {
                continue;
            }
            const bool folded = item.kind == builder::TreeItem::Kind::Robot
                                    ? robotFolded_
                                    : folded_.contains(item.name);
            if (folded && filter.empty()) {
                hiddenBelow = item.depth;
            }
            row(session, item, folded, linkBadge, sensorBadge);
        }
    }

    void row(BuilderSession& session, const builder::TreeItem& item, bool folded,
             const ImVec4& linkBadge, const ImVec4& sensorBadge) {
        const Document& document = session.document();
        widgets::TreeRowSpec spec;
        spec.depth = item.depth;
        spec.hasChildren = item.hasChildren;
        spec.folded = folded;
        spec.name = item.name.c_str();
        std::string type;
        Kind kind = Kind::Robot;
        switch (item.kind) {
            case builder::TreeItem::Kind::Robot:
                spec.icon = widgets::Icon::Scene;
                spec.strong = true;
                type = "robot";
                kind = Kind::Robot;
                break;
            case builder::TreeItem::Kind::Link: {
                kind = Kind::Link;
                const builder::Link* link = builder::findLink(document, item.name);
                const builder::Joint* joint =
                    item.joint.empty() ? nullptr : builder::findJoint(document, item.joint);
                if (joint != nullptr) {
                    type = std::string(builder::toString(joint->type)) + " · ";
                }
                if (link == nullptr || link->shapes.empty()) {
                    type += "no shape";
                } else {
                    type += shortShape(link->shapes.front().kind);
                    if (link->shapes.size() > 1) {
                        type += " +" + std::to_string(link->shapes.size() - 1);
                    }
                }
                if (joint == nullptr && link != nullptr) {
                    // The root shows its mass (design: "box · 8.00 kg").
                    if (const auto mass = builder::linkMass(*link)) {
                        char text[32];
                        std::snprintf(text, sizeof(text), " · %.2f kg", mass->mass);
                        type += text;
                    }
                }
                spec.badges = &linkBadge;
                spec.badgeCount = 1;
                break;
            }
            case builder::TreeItem::Kind::Sensor: {
                kind = Kind::Sensor;
                spec.icon = widgets::Icon::Sensors;
                const builder::Sensor* sensor = builder::findSensor(document, item.name);
                type = sensor != nullptr ? sensor->type : "";
                spec.badges = &sensorBadge;
                spec.badgeCount = 1;
                break;
            }
        }
        spec.type = type.c_str();
        const auto& selection = session.selection;
        spec.selected =
            selection.kind == kind && (kind == Kind::Robot || selection.name == item.name);
        if (kind == Kind::Link && selection.kind == Kind::Joint && selection.name == item.joint &&
            !item.joint.empty()) {
            spec.selected = true;  // the joint is chosen through its child's row
        }
        const std::string errors =
            errorsAbout(session, kind == Kind::Robot ? std::string_view{} : item.name);
        const std::string jointErrors = item.joint.empty() ? "" : errorsAbout(session, item.joint);
        spec.error = !errors.empty() || !jointErrors.empty();
        spec.strong = spec.strong || spec.selected;

        ImGui::PushID(static_cast<int>(item.kind));
        const widgets::TreeHit hit = widgets::TreeRow(item.name.c_str(), spec);
        ImGui::PopID();
        if (spec.error) {
            ImGui::SetItemTooltip("%s%s%s", errors.c_str(),
                                  !errors.empty() && !jointErrors.empty() ? "\n" : "",
                                  jointErrors.c_str());
        }
        dragAndDrop(session, item);
        switch (hit) {
            case widgets::TreeHit::Caret:
                if (kind == Kind::Robot) {
                    robotFolded_ = !robotFolded_;
                } else if (!folded_.erase(item.name)) {
                    folded_.insert(item.name);
                }
                break;
            case widgets::TreeHit::Type:
                if (!item.joint.empty()) {
                    select(session, Kind::Joint, item.joint);  // the joint by its type text
                    break;
                }
                [[fallthrough]];
            case widgets::TreeHit::Row:
                select(session, kind, kind == Kind::Robot ? std::string{} : item.name);
                break;
            case widgets::TreeHit::None:
                break;
        }
    }

    /// Links and sensors drag onto links (re-parent, move a sensor); tiles and catalog rows
    /// drop onto links too (SPEC "Add", "Sensor catalog").
    void dragAndDrop(BuilderSession& session, const builder::TreeItem& item) {
        if (!editable(session) || item.kind == builder::TreeItem::Kind::Robot) {
            return;
        }
        const bool isLink = item.kind == builder::TreeItem::Kind::Link;
        if (ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload(isLink ? kDragLink : kDragSensor, item.name.c_str(),
                                      item.name.size() + 1);
            widgets::Caption("%s %s", isLink ? "move link" : "move sensor", item.name.c_str());
            ImGui::EndDragDropSource();
        }
        if (!isLink || !ImGui::BeginDragDropTarget()) {
            return;
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kDragLink)) {
            const std::string link(static_cast<const char*>(payload->Data));
            Document next = session.document();
            const builder::Tree tree = builder::buildTree(next);
            const std::string error = builder::reparent(
                next, link, item.name, builder::linkFrames(next, tree, builder::Preview{}));
            if (error.empty()) {
                commit(session, std::move(next), "Move " + link + " to " + item.name);
            } else {
                session.problems.builder.insert(
                    session.problems.builder.begin(),
                    builder::Problem{builder::Level::Warning, link, "not moved: " + error});
            }
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kDragSensor)) {
            const std::string sensor(static_cast<const char*>(payload->Data));
            Document next = session.document();
            if (builder::Sensor* moved = builder::findSensor(next, sensor);
                moved != nullptr && moved->link != item.name) {
                moved->link = item.name;
                commit(session, std::move(next), "Move " + sensor + " to " + item.name);
            }
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kDragShape)) {
            int kind = 0;
            std::memcpy(&kind, payload->Data, sizeof(kind));
            addLinkOn(session, item.name, static_cast<builder::Shape::Kind>(kind));
        }
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kDragCatalog)) {
            addSensorOn(session, item.name, static_cast<const char*>(payload->Data));
        }
        ImGui::EndDragDropTarget();
    }

public:
    static void deleteSelection(BuilderSession& session) {
        Document next = session.document();
        const auto& selection = session.selection;
        bool done = false;
        switch (selection.kind) {
            case Kind::Link:
                // The root goes only when it is the last link: a robot keeps a link.
                done = next.links.size() > 1 && builder::removeLink(next, selection.name);
                break;
            case Kind::Joint:
                done = builder::removeJoint(next, selection.name);
                break;
            case Kind::Sensor:
                done = builder::removeSensor(next, selection.name);
                break;
            case Kind::None:
            case Kind::Robot:
                break;
        }
        if (done) {
            commit(session, std::move(next), "Delete " + selection.name);
            select(session, Kind::None, {});
        }
    }

private:
    std::array<char, 96> filter_{};
    std::set<std::string, std::less<>> folded_;
    bool robotFolded_{false};
};

class FilesPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kFiles; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const BuilderSession& session = *context.builder;
        const Document& document = session.document();
        // One entry per file: what it is, then its path in mono (SPEC "Files tab").
        const auto entry = [](const char* what, const std::string& path, const char* note) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Caption("%s", what);
            if (note != nullptr) {
                ImGui::SameLine();
                widgets::Pill(note, widgets::PillKind::Neutral);
            }
            ImGui::PushTextWrapPos(0.0F);
            widgets::Mono(path.empty() ? c::t3 : c::t1, "%s", path.empty() ? "—" : path.c_str());
            ImGui::PopTextWrapPos();
        };
        // SPEC state 6: a file that could not be read or written; the whole message is in
        // Problems.
        if (!session.fileError.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Pill("file error", widgets::PillKind::Error);
            ImGui::SameLine();
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", session.fileError.c_str());
            ImGui::PopTextWrapPos();
        }
        if (!session.busy.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Pill(session.busy.c_str(), widgets::PillKind::Info);
        }
        entry("URDF", session.path.string(), session.path.empty() ? "not saved yet" : nullptr);
        // RFC 0016 phase 3: GNSS, magnetometer, barometer and radar need the overlay file.
        entry("overlay YAML", {}, "not needed");
        ImGui::SetItemTooltip("Sensors that need the overlay file come in RFC 0016 phase 3");
        bool meshes = false;
        for (const builder::Link& link : document.links) {
            for (const builder::Shape& shape : link.shapes) {
                if (shape.kind == builder::Shape::Kind::Mesh) {
                    entry(meshes ? "" : "meshes", shape.mesh, nullptr);
                    meshes = true;
                }
            }
        }
        if (!meshes) {
            entry("meshes", {}, "none");
        }
        const std::filesystem::path autosave = autosavePath(document.robot);
        std::error_code ec;
        entry("autosave", autosave.string(),
              std::filesystem::exists(autosave, ec) ? nullptr : "none yet");
        ImGui::End();
    }

private:
    /// RFC 0016 §10 (plan WP-F): $XDG_STATE_HOME/lagrange/builder/<robot>.autosave.urdf.
    static std::filesystem::path autosavePath(const std::string& robot) {
        const char* state = std::getenv("XDG_STATE_HOME");
        const char* home = std::getenv("HOME");
        std::filesystem::path base =
            state != nullptr && *state != '\0'
                ? std::filesystem::path(state)
                : std::filesystem::path(home != nullptr ? home : "") / ".local/state";
        return base / "lagrange" / "builder" / (robot + ".autosave.urdf");
    }
};

}  // namespace

std::unique_ptr<Panel> createRobotPanel() {
    return std::make_unique<RobotPanel>();
}

std::unique_ptr<Panel> createFilesPanel() {
    return std::make_unique<FilesPanel>();
}

void deleteSelection(BuilderSession& session) {
    RobotPanel::deleteSelection(session);
}

}  // namespace lagrange::viewer::builder_ui
