// The Builder's Robot tree and Files tab (docs/design/ui/12-builder/SPEC.md "Robot (tree)").
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Files.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
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
#include <unistd.h>
#include <utility>

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

        if (std::exchange(session.renameRequested, false) && editable(session)) {
            startRename(session);
        }
        const float footer = px(26.0F);
        ImGui::BeginChild("##tree", ImVec2(0.0F, -footer));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        rows(session);
        ImGui::PopStyleVar();
        ImGui::EndChild();
        if (!renaming_.empty() && !renameShown_) {
            renaming_.clear();  // its row is gone (undone, deleted): nothing to rename
        }

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
            if (ImGui::MenuItem("Mesh…", nullptr, false, !target.empty())) {
                // The Add panel's Mesh… (file dialog or path), on the same target.
                session.meshPopupRequested = true;
                ImGui::SetWindowFocus(builder_windows::kAdd);
            }
            ImGui::SetItemTooltip("A mesh link (STL, OBJ, DAE, glTF): Add → Mesh…");
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Sensor…", nullptr, false, !target.empty())) {
            // The catalog, with the selected link as the target (SPEC "Robot (tree)").
            ImGui::SetWindowFocus(builder_windows::kCatalog);
        }
        // Joint between… (SPEC "Robot (tree)": re-parenting): the target link and its subtree
        // move under the link picked, its joint kept, as a drag onto that row does.
        const Document& document = session.document();
        const bool movable =
            !target.empty() && std::ranges::any_of(document.joints, [&](const builder::Joint& j) {
                return j.child == target;
            });
        if (ImGui::BeginMenu("Joint between…", movable)) {
            widgets::Caption("%s under", target.c_str());
            const std::vector<std::string> below = builder::subtreeOf(document, target);
            const auto parent = std::ranges::find(document.joints, target, &builder::Joint::child);
            for (const builder::Link& link : document.links) {
                if (link.name == target || std::ranges::find(below, link.name) != below.end()) {
                    continue;  // never under itself
                }
                const bool current = parent != document.joints.end() && parent->parent == link.name;
                if (ImGui::MenuItem(link.name.c_str(), nullptr, current, !current)) {
                    moveUnder(session, target, link.name);
                    break;
                }
            }
            ImGui::EndMenu();
        }
        if (!movable) {
            ImGui::SetItemTooltip("Select a link that hangs from another to move it");
        }
        ImGui::EndPopup();
    }

    /// F2 (SPEC "Robot (tree)"): the selection's row becomes its name field. Its row is made
    /// visible first: the filter cleared, the links above it unfolded.
    void startRename(const BuilderSession& session) {
        const auto& selection = session.selection;
        const Document& document = session.document();
        std::string link;  // the row that shows it
        switch (selection.kind) {
            case Kind::Robot:
                renameKind_ = builder::ItemKind::Robot;
                renaming_ = document.robot;
                break;
            case Kind::Link:
                renameKind_ = builder::ItemKind::Link;
                renaming_ = link = selection.name;
                break;
            case Kind::Joint: {
                renameKind_ = builder::ItemKind::Joint;
                renaming_ = selection.name;
                const builder::Joint* joint = builder::findJoint(document, selection.name);
                link = joint != nullptr ? joint->child : std::string{};
                break;
            }
            case Kind::Sensor: {
                renameKind_ = builder::ItemKind::Sensor;
                renaming_ = selection.name;
                const builder::Sensor* sensor = builder::findSensor(document, selection.name);
                link = sensor != nullptr ? sensor->link : std::string{};
                break;
            }
            case Kind::None:
                renaming_.clear();
                return;
        }
        renameText_ = renaming_;
        renameFocus_ = true;
        renameWaited_ = 0;
        filter_[0] = '\0';
        robotFolded_ = false;
        if (selection.kind != Kind::Link) {
            folded_.erase(link);  // a sensor's or joint's row is under (or is) that link
        }
        // Unfold every link above it.
        for (int guard = 0; !link.empty() && guard < 10000; ++guard) {
            const auto joint = std::ranges::find(document.joints, link, &builder::Joint::child);
            if (joint == document.joints.end()) {
                break;
            }
            link = joint->parent;
            folded_.erase(link);
        }
    }

    /// The row being renamed: its name as a field at the row's indent, focused when it starts;
    /// Enter, Tab or a click elsewhere renames (one undo step), Esc leaves it as it was.
    void renameRow(BuilderSession& session, int depth) {
        renameShown_ = true;
        const float indent = px(4.0F) + px(14.0F) * static_cast<float>(depth) + px(18.0F);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + indent);
        if (renameFocus_) {
            ImGui::SetKeyboardFocusHere();
        }
        std::string typed = renameText_;
        const char* what = renameKind_ == builder::ItemKind::Joint    ? "joint"
                           : renameKind_ == builder::ItemKind::Sensor ? "sensor"
                           : renameKind_ == builder::ItemKind::Robot  ? "robot"
                                                                      : "link";
        const bool committed = widgets::CommitField("##rename", &typed, what, true,
                                                    ImGui::GetContentRegionAvail().x, false);
        ImGui::SetItemTooltip("Rename: Enter, Esc to leave it; references follow the new name");
        const bool active = ImGui::IsItemActive();
        if (renameFocus_) {
            renameFocus_ = !active;  // until the field has taken the keyboard
            if (renameFocus_ && ++renameWaited_ > 3) {
                renaming_.clear();  // it never took it (a popup has the keyboard)
            }
            return;
        }
        if (committed) {
            const std::string from = renaming_;
            const std::string error = renameItem(session, renameKind_, from, typed);
            if (!error.empty()) {
                session.problems.builder.insert(
                    session.problems.builder.begin(),
                    builder::Problem{builder::Level::Warning, from, "not renamed: " + error});
            }
        }
        if (!active) {
            renaming_.clear();
        }
    }

    [[nodiscard]] bool isRenamed(const builder::TreeItem& item) const {
        if (renaming_.empty()) {
            return false;
        }
        switch (renameKind_) {
            case builder::ItemKind::Robot:
                return item.kind == builder::TreeItem::Kind::Robot;
            case builder::ItemKind::Link:
                return item.kind == builder::TreeItem::Kind::Link && item.name == renaming_;
            case builder::ItemKind::Joint:
                return item.kind == builder::TreeItem::Kind::Link && item.joint == renaming_;
            case builder::ItemKind::Sensor:
                return item.kind == builder::TreeItem::Kind::Sensor && item.name == renaming_;
        }
        return false;
    }

    void rows(BuilderSession& session) {
        renameShown_ = false;
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
            if (isRenamed(item)) {
                ImGui::PushID(static_cast<int>(item.kind));
                ImGui::PushID(item.name.c_str());
                renameRow(session, std::min(item.depth, 12));
                ImGui::PopID();
                ImGui::PopID();
                continue;
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
        // QA F17: the indent stops where the name would have no room left; deeper rows say
        // their depth instead, so a long chain stays readable.
        const float step = px(14.0F);
        const int deepest =
            std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x - px(170.0F)) / step));
        if (item.depth > deepest) {
            spec.depth = deepest;
            // Counted from the root link (the robot's row is above it).
            type = "depth " + std::to_string(item.depth - 1) + " · " + type;
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

    /// Moves `link` (and its subtree) under `parent`, keeping its place: one step, or a
    /// warning in Problems saying why not.
    static void moveUnder(BuilderSession& session, const std::string& link,
                          const std::string& parent) {
        Document next = session.document();
        const builder::Tree tree = builder::buildTree(next);
        const std::string error = builder::reparent(
            next, link, parent, builder::linkFrames(next, tree, builder::Preview{}));
        if (error.empty()) {
            commit(session, std::move(next), "Move " + link + " to " + parent);
        } else {
            session.problems.builder.insert(
                session.problems.builder.begin(),
                builder::Problem{builder::Level::Warning, link, "not moved: " + error});
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
            moveUnder(session, static_cast<const char*>(payload->Data), item.name);
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
                // A robot keeps a link (QA F7): the root of everything is not deleted, and
                // says why, instead of leaving a robot that Add cannot build on.
                done = builder::removeLink(next, selection.name);
                if (done && next.links.empty()) {
                    done = false;
                    const bool alone = session.document().links.size() == 1;
                    session.problems.files.push_back(
                        {builder::Level::Warning, selection.name,
                         alone ? "Delete: " + selection.name +
                                     " is the robot's only link, and a robot keeps one"
                               : "Delete: " + selection.name +
                                     " is the root, and every other link hangs from it; delete "
                                     "its children first, or start a New robot"});
                    ImGui::SetWindowFocus(builder_windows::kProblems);
                }
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
    // F2: the item whose row is its name field (empty: none), and that field's state.
    std::string renaming_;
    builder::ItemKind renameKind_{builder::ItemKind::Link};
    std::string renameText_;
    bool renameFocus_{false};
    bool renameShown_{false};
    int renameWaited_{0};  ///< frames the field has waited for the keyboard
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
        // RFC 0016 phase 3: the power system and GNSS, magnetometer, barometer and radar go
        // into <robot>.overlay.yaml beside the URDF (formats.md §9).
        if (builder::hasOverlay(document)) {
            const std::size_t entries = std::ranges::count_if(
                document.overlay.entries,
                [](const builder::OverlayEntry& e) { return e.sensor.empty(); });
            const std::size_t sensors = std::ranges::count_if(
                document.sensors,
                [](const builder::Sensor& s) { return builder::isOverlaySensor(s.type); });
            char note[64];
            std::snprintf(note, sizeof(note), "%zu entries · %zu sensors", entries, sensors);
            entry("overlay YAML",
                  session.path.empty() ? std::string{}
                                       : builder::overlayPathFor(session.path).string(),
                  session.path.empty() ? "not saved yet" : note);
            ImGui::SetItemTooltip("Written with the URDF; Test runs the server with --overlay");
        } else {
            entry("overlay YAML", {}, "not needed");
            ImGui::SetItemTooltip(
                "Needed for a battery, a bus, motors, or GNSS, magnetometer, barometer, radar");
        }
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
        // Code review L6: one per document file and viewer (Files.hpp).
        const std::filesystem::path autosave = builder::autosaveFile(
            builder::autosaveFolder(), document.robot, session.path, ::getpid());
        std::error_code ec;
        entry("autosave", autosave.string(),
              std::filesystem::exists(autosave, ec) ? nullptr : "none yet");
        ImGui::End();
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
