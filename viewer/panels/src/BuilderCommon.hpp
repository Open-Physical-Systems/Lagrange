#pragma once

// What the Builder's panels share (RFC 0016, docs/design/ui/12-builder/SPEC.md): every edit
// goes through commit() into the session's History (one undo step, SPEC "Properties"), and the
// selection is the session's. Internal to lagrange_viewer_panels.

#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Edits.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

using builder::Document;
using Kind = BuilderSession::Selection::Kind;

/// Makes `next` the document: one undo step labelled `label`. Steps with the same `mergeKey`
/// within a second merge (History's rule; typing in one field is one step).
inline void commit(BuilderSession& session, Document next, std::string label,
                   std::string mergeKey = {}) {
    session.history.commit(std::move(next), std::move(label), std::move(mergeKey),
                           ImGui::GetTime());
}

/// Edits are allowed in Build mode only; while testing the document is read-only (SPEC 4).
[[nodiscard]] inline bool editable(const BuilderSession& session) {
    return session.mode == BuilderSession::Mode::Build;
}

inline void select(BuilderSession& session, Kind kind, std::string name) {
    session.selection.kind = kind;
    session.selection.name = std::move(name);
}

/// The link new things go on (SPEC "Add": the selected link): the selected link, a selected
/// joint's child, a selected sensor's link, else the first root link. Empty: no link at all.
[[nodiscard]] std::string targetLink(const BuilderSession& session);

/// Short words of the tree and the strip: "rev", "cont", "fixed", "prism"; "cyl", "sphere".
[[nodiscard]] const char* shortType(builder::Joint::Type type);
[[nodiscard]] const char* shortShape(builder::Shape::Kind kind);

/// The domain dots (SPEC: they say which domain the parts belong to, nothing else).
[[nodiscard]] inline ImVec4 rigidBodyColor() {
    return widgets::domainColor("rigid_body");
}
[[nodiscard]] inline ImVec4 sensorsColor() {
    return widgets::domainColor("sensors");
}

/// The builder's errors about `item`, one per line; empty when there are none.
[[nodiscard]] std::string errorsAbout(const BuilderSession& session, std::string_view item);

/// Selects whatever is named `item` (a link, joint or sensor; the robot for ""). False when
/// nothing has that name.
bool selectByName(BuilderSession& session, std::string_view item);

/// Drag and drop payload types between the Builder's panels.
inline constexpr const char* kDragLink = "LGR_BUILDER_LINK";      // link name
inline constexpr const char* kDragSensor = "LGR_BUILDER_SENSOR";  // sensor name
inline constexpr const char* kDragShape = "LGR_BUILDER_SHAPE";    // int: Shape::Kind
inline constexpr const char* kDragCatalog = "LGR_BUILDER_TYPE";   // sensor type or kind

/// The Add panel's defaults for the next tile (SPEC "Add": joint and material), shared with the
/// tree's Add menu and drops onto a link.
struct AddDefaults {
    builder::Joint::Type joint{builder::Joint::Type::Fixed};
    int preset{-1};       ///< index into densityPresets(); -1: custom
    double density{0.0};  ///< kg/m³ when custom; 0: none given
};
[[nodiscard]] AddDefaults& addDefaults();
[[nodiscard]] double addDensity();

/// Adds a link of `kind` on `parent` with the Add defaults, commits it and selects it.
void addLinkOn(BuilderSession& session, const std::string& parent, builder::Shape::Kind kind);
/// Adds a sensor of `type` on `link`, commits it and selects it.
void addSensorOn(BuilderSession& session, const std::string& link, const std::string& type);

/// Deletes the selected link (with its subtree), joint (with its child) or sensor: one step.
void deleteSelection(BuilderSession& session);

/// The power-system forms (RFC 0016 phase 3, BuilderPower.cpp): the robot's batteries and
/// buses, and a joint's motor, in Properties. Each may commit; callers use no reference into the
/// document after them.
void robotPowerSections(BuilderSession& session);
void jointMotorSection(BuilderSession& session, const std::string& joint);

// The panels, one file each (BuilderTree.cpp, BuilderAdd.cpp, ...).
[[nodiscard]] std::unique_ptr<Panel> createRobotPanel();
[[nodiscard]] std::unique_ptr<Panel> createFilesPanel();
[[nodiscard]] std::unique_ptr<Panel> createAddPanel();
[[nodiscard]] std::unique_ptr<Panel> createTemplatesPanel();
[[nodiscard]] std::unique_ptr<Panel> createJointsPanel();
[[nodiscard]] std::unique_ptr<Panel> createPosesPanel();
[[nodiscard]] std::unique_ptr<Panel> createCatalogPanel();
[[nodiscard]] std::unique_ptr<Panel> createProblemsPanel();
[[nodiscard]] std::unique_ptr<Panel> createPropertiesPanel();
[[nodiscard]] std::unique_ptr<Panel> createHistoryPanel();
[[nodiscard]] std::unique_ptr<Panel> createDrivePanel();        // BuilderTest.cpp (WP-F)
[[nodiscard]] std::unique_ptr<Panel> createTestConsolePanel();  // BuilderTest.cpp (WP-F)

}  // namespace lagrange::viewer::builder_ui
