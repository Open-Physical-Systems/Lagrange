// The Builder workspace's panels (RFC 0016, docs/design/ui/12-builder/SPEC.md).
//
// WP0 lands the layout with a placeholder in every window, so the workspace opens (with or
// without a server, R1) and each later package replaces one placeholder line in
// createBuilderPanels() with its own panel. The window IDs are builder_windows' (R5).
#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
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

std::vector<std::unique_ptr<Panel>> createBuilderPanels() {
    namespace w = builder_windows;
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(
        placeholder(w::kRobot, "The robot's links, joints and sensors as a tree.", "WP-E1"));
    panels.push_back(
        placeholder(w::kFiles, "The document's files: URDF, overlay, meshes, autosave.", "WP-E1"));
    panels.push_back(
        placeholder(w::kAdd, "Add a link by a joint: box, cylinder, sphere, mesh.", "WP-E1"));
    panels.push_back(placeholder(w::kTemplates,
                                 "Starting robots: differential drive base, 3-joint arm, pan-tilt.",
                                 "WP-E1"));
    panels.push_back(createBuilderViewPanel());
    panels.push_back(
        placeholder(w::kJoints, "Joint preview sliders (forward kinematics only).", "WP-E2"));
    panels.push_back(placeholder(w::kPoses, "Named sets of preview positions.", "WP-E2"));
    panels.push_back(placeholder(w::kCatalog,
                                 "The sensor catalog: built-in types and kinds with their "
                                 "parameters.",
                                 "WP-E2"));
    panels.push_back(placeholder(
        w::kProblems, "The builder's checks, the loader's messages and lagrange check.", "WP-E2"));
    panels.push_back(placeholder(w::kProperties,
                                 "The selected link, joint or sensor: shapes, mass, limits, "
                                 "parameters.",
                                 "WP-E2"));
    panels.push_back(placeholder(w::kHistory, "The undo stack, newest at the top.", "WP-E2"));
    return panels;
}

}  // namespace lagrange::viewer
