#pragma once

// The Builder view's gizmos (docs/design/ui/12-builder/SPEC.md "Builder view"; RFC 0016
// Addendum A option 4): Move arrows and plane handles, Rotate rings, size handles on a shape's
// faces, the Joint axis tool and Measure. Drawn with the ImGui draw list over the view image,
// as the Viewport's overlays are (plan WP-D: no new pipeline, no new shader). The geometry is
// builder::gizmo (viewer/builder, with its gtests); this is the mouse and the drawing.
//
// Every drag is one undo step: the first change commits, later ones replace that step (undo,
// then commit again), so a pause in the drag never splits it. Esc puts the document back.
// Internal to lagrange_viewer_panels.

#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/builder/Gizmo.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

/// A text box over the Builder view (design .ov): raised surface, thin border, mono text;
/// `second` on the same line in t2. Returns its lower right corner.
ImVec2 viewOverlayBox(ImDrawList* draw, const ImVec2& min, const char* first,
                      const char* second = nullptr, const ImVec4* border = nullptr,
                      const ImVec4* text = nullptr);

class BuilderGizmo {
public:
    /// Where the view is on screen and how it looks at the robot.
    struct View {
        const Camera* camera{nullptr};
        ImVec2 origin;
        ImVec2 size;
        [[nodiscard]] builder::gizmo::Line ray(const ImVec2& mouse) const;
        /// Where `point` lands on screen; false only when it is behind the camera.
        bool toScreen(const builder::gizmo::Vec3& point, ImVec2& out) const;
        /// Metres per pixel at `point`: handles keep their size on screen.
        [[nodiscard]] double metresPerPixel(const builder::gizmo::Vec3& point) const;
    };

    /// Works out the handles of the session's tool and selection, and the one under the mouse
    /// (when `mouseInView`). Call once per frame before handle().
    void prepare(const BuilderSession& session, const std::vector<builder::Frame>& links,
                 const View& view, bool mouseInView);

    /// The mouse on the view: `pressed` when the left button went down on it this frame. True
    /// while the gizmo owns the mouse (a drag, or the press that started one): the view then
    /// neither orbits nor selects.
    bool handle(BuilderSession& session, const View& view, bool pressed);

    /// A click in the view that was not a drag, with the view's own pick (`pickedSensor`: the
    /// sensor prim hit first, or empty; `pickedLink`: the link of the prim hit first, or empty).
    /// Selects, drills into a shape, or adds a Measure point.
    void click(BuilderSession& session, const std::vector<builder::Frame>& links,
               const builder::gizmo::Line& ray, const std::string& pickedSensor,
               const std::string& pickedLink);

    /// Esc: cancels a drag, else clears Measure's points, else leaves the shape for its link.
    /// True when it did something.
    bool escape(BuilderSession& session);

    void draw(ImDrawList* draw, const BuilderSession& session, const View& view) const;

    [[nodiscard]] bool dragging() const { return drag_.kind != HandleKind::None; }
    /// The pose the Move and Rotate tools act on now (None when nothing).
    [[nodiscard]] const builder::gizmo::Target& target() const { return target_; }

private:
    enum class HandleKind : std::uint8_t { None, Axis, Plane, Free, Ring, Size, JointAxis };
    struct Handle {
        HandleKind kind{HandleKind::None};
        int axis{0};
        int sign{1};
        int shape{-1};                ///< Size: index into the link's shapes
        builder::gizmo::Vec3 at{};    ///< the handle's point (Axis: the gizmo's centre)
        builder::gizmo::Vec3 dir{};   ///< Axis, Size: the drag direction; Ring, Plane: normal
        std::vector<ImVec2> outline;  ///< on screen: a polyline (open), or a polygon (filled)
        bool filled{false};
    };
    struct Drag {
        HandleKind kind{HandleKind::None};
        Handle handle;
        builder::gizmo::Target target;
        builder::Document start;
        builder::gizmo::Line ray0;
        builder::Frame frame;              ///< the target's parent frame at the start
        builder::gizmo::Vec3 reference{};  ///< Ring: the angle's zero direction
        double t0{0.0};
        double angle{0.0};      ///< Ring: the angle turned so far (unwrapped), rad
        double lastAngle{0.0};  ///< Ring: the ring angle seen last frame
        bool committed{false};
        std::uint64_t revision{0};  ///< history's revision after our last commit
        std::string readout;
        std::string readoutValue;
    };

    void startDrag(BuilderSession& session, const Handle& handle, const View& view);
    void updateDrag(BuilderSession& session, const View& view);
    void apply(BuilderSession& session, builder::Document next, const std::string& label);

    builder::gizmo::Target target_;
    builder::gizmo::Target shape_;  ///< a shape drilled into by a second click (Kind::Shape)
    std::string jointForAxis_;      ///< the joint the Joint axis tool acts on
    builder::Frame frame_;          ///< target_'s parent frame
    builder::gizmo::Vec3 centre_{};
    double scale_{0.1};  ///< the gizmo's arm length (m) at its centre
    std::vector<Handle> handles_;
    int hovered_{-1};
    Drag drag_;
    bool pressOwned_{false};
    std::vector<builder::gizmo::Vec3> measure_;
};

}  // namespace lagrange::viewer::builder_ui
