#pragma once

// The Builder view's gizmos as geometry (docs/design/ui/12-builder/SPEC.md "Builder view";
// RFC 0016 Addendum A option 4, shapes drawn by hand): what a Move arrow, a Rotate ring, a size
// handle or the Joint axis tool does to the document when dragged, snapped to the SPEC's steps,
// and the ray tests that find what is under the mouse. The panel (viewer/panels/src/
// BuilderView.cpp) draws the handles and turns the mouse into rays; everything here is pure.
//
// Frames: the view's world is the root link's frame (linkFrames, the root at the identity).
// A pose (Pose: a joint's origin, a shape's origin, a sensor's mount) is given in its parent
// frame: the parent link's frame for a joint origin, the link's own frame for a shape or a
// sensor. Move and Rotate act along and about that parent frame's axes, so a move along x
// changes only xyz[0] and the values stay on the snap grid.
//
// Library rule (invariant 14): no ImGui, no Vulkan, no simulation module.

#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <array>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::viewer::builder::gizmo {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w x y z

/// Snap steps of the gizmos (SPEC "Builder view": "snap (5 mm, 5° by default; Ctrl toggles)").
inline constexpr double kMoveStep = 0.005;                          // m
inline constexpr double kAngleStep = 5.0 * std::numbers::pi / 180;  // rad
/// The smallest size a drag leaves a shape (m): one snap step, so a shape never vanishes or
/// turns inside out under the mouse.
inline constexpr double kMinSize = kMoveStep;

/// A half-line in the view's world (the root link's frame); `direction` of unit length.
struct Line {
    Vec3 origin{};
    Vec3 direction{1, 0, 0};
};

/// `value` rounded to the nearest multiple of `step`; `value` itself when step <= 0 (snapping
/// off). Minus zero comes back as zero.
[[nodiscard]] double snap(double value, double step);

/// The parameter t of the point `point + t·axis` (unit `axis`) closest to `ray`: how far along
/// an arrow the mouse is. nullopt when the ray runs (nearly) along the axis.
[[nodiscard]] std::optional<double> alongAxis(const Line& ray, const Vec3& point, const Vec3& axis);

/// Where `ray` meets the plane through `point` with normal `normal`, in front of its origin.
/// nullopt when it does not (behind, or the plane seen edge on).
[[nodiscard]] std::optional<Vec3> onPlane(const Line& ray, const Vec3& point, const Vec3& normal);

/// The signed angle (rad, right hand about `axis`) from `reference` to where `ray` meets the
/// ring's plane (centre `centre`, normal `axis`); nullopt when it does not meet it.
[[nodiscard]] std::optional<double> angleOnRing(const Line& ray, const Vec3& centre,
                                                const Vec3& axis, const Vec3& reference);

/// URDF rpy (roll about x, pitch about y, yaw about z, fixed axes) of a unit quaternion; the
/// inverse of toFrame's rotation. Angles within 1e-9 rad of a multiple of kAngleStep come back
/// exactly on it, so rotating by snapped steps keeps the file's numbers clean.
[[nodiscard]] Vec3 rpyOf(const Quat& rotation);
/// `v` turned by the unit quaternion `rotation`.
[[nodiscard]] Vec3 rotate(const Quat& rotation, const Vec3& v);

// --- what a gizmo acts on ---------------------------------------------------------------------

/// The pose a Move or Rotate gizmo edits.
struct Target {
    enum class Kind { None, JointOrigin, Shape, Sensor } kind{Kind::None};
    /// JointOrigin: the joint's name; Shape: the link's name; Sensor: the sensor's name.
    std::string name;
    int shape{-1};  ///< Shape: index into Link::shapes
    friend bool operator==(const Target&, const Target&) = default;
};

/// The pose `target` names in `document`; nullptr when it does not exist (any more).
[[nodiscard]] Pose* poseOf(Document& document, const Target& target);
[[nodiscard]] const Pose* poseOf(const Document& document, const Target& target);

/// The frame `target`'s pose is given in, in the view's world: the parent link's frame for a
/// joint origin, the link's frame for a shape or a sensor. `links` are the link frames in
/// document order (previewFrames). nullopt when the target or its link is missing.
[[nodiscard]] std::optional<Frame> parentFrame(const Document& document,
                                               const std::vector<Frame>& links,
                                               const Target& target);

/// `start` moved by `delta` (m) along its parent frame's axis `axis` (0 x, 1 y, 2 z), the moved
/// coordinate snapped to a multiple of `step` (m; 0: off). The other coordinates and rpy keep
/// their values.
[[nodiscard]] Pose moved(const Pose& start, int axis, double delta, double step);

/// `start` turned by `angle` (rad) about its parent frame's axis `axis` through its own origin;
/// the angle is snapped to a multiple of `step` (rad; 0: off) first. xyz is kept.
[[nodiscard]] Pose rotated(const Pose& start, int axis, double angle, double step);

// --- size handles -------------------------------------------------------------------------------

/// The full length of `shape` along its own axis `axis` (box: its size; cylinder: twice the
/// radius across x and y, the length along z; sphere: twice the radius); nullopt for a shape
/// that has no size handles (a mesh: its scale is edited in Properties). A capsule: twice the
/// radius across x and y, tip to tip (length + 2 radius) along z.
[[nodiscard]] std::optional<double> extentOf(const Shape& shape, int axis);

/// Drags the face of `shape` on side `sign` (+1 or -1) of its axis `axis` outward by `delta`
/// (m; negative pulls it in) while the opposite face stays where it is: the size value that
/// changes (box size, cylinder radius or length, sphere radius) is snapped to a multiple of
/// `step` (m; 0: off) and kept at least kMinSize, and the shape's origin moves by half the
/// change towards the dragged face. A capsule along z changes its straight length (at least 0)
/// and keeps its radius. False (shape unchanged) when it has no size handles.
bool resize(Shape& shape, int axis, int sign, double delta, double step);

// --- joint axis -------------------------------------------------------------------------------

/// The joint axis for a drag towards `direction` (in the joint frame, any length > 0): the
/// nearest of ±x, ±y, ±z of the joint frame and of its parent link's frame (`originRotation`:
/// the joint origin's rotation, toFrame(origin).rotation), or with `free` the direction itself,
/// normalised. A zero direction gives {1, 0, 0}.
[[nodiscard]] Vec3 snapAxis(const Vec3& direction, const Quat& originRotation, bool free);

// --- picking ----------------------------------------------------------------------------------

/// The shape a ray meets first, and where (Measure's points, and picking shapes that are not
/// drawn, collision-only ones). Boxes, cylinders, spheres and capsules are tested exactly, a
/// mesh by its file's triangles at its scale (Addendum A); shapes neither visual nor collision
/// are skipped. Collision-only shapes are always drawn and so always picked; `collisionShown`
/// is kept for the callers and no longer changes the result.
struct ShapeHit {
    int link{-1};   ///< index into Document::links
    int shape{-1};  ///< index into Link::shapes
    double distance{0.0};
    Vec3 point{};
};
[[nodiscard]] std::optional<ShapeHit> pickShape(const Document& document,
                                                const std::vector<Frame>& links, const Line& ray,
                                                bool collisionShown = true);

/// The distance (pixels) from point p to the segment a–b in the view.
[[nodiscard]] float distanceToSegment(const std::array<float, 2>& p, const std::array<float, 2>& a,
                                      const std::array<float, 2>& b);

/// The headless report's view of the document's geometry (the input tests read it): per link
/// its shapes (kind, size, origin), per joint its origin and axis, per sensor its mount; JSON.
[[nodiscard]] std::string geometryJson(const Document& document);

}  // namespace lagrange::viewer::builder::gizmo
