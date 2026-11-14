#pragma once

// The edits the Builder's panels make to a document, and the text helpers of their fields
// (RFC 0016 §3, SPEC "Panels"). Owner: WP-E. Every function here is pure: a panel copies the
// document, calls one of these, and commits the copy through History (one undo step).
//
// Library rule (invariant 14): no ImGui, no Vulkan, no simulation module.

#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

/// One row of the Robot tree (SPEC "Robot (tree)"): the robot, then links nested by their
/// joints in document order, each link's sensors after its child links. Links no joint reaches
/// from the root (a second root, a loop) and sensors on a missing link follow at depth 1, so an
/// invalid document still shows every item.
struct TreeItem {
    enum class Kind { Robot, Link, Sensor } kind{Kind::Robot};
    std::string name;   ///< link or sensor name; the robot's name for Kind::Robot
    int depth{0};       ///< 0 for the robot
    std::string joint;  ///< Link: the joint whose child it is (empty for a root)
    bool hasChildren{false};
};
[[nodiscard]] std::vector<TreeItem> treeItems(const Document& document);

/// What the Robot strip counts (SPEC "Strip under the workspace bar").
struct PartCounts {
    int links{0}, roots{0};
    int joints{0}, revolute{0}, continuous{0}, prismatic{0}, fixed{0};
    int sensors{0};
    int types{0};  ///< distinct built-in URDF types in use (camera, lidar, imu, ...)
    int kinds{0};  ///< distinct kinds in use (rangefinder, ...)
};
/// `catalog` tells types from kinds; without it (or for a name it does not know) "camera",
/// "lidar", "gpu_lidar", "ray", "gpu_ray", "imu", "depth", "thermal" count as types.
[[nodiscard]] PartCounts countParts(const Document& document, const SensorCatalog* catalog);

/// A name not used by any link, joint or sensor: `base` itself, else `base_2`, `base_3`, ...
[[nodiscard]] std::string uniqueName(const Document& document, std::string_view base);

/// Default sizes of a new shape (SPEC "Add"): box 0.1 m cube, cylinder r 0.05 m × 0.1 m,
/// sphere r 0.05 m. Small enough to sit on the 0.3 × 0.2 × 0.1 m base of a new document.
[[nodiscard]] Shape defaultShape(Shape::Kind kind);

/// The half extents of a link's shapes along z in its frame (max over shapes of origin z plus
/// half height; rotation ignored): where a new child goes so that it does not overlap.
[[nodiscard]] double topOf(const Link& link);

struct AddedLink {
    std::string link;
    std::string joint;
};
/// Adds a link with one shape of `kind` on `parent` by a new joint of `type` (SPEC "Add"): the
/// joint's origin is on top of the parent's shapes, the axis z, no limits (none are invented:
/// validation asks for them), and the link's density `density` (kg/m³; 0: none). Names are
/// `<kind>_link` and `<link>_joint`, made unique. Returns empty names when `parent` is not a
/// link of the document.
[[nodiscard]] AddedLink addLink(Document& document, std::string_view parent, Shape::Kind kind,
                                Joint::Type type, double density);

/// Adds a sensor of `type` (a built-in type or a kind) on `link` at the link's origin, named
/// after the type and made unique, with the update rate of the reader's default and no
/// parameters (every parameter at its catalog default). Returns the name; empty when `link` is
/// not a link of the document.
[[nodiscard]] std::string addSensor(Document& document, std::string_view link,
                                    std::string_view type);

/// Removes a link with everything under it (its joint, child links and their joints, and the
/// sensors on all of them). False when there is no such link.
bool removeLink(Document& document, std::string_view link);
/// Removes a joint by removing its child link (and so its subtree): a joint never exists
/// without its child in the builder. False when there is no such joint.
bool removeJoint(Document& document, std::string_view joint);
bool removeSensor(Document& document, std::string_view sensor);

/// Renames a link, joint, sensor or the robot (kind as in TreeItem plus joints) and every
/// reference to it (joint parent and child, sensor link, mimic leader). Returns an error
/// message, or "" when done. The new name must be validName() and not used by another item of
/// the same kind.
enum class ItemKind { Robot, Link, Joint, Sensor };
[[nodiscard]] std::string rename(Document& document, ItemKind kind, std::string_view from,
                                 std::string_view to);

/// Moves `link` (with its subtree) under `parent`: its joint keeps its type, axis and limits;
/// the joint's origin is set so that the link keeps its place relative to the root (from
/// `frames`, linkFrames() at zero preview; when `frames` does not match the document the
/// origin is kept). Returns an error message, or "" when done. Refused for the root, for a
/// parent inside the link's own subtree, and for a missing link.
[[nodiscard]] std::string reparent(Document& document, std::string_view link,
                                   std::string_view parent, const std::vector<Frame>& frames);

/// Links under `link` (by joints), not including it, in tree order.
[[nodiscard]] std::vector<std::string> subtreeOf(const Document& document, std::string_view link);

// --- field text -------------------------------------------------------------------------------

/// A number as the fields show it: up to `digits` significant digits, no trailing zeros, and
/// "0" for minus zero ("0.25", "-0.03", "1240", "1e-06").
[[nodiscard]] std::string formatNumber(double value, int digits = 6);
/// A number as typed: spaces around it are ignored, U+2212 (the design's minus) is a minus, and
/// "1,5" is not a number. nullopt for anything else or for inf/nan.
[[nodiscard]] std::optional<double> parseNumber(std::string_view text);

/// Checks a parameter's text against its catalog entry (type, min, max, enum options). Returns
/// "" when it is acceptable, else one line for the field's tooltip ("at most 40 m"). An empty
/// text is acceptable: the parameter is then at its default.
[[nodiscard]] std::string checkParam(const CatalogParam& param, std::string_view text);

/// Radians to degrees and back, for the fields shown in degrees (files hold radians).
[[nodiscard]] double toDegrees(double radians);
[[nodiscard]] double toRadians(double degrees);

}  // namespace lagrange::viewer::builder
