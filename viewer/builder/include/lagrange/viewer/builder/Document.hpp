#pragma once

// The robot being built (RFC 0016 §3). The structs between the two "RFC 0016 §3" markers are
// the RFC's, copied as they are; change them only by a note in docs/plan/builder-phase1-plan.md.
//
// Units: SI everywhere, as URDF writes them. Lengths in m, angles in rad (rpy: roll about x,
// then pitch about y, then yaw about z, fixed axes, URDF's convention), mass in kg, density in
// kg/m³, inertia in kg·m², rates in Hz, times in s.
//
// Library rule (invariant 14): viewer/builder links no ImGui, no Vulkan, no simulation module.

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace lagrange::viewer::builder {

// --- RFC 0016 phase 3: the overlay file (formats.md §9), as the file says it ------------------
// One value of an attribute: its text as typed (a number, a token, a name or path), or a list.
struct OverlayAttribute {
    std::string name;                 ///< "elec:torqueConstant", "elec:port:dc_out:maxCurrent"
    std::vector<std::string> values;  ///< one item for a scalar
    bool list{false};                 ///< written as [a, b, c]
};
/// A schema instance: "ElectricalMotorAPI_1", or "PortAPI_1:dc_out" for a multiple-apply one.
struct OverlaySchema {
    std::string name;
    std::vector<OverlayAttribute> attributes;
};
/// One `apply` entry: `prim` (a link, joint or sensor by name, or a path) with an optional
/// `create_child`, or `create` (a new prim under the robot's top prim). `sensor` is a placeholder
/// instead: the overlay-only sensor of that name (Document::sensors) is written here.
struct OverlayEntry {
    std::string prim, createChild, create;
    std::vector<OverlaySchema> schemas;
    std::string sensor;
};
struct Overlay {
    std::string header;  ///< the file's leading comment lines, kept as written
    std::vector<OverlayEntry> entries;
};

// --- RFC 0016 §3 (verbatim) -------------------------------------------------------------------
// clang-format off
struct Pose { std::array<double, 3> xyz{}; std::array<double, 3> rpy{}; };  // m, rad (URDF's)
struct Shape {
    enum class Kind { Box, Cylinder, Sphere, Mesh, Capsule } kind = Kind::Box;  // Capsule: Addendum A
    std::array<double, 3> size{};   // box: x y z; cylinder: radius, length (along z); sphere: radius; m
    std::string mesh;               // Mesh: the file as written (package://, relative or absolute)
    std::array<double, 3> scale{1, 1, 1};
    Pose origin;                    // in the link frame
    bool visual = true, collision = true, solid = true;  // solid: counts for mass (§4)
    std::array<float, 4> rgba{};
};
struct Inertial {
    double density = 0;                         // kg/m³, from a material preset or typed
    std::optional<double> mass;                 // kg, override
    std::optional<std::array<double, 6>> inertia;  // ixx ixy ixz iyy iyz izz, kg·m², about the COM
    std::optional<std::array<double, 3>> com;      // m, override
};
// Addendum A: how the link collides, the user's choice in Properties (`lagrange_collision`).
enum class CollisionChoice { Unset, Mesh, Split, Shapes, Drawn };
struct Link { std::string name; std::vector<Shape> shapes; Inertial inertial; std::string keptXml;
              CollisionChoice collision = CollisionChoice::Unset; };  // collision: Addendum A
struct Joint {
    enum class Type { Fixed, Revolute, Continuous, Prismatic } type = Type::Fixed;
    std::string name, parent, child;
    Pose origin; std::array<double, 3> axis{1, 0, 0};
    std::optional<double> lower, upper, effort, velocity;  // rad or m, N·m or N, rad/s or m/s
    double damping = 0, friction = 0;
    std::optional<std::tuple<std::string, double, double>> mimic;  // joint, multiplier, offset
    std::string keptXml;
};
struct Sensor {
    std::string name, link, type;   // type: "imu", "lidar", "camera", ... or a kind
    Pose origin; double updateRate = 10;  // Hz (the URDF reader's default)
    std::optional<double> latency, jitter; // s
    std::map<std::string, std::string, std::less<>> params;  // by the catalog's path, text as typed
};
struct Document {
    std::string robot; std::vector<Link> links; std::vector<Joint> joints;
    std::vector<Sensor> sensors; std::string keptXml;  // robot-level elements kept as written
    Overlay overlay;  // RFC 0016 phase 3 (plan note): <robot>.overlay.yaml beside the URDF
};
// clang-format on
// --- end of RFC 0016 §3 -----------------------------------------------------------------------

// Notes on the fields above, for every package that reads or writes them:
// - Pose: the joint's `<origin>` (parent frame to child frame), a shape's origin in its link's
//   frame, or a sensor's mount (`<pose>`) in its link's frame.
// - Shape::size: Box {x, y, z} full edge lengths; Cylinder {radius, length, unused}, its axis
//   along the shape frame's z; Sphere {radius, unused, unused}; Capsule {radius, length of the
//   straight part (URDF's and urdfdom's `length`), unused}, along z; Mesh unused (scale applies).
// - Link::collision (RFC 0016 Addendum A): Unset (no choice made: the shapes as they are),
//   Mesh (1: the visual mesh collides, as its convex hull), Split (2: convex parts from CoACD,
//   collision-only mesh shapes), Shapes (3: collision-only template shapes, Fit to mesh), Drawn
//   (4: collision-only shapes placed in the Builder view). Written as `lagrange_collision`.
// - Shape::rgba: linear 0..1; all zero means "no colour given" (the writer writes none).
// - Shape::solid: counts for mass (RFC §4, open question 12): by default true for collision
//   shapes; a visual-only shape is massless.
// - Inertial: an `<inertial>` read from a file is kept as overrides (mass, com, inertia) until
//   the user asks to compute it (RFC §10).
// - Joint::axis: in the joint frame; normalised by the writer, never zero (a Problem).
// - Joint limits: required for Revolute and Prismatic (no invented defaults; empty is a
//   Problem); for Continuous only effort and velocity are meaningful.
// - Joint::mimic: {leader joint name, multiplier (1), offset (rad or m)}.
// - keptXml: elements this builder does not model, verbatim, written back in place (RFC §6).
// - Sensor::type: a built-in URDF type ("imu", "lidar", "camera", ...) or a kind name
//   ("rangefinder", ...), as the catalog names it (SensorCatalog::find).
// - Sensor::params: key = the catalog parameter's path ("range/max"), value = the text as
//   typed, in the catalog's unit (SI); never converted on the way through.

/// The name of a root link of a new document, and its first shape (SPEC state 1, "Empty").
inline constexpr std::string_view kNewRootLink = "base_link";
/// Box 0.3 × 0.2 × 0.1 m (SPEC state 1: "so there is something to click").
inline constexpr std::array<double, 3> kNewRootBox{0.3, 0.2, 0.1};

/// A new document (SPEC state 1): robot `robot`, one link `base_link` with one box shape of
/// kNewRootBox, visual, collision and solid, no joints, no sensors.
[[nodiscard]] Document newDocument(std::string robot = "robot");

/// Finders by name; nullptr when there is none. Names are unique in a valid document; with
/// duplicates (a Problem) the first in document order is returned.
[[nodiscard]] const Link* findLink(const Document& document, std::string_view name);
[[nodiscard]] const Joint* findJoint(const Document& document, std::string_view name);
[[nodiscard]] const Sensor* findSensor(const Document& document, std::string_view name);
[[nodiscard]] Link* findLink(Document& document, std::string_view name);
[[nodiscard]] Joint* findJoint(Document& document, std::string_view name);
[[nodiscard]] Sensor* findSensor(Document& document, std::string_view name);

/// "fixed", "revolute", "continuous", "prismatic" (URDF's words); and back (nullopt otherwise).
[[nodiscard]] std::string_view toString(Joint::Type type);
[[nodiscard]] std::optional<Joint::Type> parseJointType(std::string_view text);
/// "box", "cylinder", "sphere", "mesh", "capsule" (URDF's element names).
[[nodiscard]] std::string_view toString(Shape::Kind kind);
/// "", "mesh", "split", "shapes", "drawn" (the `lagrange_collision` words); and back.
[[nodiscard]] std::string_view toString(CollisionChoice choice);
[[nodiscard]] std::optional<CollisionChoice> parseCollisionChoice(std::string_view text);

/// True when the joint moves (revolute, continuous or prismatic): it gets a preview slider.
[[nodiscard]] bool isMoving(Joint::Type type);

}  // namespace lagrange::viewer::builder
