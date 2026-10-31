#pragma once

// The importers' common middle: a format-neutral description of links and joints that build()
// turns into world tables and a scene. The SDFormat and MJCF readers each fill one.
#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace lagrange::importers::detail {

enum class GeomType : std::uint8_t {
    Box,
    Sphere,
    Capsule,
    Cylinder,
    Plane,
    Ellipsoid,
    Mesh,
    HeightField
};

/// A shape. Sizes follow ShapeType (MuJoCo's conventions): box half extents, sphere {radius},
/// capsule and cylinder {radius, half length}, plane {half x, half y} (0 for unbounded),
/// ellipsoid {radii}. A mesh is Description::meshes[mesh], scaled by `size`; a height field is
/// Description::heightFields[mesh], static and only ever a collider.
struct Geom {
    GeomType type{GeomType::Box};
    std::array<double, 3> size{};
    Pose pose;  // in the link frame
    std::uint32_t mesh{kNoIndex};
};

struct VisualDesc {
    Geom geom;
    std::string material;
    std::array<float, 4> rgba{0.70F, 0.70F, 0.72F, 1.0F};
};

struct CollisionDesc {
    Geom geom;
    double friction{1.0};  // Coulomb coefficient, sliding
};

struct SensorDesc {
    std::string name;
    SensorType type{SensorType::Imu};
    Pose pose;  // in the link frame
    double rateHz{10.0};
    std::map<std::string, double, std::less<>> settings;  // as World::Sensors::settings
    std::string kind;  // a custom sensor's (RFC 0008 §3), as World::Sensors::kind
    std::map<std::string, std::string, std::less<>> text;  // its text parameters
};

struct LinkDesc {
    std::string name;
    Pose pose;                    // in the world, with every joint at zero
    double mass{0.0};             // kg; 0 for scenery
    std::array<double, 3> com{};  // m, in the link frame
    // kg·m², about the centre of mass in the link axes: xx, yy, zz, xy, xz, yz
    std::array<double, 6> inertia{};
    std::vector<VisualDesc> visuals;
    std::vector<CollisionDesc> collisions;
    std::vector<SensorDesc> sensors;
};

struct JointDesc {
    std::string name;
    JointType type{JointType::Fixed};
    std::string parent;  // a link, or empty for the world
    std::string child;
    Pose anchor;                                // the joint frame in the child link's frame
    std::array<double, 3> axis{1.0, 0.0, 0.0};  // in the joint frame
    double lower{0.0}, upper{0.0};              // rad or m; equal for unlimited
    double effort{0.0};                         // N·m or N; 0 for unlimited
    double velocity{0.0};                       // rad/s or m/s; 0 for unlimited
    bool actuated{false};
};

/// One model: a top prim with its links. A link that is no joint's child is welded to the world;
/// a floating base has an explicit Free joint from the world. A link may be the child of several
/// joints (MJCF), applied in order from the parent.
struct Description {
    std::string name;
    std::vector<LinkDesc> links;
    std::vector<JointDesc> joints;
    std::vector<TriMesh> meshes;
    std::vector<HeightField> heightFields;
};

/// Adds a description to a model: one body per link, its frame at the anchor of the link's joint
/// (where World joints act), with colliders, visuals and sensors moved into that frame; one
/// massless helper body per extra joint of a link; planes on static links become ground planes
/// on the world. Names already in the model get the description's name in front. What cannot be
/// represented is listed in Model::notes; a broken tree is an error.
[[nodiscard]] Expected<void, std::string> build(const Description& description, Model& model);

/// Whether a description has a plane that build() puts on the world (a ground of its own).
[[nodiscard]] bool hasGroundPlane(const Description& description);

/// Adds `text` to the model's notes once.
void note(Model& model, std::string text);

}  // namespace lagrange::importers::detail
