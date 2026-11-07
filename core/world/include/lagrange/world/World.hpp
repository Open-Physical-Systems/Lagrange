#pragma once

#include "lagrange/world/coords/Frames.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange {

struct PrimId {
    std::uint32_t index{0};
    friend constexpr bool operator==(PrimId, PrimId) = default;
};
struct BodyId {
    std::uint32_t index{0};
    friend constexpr bool operator==(BodyId, BodyId) = default;
};
struct JointId {
    std::uint32_t index{0};
    friend constexpr bool operator==(JointId, JointId) = default;
};

enum class JointType : std::uint8_t { Fixed, Revolute, Prismatic, Spherical, Free, Continuous };

inline constexpr std::uint32_t kNoHull = 0xFFFF'FFFFU;

/// The world itself as a parent body: bodies whose parent is kWorldBody hang from the world.
inline constexpr BodyId kWorldBody{0xFFFF'FFFFU};

/// A rigid transform: position in metres, orientation as a unit quaternion (w, x, y, z).
struct Pose {
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
};

/// Collision shapes. Sizes follow MuJoCo's conventions: sphere {radius}, capsule and cylinder
/// {radius, half length}, box {half extents}, plane {half extents x, y, grid spacing}. A convex
/// hull's points are in World::hulls (Colliders::hull); its size is the half extents of their
/// bounding box about the shape origin. A height field's grid is in World::heightfields (the
/// same Colliders::hull index); its size is half its extent in x and y and its largest |z|.
/// Height fields are static: on the world only.
enum class ShapeType : std::uint8_t {
    Plane,
    Sphere,
    Capsule,
    Cylinder,
    Box,
    ConvexHull,
    HeightField
};

inline constexpr std::uint32_t kNoMimic = 0xFFFF'FFFFU;

/// A convex hull collider (formats.md §2: collision geometry is convex hulls, never render
/// meshes): its vertices and its outward-wound triangles, in the shape's frame, m.
struct ConvexHull {
    std::vector<std::array<double, 3>> points;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    /// Each triangle's plane, unit outward normal and offset (n·p ≤ offset inside), as
    /// World::addHullCollider stores it: rays test these for every beam.
    std::vector<std::array<double, 4>> faces;
};

/// The face planes of a hull, each turned so that the hull's centre is inside.
void hullFacePlanes(const ConvexHull& hull, std::vector<std::array<double, 4>>& out);

/// Terrain as a regular grid of heights (huge-worlds.md §4): `columns` x `rows` samples, cellX
/// and cellY apart, centred on the shape's origin; heights[row * columns + column] is z in the
/// shape's frame at x = (column - (columns - 1) / 2) cellX, y = (row - (rows - 1) / 2) cellY.
/// Each cell is two triangles split along the diagonal from its (column, row) corner to its
/// (column + 1, row + 1) corner, the surface every backend and sensor uses.
struct HeightField {
    std::uint32_t columns{0};
    std::uint32_t rows{0};
    double cellX{0.0};  // m
    double cellY{0.0};
    std::vector<double> heights;  // m

    /// z of the surface above (x, y) in the shape's frame; empty outside the grid.
    [[nodiscard]] std::optional<double> heightAt(double x, double y) const;
    /// The surface as triangles (counter-clockwise from above) on its sample points.
    [[nodiscard]] ConvexHull triangles() const;  // points and triangles; not convex
};

/// Custom: a sensor Lagrange does not simulate itself; a plugin implements its kind (RFC 0008 §3).
enum class SensorType : std::uint8_t {
    Imu,
    Lidar,
    Camera,
    Gnss,
    Magnetometer,
    Barometer,
    Radar,
    Custom
};

/// What a lidar sees of a surface (SensorMaterialAPI_1; docs/design/domains/sensors.md §2).
struct SurfaceMaterial {
    double reflectivity905{0.5};   // of the diffuse and specular parts together, at 905 nm
    double reflectivity1550{0.5};  // at 1550 nm
    double specular{0.0};          // fraction that is mirror-like: comes back only near normal
    double transmission{0.0};      // fraction of beams that go through (glass)
    bool retroreflective{false};   // comes back at any angle, strongly (road signs, vests)
};
inline constexpr std::uint32_t kNoMaterial = 0xFFFF'FFFFU;

/// A named surface (sensmat:preset, <sensor_material preset=...>): false for an unknown name. The
/// numbers are illustrative orders of magnitude, not measurements; give your own where it matters.
[[nodiscard]] bool surfaceMaterialPreset(std::string_view name, SurfaceMaterial& out);

/// One applied API schema on one prim, with its attribute values read once at load.
/// Lookups are by name, so they belong in declare() and init(), never in step().
struct SchemaInstance {
    std::string schema;    // "ElectricalMotorAPI_1"
    std::string instance;  // a multiple-apply schema's instance ("dc_out"); empty otherwise
    PrimId prim;
    std::map<std::string, double, std::less<>> reals;
    std::map<std::string, std::int64_t, std::less<>> integers;
    std::map<std::string, std::string, std::less<>> tokens;
    std::map<std::string, std::string, std::less<>> assetPaths;  // resolved
    std::map<std::string, std::vector<PrimId>, std::less<>> relationships;
    std::map<std::string, std::vector<double>, std::less<>> realArrays;

    [[nodiscard]] std::optional<double> real(std::string_view attr) const;
    [[nodiscard]] std::optional<std::int64_t> integer(std::string_view attr) const;
    [[nodiscard]] std::optional<std::string_view> token(std::string_view attr) const;
    [[nodiscard]] std::optional<std::string_view> assetPath(std::string_view attr) const;
    [[nodiscard]] std::span<const PrimId> rel(std::string_view name) const;
    [[nodiscard]] std::span<const double> realArray(std::string_view attr) const;
};

/// The runtime world as flat tables (structure of arrays). core/stage/ fills it from the USD
/// stage at load (invariant 5); tests fill it by hand. It holds no USD types.
///
/// Positions are double (invariant 6) and relative to the active region's origin.
struct World {
    struct Prims {
        std::vector<std::string> paths;          // "/amr/base_link/left_wheel"
        std::vector<std::string> instanceNames;  // "left_wheel": the name used in signal names
        /// Where each prim is: the nearest body at or above it (kWorldBody for fixed scenery)
        /// and its pose in that body's frame (region-local for kWorldBody), every joint at zero.
        /// The loader fills them for the prims it adds; addPrim starts them at the world origin.
        std::vector<BodyId> body;
        std::vector<Pose> poseInBody;
    } prims;

    struct Bodies {  // every vector has one entry per body
        std::vector<std::string> names;
        std::vector<double> mass;                         // kg
        std::vector<std::array<double, 3>> comLocal;      // m, in the body frame
        std::vector<std::array<double, 6>> inertiaLocal;  // kg·m², (xx, yy, zz, xy, xz, yz)
        std::vector<PrimId> prim;
        std::vector<std::uint32_t> robot;  // index into robots
        /// Moved by a pose command (body/<b>/pose_command), not by forces: traffic agents. A
        /// kinematic body hangs from the world and pushes what it meets without being pushed.
        std::vector<bool> kinematic;
        std::vector<BodyId> parent;      // kWorldBody for a root
        std::vector<Pose> poseInParent;  // with every joint at zero
    } bodies;

    struct Joints {  // every vector has one entry per joint
        std::vector<std::string> names;
        std::vector<JointType> type;
        std::vector<BodyId> parent;
        std::vector<BodyId> child;
        std::vector<std::array<double, 3>> axis;
        std::vector<double> lowerLimit, upperLimit;  // rad or m
        std::vector<double> effortLimit;             // N·m or N
        std::vector<double> velocityLimit;           // rad/s or m/s
        /// Inertia added at the joint by what drives it: a motor's rotor reflected through its
        /// gear, N²·J_rotor (kg·m², or kg for a slide). Physics adds it to the joint's mass.
        std::vector<double> armature;
        /// Viscous damping (N·m·s/rad or N·s/m) and dry friction (N·m or N) at the joint.
        std::vector<double> damping, friction;
        /// A joint that copies another: position = multiplier × mimic's + offset (rad or m);
        /// kNoMimic for the others. It takes no commands. MuJoCo couples both ways, as gears do;
        /// Jolt only moves the copy.
        std::vector<std::uint32_t> mimic;
        std::vector<double> mimicMultiplier, mimicOffset;
        /// Where the joint starts, rad or m (0 by default): a run rebuilt mid-way, when robots
        /// are spawned or deleted, starts the others' joints where they were.
        std::vector<double> initialPosition;
        std::vector<PrimId> prim;
        std::vector<bool> actuated;  // a motor or controller drives it
    } joints;

    /// Collision geometry: convex shapes only, never render meshes (CLAUDE.md §4).
    struct Colliders {             // every vector has one entry per collider
        std::vector<BodyId> body;  // kWorldBody for static geometry such as the ground
        std::vector<ShapeType> shape;
        std::vector<std::array<double, 3>> size;  // m, see ShapeType
        std::vector<Pose> poseInBody;
        std::vector<double> friction;     // Coulomb coefficient, sliding
        std::vector<std::uint32_t> hull;  // index into hulls for a ConvexHull, else kNoHull
        /// Index into surfaceMaterials, else kNoMaterial: the defaults, and always a return.
        std::vector<std::uint32_t> material;
    } colliders;
    std::vector<SurfaceMaterial> surfaceMaterials;
    std::vector<ConvexHull> hulls;
    std::vector<HeightField> heightfields;  // ShapeType::HeightField, by Colliders::hull

    /// Sensors mounted on bodies (from SensorAPI schemas; from <sensor> blocks of a URDF until
    /// the stage loader exists). Sensor frame: x forward, z up.
    struct Sensors {                     // every vector has one entry per sensor
        std::vector<std::string> names;  // "front_lidar": the name used in signal names
        std::vector<SensorType> type;
        std::vector<BodyId> body;
        std::vector<Pose> poseInBody;
        std::vector<double> rateHz;
        std::vector<PrimId> prim;
        /// Type-specific settings by name, SI units: lidar "samples", "min_angle" (rad),
        /// "max_angle", "min_range" (m), "max_range", "range_noise" (m, 1 sigma); IMU
        /// "accel_noise" (m/s², 1 sigma), "gyro_noise" (rad/s, 1 sigma); camera "width",
        /// "height" (px), "horizontal_fov" (rad), "near_clip" (m), "far_clip" (m).
        std::vector<std::map<std::string, double, std::less<>>> settings;
        /// A custom sensor's kind ("sonar", "tof"; empty for the built-in types) and its text
        /// parameters (RFC 0008 §3); its numbers are in settings.
        std::vector<std::string> kind;
        std::vector<std::map<std::string, std::string, std::less<>>> text;
    } sensors;

    std::vector<std::string> robots;  // robot root prim names
    std::vector<SchemaInstance> schemaInstances;
    std::array<double, 3> gravityLocal{0.0, 0.0, -9.80665};  // m/s², standard gravity (CGPM 1901)
    /// Where the region-local frame of every position above sits in world ENU (floating origin,
    /// docs/design/huge-worlds.md §1).
    coords::RegionOrigin region;
    /// Where world ENU (0, 0, 0) is on the Earth (profile world.geo_origin, --geo-origin). A
    /// site parameter with no default: GNSS, magnetometers and barometers need it.
    std::optional<coords::Geodetic> geoOrigin;
    std::uint32_t batchCount{1};

    /// Adds a prim and returns its id. `instanceName` is the name used in signal names.
    PrimId addPrim(std::string path, std::string instanceName);

    /// Adds a body with its prim. Inertia is about the centre of mass, in the body frame.
    BodyId addBody(std::string name, BodyId parentBody, const Pose& poseInParentBody, double massKg,
                   const std::array<double, 3>& centreOfMass, const std::array<double, 6>& inertia);
    /// Adds a joint that moves `childBody` relative to its parent; the axis is in the child frame.
    JointId addJoint(std::string name, JointType jointType, BodyId childBody,
                     const std::array<double, 3>& jointAxis, bool isActuated);
    void addSensor(std::string name, SensorType sensorType, BodyId onBody, const Pose& pose,
                   double updateRateHz, std::map<std::string, double, std::less<>> sensorSettings);
    /// A custom sensor of `sensorKind`, for a plugin to implement (RFC 0008 §3).
    void addCustomSensor(std::string name, std::string sensorKind, BodyId onBody, const Pose& pose,
                         double updateRateHz,
                         std::map<std::string, double, std::less<>> sensorSettings,
                         std::map<std::string, std::string, std::less<>> textSettings);
    void addCollider(BodyId onBody, ShapeType shapeType, const std::array<double, 3>& shapeSize,
                     const Pose& poseInOwner, double slidingFriction);
    /// A convex hull collider; its size is computed from the points.
    void addHullCollider(BodyId onBody, ConvexHull hull, const Pose& poseInOwner,
                         double slidingFriction);
    /// A height field on the world (terrain), its grid centred at `poseInWorld`.
    void addHeightFieldCollider(HeightField field, const Pose& poseInWorld, double slidingFriction);
};

/// Read-only access to the world for modules (declare and init).
class WorldView {
public:
    explicit WorldView(const World& world) : world_(&world) {}

    [[nodiscard]] const World::Bodies& bodies() const { return world_->bodies; }
    [[nodiscard]] const World::Joints& joints() const { return world_->joints; }
    [[nodiscard]] const World::Colliders& colliders() const { return world_->colliders; }
    [[nodiscard]] const World::Sensors& sensors() const { return world_->sensors; }
    /// All tables, read-only, for helpers that take a World (PoseMath.hpp).
    [[nodiscard]] const World& raw() const { return *world_; }
    [[nodiscard]] std::size_t primCount() const { return world_->prims.paths.size(); }
    [[nodiscard]] std::string_view primPath(PrimId prim) const;
    [[nodiscard]] std::string_view signalInstanceName(PrimId prim) const;
    /// Where a prim is (World::Prims::body, poseInBody).
    [[nodiscard]] BodyId primBody(PrimId prim) const { return world_->prims.body.at(prim.index); }
    [[nodiscard]] const Pose& primPoseInBody(PrimId prim) const {
        return world_->prims.poseInBody.at(prim.index);
    }
    /// Every instance of an applied schema, in load order.
    [[nodiscard]] std::vector<const SchemaInstance*> instancesOf(std::string_view schema) const;
    [[nodiscard]] std::array<double, 3> gravityLocal() const { return world_->gravityLocal; }
    [[nodiscard]] std::uint32_t batchCount() const { return world_->batchCount; }
    /// Changes when prims, bodies, joints or schema instances change; not when values change.
    [[nodiscard]] std::uint64_t topologyHash() const;

private:
    const World* world_;
};

}  // namespace lagrange
