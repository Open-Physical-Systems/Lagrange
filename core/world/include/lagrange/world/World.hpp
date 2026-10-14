#pragma once

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

/// One applied API schema on one prim, with its attribute values read once at load.
/// Lookups are by name, so they belong in declare() and init(), never in step().
struct SchemaInstance {
    std::string schema;  // "ElectricalMotorAPI_1"
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
    } prims;

    struct Bodies {  // every vector has one entry per body
        std::vector<std::string> names;
        std::vector<double> mass;                         // kg
        std::vector<std::array<double, 3>> comLocal;      // m, in the body frame
        std::vector<std::array<double, 6>> inertiaLocal;  // kg·m², (xx, yy, zz, xy, xz, yz)
        std::vector<PrimId> prim;
        std::vector<std::uint32_t> robot;  // index into robots
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
        std::vector<PrimId> prim;
    } joints;

    std::vector<std::string> robots;  // robot root prim names
    std::vector<SchemaInstance> schemaInstances;
    std::array<double, 3> gravityLocal{0.0, 0.0, -9.80665};  // m/s², standard gravity (CGPM 1901)
    std::uint32_t batchCount{1};

    /// Adds a prim and returns its id. `instanceName` is the name used in signal names.
    PrimId addPrim(std::string path, std::string instanceName);
};

/// Read-only access to the world for modules (declare and init).
class WorldView {
public:
    explicit WorldView(const World& world) : world_(&world) {}

    [[nodiscard]] const World::Bodies& bodies() const { return world_->bodies; }
    [[nodiscard]] const World::Joints& joints() const { return world_->joints; }
    [[nodiscard]] std::size_t primCount() const { return world_->prims.paths.size(); }
    [[nodiscard]] std::string_view primPath(PrimId prim) const;
    [[nodiscard]] std::string_view signalInstanceName(PrimId prim) const;
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
