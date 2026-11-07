#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"

#include <optional>
#include <string>
#include <vector>

namespace lagrange::stage {

struct ReadOptions {
    double defaultFriction{1.0};  // Coulomb coefficient of colliders without a physics material
    bool addGround{true};         // add the ground plane and slab when the stage has no plane
    double groundFriction{1.0};
    /// Empty: read the stage with its tiles (WorldTileAPI_1 prims) unloaded and listed in
    /// Model::tiles. A tile's path: read only that tile, its payload loaded (static scenery
    /// only), for the server's streaming.
    std::string tile;
};

/// Reads a USD stage (RFC 0002) into world tables and a scene: invariant 5, USD is read once
/// here and never during a step.
///
/// - Bodies: prims with PhysicsRigidBodyAPI, mass properties from PhysicsMassAPI (mass,
///   centerOfMass, diagonalInertia and principalAxes). The body tree follows the joints:
///   PhysicsFixedJoint welds body1 to body0, the other UsdPhysics joints become World joints
///   at body1's origin, a body joined to nothing is free, and one welded to no body0 is fixed
///   to the world. Revolute limits are degrees in USD; a revolute joint without limits is
///   continuous. PhysicsDriveAPI on a joint makes it actuated (torque or force).
/// - Colliders: Cube, Sphere, Cylinder, Capsule and Plane prims with PhysicsCollisionAPI, on
///   their nearest rigid-body ancestor or on the world; friction from the bound physics
///   material's dynamicFriction.
/// - Scene: every Xform and visible Gprim (not the colliders) in the stage's hierarchy, meshes
///   triangulated, colour from displayColor or a bound UsdPreviewSurface's diffuseColor.
/// - Sensors: prims with SensorAPI_1 and LidarAPI_1 or ImuAPI_1, on their nearest rigid body.
/// - Schema instances: every applied schema of the project's libraries (properties in the
///   namespaces of RFC 0002 convention 2) becomes a World::schemaInstances row with its values
///   and fallbacks, a multiple-apply schema with its instance name. Relationship targets become
///   world prims: a body's, a joint's or a sensor's, or a new prim named after the target (a
///   bus). Checked: tokens within allowedTokens, relationship targets in the stage, a motor's
///   elec:joint on a joint, ocvSoc and ocvVoltage of equal length, fractions within 0 to 1.
///   Required values are the consuming module's to check. A schema of ours that is not
///   registered (a library not built yet) is noted and skipped.
///
/// The stage must be Z-up with metersPerUnit 1 and kilogramsPerUnit 1. Joint anchors must be at
/// body1's origin (as the importers write them). Everything that is skipped is listed in
/// Model::notes; anything that cannot be simulated is an error naming the prim.
[[nodiscard]] Expected<Model, std::string> loadStage(const std::string& path,
                                                     const ReadOptions& options = {});

/// What the schema registry says about one attribute of an applied schema of ours.
struct SchemaAttribute {
    std::string unit;  // the lagrangeUnit text ("ohm", "N.m/A"); empty when dimensionless/none
    std::string doc;
    std::string type;  // the USD value type: "double", "float", "int", "token", "double[]", ...
};

/// `attribute` of `schema` ("ElectricalMotorAPI_1", "elec:windingResistance"); for a
/// multiple-apply schema the attribute names its instance ("thermal:link:to_chassis:target").
/// Empty when the schema or the attribute is not registered.
[[nodiscard]] std::optional<SchemaAttribute> schemaAttribute(const std::string& schema,
                                                             const std::string& instance,
                                                             const std::string& attribute);

/// The loader's value checks for one number (RFC 0002 "Loading": fractions within 0 to 1):
/// a message when `value` is not acceptable for `attribute` of `schema`.
[[nodiscard]] std::optional<std::string> checkSchemaReal(const std::string& schema,
                                                         const std::string& attribute,
                                                         double value);

/// Applies an overlay file (docs/design/formats.md §9) to a loaded model: the project's API
/// schemas by name, for robots from formats without them (URDF, SDFormat, MJCF) or added to a USD
/// one. YAML:
///
///     schema_version: 1
///     apply:
///       - prim: left_wheel_joint          # a link, joint or sensor by name, or a prim path
///         create_child: left_drive_motor  # optional: a new prim under it (a joint's: under
///         schemas:                        #   its parent link) that gets the schemas
///           ElectricalMotorAPI_1:
///             elec:torqueConstant: 0.085
///             elec:joint: left_wheel_joint      # relationships: names or paths, or lists
///             elec:powerPort: power/dc_48v
///       - create: power/dc_48v            # a new prim at this path under the top prim
///         schemas:
///           ElectricalBusAPI_1: { elec:bus:undervoltageTrip: 42.0 }
///
/// Values take the type of the schema's property. Asset paths are relative to the overlay file.
/// The result is checked as loadStage checks a stage. New prims become Xform scene prims that
/// move with their parent and world prims named after themselves. Every problem is reported
/// together, and a refused overlay leaves the model unchanged. A SPICE netlist (.cir, .sp, .spice,
/// .net, .ckt) is taken in its place: formats.md §9.1.
[[nodiscard]] Expected<void, std::string> applyOverlay(Model& model, const std::string& path);

/// A SPICE netlist as an overlay (formats.md §9.1), with more `*lagrange` lines from outside it,
/// as a URDF's <lagrange> block gives.
[[nodiscard]] Expected<void, std::string> applyNetlist(
    Model& model, const std::string& path, const std::vector<std::string>& directives = {});

struct WriteOptions {
    std::string doc;  // the layer's documentation string
    /// Where static colliders (on the world, not on a body) are written: a scope at the top by
    /// default; a path under the default prim for a layer used as a payload (a world tile).
    std::string staticColliders{"/colliders"};
};

/// Writes a model as a USD layer that loadStage reads back to the same model: the importers'
/// back end (URDF -> USD). Schema instances are written on their prims, values equal to the
/// schema's fallback left unauthored; asset paths below the layer's directory become relative.
/// The ground (the plane collider on the world and
/// the prim named "ground") is left out; loadStage adds it again. The lagrange_schemas plugins must
/// be on PXR_PLUGINPATH_NAME for the sensor schemas.
[[nodiscard]] Expected<void, std::string> writeStage(const Model& model, const std::string& path,
                                                     const WriteOptions& options = {});

}  // namespace lagrange::stage
