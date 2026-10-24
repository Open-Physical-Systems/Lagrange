#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"

#include <string>

namespace lagrange::stage {

struct ReadOptions {
    double defaultFriction{1.0};  // Coulomb coefficient of colliders without a physics material
    bool addGround{true};         // add the ground plane and slab when the stage has no plane
    double groundFriction{1.0};
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

struct WriteOptions {
    std::string doc;  // the layer's documentation string
};

/// Writes a model as a USD layer that loadStage reads back to the same model: the importers'
/// back end (URDF -> USD). Schema instances other than the sensors' are not written yet (they
/// come with the overlay files, roadmap 0.2-G2). The ground (the plane collider on the world and
/// the prim named "ground") is left out; loadStage adds it again. The lagrange_schemas plugins must
/// be on PXR_PLUGINPATH_NAME for the sensor schemas.
[[nodiscard]] Expected<void, std::string> writeStage(const Model& model, const std::string& path,
                                                     const WriteOptions& options = {});

}  // namespace lagrange::stage
