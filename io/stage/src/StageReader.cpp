// USD -> Model. The rules are listed on loadStage in Stage.hpp.
#include "Math.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/primDefinition.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/tokens.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/materialAPI.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/usdShade/shader.h>

namespace lagrange::stage {

namespace {

using namespace pxr;  // NOLINT(google-build-using-namespace): OpenUSD names are long enough

constexpr std::array<float, 4> kDefaultColor{0.70F, 0.70F, 0.72F, 1.0F};  // as the URDF loader
constexpr double kPlaceholderMassKg = 1e-3;

// Property namespaces of the project's schema libraries (RFC 0002 convention 2): an applied
// schema whose properties live in one of them is collected into World::schemaInstances.
constexpr std::array<std::string_view, 9> kSchemaNamespaces{
    "elec:", "thermal:", "aero:", "weather:", "emi:", "rad:", "sensmat:", "sensor:", "world:"};

// Loader checks of RFC 0002 ("Loading") that the schema definitions cannot express.
struct PairedArrays {
    std::string_view schema, first, second;
};
constexpr std::array<PairedArrays, 1> kPairedArrays{
    {{"BatteryAPI_1", "elec:battery:ocvSoc", "elec:battery:ocvVoltage"}}};
struct Fraction {
    std::string_view schema, attribute;
};
constexpr std::array<Fraction, 3> kFractions{
    {{"BatteryAPI_1", "elec:battery:initialSoc"},
     {"ElectricalMotorAPI_1", "elec:gearEfficiency"},
     {"ElectricalConverterAPI_1", "elec:conv:efficiency"}}};
struct JointTarget {
    std::string_view schema, relationship;
};
constexpr std::array<JointTarget, 1> kJointTargets{{{"ElectricalMotorAPI_1", "elec:joint"}}};
constexpr double kPlaceholderInertia = 1e-6;  // kg·m²
constexpr double kAnchorTolerance = 1e-6;     // m, joint anchor away from body1's origin

/// A transform split into pose and per-axis scale (rows of USD's row-vector matrices).
struct Placement {
    Pose pose;
    std::array<double, 3> scale{1.0, 1.0, 1.0};
};

Placement placementOf(const GfMatrix4d& m) {
    Placement out;
    out.pose.position = {m[3][0], m[3][1], m[3][2]};
    std::array<std::array<double, 3>, 3> r{};
    for (int i = 0; i < 3; ++i) {
        const double length = std::sqrt(m[i][0] * m[i][0] + m[i][1] * m[i][1] + m[i][2] * m[i][2]);
        out.scale.at(static_cast<std::size_t>(i)) = length;
        for (int j = 0; j < 3; ++j) {
            // Row i of USD's matrix is the image of axis i: column i of the rotation.
            r.at(static_cast<std::size_t>(j)).at(static_cast<std::size_t>(i)) =
                length > 0 ? m[i][j] / length : 0.0;
        }
    }
    const double trace = r[0][0] + r[1][1] + r[2][2];
    std::array<double, 4> q{};
    if (trace > 0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        q = {0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s};
    } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
        q = {(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s};
    } else if (r[1][1] > r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
        q = {(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
        q = {(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s};
    }
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    out.pose.orientation = {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
    return out;
}

std::array<double, 4> fromQuat(const GfQuatf& q) {
    return {q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]};
}

/// Rotation taking z onto the given axis token (USD gprims and joints name their axis).
std::array<double, 4> zTo(const TfToken& axis) {
    const double h = std::numbers::sqrt2 / 2.0;
    if (axis == UsdGeomTokens->x) {
        return {h, 0.0, h, 0.0};  // +90 deg about y
    }
    if (axis == UsdGeomTokens->y) {
        return {h, -h, 0.0, 0.0};  // -90 deg about x
    }
    return {1.0, 0.0, 0.0, 0.0};
}

std::array<double, 3> axisVector(const TfToken& axis) {
    if (axis == UsdPhysicsTokens->y) {
        return {0.0, 1.0, 0.0};
    }
    if (axis == UsdPhysicsTokens->z) {
        return {0.0, 0.0, 1.0};
    }
    return {1.0, 0.0, 0.0};
}

template <class T>
std::optional<T> authored(const UsdAttribute& attribute) {
    T value{};
    if (attribute && attribute.HasAuthoredValue() && attribute.Get(&value)) {
        return value;
    }
    return std::nullopt;
}

template <class T>
std::optional<T> valueOf(const UsdAttribute& attribute) {
    T value{};
    if (attribute && attribute.Get(&value)) {
        return value;
    }
    return std::nullopt;
}

class Reader {
public:
    Reader(UsdStageRefPtr stage, const ReadOptions& options, Model& model)
        : stage_(std::move(stage)), options_(options), m_(model) {}

    Expected<void, std::string> read() {
        if (auto status = checkUnits(); !status) {
            return status;
        }
        collect();
        if (auto status = buildBodies(); !status) {
            return status;
        }
        if (auto status = buildScene(); !status) {
            return status;
        }
        buildColliders();
        collectSchemas();
        if (!errors_.empty()) {
            std::string message = "the stage cannot be simulated:";
            for (const std::string& e : errors_) {
                message += "\n  " + e;
            }
            return Unexpected{message};
        }
        return {};
    }

private:
    Expected<void, std::string> checkUnits() {
        if (UsdGeomGetStageUpAxis(stage_) != UsdGeomTokens->z) {
            return Unexpected{std::string("the stage is not Z-up (upAxis); convert it first")};
        }
        if (std::abs(UsdGeomGetStageMetersPerUnit(stage_) - 1.0) > 1e-12) {
            return Unexpected{std::string("the stage's metersPerUnit is not 1")};
        }
        if (UsdPhysicsStageHasAuthoredKilogramsPerUnit(stage_) &&
            std::abs(UsdPhysicsGetStageKilogramsPerUnit(stage_) - 1.0) > 1e-12) {
            return Unexpected{std::string("the stage's kilogramsPerUnit is not 1")};
        }
        return {};
    }

    void note(std::string text) {
        if (std::find(m_.notes.begin(), m_.notes.end(), text) == m_.notes.end()) {
            m_.notes.push_back(std::move(text));
        }
    }

    [[nodiscard]] Placement world(const UsdPrim& prim) const {
        return placementOf(
            UsdGeomXformable(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default()));
    }

    /// The nearest rigid body at or above `prim`, or an empty path.
    [[nodiscard]] SdfPath bodyOf(const UsdPrim& prim) const {
        for (UsdPrim p = prim; p && !p.IsPseudoRoot(); p = p.GetParent()) {
            if (bodyIndex_.contains(p.GetPath())) {
                return p.GetPath();
            }
        }
        return {};
    }

    void collect() {
        for (const UsdPrim& prim : stage_->Traverse()) {
            if (prim.HasAPI<UsdPhysicsRigidBodyAPI>()) {
                bool enabled = true;
                UsdPhysicsRigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get(&enabled);
                if (enabled) {
                    bodyPrims_.push_back(prim);
                    bodyIndex_[prim.GetPath()] = kNoIndex;  // filled in buildBodies
                }
            }
            if (prim.IsA<UsdPhysicsJoint>()) {
                joints_.push_back(UsdPhysicsJoint(prim));
            }
        }
    }

    Expected<void, std::string> buildBodies() {
        World& w = m_.world;
        // Each body's joint to its parent: body0 -> body1.
        std::map<SdfPath, UsdPhysicsJoint> jointOf;
        for (const UsdPhysicsJoint& joint : joints_) {
            bool enabled = true;
            joint.GetJointEnabledAttr().Get(&enabled);
            SdfPathVector body0, body1;
            joint.GetBody0Rel().GetTargets(&body0);
            joint.GetBody1Rel().GetTargets(&body1);
            if (!enabled || body1.size() != 1 || !bodyIndex_.contains(body1[0])) {
                note("joint " + joint.GetPath().GetString() +
                     " is disabled or has no rigid body1; it was skipped");
                continue;
            }
            if (!body0.empty() && !bodyIndex_.contains(body0[0])) {
                errors_.push_back(joint.GetPath().GetString() + ": body0 " + body0[0].GetString() +
                                  " is not a rigid body");
                continue;
            }
            if (jointOf.contains(body1[0])) {
                errors_.push_back(body1[0].GetString() +
                                  " is body1 of two joints: kinematic loops are not supported");
                continue;
            }
            jointOf.emplace(body1[0], joint);
        }
        // Parents before children: start from the bodies no joint hangs from a body.
        std::deque<UsdPrim> queue;
        std::map<SdfPath, std::vector<UsdPrim>> children;
        for (const UsdPrim& prim : bodyPrims_) {
            const auto it = jointOf.find(prim.GetPath());
            SdfPathVector body0;
            if (it != jointOf.end()) {
                it->second.GetBody0Rel().GetTargets(&body0);
            }
            if (body0.empty()) {
                queue.push_back(prim);
            } else {
                children[body0[0]].push_back(prim);
            }
        }
        std::size_t added = 0;
        while (!queue.empty()) {
            const UsdPrim prim = queue.front();
            queue.pop_front();
            const auto it = jointOf.find(prim.GetPath());
            if (auto status = addBody(prim, it == jointOf.end() ? nullptr : &it->second); !status) {
                return status;
            }
            ++added;
            for (const UsdPrim& child : children[prim.GetPath()]) {
                queue.push_back(child);
            }
        }
        if (added != bodyPrims_.size()) {
            return Unexpected{
                std::string("the joints form a loop; kinematic loops are not supported")};
        }
        (void)w;
        return {};
    }

    Expected<void, std::string> addBody(const UsdPrim& prim, const UsdPhysicsJoint* joint) {
        World& w = m_.world;
        const Pose bodyWorld = world(prim).pose;
        BodyId parent = kWorldBody;
        Pose inParent = bodyWorld;
        if (joint != nullptr) {
            SdfPathVector body0;
            joint->GetBody0Rel().GetTargets(&body0);
            if (!body0.empty()) {
                parent = BodyId{bodyIndex_.at(body0[0])};
                inParent = relative(m_.bodyRestPose[parent.index], bodyWorld);
            }
        }
        // Mass properties, in the body frame.
        const UsdPhysicsMassAPI massApi(prim);
        double mass = authored<float>(massApi.GetMassAttr()).value_or(0.0F);
        std::array<double, 3> com{};
        if (const auto c = authored<GfVec3f>(massApi.GetCenterOfMassAttr())) {
            com = {(*c)[0], (*c)[1], (*c)[2]};
        }
        std::array<double, 6> inertia{};
        if (const auto d = authored<GfVec3f>(massApi.GetDiagonalInertiaAttr())) {
            std::array<double, 4> axes{1.0, 0.0, 0.0, 0.0};
            if (const auto q = authored<GfQuatf>(massApi.GetPrincipalAxesAttr());
                q && q->GetLength() > 0.5) {
                axes = fromQuat(*q);
            }
            inertia = inertiaTensor({(*d)[0], (*d)[1], (*d)[2]}, axes);
        }
        const bool welded = joint != nullptr && joint->GetPrim().IsA<UsdPhysicsFixedJoint>();
        if (mass <= 0.0) {
            note("rigid body " + prim.GetPath().GetString() +
                 " has no physics:mass; it was given 1 g (mass from density is not supported)");
            mass = kPlaceholderMassKg;
            inertia = {kPlaceholderInertia, kPlaceholderInertia, kPlaceholderInertia, 0, 0, 0};
        }
        const std::string name = prim.GetName().GetString();
        const BodyId body = w.addBody(name, parent, inParent, mass, com, inertia);
        bodyIndex_[prim.GetPath()] = body.index;
        worldPrim_[prim.GetPath()] = w.bodies.prim.back();
        m_.bodyRestPose.push_back(bodyWorld);
        if (joint == nullptr) {
            w.addJoint(name + "_floating", JointType::Free, body, {0.0, 0.0, 1.0}, false);
            return {};
        }
        if (welded) {
            return {};  // a fixed joint welds body1 to body0, or to the world
        }
        return addJoint(*joint, body);
    }

    Expected<void, std::string> addJoint(const UsdPhysicsJoint& joint, BodyId body) {
        World& w = m_.world;
        const std::string path = joint.GetPath().GetString();
        const GfVec3f anchor = valueOf<GfVec3f>(joint.GetLocalPos1Attr()).value_or(GfVec3f(0.0F));
        if (anchor.GetLength() > kAnchorTolerance) {
            errors_.push_back(path + ": the joint is not at body1's origin (localPos1 is not 0)");
            return {};
        }
        const std::array<double, 4> frame1 =
            fromQuat(valueOf<GfQuatf>(joint.GetLocalRot1Attr()).value_or(GfQuatf(1.0F)));
        JointType type = JointType::Fixed;
        TfToken axisToken = UsdPhysicsTokens->x;
        double lower = 0.0;
        double upper = 0.0;
        const UsdPrim prim = joint.GetPrim();
        if (prim.IsA<UsdPhysicsRevoluteJoint>()) {
            const UsdPhysicsRevoluteJoint revolute(prim);
            revolute.GetAxisAttr().Get(&axisToken);
            const float lo = valueOf<float>(revolute.GetLowerLimitAttr()).value_or(-INFINITY);
            const float hi = valueOf<float>(revolute.GetUpperLimitAttr()).value_or(INFINITY);
            if (std::isfinite(lo) && std::isfinite(hi)) {
                type = JointType::Revolute;
                constexpr double kRadians = std::numbers::pi / 180.0;
                lower = lo * kRadians;
                upper = hi * kRadians;
            } else {
                type = JointType::Continuous;
            }
        } else if (prim.IsA<UsdPhysicsPrismaticJoint>()) {
            const UsdPhysicsPrismaticJoint prismatic(prim);
            prismatic.GetAxisAttr().Get(&axisToken);
            type = JointType::Prismatic;
            const float lo = valueOf<float>(prismatic.GetLowerLimitAttr()).value_or(-INFINITY);
            const float hi = valueOf<float>(prismatic.GetUpperLimitAttr()).value_or(INFINITY);
            if (std::isfinite(lo) && std::isfinite(hi)) {
                lower = lo;
                upper = hi;
            }
        } else if (prim.IsA<UsdPhysicsSphericalJoint>()) {
            type = JointType::Spherical;
        } else {
            errors_.push_back(path + ": joint type " + prim.GetTypeName().GetString() +
                              " is not supported");
            return {};
        }
        // Torque or force drive: PhysicsDriveAPI:angular or :linear.
        const bool driven = prim.HasAPI<UsdPhysicsDriveAPI>(UsdPhysicsTokens->angular) ||
                            prim.HasAPI<UsdPhysicsDriveAPI>(UsdPhysicsTokens->linear);
        const std::array<double, 3> axis = rotate(frame1, axisVector(axisToken));
        const std::string name = prim.GetName().GetString();
        w.addJoint(name, type, body, axis, driven);
        worldPrim_[prim.GetPath()] = w.joints.prim.back();
        w.joints.lowerLimit.back() = lower;
        w.joints.upperLimit.back() = upper;
        if (driven) {
            const TfToken instance =
                type == JointType::Prismatic ? UsdPhysicsTokens->linear : UsdPhysicsTokens->angular;
            const float maxForce =
                valueOf<float>(UsdPhysicsDriveAPI(prim, instance).GetMaxForceAttr())
                    .value_or(INFINITY);
            if (std::isfinite(maxForce)) {
                w.joints.effortLimit.back() = maxForce;
            }
            m_.actuatedJoints.push_back(name);
        }
        return {};
    }

    /// The colour of a visible gprim: displayColor, else a bound UsdPreviewSurface's diffuse.
    std::array<float, 4> colorOf(const UsdPrim& prim) {
        std::array<float, 4> rgba = kDefaultColor;
        const UsdGeomGprim gprim(prim);
        VtArray<GfVec3f> colors;
        if (gprim.GetDisplayColorAttr().Get(&colors) && !colors.empty()) {
            rgba = {colors[0][0], colors[0][1], colors[0][2], 1.0F};
            VtArray<float> opacity;
            if (gprim.GetDisplayOpacityAttr().Get(&opacity) && !opacity.empty()) {
                rgba[3] = opacity[0];
            }
            return rgba;
        }
        const UsdShadeMaterial material = UsdShadeMaterialBindingAPI(prim).ComputeBoundMaterial();
        if (material) {
            const UsdShadeShader surface = material.ComputeSurfaceSource();
            if (surface) {
                GfVec3f diffuse;
                if (const UsdShadeInput input = surface.GetInput(TfToken("diffuseColor"));
                    input && input.Get(&diffuse)) {
                    rgba = {diffuse[0], diffuse[1], diffuse[2], 1.0F};
                }
            }
        }
        return rgba;
    }

    std::uint32_t material(const std::array<float, 4>& rgba) {
        for (std::uint32_t i = 0; i < m_.scene.materials.size(); ++i) {
            if (m_.scene.materials[i].rgba == rgba) {
                return i;
            }
        }
        m_.scene.materials.push_back(
            {"material_" + std::to_string(m_.scene.materials.size()), rgba});
        return static_cast<std::uint32_t>(m_.scene.materials.size() - 1);
    }

    /// The triangle mesh of a visible gprim, in its own frame (scale applied by the prim).
    std::optional<TriMesh> meshOf(const UsdPrim& prim, Pose& turn) {
        turn = Pose{};
        if (prim.IsA<UsdGeomMesh>()) {
            const UsdGeomMesh mesh(prim);
            VtArray<GfVec3f> points;
            VtArray<int> counts, indices;
            mesh.GetPointsAttr().Get(&points);
            mesh.GetFaceVertexCountsAttr().Get(&counts);
            mesh.GetFaceVertexIndicesAttr().Get(&indices);
            VtArray<GfVec3f> normals;
            mesh.GetNormalsAttr().Get(&normals);
            const bool vertexNormals = normals.size() == points.size() &&
                                       mesh.GetNormalsInterpolation() == UsdGeomTokens->vertex;
            TriMesh out;
            if (vertexNormals) {
                for (std::size_t v = 0; v < points.size(); ++v) {
                    out.positions.insert(out.positions.end(),
                                         {points[v][0], points[v][1], points[v][2]});
                    out.normals.insert(out.normals.end(),
                                       {normals[v][0], normals[v][1], normals[v][2]});
                }
            }
            std::size_t at = 0;
            for (const int count : counts) {
                for (int k = 1; k + 1 < count; ++k) {  // fan triangulation
                    const std::array<int, 3> tri{indices[at], indices[at + k], indices[at + k + 1]};
                    if (vertexNormals) {
                        for (const int i : tri) {
                            out.indices.push_back(static_cast<std::uint32_t>(i));
                        }
                        continue;
                    }
                    // Flat shading: three new vertices with the face normal.
                    const GfVec3f a = points[tri[0]], b = points[tri[1]], c = points[tri[2]];
                    GfVec3f n = GfCross(b - a, c - a);
                    if (n.GetLength() > 0) {
                        n.Normalize();
                    }
                    for (const GfVec3f& p : {a, b, c}) {
                        out.indices.push_back(static_cast<std::uint32_t>(out.positions.size() / 3));
                        out.positions.insert(out.positions.end(), {p[0], p[1], p[2]});
                        out.normals.insert(out.normals.end(), {n[0], n[1], n[2]});
                    }
                }
                at += static_cast<std::size_t>(count);
            }
            return out;
        }
        if (prim.IsA<UsdGeomCube>()) {
            const double size = valueOf<double>(UsdGeomCube(prim).GetSizeAttr()).value_or(2.0);
            return boxMesh(size, size, size);
        }
        if (prim.IsA<UsdGeomSphere>()) {
            return sphereMesh(valueOf<double>(UsdGeomSphere(prim).GetRadiusAttr()).value_or(1.0));
        }
        if (prim.IsA<UsdGeomCylinder>() || prim.IsA<UsdGeomCapsule>()) {
            const bool capsule = prim.IsA<UsdGeomCapsule>();
            const double radius =
                capsule ? valueOf<double>(UsdGeomCapsule(prim).GetRadiusAttr()).value_or(0.5)
                        : valueOf<double>(UsdGeomCylinder(prim).GetRadiusAttr()).value_or(1.0);
            const double height =
                capsule ? valueOf<double>(UsdGeomCapsule(prim).GetHeightAttr()).value_or(1.0)
                        : valueOf<double>(UsdGeomCylinder(prim).GetHeightAttr()).value_or(2.0);
            TfToken axis = UsdGeomTokens->z;
            (capsule ? UsdGeomCapsule(prim).GetAxisAttr() : UsdGeomCylinder(prim).GetAxisAttr())
                .Get(&axis);
            turn.orientation = zTo(axis);
            if (capsule) {
                note("capsule visuals are drawn as cylinders");
            }
            return cylinderMesh(radius, height);
        }
        return std::nullopt;
    }

    [[nodiscard]] static bool isCollider(const UsdPrim& prim) {
        return prim.HasAPI<UsdPhysicsCollisionAPI>();
    }

    Expected<void, std::string> buildScene() {
        std::map<SdfPath, std::uint32_t> primIndex;
        std::map<std::string, std::uint32_t> meshKey;
        const World::Bodies& bodies = m_.world.bodies;
        m_.bodyPrim.assign(bodies.names.size(), kNoIndex);
        UsdPrimRange range = stage_->Traverse();
        for (auto it = range.begin(); it != range.end(); ++it) {
            const UsdPrim prim = *it;
            const bool xform = prim.IsA<UsdGeomXformable>();
            const bool skip = !xform || prim.IsA<UsdPhysicsJoint>() || isCollider(prim) ||
                              prim.IsA<UsdShadeMaterial>() || prim.IsA<UsdGeomPlane>();
            const SdfPath parentPath = prim.GetParent().GetPath();
            const auto parent = primIndex.find(parentPath);
            if (skip || (!prim.GetParent().IsPseudoRoot() && parent == primIndex.end())) {
                it.PruneChildren();  // nothing below a skipped prim is shown
                continue;
            }
            ScenePrim scene;
            scene.name = prim.GetName().GetString();
            scene.path = prim.GetPath().GetString();
            scene.parent = parent == primIndex.end() ? kNoIndex : parent->second;
            const Placement placement = world(prim);
            const SdfPath bodyPath = bodyOf(prim);
            std::uint32_t body = kNoIndex;
            if (!bodyPath.IsEmpty()) {
                body = bodyIndex_.at(bodyPath);
            } else if (prim.HasAPI<UsdPhysicsArticulationRootAPI>()) {
                // An articulation's top prim moves with its root body.
                for (const UsdPrim& candidate : bodyPrims_) {
                    if (candidate.GetPath().HasPrefix(prim.GetPath()) &&
                        bodies.parent[bodyIndex_.at(candidate.GetPath())] == kWorldBody) {
                        body = bodyIndex_.at(candidate.GetPath());
                        break;
                    }
                }
            }
            scene.body = body;
            scene.offset =
                body == kNoIndex ? placement.pose : relative(m_.bodyRestPose[body], placement.pose);
            for (const TfToken& schema : prim.GetAppliedSchemas()) {
                scene.apiSchemas.push_back(schema.GetString());
            }
            if (prim.IsA<UsdGeomGprim>()) {
                Pose turn;
                const std::optional<TriMesh> mesh = meshOf(prim, turn);
                if (!mesh) {
                    note(prim.GetPath().GetString() + ": " + prim.GetTypeName().GetString() +
                         " is not drawn");
                    it.PruneChildren();
                    continue;
                }
                scene.typeName = "Mesh";
                const std::string key =
                    prim.IsA<UsdGeomMesh>()
                        ? prim.GetPath().GetString()
                        : prim.GetTypeName().GetString() + std::to_string(mesh->positions.size()) +
                              std::to_string(mesh->positions.empty() ? 0.0 : mesh->positions[0]);
                if (const auto found = meshKey.find(key); found != meshKey.end()) {
                    scene.mesh = found->second;
                } else {
                    m_.scene.meshes.push_back(*mesh);
                    scene.mesh = static_cast<std::uint32_t>(m_.scene.meshes.size() - 1);
                    meshKey.emplace(key, scene.mesh);
                }
                scene.material = material(colorOf(prim));
                scene.scale = {static_cast<float>(placement.scale[0]),
                               static_cast<float>(placement.scale[1]),
                               static_cast<float>(placement.scale[2])};
                scene.offset = compose(scene.offset, turn);
            } else if (prim.HasAPI(TfToken("SensorAPI_1"))) {
                if (auto status = addSensor(prim, body, scene); !status) {
                    return status;
                }
            } else {
                scene.typeName = "Xform";
            }
            m_.scene.prims.push_back(std::move(scene));
            const auto index = static_cast<std::uint32_t>(m_.scene.prims.size() - 1);
            primIndex.emplace(prim.GetPath(), index);
            if (bodyIndex_.contains(prim.GetPath())) {
                m_.bodyPrim[bodyIndex_.at(prim.GetPath())] = index;
            }
            if (m_.scene.prims[index].typeName == "Lidar" ||
                m_.scene.prims[index].typeName == "Imu") {
                m_.sensorPrim.push_back(index);
            }
        }
        // Joints' prims: the child link's prim, as the URDF loader does.
        for (std::size_t j = 0; j < m_.world.joints.names.size(); ++j) {
            m_.jointPrim.push_back(m_.bodyPrim[m_.world.joints.child[j].index]);
        }
        return {};
    }

    Expected<void, std::string> addSensor(const UsdPrim& prim, std::uint32_t body,
                                          ScenePrim& scene) {
        const std::string path = prim.GetPath().GetString();
        if (body == kNoIndex) {
            return Unexpected{path + ": a sensor must be on a rigid body"};
        }
        const auto real = [&prim](const char* name) {
            return valueOf<double>(prim.GetAttribute(TfToken(name)));
        };
        const std::optional<double> rate = real("sensor:rate");
        if (!rate || *rate <= 0.0) {
            return Unexpected{path + ": sensor:rate is required"};
        }
        std::map<std::string, double, std::less<>> settings;
        SensorType type = SensorType::Imu;
        if (prim.HasAPI(TfToken("LidarAPI_1"))) {
            type = SensorType::Lidar;
            scene.typeName = "Lidar";
            const auto integer = [&prim](const char* name) {
                return valueOf<int>(prim.GetAttribute(TfToken(name)));
            };
            const std::optional<int> columns = integer("sensor:lidar:horizontalSamples");
            const std::optional<double> rangeMin = real("sensor:lidar:rangeMin");
            const std::optional<double> rangeMax = real("sensor:lidar:rangeMax");
            if (!columns || !rangeMin || !rangeMax) {
                return Unexpected{path +
                                  ": LidarAPI_1 needs horizontalSamples, rangeMin and rangeMax"};
            }
            const GfVec2d horizontal =
                valueOf<GfVec2d>(prim.GetAttribute(TfToken("sensor:lidar:horizontalFov")))
                    .value_or(GfVec2d(-std::numbers::pi, std::numbers::pi));
            const GfVec2d vertical =
                valueOf<GfVec2d>(prim.GetAttribute(TfToken("sensor:lidar:verticalFov")))
                    .value_or(GfVec2d(0.0, 0.0));
            settings = {{"samples", *columns},
                        {"min_angle", horizontal[0]},
                        {"max_angle", horizontal[1]},
                        {"vertical_samples", integer("sensor:lidar:channels").value_or(1)},
                        {"vertical_min_angle", vertical[0]},
                        {"vertical_max_angle", vertical[1]},
                        {"min_range", *rangeMin},
                        {"max_range", *rangeMax},
                        {"range_noise", real("sensor:lidar:rangeNoiseStd").value_or(0.0)}};
        } else if (prim.HasAPI(TfToken("ImuAPI_1"))) {
            scene.typeName = "Imu";
            settings = {{"accel_noise", real("sensor:imu:accelNoiseStd").value_or(0.0)},
                        {"gyro_noise", real("sensor:imu:gyroNoiseStd").value_or(0.0)}};
        } else {
            note(path + ": a sensor without LidarAPI_1 or ImuAPI_1 is not simulated yet");
            scene.typeName = "Xform";
            return {};
        }
        m_.world.addSensor(prim.GetName().GetString(), type, BodyId{body}, scene.offset, *rate,
                           settings);
        worldPrim_[prim.GetPath()] = m_.world.sensors.prim.back();
        return {};
    }

    void buildColliders() {
        World& w = m_.world;
        const SdfPath none;
        for (const UsdPrim& prim : stage_->Traverse()) {
            if (!isCollider(prim)) {
                continue;
            }
            bool enabled = true;
            UsdPhysicsCollisionAPI(prim).GetCollisionEnabledAttr().Get(&enabled);
            if (!enabled) {
                continue;
            }
            const SdfPath bodyPath = bodyOf(prim);
            const BodyId body = bodyPath.IsEmpty() ? kWorldBody : BodyId{bodyIndex_.at(bodyPath)};
            const Placement placement = world(prim);
            Pose pose = body == kWorldBody ? placement.pose
                                           : relative(m_.bodyRestPose[body.index], placement.pose);
            const std::array<double, 3>& s = placement.scale;
            const double friction = frictionOf(prim);
            const std::string path = prim.GetPath().GetString();
            if (prim.IsA<UsdGeomCube>()) {
                const double size = valueOf<double>(UsdGeomCube(prim).GetSizeAttr()).value_or(2.0);
                w.addCollider(body, ShapeType::Box,
                              {size * s[0] / 2.0, size * s[1] / 2.0, size * s[2] / 2.0}, pose,
                              friction);
            } else if (prim.IsA<UsdGeomSphere>()) {
                const double r = valueOf<double>(UsdGeomSphere(prim).GetRadiusAttr()).value_or(1.0);
                if (std::abs(s[0] - s[1]) > 1e-9 || std::abs(s[0] - s[2]) > 1e-9) {
                    note(path + ": a non-uniformly scaled sphere collides as a sphere");
                }
                w.addCollider(body, ShapeType::Sphere, {r * s[0], 0.0, 0.0}, pose, friction);
            } else if (prim.IsA<UsdGeomCylinder>() || prim.IsA<UsdGeomCapsule>()) {
                const bool capsule = prim.IsA<UsdGeomCapsule>();
                const double r =
                    capsule ? valueOf<double>(UsdGeomCapsule(prim).GetRadiusAttr()).value_or(0.5)
                            : valueOf<double>(UsdGeomCylinder(prim).GetRadiusAttr()).value_or(1.0);
                const double h =
                    capsule ? valueOf<double>(UsdGeomCapsule(prim).GetHeightAttr()).value_or(1.0)
                            : valueOf<double>(UsdGeomCylinder(prim).GetHeightAttr()).value_or(2.0);
                TfToken axis = UsdGeomTokens->z;
                (capsule ? UsdGeomCapsule(prim).GetAxisAttr() : UsdGeomCylinder(prim).GetAxisAttr())
                    .Get(&axis);
                pose = compose(pose, Pose{{0, 0, 0}, zTo(axis)});
                w.addCollider(body, capsule ? ShapeType::Capsule : ShapeType::Cylinder,
                              {r * s[0], h * s[2] / 2.0, 0.0}, pose, friction);
            } else if (prim.IsA<UsdGeomPlane>()) {
                w.addCollider(body, ShapeType::Plane, {0.0, 0.0, 1.0}, pose, friction);
                hasPlane_ = true;
            } else if (prim.IsA<UsdGeomMesh>()) {
                note(path +
                     ": mesh colliders are not used yet; convex decomposition comes with "
                     "the collision bake tool");
            } else {
                note(path + ": collider type " + prim.GetTypeName().GetString() +
                     " is not supported");
            }
        }
    }

    double frictionOf(const UsdPrim& prim) const {
        const UsdShadeMaterial material =
            UsdShadeMaterialBindingAPI(prim).ComputeBoundMaterial(TfToken("physics"));
        if (material && material.GetPrim().HasAPI<UsdPhysicsMaterialAPI>()) {
            if (const auto f = authored<float>(
                    UsdPhysicsMaterialAPI(material.GetPrim()).GetDynamicFrictionAttr())) {
                return *f;
            }
        }
        return options_.defaultFriction;
    }

    /// The world prim of a USD prim: its body, joint or sensor, or a new prim (a bus, a battery)
    /// named after it.
    PrimId worldPrimOf(const UsdPrim& prim) {
        if (const auto found = worldPrim_.find(prim.GetPath()); found != worldPrim_.end()) {
            return found->second;
        }
        const PrimId id = m_.world.addPrim(prim.GetPath().GetString(), prim.GetName().GetString());
        worldPrim_.emplace(prim.GetPath(), id);
        return id;
    }

    /// Every applied schema of the project's libraries, with its values read once
    /// (invariant 5), then the RFC 0002 loader checks. Problems go to errors_ together.
    void collectSchemas() {
        const UsdSchemaRegistry& registry = UsdSchemaRegistry::GetInstance();
        for (const UsdPrim& prim : stage_->Traverse()) {
            // GetAppliedSchemas() leaves out schemas USD does not know; the authored list has
            // them.
            SdfTokenListOp authored;
            TfTokenVector names;
            if (prim.GetMetadata(UsdTokens->apiSchemas, &authored)) {
                authored.ApplyOperations(&names);
            }
            for (const TfToken& applied : names) {
                const auto [schema, instance] = UsdSchemaRegistry::GetTypeNameAndInstance(applied);
                const UsdPrimDefinition* definition = registry.FindAppliedAPIPrimDefinition(schema);
                if (definition == nullptr) {
                    if (ours(schema.GetString())) {
                        note(prim.GetPath().GetString() + ": " + applied.GetString() +
                             " is not a registered schema here; its values are not read");
                    }
                    continue;
                }
                if (!ours(*definition)) {
                    continue;
                }
                SchemaInstance s;
                s.schema = schema.GetString();
                s.instance = instance.GetString();
                s.prim = worldPrimOf(prim);
                for (const TfToken& property : definition->GetPropertyNames()) {
                    const TfToken name = instance.IsEmpty()
                                             ? property
                                             : UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                                   property.GetString(), instance.GetString());
                    readProperty(prim, *definition, property, name, s);
                }
                check(prim, s);
                m_.world.schemaInstances.push_back(std::move(s));
            }
        }
    }

    /// Whether `name` looks like one of ours though its library is not loaded: the families
    /// of RFC 0002's schema set end in "API_<N>" and are not USD's own.
    static bool ours(const std::string& name) {
        return name.find("API_") != std::string::npos && name.rfind("Physics", 0) != 0;
    }

    static bool ours(const UsdPrimDefinition& definition) {
        for (const TfToken& property : definition.GetPropertyNames()) {
            const std::string_view text = property.GetString();
            for (const std::string_view space : kSchemaNamespaces) {
                if (text.starts_with(space)) {
                    return true;
                }
            }
        }
        return false;
    }

    void readProperty(const UsdPrim& prim, const UsdPrimDefinition& definition,
                      const TfToken& property, const TfToken& name, SchemaInstance& s) {
        const std::string where = prim.GetPath().GetString() + ": " + name.GetString();
        if (const UsdRelationship rel = prim.GetRelationship(name)) {
            SdfPathVector targets;
            rel.GetForwardedTargets(&targets);
            std::vector<PrimId> ids;
            for (const SdfPath& target : targets) {
                const UsdPrim targetPrim = stage_->GetPrimAtPath(target.GetPrimPath());
                if (!targetPrim) {
                    errors_.push_back(where + " targets " + target.GetString() +
                                      ", which is not in the stage");
                    continue;
                }
                ids.push_back(worldPrimOf(targetPrim));
            }
            if (!ids.empty()) {
                s.relationships.emplace(name.GetString(), std::move(ids));
            }
            return;
        }
        const UsdAttribute attribute = prim.GetAttribute(name);
        VtValue value;
        if (!attribute || !attribute.Get(&value) || value.IsEmpty()) {
            return;  // no fallback and not authored: the module says whether it is required
        }
        const std::string key = name.GetString();
        if (value.IsHolding<double>()) {
            s.reals.emplace(key, value.UncheckedGet<double>());
        } else if (value.IsHolding<int>()) {
            s.integers.emplace(key, value.UncheckedGet<int>());
        } else if (value.IsHolding<bool>()) {
            s.integers.emplace(key, value.UncheckedGet<bool>() ? 1 : 0);
        } else if (value.IsHolding<TfToken>()) {
            const TfToken token = value.UncheckedGet<TfToken>();
            VtTokenArray allowed;
            if (definition.GetPropertyMetadata(property, SdfFieldKeys->AllowedTokens, &allowed) &&
                !allowed.empty() &&
                std::find(allowed.begin(), allowed.end(), token) == allowed.end()) {
                std::string list;
                for (const TfToken& a : allowed) {
                    list += (list.empty() ? "" : ", ") + a.GetString();
                }
                errors_.push_back(where + " is \"" + token.GetString() + "\"; allowed: " + list);
            }
            s.tokens.emplace(key, token.GetString());
        } else if (value.IsHolding<std::string>()) {
            s.tokens.emplace(key, value.UncheckedGet<std::string>());
        } else if (value.IsHolding<SdfAssetPath>()) {
            const SdfAssetPath& asset = value.UncheckedGet<SdfAssetPath>();
            if (asset.GetAssetPath().empty()) {
                return;
            }
            if (asset.GetResolvedPath().empty()) {
                errors_.push_back(where + " names " + asset.GetAssetPath() +
                                  ", which cannot be found");
                return;
            }
            s.assetPaths.emplace(key, asset.GetResolvedPath());
        } else if (value.IsHolding<VtDoubleArray>()) {
            const VtDoubleArray& array = value.UncheckedGet<VtDoubleArray>();
            s.realArrays.emplace(key, std::vector<double>(array.begin(), array.end()));
        } else if (value.IsHolding<GfVec2d>()) {
            const GfVec2d& v = value.UncheckedGet<GfVec2d>();
            s.realArrays.emplace(key, std::vector<double>{v[0], v[1]});
        } else if (value.IsHolding<GfVec3d>()) {
            const GfVec3d& v = value.UncheckedGet<GfVec3d>();
            s.realArrays.emplace(key, std::vector<double>{v[0], v[1], v[2]});
        } else {
            note(where + ": values of type " + attribute.GetTypeName().GetAsToken().GetString() +
                 " are not read yet");
        }
    }

    void check(const UsdPrim& prim, const SchemaInstance& s) {
        const std::string path = prim.GetPath().GetString();
        for (const PairedArrays& rule : kPairedArrays) {
            if (s.schema == rule.schema &&
                s.realArray(rule.first).size() != s.realArray(rule.second).size()) {
                errors_.push_back(path + ": " + std::string(rule.first) + " and " +
                                  std::string(rule.second) + " differ in length (" +
                                  std::to_string(s.realArray(rule.first).size()) + " and " +
                                  std::to_string(s.realArray(rule.second).size()) + ")");
            }
        }
        for (const Fraction& rule : kFractions) {
            const std::optional<double> value = s.real(rule.attribute);
            if (s.schema == rule.schema && value && !(*value >= 0.0 && *value <= 1.0)) {
                errors_.push_back(path + ": " + std::string(rule.attribute) + " is " +
                                  std::to_string(*value) + "; it must be within 0 to 1");
            }
        }
        for (const JointTarget& rule : kJointTargets) {
            if (s.schema != rule.schema) {
                continue;
            }
            const UsdRelationship rel =
                prim.GetRelationship(TfToken(std::string(rule.relationship)));
            SdfPathVector targets;
            if (rel) {
                rel.GetForwardedTargets(&targets);
            }
            for (const SdfPath& target : targets) {
                const UsdPrim joint = stage_->GetPrimAtPath(target);
                if (joint && !joint.IsA<UsdPhysicsJoint>()) {
                    errors_.push_back(path + ": " + std::string(rule.relationship) + " targets " +
                                      target.GetString() + ", which is not a joint");
                }
            }
        }
    }

public:
    [[nodiscard]] bool hasPlane() const { return hasPlane_; }

private:
    UsdStageRefPtr stage_;
    const ReadOptions& options_;
    Model& m_;
    std::vector<UsdPrim> bodyPrims_;
    std::map<SdfPath, std::uint32_t> bodyIndex_;
    std::map<SdfPath, PrimId> worldPrim_;  // USD prim -> world prim, for schema instances
    std::vector<UsdPhysicsJoint> joints_;
    std::vector<std::string> errors_;
    bool hasPlane_{false};
};

}  // namespace

Expected<Model, std::string> loadStage(const std::string& path, const ReadOptions& options) {
    const UsdStageRefPtr stage = UsdStage::Open(path);
    if (!stage) {
        return Unexpected{"cannot open " + path + " as a USD stage"};
    }
    Model model;
    model.sourcePath = path;
    Reader reader(stage, options, model);
    if (auto read = reader.read(); !read) {
        return Unexpected{read.error()};
    }
    if (options.addGround && !reader.hasPlane()) {
        // As the URDF loader: the infinite plane first among the colliders, the slab last.
        World& w = model.world;
        w.colliders.body.insert(w.colliders.body.begin(), kWorldBody);
        w.colliders.shape.insert(w.colliders.shape.begin(), ShapeType::Plane);
        w.colliders.size.insert(w.colliders.size.begin(), {0.0, 0.0, 1.0});
        w.colliders.poseInBody.insert(w.colliders.poseInBody.begin(), Pose{});
        w.colliders.friction.insert(w.colliders.friction.begin(), options.groundFriction);
        addGroundPrim(model);
    }
    const UsdPrim root = stage->GetDefaultPrim();
    model.name =
        root ? root.GetName().GetString()
             : (model.scene.prims.empty() ? std::string("stage") : model.scene.prims[0].name);
    return model;
}

}  // namespace lagrange::stage
