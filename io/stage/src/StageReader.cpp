// USD -> Model. The rules are listed on loadStage in Stage.hpp.
#include "Math.hpp"
#include "Schemas.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <regex>
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
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
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
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
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

    static constexpr int kNewerVersionsSearched = 8;  // versions above an unknown one tried

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

    /// Schema versions (formats.md §10): a prim that applies an older version of a family this
    /// build knows is reported with the command that migrates it; two versions of one family
    /// on one prim is an error.
    void checkSchemaVersions(const UsdPrim& prim) {
        static const std::regex applied(R"(^([A-Za-z][A-Za-z0-9]*)_([0-9]+)(:.+)?$)");
        const UsdSchemaRegistry& registry = UsdSchemaRegistry::GetInstance();
        std::map<std::string, int> versions;  // family + instance -> version
        // The authored list, not GetAppliedSchemas(), which leaves out unregistered names.
        SdfTokenListOp authored;
        TfTokenVector names;
        if (prim.GetMetadata(UsdTokens->apiSchemas, &authored)) {
            authored.ApplyOperations(&names);
        }
        for (const TfToken& token : names) {
            std::smatch match;
            const std::string name = token.GetString();
            if (!std::regex_match(name, match, applied)) {
                continue;
            }
            const std::string family = match[1];
            const int version = std::stoi(match[2]);
            const auto [at, inserted] = versions.emplace(family + match[3].str(), version);
            if (!inserted && at->second != version) {
                errors_.push_back(prim.GetPath().GetString() + " applies two versions of " +
                                  family);
                continue;
            }
            if (registry.FindAppliedAPIPrimDefinition(TfToken(family + "_" + match[2].str()))) {
                continue;
            }
            for (int newer = version + 1; newer <= version + kNewerVersionsSearched; ++newer) {
                const std::string current = family + "_" + std::to_string(newer);
                if (registry.FindAppliedAPIPrimDefinition(TfToken(current))) {
                    note(prim.GetPath().GetString() + " applies " + family + "_" +
                         std::to_string(version) + ", an older version of " + current +
                         ", which is not read: migrate the file with `ros2 run lagrange_migrate "
                         "migrate_schema FILE --in-place`");
                    break;
                }
            }
        }
    }

    void collect() {
        for (const UsdPrim& prim : stage_->Traverse()) {
            checkSchemaVersions(prim);
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
                m_.scene.prims[index].typeName == "Imu" ||
                m_.scene.prims[index].typeName == "Camera") {
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
        Pose mount = scene.offset;
        if (prim.HasAPI(TfToken("CameraSensorAPI_1"))) {
            const UsdGeomCamera camera(prim);
            const std::optional<int> width =
                valueOf<int>(prim.GetAttribute(TfToken("sensor:camera:width")));
            const std::optional<int> height =
                valueOf<int>(prim.GetAttribute(TfToken("sensor:camera:height")));
            if (!camera || !width || !height || *width < 1 || *height < 1) {
                return Unexpected{path +
                                  ": CameraSensorAPI_1 goes on a Camera prim and needs "
                                  "sensor:camera:width and height"};
            }
            type = SensorType::Camera;
            scene.typeName = "Camera";
            float focal = 0.0F;
            float aperture = 0.0F;
            GfVec2f clip(0.05F, 100.0F);
            camera.GetFocalLengthAttr().Get(&focal);
            camera.GetHorizontalApertureAttr().Get(&aperture);
            camera.GetClippingRangeAttr().Get(&clip);
            if (focal <= 0.0F || aperture <= 0.0F) {
                return Unexpected{path +
                                  ": the camera's focalLength and horizontalAperture must "
                                  "be positive"};
            }
            settings = {{"width", *width},
                        {"height", *height},
                        {"horizontal_fov", 2.0 * std::atan(aperture / 2.0 / focal)},
                        {"near_clip", clip[0]},
                        {"far_clip", clip[1]}};
            // The sensor looks along x; the USD camera along its -z.
            mount = compose(scene.offset, {{0, 0, 0}, conjugate(kSensorFromUsdCamera)});
        } else if (prim.HasAPI(TfToken("LidarAPI_1"))) {
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
            note(path +
                 ": a sensor without LidarAPI_1, ImuAPI_1 or CameraSensorAPI_1 is not "
                 "simulated yet");
            scene.typeName = "Xform";
            return {};
        }
        m_.world.addSensor(prim.GetName().GetString(), type, BodyId{body}, mount, *rate, settings);
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
                addMeshCollider(prim, body, pose, s, friction);
            } else {
                note(path + ": collider type " + prim.GetTypeName().GetString() +
                     " is not supported");
            }
        }
    }

    /// A mesh collider (formats.md §2: convex hulls, never render meshes): one marked
    /// physics:approximation "convexHull" whose points are convex is a hull; anything else is
    /// skipped with a note naming the collision bake.
    void addMeshCollider(const UsdPrim& prim, BodyId body, const Pose& pose,
                         const std::array<double, 3>& scale, double friction) {
        const std::string path = prim.GetPath().GetString();
        TfToken approximation = UsdPhysicsTokens->none;
        if (prim.HasAPI<UsdPhysicsMeshCollisionAPI>()) {
            UsdPhysicsMeshCollisionAPI(prim).GetApproximationAttr().Get(&approximation);
        }
        if (approximation != UsdPhysicsTokens->convexHull) {
            note(path + ": a mesh collider with approximation \"" + approximation.GetString() +
                 "\" is not used; bake it into convex hulls with `ros2 run "
                 "lagrange_collision_bake collision_bake`");
            return;
        }
        const UsdGeomMesh mesh(prim);
        VtVec3fArray points;
        VtIntArray counts;
        VtIntArray indices;
        mesh.GetPointsAttr().Get(&points);
        mesh.GetFaceVertexCountsAttr().Get(&counts);
        mesh.GetFaceVertexIndicesAttr().Get(&indices);
        ConvexHull hull;
        for (const GfVec3f& p : points) {
            hull.points.push_back({p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]});
        }
        std::size_t at = 0;
        for (const int count : counts) {
            for (int k = 1; k + 1 < count; ++k) {  // a fan per face
                const auto index = [&](std::size_t i) {
                    return static_cast<std::uint32_t>(indices[i]);
                };
                hull.triangles.push_back({index(at), index(at + k), index(at + k + 1)});
            }
            at += static_cast<std::size_t>(std::max(count, 0));
        }
        for (const auto& t : hull.triangles) {
            if (std::max({t[0], t[1], t[2]}) >= hull.points.size()) {
                errors_.push_back(path + ": a face indexes a point the mesh does not have");
                return;
            }
        }
        if (hull.points.size() < 4 || hull.triangles.size() < 4 || !convex(hull)) {
            note(path +
                 ": approximation \"convexHull\" on a mesh that is not convex; it is not "
                 "used: decompose it with `ros2 run lagrange_collision_bake "
                 "collision_bake`");
            return;
        }
        m_.world.addHullCollider(body, std::move(hull), pose, friction);
    }

    /// Whether every point lies on the inner side of every face plane (within 1e-6 of the
    /// hull's size): the face-plane ray casts are exact only for a convex mesh.
    static bool convex(const ConvexHull& hull) {
        std::array<double, 3> centre{};
        double extent = 0.0;
        for (const auto& p : hull.points) {
            for (std::size_t i = 0; i < 3; ++i) {
                centre.at(i) += p.at(i) / static_cast<double>(hull.points.size());
                extent = std::max(extent, std::abs(p.at(i)));
            }
        }
        const double tolerance = 1e-6 * std::max(extent, 1e-3);
        for (const auto& t : hull.triangles) {
            const auto& a = hull.points[t[0]];
            const auto& b = hull.points[t[1]];
            const auto& c = hull.points[t[2]];
            const std::array<double, 3> u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const std::array<double, 3> v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            const std::array<double, 3> n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                          u[0] * v[1] - u[1] * v[0]};
            const double length = std::hypot(n[0], n[1], n[2]);
            if (length <= 0.0) {
                continue;
            }
            const auto side = [&](const std::array<double, 3>& x) {
                return (n[0] * (x[0] - a[0]) + n[1] * (x[1] - a[1]) + n[2] * (x[2] - a[2])) /
                       length;
            };
            const double sign = side(centre) > 0.0 ? -1.0 : 1.0;  // outward
            for (const auto& p : hull.points) {
                if (sign * side(p) > tolerance) {
                    return false;
                }
            }
        }
        return true;
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

    void collectSchemas() {
        SchemaCollector collector(
            stage_, [this](const UsdPrim& prim) { return worldPrimOf(prim); }, errors_,
            [this](std::string text) { note(std::move(text)); });
        collector.collect(m_.world.schemaInstances);
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
    registerSchemaLibraries();
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
        w.colliders.hull.insert(w.colliders.hull.begin(), kNoHull);
        addGroundPrim(model);
    }
    const UsdPrim root = stage->GetDefaultPrim();
    model.name =
        root ? root.GetName().GetString()
             : (model.scene.prims.empty() ? std::string("stage") : model.scene.prims[0].name);
    return model;
}

}  // namespace lagrange::stage
