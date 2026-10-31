#include "lagrange/humans/Body.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>

namespace lagrange::humans {

namespace {

Pose pitched(const std::array<double, 3>& at, double angle) {
    return {at, {std::cos(angle / 2.0), 0.0, std::sin(angle / 2.0), 0.0}};
}

/// One capsule of the body, in its segment's frame: along z unless turned by `pitch`.
struct Shape {
    double radius;
    double length;  // of the cylinder between the end caps
    Pose pose;
    int colour;
};

constexpr std::array<std::array<float, 4>, 4> kColours{{
    {0.72F, 0.55F, 0.45F, 1.0F},  // skin
    {0.24F, 0.36F, 0.55F, 1.0F},  // shirt
    {0.16F, 0.17F, 0.22F, 1.0F},  // trousers
    {0.08F, 0.08F, 0.09F, 1.0F},  // shoes
}};

/// Radii are illustrative (no sensor resolves them more finely than a few centimetres).
std::array<Shape, kSegments> shapes(const Proportions& b) {
    const double h = b.height;
    const auto along = [](double r, double from, double to, int colour) {
        // A capsule from z = from to z = to, end caps included.
        const double length = std::max(std::abs(to - from) - 2.0 * r, 0.001);
        return Shape{r, length, Pose{{0.0, 0.0, (from + to) / 2.0}, {1.0, 0.0, 0.0, 0.0}}, colour};
    };
    std::array<Shape, kSegments> s{};
    s[kPelvis] = along(0.09 * h / 1.75, -0.08 * h / 1.75, 0.10 * h / 1.75, 2);
    s[kTorso] = along(0.125 * h / 1.75, 0.08 * h / 1.75, b.shoulder + 0.03 * h / 1.75, 1);
    s[kHead] = along(0.095 * h / 1.75, 0.0, 0.21 * h / 1.75, 0);
    s[kUpperArmLeft] = s[kUpperArmRight] = along(0.04 * h / 1.75, -b.upperArm, 0.02, 1);
    s[kForearmLeft] = s[kForearmRight] = along(0.033 * h / 1.75, -b.forearm, 0.0, 0);
    s[kThighLeft] = s[kThighRight] = along(0.07 * h / 1.75, -b.thigh, 0.03, 2);
    s[kShinLeft] = s[kShinRight] = along(0.05 * h / 1.75, -b.shin, 0.0, 2);
    // A foot along x, sole at -ankle: the capsule's radius is the ankle height.
    const Shape foot{b.ankle, std::max(b.foot - 2.0 * b.ankle, 0.001),
                     pitched({b.foot / 2.0 - b.heel, 0.0, 0.0}, std::numbers::pi / 2.0), 3};
    s[kFootLeft] = s[kFootRight] = foot;
    return s;
}

/// A capsule along z, centred at the origin: a cylinder and two spheres.
TriMesh capsuleMesh(double radius, double length) {
    TriMesh out;
    const auto add = [&out](const TriMesh& part, double z) {
        const auto base = static_cast<std::uint32_t>(out.positions.size() / 3);
        for (std::size_t i = 0; i < part.positions.size(); i += 3) {
            out.positions.insert(out.positions.end(),
                                 {part.positions[i], part.positions[i + 1],
                                  part.positions[i + 2] + static_cast<float>(z)});
        }
        out.normals.insert(out.normals.end(), part.normals.begin(), part.normals.end());
        for (const std::uint32_t index : part.indices) {
            out.indices.push_back(base + index);
        }
    };
    add(cylinderMesh(radius, length, 12), 0.0);
    const TriMesh cap = sphereMesh(radius, 12, 8);
    add(cap, length / 2.0);
    add(cap, -length / 2.0);
    return out;
}

}  // namespace

Proportions::Proportions(double h)
    : height(h)
    , hip(0.530 * h)
    , thigh(0.245 * h)
    , shin(0.246 * h)
    , ankle(0.039 * h)
    , foot(0.152 * h)
    , heel(0.038 * h)
    , hipSide(0.0955 * h)
    , shoulder(0.288 * h)
    , shoulderSide(0.1295 * h)
    , upperArm(0.186 * h)
    , forearm(0.254 * h) {}

std::array<Pose, kSegments> segmentPoses(const Proportions& b, const Pose& pelvis,
                                         const Posture& p) {
    std::array<Pose, kSegments> out;
    out[kPelvis] = pelvis;
    out[kTorso] = compose(pelvis, pitched({0.0, 0.0, 0.0}, p.lean));
    out[kHead] = compose(out[kTorso], Pose{{0.0, 0.0, 0.340 * b.height}, {1.0, 0.0, 0.0, 0.0}});
    const auto arm = [&](Segment upper, Segment fore, double side, double shoulder, double elbow) {
        // Flexion swings the limb forward: a rotation about −y of the hanging limb.
        out[upper] = compose(out[kTorso], pitched({0.0, side, b.shoulder}, -shoulder));
        out[fore] = compose(out[upper], pitched({0.0, 0.0, -b.upperArm}, -elbow));
    };
    arm(kUpperArmLeft, kForearmLeft, b.shoulderSide, p.shoulderLeft, p.elbowLeft);
    arm(kUpperArmRight, kForearmRight, -b.shoulderSide, p.shoulderRight, p.elbowRight);
    const auto leg = [&](Segment thigh, Segment shin, Segment foot, double side, double hip,
                         double knee, double ankle) {
        out[thigh] = compose(pelvis, pitched({0.0, side, 0.0}, -hip));
        out[shin] = compose(out[thigh], pitched({0.0, 0.0, -b.thigh}, knee));
        out[foot] = compose(out[shin], pitched({0.0, 0.0, -b.shin}, -ankle));
    };
    leg(kThighLeft, kShinLeft, kFootLeft, b.hipSide, p.hipLeft, p.kneeLeft, p.ankleLeft);
    leg(kThighRight, kShinRight, kFootRight, -b.hipSide, p.hipRight, p.kneeRight, p.ankleRight);
    return out;
}

Pose standingPelvis(const Proportions& b, const Pose& ground) {
    return compose(ground, Pose{{0.0, 0.0, b.hip}, {1.0, 0.0, 0.0, 0.0}});
}

std::vector<Spec> takeSpecs(Model& model) {
    std::vector<Spec> out;
    std::vector<SchemaInstance>& instances = model.world.schemaInstances;
    const World::Prims& prims = model.world.prims;
    for (const SchemaInstance& s : instances) {
        if (s.schema != "HumanAPI_1") {
            continue;
        }
        Spec spec;
        spec.name = prims.instanceNames.at(s.prim.index);
        spec.height = s.real("human:height").value_or(spec.height);
        spec.walkSpeed = s.real("human:walkSpeed").value_or(spec.walkSpeed);
        spec.behavior = std::string(s.token("human:behavior").value_or(spec.behavior));
        const std::span<const double> waypoints = s.realArray("human:waypoints");
        spec.waypoints.assign(waypoints.begin(), waypoints.end());
        spec.loop = s.integer("human:loop").value_or(1) != 0;
        spec.dwell = s.real("human:dwell").value_or(spec.dwell);
        spec.wanderRadius = s.real("human:wanderRadius").value_or(spec.wanderRadius);
        spec.seed = s.integer("human:seed").value_or(spec.seed);
        spec.start = prims.poseInBody.at(s.prim.index);
        out.push_back(std::move(spec));
    }
    std::erase_if(instances, [](const SchemaInstance& s) { return s.schema == "HumanAPI_1"; });
    return out;
}

std::string segmentBody(const std::string& person, std::size_t segment) {
    return "human_" + person + "_" + kSegmentNames.at(segment);
}

void addHuman(Model& model, Spec spec) {
    World& w = model.world;
    const auto taken = [&model](const std::string& name) {
        return std::ranges::any_of(model.scene.prims, [&name](const ScenePrim& prim) {
            return prim.parent == kNoIndex && prim.name == name;
        });
    };
    // The bare prim a stage left for this person, if any, becomes the person's top prim.
    std::uint32_t topPrim = kNoIndex;
    const std::vector<ScenePrim>& existing = model.scene.prims;
    for (std::uint32_t p = 0; p < existing.size(); ++p) {
        if (existing[p].parent == kNoIndex && existing[p].name == spec.name &&
            existing[p].body == kNoIndex && existing[p].mesh == kNoIndex &&
            std::ranges::none_of(existing, [p](const ScenePrim& c) { return c.parent == p; })) {
            topPrim = p;
        }
    }
    const std::string base = spec.name;
    for (int n = 2; topPrim == kNoIndex && taken(spec.name); ++n) {
        spec.name = base + "_" + std::to_string(n);
    }
    const Proportions body(spec.height);
    const auto robot = static_cast<std::uint32_t>(w.robots.size());
    w.robots.push_back(spec.name);
    ScenePrim top;
    top.name = spec.name;
    top.path = "/" + spec.name;
    top.typeName = "Xform";
    if (topPrim == kNoIndex) {
        model.scene.prims.push_back(top);
        topPrim = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
    } else {
        top.path = model.scene.prims[topPrim].path;
    }
    std::array<std::uint32_t, kColours.size()> materials{};
    for (std::size_t c = 0; c < kColours.size(); ++c) {
        model.scene.materials.push_back({spec.name + "_" + std::to_string(c), kColours.at(c)});
        materials.at(c) = static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    }
    const std::array<Shape, kSegments> parts = shapes(body);
    const std::array<Pose, kSegments> poses =
        segmentPoses(body, standingPelvis(body, spec.start), Posture{});
    constexpr double kNominalMassKg = 5.0;  // kinematic bodies are not moved by forces
    constexpr double kFriction = 0.8;
    for (std::size_t s = 0; s < kSegments; ++s) {
        const Shape& part = parts.at(s);
        const BodyId id = w.addBody(segmentBody(spec.name, s), kWorldBody, poses.at(s),
                                    kNominalMassKg, {}, {1, 1, 1, 0, 0, 0});
        w.bodies.robot.back() = robot;
        w.bodies.kinematic.back() = true;
        w.addCollider(id, ShapeType::Capsule, {part.radius, part.length / 2.0, 0.0}, part.pose,
                      kFriction);
        model.bodyRestPose.push_back(poses.at(s));
        model.scene.meshes.push_back(capsuleMesh(part.radius, part.length));
        ScenePrim prim;
        prim.name = kSegmentNames.at(s);
        prim.path = top.path + "/" + prim.name;
        prim.parent = topPrim;
        prim.typeName = "Mesh";
        prim.mesh = static_cast<std::uint32_t>(model.scene.meshes.size() - 1);
        prim.material = materials.at(static_cast<std::size_t>(part.colour));
        prim.body = id.index;
        prim.offset = part.pose;
        model.scene.prims.push_back(prim);
        model.bodyPrim.push_back(static_cast<std::uint32_t>(model.scene.prims.size() - 1));
    }
    SchemaInstance instance;
    instance.schema = "HumanAPI_1";
    instance.prim = w.addPrim("/" + spec.name, spec.name);
    instance.reals["human:height"] = spec.height;
    instance.reals["human:walkSpeed"] = spec.walkSpeed;
    instance.reals["human:dwell"] = spec.dwell;
    instance.reals["human:wanderRadius"] = spec.wanderRadius;
    instance.tokens["human:behavior"] = spec.behavior;
    instance.realArrays["human:waypoints"] = spec.waypoints;
    instance.integers["human:loop"] = spec.loop ? 1 : 0;
    instance.integers["human:seed"] = spec.seed;
    w.schemaInstances.push_back(std::move(instance));
}

}  // namespace lagrange::humans
