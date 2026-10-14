// The stage reader and writer: a model survives the round trip, hand-written USD reads as the
// rules say, and stages that cannot be simulated are refused with the prim named.
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::stage {
namespace {

std::filesystem::path scratch(const std::string& name) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_stage";
    std::filesystem::create_directories(dir);
    return dir / name;
}

std::filesystem::path writeText(const std::string& name, const std::string& text) {
    const std::filesystem::path path = scratch(name);
    std::ofstream(path) << text;
    return path;
}

std::array<double, 4> aboutZ(double angle) {
    return {std::cos(angle / 2), 0.0, 0.0, std::sin(angle / 2)};
}

std::size_t bodyNamed(const World& world, const std::string& name) {
    for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
        if (world.bodies.names[b] == name) {
            return b;
        }
    }
    ADD_FAILURE() << "no body " << name;
    return 0;
}

/// A small robot with one of everything the loader handles.
Model sampleModel() {
    Model m;
    m.name = "rover";
    World& w = m.world;
    const auto prim = [&m](const std::string& name, std::uint32_t parent, const std::string& type,
                           std::uint32_t body) {
        ScenePrim p;
        p.name = name;
        p.parent = parent;
        p.path = (parent == kNoIndex ? "" : m.scene.prims[parent].path) + "/" + name;
        p.typeName = type;
        p.body = body;
        m.scene.prims.push_back(p);
        return static_cast<std::uint32_t>(m.scene.prims.size() - 1);
    };
    // Base: free, with a full (non-diagonal) inertia tensor and an offset centre of mass.
    const Pose basePose{{1.0, 2.0, 0.15}, aboutZ(0.3)};
    const BodyId base = w.addBody("base", kWorldBody, basePose, 12.0, {0.01, 0.0, 0.03},
                                  {0.30, 0.50, 0.70, 0.02, -0.01, 0.005});
    w.addJoint("base_floating", JointType::Free, base, {0, 0, 1}, false);
    m.bodyRestPose.push_back(basePose);
    // Wheel: continuous, driven, axis y.
    const Pose wheelIn{{0.0, 0.25, -0.05}, {1, 0, 0, 0}};
    const BodyId wheel = w.addBody("wheel", base, wheelIn, 1.5, {}, {0.01, 0.02, 0.01, 0, 0, 0});
    w.addJoint("wheel_joint", JointType::Continuous, wheel, {0, 1, 0}, true);
    w.joints.effortLimit.back() = 12.0;
    m.actuatedJoints.push_back("wheel_joint");
    m.bodyRestPose.push_back(compose(basePose, wheelIn));
    // Lift: prismatic along an oblique axis, limited.
    const Pose liftIn{{0.1, 0.0, 0.2}, aboutZ(0.5)};
    const BodyId lift = w.addBody("lift", base, liftIn, 2.0, {}, {0.01, 0.01, 0.01, 0, 0, 0});
    w.addJoint("lift_joint", JointType::Prismatic, lift, {0.6, 0.0, 0.8}, false);
    w.joints.lowerLimit.back() = -0.1;
    w.joints.upperLimit.back() = 0.3;
    m.bodyRestPose.push_back(compose(basePose, liftIn));
    // Mast: welded to the base.
    const Pose mastIn{{-0.1, 0.0, 0.3}, {1, 0, 0, 0}};
    const BodyId mast = w.addBody("mast", base, mastIn, 0.5, {}, {0.001, 0.001, 0.001, 0, 0, 0});
    m.bodyRestPose.push_back(compose(basePose, mastIn));

    w.addCollider(base, ShapeType::Box, {0.3, 0.2, 0.1}, Pose{{0, 0, 0.02}, aboutZ(0.1)}, 0.8);
    w.addCollider(wheel, ShapeType::Cylinder, {0.1, 0.025, 0},
                  Pose{{0, 0, 0}, {std::numbers::sqrt2 / 2, std::numbers::sqrt2 / 2, 0, 0}}, 1.0);
    w.addCollider(lift, ShapeType::Sphere, {0.05, 0, 0}, Pose{}, 1.0);
    w.addCollider(mast, ShapeType::Capsule, {0.02, 0.1, 0}, Pose{{0, 0, 0.05}, {1, 0, 0, 0}}, 1.0);
    w.addCollider(kWorldBody, ShapeType::Box, {1.0, 0.1, 0.5}, Pose{{3, 0, 0.5}, {1, 0, 0, 0}},
                  1.0);

    const std::uint32_t top = prim("rover", kNoIndex, "Xform", base.index);
    const std::uint32_t basePrim = prim("base", top, "Xform", base.index);
    const std::uint32_t wheelPrim = prim("wheel", basePrim, "Xform", wheel.index);
    const std::uint32_t liftPrim = prim("lift", basePrim, "Xform", lift.index);
    const std::uint32_t mastPrim = prim("mast", basePrim, "Xform", mast.index);
    m.bodyPrim = {basePrim, wheelPrim, liftPrim, mastPrim};
    m.jointPrim = {basePrim, wheelPrim, liftPrim};
    m.scene.meshes.push_back(boxMesh(0.6, 0.4, 0.2));
    m.scene.materials.push_back({"paint", {0.2F, 0.4F, 0.6F, 1.0F}});
    const std::uint32_t visual = prim("visual", basePrim, "Mesh", base.index);
    m.scene.prims[visual].mesh = 0;
    m.scene.prims[visual].material = 0;
    m.scene.prims[visual].offset = Pose{{0, 0, 0.02}, {1, 0, 0, 0}};

    const Pose lidarMount{{0.0, 0.0, 0.1}, aboutZ(0.2)};
    w.addSensor("top_lidar", SensorType::Lidar, mast, lidarMount, 10.0,
                {{"samples", 900.0},
                 {"min_angle", -3.0},
                 {"max_angle", 3.0},
                 {"vertical_samples", 16.0},
                 {"vertical_min_angle", -0.25},
                 {"vertical_max_angle", 0.25},
                 {"min_range", 0.3},
                 {"max_range", 40.0},
                 {"range_noise", 0.02}});
    const std::uint32_t lidarPrim = prim("top_lidar", mastPrim, "Lidar", mast.index);
    m.scene.prims[lidarPrim].offset = lidarMount;
    w.addSensor("imu", SensorType::Imu, base, Pose{}, 200.0,
                {{"accel_noise", 0.02}, {"gyro_noise", 0.002}});
    const std::uint32_t imuPrim = prim("imu", basePrim, "Imu", base.index);
    m.sensorPrim = {lidarPrim, imuPrim};
    w.colliders.body.insert(w.colliders.body.begin(), kWorldBody);  // the ground, as loaders add
    w.colliders.shape.insert(w.colliders.shape.begin(), ShapeType::Plane);
    w.colliders.size.insert(w.colliders.size.begin(), {0, 0, 1});
    w.colliders.poseInBody.insert(w.colliders.poseInBody.begin(), Pose{});
    w.colliders.friction.insert(w.colliders.friction.begin(), 1.0);
    addGroundPrim(m);
    return m;
}

void expectPose(const Pose& a, const Pose& b, double tolerance, const std::string& what) {
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(a.position.at(k), b.position.at(k), tolerance) << what;
    }
    // q and -q are the same rotation.
    double dot = 0.0;
    for (std::size_t k = 0; k < 4; ++k) {
        dot += a.orientation.at(k) * b.orientation.at(k);
    }
    EXPECT_NEAR(std::abs(dot), 1.0, tolerance) << what;
}

TEST(StageRoundTrip, AModelReadsBackAsItWasWritten) {
    const Model original = sampleModel();
    const std::filesystem::path path = scratch("rover.usda");
    auto written = writeStage(original, path.string(), {.doc = "test"});
    ASSERT_TRUE(written) << written.error();
    auto read = loadStage(path.string());
    ASSERT_TRUE(read) << read.error();
    const Model& m = *read;
    const World& a = original.world;
    const World& b = m.world;
    EXPECT_EQ(m.name, "rover");
    // float attributes in UsdPhysics: compare to float precision.
    constexpr double kFloat = 1e-5;

    ASSERT_EQ(b.bodies.names.size(), a.bodies.names.size());
    for (std::size_t i = 0; i < a.bodies.names.size(); ++i) {
        const std::size_t j = bodyNamed(b, a.bodies.names[i]);
        const std::string name = a.bodies.names[i];
        EXPECT_NEAR(b.bodies.mass[j], a.bodies.mass[i], kFloat) << name;
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(b.bodies.comLocal[j][k], a.bodies.comLocal[i][k], kFloat) << name;
        }
        for (std::size_t k = 0; k < 6; ++k) {
            EXPECT_NEAR(b.bodies.inertiaLocal[j][k], a.bodies.inertiaLocal[i][k], kFloat) << name;
        }
        expectPose(m.bodyRestPose[j], original.bodyRestPose[i], kFloat, name + " rest pose");
        const BodyId pa = a.bodies.parent[i];
        const BodyId pb = b.bodies.parent[j];
        ASSERT_EQ(pa == kWorldBody, pb == kWorldBody) << name;
        if (pa != kWorldBody) {
            EXPECT_EQ(b.bodies.names[pb.index], a.bodies.names[pa.index]) << name;
        }
    }

    ASSERT_EQ(b.joints.names.size(), a.joints.names.size());
    for (std::size_t i = 0; i < a.joints.names.size(); ++i) {
        std::size_t j = 0;
        while (j < b.joints.names.size() && b.joints.names[j] != a.joints.names[i]) {
            ++j;
        }
        ASSERT_LT(j, b.joints.names.size()) << a.joints.names[i];
        const std::string name = a.joints.names[i];
        EXPECT_EQ(b.joints.type[j], a.joints.type[i]) << name;
        EXPECT_EQ(b.joints.actuated[j], a.joints.actuated[i]) << name;
        if (a.joints.type[i] != JointType::Free) {
            const auto& x = a.joints.axis[i];
            const double n = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
            for (std::size_t k = 0; k < 3; ++k) {
                EXPECT_NEAR(b.joints.axis[j][k], x[k] / n, kFloat) << name;
            }
        }
        EXPECT_NEAR(b.joints.lowerLimit[j], a.joints.lowerLimit[i], kFloat) << name;
        EXPECT_NEAR(b.joints.upperLimit[j], a.joints.upperLimit[i], kFloat) << name;
        if (a.joints.actuated[i]) {
            EXPECT_NEAR(b.joints.effortLimit[j], a.joints.effortLimit[i], kFloat) << name;
        }
    }
    EXPECT_EQ(m.actuatedJoints, original.actuatedJoints);

    ASSERT_EQ(b.colliders.body.size(), a.colliders.body.size());
    for (std::size_t c = 0; c < a.colliders.body.size(); ++c) {
        // Same order: the ground first, then in prim order, which follows the bodies.
        std::size_t d = 0;
        bool found = false;
        for (; d < b.colliders.body.size(); ++d) {
            const bool sameOwner =
                (a.colliders.body[c] == kWorldBody && b.colliders.body[d] == kWorldBody) ||
                (a.colliders.body[c] != kWorldBody && b.colliders.body[d] != kWorldBody &&
                 a.bodies.names[a.colliders.body[c].index] ==
                     b.bodies.names[b.colliders.body[d].index]);
            if (sameOwner && a.colliders.shape[c] == b.colliders.shape[d]) {
                found = true;
                break;
            }
        }
        ASSERT_TRUE(found) << "collider " << c;
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(b.colliders.size[d][k], a.colliders.size[c][k], kFloat) << "collider " << c;
        }
        expectPose(b.colliders.poseInBody[d], a.colliders.poseInBody[c], 1e-9,
                   "collider " + std::to_string(c));
        EXPECT_NEAR(b.colliders.friction[d], a.colliders.friction[c], kFloat) << "collider " << c;
    }

    ASSERT_EQ(b.sensors.names.size(), 2U);
    for (std::size_t s = 0; s < 2; ++s) {
        EXPECT_EQ(b.sensors.names[s], a.sensors.names[s]);
        EXPECT_EQ(b.sensors.type[s], a.sensors.type[s]);
        EXPECT_EQ(b.bodies.names[b.sensors.body[s].index], a.bodies.names[a.sensors.body[s].index]);
        EXPECT_DOUBLE_EQ(b.sensors.rateHz[s], a.sensors.rateHz[s]);
        expectPose(b.sensors.poseInBody[s], a.sensors.poseInBody[s], 1e-9, b.sensors.names[s]);
        EXPECT_EQ(b.sensors.settings[s], a.sensors.settings[s]) << b.sensors.names[s];
    }

    // The scene: same paths, types and bodies; the visual keeps its colour.
    std::vector<std::string> pathsA, pathsB;
    for (const ScenePrim& p : original.scene.prims) {
        pathsA.push_back(p.path + " " + p.typeName);
    }
    for (const ScenePrim& p : m.scene.prims) {
        pathsB.push_back(p.path + " " + p.typeName);
    }
    // USD lists prims depth first, the model in creation order: compare as sets.
    std::sort(pathsA.begin(), pathsA.end());
    std::sort(pathsB.begin(), pathsB.end());
    EXPECT_EQ(pathsB, pathsA);
    for (const ScenePrim& p : m.scene.prims) {
        if (p.name == "visual") {
            EXPECT_EQ(m.scene.materials[p.material].rgba, original.scene.materials[0].rgba);
            EXPECT_EQ(m.scene.meshes[p.mesh].indices.size(),
                      original.scene.meshes[0].indices.size());
        }
        if (p.name == "rover") {
            EXPECT_EQ(p.body, 0U) << "the articulation's top prim moves with the root body";
        }
    }
    EXPECT_TRUE(m.notes.empty()) << m.notes.front();
}

TEST(StageReader, ReadsHandWrittenUsd) {
    // A Y-axis cylinder wheel on a revolute joint with limits in degrees, a fixed-base arm link,
    // a friction material, displayColor and a scaled cube, as a user would write them.
    const std::filesystem::path path = writeText("hand.usda", R"(#usda 1.0
(
    defaultPrim = "bot"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "bot" ( prepend apiSchemas = ["PhysicsArticulationRootAPI"] )
{
    def Xform "base" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 5
        double3 xformOp:translate = (0, 0, 0.5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "box" ( prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"] )
        {
            double size = 1
            float3 xformOp:scale = (0.4, 0.2, 0.1)
            uniform token[] xformOpOrder = ["xformOp:scale"]
            rel material:binding:physics = </bot/rubber>
        }
        def Cube "look"
        {
            double size = 2
            color3f[] primvars:displayColor = [(1, 0, 0)]
        }
    }
    def Xform "arm" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 1
        double3 xformOp:translate = (0.3, 0, 0.5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cylinder "link" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )
        {
            double radius = 0.05
            double height = 0.4
            uniform token axis = "Y"
        }
    }
    def PhysicsFixedJoint "anchor"
    {
        rel physics:body1 = </bot/base>
    }
    def PhysicsRevoluteJoint "elbow" ( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )
    {
        rel physics:body0 = </bot/base>
        rel physics:body1 = </bot/arm>
        uniform token physics:axis = "Y"
        float physics:lowerLimit = -90
        float physics:upperLimit = 45
    }
    def Material "rubber" ( prepend apiSchemas = ["PhysicsMaterialAPI"] )
    {
        float physics:dynamicFriction = 0.9
    }
}
)");
    auto read = loadStage(path.string());
    ASSERT_TRUE(read) << read.error();
    const World& w = read->world;
    ASSERT_EQ(w.bodies.names.size(), 2U);
    const std::size_t base = bodyNamed(w, "base");
    const std::size_t arm = bodyNamed(w, "arm");
    EXPECT_EQ(w.bodies.parent[base], kWorldBody);
    EXPECT_EQ(w.bodies.parent[arm].index, base);
    ASSERT_EQ(w.joints.names.size(), 1U) << "the fixed joint welds base to the world";
    EXPECT_EQ(w.joints.names[0], "elbow");
    EXPECT_EQ(w.joints.type[0], JointType::Revolute);
    EXPECT_TRUE(w.joints.actuated[0]);
    EXPECT_NEAR(w.joints.lowerLimit[0], -std::numbers::pi / 2, 1e-6);
    EXPECT_NEAR(w.joints.upperLimit[0], std::numbers::pi / 4, 1e-6);
    EXPECT_NEAR(w.joints.axis[0][1], 1.0, 1e-12);
    expectPose(w.bodies.poseInParent[arm], Pose{{0.3, 0, 0}, {1, 0, 0, 0}}, 1e-12, "arm");
    // Colliders: the ground plane added first, the scaled box with its friction, the Y cylinder
    // turned onto the y axis.
    ASSERT_EQ(w.colliders.body.size(), 3U);
    EXPECT_EQ(w.colliders.shape[0], ShapeType::Plane);
    EXPECT_EQ(w.colliders.shape[1], ShapeType::Box);
    EXPECT_NEAR(w.colliders.size[1][0], 0.2, 1e-7);
    EXPECT_NEAR(w.colliders.size[1][1], 0.1, 1e-7);
    EXPECT_NEAR(w.colliders.friction[1], 0.9, 1e-6);
    EXPECT_EQ(w.colliders.shape[2], ShapeType::Cylinder);
    EXPECT_NEAR(w.colliders.size[2][1], 0.2, 1e-12);
    const std::array<double, 3> along =
        rotate(w.colliders.poseInBody[2].orientation, {0.0, 0.0, 1.0});
    EXPECT_NEAR(std::abs(along[1]), 1.0, 1e-12) << "the cylinder lies along y";
    // The visible cube is drawn red; the colliders are not drawn.
    bool red = false;
    for (const ScenePrim& p : read->scene.prims) {
        EXPECT_NE(p.name, "box");
        EXPECT_NE(p.name, "link");
        if (p.name == "look") {
            red = read->scene.materials[p.material].rgba[0] == 1.0F;
        }
    }
    EXPECT_TRUE(red);
}

TEST(StageReader, RefusesWhatCannotBeSimulated) {
    const auto load = [](const std::string& name, const std::string& body) {
        return loadStage(
            writeText(name, "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n" + body)
                .string());
    };
    auto yUp = loadStage(writeText("yup.usda", "#usda 1.0\n(\n    upAxis = \"Y\"\n)\n").string());
    ASSERT_FALSE(yUp);
    EXPECT_NE(yUp.error().find("Z-up"), std::string::npos);

    auto anchor = load("anchor.usda", R"(
def Xform "a" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
{
    float physics:mass = 1
}
def Xform "b" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
{
    float physics:mass = 1
}
def PhysicsRevoluteJoint "hinge" {
    rel physics:body0 = </a>
    rel physics:body1 = </b>
    point3f physics:localPos1 = (0, 0, 0.1)
}
)");
    ASSERT_FALSE(anchor);
    EXPECT_NE(anchor.error().find("/hinge"), std::string::npos) << anchor.error();

    auto rate = load("rate.usda", R"(
def Xform "a" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
{
    float physics:mass = 1
    def Xform "imu" ( prepend apiSchemas = ["SensorAPI_1", "ImuAPI_1"] )
    {
    }
}
)");
    ASSERT_FALSE(rate);
    EXPECT_NE(rate.error().find("sensor:rate"), std::string::npos) << rate.error();
}

}  // namespace
}  // namespace lagrange::stage
