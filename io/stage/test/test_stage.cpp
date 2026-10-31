// The stage reader and writer: a model survives the round trip, hand-written USD reads as the
// rules say, and stages that cannot be simulated are refused with the prim named.
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
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
    m.scene.materials.push_back(
        {"paint", {0.2F, 0.4F, 0.6F, 1.0F}, SurfaceLook{0.25F, 0.15F, {0.5F, 0.0F, 0.0F}}});
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
    w.colliders.hull.insert(w.colliders.hull.begin(), kNoHull);
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
            ASSERT_TRUE(m.scene.materials[p.material].look.has_value()) << "the file gave one";
            EXPECT_EQ(*m.scene.materials[p.material].look, *original.scene.materials[0].look);
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

/// A wheel on a driven joint, its motor, a battery with a port, a bus and a load: the sample
/// AMR's power system in miniature (formats.md §4).
const char* const kPowered = R"(#usda 1.0
(
    defaultPrim = "bot"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "bot"
{
    def Xform "base" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 10
    }
    def Xform "wheel" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 1
    }
    def PhysicsRevoluteJoint "wheel_joint" ( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )
    {
        rel physics:body0 = </bot/base>
        rel physics:body1 = </bot/wheel>
        uniform token physics:axis = "Y"
    }
    def Xform "power"
    {
        def Xform "battery" ( prepend apiSchemas = ["BatteryAPI_1", "PortAPI_1:dc_out", "RadiationAPI_1"] )
        {
            uniform token elec:battery:chemistry = "lfp"
            int elec:battery:seriesCells = 16
            double elec:battery:capacity = 144000
            double[] elec:battery:ocvSoc = [0, 0.5, 1]
            double[] elec:battery:ocvVoltage = [2.8, 3.28, 3.45]
            double elec:battery:r0 = 0.0015
            double elec:battery:cutoffVoltage = 44.8
            double elec:battery:maxChargeCurrent = 20
            double elec:port:dc_out:maxCurrent = 80
            rel elec:port:dc_out:connectedTo = </bot/power/dc_48v>
        }
        def Xform "dc_48v" ( prepend apiSchemas = ["ElectricalBusAPI_1"] )
        {
            double elec:bus:capacitance = 0.0047
            double elec:bus:undervoltageTrip = 42
            double elec:bus:overvoltageTrip = 60
        }
        def Xform "computer" ( prepend apiSchemas = ["ElectricalLoadAPI_1"] )
        {
            double elec:load:power = 45
            rel elec:powerPort = </bot/power/dc_48v>
        }
    }
    def Xform "left_drive_motor" ( prepend apiSchemas = ["ElectricalMotorAPI_1"] )
    {
        double elec:windingResistance = 0.12
        double elec:inductance = 0.00031
        double elec:torqueConstant = 0.085
        int elec:polePairs = 4
        double elec:gearRatio = 20
        double elec:maxCurrent = 30
        rel elec:joint = </bot/wheel_joint>
        rel elec:powerPort = </bot/power/dc_48v>
    }
}
)";

const SchemaInstance* only(const World& world, const std::string& schema) {
    const std::vector<const SchemaInstance*> found = WorldView(world).instancesOf(schema);
    EXPECT_EQ(found.size(), 1U) << schema;
    return found.empty() ? nullptr : found[0];
}

TEST(StageReader, CollectsTheElectricalSchemas) {
    auto model = loadStage(writeText("powered.usda", kPowered).string());
    ASSERT_TRUE(model) << model.error();
    const World& w = model->world;
    const WorldView view(w);

    const SchemaInstance* motor = only(w, "ElectricalMotorAPI_1");
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(view.signalInstanceName(motor->prim), "left_drive_motor");
    EXPECT_EQ(motor->real("elec:torqueConstant"), 0.085);
    EXPECT_EQ(motor->integer("elec:polePairs"), 4);
    EXPECT_EQ(motor->real("elec:gearRatio"), 20.0);
    EXPECT_EQ(motor->real("elec:resistanceTempCoeff"), 0.00393);  // the schema's fallback
    EXPECT_EQ(motor->token("elec:motorType"), "pmsm");
    EXPECT_FALSE(motor->real("elec:inductanceQ"));  // no fallback, not authored
    // elec:joint resolves to the world joint, so the network finds its signals by name.
    ASSERT_EQ(motor->rel("elec:joint").size(), 1U);
    EXPECT_EQ(motor->rel("elec:joint")[0], w.joints.prim[1]);
    EXPECT_EQ(view.signalInstanceName(motor->rel("elec:joint")[0]), "wheel_joint");

    const SchemaInstance* bus = only(w, "ElectricalBusAPI_1");
    ASSERT_NE(bus, nullptr);
    EXPECT_EQ(view.signalInstanceName(bus->prim), "dc_48v");
    EXPECT_EQ(view.primPath(bus->prim), "/bot/power/dc_48v");
    EXPECT_EQ(motor->rel("elec:powerPort")[0], bus->prim);

    const SchemaInstance* battery = only(w, "BatteryAPI_1");
    ASSERT_NE(battery, nullptr);
    EXPECT_EQ(battery->realArray("elec:battery:ocvVoltage").size(), 3U);
    EXPECT_EQ(battery->integer("elec:battery:parallelCells"), 1);
    EXPECT_EQ(battery->token("elec:battery:chemistry"), "lfp");

    // A multiple-apply port keeps its instance name and namespaced properties.
    const SchemaInstance* port = only(w, "PortAPI_1");
    ASSERT_NE(port, nullptr);
    EXPECT_EQ(port->instance, "dc_out");
    EXPECT_EQ(port->prim, battery->prim);
    EXPECT_EQ(port->real("elec:port:dc_out:maxCurrent"), 80.0);
    EXPECT_EQ(port->token("elec:port:dc_out:kind"), "dc");
    ASSERT_EQ(port->rel("elec:port:dc_out:connectedTo").size(), 1U);
    EXPECT_EQ(port->rel("elec:port:dc_out:connectedTo")[0], bus->prim);

    EXPECT_EQ(only(w, "ElectricalLoadAPI_1")->real("elec:load:power"), 45.0);
    // RadiationAPI_1 is not defined until the radiation library exists: noted, not an error.
    EXPECT_TRUE(std::any_of(model->notes.begin(), model->notes.end(), [](const std::string& n) {
        return n.find("RadiationAPI_1") != std::string::npos;
    }));
}

TEST(StageReader, SchemaInstancesSurviveAppendingModels) {
    auto first = loadStage(writeText("powered.usda", kPowered).string());
    auto second = loadStage(writeText("powered.usda", kPowered).string());
    ASSERT_TRUE(first && second);
    appendModel(*first, *second);
    const World& w = first->world;
    const WorldView view(w);
    const std::vector<const SchemaInstance*> motors = view.instancesOf("ElectricalMotorAPI_1");
    ASSERT_EQ(motors.size(), 2U);
    // The second motor drives the second robot's wheel joint and draws from its own bus.
    EXPECT_EQ(motors[1]->rel("elec:joint")[0], w.joints.prim[3]);
    const std::vector<const SchemaInstance*> buses = view.instancesOf("ElectricalBusAPI_1");
    ASSERT_EQ(buses.size(), 2U);
    EXPECT_NE(buses[0]->prim, buses[1]->prim);
    EXPECT_EQ(motors[1]->rel("elec:powerPort")[0], buses[1]->prim);
    EXPECT_EQ(view.signalInstanceName(motors[1]->prim), "left_drive_motor");
}

TEST(StageReader, RefusesBadSchemaValuesNamingThePrims) {
    const auto with = [](std::string text, const std::string& from, const std::string& to) {
        text.replace(text.find(from), from.size(), to);
        return text;
    };
    std::string bad = kPowered;
    bad = with(bad, "\"lfp\"", "\"nickel_iron\"");
    bad = with(bad, "[2.8, 3.28, 3.45]", "[2.8, 3.45]");
    bad = with(bad, "double elec:gearRatio = 20",
               "double elec:gearRatio = 20\n        double elec:gearEfficiency = 1.2");
    bad = with(bad, "rel elec:joint = </bot/wheel_joint>", "rel elec:joint = </bot/wheel>");
    bad = with(bad, "rel elec:powerPort = </bot/power/dc_48v>\n    }\n}",
               "rel elec:powerPort = </bot/power/dc_12v>\n    }\n}");
    auto model = loadStage(writeText("bad_power.usda", bad).string());
    ASSERT_FALSE(model);
    const std::string& e = model.error();
    // Every problem at once, each with its prim.
    EXPECT_NE(e.find("/bot/power/battery: elec:battery:chemistry is \"nickel_iron\""),
              std::string::npos)
        << e;
    EXPECT_NE(e.find("ocvSoc and elec:battery:ocvVoltage differ in length (3 and 2)"),
              std::string::npos)
        << e;
    EXPECT_NE(e.find("/bot/left_drive_motor: elec:gearEfficiency is 1.2"), std::string::npos) << e;
    EXPECT_NE(e.find("elec:joint targets /bot/wheel, which is not a joint"), std::string::npos)
        << e;
    EXPECT_NE(e.find("targets /bot/power/dc_12v, which is not in the stage"), std::string::npos)
        << e;
}

TEST(StageReader, AnOlderSchemaVersionIsReportedWithItsMigration) {
    // formats.md §10: an older version of a known family loads with a note naming the
    // migration; two versions of one family on one prim is an error.
    const std::string header = "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n";
    auto old = loadStage(writeText("old_schema.usda", header + R"(
def Xform "bot" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
{
    float physics:mass = 1
    def Xform "imu" ( prepend apiSchemas = ["SensorAPI_0", "ImuAPI_1"] )
    {
    }
}
)")
                             .string());
    ASSERT_TRUE(old) << old.error();
    bool noted = false;
    for (const std::string& note : old->notes) {
        noted = noted || (note.find("/bot/imu applies SensorAPI_0, an older version of "
                                    "SensorAPI_1") != std::string::npos &&
                          note.find("migrate_schema") != std::string::npos);
    }
    EXPECT_TRUE(noted);

    auto both = loadStage(writeText("two_versions.usda", header + R"(
def Xform "bot" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
{
    float physics:mass = 1
    def Xform "imu" ( prepend apiSchemas = ["SensorAPI_0", "SensorAPI_1", "ImuAPI_1"] )
    {
        double sensor:rate = 100
    }
}
)")
                              .string());
    ASSERT_FALSE(both);
    EXPECT_NE(both.error().find("/bot/imu applies two versions of SensorAPI"), std::string::npos)
        << both.error();
}

TEST(StageReader, ConvexMeshCollidersBecomeHullsAndSurviveTheRoundTrip) {
    // formats.md §2: collision geometry is convex hulls (what the collision bake writes). A
    // mesh marked convexHull that is convex is used; others are skipped with a note.
    const std::string header = "#usda 1.0\n(\n    metersPerUnit = 1\n    upAxis = \"Z\"\n)\n";
    auto model = loadStage(writeText("hulls.usda", header + R"(
def Xform "shelf"
{
    def Mesh "hull" ( prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"] )
    {
        uniform token physics:approximation = "convexHull"
        point3f[] points = [(-0.5, -0.2, 0), (0.5, -0.2, 0), (-0.5, 0.2, 0), (0.5, 0.2, 0), (-0.5, -0.2, 1), (0.5, -0.2, 1), (-0.5, 0.2, 1), (0.5, 0.2, 1)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5]
        double3 xformOp:translate = (2, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def Mesh "two_boxes" ( prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"] )
    {
        uniform token physics:approximation = "convexHull"
        point3f[] points = [(-0.5, -0.2, 0), (0.5, -0.2, 0), (-0.5, 0.2, 0), (0.5, 0.2, 0), (-0.5, -0.2, 1), (0.5, -0.2, 1), (-0.5, 0.2, 1), (0.5, 0.2, 1), (1.5, -0.2, 0), (2.5, -0.2, 0), (1.5, 0.2, 0), (2.5, 0.2, 0), (1.5, -0.2, 1), (2.5, -0.2, 1), (1.5, 0.2, 1), (2.5, 0.2, 1)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5, 8, 10, 11, 9, 12, 13, 15, 14, 8, 9, 13, 12, 10, 14, 15, 11, 8, 12, 14, 10, 9, 11, 15, 13]
    }
    def Mesh "raw" ( prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"] )
    {
        uniform token physics:approximation = "convexDecomposition"
        point3f[] points = [(-0.5, -0.2, 0), (0.5, -0.2, 0), (-0.5, 0.2, 0), (0.5, 0.2, 0), (-0.5, -0.2, 1), (0.5, -0.2, 1), (-0.5, 0.2, 1), (0.5, 0.2, 1)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5]
    }
}
)")
                               .string());
    ASSERT_TRUE(model) << model.error();
    const World& w = model->world;
    ASSERT_EQ(w.hulls.size(), 1U);
    const auto c = static_cast<std::size_t>(
        std::find(w.colliders.shape.begin(), w.colliders.shape.end(), ShapeType::ConvexHull) -
        w.colliders.shape.begin());
    ASSERT_LT(c, w.colliders.shape.size());
    EXPECT_EQ(w.hulls[0].points.size(), 8U);
    EXPECT_EQ(w.hulls[0].triangles.size(), 12U);  // the quads as fans
    EXPECT_NEAR(w.colliders.size[c][0], 0.5, 1e-6);
    EXPECT_NEAR(w.colliders.size[c][2], 1.0, 1e-6);  // about the shape origin, not the centre
    EXPECT_NEAR(w.colliders.poseInBody[c].position[0], 2.0, 1e-12);
    int notes = 0;
    for (const std::string& note : model->notes) {
        notes += note.find("/shelf/two_boxes") != std::string::npos &&
                 note.find("not convex") != std::string::npos;
        notes += note.find("/shelf/raw") != std::string::npos &&
                 note.find("collision_bake") != std::string::npos;
    }
    EXPECT_EQ(notes, 2);

    const std::filesystem::path out = scratch("hulls_out.usda");
    ASSERT_TRUE(writeStage(*model, out.string()));
    auto back = loadStage(out.string());
    ASSERT_TRUE(back) << back.error();
    ASSERT_EQ(back->world.hulls.size(), 1U);
    EXPECT_EQ(back->world.hulls[0].triangles.size(), 12U);
    for (std::size_t i = 0; i < 8; ++i) {
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(back->world.hulls[0].points[i][k], w.hulls[0].points[i][k], 1e-6);
        }
    }
}

TEST(StageWriter, SchemaInstancesSurviveTheRoundTrip) {
    auto model = loadStage(writeText("powered.usda", kPowered).string());
    ASSERT_TRUE(model) << model.error();
    const std::filesystem::path out = scratch("powered_out.usda");
    ASSERT_TRUE(writeStage(*model, out.string()));
    auto back = loadStage(out.string());
    ASSERT_TRUE(back) << back.error();
    const WorldView a(model->world);
    const WorldView b(back->world);
    ASSERT_EQ(back->world.schemaInstances.size(), model->world.schemaInstances.size());
    for (const SchemaInstance& s : model->world.schemaInstances) {
        const auto match =
            std::find_if(back->world.schemaInstances.begin(), back->world.schemaInstances.end(),
                         [&](const SchemaInstance& t) {
                             return t.schema == s.schema && t.instance == s.instance &&
                                    b.signalInstanceName(t.prim) == a.signalInstanceName(s.prim);
                         });
        ASSERT_NE(match, back->world.schemaInstances.end()) << s.schema;
        EXPECT_EQ(match->reals, s.reals) << s.schema;
        EXPECT_EQ(match->integers, s.integers) << s.schema;
        EXPECT_EQ(match->tokens, s.tokens) << s.schema;
        EXPECT_EQ(match->realArrays, s.realArrays) << s.schema;
        for (const auto& [name, targets] : s.relationships) {
            ASSERT_EQ(match->rel(name).size(), targets.size()) << name;
            for (std::size_t i = 0; i < targets.size(); ++i) {
                EXPECT_EQ(b.signalInstanceName(match->rel(name)[i]),
                          a.signalInstanceName(targets[i]))
                    << name;
            }
        }
    }
    // Fallbacks stay unauthored.
    std::ifstream file(out);
    const std::string text((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
    EXPECT_EQ(text.find("resistanceTempCoeff"), std::string::npos);
    EXPECT_NE(text.find("elec:torqueConstant = 0.085"), std::string::npos);
}

TEST(Overlay, AppliesSchemasByNameAndCreatesPrims) {
    auto model = loadStage(writeText("bare.usda", R"(#usda 1.0
(
    defaultPrim = "bot"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "bot"
{
    def Xform "base" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 10
    }
    def Xform "wheel" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 1
    }
    def PhysicsRevoluteJoint "wheel_joint" ( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )
    {
        rel physics:body0 = </bot/base>
        rel physics:body1 = </bot/wheel>
        uniform token physics:axis = "Y"
    }
}
)")
                               .string());
    ASSERT_TRUE(model) << model.error();
    const std::filesystem::path overlay = writeText("bot.overlay.yaml", R"(schema_version: 1
apply:
  - prim: wheel_joint
    create_child: drive
    schemas:
      ElectricalMotorAPI_1:
        elec:windingResistance: 0.2
        elec:inductance: 0.001
        elec:torqueConstant: 0.05
        elec:polePairs: 2
        elec:maxCurrent: 10
        elec:motorType: dc_brushed
        elec:joint: wheel_joint
        elec:powerPort: power/bus
  - create: power/bus
    schemas:
      ElectricalBusAPI_1: { elec:bus:undervoltageTrip: 9.0, elec:bus:overvoltageTrip: 15.0 }
)");
    ASSERT_TRUE(applyOverlay(*model, overlay.string()));
    const World& w = model->world;
    const WorldView view(w);
    const SchemaInstance* motor = only(w, "ElectricalMotorAPI_1");
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(view.primPath(motor->prim), "/bot/base/drive");  // on the joint's parent link
    EXPECT_EQ(motor->integer("elec:polePairs"), 2);
    EXPECT_EQ(motor->real("elec:gearRatio"), 1.0);  // the fallback
    EXPECT_EQ(motor->token("elec:motorType"), "dc_brushed");
    EXPECT_EQ(view.signalInstanceName(motor->rel("elec:joint")[0]), "wheel_joint");
    const SchemaInstance* bus = only(w, "ElectricalBusAPI_1");
    ASSERT_NE(bus, nullptr);
    EXPECT_EQ(view.primPath(bus->prim), "/bot/power/bus");
    EXPECT_EQ(motor->rel("elec:powerPort")[0], bus->prim);
    // Applying it twice is refused: the motor is there already.
    auto again = applyOverlay(*model, overlay.string());
    ASSERT_FALSE(again);
    EXPECT_NE(again.error().find("already has ElectricalMotorAPI_1"), std::string::npos)
        << again.error();
}

TEST(Overlay, RefusesMistakesTogetherAndLeavesTheModelAsItWas) {
    auto model = loadStage(writeText("powered.usda", kPowered).string());
    ASSERT_TRUE(model) << model.error();
    const std::size_t prims = model->scene.prims.size();
    const std::size_t instances = model->world.schemaInstances.size();
    const std::filesystem::path overlay = writeText("bad.overlay.yaml", R"(schema_version: 1
apply:
  - prim: no_such_link
    schemas: { ElectricalLoadAPI_1: { elec:load:power: 5 } }
  - create: power/heater
    schemas:
      ElectricalLoadAPI_1: { elec:load:power: 5, elec:load:colour: red, elec:powerPort: nowhere }
      HeaterAPI_1: {}
  - create: power/fan
    schemas:
      ElectricalLoadAPI_1: { elec:load:power: lots, elec:load:model: magic }
)");
    auto applied = applyOverlay(*model, overlay.string());
    ASSERT_FALSE(applied);
    const std::string& e = applied.error();
    EXPECT_NE(e.find("no_such_link is not a link, joint, sensor or prim"), std::string::npos) << e;
    EXPECT_NE(e.find("elec:load:colour is not a property of ElectricalLoadAPI_1"),
              std::string::npos)
        << e;
    EXPECT_NE(e.find("nowhere is not a link"), std::string::npos) << e;
    EXPECT_NE(e.find("HeaterAPI_1 is not a registered schema"), std::string::npos) << e;
    EXPECT_NE(e.find("/bot/power/fan: elec:load:power"), std::string::npos) << e;
    EXPECT_EQ(model->scene.prims.size(), prims);
    EXPECT_EQ(model->world.schemaInstances.size(), instances);

    auto version =
        applyOverlay(*model, writeText("v2.yaml", "schema_version: 2\napply: []\n").string());
    ASSERT_FALSE(version);
    EXPECT_NE(version.error().find("schema_version must be 1"), std::string::npos);
}

// Streamed worlds (huge-worlds.md §2): a stage with WorldTileAPI_1 prims opens with their
// payloads unloaded and lists them; one tile reads alone, its content placed where the whole
// stage puts it.
TEST(StageReader, TilesStayUnloadedAndReadOneByOne) {
    ReadOptions read;
    read.addGround = false;
    const auto town = loadStage(LAGRANGE_TEST_TILED_TOWN, read);
    ASSERT_TRUE(town) << town.error();
    ASSERT_EQ(town->tiles.size(), 68U);
    EXPECT_TRUE(town->world.colliders.body.empty()) << "no tile's buildings yet";
    const streaming::TileInfo* fine = nullptr;
    for (const streaming::TileInfo& t : town->tiles) {
        if (t.path == "/tiled_town/L1_2_3") {
            fine = &t;
        }
    }
    ASSERT_NE(fine, nullptr);
    EXPECT_EQ(fine->level, 1);
    EXPECT_TRUE(fine->hasCollision);
    EXPECT_EQ(fine->boundsMin, (std::array<double, 3>{512.0, 768.0, 0.0}));
    EXPECT_EQ(fine->boundsMax, (std::array<double, 3>{768.0, 1024.0, 60.0}));
    EXPECT_DOUBLE_EQ(fine->geometricError, 0.5);
    EXPECT_GT(fine->bytes, 500U) << "the payload file's size";
    EXPECT_EQ(fine->source, LAGRANGE_TEST_TILED_TOWN);

    read.tile = fine->path;
    const auto tile = loadStage(LAGRANGE_TEST_TILED_TOWN, read);
    ASSERT_TRUE(tile) << tile.error();
    EXPECT_TRUE(tile->tiles.empty());
    ASSERT_FALSE(tile->world.colliders.body.empty());
    for (std::size_t c = 0; c < tile->world.colliders.body.size(); ++c) {
        const std::array<double, 3>& p = tile->world.colliders.poseInBody[c].position;
        EXPECT_GE(p[0], 512.0);
        EXPECT_LE(p[0], 768.0);
        EXPECT_GE(p[1], 768.0);
        EXPECT_LE(p[1], 1024.0);
    }

    // Appended to a model in a region 500 m east: under the tile prim, moved into the region.
    Model world = *town;
    const ModelExtent base = extentOf(world);
    appendTile(world, *tile, fine->path, {500.0, 0.0, 0.0});
    EXPECT_EQ(world.world.colliders.body.size(), tile->world.colliders.body.size());
    EXPECT_NEAR(world.world.colliders.poseInBody[0].position[0],
                tile->world.colliders.poseInBody[0].position[0] - 500.0, 1e-12);
    bool underTile = false;
    for (const ScenePrim& prim : world.scene.prims) {
        underTile = underTile || prim.path == fine->path + "/building_0/visual";
    }
    EXPECT_TRUE(underTile);
    truncateModel(world, base);
    EXPECT_TRUE(world.world.colliders.body.empty());
    EXPECT_EQ(world.scene.prims.size(), town->scene.prims.size());
}

}  // namespace
}  // namespace lagrange::stage
