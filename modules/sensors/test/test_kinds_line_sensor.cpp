// line_sensor (sensors.md §9): tape at a known lateral offset under the array gives that offset
// and width, from a named tape prim or from a bright collider's surface material.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kTapeOffset = 0.023;  // m, to the sensor's left (+y)
constexpr double kTapeWidth = 0.03;    // m
constexpr double kSpacing = 0.01;      // m between channels, the default

/// An AGV 5 cm over a floor plane, its array looking down; tape along x, off to the left.
Model scene(std::map<std::string, double, std::less<>> settings = {}) {
    Model model;
    World& w = model.world;
    const BodyId agv = w.addBody("agv", kWorldBody, Pose{{0, 0, 0.05}}, 0.0, {}, {});
    w.addCustomSensor("guide", "line_sensor", agv, Pose{}, 50.0, std::move(settings), {});
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    model.scene.meshes.push_back(boxMesh(2.0, kTapeWidth, 0.0005));
    ScenePrim tape;
    tape.name = "tape_main";
    tape.path = "/warehouse/tape_main";
    tape.mesh = 0;
    tape.offset = {{0.0, kTapeOffset, 0.00025}};
    model.scene.prims.push_back(tape);
    return model;
}

struct Rig {
    Scheduler scheduler{SchedulerOptions{}};
    Rig(const Model& model, bool groundTruth, bool withScene = true) {
        auto modules = groundTruth ? (withScene ? createSensorSubstitutes(model)
                                                : createSensorSubstitutes(model.world))
                                   : createSensorModules(model, nullptr);
        for (auto& m : modules) {
            scheduler.add(std::move(m));
        }
        auto wired = scheduler.wire(WorldView(model.world));
        EXPECT_TRUE(wired);
        EXPECT_TRUE(scheduler.stepTick().ok());
    }
    double read(const char* q) {
        const SignalBus& bus = scheduler.bus();
        return bus.read(bus.find(std::string("sensor/guide/") + q));
    }
    std::span<const double> channels() {
        const SignalBus& bus = scheduler.bus();
        return bus.readN(bus.find("sensor/guide/channels"));
    }
};

TEST(LineSensor, IsABuiltinKind) {
    EXPECT_TRUE(isBuiltinKind("line_sensor"));
}

TEST(LineSensor, FindsATapePrimAtItsOffset) {
    const Model model = scene();
    Rig run(model, true);
    EXPECT_EQ(run.read("detected"), 1.0);
    // Channel centres at (7.5 - c) cm; 5 rays across each: the centroid of the covered rays is
    // within one ray spacing (2 mm) of the tape's centre, the width within one too.
    EXPECT_NEAR(run.read("offset"), kTapeOffset, kSpacing / 5.0);
    EXPECT_NEAR(run.read("width"), kTapeWidth, kSpacing / 5.0);
    const std::span<const double> c = run.channels();
    ASSERT_EQ(c.size(), 16U);
    EXPECT_EQ(c[5], 1.0);   // centre +2.5 cm, under the tape (0.8 to 3.8 cm)
    EXPECT_EQ(c[0], 0.0);   // +7.5 cm
    EXPECT_EQ(c[15], 0.0);  // -7.5 cm
}

TEST(LineSensor, SeesThroughItsOwnChassis) {
    Model model = scene();
    // The AGV's chassis box around the array: its own colliders are not the floor.
    model.world.addCollider(BodyId{0}, ShapeType::Box, {0.3, 0.2, 0.02}, Pose{}, 1.0);
    Rig run(model, true);
    EXPECT_EQ(run.read("detected"), 1.0);
    EXPECT_NEAR(run.read("offset"), kTapeOffset, kSpacing / 5.0);
}

TEST(LineSensor, SeesNothingBeyondItsRange) {
    const Model model = scene({{"max_range", 0.02}});
    Rig run(model, true);
    EXPECT_EQ(run.read("detected"), 0.0);
    EXPECT_TRUE(std::isnan(run.read("offset")));
}

TEST(LineSensor, FindsBrightTapeByItsSurfaceMaterial) {
    Model model = scene();
    model.scene.prims.clear();  // no named tape: a bright collider instead, 4 cm to the right
    World& w = model.world;
    SurfaceMaterial white;
    white.reflectivity905 = 0.9;
    w.surfaceMaterials.push_back(white);
    w.addCollider(kWorldBody, ShapeType::Box, {0.5, kTapeWidth / 2.0, 0.0005},
                  Pose{{0.0, -0.04, 0.0}}, 1.0);
    w.colliders.material.back() = 0;
    Rig run(model, true, false);
    EXPECT_EQ(run.read("detected"), 1.0);
    EXPECT_NEAR(run.read("offset"), -0.04, kSpacing / 5.0);
}

TEST(LineSensor, NoisyChannelsStillFindTheTape) {
    const Model model = scene({{"noise", 0.05}});
    Rig run(model, false);
    EXPECT_EQ(run.read("detected"), 1.0);
    EXPECT_NEAR(run.read("offset"), kTapeOffset, kSpacing / 2.0);
}

}  // namespace
}  // namespace lagrange::sensors
