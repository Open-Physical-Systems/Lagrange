// tag_reader (sensors.md §9): floor tags named qr_* and rfid_* are read with their id, number,
// position, and for QR their offset and yaw in the reader's frame.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kTagYaw = std::numbers::pi / 6.0;  // rad, the QR tag turned 30° about z
constexpr double kTagThickness = 0.001;             // m

ScenePrim tag(std::string name, const std::array<double, 3>& at, double yaw) {
    ScenePrim p;
    p.name = name;
    p.path = "/warehouse/" + name;
    p.mesh = 0;
    p.offset = {{at[0], at[1], at[2] + kTagThickness / 2.0},
                {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)}};
    return p;
}

/// A reader on an AGV at `at`, looking down; tags on the floor.
Model scene(const std::array<double, 3>& at, std::string mode,
            std::map<std::string, double, std::less<>> settings = {}) {
    Model model;
    World& w = model.world;
    const BodyId agv = w.addBody("agv", kWorldBody, Pose{at}, 0.0, {}, {});
    w.addCustomSensor("reader", "tag_reader", agv, Pose{}, 20.0, std::move(settings),
                      {{"mode", std::move(mode)}});
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    model.scene.meshes.push_back(boxMesh(0.05, 0.05, kTagThickness));
    model.scene.prims.push_back(tag("qr_0007", {0.3, 0.1, 0.0}, kTagYaw));
    model.scene.prims.push_back(tag("qr_0003", {1.3, 0.1, 0.0}, 0.0));
    model.scene.prims.push_back(tag("rfid_12", {2.0, 0.0, 0.0}, 0.0));
    return model;
}

struct Rig {
    Scheduler scheduler{SchedulerOptions{}};
    Rig(const Model& model, bool groundTruth) {
        auto modules =
            groundTruth ? createSensorSubstitutes(model) : createSensorModules(model, nullptr);
        for (auto& m : modules) {
            scheduler.add(std::move(m));
        }
        auto wired = scheduler.wire(WorldView(model.world));
        ok = static_cast<bool>(wired);
        if (ok) {
            EXPECT_TRUE(scheduler.stepTick().ok());
        }
    }
    double read(const char* q) {
        const SignalBus& bus = scheduler.bus();
        return bus.read(bus.find(std::string("sensor/reader/") + q));
    }
    std::span<const double> readN(const char* q) {
        const SignalBus& bus = scheduler.bus();
        return bus.readN(bus.find(std::string("sensor/reader/") + q));
    }
    bool ok{false};
};

TEST(TagReader, IsABuiltinKind) {
    EXPECT_TRUE(isBuiltinKind("tag_reader"));
}

TEST(TagReader, ReadsAQrTagsOffsetAndYaw) {
    const Model model = scene({0.3, 0.12, 0.2}, "qr");
    Rig run(model, true);
    ASSERT_TRUE(run.ok);
    EXPECT_EQ(run.read("detected"), 1.0);
    EXPECT_EQ(run.read("tag"), 1.0);  // sorted by path: qr_0003, qr_0007
    EXPECT_EQ(run.read("code"), 7.0);
    const std::span<const double> offset = run.readN("offset");
    EXPECT_NEAR(offset[0], 0.0, 1e-8);                  // the mesh is float
    EXPECT_NEAR(offset[1], -0.02, 1e-8);                // the mesh is float
    EXPECT_NEAR(offset[2], kTagThickness - 0.2, 1e-8);  // the mesh is float
    EXPECT_NEAR(run.read("yaw"), kTagYaw, 1e-8);        // the mesh is float
    const std::span<const double> where = run.readN("tag_position");
    EXPECT_NEAR(where[0], 0.3, 1e-8);  // the mesh is float
    EXPECT_NEAR(where[1], 0.1, 1e-8);  // the mesh is float
}

TEST(TagReader, QrSeesOutOfItsOwnHousingButNotThroughOthers) {
    {
        // The reader inside its AGV's box collider (a camera in its housing) still reads.
        Model model = scene({0.3, 0.12, 0.2}, "qr");
        model.world.addCollider(BodyId{0}, ShapeType::Box, {0.2, 0.2, 0.05}, Pose{}, 1.0);
        Rig run(model, true);
        ASSERT_TRUE(run.ok);
        EXPECT_EQ(run.read("detected"), 1.0);
    }
    {
        // A plate of another body between the reader and the tag hides it.
        Model model = scene({0.3, 0.12, 0.2}, "qr");
        const BodyId plate =
            model.world.addBody("plate", kWorldBody, Pose{{0.3, 0.11, 0.1}}, 0.0, {}, {});
        model.world.addCollider(plate, ShapeType::Box, {0.1, 0.1, 0.005}, Pose{}, 1.0);
        Rig run(model, true);
        ASSERT_TRUE(run.ok);
        EXPECT_EQ(run.read("detected"), 0.0);
    }
}

TEST(TagReader, QrNeedsTheTagInItsFootprint) {
    const Model model = scene({0.8, 0.1, 0.2}, "qr");  // 0.5 m from both: 68° off the axis
    Rig run(model, true);
    ASSERT_TRUE(run.ok);
    EXPECT_EQ(run.read("detected"), 0.0);
    EXPECT_EQ(run.read("tag"), -1.0);
    EXPECT_TRUE(std::isnan(run.read("yaw")));
}

TEST(TagReader, NoisyQrReadsScatterAsSet) {
    const Model model = scene({0.3, 0.12, 0.2}, "qr", {{"position_noise", 0.001}});
    Rig run(model, false);
    ASSERT_TRUE(run.ok);
    constexpr int kN = 2000;
    double s = 0.0, ss = 0.0;
    for (int i = 0; i < kN; ++i) {
        ASSERT_TRUE(run.scheduler.stepTick().ok());
        const double e = run.readN("offset")[0];
        s += e;
        ss += e * e;
    }
    EXPECT_NEAR(std::sqrt(ss / kN - (s / kN) * (s / kN)), 0.001, 0.0001);
}

TEST(TagReader, RfidReadsTheTagWithinItsRange) {
    {
        const Model model = scene({2.0, 0.05, 0.08}, "rfid");  // 9.4 cm from the tag
        Rig run(model, true);
        ASSERT_TRUE(run.ok);
        EXPECT_EQ(run.read("detected"), 1.0);
        EXPECT_EQ(run.read("tag"), 0.0);
        EXPECT_EQ(run.read("code"), 12.0);
        EXPECT_NEAR(run.readN("tag_position")[0], 2.0, 1e-8);  // the mesh is float
    }
    {
        const Model model = scene({2.0, 0.1, 0.08}, "rfid");  // 12.8 cm: out of range
        Rig run(model, true);
        ASSERT_TRUE(run.ok);
        EXPECT_EQ(run.read("detected"), 0.0);
    }
}

TEST(TagReader, RefusesAnUnknownMode) {
    const Model model = scene({0, 0, 0.2}, "barcode");
    Rig run(model, true);
    EXPECT_FALSE(run.ok);
}

}  // namespace
}  // namespace lagrange::sensors
