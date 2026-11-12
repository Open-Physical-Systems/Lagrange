// landing_target (sensors.md §9): a pad at a known offset under a downward camera gives the
// angles and distance of the geometry, its noise has the spread of the pixel-noise model, and it
// is lost behind a collider, outside the field of view, beyond its range and when too small.
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

constexpr double kHeight = 5.0;                   // m, the camera above the ground
const std::array<double, 3> kPad{1.0, 0.5, 0.0};  // m, where the pad lies
constexpr double kPadSize = 2.0;                  // m
constexpr double kPadThickness = 0.005;           // m

/// A camera 5 m up looking straight down (its x along world -z), a 2 m pad on the ground.
Model scene(std::map<std::string, double, std::less<>> settings = {},
            std::map<std::string, std::string, std::less<>> text = {}) {
    Model model;
    World& w = model.world;
    const BodyId drone = w.addBody("drone", kWorldBody, Pose{{0, 0, kHeight}}, 0.0, {}, {});
    const double h = std::numbers::sqrt2 / 2.0;
    w.addCustomSensor("pad_finder", "landing_target", drone, Pose{{0, 0, 0}, {h, 0, h, 0}}, 20.0,
                      std::move(settings), std::move(text));
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    model.scene.meshes.push_back(boxMesh(kPadSize, kPadSize, kPadThickness));
    ScenePrim field;
    field.name = "field";
    field.path = "/field";
    model.scene.prims.push_back(field);
    ScenePrim pad;
    pad.name = "landing_pad";
    pad.path = "/field/landing_pad";
    pad.parent = 0;
    pad.mesh = 0;
    pad.offset = {{kPad[0], kPad[1], kPad[2] + kPadThickness / 2.0}};
    model.scene.prims.push_back(pad);
    return model;
}

struct Rig {
    Scheduler scheduler{SchedulerOptions{}};
    explicit Rig(const Model& model, bool groundTruth) {
        auto modules =
            groundTruth ? createSensorSubstitutes(model) : createSensorModules(model, nullptr);
        for (auto& m : modules) {
            scheduler.add(std::move(m));
        }
        auto wired = scheduler.wire(WorldView(model.world));
        ok = static_cast<bool>(wired);
        if (!ok) {
            error = to_string(wired.error().front());
        }
    }
    double read(const char* q) {
        const SignalBus& bus = scheduler.bus();
        return bus.read(bus.find(std::string("sensor/pad_finder/") + q));
    }
    bool ok{false};
    std::string error;
};

TEST(LandingTarget, IsABuiltinKind) {
    EXPECT_TRUE(isBuiltinKind("landing_target"));
}

TEST(LandingTarget, GivesTheAnglesAndDistanceOfTheGeometry) {
    const Model model = scene();
    Rig run(model, true);
    ASSERT_TRUE(run.ok) << run.error;
    ASSERT_TRUE(run.scheduler.stepTick().ok());
    // The pad's top centre in the sensor frame (x down, y left, z along world +x): x = 5 - top,
    // y = 0.5, z = 1. Optical: x right = -0.5, y down = -1, z forward = 5 - top.
    const double forward = kHeight - kPadThickness;
    EXPECT_EQ(run.read("detected"), 1.0);
    EXPECT_NEAR(run.read("angle_x"), std::atan2(-0.5, forward), 1e-8);  // the mesh is float
    EXPECT_NEAR(run.read("angle_y"), std::atan2(-1.0, forward), 1e-8);  // the mesh is float
    EXPECT_NEAR(run.read("distance"), std::sqrt(forward * forward + 0.25 + 1.0),
                1e-8);  // the mesh is float
    EXPECT_NEAR(run.read("size_x"), 2.0 * std::atan(1.0 / run.read("distance")),
                1e-8);  // the mesh is float
    const SignalBus& bus = run.scheduler.bus();
    const std::span<const double> p = bus.readN(bus.find("sensor/pad_finder/position"));
    EXPECT_NEAR(p[0], forward, 1e-9);
    EXPECT_NEAR(p[1], 0.5, 1e-9);
    EXPECT_NEAR(p[2], 1.0, 1e-9);
}

TEST(LandingTarget, NoiseHasTheSpreadOfThePixelModel) {
    const Model model = scene({{"pixel_noise", 1.0}});
    Rig run(model, false);
    ASSERT_TRUE(run.ok) << run.error;
    const double forward = kHeight - kPadThickness;
    const double distance = std::sqrt(forward * forward + 1.25);
    const double focal = 640.0 / 2.0 / std::tan(1.047 / 2.0);
    // Facing the camera at cos θ = forward / distance; pixels across its side.
    const double pixels = focal * kPadSize * std::sqrt(forward / distance) / distance;
    constexpr int kN = 4000;
    double sx = 0.0, sxx = 0.0, sd = 0.0, sdd = 0.0;
    for (int i = 0; i < kN; ++i) {
        ASSERT_TRUE(run.scheduler.stepTick().ok());
        ASSERT_EQ(run.read("detected"), 1.0);
        const double ax = run.read("angle_x") - std::atan2(-0.5, forward);
        const double d = run.read("distance") - distance;
        sx += ax;
        sxx += ax * ax;
        sd += d;
        sdd += d * d;
    }
    const double angleSigma = std::sqrt(sxx / kN - (sx / kN) * (sx / kN));
    const double distanceSigma = std::sqrt(sdd / kN - (sd / kN) * (sd / kN));
    EXPECT_NEAR(angleSigma, 1.0 / focal, 0.05 / focal);                       // σ_px / f
    EXPECT_NEAR(distanceSigma, distance / pixels, 0.05 * distance / pixels);  // d σ_px / pixels
    EXPECT_NEAR(sd / kN, 0.0, 4.0 * distance / pixels / std::sqrt(kN));
}

TEST(LandingTarget, IsLostBehindACollider) {
    Model model = scene();
    model.world.addCollider(kWorldBody, ShapeType::Box, {0.3, 0.3, 0.3}, Pose{{0.5, 0.25, 2.5}},
                            1.0);
    Rig run(model, true);
    ASSERT_TRUE(run.ok) << run.error;
    ASSERT_TRUE(run.scheduler.stepTick().ok());
    EXPECT_EQ(run.read("detected"), 0.0);
    EXPECT_TRUE(std::isnan(run.read("distance")));
}

TEST(LandingTarget, IsLostOutsideTheFieldOfViewAndBeyondItsRange) {
    {
        const Model model = scene({{"horizontal_fov", 0.3}, {"vertical_fov", 0.3}});
        Rig run(model, true);
        ASSERT_TRUE(run.ok) << run.error;
        ASSERT_TRUE(run.scheduler.stepTick().ok());
        EXPECT_EQ(run.read("detected"), 0.0);  // 11° off the axis, a 17° wide view
    }
    {
        const Model model = scene({{"max_range", 4.0}});
        Rig run(model, true);
        ASSERT_TRUE(run.ok) << run.error;
        ASSERT_TRUE(run.scheduler.stepTick().ok());
        EXPECT_EQ(run.read("detected"), 0.0);
    }
}

TEST(LandingTarget, DetectsAsOftenAsItsPixelsAllow) {
    const double forward = kHeight - kPadThickness;
    const double distance = std::sqrt(forward * forward + 1.25);
    const double focal = 640.0 / 2.0 / std::tan(1.047 / 2.0);
    const double pixels = focal * kPadSize * std::sqrt(forward / distance) / distance;
    // min_pixels such that the chance is 0.3: (pixels - m) / m = 0.3.
    const double minPixels = pixels / 1.3;
    const Model model = scene({{"min_pixels", minPixels}});
    Rig run(model, false);
    ASSERT_TRUE(run.ok) << run.error;
    constexpr int kN = 4000;
    int hits = 0;
    for (int i = 0; i < kN; ++i) {
        ASSERT_TRUE(run.scheduler.stepTick().ok());
        hits += run.read("detected") == 1.0 ? 1 : 0;
    }
    EXPECT_NEAR(static_cast<double>(hits) / kN, 0.3, 4.0 * std::sqrt(0.3 * 0.7 / kN));
}

TEST(LandingTarget, RefusesATargetThatIsNotInTheScene) {
    const Model model = scene({}, {{"target", "helipad"}});
    Rig run(model, false);
    EXPECT_FALSE(run.ok);
    EXPECT_NE(run.error.find("helipad"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::sensors
