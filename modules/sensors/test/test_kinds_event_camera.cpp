// event_camera (sensors.md §9): ESIM's events for a step in log intensity (one per threshold
// crossed, at the time the line crosses it), the refractory period, threshold mismatch and
// background noise statistics; then a rendered run (llvmpipe without a GPU, skipped without
// any Vulkan device).
#include "CameraKinds.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <numeric>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;
using kinds::DvsEvent;
using kinds::EventConfig;
using kinds::EventGenerator;

EventConfig config(std::uint32_t width, std::uint32_t height) {
    EventConfig c;
    c.width = width;
    c.height = height;
    c.thresholdOn = 0.2;
    c.thresholdOff = 0.25;
    return c;
}

TEST(EventCamera, AStepFiresOneEventPerThresholdAtTheCrossingTimes) {
    EventGenerator gen(config(2, 1));
    std::vector<DvsEvent> events;
    const std::vector<float> start{0.0F, 0.0F};
    gen.feed(start, 0.0, 1, 0, events);
    EXPECT_TRUE(events.empty());  // the first frame only memorises
    // Pixel 0 rises by 1.01 ln (5 thresholds of 0.2), pixel 1 falls by 0.51 ln (2 of 0.25), over
    // 1 s: linear in time, so each crossing is at level / change.
    const std::vector<float> next{1.01F, -0.51F};
    gen.feed(next, 1.0, 1, 1, events);
    ASSERT_EQ(events.size(), 7U);
    std::vector<std::pair<double, int>> expected;
    for (int k = 1; k <= 5; ++k) {
        expected.emplace_back(0.2 * k / 1.01, 1);
    }
    for (int k = 1; k <= 2; ++k) {
        expected.emplace_back(0.25 * k / 0.51, -1);
    }
    std::sort(expected.begin(), expected.end());
    for (std::size_t k = 0; k < expected.size(); ++k) {
        EXPECT_NEAR(events[k].t, expected[k].first, 1e-6) << k;
        EXPECT_EQ(events[k].polarity, expected[k].second) << k;
        EXPECT_EQ(events[k].x, expected[k].second > 0 ? 0U : 1U) << k;
    }
    // No change, no events.
    events.clear();
    gen.feed(next, 2.0, 1, 2, events);
    EXPECT_TRUE(events.empty());
}

TEST(EventCamera, TheRefractoryPeriodHoldsTheChangeUntilItEnds) {
    EventConfig c = config(1, 1);
    c.refractory = 0.3;
    EventGenerator gen(c);
    std::vector<DvsEvent> events;
    gen.feed(std::vector<float>{0.0F}, 0.0, 1, 0, events);
    gen.feed(std::vector<float>{1.0F}, 1.0, 1, 1, events);
    // 0.2 fires, 0.4 falls inside the hold and stops the pixel for this frame.
    ASSERT_EQ(events.size(), 1U);
    EXPECT_NEAR(events[0].t, 0.2, 1e-6);
    events.clear();
    gen.feed(std::vector<float>{1.0F}, 2.0, 1, 2, events);
    // The held change fires at the start of the next interval, one at a time.
    ASSERT_EQ(events.size(), 1U);
    EXPECT_NEAR(events[0].t, 1.0, 1e-6);
}

TEST(EventCamera, ThresholdMismatchAndNoiseHaveTheirStatistics) {
    EventConfig c = config(100, 100);
    c.thresholdSigma = 0.03;
    c.noiseRate = 1.0;  // Hz per pixel
    EventGenerator gen(c);
    std::vector<DvsEvent> events;
    const std::vector<float> flat(100 * 100, 0.0F);
    gen.feed(flat, 0.0, 42, 0, events);
    double sum = 0.0, square = 0.0;
    for (std::size_t i = 0; i < flat.size(); ++i) {
        sum += gen.thresholdOn(i);
        square += gen.thresholdOn(i) * gen.thresholdOn(i);
    }
    const auto n = static_cast<double>(flat.size());
    const double mean = sum / n;
    EXPECT_NEAR(mean, 0.2, 0.002);
    EXPECT_NEAR(std::sqrt(square / n - mean * mean), 0.03, 0.002);
    // 10⁴ pixels × 1 Hz × 0.1 s = 1000 noise events, Poisson: ±3σ ≈ ±95.
    gen.feed(flat, 0.1, 42, 1, events);
    EXPECT_NEAR(static_cast<double>(events.size()), 1000.0, 95.0);
    const auto on = std::count_if(events.begin(), events.end(),
                                  [](const DvsEvent& e) { return e.polarity > 0; });
    EXPECT_NEAR(static_cast<double>(on), static_cast<double>(events.size()) / 2.0,
                3.0 * std::sqrt(250.0));
    for (const DvsEvent& e : events) {
        EXPECT_GE(e.t, 0.0);
        EXPECT_LE(e.t, 0.1);
    }
}

TEST(EventCamera, LinLogIsLnAboveTheKneeAndLinearBelow) {
    EXPECT_NEAR(kinds::linLog(255, 255, 255), std::log(255.0), 1e-5);
    EXPECT_NEAR(kinds::linLog(0, 0, 0), 0.0, 1e-9);
    // Continuous at the knee: sRGB 20 DN linear is about 79 display.
    const float below = kinds::linLog(78, 78, 78);
    const float above = kinds::linLog(80, 80, 80);
    EXPECT_LT(below, above);
    EXPECT_NEAR(below, std::log(kinds::kLinLogDn), 0.1);
}

// --- rendered ------------------------------------------------------------------------------

/// Slides the camera's body sideways at `speed` (m/s) from the origin, 1 m up.
class Slide final : public IModule {
public:
    explicit Slide(double speed) : speed_(speed) {}
    ModuleInfo info() const override { return {"test", "slide", 0, 1, true, true}; }
    Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        pose_ =
            d.output(kinds::kindSpec("body/rig/pose", SignalType::Pose, units::m, "world_local"));
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double y = speed_ * (c.t + c.dt).seconds();
        bus.writeN(pose_, std::array<double, 7>{0.0, y, 1.0, 1.0, 0.0, 0.0, 0.0});
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double speed_;
    SignalId pose_;
};

double events(double speed) {
    Model model;
    World& w = model.world;
    const BodyId rig = w.addBody("rig", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
    w.addCustomSensor(
        "dvs", "event_camera", rig, Pose{}, 100.0,
        {{"width", 64.0}, {"height", 48.0}, {"noise_rate", 0.0}, {"threshold_sigma", 0.0}}, {});
    // A white cube 3 m ahead against the sky: its edges move across the image as the rig slides.
    model.scene.meshes.push_back(boxMesh(1.0, 1.0, 1.0));
    model.scene.materials.push_back({"white", {1.0F, 1.0F, 1.0F, 1.0F}});
    ScenePrim cube;
    cube.name = "cube";
    cube.mesh = 0;
    cube.material = 0;
    cube.offset = {{3, 0, 1}};
    model.scene.prims.push_back(cube);
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> scene =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!scene) {
        return -1.0;
    }
    Scheduler scheduler;
    scheduler.add(std::make_unique<Slide>(speed));
    for (auto& module : createSensorModules(w, nullptr, scene)) {
        EXPECT_FALSE(module->info().deterministic);
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(w));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    double total = 0.0;
    const SignalBus& bus = scheduler.bus();
    for (int k = 0; k < 10; ++k) {
        EXPECT_TRUE(scheduler.stepUntil(SimTime{(k + 1) * 10'000'000LL}).ok());
        total += bus.read(bus.find("sensor/dvs/count"));
    }
    const auto image = bus.buffer(bus.readBuffer(bus.find("sensor/dvs/image")));
    EXPECT_TRUE(image && image->bytes.size() == 64U * 48U * 4U);
    return total;
}

TEST(EventCamera, AStillSceneIsSilentAndAMovingOneFires) {
    const double still = events(0.0);
    if (still < 0.0) {
        GTEST_SKIP() << "no Vulkan device";
    }
    EXPECT_EQ(still, 0.0);
    EXPECT_GT(events(2.0), 10.0);
}

TEST(EventCamera, GroundTruthRendersNothingAndFiresNothing) {
    World w;
    const BodyId rig = w.addBody("rig", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.addCustomSensor("dvs", "event_camera", rig, Pose{}, 100.0, {}, {});
    Scheduler scheduler;
    for (auto& module : createSensorSubstitutes(w)) {
        EXPECT_TRUE(module->info().deterministic);
        scheduler.add(std::move(module));
    }
    ASSERT_TRUE(scheduler.wire(WorldView(w)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{50'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    EXPECT_EQ(bus.read(bus.find("sensor/dvs/count")), 0.0);
}

}  // namespace
}  // namespace lagrange::sensors
