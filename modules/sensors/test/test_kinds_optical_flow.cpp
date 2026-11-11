// optical_flow (sensors.md §9): the flow of a sensor moving at v over a plane at height h is
// v/h, a rotation reads as the gyro does (PX4's sign), and the quality drops out of range.
#include "CameraKinds.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <cmath>
#include <memory>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

/// Writes the drone's pose (fixed `height`, looking down is the sensor's mount) and its twist.
class Mover final : public IModule {
public:
    Mover(double height, std::array<double, 3> velocity, std::array<double, 3> spin)
        : height_(height), velocity_(velocity), spin_(spin) {}
    ModuleInfo info() const override { return {"test", "mover", 0, 1, true, true}; }
    Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        pose_ =
            d.output(kinds::kindSpec("body/drone/pose", SignalType::Pose, units::m, "world_local"));
        twist_ = d.output(
            kinds::kindSpec("body/drone/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double t = (c.t + c.dt).seconds();
        // Only the position moves; the attitude stays level so the geometry stays the same.
        bus.writeN(pose_, std::array<double, 7>{velocity_[0] * t, velocity_[1] * t, height_, 1.0,
                                                0.0, 0.0, 0.0});
        bus.writeN(twist_, std::array<double, 6>{velocity_[0], velocity_[1], velocity_[2], spin_[0],
                                                 spin_[1], spin_[2]});
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double height_;
    std::array<double, 3> velocity_, spin_;
    SignalId pose_, twist_;
};

struct Result {
    double flowX, flowY, quality, distance, time;
};

/// A drone at `height` over the ground plane with a flow sensor looking straight down (its x
/// pitched to −z), at 100 Hz.
Result run(double height, std::array<double, 3> velocity, std::array<double, 3> spin,
           std::map<std::string, double, std::less<>> settings = {}, bool groundTruth = false) {
    World w;
    const BodyId drone = w.addBody("drone", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
    w.addCollider(kWorldBody, ShapeType::Plane, {}, Pose{}, 1.0);
    const double h = std::numbers::sqrt2 / 2.0;
    w.addCustomSensor("flow", "optical_flow", drone, Pose{{0, 0, 0}, {h, 0, h, 0}}, 100.0,
                      std::move(settings), {});
    Scheduler scheduler{SchedulerOptions{.deterministic = true}};
    scheduler.add(std::make_unique<Mover>(height, velocity, spin));
    auto module = createBuiltinKind(w, 0, groundTruth, nullptr, nullptr);
    EXPECT_TRUE(module);
    EXPECT_EQ(module->preferredStep(), 10_ms);
    scheduler.add(std::move(module));
    auto wired = scheduler.wire(WorldView(w));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_TRUE(scheduler.stepUntil(SimTime{50'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const std::span<const double> flow = bus.readN(bus.find("sensor/flow/flow"));
    return {flow[0], flow[1], bus.read(bus.find("sensor/flow/quality")),
            bus.read(bus.find("sensor/flow/distance")),
            bus.read(bus.find("sensor/flow/integration_time"))};
}

TEST(OpticalFlow, IsABuiltinKind) {
    EXPECT_TRUE(isBuiltinKind("optical_flow"));
}

TEST(OpticalFlow, TheMotionFieldOnTheAxisIsRotationPlusVelocityOverDepth) {
    // ω + (ẑ × v)/Z on the optical axis, PX4's sign.
    const auto f = kinds::flowRate({0.3, -0.7, 0.2}, {0.1, -0.05, 0.4}, 0.0, 0.0, 2.0);
    EXPECT_NEAR(f[0], 0.1 + 0.7 / 2.0, 1e-12);
    EXPECT_NEAR(f[1], -0.05 + 0.3 / 2.0, 1e-12);
}

TEST(OpticalFlow, MovingAtVOverAPlaneAtHeightHReadsVOverH) {
    constexpr double kV = 1.5;
    constexpr double kH = 3.0;
    const Result r = run(kH, {kV, 0.0, 0.0}, {}, {{"noise", 0.0}});
    // World +x is the optical −y (the sensor's x points down, its z forward): flow about the
    // optical x is v_y,opt/... = +v/h by ω + ẑ × v / h, integrated over the 10 ms sample.
    EXPECT_NEAR(r.flowX, kV / kH * 0.01, 1e-12);
    EXPECT_NEAR(r.flowY, 0.0, 1e-12);
    EXPECT_NEAR(r.distance, kH, 1e-9);
    EXPECT_DOUBLE_EQ(r.time, 0.01);
    EXPECT_EQ(r.quality, 255.0);
    // Sideways (world +y is the optical −x): about the optical y, −v/h.
    const Result side = run(kH, {0.0, kV, 0.0}, {}, {{"noise", 0.0}});
    EXPECT_NEAR(side.flowY, -kV / kH * 0.01, 1e-12);
    EXPECT_NEAR(side.flowX, 0.0, 1e-12);
}

TEST(OpticalFlow, ARotationReadsAsTheGyroAboutTheSameAxis) {
    // A narrow field of view, so the rays are all near the axis: the flow is ω (PX4: a
    // right-handed rotation reads positive; the gyro's delta angle compensates it).
    constexpr double kW = 0.8;
    // World +x is the optical −y, so a spin about world x is −ω about the optical y.
    const Result r = run(2.0, {}, {kW, 0.0, 0.0}, {{"noise", 0.0}, {"fov", 1e-4}});
    EXPECT_NEAR(r.flowY, -kW * 0.01, 1e-9);
    EXPECT_NEAR(r.flowX, 0.0, 1e-9);
}

TEST(OpticalFlow, NoiseHasItsSigmaAndGroundTruthHasNone) {
    const Result exact = run(3.0, {1.5, 0.0, 0.0}, {}, {{"noise", 0.5}}, true);
    EXPECT_NEAR(exact.flowX, 0.005, 1e-12);
    const Result noisy = run(3.0, {1.5, 0.0, 0.0}, {}, {{"noise", 0.5}});
    EXPECT_NE(noisy.flowX, exact.flowX);
    EXPECT_LT(std::abs(noisy.flowX - exact.flowX), 5.0 * 0.5 * 0.01);
}

TEST(OpticalFlow, OutOfRangeOrTooFastItLosesTheTexture) {
    const Result low = run(0.05, {1.0, 0.0, 0.0}, {}, {});  // under the 80 mm minimum
    EXPECT_EQ(low.quality, 0.0);
    EXPECT_EQ(low.flowX, 0.0);
    EXPECT_TRUE(std::isinf(low.distance));
    const Result fast = run(1.0, {10.0, 0.0, 0.0}, {}, {});  // 10 rad/s > 7.4 rad/s
    EXPECT_EQ(fast.quality, 0.0);
    const Result far = run(50.0, {1.0, 0.0, 0.0}, {}, {});  // beyond the 30 m default
    EXPECT_EQ(far.quality, 0.0);
}

}  // namespace
}  // namespace lagrange::sensors
