// The radar (domains/sensors.md): the radar equation, a flat plate's cross-section as ρ·area,
// its Doppler when it recedes, and the detection threshold at the range the equation gives.
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Radar.hpp"

#include <cmath>
#include <memory>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kDensity = 0.1;  // m²/m², SensorMaterialAPI_1's fallback
constexpr double kPower = 0.01;   // W
constexpr double kGain = 100.0;   // 20 dBi

TEST(Radar, TheRadarEquation) {
    const double lambda = 299792458.0 / 77e9;
    const double expected = kPower * kGain * kGain * lambda * lambda * 1.0 /
                            (std::pow(4.0 * std::numbers::pi, 3) * std::pow(50.0, 4) * 1e-13);
    EXPECT_NEAR(radarSnr(kPower, kGain, 77e9, 1.0, 50.0, 1e-13), expected, 1e-9 * expected);
    // R⁴: twice the range, a sixteenth of the signal.
    EXPECT_NEAR(radarSnr(kPower, kGain, 77e9, 1.0, 100.0, 1e-13) * 16.0, expected, 1e-9 * expected);
}

/// Moves the plate away along +x at `speed` from `start` (m).
class Plate final : public IModule {
public:
    Plate(double start, double speed) : start_(start), speed_(speed) {}
    ModuleInfo info() const override { return {"test", "plate", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "body/plate/pose_command";
        s.type = SignalType::Pose;
        s.unit = units::m;
        s.frame = "world_local";
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const double x = start_ + speed_ * (c.t + c.dt).seconds();
        bus.writeN(out_, std::array<double, 7>{x, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0});
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double start_, speed_;
    SignalId out_;
};

struct Run {
    World world;
    std::unique_ptr<Scheduler> scheduler;
};

/// A radar 1 m up looking along +x at a 2 m × 1 m plate, fine rays (0.01 rad resolution).
std::unique_ptr<Run> run(double start, double speed, double noiseFloor) {
    auto r = std::make_unique<Run>();
    World& w = r->world;
    Pose mountPose;
    mountPose.position = {0.0, 0.0, 1.0};
    const BodyId mount = w.addBody("mount", kWorldBody, mountPose, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.addSensor("front_radar", SensorType::Radar, mount, Pose{}, 20.0,
                {{"max_range", 100.0},
                 {"azimuth_fov", 0.5},
                 {"elevation_fov", 0.2},
                 {"azimuth_resolution", 0.01},
                 {"range_resolution", 0.2},
                 {"velocity_resolution", 0.1},
                 {"tx_power", kPower},
                 {"antenna_gain", kGain},
                 {"noise_floor", noiseFloor},
                 {"detection_threshold", 10.0},
                 {"carrier_frequency", 77e9}});
    Pose platePose;
    platePose.position = {start, 0.0, 1.0};
    const BodyId plate = w.addBody("plate", kWorldBody, platePose, 10.0, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
    w.addCollider(plate, ShapeType::Box, {0.01, 1.0, 0.5}, Pose{}, 0.8);  // faces the radar
    r->scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r->scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    r->scheduler->add(std::make_unique<Plate>(start, speed));
    r->scheduler->add(createRadar(w, 0, false));
    auto wired = r->scheduler->wire(WorldView(w));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return wired ? std::move(r) : nullptr;
}

TEST(Radar, APlatesCrossSectionIsItsAreaTimesTheDensity) {
    auto r = run(20.0, 0.0, 1e-15);
    ASSERT_TRUE(r);
    ASSERT_TRUE(r->scheduler->stepUntil(SimTime{120'000'000}).ok());
    const SignalBus& bus = r->scheduler->bus();
    ASSERT_EQ(bus.read(bus.find("sensor/front_radar/count")), 1.0);
    const std::span<const double> d = bus.readN(bus.find("sensor/front_radar/detections"));
    EXPECT_NEAR(d[4], kDensity * 2.0 * 1.0, 0.03 * kDensity * 2.0);  // σ = ρ·A, head on
    EXPECT_NEAR(d[0], 19.99, 0.2);                                   // the front face
    EXPECT_NEAR(d[1], 0.0, 0.01);
    EXPECT_NEAR(d[5], radarSnr(kPower, kGain, 77e9, d[4], 19.99, 1e-15), 0.01 * d[5]);
}

TEST(Radar, DopplerOfARecedingPlate) {
    auto r = run(20.0, 10.0, 1e-15);
    ASSERT_TRUE(r);
    ASSERT_TRUE(r->scheduler->stepUntil(SimTime{520'000'000}).ok());
    const SignalBus& bus = r->scheduler->bus();
    ASSERT_GE(bus.read(bus.find("sensor/front_radar/count")), 1.0);
    const std::span<const double> d = bus.readN(bus.find("sensor/front_radar/detections"));
    EXPECT_NEAR(d[3], 10.0, 0.1);  // receding: positive range rate
}

TEST(Radar, DetectsUpToTheRangeTheEquationGives) {
    // The noise floor for which σ = 0.2 m² reaches the threshold (10) at 25 m.
    const double noise = radarSnr(kPower, kGain, 77e9, 0.2, 25.0, 1.0) / 10.0;
    for (const auto& [range, detected] : {std::pair{22.0, true}, std::pair{28.0, false}}) {
        auto r = run(range, 0.0, noise);
        ASSERT_TRUE(r);
        ASSERT_TRUE(r->scheduler->stepUntil(SimTime{120'000'000}).ok());
        const SignalBus& bus = r->scheduler->bus();
        EXPECT_EQ(bus.read(bus.find("sensor/front_radar/count")), detected ? 1.0 : 0.0) << range;
    }
}

}  // namespace
}  // namespace lagrange::sensors
