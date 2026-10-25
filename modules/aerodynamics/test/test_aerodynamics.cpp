// The aerodynamics domain against aerodynamics.md §7: terminal velocity (closed form), the
// logarithmic wind profile, and the Dryden turbulence's variances and correlation time.
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::aerodynamics {
namespace {

using namespace literals;

constexpr double kRho = 1.225;  // kg/m³, the test's site (ISA sea level)
constexpr double kGravity = 9.80665;

/// A point mass falling or held, at 1 ms: integrates the aerodynamic force and gravity.
class Body final : public IModule {
public:
    Body(double mass, bool held) : mass_(mass), held_(held) {}
    std::array<double, 3> position{0.0, 0.0, 500.0};
    std::array<double, 3> velocity{};
    std::array<double, 6> lastWrench{};
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "body", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec p;
        p.name = "body/ball/pose";
        p.type = SignalType::Pose;
        p.unit = units::m;
        p.frame = "world_local";
        pose_ = d.output(p);
        SignalSpec t;
        t.name = "body/ball/twist";
        t.type = SignalType::Wrench;
        t.unit = units::m_per_s;
        t.frame = "world_local";
        twist_ = d.output(t);
        SignalSpec w;
        w.name = "body/ball/wrench/aero";
        w.type = SignalType::Wrench;
        w.unit = units::N;
        w.frame = "world_local";
        w.hold = Hold::WindowAverage;
        wrench_ = d.input(w);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        const auto w = bus.readN(wrench_);
        std::copy(w.begin(), w.end(), lastWrench.begin());
        if (!held_) {
            const double dt = c.dt.seconds();
            for (std::size_t k = 0; k < 3; ++k) {
                velocity[k] += (w[k] / mass_ - (k == 2 ? kGravity : 0.0)) * dt;
                position[k] += velocity[k] * dt;
            }
        }
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus) const {
        bus.writeN(pose_, std::array<double, 7>{position[0], position[1], position[2], 1.0, 0.0,
                                                0.0, 0.0});
        bus.writeN(twist_,
                   std::array<double, 6>{velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0});
    }
    double mass_;
    bool held_;
    SignalId pose_, twist_, wrench_;
};

/// A free sphere of radius 0.1 m with AeroAPI_1 (Cd 0.47 on every axis), over a ground plane.
World sphere(double mass) {
    World w;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    const BodyId ball =
        w.addBody("ball", kWorldBody, Pose{}, mass, {}, {0.01, 0.01, 0.01, 0, 0, 0});
    w.addJoint("ball_free", JointType::Free, ball, {0, 0, 1}, false);
    w.schemaInstances.push_back({});
    SchemaInstance& aero = w.schemaInstances.back();
    aero.schema = "AeroAPI_1";
    aero.prim = w.bodies.prim.back();
    const double area = std::numbers::pi * 0.1 * 0.1;
    aero.realArrays = {{"aero:referenceArea", {area, area, area}},
                       {"aero:dragCoeff", {0.47, 0.47, 0.47}}};
    return w;
}

TEST(Aerodynamics, AFallingSphereReachesItsTerminalVelocity) {
    constexpr double kMass = 0.5;
    const World world = sphere(kMass);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto body = std::make_unique<Body>(kMass, false);
    Body* ball = body.get();
    scheduler.add(std::move(body));
    ModuleConfigData air;
    air.values["air_density"] = kRho;
    scheduler.add(std::make_unique<AeroDrag>(), ModuleConfig(air));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{12'000'000'000}).ok());
    const double area = std::numbers::pi * 0.1 * 0.1;
    const double terminal = std::sqrt(2.0 * kMass * kGravity / (kRho * 0.47 * area));
    EXPECT_NEAR(-ball->velocity[2], terminal, 0.005 * terminal);
}

TEST(Aerodynamics, AHeldSphereFeelsTheMeanWindFromTheProfile) {
    const World world = sphere(1.0);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto body = std::make_unique<Body>(1.0, true);
    Body* ball = body.get();
    ball->position = {0.0, 0.0, 30.0};
    scheduler.add(std::move(body));
    ModuleConfigData config;
    config.values["air_density"] = kRho;
    config.values["wind_speed"] = 8.0;  // m/s at 10 m
    config.values["wind_from"] = 90.0;  // from the east: it blows towards −x
    config.values["wind_reference_height"] = 10.0;
    config.values["roughness_length"] = 0.03;  // m, open flat terrain
    scheduler.add(std::make_unique<AeroDrag>(), ModuleConfig(config));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{50'000'000}).ok());
    const double u = 8.0 * std::log(30.0 / 0.03) / std::log(10.0 / 0.03);
    EXPECT_NEAR(meanWindAt(AeroDrag::windOf(ModuleConfig(config)).value(), 30.0), u, 1e-12);
    const double area = std::numbers::pi * 0.1 * 0.1;
    EXPECT_NEAR(ball->lastWrench[0], -0.5 * kRho * 0.47 * area * u * u, 1e-9);
    EXPECT_NEAR(ball->lastWrench[1], 0.0, 1e-12);
}

TEST(Aerodynamics, DrydenTurbulenceHasItsSpecifiedVarianceAndCorrelation) {
    // 50 m above ground, W20 = 10 m/s, airspeed 8 m/s. The filters are exact for any step, so
    // 20 Hz for 28 h: about 4000 longitudinal correlation times (Lu/V ≈ 25 s).
    constexpr double kDt = 0.05;
    constexpr double kHeight = 50.0;
    constexpr double kAirspeed = 8.0;
    constexpr double kW20 = 10.0;
    constexpr std::size_t kSteps = 2'000'000;
    Dryden dryden;
    random::Stream rng(20261025, 0, 0);
    double su = 0.0, sv = 0.0, sw = 0.0, lagged = 0.0, previous = 0.0;
    const std::size_t lagSteps = 200;  // 10 s
    std::vector<double> history(lagSteps, 0.0);
    for (std::size_t k = 0; k < kSteps; ++k) {
        const std::array<double, 5> n{rng.normal(), rng.normal(), rng.normal(), rng.normal(),
                                      rng.normal()};
        const std::array<double, 3> g = dryden.step(kDt, kHeight, kAirspeed, kW20, n);
        su += g[0] * g[0];
        sv += g[1] * g[1];
        sw += g[2] * g[2];
        previous = history[k % lagSteps];
        lagged += g[0] * previous;
        history[k % lagSteps] = g[0];
    }
    const double h = kHeight / 0.3048;
    const double sigmaW = 0.1 * kW20;
    const double sigmaU = sigmaW / std::pow(0.177 + 0.000823 * h, 0.4);
    const auto n = static_cast<double>(kSteps);
    EXPECT_NEAR(std::sqrt(su / n), sigmaU, 0.05 * sigmaU);
    EXPECT_NEAR(std::sqrt(sv / n), sigmaU, 0.05 * sigmaU);
    EXPECT_NEAR(std::sqrt(sw / n), sigmaW, 0.05 * sigmaW);
    // Longitudinal autocorrelation at 10 s: e^(−τ·V/Lu).
    const double lu = h / std::pow(0.177 + 0.000823 * h, 1.2) * 0.3048;  // m
    const double expected = std::exp(-10.0 * kAirspeed / lu);
    EXPECT_NEAR(lagged / su, expected, 0.03);
}

TEST(Aerodynamics, RefusesWithoutTheAirDensityOrAnIncompleteWind) {
    const World world = sphere(1.0);
    Scheduler noDensity;
    noDensity.add(std::make_unique<Body>(1.0, true));
    noDensity.add(std::make_unique<AeroDrag>());
    EXPECT_FALSE(noDensity.wire(WorldView(world)));
    ModuleConfigData config;
    config.values["air_density"] = kRho;
    config.values["wind_speed"] = 5.0;  // no direction, height or roughness
    EXPECT_FALSE(AeroDrag::windOf(ModuleConfig(config)));
}

}  // namespace
}  // namespace lagrange::aerodynamics
