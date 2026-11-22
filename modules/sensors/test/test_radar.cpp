// The radar (domains/sensors.md): the radar equation, a flat plate's cross-section as ρ·area,
// its Doppler when it recedes, and the detection threshold at the range the equation gives.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Radar.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <vector>

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

/// The weather's rain: env/rain_rate, m/s.
class Rain final : public IModule {
public:
    explicit Rain(double mmPerHour) : rate_(mmPerHour / 3.6e6) {}
    ModuleInfo info() const override { return {"weather", "rain", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "env/rain_rate";
        s.unit = units::m_per_s;
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(out_, rate_);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(out_, rate_);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double rate_;
    SignalId out_;
};

struct Run {
    World world;
    std::unique_ptr<Scheduler> scheduler;
};

/// A radar 1 m up looking along +x at a 2 m × 1 m plate, fine rays (0.01 rad resolution).
std::unique_ptr<Run> run(double start, double speed, double noiseFloor,
                         const std::shared_ptr<gpu::RayContext>& rayContext = nullptr,
                         bool clutter = false, double rainMmPerHour = 0.0) {
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
    if (clutter) {
        // A fixed sphere and a tilted box off to the sides, and the ground (dropped as clutter).
        Pose ball;
        ball.position = {12.0, 2.5, 1.0};
        w.addCollider(kWorldBody, ShapeType::Sphere, {0.6, 0.0, 0.0}, ball, 0.8);
        Pose crate;
        crate.position = {30.0, -4.0, 1.2};
        crate.orientation = {std::cos(0.3), 0.0, 0.0, std::sin(0.3)};
        w.addCollider(kWorldBody, ShapeType::Box, {0.8, 0.8, 0.8}, crate, 0.8);
        w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 0.8);
    }
    r->scheduler =
        std::make_unique<Scheduler>(SchedulerOptions{.deterministic = rayContext == nullptr});
    r->scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    r->scheduler->add(std::make_unique<Plate>(start, speed));
    if (rainMmPerHour > 0.0) {
        r->scheduler->add(std::make_unique<Rain>(rainMmPerHour));
    }
    ModuleConfigData config;
    config.tier = rayContext ? CapabilityTier::Tier1 : CapabilityTier::Tier0;
    r->scheduler->add(createRadar(w, 0, false, rayContext), ModuleConfig(config));
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

// ITU-R P.838-3: its equations (2) and (3) with Tables 1–4 give the coefficients its Table 5 lists
// (rounded there to four or five figures).
TEST(Radar, RainCoefficientsAreItuRP838s) {
    struct Row {
        double ghz, kH, aH, kV, aV;
    };
    for (const Row& row :
         {Row{10.0, 0.01217, 1.2571, 0.01129, 1.2156}, Row{30.0, 0.2403, 0.9485, 0.2291, 0.9129},
          Row{60.0, 0.8606, 0.7656, 0.8515, 0.7486}, Row{77.0, 1.1320, 0.7177, 1.1276, 0.7073},
          Row{79.0, 1.1579, 0.7135, 1.1541, 0.7038}, Row{100.0, 1.3671, 0.6815, 1.3680, 0.6765}}) {
        // Horizontal: τ = 0 on a level path; vertical: τ = 90°.
        const RainCoefficients h = rainCoefficients(row.ghz * 1e9, 0.0, 0.0);
        const RainCoefficients v = rainCoefficients(row.ghz * 1e9, 0.0, std::numbers::pi / 2.0);
        EXPECT_NEAR(h.k, row.kH, 0.002 * row.kH) << row.ghz << " GHz";
        EXPECT_NEAR(h.alpha, row.aH, 0.002 * row.aH) << row.ghz << " GHz";
        EXPECT_NEAR(v.k, row.kV, 0.002 * row.kV) << row.ghz << " GHz";
        EXPECT_NEAR(v.alpha, row.aV, 0.002 * row.aV) << row.ghz << " GHz";
    }
    // 77 GHz in 25 mm/h, at 45°: γ = k·R^α with equations (4) and (5) from the table's values.
    const double k = (1.1320 + 1.1276) / 2.0;
    const double alpha = (1.1320 * 0.7177 + 1.1276 * 0.7073) / (2.0 * k);
    const double gamma = k * std::pow(25.0, alpha);
    EXPECT_NEAR(rainAttenuation(77e9, 25.0, 0.0, std::numbers::pi / 4.0), gamma, 0.003 * gamma);
    std::printf("77 GHz, 25 mm/h: %.2f dB/km\n", gamma);
    EXPECT_EQ(rainAttenuation(77e9, 0.0, 0.0, 0.0), 0.0);
}

// In rain, an echo comes back weaker by the two-way path loss 10^(−2·γ·R/10⁴).
TEST(Radar, RainTakesTheTwoWayPathLossOffAnEcho) {
    auto dry = run(40.0, 0.0, 1e-15);
    auto wet = run(40.0, 0.0, 1e-15, nullptr, false, 50.0);
    ASSERT_TRUE(dry && wet);
    ASSERT_TRUE(dry->scheduler->stepUntil(SimTime{120'000'000}).ok());
    ASSERT_TRUE(wet->scheduler->stepUntil(SimTime{120'000'000}).ok());
    const SignalBus& a = dry->scheduler->bus();
    const SignalBus& b = wet->scheduler->bus();
    ASSERT_EQ(a.read(a.find("sensor/front_radar/count")), 1.0);
    ASSERT_EQ(b.read(b.find("sensor/front_radar/count")), 1.0);
    const std::span<const double> d0 = a.readN(a.find("sensor/front_radar/detections"));
    const std::span<const double> d1 = b.readN(b.find("sensor/front_radar/detections"));
    const double gamma = rainAttenuation(77e9, 50.0, 0.0, std::numbers::pi / 4.0);
    const double expected = std::pow(10.0, -2.0 * gamma * d0[0] / 1e4);
    std::printf("50 mm/h at 77 GHz: %.2f dB/km; at %.2f m the echo keeps %.4f (expected %.4f)\n",
                gamma, d0[0], d1[5] / d0[5], expected);
    // (The reported ranges carry measurement noise, larger on the weaker echo, as on a real
    // radar; the loss is taken at the cell's own range.)
    EXPECT_NEAR(d1[5] / d0[5], expected, 1e-3 * expected);
    EXPECT_NEAR(d1[0], d0[0], 0.05);  // the same target, weaker
}

/// FNV-1a over the bits of every detection and the count.
std::uint64_t fingerprint(const SignalBus& bus) {
    std::uint64_t h = 1469598103934665603ULL;
    const auto mix = [&h](double v) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        for (int i = 0; i < 8; ++i) {
            h = (h ^ ((bits >> (8 * i)) & 0xFFU)) * 1099511628211ULL;
        }
    };
    mix(bus.read(bus.find("sensor/front_radar/count")));
    for (const double v : bus.readN(bus.find("sensor/front_radar/detections"))) {
        mix(v);
    }
    return h;
}

// The RadarAPI_1 radar's numbers did not change when the imaging radar (kind imaging_radar) came
// to share its code: the CPU reference's detections, bit for bit, as v1.2.1 gave them.
TEST(Radar, TheCpuReferenceKeepsItsNumbers) {
    auto r = run(20.0, 5.0, 1e-15, nullptr, true);
    ASSERT_TRUE(r);
    std::vector<std::uint64_t> got;
    for (const std::int64_t t : {120'000'000, 520'000'000}) {
        ASSERT_TRUE(r->scheduler->stepUntil(SimTime{t}).ok());
        got.push_back(fingerprint(r->scheduler->bus()));
    }
    EXPECT_GE(r->scheduler->bus().read(r->scheduler->bus().find("sensor/front_radar/count")), 3.0);
    EXPECT_EQ(got[0], 0x8000457c4b9153edULL);
    EXPECT_EQ(got[1], 0x7b9ade2ee822847fULL);
}

const std::shared_ptr<gpu::RayContext>& context() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        auto created = gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
        if (!created) {
            std::fprintf(stderr, "no Vulkan device for the GPU radar: %s\n", error.c_str());
        }
        return created;
    }();
    return made;
}

// The GPU rays (float ranges) find what the CPU reference finds: the same detections, ranges
// within 5 mm, cross-sections within 1 %, Doppler within 1 cm/s.
TEST(Radar, OnTheGpuAgreesWithTheCpuReference) {
    if (!context()) {
        GTEST_SKIP() << "no Vulkan device";
    }
    auto cpu = run(20.0, 5.0, 1e-15, nullptr, true);
    auto gpu = run(20.0, 5.0, 1e-15, context(), true);
    ASSERT_TRUE(cpu && gpu);
    for (const std::int64_t t : {120'000'000, 520'000'000}) {
        ASSERT_TRUE(cpu->scheduler->stepUntil(SimTime{t}).ok());
        ASSERT_TRUE(gpu->scheduler->stepUntil(SimTime{t}).ok());
        const SignalBus& a = cpu->scheduler->bus();
        const SignalBus& b = gpu->scheduler->bus();
        const double count = a.read(a.find("sensor/front_radar/count"));
        ASSERT_GE(count, 3.0);  // the plate, the ball, the crate
        ASSERT_EQ(b.read(b.find("sensor/front_radar/count")), count);
        const std::span<const double> x = a.readN(a.find("sensor/front_radar/detections"));
        const std::span<const double> y = b.readN(b.find("sensor/front_radar/detections"));
        for (std::size_t i = 0; i < static_cast<std::size_t>(count); ++i) {
            const std::size_t k = i * kRadarFields;
            // Noise of resolution/√(2·SNR) is drawn the same on both: ranges keep their order.
            EXPECT_NEAR(y[k + 0], x[k + 0], 5e-3) << i;
            EXPECT_NEAR(y[k + 1], x[k + 1], 1e-3) << i;
            EXPECT_NEAR(y[k + 2], x[k + 2], 1e-3) << i;
            EXPECT_NEAR(y[k + 3], x[k + 3], 1e-2) << i;
            EXPECT_NEAR(y[k + 4], x[k + 4], 0.01 * x[k + 4]) << i;
            EXPECT_NEAR(y[k + 5], x[k + 5], 0.02 * x[k + 5]) << i;
        }
    }
}

}  // namespace
}  // namespace lagrange::sensors
