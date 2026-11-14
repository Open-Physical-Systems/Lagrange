// The IMU error model (roadmap 0.2-D2): white noise and rate random walk checked by their Allan
// deviation, the Gauss-Markov bias by its spread and correlation, turn-on bias, saturation,
// quantisation, and the bias surviving a save and restore.
#include "lagrange/module/StateIO.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

constexpr double kGravity = 9.80665;  // World::gravityLocal default
constexpr double kRateHz = 100.0;
constexpr double kDt = 1.0 / kRateHz;

/// A body held still (or spinning about z at `spin`), feeding the IMU its inputs.
class Still final : public IModule {
public:
    explicit Still(double spin) : spin_(spin) {}
    [[nodiscard]] ModuleInfo info() const override { return {"rigid_body", "still"}; }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        pose_ = d.output(spec("body/box/pose", SignalType::Pose, units::m, "world_local"));
        twist_ =
            d.output(spec("body/box/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        accel_ =
            d.output(spec("body/box/acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    static SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame) {
        SignalSpec out;
        out.name = std::move(name);
        out.type = type;
        out.unit = unit;
        out.frame = std::move(frame);
        return out;
    }
    void write(SignalBus& bus) const {
        bus.writeN(pose_, std::array<double, 7>{0, 0, 0, 1, 0, 0, 0});
        bus.writeN(twist_, std::array<double, 6>{0, 0, 0, 0, 0, spin_});
        bus.writeN(accel_, std::array<double, 6>{0, 0, kGravity, 0, 0, 0});  // proper: up
    }
    double spin_;
    SignalId pose_, twist_, accel_;
};

World imuWorld(std::map<std::string, double, std::less<>> settings) {
    World world;
    const BodyId box = world.addBody("box", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    world.addJoint("box_free", JointType::Free, box, {0, 0, 1}, false);
    world.addSensor("imu", SensorType::Imu, box, Pose{}, kRateHz, std::move(settings));
    return world;
}

struct Record {
    std::vector<std::array<double, 3>> gyro, accel;
};

std::unique_ptr<Scheduler> start(const World& world, std::uint64_t seed, double spin = 0.0) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.runSeed = seed});
    scheduler->add(std::make_unique<Still>(spin));
    for (auto& module : createSensorModules(world)) {
        scheduler->add(std::move(module));
    }
    auto wired = scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return scheduler;
}

void run(Scheduler& scheduler, std::size_t samples, Record& out) {
    const SignalBus& bus = scheduler.bus();
    const SignalId gyro = bus.find("sensor/imu/angular_rate");
    const SignalId accel = bus.find("sensor/imu/specific_force");
    for (std::size_t k = 0; k < samples; ++k) {
        ASSERT_TRUE(scheduler.stepTick().ok());
        const auto g = bus.readN(gyro);
        const auto a = bus.readN(accel);
        out.gyro.push_back({g[0], g[1], g[2]});
        out.accel.push_back({a[0], a[1], a[2]});
    }
}

/// Overlapping Allan variance of rate samples at cluster time m·dt (IEEE Std 952, annex C).
double allanVariance(const std::vector<std::array<double, 3>>& samples, std::size_t axis,
                     std::size_t m) {
    std::vector<double> theta(samples.size() + 1, 0.0);  // integrated rate
    for (std::size_t k = 0; k < samples.size(); ++k) {
        theta[k + 1] = theta[k] + samples[k][axis] * kDt;
    }
    const std::size_t n = theta.size();
    double sum = 0.0;
    for (std::size_t k = 0; k + 2 * m < n; ++k) {
        const double d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k];
        sum += d * d;
    }
    const double tau = static_cast<double>(m) * kDt;
    return sum / (2.0 * tau * tau * static_cast<double>(n - 2 * m));
}

// Acceptance (roadmap 0.2-D2): the Allan deviation of a 4-hour static record follows the
// configured noise density N (slope -1/2) and rate random walk K (slope +1/2):
// sigma^2(tau) = N^2 / tau + K^2 tau / 3 (IEEE Std 952, table C.1). The deviation of the
// estimate grows with tau as fewer clusters fit in the record, hence the wider bound there; the
// three axes are averaged.
TEST(ImuNoise, AllanDeviationOfAStaticRecordMatchesTheDensityAndRandomWalk) {
    constexpr double kDensity = 0.0035;  // rad/s/sqrt(Hz)
    constexpr double kWalk = 2e-5;       // rad/s/sqrt(s); crossover sqrt(3) N / K = 303 s
    constexpr double kGaussMarkov = 0.01;
    constexpr double kCorrelationS = 50.0;
    const World world = imuWorld({{"gyro_noise_density", kDensity},
                                  {"gyro_random_walk", kWalk},
                                  {"accel_dynamic_bias_stddev", kGaussMarkov},
                                  {"accel_dynamic_bias_correlation_time", kCorrelationS}});
    auto scheduler = start(world, 7);
    Record record;
    run(*scheduler, static_cast<std::size_t>(4 * 3600 * kRateHz), record);

    struct Point {
        double tau, tolerance;
    };
    for (const Point point : {Point{0.1, 0.03}, Point{1.0, 0.05}, Point{10.0, 0.08},
                              Point{100.0, 0.15}, Point{1000.0, 0.30}}) {
        const auto m = static_cast<std::size_t>(std::llround(point.tau / kDt));
        double variance = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            variance += allanVariance(record.gyro, axis, m) / 3.0;
        }
        const double expected =
            std::sqrt(kDensity * kDensity / point.tau + kWalk * kWalk * point.tau / 3.0);
        const double measured = std::sqrt(variance);
        std::printf("tau %7.1f s: Allan deviation %.4g rad/s, model %.4g (%+.1f %%)\n", point.tau,
                    measured, expected, 100.0 * (measured / expected - 1.0));
        EXPECT_NEAR(measured / expected, 1.0, point.tolerance) << "tau " << point.tau << " s";
    }

    // The accelerometer's Gauss-Markov bias: <dynamic_bias_stddev> drives it, as Gazebo reads it
    // (gz-sensors GaussianNoiseModel.cc), so its spread is stddev·sqrt(tau / 2), and its
    // autocorrelation at one correlation time is 1/e. 4 h hold 144 correlation times,
    // so the spread is known to about 1/sqrt(144) = 8 %.
    double spread = 0.0, lagged = 0.0;
    const auto lag = static_cast<std::size_t>(kCorrelationS / kDt);
    std::size_t pairs = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        for (std::size_t k = 0; k < record.accel.size(); ++k) {
            const double b = record.accel[k][axis] - (axis == 2 ? kGravity : 0.0);
            spread += b * b;
            if (k + lag < record.accel.size()) {
                const double later = record.accel[k + lag][axis] - (axis == 2 ? kGravity : 0.0);
                lagged += b * later;
                ++pairs;
            }
        }
    }
    spread = std::sqrt(spread / (3.0 * static_cast<double>(record.accel.size())));
    const double correlation = lagged / static_cast<double>(pairs) / (spread * spread);
    std::printf("Gauss-Markov: deviation %.4g (model %.4g), correlation at tau_c %.3f (1/e %.3f)\n",
                spread, kGaussMarkov * std::sqrt(kCorrelationS / 2.0), correlation, std::exp(-1.0));
    EXPECT_NEAR(spread / (kGaussMarkov * std::sqrt(kCorrelationS / 2.0)), 1.0, 0.2);
    EXPECT_NEAR(correlation, std::exp(-1.0), 0.1);
}

TEST(ImuNoise, TurnOnBiasIsConstantWithARandomSign) {
    const World world = imuWorld({{"gyro_bias_mean", 0.05}});
    int positive = 0;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        auto scheduler = start(world, seed);
        Record record;
        run(*scheduler, 50, record);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(std::abs(record.gyro[0][axis]), 0.05, 1e-15);
            EXPECT_EQ(record.gyro.back()[axis], record.gyro[0][axis]) << "constant in a run";
            positive += record.gyro[0][axis] > 0.0 ? 1 : 0;
        }
    }
    EXPECT_GT(positive, 40);  // of 120: either sign, about half each
    EXPECT_LT(positive, 80);
}

TEST(ImuNoise, SaturatesAtTheRangeAndQuantisesToThePrecision) {
    const World world =
        imuWorld({{"gyro_range", 1.5}, {"accel_noise", 0.05}, {"accel_precision", 0.01}});
    auto scheduler = start(world, 3, 2.0);  // spinning at 2 rad/s, beyond the range
    Record record;
    run(*scheduler, 100, record);
    for (const auto& g : record.gyro) {
        EXPECT_EQ(g[2], 1.5);
    }
    for (const auto& a : record.accel) {
        for (const double v : a) {
            EXPECT_NEAR(v / 0.01, std::round(v / 0.01), 1e-9) << v;
        }
    }
}

TEST(ImuNoise, TheBiasIsPartOfTheSavedState) {
    const World world = imuWorld({{"gyro_bias_stddev", 0.02},
                                  {"gyro_random_walk", 1e-3},
                                  {"gyro_dynamic_bias_stddev", 0.01},
                                  {"gyro_dynamic_bias_correlation_time", 5.0}});
    auto scheduler = start(world, 11);
    Record before;
    run(*scheduler, 200, before);
    StateWriter writer;
    ASSERT_TRUE(scheduler->saveState(writer).ok());
    Record first;
    run(*scheduler, 300, first);
    StateReader reader(writer.bytes());
    ASSERT_TRUE(scheduler->restoreState(reader).ok());
    Record second;
    run(*scheduler, 300, second);
    EXPECT_EQ(first.gyro, second.gyro);
    EXPECT_NE(first.gyro.front(), first.gyro.back()) << "the bias moved";
}

}  // namespace
}  // namespace lagrange::sensors
