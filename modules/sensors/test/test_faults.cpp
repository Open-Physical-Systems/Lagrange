// Sensor faults (sensors.md "Sensor faults (as built)"): each mode on a lidar, an IMU and a
// stand-in sensor with a buffer, determinism, saved states and the ground-truth substitute.
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Faults.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

constexpr double kGravity = 9.80665;  // World::gravityLocal default

Pose at(double x, double y, double z) {
    return Pose{{x, y, z}};
}

SensorFault fault(const std::string& text) {
    auto parsed = parseSensorFault(text);
    EXPECT_TRUE(parsed) << (parsed ? "" : parsed.error());
    return parsed ? *parsed : SensorFault{};
}

/// A 6 m x 4 m room and a 10 Hz lidar on a post at (1, 0.5, 0.5): four beams, 0°, 90°, 180°
/// and 270°, which read 2, 1.5, 4 and 2.5 m without noise.
World room(double noise) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, at(0, 2.1, 1), 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, at(0, -2.1, 1), 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, at(3.1, 0, 1), 1.0);
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, at(-3.1, 0, 1), 1.0);
    const BodyId post = world.addBody("post", kWorldBody, at(1, 0.5, 0), 0.0, {}, {});
    world.addSensor("scanner", SensorType::Lidar, post, at(0, 0, 0.5), 10.0,
                    {{"samples", 4.0},
                     {"min_angle", 0.0},
                     {"max_angle", 1.5 * std::numbers::pi},
                     {"min_range", 0.1},
                     {"max_range", 10.0},
                     {"range_noise", noise}});
    return world;
}
const std::vector<double> kRoom{2.0, 1.5, 4.0, 2.5};

/// A run of `world`'s sensors (real or ground-truth substitutes) with `faults`.
std::unique_ptr<Scheduler> run(const World& world, const std::vector<SensorFault>& faults,
                               std::uint64_t seed = 0, bool substitutes = false) {
    auto scheduler = std::make_unique<Scheduler>(SchedulerOptions{.runSeed = seed});
    for (auto& module : substitutes ? createSensorSubstitutes(world) : createSensorModules(world)) {
        scheduler->add(withFaults(std::move(module), faults));
    }
    auto wired = scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return scheduler;
}

/// Each lidar sample's ranges and stamp, every 100 ms from 0.1 s to `seconds`.
struct Scan {
    std::int64_t stamp{0};
    std::vector<double> ranges;
    double active{0.0}, dropped{0.0};
};
std::vector<Scan> scans(Scheduler& scheduler, double seconds) {
    std::vector<Scan> out;
    EXPECT_EQ(checkFaultsWired(scheduler.bus(), std::vector{fault("scanner:ranges=stuck")}), "");
    const SignalBus& bus = scheduler.bus();
    const SignalId ranges = bus.find("sensor/scanner/ranges");
    const SignalId active = bus.find("sensor/scanner/fault/active");
    const SignalId dropped = bus.find("sensor/scanner/fault/dropped");
    for (int k = 1; k <= static_cast<int>(std::lround(seconds * 10.0)); ++k) {
        EXPECT_TRUE(scheduler.stepUntil(SimTime{k * 100'000'000LL}).ok());
        const std::span<const double> r = bus.readN(ranges);
        out.push_back({bus.stamp(ranges).ns,
                       {r.begin(), r.end()},
                       active.valid() ? bus.read(active) : -1.0,
                       dropped.valid() ? bus.read(dropped) : -1.0});
    }
    return out;
}

// --- parsing -------------------------------------------------------------------------------

TEST(SensorFaults, ParseTheCommandLineForm) {
    const SensorFault bias = fault("imu:angular_rate=bias,value=0.02,at=5,component=2");
    EXPECT_EQ(bias.sensor, "imu");
    EXPECT_EQ(bias.quantity, "angular_rate");
    EXPECT_EQ(bias.mode, FaultMode::Bias);
    EXPECT_DOUBLE_EQ(bias.value, 0.02);
    EXPECT_DOUBLE_EQ(bias.at, 5.0);
    EXPECT_TRUE(std::isinf(bias.duration));
    EXPECT_EQ(bias.component, 2);
    const SensorFault drop = fault("front_lidar=dropout,at=1.5,for=2,probability=0.25,nan=true");
    EXPECT_EQ(drop.quantity, "*");
    EXPECT_DOUBLE_EQ(drop.duration, 2.0);
    EXPECT_DOUBLE_EQ(drop.probability, 0.25);
    EXPECT_TRUE(drop.nan);
    // Written back, read back the same.
    for (const SensorFault& f :
         {bias, drop, fault("gps:geodetic=drift,rate=-1e-7"), fault("imu=stuck,at=3,for=0.5"),
          fault("imu=disconnect,at=9"), fault("mag:magnetic_field=noise,sigma=2e-6,for=1")}) {
        const SensorFault again = fault(to_string(f));
        EXPECT_EQ(to_string(again), to_string(f));
        EXPECT_EQ(again.mode, f.mode);
        EXPECT_EQ(again.duration, f.duration);
    }
}

TEST(SensorFaults, RefuseWhatCannotWork) {
    for (const char* text : {"imu", "imu=", "imu=broken", "imu:angular_rate=bias",  // no value
                             "imu=bias,value=1",                      // bias of every quantity
                             "imu:angular_rate=noise,sigma=0",        // no noise
                             "imu=dropout,probability=0",             // never
                             "imu=dropout,probability=1.5",           // more than always
                             "imu=disconnect,for=2",                  // disconnect is for good
                             "imu=dropout,at=-1",                     // before the start
                             "imu=dropout,for=0",                     // no window
                             "imu:angular_rate=bias,value=1,rate=2",  // a key of another mode
                             "imu:angular_rate=bias,value=x",         // not a number
                             "imu:angular_rate=bias,value=1,component=1.5"}) {
        EXPECT_FALSE(parseSensorFault(text)) << text;
    }
}

TEST(SensorFaults, NoFaultsLeaveTheModuleAsItIs) {
    const World world = room(0.0);
    auto modules = createSensorModules(world);
    IModule* raw = modules.front().get();
    EXPECT_EQ(withFaults(std::move(modules.front()), {}).get(), raw);
    // No fault of its sensors: it runs as before, with no fault signals.
    auto scheduler = run(world, {fault("other=disconnect")});
    EXPECT_FALSE(scheduler->bus().find("sensor/scanner/fault/active").valid());
    EXPECT_FALSE(scheduler->bus().find("sensor/other/fault/active").valid());
    EXPECT_NE(checkFaultsWired(scheduler->bus(), std::vector{fault("other=disconnect")})
                  .find("no sensor other"),
              std::string::npos);
    EXPECT_NE(checkFaultsWired(scheduler->bus(), std::vector{fault("scanner:speed=stuck")})
                  .find("has no quantity speed; it has ranges"),
              std::string::npos);
}

TEST(SensorFaults, AWrappedSensorDrawsTheSameNoiseAsBefore) {
    // Its name, seed and step stay: until a fault starts, nothing differs from a run without.
    const World world = room(0.05);
    const std::vector<Scan> plain = scans(*run(world, {}, 5), 1.0);
    const std::vector<Scan> wrapped =
        scans(*run(world, {fault("scanner:ranges=bias,value=1,at=100")}, 5), 1.0);
    for (std::size_t k = 0; k < plain.size(); ++k) {
        EXPECT_EQ(wrapped[k].ranges, plain[k].ranges) << k;
        EXPECT_EQ(wrapped[k].stamp, plain[k].stamp) << k;
    }
}

// --- each mode on the lidar ------------------------------------------------------------------

TEST(SensorFaults, BiasIsAStepAtItsTime) {
    auto scheduler = run(room(0.0), {fault("scanner:ranges=bias,value=0.5,at=0.25")});
    const std::vector<Scan> s = scans(*scheduler, 0.5);
    for (std::size_t k = 0; k < s.size(); ++k) {
        const double offset = k >= 2 ? 0.5 : 0.0;  // samples at 0.1, 0.2 | 0.3, 0.4, 0.5
        for (std::size_t b = 0; b < 4; ++b) {
            EXPECT_NEAR(s[k].ranges[b], kRoom[b] + offset, 1e-12) << k << " " << b;
        }
        EXPECT_EQ(s[k].active, k >= 2 ? 1.0 : 0.0);
        EXPECT_EQ(s[k].stamp, static_cast<std::int64_t>(k + 1) * 100'000'000LL);
    }
}

TEST(SensorFaults, DriftRampsFromItsTimeOnOneComponent) {
    auto scheduler =
        run(room(0.0), {fault("scanner:ranges=drift,rate=1,at=0.2,for=0.25,component=2")});
    const std::vector<Scan> s = scans(*scheduler, 0.6);
    for (std::size_t k = 0; k < s.size(); ++k) {
        const double t = 0.1 * static_cast<double>(k + 1);
        const double drift = t >= 0.2 - 1e-9 && t < 0.45 ? t - 0.2 : 0.0;
        EXPECT_NEAR(s[k].ranges[2], 4.0 + drift, 1e-9) << t;
        EXPECT_NEAR(s[k].ranges[0], 2.0, 1e-12) << "other components stay";
    }
    EXPECT_EQ(s.back().active, 0.0) << "the window closed";
}

TEST(SensorFaults, DropoutWithholdsSamplesInItsWindow) {
    auto scheduler = run(room(0.0), {fault("scanner=dropout,at=0.25,for=0.2")});
    const std::vector<Scan> s = scans(*scheduler, 0.6);
    // Samples at 0.3 and 0.4 never come: the stamp stays at 0.2 s, readers keep the last value.
    EXPECT_EQ(s[1].stamp, 200'000'000);
    EXPECT_EQ(s[2].stamp, 200'000'000);
    EXPECT_EQ(s[3].stamp, 200'000'000);
    EXPECT_EQ(s[4].stamp, 500'000'000) << "back after the window";
    EXPECT_EQ(s[3].dropped, 2.0);
    EXPECT_EQ(s[5].dropped, 2.0);
    EXPECT_EQ(s[3].active, 1.0);
    EXPECT_EQ(s[4].active, 0.0);
    EXPECT_NEAR(s[4].ranges[0], 2.0, 1e-12);
}

TEST(SensorFaults, DropoutAsNanStillDelivers) {
    auto scheduler = run(room(0.0), {fault("scanner:ranges=dropout,at=0.25,for=0.1,nan=1")});
    const std::vector<Scan> s = scans(*scheduler, 0.4);
    EXPECT_EQ(s[2].stamp, 300'000'000);
    for (const double r : s[2].ranges) {
        EXPECT_TRUE(std::isnan(r));
    }
    EXPECT_NEAR(s[3].ranges[1], 1.5, 1e-12);
}

TEST(SensorFaults, DisconnectIsForGood) {
    auto scheduler = run(room(0.0), {fault("scanner=disconnect,at=0.3")});
    const std::vector<Scan> s = scans(*scheduler, 2.0);
    for (std::size_t k = 1; k < s.size(); ++k) {
        EXPECT_EQ(s[k].stamp, 200'000'000) << k;
    }
    EXPECT_EQ(s.back().dropped, 18.0);
    EXPECT_EQ(s.back().active, 1.0);
}

TEST(SensorFaults, StuckFreezesTheLastReading) {
    // With range noise every live scan differs; a stuck one repeats the last before the window.
    auto scheduler = run(room(0.02), {fault("scanner:ranges=stuck,at=0.25,for=0.3")});
    const std::vector<Scan> s = scans(*scheduler, 0.8);
    EXPECT_NE(s[0].ranges, s[1].ranges);
    for (std::size_t k = 2; k <= 4; ++k) {  // 0.3, 0.4, 0.5
        EXPECT_EQ(s[k].ranges, s[1].ranges) << k;
        EXPECT_EQ(s[k].stamp, static_cast<std::int64_t>(k + 1) * 100'000'000LL)
            << "a stuck sensor still delivers";
    }
    EXPECT_NE(s[5].ranges, s[1].ranges) << "live again after the window";
    EXPECT_NEAR(s[5].ranges[0], 2.0, 0.1);
}

TEST(SensorFaults, NoiseBurstHasItsSpreadAndRepeats) {
    World world = room(0.0);
    world.sensors.settings[0]["samples"] = 720.0;
    world.sensors.settings[0]["max_angle"] = std::numbers::pi;
    world.sensors.settings[0]["min_angle"] = -std::numbers::pi;
    const auto burst = [&](std::uint64_t seed) {
        auto scheduler =
            run(world, {fault("scanner:ranges=noise,sigma=0.1,at=0.15,for=0.1")}, seed);
        return scans(*scheduler, 0.3);
    };
    const std::vector<Scan> a = burst(7);
    const std::vector<Scan> exact = scans(*run(world, {}), 0.3);
    EXPECT_EQ(a[0].ranges, exact[0].ranges) << "before the burst";
    EXPECT_EQ(a[2].ranges, exact[2].ranges) << "after the burst";
    double squares = 0.0;
    for (std::size_t i = 0; i < 720; ++i) {
        squares += std::pow(a[1].ranges[i] - exact[1].ranges[i], 2);
    }
    EXPECT_NEAR(std::sqrt(squares / 720.0), 0.1, 0.012);
    EXPECT_EQ(burst(7)[1].ranges, a[1].ranges) << "same seed, same noise";
    EXPECT_NE(burst(8)[1].ranges, a[1].ranges);
}

TEST(SensorFaults, RandomDropoutsRepeatWithTheSeed) {
    const auto lost = [](std::uint64_t seed) {
        auto scheduler = run(room(0.0), {fault("scanner=dropout,probability=0.3")}, seed);
        std::vector<bool> missing;
        std::int64_t last = 0;
        for (const Scan& scan : scans(*scheduler, 40.0)) {
            missing.push_back(scan.stamp == last);
            last = scan.stamp;
        }
        return missing;
    };
    const std::vector<bool> a = lost(3);
    const auto count = static_cast<double>(std::count(a.begin(), a.end(), true));
    EXPECT_NEAR(count / 400.0, 0.3, 0.07);  // 400 draws: three sigma is 0.069
    EXPECT_EQ(lost(3), a);
    EXPECT_NE(lost(4), a);
}

TEST(SensorFaults, ASavedRunGoesOnTheSameWay) {
    const std::vector<SensorFault> faults{fault("scanner:ranges=stuck,at=0.5,for=0.4"),
                                          fault("scanner:ranges=noise,sigma=0.05,at=1.2"),
                                          fault("scanner=dropout,probability=0.4,at=1.0")};
    auto first = run(room(0.02), faults, 11);
    (void)scans(*first, 0.7);  // stuck since 0.5
    StateWriter saved;
    ASSERT_TRUE(first->saveState(saved).ok());
    const std::vector<Scan> straight = [&] {
        std::vector<Scan> out;
        for (int k = 8; k <= 25; ++k) {
            EXPECT_TRUE(first->stepUntil(SimTime{k * 100'000'000LL}).ok());
            const SignalId id = first->bus().find("sensor/scanner/ranges");
            const std::span<const double> r = first->bus().readN(id);
            out.push_back({first->bus().stamp(id).ns, {r.begin(), r.end()}});
        }
        return out;
    }();
    auto second = run(room(0.02), faults, 11);
    StateReader reader(saved.bytes());
    ASSERT_TRUE(second->restoreState(reader).ok());
    for (int k = 8; k <= 25; ++k) {
        ASSERT_TRUE(second->stepUntil(SimTime{k * 100'000'000LL}).ok());
        const SignalId id = second->bus().find("sensor/scanner/ranges");
        const std::span<const double> r = second->bus().readN(id);
        EXPECT_EQ(second->bus().stamp(id).ns, straight[k - 8].stamp) << k;
        EXPECT_EQ(std::vector<double>(r.begin(), r.end()), straight[k - 8].ranges) << k;
    }
}

TEST(SensorFaults, TheGroundTruthSubstituteIsFaultedToo) {
    // A fault is in the wiring, not in the sensor's model: exact values, then the fault.
    auto scheduler =
        run(room(0.05), {fault("scanner:ranges=bias,value=1,at=0.15")}, 0, /*substitutes=*/true);
    const std::vector<Scan> s = scans(*scheduler, 0.3);
    EXPECT_TRUE(std::isinf(s[0].ranges[0])) << "no rendering: no returns";
    EXPECT_TRUE(std::isinf(s[1].ranges[0]));
    EXPECT_EQ(s[1].active, 1.0);
    auto stuck = run(room(0.05), {fault("scanner=disconnect,at=0.15")}, 0, true);
    const std::vector<Scan> d = scans(*stuck, 0.3);
    EXPECT_EQ(d[2].stamp, 100'000'000);
}

// --- the IMU on a box at rest ------------------------------------------------------------------

std::unique_ptr<Scheduler> imuRun(const std::vector<SensorFault>& faults) {
    static World world = [] {
        World w;
        w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, 1.0);
        const BodyId box = w.addBody("box", kWorldBody, at(0, 0, 0.1), 1.0, {},
                                     {0.0067, 0.0067, 0.0067, 0.0, 0.0, 0.0});
        w.addJoint("box_free", JointType::Free, box, {0, 0, 1}, false);
        w.addCollider(box, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, 1.0);
        w.addSensor("imu", SensorType::Imu, box, Pose{}, 200.0, {});
        return w;
    }();
    auto scheduler = std::make_unique<Scheduler>();
    scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    for (auto& module : createSensorModules(world)) {
        scheduler->add(withFaults(std::move(module), faults));
    }
    auto wired = scheduler->wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return scheduler;
}

TEST(SensorFaults, AnImuBiasJumpAndADropoutOfTheWholeSensor) {
    auto scheduler = imuRun({fault("imu:angular_rate=bias,value=0.05,at=0.5,component=2"),
                             fault("imu:specific_force=bias,value=-0.3,at=0.5"),
                             fault("imu=dropout,at=0.8,for=0.1")});
    const SignalBus& bus = scheduler->bus();
    const SignalId rate = bus.find("sensor/imu/angular_rate");
    const SignalId force = bus.find("sensor/imu/specific_force");
    ASSERT_TRUE(scheduler->stepUntil(SimTime{400'000'000}).ok());
    EXPECT_NEAR(bus.readN(rate)[2], 0.0, 1e-3);
    EXPECT_NEAR(bus.readN(force)[2], kGravity, 0.01 * kGravity);
    ASSERT_TRUE(scheduler->stepUntil(SimTime{700'000'000}).ok());
    EXPECT_NEAR(bus.readN(rate)[2], 0.05, 1e-3);
    EXPECT_NEAR(bus.readN(rate)[0], 0.0, 1e-3) << "one component only";
    EXPECT_NEAR(bus.readN(force)[0], -0.3, 0.02);
    EXPECT_NEAR(bus.readN(force)[2], kGravity - 0.3, 0.01 * kGravity);
    // Inside the dropout both quantities stop together, at the last sample before it.
    ASSERT_TRUE(scheduler->stepUntil(SimTime{850'000'000}).ok());
    EXPECT_EQ(bus.stamp(rate).ns, 795'000'000);
    EXPECT_EQ(bus.stamp(force).ns, 795'000'000);
    EXPECT_EQ(bus.read(bus.find("sensor/imu/fault/active")), 1.0);
    ASSERT_TRUE(scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_EQ(bus.stamp(rate).ns, 1'000'000'000);
    EXPECT_EQ(bus.read(bus.find("sensor/imu/fault/dropped")), 20.0);  // 0.8 to 0.895 at 200 Hz
}

// --- a sensor with a buffer -------------------------------------------------------------------

/// Writes its step number as `sensor/cam/image` (one byte) and `sensor/cam/count`, every 10 ms.
class Counter final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"sensors", "counter"}; }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec image;
        image.name = "sensor/cam/image";
        image.type = SignalType::Image;
        image.unit = units::one;
        image_ = d.output(image);
        SignalSpec count;
        count.name = "sensor/cam/count";
        count.type = SignalType::Int;
        count.unit = units::one;
        count_ = d.output(count);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext& context, SignalBus& bus) override {
        auto buffer = std::make_shared<Buffer>();
        buffer->width = 1;
        buffer->height = 1;
        buffer->encoding = "bytes";
        buffer->bytes = {static_cast<std::byte>(context.stepIndex)};
        bus.publishBuffer(image_, std::move(buffer));
        bus.write(count_, static_cast<double>(context.stepIndex));
        return {};
    }

private:
    SignalId image_, count_;
};

TEST(SensorFaults, BuffersCanBeStuckOrDroppedButNotBiased) {
    const World world;
    {
        Scheduler scheduler;
        scheduler.add(withFaults(std::make_unique<Counter>(), {fault("cam=stuck,at=0.05")}));
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
        const SignalBus& bus = scheduler.bus();
        const SignalId image = bus.find("sensor/cam/image");
        for (int k = 1; k <= 12; ++k) {
            ASSERT_TRUE(scheduler.stepUntil(SimTime{k * 10'000'000LL}).ok());
            const auto buffer = bus.buffer(bus.readBuffer(image));
            ASSERT_NE(buffer, nullptr) << k;
            // Steps 0..3 commit at 10..40 ms; from 50 ms it shows step 3's for good.
            EXPECT_EQ(std::to_integer<int>(buffer->bytes.at(0)), std::min(k - 1, 3)) << k;
            EXPECT_EQ(bus.read(bus.find("sensor/cam/count")), std::min(k - 1, 3)) << k;
            EXPECT_EQ(bus.stamp(image).ns, k * 10'000'000LL);
        }
    }
    {
        Scheduler scheduler;
        scheduler.add(withFaults(std::make_unique<Counter>(), {fault("cam=dropout,at=0.05")}));
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
        ASSERT_TRUE(scheduler.stepUntil(SimTime{100'000'000}).ok());
        const SignalBus& bus = scheduler.bus();
        EXPECT_EQ(bus.stamp(bus.find("sensor/cam/image")).ns, 40'000'000);
        EXPECT_EQ(bus.read(bus.find("sensor/cam/fault/dropped")), 6.0);
    }
    {
        Scheduler scheduler;
        scheduler.add(withFaults(std::make_unique<Counter>(), {fault("cam:image=bias,value=1")}));
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("applies to numbers only"),
                  std::string::npos);
    }
}

/// Writes `sensor/sparse/value` = 10 once at the start and never again, as a module that does
/// not write every step: its pending value stays and commits again.
class Sparse final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override { return {"sensors", "sparse"}; }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec value;
        value.name = "sensor/sparse/value";
        value.unit = units::m;
        value_ = d.output(value);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext& context, SignalBus& bus) override {
        if (context.stepIndex == 0) {
            bus.write(value_, 10.0);
        }
        return {};
    }

private:
    SignalId value_;
};

TEST(SensorFaults, AreNeverAppliedTwiceToAValueWrittenOnce) {
    const World world;
    Scheduler scheduler;
    scheduler.add(withFaults(std::make_unique<Sparse>(), {fault("sparse:value=bias,value=1")}));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    for (int k = 1; k <= 5; ++k) {
        ASSERT_TRUE(scheduler.stepUntil(SimTime{k * 10'000'000LL}).ok());
        EXPECT_DOUBLE_EQ(scheduler.bus().read(scheduler.bus().find("sensor/sparse/value")), 11.0)
            << k;
    }
}

TEST(SensorFaults, ASensorNamedFaultTakesFaults) {
    // Its signals (sensor/fault/ranges) held the "/fault/" the faults' own outputs are told
    // apart by: the run was refused, "no sensor fault writes anything in this run".
    World world = room(0.0);
    world.sensors.names[0] = "fault";
    const std::vector<SensorFault> faults{fault("fault:ranges=bias,value=0.5")};
    auto scheduler = run(world, faults);
    EXPECT_EQ(checkFaultsWired(scheduler->bus(), faults), "");
    ASSERT_TRUE(scheduler->stepUntil(SimTime{200'000'000}).ok());
    const std::span<const double> r =
        scheduler->bus().readN(scheduler->bus().find("sensor/fault/ranges"));
    EXPECT_NEAR(r[0], kRoom[0] + 0.5, 1e-12);
    EXPECT_EQ(scheduler->bus().read(scheduler->bus().find("sensor/fault/fault/active")), 1.0);
}

}  // namespace
}  // namespace lagrange::sensors
