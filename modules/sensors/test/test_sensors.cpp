#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

constexpr double kGravity = 9.80665;  // World::gravityLocal default

Pose at(double x, double y, double z) {
    return Pose{{x, y, z}};
}

/// Rotation by `angle` about the unit axis (x, y, z).
Pose turned(double angle, double x, double y, double z) {
    const double s = std::sin(angle / 2.0);
    return Pose{{0.0, 0.0, 0.0}, {std::cos(angle / 2.0), s * x, s * y, s * z}};
}

// --- ray casts against closed forms ---------------------------------------------------------

TEST(Raycast, Plane) {
    EXPECT_DOUBLE_EQ(raycast(ShapeType::Plane, {}, Pose{}, {0, 0, 2}, {0, 0, -1}), 2.0);
    EXPECT_EQ(raycast(ShapeType::Plane, {}, Pose{}, {0, 0, 2}, {1, 0, 0}), kNoHit);
    EXPECT_EQ(raycast(ShapeType::Plane, {}, Pose{}, {0, 0, 2}, {0, 0, 1}), kNoHit);
}

TEST(Raycast, SphereFromOutsideAndInside) {
    EXPECT_DOUBLE_EQ(raycast(ShapeType::Sphere, {1, 0, 0}, at(5, 0, 0), {0, 0, 0}, {1, 0, 0}), 4.0);
    EXPECT_DOUBLE_EQ(raycast(ShapeType::Sphere, {1, 0, 0}, at(0, 0, 0), {0, 0, 0}, {0, 1, 0}), 1.0);
    EXPECT_EQ(raycast(ShapeType::Sphere, {1, 0, 0}, at(5, 3, 0), {0, 0, 0}, {1, 0, 0}), kNoHit);
}

TEST(Raycast, RotatedBox) {
    // A unit cube turned 45° about z: its corner points at the ray, sqrt(2)/2 from the centre.
    const Pose pose = compose(at(3, 0, 0), turned(std::numbers::pi / 4.0, 0, 0, 1));
    EXPECT_NEAR(raycast(ShapeType::Box, {0.5, 0.5, 0.5}, pose, {0, 0, 0}, {1, 0, 0}),
                3.0 - std::sqrt(0.5), 1e-12);
    // From inside, the ray leaves through the face 0.5 away.
    EXPECT_NEAR(raycast(ShapeType::Box, {0.5, 0.5, 0.5}, Pose{}, {0, 0, 0}, {0, 0, 1}), 0.5, 1e-12);
}

TEST(Raycast, CylinderSideAndCaps) {
    // Axis along z, radius 0.5, half length 1, at x = 2.
    EXPECT_NEAR(raycast(ShapeType::Cylinder, {0.5, 1.0, 0}, at(2, 0, 0), {0, 0, 0}, {1, 0, 0}), 1.5,
                1e-12);
    // Straight down onto the top cap.
    EXPECT_NEAR(raycast(ShapeType::Cylinder, {0.5, 1.0, 0}, at(2, 0, 0), {2.2, 0, 5}, {0, 0, -1}),
                4.0, 1e-12);
    // Above the top: misses the side.
    EXPECT_EQ(raycast(ShapeType::Cylinder, {0.5, 1.0, 0}, at(2, 0, 0), {0, 0, 1.5}, {1, 0, 0}),
              kNoHit);
}

TEST(Raycast, CapsuleEnds) {
    // Along z, radius 0.5, half length 1: the top end is a sphere centred at z = 1.
    EXPECT_NEAR(raycast(ShapeType::Capsule, {0.5, 1.0, 0}, Pose{}, {0, 0, 5}, {0, 0, -1}), 3.5,
                1e-12);
    EXPECT_NEAR(raycast(ShapeType::Capsule, {0.5, 1.0, 0}, Pose{}, {-3, 0, 0}, {1, 0, 0}), 2.5,
                1e-12);
}

// --- lidar in a room ------------------------------------------------------------------------

/// A 6 m x 4 m room of 0.2 m walls, and a lidar fixed to a post at (1, 0.5, 0.5).
World room(std::uint32_t samples, double noise) {
    World world;
    const Pose none;
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, at(0, 2.1, 1), 1.0);   // y = 2
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 0.1, 1.0}, at(0, -2.1, 1), 1.0);  // y = -2
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, at(3.1, 0, 1), 1.0);   // x = 3
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 2.0, 1.0}, at(-3.1, 0, 1), 1.0);  // x = -3
    const BodyId post = world.addBody("post", kWorldBody, at(1, 0.5, 0), 0.0, {}, {});
    world.addSensor("scanner", SensorType::Lidar, post, at(0, 0, 0.5), 10.0,
                    {{"samples", static_cast<double>(samples)},
                     {"min_angle", -std::numbers::pi},
                     {"max_angle", std::numbers::pi / 2.0},
                     {"min_range", 0.1},
                     {"max_range", 10.0},
                     {"range_noise", noise}});
    (void)none;
    return world;
}

TEST(Lidar, RangesInARoomAreTheDistancesToTheWalls) {
    const World world = room(7, 0.0);  // -180°, -135°, ..., +90° in 45° steps
    Scheduler scheduler;
    for (auto& module : createSensorModules(world)) {
        EXPECT_EQ(module->preferredStep(), 100_ms);
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    EXPECT_EQ(scheduler.plan().front().label, "sensors/lidar_scanner");
    ASSERT_TRUE(scheduler.stepTick().ok());
    const std::span<const double> ranges =
        scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
    ASSERT_EQ(ranges.size(), 7U);
    EXPECT_NEAR(ranges[0], 4.0, 1e-12);                   // -180°: wall x = -3, from x = 1
    EXPECT_NEAR(ranges[2], 2.5, 1e-12);                   // -90°: wall y = -2, from y = 0.5
    EXPECT_NEAR(ranges[4], 2.0, 1e-12);                   // 0°: wall x = 3
    EXPECT_NEAR(ranges[5], 1.5 * std::sqrt(2.0), 1e-12);  // 45°: wall y = 2 first
    EXPECT_NEAR(ranges[6], 1.5, 1e-12);                   // 90°: wall y = 2
}

TEST(Lidar, RingsOfA3dLidarHitTheFloorAndTheWalls) {
    // Sensor 0.5 m above the floor at (1, 0.5); three rings at -30, 0 and +30 degrees; four
    // columns at 0, 90, 180 and 270 degrees (the last equals -90).
    World world = room(4, 0.0);
    world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 0}, Pose{}, 1.0);
    auto& settings = world.sensors.settings[0];
    settings["min_angle"] = 0.0;
    settings["max_angle"] = 1.5 * std::numbers::pi;
    settings["vertical_samples"] = 3.0;
    settings["vertical_min_angle"] = -std::numbers::pi / 6.0;
    settings["vertical_max_angle"] = std::numbers::pi / 6.0;
    Scheduler scheduler;
    for (auto& module : createSensorModules(world)) {
        scheduler.add(std::move(module));
    }
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    const std::span<const double> r =
        scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
    ASSERT_EQ(r.size(), 12U);  // ring after ring, lowest first
    const double cos30 = std::cos(std::numbers::pi / 6.0);
    // Lowest ring: the floor, 0.5 m below, at 0.5 / sin 30 = 1 m, before any wall.
    for (int column = 0; column < 4; ++column) {
        EXPECT_NEAR(r[column], 1.0, 1e-12) << "column " << column;
    }
    // Middle ring: the walls, as the 2D scanner sees them.
    EXPECT_NEAR(r[4], 2.0, 1e-12);  // 0 deg: x = 3
    EXPECT_NEAR(r[5], 1.5, 1e-12);  // 90 deg: y = 2
    EXPECT_NEAR(r[6], 4.0, 1e-12);  // 180 deg: x = -3
    EXPECT_NEAR(r[7], 2.5, 1e-12);  // 270 deg: y = -2
    // Top ring: the same walls, further along the slanted beam.
    EXPECT_NEAR(r[8], 2.0 / cos30, 1e-12);
    EXPECT_NEAR(r[9], 1.5 / cos30, 1e-12);
}

TEST(Lidar, NoiseHasTheConfiguredSpreadAndIsDeterministic) {
    const World world = room(720, 0.02);
    const auto scan = [&](std::uint64_t seed) {
        Scheduler scheduler(SchedulerOptions{.runSeed = seed});
        for (auto& module : createSensorModules(world)) {
            scheduler.add(std::move(module));
        }
        EXPECT_TRUE(scheduler.wire(WorldView(world)));
        EXPECT_TRUE(scheduler.stepTick().ok());
        const std::span<const double> r =
            scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
        return std::vector<double>(r.begin(), r.end());
    };
    const std::vector<double> a = scan(1);
    EXPECT_EQ(scan(1), a);
    EXPECT_NE(scan(2), a);
    const std::vector<double> exact = [&] {
        const World quiet = room(720, 0.0);
        Scheduler scheduler;
        for (auto& module : createSensorModules(quiet)) {
            scheduler.add(std::move(module));
        }
        EXPECT_TRUE(scheduler.wire(WorldView(quiet)));
        EXPECT_TRUE(scheduler.stepTick().ok());
        const std::span<const double> r =
            scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
        return std::vector<double>(r.begin(), r.end());
    }();
    double squares = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        squares += (a[i] - exact[i]) * (a[i] - exact[i]);
    }
    EXPECT_NEAR(std::sqrt(squares / static_cast<double>(a.size())), 0.02, 0.003);
}

// --- IMU on a simulated body ----------------------------------------------------------------

std::array<double, 6> diagonal(double v) {
    return {v, v, v, 0.0, 0.0, 0.0};
}

TEST(Imu, AtRestReadsGravityInTheSensorFrame) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, 1.0);
    const BodyId box = world.addBody("box", kWorldBody, at(0, 0, 0.1), 1.0, {}, diagonal(0.0067));
    world.addJoint("box_free", JointType::Free, box, {0, 0, 1}, false);
    world.addCollider(box, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, 1.0);
    // Mounted rolled 90° about x: the body's up axis is the sensor's +y.
    world.addSensor("imu", SensorType::Imu, box, turned(std::numbers::pi / 2.0, 1, 0, 0), 200.0,
                    {});
    Scheduler scheduler;
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    for (auto& module : createSensorModules(world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const std::span<const double> force = bus.readN(bus.find("sensor/imu/specific_force"));
    EXPECT_NEAR(force[0], 0.0, 0.02);
    EXPECT_NEAR(force[1], kGravity, 0.01 * kGravity);
    EXPECT_NEAR(force[2], 0.0, 0.02);
    const std::span<const double> rate = bus.readN(bus.find("sensor/imu/angular_rate"));
    EXPECT_NEAR(std::hypot(rate[0], rate[1], rate[2]), 0.0, 1e-3);
    EXPECT_EQ(scheduler.writerOf(bus.find("sensor/imu/specific_force")).dt, 5_ms);
}

/// Writes a constant torque to one joint.
class Torque final : public IModule {
public:
    explicit Torque(double value) : value_(value) {}
    [[nodiscard]] ModuleInfo info() const override { return {"controller", "torque"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "joint/axle/torque";
        spec.unit = units::N_m;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, value_);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(id_, value_);
        return {};
    }

private:
    double value_;
    SignalId id_;
};

TEST(Imu, ReadsTheSpinOfAFlywheelInTheSensorFrame) {
    World world;
    world.gravityLocal = {0.0, 0.0, 0.0};
    const BodyId wheel =
        world.addBody("wheel", kWorldBody, Pose{}, 2.0, {}, {0.01, 0.01, 0.02, 0, 0, 0});
    world.addJoint("axle", JointType::Continuous, wheel, {0, 0, 1}, true);
    // Mounted upside down (180° about x): the spin about +z reads as -z.
    world.addSensor("imu", SensorType::Imu, wheel, turned(std::numbers::pi, 1, 0, 0), 100.0, {});
    Scheduler scheduler;
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(std::make_unique<Torque>(0.04));  // 2 rad/s² on 0.02 kg·m²
    for (auto& module : createSensorModules(world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    const double spin = bus.read(bus.find("joint/axle/velocity"));
    EXPECT_NEAR(spin, 2.0, 1e-6);
    // The IMU sampled at its last 10 ms step: within one step of the joint's speed.
    EXPECT_NEAR(bus.readN(bus.find("sensor/imu/angular_rate"))[2], -spin, 2.0 * 0.01 + 1e-6);
}

TEST(Imu, OnAStaticBodyIsAConfigurationError) {
    World world;
    const BodyId post = world.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    world.addSensor("imu", SensorType::Imu, post, Pose{}, 100.0, {});
    Scheduler scheduler;
    for (auto& module : createSensorModules(world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(wired.error().front().detail.find("never moves"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::sensors
