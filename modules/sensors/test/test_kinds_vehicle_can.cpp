// Kind "vehicle_can" (sensors.md §9): a car's own CAN sensors from its joints and chassis,
// against closed forms: wheel speeds ω·r and their tone-ring quantisation, the steering wheel
// angle from Ackermann road-wheel angles, yaw rate and v·ω on a circle, the gear, and a refusal
// on a robot that is not a car.
#include "kinds_test_util.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors {
namespace {

using namespace test;

constexpr double kRadius = 0.3;  // m
constexpr double kL = 2.8;       // m, wheelbase
constexpr double kHalfTrack = 0.8;
const std::array<std::string, 4> kWheels{"fl_spin", "fr_spin", "rl_spin", "rr_spin"};

/// A car: chassis "chassis" (VehicleAPI_1), front wheels on steering knuckles, rear wheels on
/// the chassis, each spin joint with WheelAPI_1; the sensor on the chassis.
void car(World& w, std::map<std::string, double, std::less<>> settings = {}, bool vehicle = true) {
    w.robots = {"car"};
    const BodyId chassis = w.addBody("chassis", kWorldBody, Pose{}, 1500.0, {}, {1, 1, 1, 0, 0, 0});
    w.addJoint("chassis_free", JointType::Free, chassis, {0, 0, 1}, false);
    if (vehicle) {
        SchemaInstance v;
        v.schema = "VehicleAPI_1";
        v.prim = w.bodies.prim.back();
        w.schemaInstances.push_back(v);
    }
    const auto wheel = [&](const std::string& side, double x, double y, bool front) {
        Pose at;
        at.position = {x, y, 0.0};
        BodyId parent = chassis;
        std::optional<PrimId> steer;
        if (front) {
            parent = w.addBody(side + "_knuckle", chassis, at, 5.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
            w.addJoint(side + "_steer", JointType::Revolute, parent, {0, 0, 1}, true);
            steer = w.joints.prim.back();
            at = Pose{};
        }
        const BodyId body =
            w.addBody(side + "_wheel", parent, at, 10.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
        w.addJoint(side + "_spin", JointType::Continuous, body, {0, 1, 0}, front ? false : true);
        SchemaInstance s;
        s.schema = "WheelAPI_1";
        s.prim = w.joints.prim.back();
        s.reals = {{"wheel:radius", kRadius}};
        s.tokens = {{"wheel:axle", front ? "front" : "rear"}};
        s.integers = {{"wheel:steered", front ? 1 : 0}};
        if (steer) {
            s.relationships = {{"wheel:steerJoint", {*steer}}};
        }
        w.schemaInstances.push_back(s);
    };
    wheel("fl", kL / 2.0, kHalfTrack, true);
    wheel("fr", kL / 2.0, -kHalfTrack, true);
    wheel("rl", -kL / 2.0, kHalfTrack, false);
    wheel("rr", -kL / 2.0, -kHalfTrack, false);
    w.addCustomSensor("can", "vehicle_can", chassis, Pose{}, 100.0, std::move(settings), {});
}

double ackermann(double delta, double y) {
    const double t = std::tan(delta);
    return std::atan(kL * t / (kL - y * t));
}

/// Feeds: on a circle of radius v/ω (straight for ω = 0) at speed v, wheels rolling at v/r,
/// steering at bicycle angle `delta`.
std::vector<Feed> drive(double v, double omega, double delta) {
    std::vector<Feed> f;
    f.push_back({makeSpec("body/chassis/pose", SignalType::Pose, units::m, "world_local"),
                 [v, omega](double t) {
                     const double yaw = omega * t;
                     const double x = omega == 0.0 ? v * t : v / omega * std::sin(yaw);
                     const double y = omega == 0.0 ? 0.0 : v / omega * (1.0 - std::cos(yaw));
                     return std::vector<double>{
                         x, y, 0.0, std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
                 }});
    f.push_back({makeSpec("body/chassis/twist", SignalType::Wrench, units::m_per_s, "world_local"),
                 [v, omega](double t) {
                     return std::vector<double>{
                         v * std::cos(omega * t), v * std::sin(omega * t), 0.0, 0.0, 0.0, omega};
                 }});
    for (const std::string& j : kWheels) {
        f.push_back({makeSpec("joint/" + j + "/position", SignalType::Scalar, units::rad),
                     [v](double t) { return std::vector<double>{v / kRadius * t}; }});
        f.push_back(constant("joint/" + j + "/velocity", SignalType::Scalar, units::rad_per_s,
                             {v / kRadius}));
    }
    f.push_back(constant("joint/fl_steer/position", SignalType::Scalar, units::rad,
                         {ackermann(delta, kHalfTrack)}));
    f.push_back(constant("joint/fr_steer/position", SignalType::Scalar, units::rad,
                         {ackermann(delta, -kHalfTrack)}));
    return f;
}

TEST(VehicleCan, IsBuiltin) {
    EXPECT_TRUE(isBuiltinKind("vehicle_can"));
}

TEST(VehicleCan, StraightAtTenMetresASecond) {
    KindRun r;
    car(r.world);
    ASSERT_TRUE(start(r, drive(10.0, 0.0, 0.0), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    for (const double s : r.readN("sensor/can/wheel_speeds")) {
        EXPECT_NEAR(s, 10.0, 1e-9);
    }
    EXPECT_NEAR(r.read("sensor/can/speed"), 10.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/can/steering_wheel_angle"), 0.0, 1e-12);
    EXPECT_NEAR(r.read("sensor/can/yaw_rate"), 0.0, 1e-12);
    EXPECT_NEAR(r.read("sensor/can/longitudinal_acceleration"), 0.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/can/lateral_acceleration"), 0.0, 1e-9);
    EXPECT_EQ(r.read("sensor/can/valid"), 1.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/can/gear")));  // no vehicle domain
}

TEST(VehicleCan, OnACircle) {
    // v = 10 m/s, ω = 0.5 rad/s: lateral acceleration v·ω = 5 m/s² (the 10 ms difference of the
    // velocity is off by v·ω·(1 − sinc) and leans back by ω·dt/2: 12 mm/s² ahead).
    KindRun r;
    car(r.world);
    ASSERT_TRUE(start(r, drive(10.0, 0.5, 0.0), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/can/yaw_rate"), 0.5, 1e-12);
    EXPECT_NEAR(r.read("sensor/can/lateral_acceleration"), 5.0, 1e-3);
    EXPECT_NEAR(r.read("sensor/can/longitudinal_acceleration"), 0.0, 0.02);
}

TEST(VehicleCan, SteeringWheelAngle) {
    // Ackermann road-wheel angles for a bicycle angle of 0.1 rad: 15.5 × 0.1 at the wheel.
    KindRun r;
    car(r.world);
    ASSERT_TRUE(start(r, drive(0.0, 0.0, 0.1), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/can/steering_wheel_angle"), 1.55, 1e-12);
    // Quantised to 0.1° without ground truth, with a ratio of its own.
    KindRun q;
    car(q.world, {{"steering_ratio", 14.0}});
    ASSERT_TRUE(start(q, drive(0.0, 0.0, 0.1), false));
    ASSERT_TRUE(q.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const double step = 0.1 * std::numbers::pi / 180.0;
    EXPECT_NEAR(q.read("sensor/can/steering_wheel_angle"), std::round(1.4 / step) * step, 1e-12);
}

TEST(VehicleCan, ToneRingTiming) {
    // 48 teeth at 33.3 rad/s: an edge every 3.9 ms, timed between edges, reads 10 m/s exactly
    // at a steady speed.
    KindRun r;
    car(r.world);
    ASSERT_TRUE(start(r, drive(10.0, 0.0, 0.0), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    for (const double s : r.readN("sensor/can/wheel_speeds")) {
        EXPECT_NEAR(s, 10.0, 1e-9);
    }
}

TEST(VehicleCan, ToneRingWhenTheWheelStops) {
    // Rolling at 1 m/s for 1 s, then still: the estimate falls as one tooth's 3.9 cm over the
    // time since the last edge, so 1 s later it reads under 4 cm/s.
    const double pitch = 2.0 * std::numbers::pi / 48.0 * kRadius;  // m per tooth
    KindRun r;
    car(r.world);
    std::vector<Feed> f = drive(0.0, 0.0, 0.0);
    for (Feed& feed : f) {
        if (feed.spec.name == "joint/rr_spin/position") {
            feed.value = [](double t) {
                return std::vector<double>{std::min(t, 1.0) * 1.0 / kRadius};
            };
        }
    }
    ASSERT_TRUE(start(r, std::move(f), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{900'000'000}).ok());
    EXPECT_NEAR(r.readN("sensor/can/wheel_speeds")[3], 1.0, 1e-9);
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{2'000'000'000}).ok());
    const double s = r.readN("sensor/can/wheel_speeds")[3];
    EXPECT_GT(s, 0.0);
    EXPECT_LE(s, pitch / 1.0 + 1e-9);
}

TEST(VehicleCan, Gear) {
    KindRun r;
    car(r.world);
    std::vector<Feed> f = drive(5.0, 0.0, 0.0);
    f.push_back(constant("vehicle/car/gear", SignalType::Int, units::one, {3}));
    ASSERT_TRUE(start(r, std::move(f), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_EQ(r.read("sensor/can/gear"), 3.0);
}

TEST(VehicleCan, OnlyOnACar) {
    World w;
    car(w, {}, false);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto module = createBuiltinKind(w, 0, false, nullptr, nullptr);
    ASSERT_TRUE(module);
    scheduler.add(std::move(module));
    EXPECT_FALSE(scheduler.wire(WorldView(w)));
}

TEST(VehicleCan, RestoredRunRepeats) {
    // Its last velocity and tooth clocks are state: no acceleration spike after a restore.
    KindRun r;
    car(r.world);
    ASSERT_TRUE(start(r, drive(10.0, 0.5, 0.0), false));
    expectRestoredRunRepeats(r,
                             {"sensor/can/wheel_speeds", "sensor/can/longitudinal_acceleration",
                              "sensor/can/lateral_acceleration"},
                             500'000'000, 1'500'000'000, 10'000'000);
}

}  // namespace
}  // namespace lagrange::sensors
