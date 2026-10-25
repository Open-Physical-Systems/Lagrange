// Vehicle dynamics end to end on MuJoCo (domains/vehicle_dynamics.md §7): ride height and
// static loads, braking distance against v²/(2μg), steady-state cornering against the linear
// bicycle model's understeer gradient, the engine map and its gears, the ideal substitute.
#include "lagrange/electrical/IdealTorqueSource.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/services/WrenchSum.hpp"
#include "lagrange/vehicle_dynamics/VehicleDynamics.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::vehicle_dynamics {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kRadius = 0.316;
constexpr double kRideHeight = kRadius + 0.25;  // chassis origin: wheel centres 0.25 m below
constexpr double kMu = 0.9;
constexpr double kCornering = 70000.0;  // N/rad per tyre
constexpr double kLongitudinal = 80000.0;

struct Options {
    bool engine{false};
};

/// A 1.5 t saloon: a chassis on a free joint, front wheels on steering knuckles, rear-wheel
/// drive, linear tyres (μ = 0.9) on a level plane.
World car(const Options& o = {}) {
    World w;
    w.robots = {"car"};
    w.addCollider(kWorldBody, ShapeType::Plane, {100.0, 100.0, 1.0}, Pose{}, 1.0);
    Pose start;
    start.position = {0.0, 0.0, kRideHeight};
    const BodyId chassis = w.addBody("chassis", kWorldBody, start, 1400.0, {0.1, 0.0, 0.0},
                                     {607, 2591, 2740, 0, 0, 0});
    w.addJoint("root", JointType::Free, chassis, {0, 0, 1}, false);
    SchemaInstance vehicle;
    vehicle.schema = "VehicleAPI_1";
    vehicle.prim = w.bodies.prim[chassis.index];
    vehicle.tokens = {{"vehicle:powertrain", o.engine ? "ice_map" : "electric"}};
    vehicle.reals = {{"vehicle:maxDriveTorque", 2000.0},  {"vehicle:maxSteerAngle", 0.6},
                     {"vehicle:maxBrakeTorque", 12000.0}, {"vehicle:brakeBias", 0.6},
                     {"vehicle:steerRate", 2.0},          {"vehicle:finalDriveRatio", 3.9}};
    if (o.engine) {
        // A 150 kW petrol engine's full-throttle curve (illustrative), five gears.
        vehicle.realArrays = {{"vehicle:engineSpeeds", {80, 200, 400, 600, 700}},
                              {"vehicle:engineTorques", {150, 260, 280, 240, 200}},
                              {"vehicle:gearRatios", {3.5, 2.1, 1.4, 1.0, 0.8}}};
    }
    w.schemaInstances.push_back(vehicle);
    for (const double x : {1.35, -1.35}) {
        for (const double y : {0.78, -0.78}) {
            const bool front = x > 0.0;
            const std::string side =
                std::string(front ? "front" : "rear") + (y > 0 ? "_left" : "_right");
            Pose at;
            at.position = {x, y, -0.25};
            BodyId hub = chassis;
            std::optional<PrimId> steer;
            if (front) {
                hub =
                    w.addBody(side + "_knuckle", chassis, at, 5.0, {}, {0.05, 0.05, 0.05, 0, 0, 0});
                w.addJoint(side + "_steer", JointType::Revolute, hub, {0, 0, 1}, true);
                steer = w.joints.prim.back();
                at.position = {0.0, 0.0, 0.0};
            }
            const BodyId wheel =
                w.addBody(side + "_wheel", hub, at, 15.0, {}, {0.4, 0.75, 0.4, 0, 0, 0});
            w.addJoint(side + "_spin", JointType::Continuous, wheel, {0, 1, 0}, true);
            SchemaInstance s;
            s.schema = "WheelAPI_1";
            s.prim = w.joints.prim.back();
            s.tokens = {{"wheel:axle", front ? "front" : "rear"}};
            s.integers = {{"wheel:steered", front ? 1 : 0}, {"wheel:driven", front ? 0 : 1}};
            s.reals = {{"wheel:radius", kRadius},
                       {"wheel:restLength", 0.3},
                       {"wheel:travel", 0.1},
                       {"wheel:springStiffness", 30000.0},
                       {"wheel:damping", 3000.0},
                       {"wheel:friction", kMu},
                       {"wheel:longitudinalStiffness", kLongitudinal},
                       {"wheel:corneringStiffness", kCornering}};
            if (steer) {
                s.relationships = {{"wheel:steerJoint", {*steer}}};
            }
            w.schemaInstances.push_back(s);
        }
    }
    return w;
}

/// Throttle, brake and steering from a function of time and the speed.
class Driver final : public IModule {
public:
    using Command = std::function<std::array<double, 3>(double t, double speed)>;
    explicit Driver(Command command) : command_(std::move(command)) {}
    ModuleInfo info() const override { return {"test", "driver", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{1'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        const auto out = [&](const char* name, Unit unit) {
            SignalSpec s;
            s.name = std::string("vehicle/car/") + name;
            s.unit = unit;
            return d.output(s);
        };
        throttle_ = out("throttle", units::one);
        brake_ = out("brake", units::one);
        steer_ = out("steering_angle", units::rad);
        SignalSpec speed;
        speed.name = "vehicle/car/speed";
        speed.unit = units::m_per_s;
        speed.feedback = true;
        speed_ = d.input(speed);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, 0.0, 0.0);
        return {};
    }
    Status step(const StepContext& c, SignalBus& bus) override {
        write(bus, c.t.seconds(), bus.read(speed_));
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus, double t, double speed) {
        const auto c = command_(t, speed);
        bus.write(throttle_, c[0]);
        bus.write(brake_, c[1]);
        bus.write(steer_, c[2]);
    }
    Command command_;
    SignalId throttle_, brake_, steer_, speed_;
};

struct Run {
    World world;
    std::unique_ptr<Scheduler> scheduler;
    std::shared_ptr<sensors::ShapeCaster> ground;
    [[nodiscard]] double read(const std::string& name) const {
        return scheduler->bus().read(scheduler->bus().find(name));
    }
    [[nodiscard]] std::span<const double> readN(const std::string& name) const {
        return scheduler->bus().readN(scheduler->bus().find(name));
    }
};

std::unique_ptr<Run> drive(World world, Driver::Command command, bool ideal = false) {
    auto r = std::make_unique<Run>();
    r->world = std::move(world);
    r->ground = std::make_shared<sensors::ShapeCaster>();
    const sensors::ShapeCaster::Shape plane{ShapeType::Plane, {100.0, 100.0, 1.0}, Pose{}};
    r->ground->update(std::span(&plane, 1));
    r->scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r->scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    r->scheduler->add(std::make_unique<Driver>(std::move(command)));
    ModuleConfigData config;
    config.values["tire_model"] = std::string("linear");
    r->scheduler->add(std::make_unique<VehicleDynamics>(ideal, r->ground, std::vector{1.0}),
                      ModuleConfig(config));
    r->scheduler->add(std::make_unique<electrical::IdealTorqueSource>());
    r->scheduler->add(std::make_unique<services::WrenchSum>());
    auto wired = r->scheduler->wire(WorldView(r->world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return wired ? std::move(r) : nullptr;
}

bool stepTo(Run& r, double t) {
    return r.scheduler->stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok();
}

TEST(VehicleDynamics, SettlesAtRideHeightOnItsStaticLoads) {
    auto r = drive(car(), [](double, double) { return std::array<double, 3>{}; });
    ASSERT_TRUE(r);
    ASSERT_TRUE(stepTo(*r, 3.0));
    EXPECT_NEAR(r->readN("body/chassis/pose")[2], kRideHeight, 0.01);
    const double weight = (1400.0 + 4 * 15.0 + 2 * 5.0) * kGravity;
    double total = 0.0;
    double front = 0.0;
    for (const char* w : {"front_left", "front_right", "rear_left", "rear_right"}) {
        const double fz = r->read(std::string("wheel/") + w + "_spin/normal_force");
        total += fz;
        front += std::string(w).starts_with("front") ? fz : 0.0;
    }
    EXPECT_NEAR(total, weight, 0.01 * weight);
    EXPECT_GT(front, 0.5 * total);  // the centre of mass is ahead of the middle
    EXPECT_NEAR(r->read("vehicle/car/speed"), 0.0, 0.01);
}

TEST(VehicleDynamics, BrakingDistanceIsVSquaredOverTwoMuG) {
    // Up to 20 m/s, then full brakes with the wheels locked: μ·m·g decelerates the car.
    constexpr double kSpeed = 20.0;
    double brakeAt = -1.0;
    auto r = drive(car(), [&](double t, double v) {
        if (brakeAt < 0.0 && v >= kSpeed) {
            brakeAt = t;
        }
        return brakeAt < 0.0 ? std::array<double, 3>{1.0, 0.0, 0.0}
                             : std::array<double, 3>{0.0, 1.0, 0.0};
    });
    ASSERT_TRUE(r);
    double start = 0.0;
    double v0 = 0.0;
    for (int i = 1; i <= 40000; ++i) {
        ASSERT_TRUE(stepTo(*r, i * 1e-3));
        if (brakeAt > 0.0 && v0 == 0.0) {
            start = r->readN("body/chassis/pose")[0];
            v0 = r->read("vehicle/car/speed");
        }
        if (brakeAt > 0.0 && r->read("vehicle/car/speed") < 0.05) {
            break;
        }
    }
    ASSERT_GT(v0, 0.0) << "never reached the speed";
    const double distance = r->readN("body/chassis/pose")[0] - start;
    const double expected = v0 * v0 / (2.0 * kMu * kGravity);
    EXPECT_NEAR(distance, expected, 0.05 * expected) << "from " << v0 << " m/s";
    RecordProperty("braking_distance",
                   std::to_string(distance) + " m, expected " + std::to_string(expected));
}

TEST(VehicleDynamics, SteadyCorneringMatchesTheBicycleModel) {
    // At 15 m/s with 0.02 rad of steering, the yaw rate of the linear bicycle model with its
    // understeer gradient K = W_f/C_f − W_r/C_r: r = V·δ / (L + K·V²/g).
    constexpr double kSpeed = 15.0;
    constexpr double kDelta = 0.02;
    double integral = 0.0;
    auto r = drive(car(), [&](double t, double v) {
        const double error = kSpeed - v;
        integral += error * 1e-3;
        const double throttle = std::clamp(0.5 * error + 0.2 * integral, -1.0, 1.0);
        return std::array<double, 3>{throttle, 0.0, t > 6.0 ? kDelta : 0.0};
    });
    ASSERT_TRUE(r);
    ASSERT_TRUE(stepTo(*r, 14.0));
    const double v = r->read("vehicle/car/speed");
    const double yaw = r->readN("body/chassis/twist")[5];
    // Axle loads from the static normal forces measured at rest with this car.
    const double mass = 1400.0 + 4 * 15.0 + 2 * 5.0;
    const double wheelbase = 2.7;
    // The centre of mass of the whole car: the chassis's at +0.1 m, the wheels' symmetric.
    const double comX = 1400.0 * 0.1 / mass;
    const double wf = mass * kGravity * (comX + 1.35) / wheelbase;
    const double wr = mass * kGravity - wf;
    const double k = wf / (2.0 * kCornering) - wr / (2.0 * kCornering);
    const double expected = v * kDelta / (wheelbase + k * v * v / kGravity);
    EXPECT_NEAR(v, kSpeed, 0.2);
    EXPECT_NEAR(yaw, expected, 0.03 * expected) << "understeer gradient " << k;
    RecordProperty("yaw_rate",
                   std::to_string(yaw) + " rad/s, expected " + std::to_string(expected));
}

TEST(VehicleDynamics, AnEngineShiftsUpThroughItsGears) {
    Options o;
    o.engine = true;
    auto r = drive(car(o), [](double, double) { return std::array<double, 3>{1.0, 0.0, 0.0}; });
    ASSERT_TRUE(r);
    double highest = 0.0;
    for (int i = 1; i <= 20; ++i) {
        ASSERT_TRUE(stepTo(*r, i * 1.0));
        highest = std::max(highest, r->read("vehicle/car/gear"));
        EXPECT_LE(r->read("vehicle/car/engine_speed"), 700.0 * 1.05);
    }
    EXPECT_GE(highest, 4.0);
    EXPECT_GT(r->read("vehicle/car/speed"), 30.0);
}

TEST(VehicleDynamics, TheIdealSubstituteFollowsTheBicycleModel) {
    auto r = drive(
        car(),
        [](double t, double) {
            return std::array<double, 3>{t < 3.0 ? 0.5 : 0.0, 0.0, t > 3.0 ? 0.1 : 0.0};
        },
        true);
    ASSERT_TRUE(r);
    ASSERT_TRUE(stepTo(*r, 6.0));
    const double v = r->read("vehicle/car/speed");
    const double mass = 1400.0 + 4 * 15.0 + 2 * 5.0;
    EXPECT_NEAR(v, 0.5 * 2000.0 / kRadius / mass * 3.0, 0.05);
    const double yaw = r->readN("body/chassis/twist")[5];
    EXPECT_NEAR(yaw, v * std::tan(0.1) / 2.7, 0.03 * v * std::tan(0.1) / 2.7);
}

TEST(VehicleDynamics, AckermannTurnsTheInnerWheelMore) {
    const double inner = ackermannAngle(0.2, 2.7, 0.78);
    const double outer = ackermannAngle(0.2, 2.7, -0.78);
    EXPECT_GT(inner, 0.2);
    EXPECT_LT(outer, 0.2);
    // Both wheels' axes meet on the rear axle's line: L/tan δ ∓ y are the turn radii.
    EXPECT_NEAR(2.7 / std::tan(inner) + 0.78, 2.7 / std::tan(outer) - 0.78, 1e-9);
}

}  // namespace
}  // namespace lagrange::vehicle_dynamics
