// The v0.4 low-battery scenario (docs/design/robot-classes.md §5): a nearly empty pack sags
// under load, and the motors, limited by the bus voltage, give less thrust. The sample
// quadcopter climbs at full throttle (the built-in controller asked for 1000 m) on a full and on
// a 10 % battery: MuJoCo, the electrical network at 50 µs, the rotors. (At 3 % the pack sags
// below the bus's undervoltage trip at full load: the drives trip and it does not take off.)
#include "lagrange/electrical/Network.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/server/Multicopter.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/services/WrenchSum.hpp"

#include <numbers>
#include <optional>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

using namespace literals;

struct Climb {
    double height{0.0};    // m above the start after 2.5 s
    double busVolts{0.0};  // V at full throttle (1 s)
    double thrust{0.0};    // N, the four rotors at full throttle (1 s)
};

Climb climb(double stateOfCharge) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    EXPECT_TRUE(robot) << robot.error();
    World& world = robot->world;
    for (SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "BatteryAPI_1") {
            s.reals["elec:battery:initialSoc"] = stateOfCharge;
        }
    }
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    MulticopterController::Settings flight;
    flight.altitudeM = 1000.0;  // far above: the controller asks for all the thrust there is
    flight.takeoffAfterS = 0.0;
    scheduler.add(std::make_unique<MulticopterController>(flight), ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const double start = bus.readN(bus.find("body/base_link/pose"))[2];
    EXPECT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    Climb out;
    out.busVolts = bus.read(bus.find("bus/dc_bus/voltage"));
    for (int i = 1; i <= 4; ++i) {
        out.thrust += bus.read(bus.find("rotor/rotor_" + std::to_string(i) + "_joint/thrust"));
    }
    EXPECT_TRUE(scheduler.stepUntil(SimTime{2'500'000'000}).ok());
    out.height = bus.readN(bus.find("body/base_link/pose"))[2] - start;
    return out;
}

TEST(Multicopter, ALowBatterySagsAndGivesLessThrust) {
    const Climb full = climb(1.0);
    const Climb low = climb(0.10);
    std::printf("full pack: %.1f m in 2.5 s, %.2f V, %.1f N; 10 %% pack: %.1f m, %.2f V, %.1f N\n",
                full.height, full.busVolts, full.thrust, low.height, low.busVolts, low.thrust);
    EXPECT_LT(low.busVolts, full.busVolts - 1.5);  // 4 cells: 4.2 V full, 3.6 V at 10 %
    EXPECT_LT(low.thrust, 0.9 * full.thrust);      // the voltage limits the rotor speed
    EXPECT_LT(low.height, 0.8 * full.height);
    EXPECT_GT(low.height, 0.0);  // it still flies
}

struct Hover {
    double height{0.0};      // m above the start after 8 s
    double rotorSpeed{0.0};  // rad/s, rotor 1 about its joint axis
    double power{0.0};       // W from the battery
};

/// The built-in controller hovering `path` 2 m above its start, on the rotors' `fidelity`.
Hover hover(const std::string& path, const std::string& fidelity) {
    auto robot = loadModel(path);
    EXPECT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    ModuleConfigData rotors = air;
    rotors.values["fidelity"] = fidelity;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(rotors));
    scheduler.add(std::make_unique<MulticopterController>(MulticopterController::Settings{}),
                  ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(robot->world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const double start = bus.readN(bus.find("body/base_link/pose"))[2];
    EXPECT_TRUE(scheduler.stepUntil(SimTime{8'000'000'000}).ok());
    return {bus.readN(bus.find("body/base_link/pose"))[2] - start,
            bus.read(bus.find("joint/rotor_1_joint/velocity")),
            bus.read(bus.find("battery/battery/power"))};
}

// Propellers as built (step 9): the sample hovers alike on its coefficients and on its blades,
// which were matched to the same measured propeller; and with its motors upside down under the
// arms, pushers whose joints turn the other way about their downward axes, it hovers too.
TEST(Multicopter, HoversOnItsBladesAndWithInvertedMotors) {
    const Hover coefficients = hover(LAGRANGE_TEST_QUAD_USDA, "coefficients");
    const Hover blades = hover(LAGRANGE_TEST_QUAD_USDA, "bem");
    const Hover inverted = hover(LAGRANGE_TEST_QUAD_INVERTED_USDA, "bem");
    for (const Hover& h : {coefficients, blades, inverted}) {
        EXPECT_NEAR(h.height, 2.0, 0.05);
    }
    EXPECT_NEAR(blades.rotorSpeed, coefficients.rotorSpeed, 0.02 * coefficients.rotorSpeed);
    EXPECT_NEAR(blades.power, coefficients.power, 0.03 * coefficients.power);
    EXPECT_GT(blades.rotorSpeed, 0.0);    // rotor 1 is ccw about its upward axis
    EXPECT_LT(inverted.rotorSpeed, 0.0);  // the same sense seen from above, about a downward axis
    EXPECT_NEAR(inverted.rotorSpeed, -blades.rotorSpeed, 0.01 * blades.rotorSpeed);
}

// Steering (/cmd_vel): velocity commands move the set point in the heading frame and turn the
// heading; without commands the quadcopter holds where it is.
TEST(Multicopter, FliesWhereItsVelocityCommandsSay) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = false});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    std::optional<MulticopterController::FlightCommand> command;
    scheduler.add(std::make_unique<MulticopterController>(MulticopterController::Settings{},
                                                          [&command] { return command; }),
                  ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    double t = 0.0;
    const auto fly = [&](double seconds) {
        t += seconds;
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok());
        const std::span<const double> p = bus.readN(pose);
        return std::array<double, 3>{p[0], p[1], p[2]};
    };
    const std::array<double, 3> hover = fly(5.0);
    // Forward at 1 m/s for 3 s, then 3 s without commands: 3 m along x, and it stays there.
    command = MulticopterController::FlightCommand{1.0, 0.0, 0.0, 0.0};
    (void)fly(3.0);
    command.reset();
    const std::array<double, 3> stopped = fly(3.0);
    EXPECT_NEAR(stopped[0] - hover[0], 3.0, 0.2);
    EXPECT_NEAR(stopped[1] - hover[1], 0.0, 0.1);
    EXPECT_NEAR(fly(2.0)[0], stopped[0], 0.05);  // holds
    // Turn 90° left (π/4 rad/s for 2 s), then forward 2 m: along y now.
    command = MulticopterController::FlightCommand{0.0, 0.0, 0.0, std::numbers::pi / 4.0};
    (void)fly(2.0);
    command = MulticopterController::FlightCommand{1.0, 0.0, 0.0, 0.0};
    (void)fly(2.0);
    command.reset();
    const std::array<double, 3> turned = fly(3.0);
    EXPECT_NEAR(turned[0] - stopped[0], 0.0, 0.2);
    EXPECT_NEAR(turned[1] - stopped[1], 2.0, 0.2);
    // Up 1 m/s for 1 s.
    command = MulticopterController::FlightCommand{0.0, 0.0, 1.0, 0.0};
    (void)fly(1.0);
    command.reset();
    EXPECT_NEAR(fly(3.0)[2] - turned[2], 1.0, 0.1);
}

// --no-demo (Settings::waitForCommand): it stays on the ground with its rotors still until a
// command asks to climb, flies by commands only, and lands and stops its rotors when brought
// down.
TEST(Multicopter, WaitsOnTheGroundForACommandAndLands) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = false});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    std::optional<MulticopterController::FlightCommand> command;
    MulticopterController::Settings flight;
    flight.waitForCommand = true;
    scheduler.add(std::make_unique<MulticopterController>(flight, [&command] { return command; }),
                  ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    const SignalId thrust = bus.find("rotor/rotor_1_joint/thrust");
    double t = 0.0;
    const auto fly = [&](double seconds) {
        t += seconds;
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok());
        return bus.readN(pose)[2];
    };
    const double ground = fly(0.001);
    // No command, and a command that does not climb: it stays down, its rotors still.
    EXPECT_NEAR(fly(3.0), ground, 0.01);
    command = MulticopterController::FlightCommand{1.0, 0.0, 0.0, 0.0};
    EXPECT_NEAR(fly(1.0), ground, 0.01);
    EXPECT_NEAR(bus.read(thrust), 0.0, 1e-6);
    // Up at 1 m/s for 2 s, then hold: it climbs from where it was, to about 2 m.
    command = MulticopterController::FlightCommand{0.0, 0.0, 1.0, 0.0};
    (void)fly(2.0);
    command.reset();
    const double up = fly(3.0);
    EXPECT_NEAR(up - ground, 2.0, 0.25);
    EXPECT_NEAR(fly(2.0), up, 0.05);  // holds without commands
    // Down at 1 m/s until it is on the ground: it lands and its rotors stop.
    command = MulticopterController::FlightCommand{0.0, 0.0, -1.0, 0.0};
    EXPECT_NEAR(fly(6.0), ground, 0.03);
    // Its rotors coast down once the motors stop: 2 s on, under 1 % of a hover's thrust each
    // (about 5 N), and it stays down with no command.
    command.reset();
    EXPECT_NEAR(fly(2.0), ground, 0.03);
    EXPECT_LT(bus.read(thrust), 0.05);
}

// From the ground with every stick at once (forward, sideways, up and turning, as a radio's
// sticks often are at take-off): it lifts off straight, then flies, instead of the set point
// running away sideways while its legs still hold it.
TEST(Multicopter, TakesOffByHandWithEveryStickAtOnce) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = false});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    std::optional<MulticopterController::FlightCommand> command =
        MulticopterController::FlightCommand{1.8, -0.75, 0.48, 0.27};
    MulticopterController::Settings flight;
    flight.waitForCommand = true;
    scheduler.add(std::make_unique<MulticopterController>(flight, [&command] { return command; }),
                  ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    const std::array<double, 3> start{bus.readN(pose)[0], bus.readN(pose)[1], bus.readN(pose)[2]};
    ASSERT_TRUE(scheduler.stepUntil(SimTime{4'000'000'000}).ok());
    const std::span<const double> p = bus.readN(pose);
    // 0.48 m/s up for 4 s, about 1.9 m; and well away from where it started, along its arc.
    EXPECT_NEAR(p[2] - start[2], 1.9, 0.3);
    EXPECT_GT(std::hypot(p[0] - start[0], p[1] - start[1]), 4.0);
    // Upright: the body's z axis within 30° of vertical (q = w, x, y, z).
    const double up = 1.0 - 2.0 * (p[4] * p[4] + p[5] * p[5]);
    EXPECT_GT(up, std::cos(std::numbers::pi / 6.0));
}

}  // namespace
}  // namespace lagrange::server
