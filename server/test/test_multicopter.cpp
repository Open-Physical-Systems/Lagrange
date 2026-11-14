// The v0.4 low-battery scenario (docs/design/robot-classes.md §5): a nearly empty pack sags
// under load, and the motors, limited by the bus voltage, give less thrust. The sample
// quadcopter climbs at full throttle (the built-in controller asked for 1000 m) on a full and on
// a 10 % battery: MuJoCo, the electrical network at 50 µs, the rotors. (At 3 % the pack sags
// below the bus's undervoltage trip at full load: the drives trip and it does not take off.)
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/electrical/Network.hpp"
#include "lagrange/mavlink/RotorTopSpeed.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/server/Multicopter.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/services/WrenchSum.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
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

// Angle mode (FlightCommand::Mode::Angle), in a 6 m/s wind blowing towards +x: in position mode
// it holds its place, leaning into the wind; in angle mode with the sticks centred it flies
// level and the wind carries it downwind, its height still held; half stick forward tilts it
// atan(tan 30° / 2) ≈ 16°; back in position mode it holds where it then is.
TEST(Multicopter, AngleModeLetsTheWindCarryIt) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    Scheduler scheduler(SchedulerOptions{.deterministic = false});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData aero;
    aero.values["air_density"] = 1.225;
    aero.values["wind_speed"] = 6.0;
    aero.values["wind_from"] = 270.0;  // from the west: it blows towards +x
    aero.values["wind_reference_height"] = 2.0;
    aero.values["roughness_length"] = 0.03;
    aero.values["downwash"] = false;
    scheduler.add(std::make_unique<aerodynamics::AeroDrag>(), ModuleConfig(aero));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    using Mode = MulticopterController::FlightCommand::Mode;
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
        return std::array<double, 7>{p[0], p[1], p[2], p[3], p[4], p[5], p[6]};
    };
    // The tilt of the body's z axis from the vertical towards +x, degrees.
    const auto leanX = [](const std::array<double, 7>& p) {
        const double w = p[3], x = p[4], y = p[5], z = p[6];
        return std::atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y)) * 180.0 /
               std::numbers::pi;
    };
    (void)fly(6.0);
    const std::array<double, 7> held = fly(3.0);
    EXPECT_NEAR(fly(2.0)[0], held[0], 0.05);  // position mode holds against the wind
    EXPECT_LT(leanX(held), -1.0);             // leaning into it (towards −x)

    command = MulticopterController::FlightCommand{0.0, 0.0, 0.0, 0.0, Mode::Angle};
    const std::array<double, 7> start = fly(0.0);
    const std::array<double, 7> drifted = fly(4.0);
    EXPECT_GT(drifted[0] - start[0], 2.0);    // carried downwind
    EXPECT_NEAR(drifted[2], start[2], 0.15);  // the height still held
    EXPECT_NEAR(leanX(drifted), 0.0, 1.5);    // level

    command = MulticopterController::FlightCommand{2.5, 0.0, 0.0, 0.0, Mode::Angle};
    const double tilted = leanX(fly(1.0));
    EXPECT_NEAR(tilted,
                std::atan(std::tan(std::numbers::pi / 6.0) / 2.0) * 180.0 / std::numbers::pi, 2.0);

    command.reset();  // position mode again: it stops and holds
    const std::array<double, 7> stopping = fly(4.0);
    EXPECT_NEAR(fly(3.0)[0], stopping[0], 0.1);
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

// Flown by hand into a ceiling (a fixed box 2 m above the start) with the stick held up, then
// forward, then let go: the set point stays on its leash, so the drone presses on the ceiling
// without tipping over, and comes down as soon as it is asked to (the set point did not run
// metres above the ceiling while it was held there).
TEST(Multicopter, PressedUnderACeilingItStaysUprightAndComesBack) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    World& world = robot->world;
    const auto base = std::ranges::find(world.bodies.names, "base_link");
    ASSERT_NE(base, world.bodies.names.end());
    const double start =
        world.bodies.poseInParent[static_cast<std::size_t>(base - world.bodies.names.begin())]
            .position[2];
    // The ceiling: a 6 m x 6 m slab 0.3 m thick (half extents), its underside 2 m above the start.
    world.addCollider(kWorldBody, ShapeType::Box, {3.0, 3.0, 0.15}, Pose{{0.0, 0.0, start + 2.15}},
                      1.0);
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = 1.225;
    std::optional<MulticopterController::FlightCommand> command;
    MulticopterController::Settings flight;
    flight.waitForCommand = true;
    Scheduler run(SchedulerOptions{.deterministic = false});
    run.add(std::make_unique<rigid_body::MujocoBackend>());
    run.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    run.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    run.add(std::make_unique<MulticopterController>(flight, [&command] { return command; }),
            ModuleConfig(air));
    run.add(std::make_unique<services::WrenchSum>());
    auto wired = run.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = run.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    double t = 0.0;
    double worstUp = 1.0;  // the lowest z of the body's z axis seen (cos of the tilt)
    const auto fly = [&](double seconds) {
        for (const double end = t + seconds; t < end - 1e-9;) {
            t += 0.05;
            EXPECT_TRUE(run.stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok());
            const std::span<const double> p = bus.readN(pose);
            worstUp = std::min(worstUp, 1.0 - 2.0 * (p[4] * p[4] + p[5] * p[5]));
        }
        return bus.readN(pose)[2];
    };
    // Up at full stick (2 m/s) for 8 s: about 1 s to the ceiling, then 7 s pressed under it.
    command = MulticopterController::FlightCommand{0.0, 0.0, 2.0, 0.0};
    const double pressed = fly(8.0);
    EXPECT_GT(pressed - start, 1.5);
    // Forward as well for 3 s, still pressed up, then the sticks let go for 8 s.
    command = MulticopterController::FlightCommand{2.0, 0.0, 2.0, 0.0};
    (void)fly(3.0);
    command.reset();
    const double released = fly(8.0);
    EXPECT_GT(released - start, 1.5);  // still up there, not fallen
    EXPECT_GT(worstUp, std::cos(std::numbers::pi / 4.0)) << "tipped past 45°";
    // Down at 1 m/s for 1.5 s: it comes down right away, ramping to that speed (about 1 m).
    command = MulticopterController::FlightCommand{0.0, 0.0, -1.0, 0.0};
    const double down = fly(1.5);
    EXPECT_GT(released - down, 0.5) << "the set point was left far above the ceiling";
}

// Full stick forward (5 m/s) for 4 s, then let go: the commanded velocity ramps within PX4's
// position-mode limits (3 m/s², 8 m/s³), so it leans well short of the 30° cap, keeps its
// height, brakes to a stop and stays where it stopped, instead of throwing itself at the cap and
// flying back to where the stick was let go.
TEST(Multicopter, FullStickThenLetGoStartsAndStopsSmoothly) {
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
    double t = 0.0;
    double worstUp = 1.0;  // cos of the largest tilt seen
    double lowest = 1e9;
    double highest = -1e9;
    double furthest = -1e9;
    const auto fly = [&](double seconds) {
        for (const double end = t + seconds; t < end - 1e-9;) {
            t += 0.01;
            EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok());
            const std::span<const double> p = bus.readN(pose);
            worstUp = std::min(worstUp, 1.0 - 2.0 * (p[4] * p[4] + p[5] * p[5]));
            lowest = std::min(lowest, p[2]);
            highest = std::max(highest, p[2]);
            furthest = std::max(furthest, p[0]);
        }
        return bus.readN(pose)[0];
    };
    // Up to 2 m and hold, then measure from there.
    command = MulticopterController::FlightCommand{0.0, 0.0, 1.0, 0.0};
    (void)fly(2.0);
    command.reset();
    const double start = fly(3.0);
    worstUp = 1.0;
    lowest = highest = bus.readN(pose)[2];
    command = MulticopterController::FlightCommand{5.0, 0.0, 0.0, 0.0};
    const double letGo = fly(4.0);
    command.reset();
    const double stopped = fly(5.0);
    const double tilt = std::acos(worstUp) * 180.0 / std::numbers::pi;
    std::printf(
        "let go at %.2f m, stopped at %.2f m (furthest %.2f), worst tilt %.1f deg, "
        "height within %.2f m\n",
        letGo - start, stopped - start, furthest - start, tilt, highest - lowest);
    EXPECT_LT(tilt, 25.0);              // 3 m/s² is about 17°, plus the turn-in
    EXPECT_LT(highest - lowest, 0.15);  // keeps its height
    EXPECT_GT(stopped - letGo, 2.5);    // brakes: 5 m/s at 3 m/s² is about 4 m
    EXPECT_LT(stopped - letGo, 6.0);
    EXPECT_GT(stopped, furthest - 0.15);  // stays where it stopped, does not fly back
}

// Off a platform (a 3 m block it starts on): told the ground under it, it flies off the edge,
// comes down past the platform's top and lands on the ground below, instead of taking the
// height it started at for the ground (where it would stop descending and refuse to move).
TEST(Multicopter, FliesOffAPlatformAndLandsBelowIt) {
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    World& world = robot->world;
    const auto base = std::ranges::find(world.bodies.names, "base_link");
    ASSERT_NE(base, world.bodies.names.end());
    Pose& start =
        world.bodies.poseInParent[static_cast<std::size_t>(base - world.bodies.names.begin())];
    const double onGround = start.position[2];
    start.position[2] += 3.0;  // on the block
    world.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 1.5}, Pose{{0.0, 0.0, 1.5}}, 1.0);
    // The ground for the controller: the block and the ground plane.
    std::vector<sensors::ShapeCaster::Shape> shapes;
    for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
        if (world.colliders.body[c] == kWorldBody) {
            sensors::ShapeCaster::Shape shape;
            shape.type = world.colliders.shape[c];
            shape.size = world.colliders.size[c];
            shape.pose = world.colliders.poseInBody[c];
            shapes.push_back(shape);
        }
    }
    auto ground = std::make_shared<sensors::ShapeCaster>();
    ground->update(shapes);
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
    scheduler.add(std::make_unique<MulticopterController>(
                      flight, [&command] { return command; }, ground),
                  ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    const SignalId pose = bus.find("body/base_link/pose");
    const SignalId thrust = bus.find("rotor/rotor_1_joint/thrust");
    double t = 0.0;
    const auto fly = [&](double seconds) {
        t += seconds;
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(t * 1e9)}).ok());
        return std::vector<double>(bus.readN(pose).begin(), bus.readN(pose).end());
    };
    const double top = fly(0.5)[2];
    command = MulticopterController::FlightCommand{0.0, 0.0, 1.0, 0.0};
    (void)fly(1.5);  // up off the block
    command = MulticopterController::FlightCommand{0.0, 2.0, 0.0, 0.0};
    const std::vector<double> out = fly(3.0);  // left, past its edge
    EXPECT_GT(out[1], 2.5);
    command = MulticopterController::FlightCommand{0.0, 0.0, -1.0, 0.0};
    const std::vector<double> low = fly(6.0);  // down, past the block's top, to the ground
    std::printf("off the block: started %.2f m up, out to y %.2f, came down to %.2f m\n", top,
                out[1], low[2]);
    EXPECT_LT(low[2], top - 2.5);
    EXPECT_NEAR(low[2], onGround, 0.05);  // on the ground
    command.reset();
    (void)fly(4.0);                    // its rotors coast down once the motors stop
    EXPECT_LT(bus.read(thrust), 0.1);  // landed: rotors coasting, under 2 % of a hover's
}

// --thrust-limit 0.8: asked for all the thrust there is (a set point 1000 m up), each rotor
// climbs to sqrt(0.8) of its loaded top speed (an autopilot's full output, mavlink::rotorTopSpeed)
// and no further, so the four give 80 % of the thrust they would at their top speed.
TEST(Multicopter, AThrustLimitCapsEachRotor) {
    constexpr double kLimit = 0.8;
    constexpr double kRho = 1.225;  // kg/m^3, ISA sea level
    auto robot = loadModel(LAGRANGE_TEST_QUAD_USDA);
    ASSERT_TRUE(robot) << robot.error();
    double top = 0.0;  // rad/s, the first rotor's loaded top speed
    for (const SchemaInstance& s : robot->world.schemaInstances) {
        if (s.schema == "RotorAPI_1") {
            top = mavlink::rotorTopSpeed(WorldView(robot->world), s, kRho).speed;
            break;
        }
    }
    ASSERT_GT(top, 0.0);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData power;
    power.dt = 50_us;
    power.values["exchange_step"] = 0.001;
    scheduler.add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
    ModuleConfigData air;
    air.values[rotor_aero::kAirDensityKey] = kRho;
    scheduler.add(std::make_unique<rotor_aero::RotorAero>(), ModuleConfig(air));
    MulticopterController::Settings flight;
    flight.altitudeM = 1000.0;
    flight.takeoffAfterS = 0.0;
    flight.thrustLimit = kLimit;
    scheduler.add(std::make_unique<MulticopterController>(flight), ModuleConfig(air));
    scheduler.add(std::make_unique<services::WrenchSum>());
    auto wired = scheduler.wire(WorldView(robot->world));
    ASSERT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    const SignalBus& bus = scheduler.bus();
    double fastest = 0.0;
    double thrust = 0.0;      // N, the four rotors over the last half second
    double atTopSpeed = 0.0;  // N, what they would give at their top speed (thrust ~ omega^2)
    for (int k = 1; k <= 1500; ++k) {
        ASSERT_TRUE(scheduler.stepUntil(SimTime{k * 1'000'000LL}).ok());
        for (int i = 1; i <= 4; ++i) {
            const std::string joint = "rotor_" + std::to_string(i) + "_joint";
            const double omega = std::abs(bus.read(bus.find("joint/" + joint + "/velocity")));
            const double f = bus.read(bus.find("rotor/" + joint + "/thrust"));
            fastest = std::max(fastest, omega);
            if (k > 1000) {  // the last half second, at the limit
                thrust += f / 500.0;
                atTopSpeed += f * (top * top) / (omega * omega) / 500.0;
            }
        }
    }
    std::printf(
        "top speed %.0f rad/s; limited to %.0f %%: fastest rotor %.0f rad/s (%.3f of top), "
        "%.1f N, %.0f %% of %.1f N at top speed\n",
        top, 100.0 * kLimit, fastest, fastest / top, thrust, 100.0 * thrust / atTopSpeed,
        atTopSpeed);
    EXPECT_LT(fastest, std::sqrt(kLimit) * top * 1.01);  // never past the cap
    EXPECT_GT(fastest, std::sqrt(kLimit) * top * 0.97);  // and it does reach it
    EXPECT_NEAR(thrust / atTopSpeed, kLimit, 0.03);
}

// RFC 0013: --emi-motor-leads lays a wire from the battery to each motor carrying its bus
// current; the hovering quadcopter's magnetometer then reads a disturbance that grows with the
// current, and says what it does to its heading.
TEST(Multicopter, ItsMotorLeadsDisturbItsCompass) {
    // The battery under the frame and back from its middle: a symmetric star of leads from right
    // under the compass would cancel there, and a lead passing within a centimetre of it would
    // swamp the Earth's field (as it does on a real drone).
    const std::array<double, 3> battery{-0.06, 0.0, -0.06};
    const auto hover = [&](std::optional<double> leads) {
        ServerOptions o;
        o.robotPath = LAGRANGE_TEST_QUAD_USDA;
        o.worldPath.clear();
        o.ros = false;
        o.realTimeFactor = 0.0;
        o.airDensityKgM3 = 1.225;
        o.geoOrigin = coords::Geodetic{47.397742 * std::numbers::pi / 180.0,
                                       8.545594 * std::numbers::pi / 180.0, 488.0};
        o.emiMotorLeads = leads;
        o.emiBatteryAt = battery;
        auto server = std::make_unique<Server>(o);
        const Status loaded = server->load();
        EXPECT_TRUE(loaded.ok()) << loaded.message;
        EXPECT_TRUE(server->stepUntil(SimTime{6'000'000'000}).ok());  // up and hovering
        return server;
    };
    auto quiet = hover(std::nullopt);
    EXPECT_TRUE(quiet->read("sensor/magnetometer/interference").empty());  // no victim
    auto led = hover(0.005);
    const double share = led->read("sensor/magnetometer/interference").at(0);
    const double heading = led->read("sensor/magnetometer/heading_error").at(0);
    double current = 0.0;
    for (int i = 1; i <= 4; ++i) {
        current += led->read("motor/rotor_" + std::to_string(i) + "_motor/bus_current").at(0);
    }
    std::printf(
        "hover: %.2f A in the leads, compass disturbed by %.2f %% of the Earth's field, "
        "heading off by %.3f deg\n",
        current, 100.0 * share, heading * 180.0 / std::numbers::pi);
    EXPECT_GT(current, 1.0);
    EXPECT_GT(share, 0.0);
    EXPECT_LT(share, 1.0);  // a disturbance, not a new field
    EXPECT_TRUE(std::isfinite(heading));
    // Wider leads, the same currents: a larger loop, a larger field.
    auto wide = hover(0.02);
    EXPECT_GT(wide->read("sensor/magnetometer/interference").at(0), share);
}

// QA 2026-11-12 #23: a second multicopter spawned beside the first is passive (cli.md "Spawning
// robots"); the first flies on. The controller took every RotorAPI_1 joint of the run as one
// airframe, found rotors on two bodies and gave up, so both stood on the ground.
TEST(Multicopter, ASecondQuadcopterSpawnedBesideItLeavesItFlying) {
    ServerOptions o;
    o.robotPath = LAGRANGE_TEST_QUAD_USDA;
    o.worldPath.clear();
    o.ros = false;
    o.realTimeFactor = 0.0;
    o.airDensityKgM3 = 1.225;
    Server server(o);
    const Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(SimTime{500'000'000}).ok());
    const double ground = server.read("body/base_link/pose").at(2);
    const auto mate = server.spawnRobot(LAGRANGE_TEST_QUAD_USDA, "q2", Pose{{3.0, 0.0, 0.0}, {}});
    ASSERT_TRUE(mate) << mate.error().message;
    ASSERT_TRUE(server.stepUntil(SimTime{6'000'000'000}).ok());
    const double flown = server.read("body/base_link/pose").at(2);
    EXPECT_NEAR(flown - ground, 2.0, 0.3) << "the first hovers 2 m above its start";
    // The copy stays where it was put, its rotors still: robots after the first are passive.
    EXPECT_LT(server.read("body/q2_base_link/pose").at(2), ground + 0.2);
}

}  // namespace
}  // namespace lagrange::server
