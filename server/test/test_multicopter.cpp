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

}  // namespace
}  // namespace lagrange::server
