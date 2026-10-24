// The program of docs/guide/writing-a-module.md: the sample AMR in the sample warehouse, driven
// by the demo figure-eight, with the tutorial's Odometer wired next to the MuJoCo backend.
//
//   odometer_demo [SECONDS]      prints odometry against the true pose once a second
//   odometer_demo --wrong-unit   adds a module that reads a wheel speed in m/s: wiring refuses
#include "lagrange/electrical/IdealTorqueSource.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/tutorial/Odometer.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace {

using namespace lagrange;
using namespace lagrange::literals;

/// A module with a mistake: it reads the wheel speed as a linear velocity.
class WrongUnitReader final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", "wrong_unit", 0, 1, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 10_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) override {
        SignalSpec spec;
        spec.name = "joint/left_wheel_joint/velocity";
        spec.unit = units::m_per_s;  // the writer says rad/s
        speed_ = declarer.input(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus&) override { return {}; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId speed_;
};

std::string asset(const char* relative) {
    try {
        return (ament_index_cpp::get_package_share_path("lagrange_assets") / relative).string();
    } catch (const std::exception&) {
        return {};
    }
}

/// Heading (rad, about z) of a unit quaternion (w, x, y, z).
double yaw(const double* q) {
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

}  // namespace

int main(int argc, char** argv) {
    const bool wrongUnit = argc > 1 && std::string_view(argv[1]) == "--wrong-unit";
    const double seconds = argc > 1 && !wrongUnit ? std::atof(argv[1]) : 10.0;

    // 1. The world: the robot and the fixed scenery, as lagrange_server loads them.
    auto robot = server::loadUrdf(asset("robots/amr_diffdrive/amr_diffdrive.urdf"));
    if (!robot) {
        std::fprintf(stderr, "%s\n", robot.error().c_str());
        return 1;
    }
    if (auto added = server::addUrdf(*robot, asset("worlds/warehouse_small/warehouse_small.urdf"));
        !added) {
        std::fprintf(stderr, "%s\n", added.error().c_str());
        return 1;
    }

    // 2. The modules. None knows about the others; the scheduler connects them by signal name.
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(std::make_unique<server::DriveCommand>(server::DriveCommand::Settings{},
                                                         nullptr));  // the demo figure-eight
    server::DiffDriveController::Settings drive;
    drive.leftJoint = "left_wheel_joint";
    drive.rightJoint = "right_wheel_joint";
    scheduler.add(std::make_unique<server::DiffDriveController>(drive));
    scheduler.add(std::make_unique<electrical::IdealTorqueSource>());  // torque_command -> torque
    scheduler.add(std::make_unique<tutorial::Odometer>(tutorial::Odometer::Settings{}));
    if (wrongUnit) {
        scheduler.add(std::make_unique<WrongUnitReader>());
    }

    // 3. Wiring: names matched, units and types checked, execution order fixed.
    if (auto wired = scheduler.wire(WorldView(robot->world)); !wired) {
        std::fprintf(stderr, "the simulation could not start:\n");
        for (const WiringError& error : wired.error()) {
            std::fprintf(stderr, "  %s\n", to_string(error).c_str());
        }
        return 1;
    }

    // 4. Run, and compare the odometer with the true pose from physics.
    const SignalBus& bus = scheduler.bus();
    const SignalId distance = bus.find("base/odometer/distance");
    const SignalId heading = bus.find("base/odometer/heading");
    const SignalId pose = bus.find("body/base_link/pose");
    const double startYaw = yaw(bus.readN(pose).data() + 3);
    std::printf("%6s %12s %12s %12s\n", "t (s)", "odom (m)", "odom (rad)", "true (rad)");
    for (int t = 1; t <= static_cast<int>(seconds); ++t) {
        if (Status status =
                scheduler.stepUntil(SimTime{static_cast<std::int64_t>(t) * 1'000'000'000});
            !status.ok()) {
            std::fprintf(stderr, "%s\n", status.message.c_str());
            return 1;
        }
        // Both headings wrapped to (-pi, pi] for the comparison.
        constexpr double kTurn = 2.0 * 3.141592653589793;
        const double trueYaw = yaw(bus.readN(pose).data() + 3) - startYaw;
        std::printf("%6d %12.3f %12.3f %12.3f\n", t, bus.read(distance),
                    std::remainder(bus.read(heading), kTurn), std::remainder(trueYaw, kTurn));
    }
    return 0;
}
