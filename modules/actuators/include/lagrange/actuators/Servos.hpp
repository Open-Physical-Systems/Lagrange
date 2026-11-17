#pragma once

// The actuators domain's servo module (RFC 0020 §4.3): every joint with ServoAPI_1, stepped at
// 1 ms. It reads the servo's command (act/<s>/pulse_width through the pulse map, or
// act/<s>/goal_position; a smart servo's goal_velocity or goal_current in those operating modes),
// moves the set point through the profile limits, and drives the joint by one of three paths:
//
//   - `voltage`: the joint's ElectricalMotorAPI_1 (dc_brushed) is in voltage mode and the
//     electrical domain runs: the board's loop runs here and writes joint/<j>/voltage_command,
//     the duty times bus/<b>/voltage (a smart servo's current held to elec:maxCurrent);
//   - `position`: the motor is in position mode: the drive runs the loop (at its own step) and
//     this module writes the set point to joint/<j>/position_command (NaN: torque off);
//   - `ideal`: no motor, or the electrical domain substituted: joint/<j>/torque_command from the
//     datasheet's torque line, τ = τ_stall·(u − ω/ω0).
//
// Out: act/<s>/present_position (quantised by the resolution), present_velocity, present_load
// (N·m), goal (the set point), overload (Bool: the smart servo's overload or overheating
// shutdown, latched until torque_enable goes off and on), and with a motor present_current (A)
// and temperature (K). In: act/<s>/torque_enable (optional, 1).
//
// Substitutes (§3.3): `ideal` runs the same paths with a perfect board (no deadband, no sensing
// step, no profile, no shutdown); `constant` holds the start position and ignores commands;
// `zero` (Mode::Off) turns every servo's torque off.

#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/actuators/Servo.hpp"
#include "lagrange/module/IModule.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::actuators {

/// One servo of the stage, with its attributes, its motor and the path it drives the joint by.
struct ServoSpec {
    enum class Drive : std::uint8_t { Ideal, Voltage, Position };
    std::string name;   // the joint's signal instance name: act/<name>/...
    std::string joint;  // the joint's name in World::Joints
    PrimId prim{};
    ServoParams params;
    Drive drive{Drive::Ideal};
    std::string motor;              // the motor's signal instance name; empty without one
    std::string bus;                // the motor's bus's signal instance name (voltage path)
    Armature armature;              // the motor through its gear (voltage path and present_current)
    double start{0.0};              // rad, the joint's starting position
    double lower{0.0}, upper{0.0};  // rad, the joint's range (lower ≥ upper: none)
};

/// Every servo of the world. `electrical`: whether the electrical domain runs (else every servo
/// takes the ideal path). An error names what is wrong (a required attribute missing, a motor in
/// a mode a servo cannot drive).
[[nodiscard]] std::vector<ServoSpec> servosOf(const WorldView& world, bool electrical,
                                              std::string& error);

class Servos final : public IModule {
public:
    enum class Mode : std::uint8_t { Full, Ideal, Hold, Off };

    /// `commands`: given outside the bus (Server::commandJoint), keyed "<servo>/position",
    /// "<servo>/velocity", "<servo>/current" and "<servo>/torque_enable".
    explicit Servos(Mode mode = Mode::Full, std::shared_ptr<Commands> commands = {})
        : mode_(mode), commands_(std::move(commands)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Config key: 1 when the electrical domain runs (the default), 0 when it is substituted.
    static constexpr const char* kElectricalKey = "electrical";

private:
    struct In {
        SignalId position, velocity, goalPosition, pulseWidth, goalVelocity, goalCurrent,
            torqueEnable, busVoltage, current, torque, temperature;
    };
    struct Out {
        SignalId command, presentPosition, presentVelocity, presentLoad, presentCurrent,
            temperature, goal, overload;
    };
    struct State {
        double goal{0.0};  // rad; or rad/s, A in those modes
        double busGoalPosition{0.0}, busPulseWidth{0.0}, busGoalVelocity{0.0},
            busGoalCurrent{0.0};  // last seen on the bus (NaN: nothing yet)
        double enable{1.0}, busEnable{1.0};
        double lastTorque{0.0};
        Profile profile;
        LoopState loop;
        Shutdown shutdown;
    };

    Mode mode_;
    std::shared_ptr<Commands> commands_;
    std::uint64_t seenGeneration_{0};
    std::vector<ServoSpec> specs_;
    std::vector<In> in_;
    std::vector<Out> out_;
    std::vector<State> state_;
};

}  // namespace lagrange::actuators
