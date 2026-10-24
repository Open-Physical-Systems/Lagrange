#pragma once

#include "lagrange/electrical/Models.hpp"
#include "lagrange/module/IModule.hpp"

#include <string>
#include <vector>

namespace lagrange::electrical {

/// The electrical domain at the `averaged` fidelity (docs/design/domains/electrical.md): the
/// robot's power system as the stage describes it with the electrical schemas (RFC 0002).
///
/// - Every ElectricalMotorAPI_1 is a drive (Models.hpp: dq PMSM/BLDC or brushed DC with PI
///   current loops) that turns joint/<j>/torque_command into joint/<j>/torque, limited by its
///   current limit and by the voltage of the bus it draws from.
/// - Every ElectricalBusAPI_1 is a node with its capacitance, solved each step with the
///   capacitor's exact companion (backward Euler of the node equation, so stiff batteries and
///   small capacitors stay stable) against the batteries' Thévenin sources and the currents the
///   drives and loads draw at the previous voltage.
/// - Every BatteryAPI_1 is an equivalent circuit (OCV curve, r0(T), two RC pairs) on the bus its
///   port connects to; charging beyond maxChargeCurrent is refused, so regeneration raises the
///   bus voltage, and a brake resistor (elec:bus:brakeResistor) switches in above the
///   overvoltage trip.
/// - Below elec:bus:undervoltageTrip the bus browns out: its loads switch off until the voltage
///   recovers by kRecoveryFraction, and its drives trip (motor/<m>/fault = 1) and stay off
///   until the bus has recovered and their torque command is back to zero, as a drive's
///   undervoltage fault latches. Above the overvoltage trip without a brake resistor the drives
///   switch off (fault = 2) until it recovers.
/// - ElectricalLoadAPI_1 loads draw their power (load/<l>/power_demand when connected).
/// - Joints with a torque command but no motor are driven ideally, as the substitute does.
///
/// Not yet: converters (ElectricalConverterAPI_1 is refused), velocity and voltage control
/// modes, FMU behaviour models (ignored at this fidelity), motors on prismatic joints, the
/// rotor inertia (physics does not take it yet). Default step 100 µs.
class ElectricalNetwork final : public IModule {
public:
    /// The fraction above the undervoltage trip (or below the overvoltage trip) at which a
    /// tripped bus recovers: hysteresis so that a bus at the threshold does not chatter. A
    /// modelling choice, not a hardware value.
    static constexpr double kRecoveryFraction = 0.02;

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// What it simulates, after declare: names as in the signal names.
    [[nodiscard]] std::vector<std::string> motors() const;
    [[nodiscard]] std::vector<std::string> buses() const;
    [[nodiscard]] std::vector<std::string> idealJoints() const;

private:
    struct Motor {
        std::string name, joint;
        MotorParams params;
        MotorState state;
        std::size_t bus{0};
        bool tripped{false};  // undervoltage fault, latched
        SignalId command, velocity, temperature, derating;
        SignalId torque, currentD, currentQ, voltageD, voltageQ, busCurrent, loss, fault;
    };
    struct Battery {
        std::string name;
        BatteryParams params;
        BatteryState state;
        double initialSoc{1.0};
        std::size_t bus{0};
        double current{0.0};  // A, positive discharging, this step
        SignalId temperature, soc, terminalVoltage, currentOut, loss;
    };
    struct Bus {
        std::string name;
        double capacitance{0.0}, undervoltage{0.0}, overvoltage{0.0}, brakeResistance{0.0};
        double voltage{0.0};
        double sink{0.0};  // A drawn by drives and loads this step
        bool brownout{false};
        bool overvoltageTripped{false};
        SignalId voltageOut, brownoutOut;
    };
    struct Load {
        std::string name;
        bool resistive{false};
        double power{0.0}, minVoltage{0.0};
        double resistance{0.0};  // ohm, resistive loads: set at init from the bus voltage
        std::size_t bus{0};
        SignalId demand, currentOut, poweredOut;
    };
    struct Passthrough {
        std::string joint;
        SignalId command, torque;
    };

    void solveBus(std::size_t b, double dt, std::vector<double>& temperatures);

    std::vector<Motor> motors_;
    std::vector<Battery> batteries_;
    std::vector<Bus> buses_;
    std::vector<Load> loads_;
    std::vector<Passthrough> ideal_;
    std::vector<double> batteryTemperature_;  // K, read each step
    std::vector<bool> limitedScratch_;        // batteries at their charge limit, per solve
};

}  // namespace lagrange::electrical
