#pragma once

#include "lagrange/electrical/FmuHost.hpp"
#include "lagrange/electrical/Models.hpp"
#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <map>
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
/// - An ElectricalConverterAPI_1 is an averaged DC-DC regulator: on its output bus an ideal
///   source at elec:conv:outputVoltage within the node equation, delivering 0 to
///   maxOutputCurrent (no reverse flow; at the limit the bus sags); on its input bus a sink of
///   the output power over the efficiency, drawn at the input voltage one step late, like a
///   load. It stops while its input bus is browned out. One converter per output bus.
/// - Joints with a torque command but no motor are driven ideally, as the substitute does.
///
/// Coupling with physics (architecture.md §8). Config "exchange_step" (s) is the physics step
/// H that the joint velocities arrive at. With it and "speed_prediction" (default true), each
/// drive integrates a local rotor model J_eff·ω̇ = τ − τ_load between physics updates for its
/// back-EMF, J_eff from joint/<j>/effective_inertia (or the world's estimate) and τ_load from
/// the previous window, and resyncs to the true velocity every H. At declare it checks H/τm per
/// motor, τm = J·R/(N²·Kt·Ke) the mechanical time constant at the joint with J estimated from
/// the world (the joint's child body and armature at rest), against kMaxStepRatioPredicted or
/// kMaxStepRatio, and refuses to start with the motor named when the exchange would be unstable.
///
/// Fidelity `fmu`: a motor with elec:behaviorModel is simulated by its FMU (FmuComponent, added
/// by the caller from motorFmuBindings); the network takes its motor/<m>/bus_current into the
/// bus and leaves its joint to it. At `averaged` the behaviour models are ignored.
///
/// elec:controlMode: `torque` reads joint/<j>/torque_command; `velocity` reads
/// joint/<j>/velocity_command through the drive's own PI speed loop, critically damped at
/// elec:velocityLoopBandwidth on the joint's inertia, over the current loops; `voltage` reads
/// joint/<j>/voltage_command straight to the winding. Those two commands are required inputs: a
/// controller writing only torque commands stops the start with MissingInput. On a prismatic
/// joint everything at the joint is linear (MotorParams::gearRatio).
///
/// Not yet: battery FMUs. Default step 100 µs.
/// The joints whose motor is not in torque mode, with its mode: "velocity" or "voltage".
[[nodiscard]] std::map<std::string, std::string> driveModes(const WorldView& world);

class ElectricalNetwork final : public IModule {
public:
    /// H/τm above which the exchange is refused without and with speed prediction
    /// (architecture.md §8). Set from the coupling tests (test_coupling): with prediction a
    /// rotor at H/τm ≈ 8 tracks the monolithic solution within 2 %.
    static constexpr double kMaxStepRatio = 0.5;
    static constexpr double kMaxStepRatioPredicted = 10.0;

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
        bool slide{false};    // on a prismatic joint: force, m/s, kg at the joint
        enum class Mode : std::uint8_t { Torque, Velocity, Voltage } mode{Mode::Torque};
        double velocityBandwidth{0.0};  // rad/s
        double velocityIntegral{0.0};   // N·m or N, the speed loop's
        SignalId velocityCommand, voltageCommand;
        double worldInertia{0.0};  // kg·m², the joint's at rest, from the world
        // Speed prediction between physics updates.
        double predicted{0.0}, loadEstimate{0.0}, lastSample{0.0}, torqueSum{0.0};
        std::uint32_t stepsSince{0};
        bool synced{false};
        SignalId effectiveInertia;
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
        SignalId temperature, soc, terminalVoltage, currentOut, loss, power;
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
    struct Converter {
        std::string name;
        std::size_t input{0}, output{0};  // buses
        double outputVoltage{0.0}, efficiency{1.0}, maxCurrent{0.0};
        double current{0.0};  // A delivered to the output bus this step
        SignalId outputCurrent, inputCurrent, loss;
    };
    struct FmuMotor {  // a motor its FMU simulates
        std::string name;
        std::size_t bus{0};
        SignalId busCurrent;
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
    std::vector<Converter> converters_;
    std::vector<Passthrough> ideal_;
    std::vector<FmuMotor> fmuMotors_;
    std::vector<double> batteryTemperature_;  // K, read each step
    std::vector<double> converterInput_;      // A, each converter's input current this step
    std::vector<bool> limitedScratch_;        // batteries at their charge limit, per solve
    std::uint32_t exchangeSteps_{0};          // electrical steps per physics step; 0: unknown
    bool predict_{false};
};

/// Adds every motor's rotor inertia, reflected through its gear (N²·elec:rotorInertia), to the
/// armature of the joint it drives, so that physics accelerates the rotor too. Call once on the
/// loaded world, before physics is built.
void applyRotorInertia(World& world);

/// The FMUs that simulate motors at fidelity `fmu`: every ElectricalMotorAPI_1 with an
/// elec:behaviorModel, with the placeholders of its map ({joint}, {bus}, {prim}) and its
/// attributes. Add one FmuComponent per binding after the network.
[[nodiscard]] std::vector<FmuBinding> motorFmuBindings(const WorldView& world);

}  // namespace lagrange::electrical
