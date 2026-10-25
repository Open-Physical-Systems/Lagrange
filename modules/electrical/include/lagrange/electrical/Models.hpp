#pragma once

// The averaged electrical models of docs/design/domains/electrical.md §2, free of the bus and
// the scheduler so that the validation tests can drive them directly. SI units throughout.
#include <vector>

namespace lagrange::electrical {

/// A motor with its drive: a dq-frame PMSM/BLDC (peak-amplitude convention) or a brushed DC
/// machine, with PI current loops tuned by pole-zero cancellation to `currentLoopBandwidth`.
struct MotorParams {
    bool dq{true};               // false: brushed DC, one axis (the q fields)
    double resistance{0.0};      // ohm, phase (or armature) at referenceTemperature
    double inductanceD{0.0};     // H
    double inductanceQ{0.0};     // H
    double torqueConstant{0.0};  // N·m per peak q-axis ampere (brushed: per ampere), motor shaft
    int polePairs{1};
    double gearRatio{1.0};                // motor revolutions per joint revolution
    double gearEfficiency{1.0};           // 0..1
    double maxCurrent{0.0};               // A, peak
    double referenceTemperature{298.15};  // K
    double resistanceTempCoeff{0.0};      // 1/K
    double currentLoopBandwidth{6283.0};  // rad/s

    /// Permanent-magnet flux linkage (Wb) from the torque constant: Kt = 1.5·p·λ for the
    /// peak-amplitude dq convention (electrical.md §2).
    [[nodiscard]] double fluxLinkage() const;
    /// Back-EMF per mechanical rad/s, line-neutral peak: p·λ = Kt / 1.5 (dq), Kt (brushed).
    [[nodiscard]] double backEmfConstant() const;
    /// Winding resistance at `temperature` (K).
    [[nodiscard]] double resistanceAt(double temperature) const;
};

struct MotorState {
    double id{0.0}, iq{0.0};                // A
    double integralD{0.0}, integralQ{0.0};  // V, the PI integrators
};

struct MotorInputs {
    double torqueCommand{0.0};   // N·m at the joint
    double jointVelocity{0.0};   // rad/s
    double busVoltage{0.0};      // V
    double temperature{298.15};  // K, of the winding
    double derating{1.0};        // 0..1, scales the current limit
    bool enabled{true};          // false: the drive is off (fault) and the currents are zero
    /// Speed prediction (architecture.md §8): the inertia the joint turns, kg·m², and the load
    /// torque on it, N·m. With an inertia the q-axis current and the joint speed are advanced
    /// together, exactly for the voltages held over the step, and the joint torque is the
    /// step's average, so the impulse matches the predicted speed change.
    double inertia{0.0};
    double loadTorque{0.0};
};

struct MotorOutputs {
    double jointTorque{0.0};    // N·m
    double vd{0.0}, vq{0.0};    // V, applied
    double busCurrent{0.0};     // A, drawn from the bus (negative when regenerating)
    double lossPower{0.0};      // W, copper loss
    double motorTorque{0.0};    // N·m at the motor shaft
    double jointVelocity{0.0};  // rad/s at the end of the step, predicted (with an inertia)
};

/// Advances the winding currents by `dt` under constant applied voltages `vd`, `vq` and speed:
/// the exact solution of each axis's R-L equation for inputs held over the step, so
/// unconditionally stable and exact for a voltage step (the cross-coupling and back-EMF terms
/// use the speed and currents at the start of the step).
void advanceCurrents(const MotorParams& p, MotorState& s, double vd, double vq,
                     double motorVelocity, double resistance, double dt);

/// One step of the drive: current references from the torque command (limited by maxCurrent and
/// derating), PI current control with decoupling, the voltage limit of the bus (|v| ≤ Vbus/√3
/// for a sine-modulated three-phase inverter, Vbus for an H bridge), then the winding currents.
MotorOutputs stepMotor(const MotorParams& p, MotorState& s, const MotorInputs& in, double dt);

/// A battery pack as an equivalent circuit (electrical.md §2): per-cell OCV(SoC) curve, series
/// resistance r0(T) and two RC pairs, seriesCells × parallelCells cells.
struct BatteryParams {
    int seriesCells{1};
    int parallelCells{1};
    double capacity{0.0};                                // C, of the pack
    std::vector<double> ocvSoc;                          // ascending
    std::vector<double> ocvVoltage;                      // V per cell
    double r0{0.0}, r1{0.0}, c1{0.0}, r2{0.0}, c2{0.0};  // per cell
    double referenceTemperature{298.15};                 // K
    double r0TempCoeff{0.0};                             // 1/K
    double cutoffVoltage{0.0};                           // V, pack
    double maxChargeCurrent{0.0};                        // A, pack

    /// Open-circuit voltage of the pack at `soc`, linear between breakpoints, flat outside.
    [[nodiscard]] double openCircuitVoltage(double soc) const;
};

struct BatteryState {
    double soc{1.0};
    double v1{0.0}, v2{0.0};  // V per cell across the RC pairs
    bool cut{false};          // empty: disconnected by its protection
};

/// The pack as a Thévenin source for the bus: terminal voltage = emf − current × resistance.
struct Thevenin {
    double emf{0.0};         // V
    double resistance{0.0};  // ohm, > 0
};

[[nodiscard]] Thevenin theveninOf(const BatteryParams& p, const BatteryState& s,
                                  double temperature);

/// Advances the state of charge and the RC pairs by `dt` with the pack current `current`
/// (A, positive discharging) held over the step (exact for the RC pairs).
void advanceBattery(const BatteryParams& p, BatteryState& s, double current, double dt);

}  // namespace lagrange::electrical
