#pragma once

#include "lagrange/module/IModule.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::rotor_aero {

/// The air density, kg/m³, from the module config ("air_density": the profile's
/// rotor_aero.air_density or the server's --air-density). A site parameter (altitude and
/// weather), so there is no default and the rotor modules refuse to start without it
/// (CLAUDE.md §10). The ISA sea-level value is 1.225 kg/m³.
inline constexpr const char* kAirDensityKey = "air_density";

/// Rotor parameters, from RotorAPI_1 (formats.md §3.3).
struct RotorParams {
    double radius{0.0};       // m
    double thrustCoeff{0.0};  // CT, T = CT·ρ·n²·D⁴, n in rev/s
    double powerCoeff{0.0};   // CP, Q = CP·ρ·n²·D⁵ / (2π)
    double spinSign{1.0};     // +1 ccw: ω > 0 about the joint axis makes thrust; −1 cw
};

/// Static thrust (N, along the joint axis) and shaft torque (N·m, about the joint axis, opposing
/// the rotation) at joint speed `omega` (rad/s) in air of density `rho`. Spinning against its
/// direction a rotor pushes the other way, with the same coefficients (a simplification).
struct Load {
    double thrust{0.0};
    double torque{0.0};
};
[[nodiscard]] Load staticLoad(const RotorParams& p, double omega, double rho);

/// Cheeseman–Bennett ground effect (aerodynamics.md §2): T/T∞ = 1 / (1 − (R / 4z)²) at height
/// z above the ground. Below z = kGroundEffectMinHeight·R the factor is held at its value there
/// (the formula diverges at z = R/4, where the design's clamp would sit).
inline constexpr double kGroundEffectMinHeight = 0.5;  // ×R; the validated range starts here
[[nodiscard]] double groundEffect(double radius, double height);

/// The rotor_aero domain at the `coefficients` fidelity (docs/design/domains/aerodynamics.md):
/// every joint with RotorAPI_1 is a propeller whose static thrust and shaft torque follow its
/// coefficients, with a first-order inflow lag (config "inflow_time_constant", s; 0, the
/// default, for none) and ground effect against the world's ground plane (config
/// "ground_effect", default true).
///
/// Per rotor joint j with child link c it reads joint/<j>/velocity and body/<c>/pose, and writes
/// rotor/<j>/thrust (N), rotor/<j>/aero_torque (N·m, on the rotor about its axis),
/// rotor/<j>/induced_velocity (m/s, momentum theory at the current thrust), the thrust as
/// body/<c>/wrench/rotor (summed by WrenchSum into body/<c>/external_wrench; force applied at
/// the link's origin, torque about its centre of mass) and the shaft torque as
/// joint/<j>/load_torque. Not yet: advance-ratio corrections in forward flight, blade flapping,
/// ground effect over scenery other than the ground plane, the `bem` fidelity. Default step 1 ms.
class RotorAero final : public IModule {
public:
    /// `ideal` makes the substitute (aerodynamics.md §5): T = k·ω² with k from the thrust
    /// coefficient at the configured density, no lag, no ground effect.
    explicit RotorAero(bool ideal = false) : ideal_(ideal) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The rotor joints' names, as in the signal names, after declare.
    [[nodiscard]] std::vector<std::string> rotors() const;

private:
    struct Rotor {
        std::string joint;
        RotorParams params;
        std::array<double, 3> axis{0.0, 0.0, 1.0};  // joint axis in the child frame, unit
        std::array<double, 3> com{};                // child's centre of mass, child frame
        double thrust{0.0};                         // N, the lagged thrust
        SignalId velocity, pose;
        SignalId thrustOut, torqueOut, inducedOut, wrenchOut, loadOut;
    };

    bool ideal_;
    double rho_{0.0};
    double lag_{0.0};  // s
    bool groundEffect_{true};
    bool hasGround_{false};
    std::array<double, 3> groundPoint_{}, groundNormal_{0.0, 0.0, 1.0};
    std::vector<Rotor> rotors_;
};

}  // namespace lagrange::rotor_aero
