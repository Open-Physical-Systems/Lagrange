#pragma once

#include "lagrange/module/IModule.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::server {

/// A built-in flight controller for multicopters, the counterpart of the AMR's demo drive: it
/// flies the robot when no autopilot does (PX4 or ArduPilot through the MAVLink bridge). It
/// holds a position set point: x and y where the robot starts, and `altitudeM` above the
/// start once `takeoffAfterS` has passed.
///
/// Cascade, every millisecond: position PD with gravity feed-forward gives the thrust vector;
/// geometric attitude control on SO(3) (T. Lee, M. Leok, N. H. McClamroch, "Geometric tracking
/// control of a quadrotor UAV on SE(3)", IEEE CDC 2010, eqs. 11-12) gives the body torques; a
/// mixer from the rotor positions and each propeller's torque-to-thrust ratio (CP·D / (2π·CT))
/// gives each rotor's thrust; each rotor then has a speed loop, with the propeller's
/// aerodynamic torque as feed-forward, writing joint/<j>/torque_command. Mass, inertia and
/// propeller coefficients come from the world (RotorAPI_1), the air density from the config
/// ("air_density", as rotor_aero).
class MulticopterController final : public IModule {
public:
    struct Settings {
        double altitudeM{2.0};      // m above the start, the hover set point
        double takeoffAfterS{1.0};  // s on the ground before taking off
        double positionHz{0.6};     // natural frequency of the position loop
        double attitudeHz{3.0};     // natural frequency of the attitude loop
        double damping{0.9};        // damping ratio of both
        double speedGain{0.02};     // N·m per rad/s of rotor speed error
    };

    explicit MulticopterController(Settings settings) : settings_(settings) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Whether `world` is a multicopter this controller can fly: three or more RotorAPI_1
    /// joints, all on one body.
    [[nodiscard]] static bool canFly(const WorldView& world);

private:
    struct Rotor {
        std::array<double, 3> position{};  // hub in the base frame, m
        double direction{1.0};             // +1 ccw, −1 cw about the thrust direction (+z)
        double spin{1.0};                  // the same about the joint axis (turned for a pusher)
        double thrustPerOmega2{0.0};       // N per (rad/s)²
        double torquePerOmega2{0.0};       // N·m per (rad/s)²
        SignalId velocity, command;
    };

    Settings settings_;
    std::vector<Rotor> rotors_;
    std::vector<std::array<double, 4>> mixer_;  // per rotor: thrust from (T, τx, τy, τz)
    double mass_{0.0};
    std::array<double, 3> inertia_{};  // base, principal, kg·m²
    std::array<double, 3> home_{};
    bool homed_{false};
    SignalId pose_, twist_;
};

}  // namespace lagrange::server
