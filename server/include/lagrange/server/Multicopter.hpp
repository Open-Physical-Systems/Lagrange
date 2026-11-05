#pragma once

#include "lagrange/module/IModule.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::server {

/// A built-in flight controller for multicopters, the counterpart of the AMR's demo drive: it
/// flies the robot when no autopilot does (PX4 or ArduPilot through the MAVLink bridge). It
/// holds a position set point: x and y where the robot starts, and `altitudeM` above the
/// start once `takeoffAfterS` has passed. With `waitForCommand` (`--no-demo`) it stays on the
/// ground instead until a command asks to climb, then flies by commands only, and lands and
/// stops its rotors when commanded down to the ground.
///
/// In angle mode (FlightCommand::Mode::Angle) the horizontal position loop is off: the command
/// tilts it and the wind carries it; the height loop stays.
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
        // Limits on what a velocity command may ask for.
        double maxHorizontalMps{5.0};
        double maxVerticalMps{2.0};
        double maxYawRateRadps{1.5};
        // Angle mode: the tilt at full stick (a command of maxHorizontalMps), the same as the
        // limit position mode never asks beyond.
        double maxAngleTiltRad{0.5235987755982988};  // 30°
        double minHeightM{0.3};      // m above the start the set point never goes below
        bool waitForCommand{false};  // on the ground until a command asks to climb
    };

    /// A velocity command: forward and left in the drone's heading frame, up, and yaw rate.
    struct FlightCommand {
        /// Position: forward and left are speeds over the ground, and it holds where it is
        /// (GPS position mode: it leans into the wind as far as it must). Angle: forward and left
        /// over maxHorizontalMps are the share of full stick, which tilts it up to
        /// maxAngleTiltRad; nothing holds it in place, so the wind carries it (angle mode with
        /// altitude hold). Up and yaw rate are the same in both.
        enum class Mode { Position, Angle };
        double forwardMps{0.0};
        double leftMps{0.0};
        double upMps{0.0};
        double yawRateRadps{0.0};
        Mode mode{Mode::Position};
    };
    /// Where commands come from (ROS /cmd_vel); nullopt: none recent, hold where it is.
    using CommandSource = std::function<std::optional<FlightCommand>()>;

    explicit MulticopterController(Settings settings, CommandSource commands = {})
        : settings_(settings), commands_(std::move(commands)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;
    void shiftOrigin(const std::array<double, 3>& shift) override;

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
    CommandSource commands_;
    std::array<double, 3> setpoint_{};  // world-local, m: where it holds or flies to
    double heading_{0.0};               // rad, the yaw it holds
    bool airborne_{false};              // waitForCommand: taken off, not landed since
    SignalId pose_, twist_;
    // With the weather (config "weather_air"): its air density against the one the mixer was
    // built for, as a flight controller's hover-thrust estimate follows the air (an ideal one).
    SignalId densityIn_;
    double designDensity_{0.0};
};

}  // namespace lagrange::server
