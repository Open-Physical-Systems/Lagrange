#pragma once

// Internal to lagrange_mavlink: the electronic speed controllers both autopilot bridges drive.

#include "lagrange/mavlink/RotorTopSpeed.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/services/Log.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::mavlink {

/// One ESC per RotorAPI_1 joint, in joint-name order (rotor_1 is the autopilot's motor 1): an
/// output u in [0, 1] is a speed set point u·top in the rotor's own direction, held by a speed
/// loop whose torque command, with the propeller's drag as feed-forward, goes to the electrical
/// domain (joint/<j>/torque_command) like any controller's. Each rotor's top speed is the one
/// given (--mavlink-max-rotor-speed), else its loaded top speed from its motor and battery
/// (rotorTopSpeed), so that full output is what the motor can do.
///
/// With `idealMotors` (the electrical domain off: its ideal torque source delivers the command
/// exactly) nothing stands between the speed loop and the rotor, as for the built-in controller
/// (MulticopterController): each rotor's torque command is bounded by its drive's, the
/// propeller's torque at the rotor's loaded top speed (rotorTopSpeed, or the full-output speed
/// when that is higher), and the speed gain is cut so that the loop's gain per ESC step,
/// K·dt/J (J the rotor's inertia about its axis), stays at kSpeedLoopGain: without the drives'
/// current loops, a set point step otherwise asks for tens of N·m of a 25 g propeller, which
/// the frame feels as the motors' reaction, and at one and above the loop overshoots each step.
class Escs {
public:
    /// Highest speed loop gain per ESC step with ideal motors: below one the loop does not
    /// overshoot; half leaves margin for the propeller's drag and the physics' substeps.
    static constexpr double kSpeedLoopGain = 0.5;

    /// `thrustLimit`: the share of full thrust the autopilot's full output gives, by a top speed
    /// of sqrt(thrustLimit) times the rotor's (thrust goes with the speed squared).
    /// `idealMotors`, `stepS` (the ESCs' step, s): see the class comment; without ideal motors
    /// the loop is the same as before them, bit for bit.
    Status declare(const WorldView& world, SignalDeclarer& d, double airDensity,
                   std::optional<double> maxSpeed, double speedGain, std::string_view who,
                   double thrustLimit = 1.0, bool idealMotors = false, double stepS = 0.0) {
        rotors_.clear();
        std::map<std::string, const SchemaInstance*> byName;
        for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
            byName[std::string(world.signalInstanceName(s->prim))] = s;
        }
        if (byName.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "an autopilot needs rotors (RotorAPI_1)");
        }
        for (const auto& [name, s] : byName) {
            const double diameter = 2.0 * s->real("aero:rotor:radius").value_or(0.0);
            Rotor r;
            // The motor's sense about the joint axis: its direction about the thrust, turned for
            // a pusher, whose joint axis points against the thrust.
            r.spin = (s->token("aero:rotor:direction").value_or("ccw") == "ccw" ? 1.0 : -1.0) *
                     (s->token("aero:rotor:mount").value_or("puller") == "pusher" ? -1.0 : 1.0);
            // Q = CP·ρ·n²·D⁵/(2π) with n = ω/(2π): CP·ρ·D⁵/(8π³) per (rad/s)².
            r.torquePerOmega2 = s->real("aero:rotor:powerCoeff").value_or(0.0) * airDensity *
                                std::pow(diameter, 5) /
                                (8.0 * std::numbers::pi * std::numbers::pi * std::numbers::pi);
            const RotorTopSpeed top = maxSpeed
                                          ? RotorTopSpeed{*maxSpeed, "--mavlink-max-rotor-speed"}
                                          : rotorTopSpeed(world, *s, airDensity);
            const double share = std::clamp(thrustLimit, 0.0, 1.0);
            r.top = top.speed * std::sqrt(share);
            std::string limited;
            if (share < 1.0) {
                limited = ", thrust limited to " + std::to_string(std::lround(100.0 * share)) +
                          " % of " + std::to_string(std::lround(top.speed)) + " rad/s's";
            }
            services::log(services::LogLevel::Info, std::string(who),
                          name + ": full output is " + std::to_string(std::lround(r.top)) +
                              " rad/s (" + top.from + limited + ")");
            r.gain = speedGain;
            if (idealMotors) {
                // The drive's torque at the rotor's loaded top speed (its current limit, when
                // that is what limits it), as the electrical domain would bound it; never below
                // what holding full output takes.
                const RotorTopSpeed drive = rotorTopSpeed(world, *s, airDensity);
                const double capSpeed = std::max({drive.speed, top.speed, r.top});
                r.torqueLimit = r.torquePerOmega2 * capSpeed * capSpeed;
                const double inertia = axisInertia(world, s->prim);
                if (inertia > 0.0 && stepS > 0.0) {
                    r.gain = std::min(speedGain, kSpeedLoopGain * inertia / stepS);
                }
                services::log(services::LogLevel::Info, std::string(who),
                              name + ": ideal motor, torque command bounded to " +
                                  std::to_string(r.torqueLimit) + " N·m (" + drive.from +
                                  "), speed gain " + std::to_string(r.gain) + " N·m per rad/s");
            }
            const std::string base = "joint/" + name + "/";
            SignalSpec velocity;
            velocity.name = base + "velocity";
            velocity.unit = units::rad_per_s;
            velocity.feedback = true;
            r.velocity = d.input(velocity);
            SignalSpec command;
            command.name = base + "torque_command";
            command.unit = units::N_m;
            r.command = d.output(command);
            rotors_.push_back(r);
        }
        return {};
    }

    void stop(SignalBus& bus) const {
        for (const Rotor& r : rotors_) {
            bus.write(r.command, 0.0);
        }
    }

    /// Drives rotor i from outputs[i] (0 to 1); rotors without an output stop.
    void drive(SignalBus& bus, std::span<const double> outputs) const {
        for (std::size_t i = 0; i < rotors_.size(); ++i) {
            const Rotor& r = rotors_[i];
            const double u = i < outputs.size() ? std::clamp(outputs[i], 0.0, 1.0) : 0.0;
            const double target = r.spin * u * r.top;
            const double feedForward = r.spin * r.torquePerOmega2 * target * target;
            double command = u > 0.0 ? feedForward + r.gain * (target - bus.read(r.velocity)) : 0.0;
            if (r.torqueLimit > 0.0) {
                command = std::clamp(command, -r.torqueLimit, r.torqueLimit);
            }
            bus.write(r.command, command);
        }
    }

    [[nodiscard]] std::size_t count() const { return rotors_.size(); }
    /// Rotor i's torque bound (N·m; 0: none) and speed gain (N·m per rad/s), for tests.
    [[nodiscard]] double torqueLimit(std::size_t i) const { return rotors_.at(i).torqueLimit; }
    [[nodiscard]] double gain(std::size_t i) const { return rotors_.at(i).gain; }

private:
    struct Rotor {
        double spin{1.0};
        double top{0.0};  // rad/s at full output
        double torquePerOmega2{0.0};
        double gain{0.0};         // N·m per rad/s
        double torqueLimit{0.0};  // N·m (ideal motors); 0: none
        SignalId velocity, command;
    };

    /// The inertia (kg·m²) about its axis of what the rotor joint `prim` turns: its child body
    /// about the axis through the joint (in the child frame), plus the joint's armature; 0 when
    /// no joint has that prim.
    static double axisInertia(const WorldView& world, PrimId prim) {
        const World::Joints& joints = world.joints();
        const World::Bodies& bodies = world.bodies();
        for (std::size_t j = 0; j < joints.prim.size(); ++j) {
            if (joints.prim[j] != prim) {
                continue;
            }
            const std::uint32_t b = joints.child[j].index;
            const std::array<double, 3> a = joints.axis[j];
            const double norm = std::hypot(a[0], a[1], a[2]);
            if (norm <= 0.0 || b >= bodies.mass.size()) {
                return 0.0;
            }
            const std::array<double, 3> n{a[0] / norm, a[1] / norm, a[2] / norm};
            const auto& in = bodies.inertiaLocal[b];  // xx, yy, zz, xy, xz, yz about the COM
            const double aboutCom = (in[0] * n[0] * n[0]) + (in[1] * n[1] * n[1]) +
                                    (in[2] * n[2] * n[2]) + (2.0 * in[3] * n[0] * n[1]) +
                                    (2.0 * in[4] * n[0] * n[2]) + (2.0 * in[5] * n[1] * n[2]);
            // Parallel axes: the COM's distance from the axis through the joint.
            const auto& c = bodies.comLocal[b];
            const double along = (c[0] * n[0]) + (c[1] * n[1]) + (c[2] * n[2]);
            const double off2 = (c[0] * c[0]) + (c[1] * c[1]) + (c[2] * c[2]) - (along * along);
            const double armature = j < joints.armature.size() ? joints.armature[j] : 0.0;
            return aboutCom + (bodies.mass[b] * std::max(off2, 0.0)) + armature;
        }
        return 0.0;
    }

    std::vector<Rotor> rotors_;
};

}  // namespace lagrange::mavlink
