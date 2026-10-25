#pragma once

// Internal to lagrange_mavlink: the electronic speed controllers both autopilot bridges drive.

#include "lagrange/module/IModule.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace lagrange::mavlink {

/// One ESC per RotorAPI_1 joint, in joint-name order (rotor_1 is the autopilot's motor 1): an
/// output u in [0, 1] is a speed set point u·maxSpeed in the rotor's own direction, held by a
/// speed loop whose torque command, with the propeller's drag as feed-forward, goes to the
/// electrical domain (joint/<j>/torque_command) like any controller's.
class Escs {
public:
    Status declare(const WorldView& world, SignalDeclarer& d, double airDensity, double maxSpeed,
                   double speedGain) {
        rotors_.clear();
        maxSpeed_ = maxSpeed;
        gain_ = speedGain;
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
            const double target = r.spin * u * maxSpeed_;
            const double feedForward = r.spin * r.torquePerOmega2 * target * target;
            bus.write(r.command,
                      u > 0.0 ? feedForward + gain_ * (target - bus.read(r.velocity)) : 0.0);
        }
    }

    [[nodiscard]] std::size_t count() const { return rotors_.size(); }

private:
    struct Rotor {
        double spin{1.0};
        double torquePerOmega2{0.0};
        SignalId velocity, command;
    };
    std::vector<Rotor> rotors_;
    double maxSpeed_{0.0};
    double gain_{0.0};
};

}  // namespace lagrange::mavlink
