#include "lagrange/mavlink/RotorTopSpeed.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <optional>

namespace lagrange::mavlink {

namespace {

constexpr double kLegacyTopSpeed = 850.0;  // rad/s: the sample quadcopter's, before this
constexpr double kThrustToWeight = 2.0;    // without a motor: lift twice the weight
constexpr int kBisections = 80;            // halvings of the speed interval (to 1e-20 of it)

std::string format(const char* pattern, double a, double b = 0.0) {
    std::array<char, 160> text{};
    std::snprintf(text.data(), text.size(), pattern, a, b);
    return text.data();
}

/// The battery's full-charge open-circuit voltage: its cells times the top of its OCV curve.
std::optional<double> fullVoltage(const SchemaInstance& battery) {
    const auto cells = battery.integer("elec:battery:seriesCells");
    const auto ocv = battery.realArray("elec:battery:ocvVoltage");
    if (!cells || *cells < 1 || ocv.empty()) {
        return std::nullopt;
    }
    return static_cast<double>(*cells) * *std::ranges::max_element(ocv);
}

/// The full voltage of the battery that feeds `motor`: the one whose port connects straight to
/// the prim the motor's elec:powerPort names, else the highest of all.
std::optional<double> supplyVoltage(const WorldView& world, const SchemaInstance& motor) {
    const auto power = motor.rel("elec:powerPort");
    std::optional<double> highest;
    for (const SchemaInstance* battery : world.instancesOf("BatteryAPI_1")) {
        const auto volts = fullVoltage(*battery);
        if (!volts) {
            continue;
        }
        highest = std::max(highest.value_or(0.0), *volts);
        for (const SchemaInstance* port : world.instancesOf("PortAPI_1")) {
            if (port->prim != battery->prim || power.empty()) {
                continue;
            }
            const auto to = port->rel("elec:port:" + port->instance + ":connectedTo");
            if (!to.empty() && to[0] == power[0]) {
                return volts;
            }
        }
    }
    return highest;
}

const SchemaInstance* motorOf(const WorldView& world, PrimId joint) {
    for (const SchemaInstance* motor : world.instancesOf("ElectricalMotorAPI_1")) {
        const auto driven = motor->rel("elec:joint");
        if (!driven.empty() && driven[0] == joint) {
            return motor;
        }
    }
    return nullptr;
}

/// The mass of the robot the rotor joint belongs to, kg (0 when the joint is not found).
double robotMass(const WorldView& world, PrimId joint) {
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    for (std::size_t j = 0; j < joints.prim.size(); ++j) {
        if (joints.prim[j] != joint) {
            continue;
        }
        const std::uint32_t robot = bodies.robot.at(joints.child[j].index);
        double mass = 0.0;
        for (std::size_t b = 0; b < bodies.mass.size(); ++b) {
            if (bodies.robot[b] == robot) {
                mass += bodies.mass[b];
            }
        }
        return mass;
    }
    return 0.0;
}

}  // namespace

RotorTopSpeed rotorTopSpeed(const WorldView& world, const SchemaInstance& rotor,
                            double airDensity) {
    const double diameter = 2.0 * rotor.real("aero:rotor:radius").value_or(0.0);
    constexpr double pi = std::numbers::pi;
    // The propeller's torque Q = CP·ρ·n²·D⁵/(2π) with n = ω/(2π): cq·ω² at the joint.
    const double cq = rotor.real("aero:rotor:powerCoeff").value_or(0.0) * airDensity *
                      std::pow(diameter, 5) / (8.0 * pi * pi * pi);

    if (const SchemaInstance* motor = motorOf(world, rotor.prim)) {
        const auto volts = supplyVoltage(world, *motor);
        const auto kt = motor->real("elec:torqueConstant");
        const auto maxCurrent = motor->real("elec:maxCurrent");
        if (volts && kt && *kt > 0.0) {
            const bool dq = motor->token("elec:motorType").value_or("pmsm") != "dc_brushed";
            const double ke = dq ? *kt / 1.5 : *kt;  // peak dq convention (electrical.md)
            const double r = motor->real("elec:windingResistance").value_or(0.0);
            const double lq = motor->real("elec:inductanceQ")
                                  .value_or(motor->real("elec:inductance").value_or(0.0));
            const double poles = static_cast<double>(motor->integer("elec:polePairs").value_or(1));
            const double gear = motor->real("elec:gearRatio").value_or(1.0);
            const double efficiency = motor->real("elec:gearEfficiency").value_or(1.0);
            const double limit = dq ? *volts / std::sqrt(3.0) : *volts;
            // The voltage the drive needs to hold the rotor at ω (joint side), in steady state.
            const auto needed = [&](double omega) {
                const double motorSpeed = gear * omega;
                const double current = cq * omega * omega / (gear * efficiency * *kt);
                const double vq = r * current + ke * motorSpeed;
                const double vd = dq ? poles * motorSpeed * lq * current : 0.0;
                return std::hypot(vq, vd);
            };
            double lo = 0.0;
            double hi = limit / (ke * gear);  // no load: the back-EMF alone takes it all
            for (int k = 0; k < kBisections; ++k) {
                const double mid = 0.5 * (lo + hi);
                (needed(mid) > limit ? hi : lo) = mid;
            }
            double top = lo;
            std::string why = "voltage";
            if (cq > 0.0 && maxCurrent && *maxCurrent > 0.0) {
                const double currentLimited = std::sqrt(*maxCurrent * gear * efficiency * *kt / cq);
                if (currentLimited < top) {
                    top = currentLimited;
                    why = "current";
                }
            }
            return {top, std::string(world.signalInstanceName(motor->prim)) +
                             format(" on %.1f V full charge, ", *volts) + why + "-limited"};
        }
    }

    const double thrustCoeff = rotor.real("aero:rotor:thrustCoeff").value_or(0.0);
    const double mass = robotMass(world, rotor.prim);
    const std::size_t rotors = world.instancesOf("RotorAPI_1").size();
    const auto g = world.gravityLocal();
    const double weight = mass * std::hypot(g[0], g[1], g[2]);
    if (thrustCoeff > 0.0 && airDensity > 0.0 && diameter > 0.0 && weight > 0.0 && rotors > 0) {
        // Each rotor's share of twice the weight: T = CT·ρ·n²·D⁴, ω = 2π·n.
        const double thrust = kThrustToWeight * weight / static_cast<double>(rotors);
        const double n = std::sqrt(thrust / (thrustCoeff * airDensity * std::pow(diameter, 4)));
        return {2.0 * pi * n, format("no motor: thrust for %.0f times the weight of %.2f kg",
                                     kThrustToWeight, mass)};
    }
    return {kLegacyTopSpeed, "no motor and no thrust coefficient: the default"};
}

}  // namespace lagrange::mavlink
