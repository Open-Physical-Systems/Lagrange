#include "lagrange/actuators/Solenoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace lagrange::actuators {

namespace {

/// The index of the table's segment for x: [k, k + 1], held to the end segments.
std::size_t segment(const SolenoidParams& p, double x) {
    const std::size_t n = p.position.size();
    const auto above = std::upper_bound(p.position.begin(), p.position.end(), x);
    const auto k =
        static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, (above - p.position.begin()) - 1));
    return std::min(k, n - 2);
}

}  // namespace

std::string checkSolenoid(const SolenoidParams& p) {
    if (p.position.size() < 2 || p.position.size() != p.inductance.size()) {
        return "act:solenoid:inductanceStrokes and inductances need at least two points, as many "
               "of each";
    }
    for (std::size_t k = 0; k < p.position.size(); ++k) {
        if (!(p.inductance[k] > 0.0)) {
            return "act:solenoid:inductances must be over 0";
        }
        if (k > 0 && !(p.position[k] > p.position[k - 1])) {
            return "act:solenoid:inductanceStrokes must rise";
        }
    }
    if (!(p.resistance > 0.0)) {
        return "act:solenoid:resistance must be over 0";
    }
    if (!(p.stroke > 0.0)) {
        return "act:solenoid:stroke must be over 0";
    }
    if (!(p.ratedVoltage > 0.0)) {
        return "act:solenoid:ratedVoltage must be over 0";
    }
    if (p.position.front() > 0.0 || p.position.back() < p.stroke) {
        return "act:solenoid:inductanceStrokes must cover the stroke, 0 to act:solenoid:stroke";
    }
    if (!(p.springRate >= 0.0) || !(p.springPreload >= 0.0)) {
        return "act:solenoid:springRate and springPreload must not be negative";
    }
    return {};
}

double solenoidInductance(const SolenoidParams& p, double x) {
    if (x <= p.position.front()) {
        return p.inductance.front();
    }
    if (x >= p.position.back()) {
        return p.inductance.back();
    }
    const std::size_t k = segment(p, x);
    const double f = (x - p.position[k]) / (p.position[k + 1] - p.position[k]);
    return p.inductance[k] + f * (p.inductance[k + 1] - p.inductance[k]);
}

double solenoidInductanceSlope(const SolenoidParams& p, double x) {
    const std::size_t k = segment(p, x);
    return (p.inductance[k + 1] - p.inductance[k]) / (p.position[k + 1] - p.position[k]);
}

double solenoidPull(const SolenoidParams& p, double current, double x) {
    return 0.5 * current * current * solenoidInductanceSlope(p, x);
}

double solenoidSpring(const SolenoidParams& p, double x) {
    return -(p.springPreload + p.springRate * x);
}

double stepSolenoidFlux(const SolenoidParams& p, double flux, double voltage, double x, double dt) {
    const double inductance = solenoidInductance(p, x);
    const double current = flux / inductance;
    const double steady = voltage / p.resistance;
    const double tau = inductance / p.resistance;
    return inductance * (steady + (current - steady) * std::exp(-dt / tau));
}

}  // namespace lagrange::actuators
