#include "lagrange/actuators/Brake.hpp"

#include <cmath>

namespace lagrange::actuators {

void stepBrake(const BrakeParams& p, BrakeState& s, double voltage, double dt) {
    const double steady = voltage / p.coilResistance;
    if (p.coilInductance > 0.0) {
        const double tau = p.coilInductance / p.coilResistance;
        s.current = steady + (s.current - steady) * std::exp(-dt / tau);
    } else {
        s.current = steady;
    }
    if (s.engaged && s.current >= p.releaseCurrent) {
        s.engaged = false;
    } else if (!s.engaged && s.current <= p.dropoutCurrent) {
        s.engaged = true;
    }
}

double brakeFriction(const BrakeParams& p, const BrakeState& s, double speed) {
    if (!s.engaged) {
        return 0.0;
    }
    return std::abs(speed) < kBrakeRestSpeed ? p.holding : p.dynamic;
}

}  // namespace lagrange::actuators
