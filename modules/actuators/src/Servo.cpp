#include "lagrange/actuators/Servo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::actuators {

double pulseToAngle(const ServoParams& p, double pulse) {
    const double span = p.pulseMax - p.pulseMin;
    if (!(std::abs(span) > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return p.angleMin + (pulse - p.pulseMin) / span * (p.angleMax - p.angleMin);
}

double quantise(double x, double step) {
    if (!(step > 0.0)) {
        return x;
    }
    return std::round(x / step) * step;
}

double deadband(double error, double band) {
    if (std::abs(error) <= band) {
        return 0.0;
    }
    return error > 0.0 ? error - band : error + band;
}

double positionDuty(const ServoParams& p, LoopState& s, double target, double targetRate,
                    double position, double velocity, double dt, double uMin, double uMax) {
    const double error = deadband(target - quantise(position, p.resolution), p.deadband);
    if (error == 0.0) {
        return 0.0;  // the comparator is off: no drive, the integrator held
    }
    const double wanted = p.kp * error + p.ki * s.integral + p.kd * (targetRate - velocity);
    const double u = std::clamp(wanted, uMin, uMax);
    if (u == wanted) {
        s.integral += error * dt;
    }
    return u;
}

double velocityDuty(const ServoParams& p, LoopState& s, double goal, double velocity, double dt,
                    double uMin, double uMax) {
    const double error = goal - velocity;
    const double wanted = p.velocityKp * error + p.velocityKi * s.integral;
    const double u = std::clamp(wanted, uMin, uMax);
    if (u == wanted) {
        s.integral += error * dt;
    }
    return u;
}

double idealTorque(const ServoParams& p, double u, double velocity) {
    if (!(p.stallTorque > 0.0) || !(p.noLoadSpeed > 0.0)) {
        return 0.0;
    }
    // Not held to ±τ_stall: back-driven against its duty, a DC motor brakes harder than it can
    // stall (its back-EMF adds to the applied voltage), as the electrical fidelity does too.
    return p.stallTorque * (u - velocity / p.noLoadSpeed);
}

void currentLimitedDuty(const Armature& a, double busVoltage, double velocity, double& uMin,
                        double& uMax) {
    uMin = -1.0;
    uMax = 1.0;
    if (!(a.maxCurrent > 0.0) || !(busVoltage > 0.0)) {
        return;
    }
    const double emf = a.backEmf * velocity;
    const double span = a.maxCurrent * a.resistance;
    uMin = std::clamp((emf - span) / busVoltage, -1.0, 1.0);
    uMax = std::clamp((emf + span) / busVoltage, -1.0, 1.0);
}

double currentDuty(const Armature& a, double busVoltage, double current, double velocity) {
    if (!(busVoltage > 0.0)) {
        return 0.0;
    }
    return std::clamp((a.resistance * current + a.backEmf * velocity) / busVoltage, -1.0, 1.0);
}

double stallTorqueOf(const Armature& a, double busVoltage) {
    return a.resistance > 0.0 ? a.torquePerAmp * busVoltage / a.resistance : 0.0;
}

double noLoadSpeedOf(const Armature& a, double busVoltage) {
    return a.backEmf > 0.0 ? busVoltage / a.backEmf : 0.0;
}

void stepProfile(Profile& s, double goal, double maxVelocity, double maxAcceleration, double dt) {
    const bool limitV = maxVelocity > 0.0;
    const bool limitA = maxAcceleration > 0.0;
    if ((!limitV && !limitA) || !(dt > 0.0)) {
        s.position = goal;
        s.velocity = 0.0;
        return;
    }
    const double error = goal - s.position;
    const double direction = error > 0.0 ? 1.0 : error < 0.0 ? -1.0 : 0.0;
    // The speed that still stops on the goal, at most the limit: the braking ramp v² = 2·a·|e| in
    // its discrete form, v = n·a·dt with n(n + 1)/2·a·dt² = |e| (the steps of a ramp down sum to
    // the distance left), so the move ends on time rather than early.
    double wanted = limitV ? maxVelocity : std::numeric_limits<double>::infinity();
    if (limitA) {
        const double ramp = maxAcceleration * dt;
        const double steps = 0.5 * (std::sqrt(1.0 + 8.0 * std::abs(error) / (ramp * dt)) - 1.0);
        wanted = std::min(wanted, steps * ramp);
    }
    wanted = std::min(wanted, std::abs(error) / dt);  // never past the goal in one step
    wanted *= direction;
    if (limitA) {
        const double change = maxAcceleration * dt;
        s.velocity = std::clamp(wanted, s.velocity - change, s.velocity + change);
    } else {
        s.velocity = wanted;
    }
    const double before = goal - s.position;
    s.position += s.velocity * dt;
    if ((goal - s.position) * before <= 0.0) {  // on or past the goal: stop on it
        s.position = goal;
        s.velocity = 0.0;
    }
}

void stepShutdown(const ServoParams& p, Shutdown& s, bool saturated, double temperature,
                  double dt) {
    s.saturatedFor = saturated ? s.saturatedFor + dt : 0.0;
    if (p.overloadTime > 0.0 && s.saturatedFor >= p.overloadTime) {
        s.overload = true;
    }
    if (p.maxTemperature > 0.0 && temperature >= p.maxTemperature) {
        s.overheated = true;
    }
}

}  // namespace lagrange::actuators
