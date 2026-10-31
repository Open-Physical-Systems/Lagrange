// The averaged models of docs/design/domains/electrical.md §2.
#include "lagrange/electrical/Models.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace lagrange::electrical {

namespace {

// Power and torque factor of the peak-amplitude (amplitude-invariant) Park transform:
// P = 1.5·(vd·id + vq·iq), τ = 1.5·p·(λ·iq + (Ld − Lq)·id·iq). Source: any drives text, e.g.
// Krause, Analysis of Electric Machinery, the "arbitrary reference frame" chapter.
constexpr double kDqFactor = 1.5;

// The largest fundamental phase voltage (peak) a three-phase inverter applies without
// overmodulation is Vbus/√3 with space-vector (or third-harmonic) modulation.
const double kSqrt3 = std::sqrt(3.0);

// Largest current-loop bandwidth × step the drive model runs at (ωc·dt): stable and well damped.
constexpr double kMaxLoopBandwidthSteps = 1.0;

/// One axis of L·di/dt = u − R·i, with u held over the step: the exact solution.
double advanceAxis(double current, double u, double resistance, double inductance, double dt) {
    if (inductance <= 0.0) {
        return resistance > 0.0 ? u / resistance : current;
    }
    if (resistance <= 0.0) {
        return current + u * dt / inductance;
    }
    const double decay = std::exp(-resistance * dt / inductance);
    return decay * current + (1.0 - decay) * u / resistance;
}

/// x' = e^(A·h)·x + A⁻¹·(e^(A·h) − I)·b for a 2×2 A with a positive determinant: the exact step
/// of x' = A·x + b with b held. e^(A·h) from the Cayley–Hamilton form e^(s·h)·(c·I + k·(A − s·I)),
/// s = tr/2, with c, k from cosh/sinh or cos/sin of the eigenvalues' spread.
void exactStep2(const std::array<double, 4>& a, const std::array<double, 2>& b,
                std::array<double, 2>& x, double h) {
    const double s = 0.5 * (a[0] + a[3]);
    const double det = a[0] * a[3] - a[1] * a[2];
    const double q = s * s - det;
    double c = 1.0;
    double k = h;
    if (q > 0.0) {
        const double d = std::sqrt(q);
        c = std::cosh(d * h);
        k = std::sinh(d * h) / d;
    } else if (q < 0.0) {
        const double d = std::sqrt(-q);
        c = std::cos(d * h);
        k = std::sin(d * h) / d;
    }
    const double e = std::exp(s * h);
    const std::array<double, 4> m{e * (c + k * (a[0] - s)), e * k * a[1], e * k * a[2],
                                  e * (c + k * (a[3] - s))};
    // (e^(A·h) − I)·b, then A⁻¹ of it.
    const double r0 = (m[0] - 1.0) * b[0] + m[1] * b[1];
    const double r1 = m[2] * b[0] + (m[3] - 1.0) * b[1];
    const double y0 = (a[3] * r0 - a[1] * r1) / det;
    const double y1 = (-a[2] * r0 + a[0] * r1) / det;
    const double x0 = m[0] * x[0] + m[1] * x[1] + y0;
    const double x1 = m[2] * x[0] + m[3] * x[1] + y1;
    x = {x0, x1};
}

}  // namespace

double MotorParams::fluxLinkage() const {
    return dq ? torqueConstant / (kDqFactor * polePairs) : 0.0;
}

double MotorParams::backEmfConstant() const {
    return dq ? polePairs * fluxLinkage() : torqueConstant;
}

double MotorParams::resistanceAt(double temperature) const {
    return resistance * (1.0 + resistanceTempCoeff * (temperature - referenceTemperature));
}

void advanceCurrents(const MotorParams& p, MotorState& s, double vd, double vq,
                     double motorVelocity, double resistance, double dt) {
    if (!p.dq) {
        // Armature: L·di/dt = v − R·i − Kt·ω (Ke = Kt in SI units).
        s.id = 0.0;
        s.iq =
            advanceAxis(s.iq, vq - p.torqueConstant * motorVelocity, resistance, p.inductanceQ, dt);
        return;
    }
    const double we = p.polePairs * motorVelocity;
    const double ud = vd + we * p.inductanceQ * s.iq;
    const double uq = vq - we * (p.inductanceD * s.id + p.fluxLinkage());
    s.id = advanceAxis(s.id, ud, resistance, p.inductanceD, dt);
    s.iq = advanceAxis(s.iq, uq, resistance, p.inductanceQ, dt);
}

MotorOutputs stepMotor(const MotorParams& p, MotorState& s, const MotorInputs& in, double dt) {
    MotorOutputs out;
    if (!in.enabled || in.busVoltage <= 0.0) {
        s = MotorState{};
        return out;
    }
    const double motorVelocity = p.gearRatio * in.jointVelocity;
    const double resistance = p.resistanceAt(in.temperature);
    // Torque at the motor for the joint torque asked: the gear loses (1 − η) of the power in
    // whichever direction it flows.
    const bool motoring = in.torqueCommand * in.jointVelocity >= 0.0;
    const double gear = motoring ? p.gearEfficiency : 1.0 / p.gearEfficiency;
    const double motorTorque = in.torqueCommand / (p.gearRatio * gear);
    const double limit = p.maxCurrent * std::clamp(in.derating, 0.0, 1.0);
    const double iqRef = std::clamp(motorTorque / p.torqueConstant, -limit, limit);

    // PI current loops with the plant's zero cancelled: Kp = L·ωc, Ki = R·ωc, giving a
    // first-order closed loop of bandwidth ωc. Tuned on the nominal resistance, as a drive is.
    // A discrete loop cannot be faster than its step resolves: above ωc·dt ≈ 2 it oscillates and
    // diverges. The bandwidth is capped at 1/dt (the network warns when it caps it).
    const double wc = std::min(p.currentLoopBandwidth, kMaxLoopBandwidthSteps / dt);
    const double ki = p.resistance * wc;
    const double ed = -s.id;
    const double eq = iqRef - s.iq;
    double vd = 0.0;
    double vq = 0.0;
    double limitVoltage = in.busVoltage;
    double magnitude = 0.0;
    if (p.dq) {
        const double we = p.polePairs * motorVelocity;
        vd = p.inductanceD * wc * ed + s.integralD - we * p.inductanceQ * s.iq;
        vq = p.inductanceQ * wc * eq + s.integralQ + we * (p.inductanceD * s.id + p.fluxLinkage());
        limitVoltage = in.busVoltage / kSqrt3;
        magnitude = std::hypot(vd, vq);
    } else {
        vq = p.inductanceQ * wc * eq + s.integralQ + p.torqueConstant * motorVelocity;
        magnitude = std::abs(vq);
    }
    if (in.voltageMode) {
        vd = 0.0;
        vq = std::clamp(in.voltageCommand, -limitVoltage, limitVoltage);
        s.integralD = s.integralQ = 0.0;
    } else if (magnitude > limitVoltage) {
        // Saturated: scale the vector onto the limit and hold the integrators (anti-windup by
        // conditional integration).
        const double scale = limitVoltage / magnitude;
        vd *= scale;
        vq *= scale;
    } else {
        s.integralD += ki * ed * dt;
        s.integralQ += ki * eq * dt;
    }

    const double idStart = s.id;
    const double iqStart = s.iq;
    const double factor = p.dq ? kDqFactor : 1.0;
    if (in.inertia > 0.0 && p.inductanceQ > 0.0) {
        // Speed prediction: q-axis current and speed as one linear system, the back-EMF
        // implicit (architecture.md §8). Motor side, ω_m = N·ω:
        //   Lq·di/dt = u − R·i − Ke·ω_m,   J/N²·dω_m/dt = g·Kt·i − τ_load/N
        // with u the q voltage less the explicit d-axis cross term, g the gear's efficiency in
        // the direction power flows at the step's start. Reluctance torque is left out here.
        const double n = p.gearRatio;
        const double we = p.dq ? p.polePairs * motorVelocity : 0.0;
        const double efficiency = p.torqueConstant * s.iq * motorVelocity >= 0.0
                                      ? p.gearEfficiency
                                      : 1.0 / p.gearEfficiency;
        if (p.dq) {
            s.id = advanceAxis(s.id, vd + we * p.inductanceQ * s.iq, resistance, p.inductanceD, dt);
        }
        const double u = vq - (p.dq ? we * p.inductanceD * idStart : 0.0);
        const double ke = p.backEmfConstant();
        const std::array<double, 4> a{-resistance / p.inductanceQ, -ke / p.inductanceQ,
                                      n * n * efficiency * p.torqueConstant / in.inertia, 0.0};
        const std::array<double, 2> b{u / p.inductanceQ, -n * in.loadTorque / in.inertia};
        std::array<double, 2> x{s.iq, motorVelocity};
        exactStep2(a, b, x, dt);
        s.iq = x[0];
        out.jointVelocity = x[1] / n;
        out.vd = vd;
        out.vq = vq;
        out.motorTorque = p.torqueConstant * s.iq;
        // The step's average torque: what turned the inertia, plus the load it held.
        out.jointTorque = in.inertia * (out.jointVelocity - in.jointVelocity) / dt + in.loadTorque;
        const double id = 0.5 * (idStart + s.id);
        const double iq = 0.5 * (iqStart + s.iq);
        out.busCurrent = factor * (vd * id + vq * iq) / in.busVoltage;
        out.lossPower = factor * resistance * (id * id + iq * iq);
        return out;
    }
    advanceCurrents(p, s, vd, vq, motorVelocity, resistance, dt);
    // Step averages for the power and loss, so that energy balances to second order in dt.
    const double id = 0.5 * (idStart + s.id);
    const double iq = 0.5 * (iqStart + s.iq);
    out.vd = vd;
    out.vq = vq;
    out.motorTorque =
        p.dq ? kDqFactor * p.polePairs *
                   (p.fluxLinkage() * s.iq + (p.inductanceD - p.inductanceQ) * s.id * s.iq)
             : p.torqueConstant * s.iq;
    const double gearOut =
        out.motorTorque * motorVelocity >= 0.0 ? p.gearEfficiency : 1.0 / p.gearEfficiency;
    out.jointTorque = out.motorTorque * p.gearRatio * gearOut;
    const double power = factor * (vd * id + vq * iq);
    out.busCurrent = power / in.busVoltage;
    out.lossPower = factor * resistance * (id * id + iq * iq);
    return out;
}

double BatteryParams::openCircuitVoltage(double soc) const {
    if (ocvSoc.empty()) {
        return 0.0;
    }
    double cell = ocvVoltage.front();
    if (soc >= ocvSoc.back()) {
        cell = ocvVoltage.back();
    } else if (soc > ocvSoc.front()) {
        const auto upper = std::upper_bound(ocvSoc.begin(), ocvSoc.end(), soc);
        const auto k = static_cast<std::size_t>(upper - ocvSoc.begin());
        const double t = (soc - ocvSoc[k - 1]) / (ocvSoc[k] - ocvSoc[k - 1]);
        cell = ocvVoltage[k - 1] + t * (ocvVoltage[k] - ocvVoltage[k - 1]);
    }
    return seriesCells * cell;
}

Thevenin theveninOf(const BatteryParams& p, const BatteryState& s, double temperature) {
    const double r0 = p.r0 * (1.0 + p.r0TempCoeff * (temperature - p.referenceTemperature));
    // An RC pair without capacitance is a plain resistance; with it, its voltage is a state.
    const double cell = r0 + (p.c1 > 0.0 ? 0.0 : p.r1) + (p.c2 > 0.0 ? 0.0 : p.r2);
    return {p.openCircuitVoltage(s.soc) - p.seriesCells * (s.v1 + s.v2),
            p.seriesCells * cell / p.parallelCells};
}

void advanceBattery(const BatteryParams& p, BatteryState& s, double current, double dt) {
    const double cell = current / p.parallelCells;
    const auto pair = [&](double& v, double r, double c) {
        if (r > 0.0 && c > 0.0) {
            const double decay = std::exp(-dt / (r * c));
            v = decay * v + (1.0 - decay) * r * cell;
        }
    };
    pair(s.v1, p.r1, p.c1);
    pair(s.v2, p.r2, p.c2);
    s.soc = std::clamp(s.soc - current * dt / p.capacity, 0.0, 1.0);
    if (s.soc <= 0.0 || p.openCircuitVoltage(s.soc) <= p.cutoffVoltage) {
        s.cut = true;
    }
}

}  // namespace lagrange::electrical
