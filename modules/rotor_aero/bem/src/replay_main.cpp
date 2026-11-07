// lagrange_rotor_replay: Lagrange's rotor models against measured flight (rotor_aero.md §9).
//
// Reads flight logs in the NeuroBEM format (Bauersfeld et al., RSS 2021; 400 Hz: body-frame
// velocity, angular rate and acceleration, motor speeds), computes the force and torque that
// Lagrange's rotors make from the measured motor speeds and motion (the `coefficients` fidelity
// with --ct/--cp, or `bem` with --propeller), and compares them with what the measured motion
// says acted on the vehicle: m·a and J·dω/dt + ω × Jω in the body frame. Prints the RMS errors
// as NeuroBEM's Table II does (Fxy, Fz, Mxy, Mz, F, M). With --fit it first identifies the
// model's free parameters on the files after --train (CT and CP by linear least squares; the
// propeller's blade angle by a one-dimensional search on Fz), then evaluates on the others.
#include "lagrange/rotor_aero/Propeller.hpp"
#include "lagrange/rotor_aero/RotorAero.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace lagrange::rotor_aero;
using Vec3 = std::array<double, 3>;

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

struct Sample {
    Vec3 angularAcceleration, angularVelocity, acceleration, velocity;
    std::array<double, 4> motor;
};

/// One NeuroBEM processed CSV: t, ang acc (3), ang vel (3), quaternion (4), acc (3), vel (3),
/// pos (3), motor speeds (4, rad/s), motor accelerations (4), battery voltage; body frame
/// front-left-up.
std::vector<Sample> readLog(const std::string& path, std::size_t stride) {
    std::ifstream in(path);
    std::vector<Sample> out;
    std::string line;
    std::size_t row = 0;
    while (std::getline(in, line)) {
        if (line.empty() || ((line[0] < '0' || line[0] > '9') && line[0] != '-')) {
            continue;  // header
        }
        if (row++ % stride != 0) {
            continue;
        }
        std::istringstream fields(line);
        std::vector<double> v;
        for (std::string cell; std::getline(fields, cell, ',');) {
            v.push_back(std::strtod(cell.c_str(), nullptr));
        }
        if (v.size() < 24) {
            continue;
        }
        out.push_back({{v[1], v[2], v[3]},
                       {v[4], v[5], v[6]},
                       {v[11], v[12], v[13]},
                       {v[14], v[15], v[16]},
                       {v[20], v[21], v[22], v[23]}});
    }
    return out;
}

struct Vehicle {
    double mass{0.772};                    // kg (dataset readme)
    Vec3 inertia{0.0025, 0.0021, 0.0043};  // kg·m² (dataset readme)
    double arm{0.13};                      // m, centre to motor (their setupKingfisher.m)
    // Per motor, +1 turning ccw about +z (body up): fitted on the logs (the yaw reaction of a
    // quadratic torque model is positive with these and negative the other way round).
    std::array<double, 4> spin{1, 1, -1, -1};
    double density{1.204};  // kg/m³ (their bem_parameters.yaml)
    // Motor order of the logs: back right, front right, back left, front left.
    [[nodiscard]] std::array<Vec3, 4> rotors() const {
        const double l = arm / std::sqrt(2.0);
        return {{{-l, -l, 0.0}, {l, -l, 0.0}, {-l, l, 0.0}, {l, l, 0.0}}};
    }
};

struct Model {
    // coefficients
    RotorParams coefficients;
    bool bem{false};
    Propeller propeller;  // ccw; the cw ones mirror it
    PropellerTable ccw, cw;
    void build() {
        if (!bem) {
            return;
        }
        Propeller left = propeller;
        left.hand = "ccw";
        Propeller right = propeller;
        right.hand = "cw";
        ccw = PropellerTable::build(left, false);
        cw = PropellerTable::build(right, false);
    }
};

struct Wrench {
    Vec3 force{}, torque{};
};

Wrench predict(const Model& model, const Vehicle& vehicle, const Sample& s) {
    Wrench w;
    const auto rotors = vehicle.rotors();
    const Air air{vehicle.density, 0.0};
    for (std::size_t i = 0; i < 4; ++i) {
        const double spin = vehicle.spin.at(i);
        const double omega = std::abs(s.motor.at(i));
        Vec3 f{};
        Vec3 hub{};
        double q = 0.0;
        if (model.bem) {
            const Vec3 r = rotors.at(i);
            const Vec3 turn = cross(s.angularVelocity, r);
            const Vec3 v{s.velocity[0] + turn[0], s.velocity[1] + turn[1], s.velocity[2] + turn[2]};
            const double edgewise = std::hypot(v[0], v[1]);
            const BemLoad load = (spin > 0 ? model.ccw : model.cw).at(omega, v[2], air, edgewise);
            f = {0.0, 0.0, load.thrust};
            q = load.torque;
            if (edgewise > 1e-9) {
                // e1: the way the edgewise air moves past the propeller; e2 = z × e1 in its hand.
                const Vec3 e1{-v[0] / edgewise, -v[1] / edgewise, 0.0};
                const Vec3 e2 = cross({0.0, 0.0, 1.0}, e1);
                for (std::size_t k = 0; k < 3; ++k) {
                    f.at(k) += load.inPlane[0] * e1.at(k) + spin * load.inPlane[1] * e2.at(k);
                    hub.at(k) = load.hubMoment[0] * e1.at(k) + spin * load.hubMoment[1] * e2.at(k);
                }
            }
        } else {
            RotorParams p = model.coefficients;
            p.spinSign = 1.0;
            const Load load = staticLoad(p, omega, vehicle.density);
            f = {0.0, 0.0, load.thrust};
            q = load.torque;
        }
        const Vec3 arm = cross(rotors.at(i), f);
        for (std::size_t k = 0; k < 3; ++k) {
            w.force.at(k) += f.at(k);
            w.torque.at(k) += arm.at(k) + hub.at(k);
        }
        // The air holds the propeller back; the motor's reaction turns the body the other way.
        w.torque[2] -= spin * q;
    }
    return w;
}

Wrench measured(const Vehicle& vehicle, const Sample& s) {
    Wrench w;
    const Vec3& j = vehicle.inertia;
    const Vec3 jw{j[0] * s.angularVelocity[0], j[1] * s.angularVelocity[1],
                  j[2] * s.angularVelocity[2]};
    const Vec3 gyro = cross(s.angularVelocity, jw);
    for (std::size_t k = 0; k < 3; ++k) {
        w.force.at(k) = vehicle.mass * s.acceleration.at(k);
        w.torque.at(k) = j.at(k) * s.angularAcceleration.at(k) + gyro.at(k);
    }
    return w;
}

struct Errors {
    double fx2{0}, fy2{0}, fz2{0}, mx2{0}, my2{0}, mz2{0};
    std::size_t n{0};
    void add(const Wrench& p, const Wrench& m) {
        const auto sq = [](double a) { return a * a; };
        fx2 += sq(p.force[0] - m.force[0]);
        fy2 += sq(p.force[1] - m.force[1]);
        fz2 += sq(p.force[2] - m.force[2]);
        mx2 += sq(p.torque[0] - m.torque[0]);
        my2 += sq(p.torque[1] - m.torque[1]);
        mz2 += sq(p.torque[2] - m.torque[2]);
        ++n;
    }
    void print(const char* label) const {
        const double k = static_cast<double>(std::max<std::size_t>(n, 1));
        std::printf("%-28s Fxy %.3f  Fz %.3f  Mxy %.4f  Mz %.4f  F %.3f  M %.4f  (%zu samples)\n",
                    label, std::sqrt((fx2 + fy2) / (2 * k)), std::sqrt(fz2 / k),
                    std::sqrt((mx2 + my2) / (2 * k)), std::sqrt(mz2 / k),
                    std::sqrt((fx2 + fy2 + fz2) / (3 * k)), std::sqrt((mx2 + my2 + mz2) / (3 * k)),
                    n);
    }
};

int usage() {
    std::fprintf(stderr,
                 "usage: lagrange_rotor_replay (--ct CT --cp CP --radius M | --propeller FILE) "
                 "[--pitch DEG] [--arm M] [--spin a,b,c,d] [--stride N] [--fit] "
                 "[--train LOG.csv... --test] LOG.csv...\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    Model model;
    Vehicle vehicle;
    std::vector<std::string> train, test;
    bool fit = false;
    bool inTrain = false;
    double pitch = 0.0;  // deg, added to every blade angle
    std::size_t stride = 1;
    for (int a = 1; a < argc; ++a) {
        const std::string arg = argv[a];
        const auto next = [&]() -> std::string {
            if (a + 1 >= argc) {
                std::exit(usage());
            }
            return argv[++a];
        };
        if (arg == "--ct") {
            model.coefficients.thrustCoeff = std::stod(next());
        } else if (arg == "--cp") {
            model.coefficients.powerCoeff = std::stod(next());
        } else if (arg == "--radius") {
            model.coefficients.radius = std::stod(next());
        } else if (arg == "--propeller") {
            std::string error;
            model.propeller = loadPropeller(next(), error);
            if (model.propeller.radius <= 0.0) {
                std::fprintf(stderr, "lagrange_rotor_replay: %s\n", error.c_str());
                return 1;
            }
            model.bem = true;
        } else if (arg == "--pitch") {
            pitch = std::stod(next());
        } else if (arg == "--arm") {
            vehicle.arm = std::stod(next());
        } else if (arg == "--spin") {
            std::istringstream s(next());
            std::string v;
            for (std::size_t i = 0; i < 4 && std::getline(s, v, ','); ++i) {
                vehicle.spin.at(i) = std::stod(v);
            }
        } else if (arg == "--stride") {
            stride = std::stoul(next());
        } else if (arg == "--fit") {
            fit = true;
        } else if (arg == "--train") {
            inTrain = true;
        } else if (arg == "--test") {
            inTrain = false;
        } else if (!arg.empty() && arg[0] == '-') {
            return usage();
        } else {
            (inTrain ? train : test).push_back(arg);
        }
    }
    if (test.empty() || (!model.bem && model.coefficients.radius <= 0.0)) {
        return usage();
    }
    const auto load = [&](const std::vector<std::string>& files) {
        std::vector<Sample> all;
        for (const std::string& f : files) {
            const std::vector<Sample> s = readLog(f, stride);
            all.insert(all.end(), s.begin(), s.end());
        }
        return all;
    };
    if (fit) {
        const std::vector<Sample> samples = load(train);
        if (samples.empty()) {
            std::fprintf(stderr, "lagrange_rotor_replay: --fit needs --train files\n");
            return 1;
        }
        if (!model.bem) {
            // Fz = CT·ρ·D⁴·Σn², Mz = −CP·ρ·D⁵/(2π)·Σ spin·n²: one unknown each, least squares.
            const double d = 2.0 * model.coefficients.radius;
            double tn = 0.0, tt = 0.0, qn = 0.0, qq = 0.0;
            for (const Sample& s : samples) {
                double n2 = 0.0, sn2 = 0.0;
                for (std::size_t i = 0; i < 4; ++i) {
                    const double n = s.motor.at(i) / (2.0 * std::numbers::pi);
                    n2 += n * n;
                    sn2 += vehicle.spin.at(i) * n * n;
                }
                const Wrench m = measured(vehicle, s);
                const double xt = vehicle.density * std::pow(d, 4) * n2;
                const double xq =
                    -vehicle.density * std::pow(d, 5) / (2.0 * std::numbers::pi) * sn2;
                tn += xt * m.force[2];
                tt += xt * xt;
                qn += xq * m.torque[2];
                qq += xq * xq;
            }
            model.coefficients.thrustCoeff = tn / tt;
            model.coefficients.powerCoeff = qn / qq;
            std::printf("fitted on %zu samples: CT %.5f, CP %.5f\n", samples.size(),
                        model.coefficients.thrustCoeff, model.coefficients.powerCoeff);
        } else {
            // The blade angle that minimises Fz's RMS on the training flights: golden section.
            const auto cost = [&](double deg) {
                Model trial = model;
                trial.propeller = withBladeAngle(model.propeller, deg * std::numbers::pi / 180.0);
                trial.build();
                double sum = 0.0;
                for (const Sample& s : samples) {
                    const double e =
                        predict(trial, vehicle, s).force[2] - measured(vehicle, s).force[2];
                    sum += e * e;
                }
                return sum;
            };
            double lo = -8.0, hi = 8.0;
            const double g = (std::sqrt(5.0) - 1.0) / 2.0;
            double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
            double f1 = cost(x1), f2 = cost(x2);
            while (hi - lo > 0.02) {
                if (f1 < f2) {
                    hi = x2;
                    x2 = x1;
                    f2 = f1;
                    x1 = hi - g * (hi - lo);
                    f1 = cost(x1);
                } else {
                    lo = x1;
                    x1 = x2;
                    f1 = f2;
                    x2 = lo + g * (hi - lo);
                    f2 = cost(x2);
                }
            }
            pitch = 0.5 * (lo + hi);
            std::printf("fitted on %zu samples: blade angle %+.2f deg\n", samples.size(), pitch);
        }
    }
    if (model.bem && pitch != 0.0) {
        model.propeller = withBladeAngle(model.propeller, pitch * std::numbers::pi / 180.0);
    }
    model.build();
    Errors all;
    for (const std::string& f : test) {
        Errors one;
        for (const Sample& s : readLog(f, stride)) {
            const Wrench p = predict(model, vehicle, s);
            const Wrench m = measured(vehicle, s);
            one.add(p, m);
            all.add(p, m);
        }
        const std::string name = f.substr(f.find_last_of('/') + 1);
        one.print(name.c_str());
    }
    all.print("all");
    return 0;
}
