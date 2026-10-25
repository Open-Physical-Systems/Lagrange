// Propeller files and blade-element momentum theory (Propeller.hpp, aerodynamics.md §2.1).
#include "Stations.hpp"
#include "lagrange/rotor_aero/Propeller.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <thread>

#include <yaml-cpp/yaml.h>

namespace lagrange::rotor_aero {

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
// Angle of attack (rad) over which a stalled section goes from its attached values to a flat
// plate's. A modelling choice: real sections stall over a few degrees, depending on the section
// and the Reynolds number.
constexpr double kStallBlend = 8.0 * kDeg;
// Flat plate normal-force coefficient 2·sin α (Hoerner, Fluid-Dynamic Lift, 1985, ch. 4: about
// 2 for an infinite plate). TODO(verify): the chapter's exact figure for two-dimensional plates.
constexpr double kPlateNormal = 2.0;
// Above this Mach number the Prandtl–Glauert factor is held (it diverges at 1; the model has no
// transonic drag rise).
constexpr double kMaxPrandtlGlauertMach = 0.7;
// Blade elements per blade for the integration (uniform in radius).
constexpr int kElements = 40;
// Swirl (tangential induction) passes; the swirl is small, so a few fixed-point passes converge.
constexpr int kSwirlPasses = 4;
// Root finding on the bracketed residual (Illinois variant of regula falsi): iterations and the
// relative tolerance on the induced velocity.
constexpr int kRootIterations = 80;
constexpr double kRootTolerance = 1e-10;
// The table of a propeller (PropellerTable): inflow angles over the full circle, and Mach steps.
constexpr int kTableAngles = 1440;
constexpr double kTableMachStep = 0.1;
constexpr int kTableMachs = 10;  // 0 to 0.9
// A folding blade's stop either way when the file gives none: folded nearly flat along the
// motor. A modelling choice; real hinges vary.
constexpr double kDefaultFoldLimitDeg = 85.0;

double wrapAngle(double a) {
    a = std::fmod(a + std::numbers::pi, 2.0 * std::numbers::pi);
    return a <= 0.0 ? a + std::numbers::pi : a - std::numbers::pi;
}

double attachedDrag(const Aerofoil& a, double cl) {
    const double d = cl - a.clAtCdMin;
    return a.cdMin + a.cdLift * d * d;
}

Aerofoil lerp(const Aerofoil& a, const Aerofoil& b, double t) {
    const auto mix = [t](double x, double y) { return x + t * (y - x); };
    return {mix(a.liftSlope, b.liftSlope),
            mix(a.zeroLiftAngle, b.zeroLiftAngle),
            mix(a.clMax, b.clMax),
            mix(a.clMin, b.clMin),
            mix(a.cdMin, b.cdMin),
            mix(a.clAtCdMin, b.clAtCdMin),
            mix(a.cdLift, b.cdLift),
            mix(a.camber, b.camber),
            mix(a.camberPosition, b.camberPosition)};
}

}  // namespace

BladeStation stationAt(const std::vector<BladeStation>& s, double x) {
    if (x <= s.front().radius) {
        return s.front();
    }
    for (std::size_t i = 1; i < s.size(); ++i) {
        if (x <= s[i].radius) {
            const double t = (x - s[i - 1].radius) / (s[i].radius - s[i - 1].radius);
            const auto mix = [t](double a, double b) { return a + t * (b - a); };
            return {x, mix(s[i - 1].chord, s[i].chord), mix(s[i - 1].twist, s[i].twist),
                    mix(s[i - 1].thickness, s[i].thickness),
                    lerp(s[i - 1].aerofoil, s[i].aerofoil, t)};
        }
    }
    BladeStation last = s.back();
    last.radius = x;
    return last;
}

namespace {

/// Blade-element forces per unit span, all blades, at axial and swirl induced velocities.
struct Element {
    double radius{0.0};  // m
    double chord{0.0};   // m
    double twist{0.0};   // rad
    Aerofoil aerofoil;
};

struct ElementLoad {
    double thrust{0.0};  // N/m along the thrust direction
    double torque{0.0};  // N·m/m, the air's torque against rotation in the hand
    double sinPhi{1.0};  // |axial flow| / resultant, for the tip loss
};

ElementLoad elementLoad(const Element& e, int blades, double omega, double axialSpeed,
                        double inducedAxial, double inducedSwirl, const Air& air) {
    // The air as the blade meets it, in (rotation direction, thrust direction) coordinates: from
    // the front at axialSpeed + w, from ahead at ω·r − w_t.
    const double ua = axialSpeed + inducedAxial;
    const double ut = omega * e.radius - inducedSwirl;
    const double w2 = ua * ua + ut * ut;
    if (w2 <= 0.0) {
        return {};
    }
    const double w = std::sqrt(w2);
    const double de = ut / w;
    const double dt = ua / w;
    const double sb = std::sin(e.twist);
    const double cb = std::cos(e.twist);
    const double alpha = std::atan2(de * sb - dt * cb, de * cb + dt * sb);
    const double mach = air.speedOfSound > 0.0 ? w / air.speedOfSound : 0.0;
    const SectionCoefficients c = sectionCoefficients(e.aerofoil, alpha, mach);
    const double q = 0.5 * air.density * w2 * e.chord * blades;
    // Lift at right angles to the oncoming air, drag along it.
    const double forceThrust = q * (c.cl * de - c.cd * dt);
    const double forceRotation = q * (-c.cl * dt - c.cd * de);
    return {forceThrust, -forceRotation * e.radius, std::abs(dt)};
}

/// Prandtl's tip-loss factor at radius r of a rotor of tip radius R with `blades` blades.
double tipLoss(int blades, double r, double tip, double sinPhi) {
    const double f = 0.5 * blades * (tip - r) / (r * std::max(sinPhi, 1e-6));
    return 2.0 / std::numbers::pi * std::acos(std::exp(-f));
}

}  // namespace

SectionCoefficients sectionCoefficients(const Aerofoil& a, double alpha, double mach) {
    alpha = wrapAngle(alpha);
    if (a.liftSlope <= 0.0) {
        return {0.0, a.cdMin};  // a section that only drags (a round shank)
    }
    const double m = std::min(std::abs(mach), kMaxPrandtlGlauertMach);
    const double slope = a.liftSlope / std::sqrt(1.0 - m * m);
    const double stallUp = a.zeroLiftAngle + a.clMax / slope;
    const double stallDown = a.zeroLiftAngle + a.clMin / slope;
    const double plateNormal = kPlateNormal * std::sin(alpha);
    const double plateLift = plateNormal * std::cos(alpha);
    const double plateDrag = plateNormal * std::sin(alpha) + a.cdMin;
    if (alpha >= stallDown && alpha <= stallUp) {
        const double cl = slope * (alpha - a.zeroLiftAngle);
        return {cl, attachedDrag(a, cl)};
    }
    const bool up = alpha > stallUp;
    const double edgeLift = up ? a.clMax : a.clMin;
    const double t = std::min(1.0, (up ? alpha - stallUp : stallDown - alpha) / kStallBlend);
    return {(1.0 - t) * edgeLift + t * plateLift,
            (1.0 - t) * attachedDrag(a, edgeLift) + t * std::max(plateDrag, a.cdMin)};
}

Propeller loadPropeller(const std::string& path, std::string& error) {
    Propeller p;
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (root["format"].as<std::string>("") != "lagrange-propeller-1") {
            error = path + ": not a propeller file (format: lagrange-propeller-1)";
            return {};
        }
        p.name = root["name"].as<std::string>("");
        p.radius = 0.5 * root["diameter"].as<double>(0.0);
        p.blades = root["blades"].as<int>(0);
        p.hand = root["hand"].as<std::string>("");
        std::map<std::string, Aerofoil> aerofoils;
        for (const auto& entry : root["aerofoils"]) {
            const YAML::Node n = entry.second;
            Aerofoil a;
            a.liftSlope = n["lift_slope"].as<double>();
            a.zeroLiftAngle = n["zero_lift_angle"].as<double>() * kDeg;
            a.clMax = n["cl_max"].as<double>();
            a.clMin = n["cl_min"].as<double>();
            a.cdMin = n["cd_min"].as<double>();
            a.clAtCdMin = n["cl_at_cd_min"].as<double>(0.0);
            a.cdLift = n["cd_lift"].as<double>(0.0);
            a.camber = n["camber"].as<double>(0.0);
            a.camberPosition = n["camber_position"].as<double>(0.4);
            aerofoils[entry.first.as<std::string>()] = a;
        }
        for (const YAML::Node& n : root["stations"]) {
            BladeStation s;
            s.radius = n["r"].as<double>();
            s.chord = n["chord"].as<double>();
            s.twist = n["twist"].as<double>() * kDeg;
            s.thickness = n["thickness"].as<double>(0.12);
            const auto found = aerofoils.find(n["aerofoil"].as<std::string>(""));
            if (found == aerofoils.end()) {
                error = path + ": station at r = " + std::to_string(s.radius) +
                        " names no aerofoil of the file";
                return {};
            }
            s.aerofoil = found->second;
            // Per-station corrections of the section (e.g. for its thickness).
            if (n["zero_lift_angle"]) {
                s.aerofoil.zeroLiftAngle = n["zero_lift_angle"].as<double>() * kDeg;
            }
            if (n["cd_min"]) {
                s.aerofoil.cdMin = n["cd_min"].as<double>();
            }
            p.stations.push_back(s);
        }
        if (const YAML::Node f = root["folding"]) {
            Folding folding;
            folding.hinge = f["hinge"].as<double>();
            folding.bladeMass = f["blade_mass"].as<double>();
            folding.limit = f["limit"].as<double>(kDefaultFoldLimitDeg) * kDeg;
            p.folding = folding;
        }
    } catch (const YAML::Exception& e) {
        error = path + ": " + e.what();
        return {};
    }
    std::ranges::sort(p.stations, {}, &BladeStation::radius);
    const bool ordered = !p.stations.empty() && p.stations.front().radius > 0.0 &&
                         p.stations.back().radius <= 1.0 &&
                         std::ranges::adjacent_find(p.stations, [](const auto& a, const auto& b) {
                             return a.radius == b.radius;
                         }) == p.stations.end();
    const bool foldingOk = !p.folding || (p.folding->hinge > 0.0 && p.folding->hinge < 0.5 &&
                                          p.folding->bladeMass > 0.0 && p.folding->limit > 0.0 &&
                                          p.folding->limit <= 0.5 * std::numbers::pi);
    if (!foldingOk) {
        error = path +
                ": folding needs a hinge at r/R in (0, 0.5), blade_mass > 0 and a limit "
                "in (0, 90] degrees";
        return {};
    }
    if (p.radius <= 0.0 || p.blades < 1 || (p.hand != "ccw" && p.hand != "cw") || !ordered ||
        std::ranges::any_of(p.stations, [](const BladeStation& s) { return s.chord <= 0.0; })) {
        error = path +
                ": a propeller needs diameter > 0, blades >= 1, hand ccw or cw, and stations at "
                "distinct radius fractions in (0, 1] with chord > 0";
        return {};
    }
    return p;
}

Propeller withBladeAngle(Propeller p, double delta) {
    for (BladeStation& s : p.stations) {
        s.twist += delta;
    }
    return p;
}

BemLoad bemLoad(const Propeller& p, double omega, double axialSpeed, const Air& air) {
    BemLoad out;
    if (p.stations.empty() || p.radius <= 0.0 || air.density <= 0.0) {
        return out;
    }
    const double root = p.stations.front().radius * p.radius;
    const double dr = (p.radius - root) / kElements;
    double inducedThrust = 0.0;
    double thrustMoment = 0.0;
    for (int k = 0; k < kElements; ++k) {
        const double r = root + (k + 0.5) * dr;
        const BladeStation s = stationAt(p.stations, r / p.radius);
        const Element e{r, s.chord * p.radius, s.twist, s.aerofoil};
        // Momentum per unit span through the annulus, at axial induced velocity w (air pushed
        // back) and swirl w_t: dT = 4πrρF·|V + w|·w, dQ = 4πr²ρF·|V + w|·w_t.
        const auto momentum = [&](double w, double sinPhi) {
            return 4.0 * std::numbers::pi * r * air.density *
                   tipLoss(p.blades, r, p.radius, sinPhi) * std::abs(axialSpeed + w);
        };
        double swirl = 0.0;
        double w = 0.0;
        ElementLoad load;
        const double span = 2.0 * (std::abs(omega) * r + std::abs(axialSpeed)) + 1.0;
        for (int pass = 0; pass < kSwirlPasses; ++pass) {
            const auto residual = [&](double wi) {
                const ElementLoad l = elementLoad(e, p.blades, omega, axialSpeed, wi, swirl, air);
                return l.thrust - momentum(wi, l.sinPhi) * wi;
            };
            // The working state: the root on the side of w = 0 that the blade's thrust there
            // points to (air pushed back for thrust, drawn forward for negative thrust).
            const double g0 = residual(0.0);
            double lo = 0.0;
            double hi = g0 >= 0.0 ? span : -span;
            if (g0 == 0.0 || (residual(hi) > 0.0) == (g0 > 0.0)) {
                w = 0.0;  // no momentum solution: blade forces with no induced velocity
                out.vortexRing = out.vortexRing || g0 != 0.0;
            } else {
                // Illinois: regula falsi that halves the stale end's residual.
                double glo = g0;
                double ghi = residual(hi);
                int side = 0;
                for (int i = 0; i < kRootIterations; ++i) {
                    w = (lo * ghi - hi * glo) / (ghi - glo);
                    const double g = residual(w);
                    if ((g > 0.0) == (glo > 0.0)) {
                        lo = w;
                        glo = g;
                        if (side == -1) {
                            ghi *= 0.5;
                        }
                        side = -1;
                    } else {
                        hi = w;
                        ghi = g;
                        if (side == 1) {
                            glo *= 0.5;
                        }
                        side = 1;
                    }
                    if (std::abs(hi - lo) < kRootTolerance * span || g == 0.0) {
                        break;
                    }
                }
            }
            load = elementLoad(e, p.blades, omega, axialSpeed, w, swirl, air);
            const double flux = momentum(w, load.sinPhi) * r;
            const double next = flux > 1e-9 ? load.torque / flux : 0.0;
            swirl = std::clamp(next, -0.5 * std::abs(omega * r), 0.5 * std::abs(omega * r));
        }
        // Descending into the wake the rotor makes (−2·w < V < 0): the vortex-ring state.
        out.vortexRing = out.vortexRing || (axialSpeed < 0.0 && axialSpeed > -2.0 * w && w > 0.0);
        out.thrust += load.thrust * dr;
        out.torque += load.torque * dr;
        inducedThrust += load.thrust * dr * w;
        thrustMoment += load.thrust * dr * r;
    }
    out.inducedVelocity = out.thrust != 0.0 ? inducedThrust / out.thrust : 0.0;
    out.thrustRadius =
        out.thrust != 0.0 ? std::clamp(thrustMoment / out.thrust, root, p.radius) : 0.75 * p.radius;
    return out;
}

PropellerTable PropellerTable::build(const Propeller& p, bool compressible) {
    PropellerTable t;
    t.radius_ = p.radius;
    t.machs_ = compressible ? kTableMachs : 1;
    t.rows_.resize(static_cast<std::size_t>(t.machs_) * (kTableAngles + 1));
    // At unit resultant tip speed (Ω²R² + V² = 1 m²/s²) and density, in air whose speed of sound
    // makes the row's tip Mach number.
    // One thread per Mach row: the rows are independent, so the table is the same however the
    // threads run.
    std::vector<std::thread> threads;
    for (int m = 0; m < t.machs_; ++m) {
        threads.emplace_back([&t, &p, m] {
            const double mach = m * kTableMachStep;
            const Air air{1.0, mach > 0.0 ? 1.0 / mach : 0.0};
            for (int k = 0; k <= kTableAngles; ++k) {
                const double theta = -std::numbers::pi + 2.0 * std::numbers::pi * k / kTableAngles;
                const BemLoad l = bemLoad(p, std::cos(theta) / p.radius, std::sin(theta), air);
                t.rows_[static_cast<std::size_t>(m) * (kTableAngles + 1) + k] = {
                    l.thrust, l.torque, l.inducedVelocity, l.thrustRadius / p.radius, l.vortexRing};
            }
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }
    return t;
}

BemLoad PropellerTable::at(double omega, double axialSpeed, const Air& air) const {
    const double tip = omega * radius_;
    const double speed2 = tip * tip + axialSpeed * axialSpeed;
    if (rows_.empty() || speed2 <= 0.0) {
        return {};
    }
    const double speed = std::sqrt(speed2);
    // Position in the table: inflow angle, then Mach number (held at the table's last row).
    const double u =
        (std::atan2(axialSpeed, tip) + std::numbers::pi) / (2.0 * std::numbers::pi) * kTableAngles;
    const int k = std::clamp(static_cast<int>(u), 0, kTableAngles - 1);
    const double fu = u - k;
    double v = 0.0;
    if (machs_ > 1 && air.speedOfSound > 0.0) {
        v = std::min(speed / air.speedOfSound / kTableMachStep, machs_ - 1.0);
    }
    const int m = std::min(static_cast<int>(v), machs_ - 2 < 0 ? 0 : machs_ - 2);
    const double fv = machs_ > 1 ? v - m : 0.0;
    const auto row = [&](int mi, int ki) {
        return rows_[static_cast<std::size_t>(mi) * (kTableAngles + 1) + ki];
    };
    const auto mix = [&](auto field) {
        const double a = field(row(m, k)) + fu * (field(row(m, k + 1)) - field(row(m, k)));
        if (machs_ == 1) {
            return a;
        }
        const double b =
            field(row(m + 1, k)) + fu * (field(row(m + 1, k + 1)) - field(row(m + 1, k)));
        return a + fv * (b - a);
    };
    BemLoad out;
    out.thrust = air.density * speed2 * mix([](const Row& r) { return r.thrust; });
    out.torque = air.density * speed2 * mix([](const Row& r) { return r.torque; });
    out.inducedVelocity = speed * mix([](const Row& r) { return r.induced; });
    out.thrustRadius = radius_ * mix([](const Row& r) { return r.thrustRadius; });
    out.vortexRing = row(m, fu < 0.5 ? k : k + 1).vortexRing;
    return out;
}

FoldingBlade::FoldingBlade(const Propeller& p)
    : blades_(p.blades)
    , hinge_(p.folding ? p.folding->hinge * p.radius : 0.0)
    , mass_(p.folding ? p.folding->bladeMass : 0.0)
    , limit_(p.folding ? p.folding->limit : 0.0) {
    if (!p.folding || p.stations.empty()) {
        return;
    }
    // Mass along the blade outboard of the hinge as chord²·thickness (the section's area).
    constexpr int kSlices = 50;
    const double ds = (p.radius - hinge_) / kSlices;
    double total = 0.0, first = 0.0, second = 0.0;
    for (int k = 0; k < kSlices; ++k) {
        const double s = (k + 0.5) * ds;
        const BladeStation st = stationAt(p.stations, (hinge_ + s) / p.radius);
        const double w = st.chord * st.chord * st.thickness * ds;
        total += w;
        first += w * s;
        second += w * s * s;
    }
    s1_ = mass_ * first / total;
    s2_ = mass_ * second / total;
    centre_ = hinge_ + first / total;
}

double FoldingBlade::angle(double omega, double thrust, double thrustRadius,
                           double gravityAlong) const {
    if (mass_ <= 0.0) {
        return 0.0;
    }
    // Moment towards the front: one blade's thrust and its weight's component along the axis.
    const double push =
        thrust / blades_ * std::max(thrustRadius - hinge_, 0.0) + gravityAlong * s1_;
    const double spin = omega * omega;
    const auto balance = [&](double beta) {
        return push * std::cos(beta) -
               spin * std::sin(beta) * (hinge_ * s1_ + std::cos(beta) * s2_);
    };
    // Stopped (or nearly): folded to the stop the moment pulls it to.
    if (spin * (hinge_ * s1_ + s2_) < 1e-9 * std::abs(push) + 1e-15) {
        return push >= 0.0 ? limit_ : -limit_;
    }
    double lo = -limit_;
    double hi = limit_;
    if (balance(lo) <= 0.0) {
        return lo;
    }
    if (balance(hi) >= 0.0) {
        return hi;
    }
    for (int i = 0; i < kRootIterations; ++i) {
        const double mid = 0.5 * (lo + hi);
        (balance(mid) > 0.0 ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

PropellerCoefficients coefficientsAt(const Propeller& p, double advanceRatio, double tipSpeed,
                                     const Air& air) {
    const double d = 2.0 * p.radius;
    const double n = tipSpeed / (std::numbers::pi * d);  // rev/s
    const BemLoad l = bemLoad(p, 2.0 * std::numbers::pi * n, advanceRatio * n * d, air);
    const double d4 = d * d * d * d;
    return {l.thrust / (air.density * n * n * d4),
            l.torque * 2.0 * std::numbers::pi * n / (air.density * n * n * n * d4 * d)};
}

}  // namespace lagrange::rotor_aero
