// The lamp model of RFC 0021; rules in Lamp.hpp.
#include "lagrange/lights/Lamp.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::lights {

namespace {

constexpr double kPi = std::numbers::pi;

// Relative intensity at the beam edge (half the beam angle) and the field edge (half the field
// angle): the 50 % and 10 % of ANSI C78.379's definitions for reflector lamps (RFC 0021 §2;
// TODO(verify) the clause, the standard was not opened).
constexpr double kBeamEdge = 0.5;
constexpr double kFieldEdge = 0.1;

// Simpson intervals per piece of the taper: smooth pieces, so the error is far below 1e-6.
constexpr int kIntervals = 2000;

std::string ofLamp(const std::string& name, const std::string& text) {
    return "lamp " + name + ": " + text;
}

/// The bus a load's elec:powerPort leads to: a bus prim, or a port connected to one.
std::string busOf(const WorldView& world, const SchemaInstance& load) {
    const auto targets = load.rel("elec:powerPort");
    if (targets.empty()) {
        return {};
    }
    const PrimId target = targets[0];
    for (const SchemaInstance* bus : world.instancesOf("ElectricalBusAPI_1")) {
        if (bus->prim == target) {
            return std::string(world.signalInstanceName(target));
        }
    }
    for (const SchemaInstance* port : world.instancesOf("PortAPI_1")) {
        if (port->prim != target) {
            continue;
        }
        for (const PrimId next : port->rel("elec:port:" + port->instance + ":connectedTo")) {
            for (const SchemaInstance* bus : world.instancesOf("ElectricalBusAPI_1")) {
                if (bus->prim == next) {
                    return std::string(world.signalInstanceName(next));
                }
            }
        }
    }
    return {};
}

}  // namespace

std::vector<LampSpec> lampsOf(const WorldView& world, std::string& error) {
    std::vector<LampSpec> out;
    const World& raw = world.raw();
    for (const SchemaInstance* s : world.instancesOf("LampAPI_1")) {
        LampSpec lamp;
        lamp.prim = s->prim;
        lamp.name = std::string(world.signalInstanceName(s->prim));
        // A lamp on a link's own prim is at the link's origin.
        lamp.body = world.primBody(s->prim);
        lamp.poseInBody = world.primPoseInBody(s->prim);
        for (std::uint32_t b = 0; b < raw.bodies.prim.size(); ++b) {
            if (raw.bodies.prim[b] == s->prim) {
                lamp.body = BodyId{b};
                lamp.poseInBody = Pose{};
            }
        }
        const std::string kind(s->token("lamp:kind").value_or("spot"));
        if (kind == "point") {
            lamp.kind = LampKind::Point;
        } else if (kind == "beacon") {
            lamp.kind = LampKind::Beacon;
        } else if (kind != "spot") {
            error = ofLamp(lamp.name, "lamp:kind " + kind + " is not spot, point or beacon");
            return {};
        }
        lamp.intensity = s->real("lamp:luminousIntensity").value_or(0.0);
        lamp.beamAngle = s->real("lamp:beamAngle").value_or(0.0);
        lamp.fieldAngle = s->real("lamp:fieldAngle").value_or(0.0);
        lamp.colorTemperature = s->real("lamp:colorTemperature").value_or(0.0);
        if (const auto c = s->realArray("lamp:color"); c.size() == 3) {
            lamp.color = {c[0], c[1], c[2]};
        }
        lamp.efficacy = s->real("lamp:luminousEfficacy").value_or(0.0);
        lamp.heatFraction = s->real("lamp:heatFraction").value_or(1.0);
        lamp.riseTime = s->real("lamp:riseTime").value_or(0.0);
        lamp.blinkFrequency = s->real("lamp:blinkFrequency").value_or(0.0);
        lamp.blinkDuty = s->real("lamp:blinkDuty").value_or(0.5);
        lamp.rotationSpeed = s->real("lamp:rotationSpeed").value_or(0.0);
        lamp.voltageExponent = s->real("lamp:voltageExponent").value_or(0.0);
        lamp.initialCommand = s->real("lamp:initialCommand").value_or(0.0);
        if (!(lamp.intensity > 0.0)) {
            error = ofLamp(lamp.name, "lamp:luminousIntensity is required and over 0 (cd)");
            return {};
        }
        if (lamp.kind != LampKind::Point) {
            if (!(lamp.beamAngle > 0.0 && lamp.beamAngle < kPi)) {
                error = ofLamp(lamp.name, "lamp:beamAngle is required for a " + kind +
                                              " lamp, over 0 and under pi rad");
                return {};
            }
            if (lamp.fieldAngle == 0.0) {
                lamp.fieldAngle = std::min(2.0 * lamp.beamAngle, 2.0 * kPi);
            }
            if (!(lamp.fieldAngle > lamp.beamAngle && lamp.fieldAngle <= 2.0 * kPi)) {
                error = ofLamp(lamp.name,
                               "lamp:fieldAngle must be over lamp:beamAngle and at "
                               "most 2 pi rad");
                return {};
            }
        }
        for (const auto& [value, what] :
             {std::pair{lamp.colorTemperature, "lamp:colorTemperature"},
              std::pair{lamp.efficacy, "lamp:luminousEfficacy"},
              std::pair{lamp.riseTime, "lamp:riseTime"},
              std::pair{lamp.blinkFrequency, "lamp:blinkFrequency"},
              std::pair{lamp.voltageExponent, "lamp:voltageExponent"}}) {
            if (!(value >= 0.0) || !std::isfinite(value)) {
                error = ofLamp(lamp.name, std::string(what) + " must be 0 or more");
                return {};
            }
        }
        for (const SchemaInstance* load : world.instancesOf("ElectricalLoadAPI_1")) {
            if (load->prim != s->prim) {
                continue;
            }
            if (load->token("elec:load:model").value_or("constant_power") != "signal") {
                error = ofLamp(lamp.name,
                               "its ElectricalLoadAPI_1 must have elec:load:model = "
                               "signal, so that the lamp sets what it draws");
                return {};
            }
            if (!(lamp.efficacy > 0.0)) {
                error = ofLamp(lamp.name,
                               "a lamp that is a load needs lamp:luminousEfficacy "
                               "over 0 (lm/W)");
                return {};
            }
            lamp.load = true;
            lamp.bus = busOf(world, *load);
        }
        out.push_back(std::move(lamp));
    }
    return out;
}

double taper(const LampSpec& lamp, double theta) {
    if (lamp.kind == LampKind::Point) {
        return 1.0;
    }
    const double beam = 0.5 * lamp.beamAngle;
    const double field = 0.5 * lamp.fieldAngle;
    theta = std::abs(theta);
    if (theta <= beam) {
        const double c = std::cos(0.5 * kPi * theta / beam);
        return kBeamEdge + (1.0 - kBeamEdge) * c * c;
    }
    if (theta <= field) {
        const double c = std::cos(0.5 * kPi * (theta - beam) / (field - beam));
        return kFieldEdge + (kBeamEdge - kFieldEdge) * c * c;
    }
    return 0.0;
}

double effectiveSolidAngle(const LampSpec& lamp) {
    if (lamp.kind == LampKind::Point) {
        return 4.0 * kPi;
    }
    const double beam = 0.5 * lamp.beamAngle;
    const double field = std::min(0.5 * lamp.fieldAngle, kPi);
    const auto f = [&lamp](double theta) { return taper(lamp, theta); };
    // The two smooth pieces apart, so Simpson's rule never straddles a kink.
    const double inner = integrateOverCone(f, beam, kIntervals);
    const double outer =
        integrateOverCone(f, field, 2 * kIntervals) - integrateOverCone(f, beam, 2 * kIntervals);
    return inner + outer;
}

double coneSolidAngle(double alpha) {
    return 2.0 * kPi * (1.0 - std::cos(alpha));
}

bool blinkOn(double t, double frequency, double duty) {
    if (frequency <= 0.0) {
        return true;
    }
    const double cycles = t * frequency;
    // A tiny allowance, so an edge that falls on a step boundary is not lost to rounding.
    const double phase = cycles - std::floor(cycles + 1e-9);
    return std::max(phase, 0.0) < duty - 1e-9;
}

std::array<double, 3> beamAxis(const LampSpec& lamp, double t) {
    if (lamp.kind != LampKind::Beacon) {
        return {1.0, 0.0, 0.0};
    }
    const double a = lamp.rotationSpeed * t;
    return {std::cos(a), std::sin(a), 0.0};
}

double illuminance(const LampSpec& lamp, double intensity, const Pose& lampPose,
                   const std::array<double, 3>& point, const std::array<double, 3>& normal,
                   double t, double nearLimit) {
    const std::array<double, 3> to{point[0] - lampPose.position[0], point[1] - lampPose.position[1],
                                   point[2] - lampPose.position[2]};
    const double d = std::max(std::hypot(to[0], to[1], to[2]), nearLimit);
    const std::array<double, 3> u{to[0] / d, to[1] / d, to[2] / d};
    // The surface sees the lamp along -u.
    const double incidence = -(u[0] * normal[0] + u[1] * normal[1] + u[2] * normal[2]);
    if (incidence <= 0.0 || intensity <= 0.0) {
        return 0.0;
    }
    const std::array<double, 3> axis = rotate(lampPose.orientation, beamAxis(lamp, t));
    const double c = std::clamp(u[0] * axis[0] + u[1] * axis[1] + u[2] * axis[2], -1.0, 1.0);
    return intensity * taper(lamp, std::acos(c)) * incidence / (d * d);
}

}  // namespace lagrange::lights
