// The aerodynamics domain: link drag, the mean wind profile and Dryden turbulence
// (Aerodynamics.hpp; docs/design/domains/aerodynamics.md §2).
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::aerodynamics {

using namespace literals;

namespace {

using Vec3 = std::array<double, 3>;

constexpr double kFeetPerMetre = 1.0 / 0.3048;  // international foot (exact)
constexpr double kW20HeightM = 20.0 * 0.3048;   // 20 ft: where MIL-F-8785C's W20 is measured
constexpr double kMinHeightFt = 10.0;           // the low-altitude model's lower bound
constexpr double kMaxHeightFt = 1000.0;         // and its upper bound (medium altitude above)
constexpr double kMinAirspeed = 1.0;            // m/s: the filters need motion through the air
// Two-sided intensity of the white noise driving the forming filters: π, so that the outputs'
// variances are σ² (∫|H|²dω = 2σ² over all ω for each Dryden filter).
constexpr double kNoiseIntensity = std::numbers::pi;

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

/// One stage x' = (−x + n)/T over `h`, exact, for noise of intensity kNoiseIntensity.
double lag(double x, double h, double t, double normal) {
    const double decay = std::exp(-h / t);
    const double variance = kNoiseIntensity * (1.0 - decay * decay) / (2.0 * t);
    return decay * x + std::sqrt(variance) * normal;
}

/// Two identical stages a' = (−a + n)/T, b' = (−b + a)/T over `h`, exact: the transition and
/// the noise covariance of the pair in closed form, the noise drawn through its Cholesky factor.
void lagPair(double& a, double& b, double h, double t, double n1, double n2) {
    const double e = std::exp(-h / t);
    const double r = h / t;
    const double a1 = e * a;
    const double b1 = e * (b + r * a);
    // Q = π ∫0^h e^{−2s/T} [[1/T², s/T³], [s/T³, s²/T⁴]] ds.
    const double e2 = e * e;
    const double half = t / 2.0;
    const double i0 = half * (1.0 - e2);
    const double i1 = half * half * (1.0 - e2 * (1.0 + 2.0 * r));
    const double i2 = 2.0 * half * half * half -
                      e2 * (h * h * half + 2.0 * h * half * half + 2.0 * half * half * half);
    const double qaa = kNoiseIntensity * i0 / (t * t);
    const double qab = kNoiseIntensity * i1 / (t * t * t);
    const double qbb = kNoiseIntensity * i2 / (t * t * t * t);
    const double l11 = std::sqrt(qaa);
    const double l21 = l11 > 0.0 ? qab / l11 : 0.0;
    const double l22 = std::sqrt(std::max(0.0, qbb - l21 * l21));
    a = a1 + l11 * n1;
    b = b1 + l21 * n1 + l22 * n2;
}

}  // namespace

double meanWindAt(const Wind& wind, double height) {
    if (wind.speed == 0.0 || height <= wind.roughnessLength) {
        return 0.0;
    }
    return wind.speed * std::log(height / wind.roughnessLength) /
           std::log(wind.referenceHeight / wind.roughnessLength);
}

std::array<double, 3> Dryden::step(double dt, double height, double airspeed, double w20,
                                   const std::array<double, 5>& normal) {
    const double h = std::clamp(height * kFeetPerMetre, kMinHeightFt, kMaxHeightFt);
    const double v = std::max(airspeed, kMinAirspeed) * kFeetPerMetre;  // ft/s, as L
    const double denominator = 0.177 + 0.000823 * h;
    const double lw = h;
    const double luv = h / std::pow(denominator, 1.2);
    const double sw = 0.1 * w20;
    const double suv = sw / std::pow(denominator, 0.4);
    // Longitudinal: σu·√(2Lu/(πV))/(1+Tu·s), Tu = Lu/V.
    const double tu = luv / v;
    state_[0] = lag(state_[0], dt, tu, normal[0]);
    const double u = suv * std::sqrt(2.0 * luv / (std::numbers::pi * v)) * state_[0];
    // Lateral and vertical: σ·√(L/(πV))·(1+√3·T·s)/(1+T·s)² = K·(√3·a + (1−√3)·b).
    const auto second = [&](double& a, double& b, double l, double sigma, double n1, double n2) {
        const double t = l / v;
        lagPair(a, b, dt, t, n1, n2);
        const double k = sigma * std::sqrt(l / (std::numbers::pi * v));
        return k * (std::sqrt(3.0) * a + (1.0 - std::sqrt(3.0)) * b);
    };
    const double lateral = second(state_[1], state_[2], luv, suv, normal[1], normal[2]);
    const double vertical = second(state_[3], state_[4], lw, sw, normal[3], normal[4]);
    return {u, lateral, vertical};
}

Expected<Wind, std::string> AeroDrag::windOf(const ModuleConfig& config) {
    Wind wind;
    const std::optional<double> speed = config.get<double>("wind_speed");
    if (!speed || *speed == 0.0) {
        return wind;
    }
    const auto from = config.get<double>("wind_from");
    const auto reference = config.get<double>("wind_reference_height");
    const auto roughness = config.get<double>("roughness_length");
    if (*speed < 0.0 || !from || !reference || *reference <= 0.0 || !roughness ||
        *roughness <= 0.0 || *roughness >= *reference) {
        return Unexpected{std::string(
            "the wind needs wind_speed >= 0 (m/s), wind_from (degrees), wind_reference_height "
            "(m) and the terrain's roughness_length (m, below the reference height)")};
    }
    wind.speed = *speed;
    wind.fromDegrees = *from;
    wind.referenceHeight = *reference;
    wind.roughnessLength = *roughness;
    const std::string turbulence = config.get<std::string>("turbulence").value_or("none");
    if (turbulence != "none" && turbulence != "dryden") {
        return Unexpected{std::string("turbulence is none or dryden")};
    }
    wind.dryden = turbulence == "dryden";
    return wind;
}

ModuleInfo AeroDrag::info() const {
    return {"aerodynamics", "drag", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration AeroDrag::preferredStep() const {
    return 1_ms;
}

Status AeroDrag::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    links_.clear();
    const auto rho = config.get<double>("air_density");
    if (!world.instancesOf("AeroAPI_1").empty() && (!rho || *rho <= 0.0)) {
        return Status::error(StatusCode::InvalidConfig,
                             "aerodynamics needs the site's air density (kg/m³); it has no "
                             "default");
    }
    rho_ = rho.value_or(0.0);
    auto wind = windOf(config);
    if (!wind) {
        return Status::error(StatusCode::InvalidConfig, wind.error());
    }
    wind_ = *wind;
    const World::Colliders& colliders = world.colliders();
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        if (colliders.body[c] == kWorldBody && colliders.shape[c] == ShapeType::Plane) {
            groundPoint_ = colliders.poseInBody[c].position;
            break;
        }
    }
    const World::Bodies& bodies = world.bodies();
    const std::vector<bool> moving = movingBodies(world.raw());
    for (const SchemaInstance* s : world.instancesOf("AeroAPI_1")) {
        const auto b =
            static_cast<std::size_t>(std::ranges::find(bodies.prim, s->prim) - bodies.prim.begin());
        const std::string where(world.primPath(s->prim));
        if (b >= bodies.prim.size() || !moving[b]) {
            return Status::error(StatusCode::InvalidConfig,
                                 where + ": AeroAPI_1 belongs on a link that moves");
        }
        if (s->token("aero:model").value_or("drag_box") == "none") {
            continue;
        }
        const auto vector = [&](const char* attr, Vec3 fallback) {
            const auto& v = s->realArray(attr);
            return v.size() == 3 ? Vec3{v[0], v[1], v[2]} : fallback;
        };
        Link link;
        link.name = std::string(world.signalInstanceName(s->prim));
        link.area = vector("aero:referenceArea", {-1.0, -1.0, -1.0});
        if (link.area[0] < 0.0 || link.area[1] < 0.0 || link.area[2] < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 where + ": AeroAPI_1 needs aero:referenceArea (m², three)");
        }
        link.cd = vector("aero:dragCoeff", {1.0, 1.0, 1.0});
        const Vec3 cop = vector("aero:centerOfPressure", {0.0, 0.0, 0.0});
        const Vec3& com = bodies.comLocal[b];
        link.centre = {cop[0] - com[0], cop[1] - com[1], cop[2] - com[2]};  // from the COM
        link.damping = vector("aero:angularDamping", {0.0, 0.0, 0.0});
        const std::string base = "body/" + link.name + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        link.pose = d.input(pose);
        SignalSpec twist = spec(base + "twist", SignalType::Wrench, units::m_per_s, "world_local");
        twist.feedback = true;
        link.twist = d.input(twist);
        link.wrench =
            d.output(spec(base + "wrench/aero", SignalType::Wrench, units::N, "world_local"));
        link.airspeed = d.output(spec(base + "airspeed", SignalType::Scalar, units::m_per_s));
        links_.push_back(std::move(link));
    }
    gust_ = d.output(spec("env/wind/gust", SignalType::Vec3, units::m_per_s, "world_local"));
    // The weather's wind grid (config "weather": it runs): inside it, the mean wind is the grid's.
    windField_ = SignalId{};
    if (config.get<bool>("weather").value_or(false)) {
        SignalSpec field =
            spec("env/wind/velocity_field", SignalType::Field3D, units::m_per_s, "world_local");
        field.feedback = true;
        windField_ = d.input(field);
    }
    windVelocity_ =
        d.output(spec("env/wind/velocity", SignalType::Vec3, units::m_per_s, "world_local"));
    return {};
}

Status AeroDrag::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (Link& link : links_) {
        link.turbulence.reset();
        bus.writeN(link.wrench, std::array<double, 6>{});
        bus.write(link.airspeed, 0.0);
    }
    bus.writeN(gust_, Vec3{});
    bus.writeN(windVelocity_, Vec3{});
    return {};
}

Status AeroDrag::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    // The direction the wind blows towards, in ENU.
    const double from = wind_.fromDegrees * std::numbers::pi / 180.0;
    const Vec3 downwind{-std::sin(from), -std::cos(from), 0.0};
    const Vec3 left{-downwind[1], downwind[0], 0.0};
    const std::shared_ptr<const fields::Field> field =
        windField_.valid() ? windFieldSignal_.read(bus, windField_) : nullptr;
    for (std::size_t i = 0; i < links_.size(); ++i) {
        Link& link = links_[i];
        const std::span<const double> p = bus.readN(link.pose);
        const std::span<const double> t = bus.readN(link.twist);
        const std::array<double, 4> q{p[3], p[4], p[5], p[6]};
        const Vec3 arm = rotate(q, link.centre);  // COM to centre of pressure, world
        const Vec3 omega{t[3], t[4], t[5]};
        const Vec3 spin = cross(omega, arm);
        const double height = p[2] - groundPoint_[2];
        const double mean = meanWindAt(wind_, height);
        Vec3 air{mean * downwind[0], mean * downwind[1], 0.0};
        if (field) {
            const fields::Vec3 at{p[0] + arm[0], p[1] + arm[1], p[2] + arm[2]};
            const auto& box = field->bounds();
            if (at[0] >= box[0][0] && at[0] <= box[1][0] && at[1] >= box[0][1] &&
                at[1] <= box[1][1] && at[2] >= box[0][2] && at[2] <= box[1][2]) {
                air = field->sampleVector(at);
            }
        }
        Vec3 velocity{t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};
        if (wind_.dryden && wind_.speed > 0.0) {
            const double w20 = meanWindAt(wind_, kW20HeightM);
            const double airspeed =
                std::hypot(velocity[0] - air[0], velocity[1] - air[1], velocity[2] - air[2]);
            random::Stream rng(context.seed, context.stepIndex, static_cast<std::uint32_t>(i));
            const std::array<double, 5> normals{rng.normal(), rng.normal(), rng.normal(),
                                                rng.normal(), rng.normal()};
            const Vec3 g = link.turbulence.step(dt, height, airspeed, w20, normals);
            for (std::size_t k = 0; k < 3; ++k) {
                air[k] += g[0] * downwind[k] + g[1] * left[k];
            }
            air[2] += g[2];
            if (i == 0) {
                bus.writeN(gust_, Vec3{g[0] * downwind[0] + g[1] * left[0],
                                       g[0] * downwind[1] + g[1] * left[1], g[2]});
            }
        }
        if (i == 0) {
            bus.writeN(windVelocity_, air);
        }
        // Drag in the body frame: −½·ρ·|v|·(Cd ∘ A ∘ v), v the link's velocity through the air.
        const Vec3 relative{velocity[0] - air[0], velocity[1] - air[1], velocity[2] - air[2]};
        const Vec3 local = rotate(conjugate(q), relative);
        const double speed = std::hypot(local[0], local[1], local[2]);
        Vec3 force{};
        for (std::size_t k = 0; k < 3; ++k) {
            force[k] = -0.5 * rho_ * speed * link.cd[k] * link.area[k] * local[k];
        }
        const Vec3 omegaLocal = rotate(conjugate(q), omega);
        const Vec3 damping{-link.damping[0] * omegaLocal[0], -link.damping[1] * omegaLocal[1],
                           -link.damping[2] * omegaLocal[2]};
        const Vec3 f = rotate(q, force);
        const Vec3 moment = cross(arm, f);
        const Vec3 tau = rotate(q, damping);
        bus.writeN(link.wrench, std::array<double, 6>{f[0], f[1], f[2], moment[0] + tau[0],
                                                      moment[1] + tau[1], moment[2] + tau[2]});
        bus.write(link.airspeed, speed);
    }
    return {};
}

Status AeroDrag::saveState(StateWriter& writer) const {
    for (const Link& link : links_) {
        for (const double v : link.turbulence.state()) {
            writer.put(v);
        }
    }
    return {};
}

Status AeroDrag::restoreState(StateReader& reader) {
    for (Link& link : links_) {
        std::array<double, 5> s{};
        for (double& v : s) {
            if (!reader.get(v)) {
                return Status::error(StatusCode::InvalidConfig, "aerodynamics state truncated");
            }
        }
        link.turbulence.setState(s);
    }
    return {};
}

}  // namespace lagrange::aerodynamics
