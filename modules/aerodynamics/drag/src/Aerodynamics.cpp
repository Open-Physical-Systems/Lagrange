// The aerodynamics domain: link drag, the mean wind profile and Dryden turbulence
// (Aerodynamics.hpp; docs/design/domains/aerodynamics.md §2).
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
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

namespace {

std::size_t rootOf(const World::Bodies& bodies, std::size_t b) {
    for (std::size_t guard = 0; guard < bodies.parent.size(); ++guard) {
        const BodyId p = bodies.parent[b];
        if (p == kWorldBody) {
            break;
        }
        b = p.index;
    }
    return b;
}

/// Root body → group, in the order the rotors come.
std::map<std::size_t, std::size_t> rotorRoots(const WorldView& world) {
    std::map<std::size_t, std::size_t> out;
    const World::Joints& joints = world.joints();
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const auto j =
            static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) - joints.prim.begin());
        const auto radius = s->real("aero:rotor:radius");
        if (j < joints.prim.size() && radius && *radius > 0.0) {
            out.try_emplace(rootOf(world.bodies(), joints.child[j].index), out.size());
        }
    }
    return out;
}

}  // namespace

std::vector<RotorSource> rotorSources(const WorldView& world) {
    const std::map<std::size_t, std::size_t> roots = rotorRoots(world);
    std::vector<RotorSource> out;
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const auto j =
            static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) - joints.prim.begin());
        const auto radius = s->real("aero:rotor:radius");
        if (j >= joints.prim.size() || !radius || *radius <= 0.0) {
            continue;  // rotor_aero reports it
        }
        const std::size_t child = joints.child[j].index;
        out.push_back({std::string(world.signalInstanceName(s->prim)),
                       std::string(world.signalInstanceName(bodies.prim[child])), *radius,
                       roots.at(rootOf(bodies, child))});
    }
    return out;
}

std::vector<std::size_t> rotorGroupOfBodies(const WorldView& world) {
    const std::map<std::size_t, std::size_t> roots = rotorRoots(world);
    const World::Bodies& bodies = world.bodies();
    std::vector<std::size_t> out(bodies.prim.size(), Downwash::kNoGroup);
    for (std::size_t b = 0; b < out.size(); ++b) {
        if (const auto it = roots.find(rootOf(bodies, b)); it != roots.end()) {
            out[b] = it->second;
        }
    }
    return out;
}

std::vector<Downwash::Box> downwashObstacles(const World& world) {
    // Boxes fixed to the world: on it, or on a body that does not move (a URDF world's links),
    // placed through the chain of fixed parents.
    const std::vector<bool> moving = movingBodies(world);
    const World::Bodies& bodies = world.bodies;
    const auto poseOf = [&](BodyId b) {
        Pose pose{};
        for (std::size_t guard = 0; b != kWorldBody && guard <= bodies.parent.size(); ++guard) {
            pose = compose(bodies.poseInParent[b.index], pose);
            b = bodies.parent[b.index];
        }
        return pose;
    };
    std::vector<Downwash::Box> out;
    const World::Colliders& c = world.colliders;
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        if (c.shape[i] != ShapeType::Box || (c.body[i] != kWorldBody && moving[c.body[i].index])) {
            continue;
        }
        const Pose pose =
            c.body[i] == kWorldBody ? c.poseInBody[i] : compose(poseOf(c.body[i]), c.poseInBody[i]);
        out.push_back({pose.position, pose.orientation, c.size[i]});
    }
    return out;
}

double wallInflow(double gapOverDiameter) {
    if (gapOverDiameter >= kWallReach) {
        return 0.0;
    }
    return kWallInflowGain * std::exp(-std::max(gapOverDiameter, 0.0) / kWallInflowLength);
}

double fountainLift(double heightOverDiagonal) {
    if (heightOverDiagonal >= kFountainReach || heightOverDiagonal < 0.0) {
        return 0.0;
    }
    const double x = std::max(heightOverDiagonal, kFountainLowest);
    return kFountainGain *
           (std::exp(-x / kFountainLength) - std::exp(-kFountainReach / kFountainLength));
}

double dustThreshold(double diameter, double rho, double gravity) {
    return kBagnoldA * std::sqrt((kGrainDensity - rho) / rho * gravity * diameter);
}

double liftedDust(double peakOutwash, double diameter, double most, double rho, double gravity) {
    const double friction = kWallJetFriction * peakOutwash;
    const double threshold = dustThreshold(diameter, rho, gravity);
    if (friction <= threshold || most <= 0.0) {
        return 0.0;
    }
    const double ratio = threshold / friction;
    return most * (1.0 - ratio * ratio);
}

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
    hasGround_ = false;
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        if (colliders.body[c] == kWorldBody && colliders.shape[c] == ShapeType::Plane) {
            groundPoint_ = colliders.poseInBody[c].position;
            groundNormal_ = rotate(colliders.poseInBody[c].orientation, {0.0, 0.0, 1.0});
            hasGround_ = true;
            break;
        }
    }
    const World::Bodies& bodies = world.bodies();
    const Vec3 g = world.gravityLocal();
    gravity_ = std::hypot(g[0], g[1], g[2]);
    downwashOn_ = config.get<bool>("downwash").value_or(true);
    const std::string model = config.get<std::string>("downwash_model").value_or("fitted");
    if (model != "fitted" && model != "lbm") {
        return Status::error(StatusCode::InvalidConfig,
                             "aerodynamics.downwash_model is fitted or lbm, not '" + model + "'");
    }
    lbmWanted_ = downwashOn_ && model == "lbm";
    lbmCell_ = config.get<double>("downwash_lbm_cell").value_or(0.04);
    lbmCpu_ = config.get<bool>("downwash_lbm_cpu").value_or(false);
    if (lbmCell_ <= 0.0) {
        return Status::error(StatusCode::InvalidConfig, "downwash_lbm_cell must be > 0 m");
    }
    rotors_.clear();
    const std::vector<std::size_t> groupOfBody = rotorGroupOfBodies(world);
    std::size_t groups = 0;
    std::map<std::string, SignalId> twists;  // one input per body
    const auto twistOf = [&](const std::string& body) {
        auto it = twists.find(body);
        if (it == twists.end()) {
            SignalSpec t =
                spec("body/" + body + "/twist", SignalType::Wrench, units::m_per_s, "world_local");
            t.feedback = true;
            it = twists.emplace(body, d.input(t)).first;
        }
        return it->second;
    };
    std::map<std::string, SignalId> poses;
    const auto poseOf = [&](const std::string& body) {
        auto it = poses.find(body);
        if (it == poses.end()) {
            SignalSpec p =
                spec("body/" + body + "/pose", SignalType::Pose, units::m, "world_local");
            p.feedback = true;
            it = poses.emplace(body, d.input(p)).first;
        }
        return it->second;
    };
    if (downwashOn_) {
        for (const RotorSource& source : rotorSources(world)) {
            RotorLink r;
            r.radius = source.radius;
            r.group = source.group;
            groups = std::max(groups, source.group + 1);
            const std::string base = "rotor/" + source.joint + "/";
            SignalSpec thrust = spec(base + "thrust", SignalType::Scalar, units::N);
            thrust.feedback = true;
            r.thrust = d.input(thrust);
            SignalSpec induced =
                spec(base + "induced_velocity", SignalType::Scalar, units::m_per_s);
            induced.feedback = true;
            r.induced = d.input(induced);
            SignalSpec axis =
                spec(base + "thrust_axis", SignalType::Vec3, units::one, "world_local");
            axis.feedback = true;
            r.axis = d.input(axis);
            r.pose = poseOf(source.link);
            r.twist = twistOf(source.link);
            r.air = d.output(
                spec(base + "air_velocity", SignalType::Vec3, units::m_per_s, "world_local"));
            rotors_.push_back(r);
        }
    }
    groupCount_ = groups;
    if (hasGround_) {
        downwash_.setGround(groundPoint_, groundNormal_);
    } else {
        downwash_.clearGround();
    }
    downwash_.setObstacles(downwashObstacles(world.raw()));
    others_ = downwash_;
    airBox_ = SignalId{};
    if (lbmWanted_ && !rotors_.empty()) {
        airBox_ =
            d.output(spec("aero/air_box", SignalType::Field3D, units::m_per_s, "world_local"));
    }
    dustOut_ = SignalId{};
    const auto dustDiameter = config.get<double>("ground_dust_diameter");
    const auto dustMost = config.get<double>("ground_dust_concentration");
    if (dustDiameter || dustMost) {
        if (!dustDiameter || *dustDiameter <= 0.0 || !dustMost || *dustMost < 0.0 || rho_ <= 0.0) {
            return Status::error(
                StatusCode::InvalidConfig,
                "loose ground dust needs ground_dust_diameter (m) and "
                "ground_dust_concentration (kg/m³), together, and the air density");
        }
        dustDiameter_ = *dustDiameter;
        dustMost_ = *dustMost;
        if (!rotors_.empty()) {
            dustOut_ = d.output(spec("env/dust/downwash", SignalType::Scalar, units::kg_per_m3));
        }
    }
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
        link.pose = poseOf(link.name);
        link.twist = twistOf(link.name);
        link.group = downwashOn_ ? groupOfBody[b] : Downwash::kNoGroup;
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
    for (const RotorLink& r : rotors_) {
        bus.writeN(r.air, Vec3{});
    }
    if (dustOut_.valid()) {
        bus.write(dustOut_, 0.0);
    }
    downwash_.update({});
    others_.update({});
    live_.reset();
    published_.reset();
    liveGroup_ = Downwash::kNoGroup;
    if (airBox_.valid()) {
        bus.publishBuffer(airBox_, fields::fieldBuffer(nullptr));
        auto live = std::make_unique<LiveAir>();
        LiveAir::Settings settings;
        settings.cell = lbmCell_;
        settings.airDensity = rho_;
        settings.cpu = lbmCpu_;
        std::string why;
        if (live->start(settings, why)) {
            live->setGround(hasGround_, groundPoint_[2]);
            live->setObstacles(downwash_.obstacles());
            live_ = std::move(live);
            liveGroup_ = rotors_.front().group;
        } else {
            std::fprintf(stderr,
                         "lagrange: aerodynamics: the live air box needs a hardware GPU: %s; the "
                         "fitted downwash runs instead\n",
                         why.c_str());
        }
    }
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
    // The mean air at a point: the wind grid inside it, else the logarithmic profile.
    const auto meanAir = [&](const Vec3& at) {
        if (field) {
            const auto& box = field->bounds();
            if (at[0] >= box[0][0] && at[0] <= box[1][0] && at[1] >= box[0][1] &&
                at[1] <= box[1][1] && at[2] >= box[0][2] && at[2] <= box[1][2]) {
                return field->sampleVector(at);
            }
        }
        const double mean = meanWindAt(wind_, at[2] - groundPoint_[2]);
        return Vec3{mean * downwind[0], mean * downwind[1], 0.0};
    };
    // The rotors' wakes, from last step's thrust: one group per robot, leaning with the air.
    std::vector<Downwash::Rotor> wakes;
    std::vector<Vec3> relative(groupCount_, Vec3{});
    std::vector<int> counted(groupCount_, 0);
    for (const RotorLink& r : rotors_) {
        const std::span<const double> p = bus.readN(r.pose);
        const std::span<const double> v = bus.readN(r.twist);
        const std::span<const double> a = bus.readN(r.axis);
        Downwash::Rotor w;
        w.hub = {p[0], p[1], p[2]};
        w.thrustAxis = {a[0], a[1], a[2]};
        w.radius = r.radius;
        w.induced = bus.read(r.induced);
        w.thrust = bus.read(r.thrust);
        w.group = r.group;
        wakes.push_back(w);
        const Vec3 air = meanAir(w.hub);
        for (std::size_t k = 0; k < 3; ++k) {
            relative[r.group][k] += air[k] - v[k];
        }
        ++counted[r.group];
    }
    for (std::size_t g = 0; g < groupCount_; ++g) {
        for (double& c : relative[g]) {
            c /= std::max(counted[g], 1);
        }
    }
    downwash_.update(wakes, relative);
    // The live box: the boxed multirotor's rotors push its air; every other multirotor keeps its
    // fitted wake (others_).
    if (live_) {
        std::vector<Downwash::Rotor> rest;
        std::vector<LbmBox::Rotor> discs;
        Vec3 centre{};
        for (const Downwash::Rotor& w : wakes) {
            if (w.group != liveGroup_) {
                rest.push_back(w);
                continue;
            }
            LbmBox::Rotor disc;
            disc.centre = w.hub;
            disc.axis = w.thrustAxis;
            disc.radius = w.radius;
            disc.thrust = std::max(0.0, w.thrust);
            discs.push_back(disc);
            for (std::size_t k = 0; k < 3; ++k) {
                centre[k] += w.hub[k];
            }
        }
        for (double& c : centre) {
            c /= static_cast<double>(std::max<std::size_t>(discs.size(), 1));
        }
        others_.update(rest, relative);
        live_->setAmbient(meanAir(centre));  // the mean wind at the drone blows through the box
        live_->advance(dt, centre, discs);
        if (airBox_.valid()) {
            constexpr double kFieldInterval = 0.1;  // s: the viewer asks a few times a second
            std::shared_ptr<const fields::Field> now = live_->field(kFieldInterval);
            if (now != published_) {
                published_ = now;
                bus.publishBuffer(airBox_, fields::fieldBuffer(now.get()));
            }
        }
    }
    // Other robots' rotors at a point, for a link or rotor of `group`: the box inside it.
    const auto wakeAt = [&](const Vec3& at, std::size_t group) {
        if (live_ && live_->contains(at)) {
            Vec3 w = others_.velocityAt(at, group);
            if (group != liveGroup_) {
                const Vec3 box = live_->velocityAt(at);
                for (std::size_t k = 0; k < 3; ++k) {
                    w[k] += box[k];
                }
            }
            return w;
        }
        return downwash_.velocityAt(at, group);
    };
    // The fountain under each low multirotor: a push up its thrust axis on its first link.
    std::vector<Vec3> fountain(groupCount_, Vec3{});
    for (const Downwash::Group& g : downwash_.groups()) {
        if (!g.hits || g.length <= 0.0 || g.id >= groupCount_) {
            continue;
        }
        const double height = (g.centre[0] - g.hitPoint[0]) * g.hitNormal[0] +
                              (g.centre[1] - g.hitPoint[1]) * g.hitNormal[1] +
                              (g.centre[2] - g.hitPoint[2]) * g.hitNormal[2];
        double thrust = 0.0;
        for (const Downwash::Rotor& r : g.rotors) {
            thrust += std::max(0.0, r.thrust);
        }
        const double lift = fountainLift(height / g.length) * thrust;
        fountain[g.id] = {-lift * g.down[0], -lift * g.down[1], -lift * g.down[2]};
    }
    std::vector<bool> fountainApplied(groupCount_, false);
    Vec3 firstGust{};
    for (std::size_t i = 0; i < links_.size(); ++i) {
        Link& link = links_[i];
        const std::span<const double> p = bus.readN(link.pose);
        const std::span<const double> t = bus.readN(link.twist);
        const std::array<double, 4> q{p[3], p[4], p[5], p[6]};
        const Vec3 arm = rotate(q, link.centre);  // COM to centre of pressure, world
        const Vec3 omega{t[3], t[4], t[5]};
        const Vec3 spin = cross(omega, arm);
        const double height = p[2] - groundPoint_[2];
        const Vec3 at{p[0] + arm[0], p[1] + arm[1], p[2] + arm[2]};
        Vec3 air = meanAir(at);
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
                firstGust = {g[0] * downwind[0] + g[1] * left[0],
                             g[0] * downwind[1] + g[1] * left[1], g[2]};
                bus.writeN(gust_, firstGust);
            }
        }
        if (i == 0) {
            bus.writeN(windVelocity_, air);
        }
        // Other robots' rotors blow on it.
        const Vec3 wake = wakeAt(at, link.group);
        for (std::size_t k = 0; k < 3; ++k) {
            air[k] += wake[k];
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
        Vec3 f = rotate(q, force);
        const Vec3 moment = cross(arm, f);
        if (link.group < groupCount_ && !fountainApplied[link.group]) {
            fountainApplied[link.group] = true;  // on its first link, at its centre of mass
            for (std::size_t k = 0; k < 3; ++k) {
                f[k] += fountain[link.group][k];
            }
        }
        const Vec3 tau = rotate(q, damping);
        bus.writeN(link.wrench, std::array<double, 6>{f[0], f[1], f[2], moment[0] + tau[0],
                                                      moment[1] + tau[1], moment[2] + tau[2]});
        bus.write(link.airspeed, speed);
    }
    // Each rotor's air: the wind and gust plus the other robots' wakes at its hub, and a wall
    // beside it: more air down through the rotors on its side, less on the far side (wallInflow).
    struct WallSide {
        double gap{std::numeric_limits<double>::infinity()};  // m, nearest rotor tip to the wall
        double reach{0.0};  // m, the farthest hub towards the wall from the centre
        Vec3 centre{}, towards{};
    };
    std::vector<WallSide> sides(groupCount_);
    for (const Downwash::Group& g : downwash_.groups()) {
        WallSide& side = sides[g.id];
        Vec3 normal{};
        const double distance = downwash_.nearestWall(g.centre, normal);
        if (!std::isfinite(distance)) {
            continue;
        }
        side.centre = g.centre;
        side.towards = {-normal[0], -normal[1], -normal[2]};
        double tip = 0.0;
        for (const Downwash::Rotor& r : g.rotors) {
            const double along = (r.hub[0] - g.centre[0]) * side.towards[0] +
                                 (r.hub[1] - g.centre[1]) * side.towards[1] +
                                 (r.hub[2] - g.centre[2]) * side.towards[2];
            side.reach = std::max(side.reach, along);
            tip = std::max(tip, along + r.radius);
        }
        side.gap = distance - tip;
    }
    for (std::size_t i = 0; i < rotors_.size(); ++i) {
        const Downwash::Rotor& w = wakes[i];
        const Vec3 mean = meanAir(w.hub);
        Vec3 wake = wakeAt(w.hub, rotors_[i].group);
        const bool boxed = live_ && w.group == liveGroup_;
        if (boxed) {
            // The box's air over the disc less the disc's own: walls, ground and the other
            // rotors of the drone are in it, so no fitted wall effect on top.
            LbmBox::Rotor disc;
            disc.centre = w.hub;
            disc.axis = w.thrustAxis;
            disc.radius = w.radius;
            disc.thrust = std::max(0.0, w.thrust);
            const Vec3 box = live_->airAtRotor(disc);
            for (std::size_t k = 0; k < 3; ++k) {
                wake[k] += box[k];
            }
        }
        Vec3 air{mean[0] + firstGust[0] + wake[0], mean[1] + firstGust[1] + wake[1],
                 mean[2] + firstGust[2] + wake[2]};
        const WallSide& side = sides[w.group];
        if (!boxed && w.thrust > 0.0 && std::isfinite(side.gap) && side.reach > 0.0) {
            // +1 for a rotor as far towards the wall as any, −1 for one as far away.
            const double share = ((w.hub[0] - side.centre[0]) * side.towards[0] +
                                  (w.hub[1] - side.centre[1]) * side.towards[1] +
                                  (w.hub[2] - side.centre[2]) * side.towards[2]) /
                                 side.reach;
            const double extra =
                w.induced * wallInflow(side.gap / (2.0 * w.radius)) * std::clamp(share, -1.0, 1.0);
            for (std::size_t k = 0; k < 3; ++k) {
                air[k] -= extra * w.thrustAxis[k];
            }
        }
        bus.writeN(rotors_[i].air, air);
    }
    if (dustOut_.valid()) {
        bus.write(dustOut_,
                  liftedDust(downwash_.peakOutwash(), dustDiameter_, dustMost_, rho_, gravity_));
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
