// The fire domain (RFC 0022 step 3); rules on the classes in Fires.hpp.
#include "lagrange/fields/Field.hpp"
#include "lagrange/fire/Fires.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <numbers>
#include <string_view>

namespace lagrange::fire {

using namespace literals;

namespace {

// kg/s and kg/(m²·s): no named unit in core, the dimension is what wiring checks.
constexpr Unit kKgPerS{{.mass = 1, .time = -1}, 1.0, QuantityKind::Unspecified};
constexpr Unit kKgPerM2S{{.length = -2, .mass = 1, .time = -1}, 1.0, QuantityKind::Unspecified};
constexpr double kWaterDensity = 1000.0;  // kg/m³: a rain rate in m/s as kg/(m²·s) (mm/s)
// Closer than this many fire diameters the point-source model over-predicts (NISTIR 6546 §3:
// "overly conservative within a few fire diameters"). TODO(verify): a published R/D limit.
constexpr double kNearFieldDiameters = 2.0;
constexpr double kSameSpot = 1e-3;  // m: a collider at a prim's origin is the prim's own

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

SignalSpec optionalInput(std::string name, SignalType type, Unit unit, double fallback,
                         std::string frame = {}) {
    SignalSpec s = spec(std::move(name), type, unit, std::move(frame));
    s.optional = true;
    s.feedback = true;
    s.defaultValue = fallback;
    return s;
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double norm(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
Vec3 pointIn(const Pose& pose, const Vec3& local) {
    return compose(pose, Pose{local, {1.0, 0.0, 0.0, 0.0}}).position;
}

/// Half the height of a collider above its centre, upright (m).
double halfHeight(ShapeType type, const std::array<double, 3>& size) {
    switch (type) {
        case ShapeType::Sphere:
            return size[0];
        case ShapeType::Capsule:
            return size[1] + size[0];
        case ShapeType::Cylinder:
            return size[1];
        case ShapeType::Box:
        case ShapeType::ConvexHull:
            return size[2];
        default:
            return 0.0;
    }
}

/// Its footprint, upright (m²).
double footprint(ShapeType type, const std::array<double, 3>& size) {
    switch (type) {
        case ShapeType::Sphere:
        case ShapeType::Capsule:
        case ShapeType::Cylinder:
            return std::numbers::pi * size[0] * size[0];
        case ShapeType::Box:
        case ShapeType::ConvexHull:
            return 4.0 * size[0] * size[1];
        default:
            return 0.0;
    }
}

/// A prim's body and pose in it. A relationship or schema on a link names its body's own prim,
/// which the prim table of a URDF robot does not map to the body: look it up directly.
std::pair<BodyId, Pose> bodyAndPose(const WorldView& world, PrimId prim, bool& ownPrim) {
    const std::vector<PrimId>& own = world.raw().bodies.prim;
    for (std::uint32_t b = 0; b < own.size(); ++b) {
        if (own[b] == prim) {
            ownPrim = true;
            return {BodyId{b}, Pose{}};
        }
    }
    ownPrim = false;
    return {world.primBody(prim), world.primPoseInBody(prim)};
}

/// The colliders that are a prim's own: every collider of its body when the prim is the body's
/// own prim, else those centred at the prim's origin.
std::vector<std::uint32_t> collidersOf(const WorldView& world, BodyId body, const Pose& pose,
                                       bool ownPrim) {
    const World::Colliders& c = world.colliders();
    std::vector<std::uint32_t> out;
    for (std::uint32_t i = 0; i < c.body.size(); ++i) {
        if (c.body[i] != body || c.shape[i] == ShapeType::Plane ||
            c.shape[i] == ShapeType::HeightField) {
            continue;
        }
        if (ownPrim || norm(sub(c.poseInBody[i].position, pose.position)) < kSameSpot) {
            out.push_back(i);
        }
    }
    return out;
}

std::string fireError(const std::string& name, const std::string& text) {
    return "fire " + name + ": " + text;
}

}  // namespace

// ---------------------------------------------------------------------------------- FireSpec

double FireSpec::fullHrr(double burningArea) const {
    switch (growth) {
        case Growth::TSquared:
            return hrrPerArea * burningArea;
        case Growth::Pool:
            return poolHrr(poolBurningRate, poolExtinction, heatOfCombustion, burningArea);
        case Growth::Curve: {
            double peak = 0.0;
            for (std::size_t i = 1; i < curve.size(); i += 2) {
                peak = std::max(peak, curve[i]);
            }
            return area > 0.0 ? peak * burningArea / area : 0.0;
        }
    }
    return 0.0;
}

std::vector<FireSpec> firesOf(const WorldView& world, std::string& error) {
    std::vector<FireSpec> out;
    const World::Colliders& colliders = world.colliders();
    for (const SchemaInstance* s : world.instancesOf("FireAPI_1")) {
        FireSpec f;
        f.prim = s->prim;
        f.name = std::string(world.signalInstanceName(s->prim));
        const auto fail = [&](const std::string& text) {
            error = fireError(f.name, text);
            return std::vector<FireSpec>{};
        };
        if (const auto preset = s->token("fire:preset"); preset && !preset->empty()) {
            return fail("no fuel preset " + std::string(*preset) +
                        ": the presets ship empty until their cited values are confirmed (RFC "
                        "0022 open question 3); give the fuel's values with their source");
        }
        bool ownPrim = false;
        const auto [body, pose] = bodyAndPose(world, s->prim, ownPrim);
        f.body = body;
        f.colliders = collidersOf(world, body, pose, ownPrim);
        const std::string fireClass(s->token("fire:class").value_or("a"));
        if (fireClass == "a") {
            f.fireClass = FireClass::A;
        } else if (fireClass == "b") {
            f.fireClass = FireClass::B;
        } else if (fireClass == "c") {
            f.fireClass = FireClass::C;
        } else {
            return fail("fire:class " + fireClass + " is not a, b or c");
        }
        const std::string growth(s->token("fire:growth").value_or("t_squared"));
        if (growth == "pool") {
            f.growth = Growth::Pool;
        } else if (growth != "t_squared") {
            return fail("fire:growth " + growth + " is not t_squared or pool");
        }
        const auto curve = s->realArray("fire:hrrCurve");
        if (!curve.empty()) {
            if (curve.size() < 4 || curve.size() % 2 != 0) {
                return fail("fire:hrrCurve needs two or more pairs of time (s) and HRR (W)");
            }
            for (std::size_t i = 0; i < curve.size(); i += 2) {
                if (curve[i + 1] < 0.0 || (i > 0 && !(curve[i] > curve[i - 2]))) {
                    return fail("fire:hrrCurve: times must increase and HRR be 0 or more");
                }
            }
            f.curve.assign(curve.begin(), curve.end());
            f.growth = Growth::Curve;
        }
        const auto required = [&](const char* name) -> std::optional<double> {
            const auto v = s->real(name);
            if (!v) {
                error = fireError(f.name, std::string(name) + " is required (the fuel's data)");
            }
            return v;
        };
        const auto positive = [&](const char* name) -> std::optional<double> {
            const auto v = required(name);
            if (v && !(*v > 0.0)) {
                error = fireError(f.name, std::string(name) + " must be over 0");
                return std::nullopt;
            }
            return v;
        };
        const auto mass = positive("fire:fuelMass");
        const auto heat = positive("fire:heatOfCombustion");
        const auto chi = required("fire:radiativeFraction");
        const auto soot = required("fire:sootYield");
        if (!mass || !heat || !chi || !soot) {
            return {};
        }
        f.fuelMass = *mass;
        f.heatOfCombustion = *heat;
        f.radiativeFraction = *chi;
        f.sootYield = *soot;
        if (f.radiativeFraction < 0.0 || f.radiativeFraction > 1.0 || f.sootYield < 0.0) {
            return fail("fire:radiativeFraction must be 0 to 1 and fire:sootYield 0 or more");
        }
        if (f.growth == Growth::TSquared) {
            const auto alpha = positive("fire:alpha");
            const auto perArea = positive("fire:hrrPerArea");
            if (!alpha || !perArea) {
                return {};
            }
            f.alpha = *alpha;
            f.hrrPerArea = *perArea;
        } else if (f.growth == Growth::Pool) {
            const auto rate = positive("fire:poolBurningRate");
            const auto kb = positive("fire:poolExtinction");
            if (!rate || !kb) {
                return {};
            }
            f.poolBurningRate = *rate;
            f.poolExtinction = *kb;
        }
        if (f.fireClass != FireClass::B) {
            const auto critical = positive("fire:criticalWaterRate");
            if (!critical) {
                return {};
            }
            f.criticalWaterRate = *critical;
        } else if (const auto critical = s->real("fire:criticalWaterRate")) {
            f.criticalWaterRate = *critical;
        }
        f.decayFraction = s->real("fire:decayFraction").value_or(0.2);
        if (f.decayFraction < 0.0 || f.decayFraction > 1.0) {
            return fail("fire:decayFraction must be 0 to 1");
        }
        f.ignitionTime = s->real("fire:ignitionTime").value_or(-1.0);
        f.ignitionTemperature = s->real("fire:ignitionTemperature");
        f.blowOffSpeed = s->real("fire:blowOffSpeed");
        f.blowOffMaxHrr = s->real("fire:blowOffMaxHrr");
        f.foamBlanketThickness = s->real("fire:foamBlanketThickness");
        if ((f.blowOffSpeed && !(*f.blowOffSpeed > 0.0)) ||
            (f.foamBlanketThickness && !(*f.foamBlanketThickness > 0.0)) ||
            (f.ignitionTemperature && !(*f.ignitionTemperature > 0.0))) {
            return fail(
                "fire:blowOffSpeed, foamBlanketThickness and ignitionTemperature must be "
                "over 0 when given");
        }
        // The burning area: authored, else the footprint of its own colliders; the flame's base
        // is on their top (upright: their rotation is not taken into account).
        double footprintArea = 0.0;
        double top = -std::numeric_limits<double>::infinity();
        for (const std::uint32_t c : f.colliders) {
            footprintArea += footprint(colliders.shape[c], colliders.size[c]);
            top = std::max(top, colliders.poseInBody[c].position[2] +
                                    halfHeight(colliders.shape[c], colliders.size[c]));
        }
        f.area = s->real("fire:area").value_or(footprintArea);
        if (!(f.area > 0.0)) {
            return fail("fire:area is needed: " + std::string(world.primPath(s->prim)) +
                        " has no collider at its origin to take the footprint from");
        }
        f.baseInBody = pose.position;
        if (!f.colliders.empty()) {
            f.baseInBody[2] = top;
        }
        out.push_back(std::move(f));
    }
    return out;
}

// ------------------------------------------------------------------------------------ Burner

Burner::Burner(const FireSpec& spec, double extinguishFraction)
    : spec_(&spec), extinguish_(extinguishFraction) {}

double Burner::burningArea() const {
    return spec_ != nullptr ? spec_->area * (1.0 - foam_) : 0.0;
}

double Burner::fuelRemaining() const {
    return spec_ != nullptr ? std::max(spec_->fuelMass - burnt_, 0.0) : 0.0;
}

double Burner::massFlux() const {
    const double area = burningArea();
    return spec_ != nullptr && area > 0.0 ? hrr_ / (spec_->heatOfCombustion * area) : 0.0;
}

void Burner::holdFull() {
    if (spec_ == nullptr) {
        return;
    }
    state_ = FireState::FullyInvolved;
    free_ = hrr_ = spec_->fullHrr(spec_->area);
}

void Burner::step(double t, double dt, const Exposure& exposure) {
    if (spec_ == nullptr || isOut(state_)) {
        return;
    }
    const FireSpec& f = *spec_;
    if (state_ == FireState::NotIgnited) {
        const bool onTime = f.ignitionTime >= 0.0 && t + dt > f.ignitionTime;
        const bool hot =
            f.ignitionTemperature && exposure.nodeTemperature >= *f.ignitionTemperature;
        if (!onTime && !exposure.ignite && !hot) {
            return;
        }
        state_ = FireState::Growing;
        // By the clock, its growth started at the ignition time; otherwise at this step.
        clock_ = onTime ? std::max(t - f.ignitionTime, 0.0) : 0.0;
    }
    const auto putOut = [&](FireState why) {
        state_ = why;
        free_ = hrr_ = 0.0;
        cooling_ = steam_ = 0.0;
    };
    // Foam smothers: the blanket covers its share of the burning area (RFC 0022 §5.3).
    if (exposure.foamSupply > 0.0 && f.foamBlanketThickness) {
        foam_ = std::min(1.0, foam_ + exposure.foamSupply * dt / *f.foamBlanketThickness);
        if (foam_ >= 1.0) {
            putOut(FireState::OutSmothered);
            return;
        }
    }
    // A small flame blows off above its fuel's measured speed (§5.2).
    if (f.blowOffSpeed && exposure.airSpeed > *f.blowOffSpeed &&
        (!f.blowOffMaxHrr || hrr_ < *f.blowOffMaxHrr)) {
        putOut(FireState::OutBlownOff);
        return;
    }
    const double area = burningArea();
    const double full = f.fullHrr(area);
    waterRate_ = std::max(exposure.waterRate, 0.0);
    hazard_ = f.fireClass == FireClass::C && waterRate_ > 0.0;
    // Water at or above the critical rate stops the growth and decays the fire by Evans' law;
    // never on a burning liquid, under which it sinks (§5.1).
    const bool suppress = f.fireClass != FireClass::B && f.criticalWaterRate && waterRate_ > 0.0 &&
                          waterRate_ >= *f.criticalWaterRate;
    if (suppress) {
        if (atWater_ <= 0.0) {
            atWater_ = std::max(hrr_, std::numeric_limits<double>::min());
        }
        free_ *= std::exp(-dt / evansTau(waterRate_));
        if (f.growth == Growth::TSquared) {
            clock_ = std::sqrt(free_ / f.alpha);  // grows again from here if the water stops
        }
    } else {
        atWater_ = 0.0;
        clock_ += dt;
        switch (f.growth) {
            case Growth::TSquared:
                free_ = std::min(tSquaredHrr(f.alpha, clock_), full);
                break;
            case Growth::Pool:
                free_ = full;
                break;
            case Growth::Curve:
                free_ = curveHrr(f.curve, clock_) * (1.0 - foam_);
                break;
        }
    }
    const double factor = burnOutFactor(f.fuelMass - burnt_, f.fuelMass, f.decayFraction);
    hrr_ = free_ * factor;
    burnt_ += hrr_ * dt / f.heatOfCombustion;
    // What the water takes up: never more than it can boil, nor more than the fire gives.
    const double perKg = waterHeatPerKg(exposure.waterTemperature);
    cooling_ = std::min(waterRate_ * area * perKg, hrr_);
    steam_ = cooling_ / perKg;
    // Scaled by the remaining mass, Q falls exponentially in time over the tail: it is out once
    // it is down to the same share as water puts it out at.
    const double remaining = f.fuelMass - burnt_;
    if (remaining <= 0.0 ||
        (factor < 1.0 && remaining <= extinguish_ * f.decayFraction * f.fuelMass)) {
        putOut(FireState::OutBurnt);
    } else if (suppress && hrr_ < extinguish_ * atWater_) {
        putOut(FireState::OutWater);
    } else if (suppress || factor < 1.0 ||
               (f.growth == Growth::Curve && hrr_ < full * (1.0 - 1e-12) && clock_ > 0.0 &&
                curveHrr(f.curve, clock_ + dt) < curveHrr(f.curve, clock_))) {
        state_ = FireState::Decaying;
    } else if (f.growth == Growth::Pool || hrr_ >= full * (1.0 - 1e-12)) {
        state_ = FireState::FullyInvolved;
    } else {
        state_ = FireState::Growing;
    }
}

void Burner::save(std::vector<double>& out) const {
    out.insert(out.end(), {static_cast<double>(state_), clock_, free_, hrr_, burnt_, atWater_,
                           foam_, waterRate_, cooling_, steam_, hazard_ ? 1.0 : 0.0});
}

bool Burner::restore(std::span<const double> in) {
    if (in.size() != 11) {
        return false;
    }
    state_ = static_cast<FireState>(static_cast<int>(in[0]));
    clock_ = in[1];
    free_ = in[2];
    hrr_ = in[3];
    burnt_ = in[4];
    atWater_ = in[5];
    foam_ = in[6];
    waterRate_ = in[7];
    cooling_ = in[8];
    steam_ = in[9];
    hazard_ = in[10] != 0.0;
    return true;
}

// ------------------------------------------------------------------------------------- Fires

Fires::Fires(Mode mode, std::shared_ptr<const services::SceneQuery> scenery,
             std::vector<std::uint32_t> sceneryColliders)
    : mode_(mode), scenery_(std::move(scenery)), sceneryColliders_(std::move(sceneryColliders)) {}

ModuleInfo Fires::info() const {
    const std::string_view name = mode_ == Mode::Zero       ? "zero"
                                  : mode_ == Mode::Constant ? "constant"
                                                            : "correlations";
    return {"fire", name, 1, 0, mode_ != Mode::Simulate};
}

Duration Fires::preferredStep() const {
    return 100_ms;
}

Status Fires::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    std::string error;
    specs_ = firesOf(world, error);
    if (!error.empty()) {
        return Status::error(StatusCode::InvalidConfig, error);
    }
    out_.assign(specs_.size(), Out{});
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const std::string base = "fire/" + specs_[k].name + "/";
        out_[k].hrr = d.output(spec(base + "hrr", SignalType::Scalar, units::W));
        out_[k].state = d.output(spec(base + "state", SignalType::Int, units::one));
    }
    // The smoke's field and mass in the air, written in every mode (none under zero).
    smokeOn_ = config.get<bool>("smoke").value_or(true);
    smokeField_ = d.output(spec("env/smoke/extinction", SignalType::Field3D, units::per_m));
    smokeMass_ = d.output(spec("env/smoke/mass", SignalType::Scalar, units::kg));
    if (mode_ == Mode::Zero) {
        smokeOn_ = false;
        return {};
    }
    SmokeSettings smoke;
    smoke.voxel = config.get<double>("smoke_voxel").value_or(smoke.voxel);
    smoke.reach = config.get<double>("smoke_reach").value_or(smoke.reach);
    smoke.height = config.get<double>("smoke_height").value_or(smoke.height);
    const double most =
        config.get<double>("smoke_max_puffs").value_or(static_cast<double>(smoke.maxPuffs));
    smokePeriod_ = config.get<double>("smoke_period").value_or(0.5);
    if (!(smoke.voxel > 0.0) || !(smoke.reach > 0.0) || !(smoke.height > 0.0) || !(most >= 2.0) ||
        !(smokePeriod_ > 0.0)) {
        return Status::error(StatusCode::InvalidConfig,
                             "fire: smoke_voxel, smoke_reach, smoke_height and smoke_period must "
                             "be > 0, smoke_max_puffs at least 2");
    }
    smoke.maxPuffs = static_cast<std::size_t>(most);
    extinguishFraction_ = config.get<double>("extinguish_fraction").value_or(0.01);
    if (!(extinguishFraction_ > 0.0 && extinguishFraction_ < 1.0)) {
        return Status::error(StatusCode::InvalidConfig, "fire.extinguish_fraction must be 0 to 1");
    }
    siteDensity_ = config.get<double>("air_density").value_or(0.0);
    siteTemperature_ = config.get<double>("ambient_temperature").value_or(0.0);
    gravity_ = world.gravityLocal();
    // Water from the run's options (--fire-water NAME=RATE[@T]): a sprinkler's rate from T on.
    schedule_.clear();
    if (const auto* entries = config.find<std::vector<std::string>>("water_schedule")) {
        for (const std::string& entry : *entries) {
            const std::size_t eq = entry.find('=');
            const std::string name = entry.substr(0, eq);
            const auto it =
                std::ranges::find_if(specs_, [&](const FireSpec& f) { return f.name == name; });
            Schedule s;
            char* end = nullptr;
            const std::string rest = eq == std::string::npos ? "" : entry.substr(eq + 1);
            s.rate = std::strtod(rest.c_str(), &end);
            if (end != nullptr && *end == '@') {
                s.start = std::strtod(end + 1, &end);
            }
            if (it == specs_.end() || end == nullptr || *end != '\0' || !(s.rate >= 0.0) ||
                !std::isfinite(s.start)) {
                return Status::error(StatusCode::InvalidConfig,
                                     "--fire-water " + entry +
                                         ": NAME=RATE[@T] with a fire of the stage, RATE in "
                                         "kg/(m²·s) (mm/s of water), T in s");
            }
            s.fire = static_cast<std::size_t>(it - specs_.begin());
            schedule_.push_back(s);
        }
    }
    const World& raw = world.raw();
    rest_ = restPoses(raw);
    const std::vector<bool> moving = movingBodies(raw);
    movingIndex_.assign(raw.bodies.names.size(), -1);
    poses_.clear();
    const auto need = [&](BodyId b) {
        if (b == kWorldBody || b.index >= moving.size() || !moving[b.index] ||
            movingIndex_[b.index] >= 0) {
            return;
        }
        SignalSpec pose = spec("body/" + raw.bodies.names[b.index] + "/pose", SignalType::Pose,
                               units::m, "world_local");
        pose.feedback = true;
        movingIndex_[b.index] = static_cast<int>(poses_.size());
        poses_.push_back(d.input(pose));
    };
    // The thermal nodes the fires heat: each with a radiating area.
    targets_.clear();
    std::map<std::uint32_t, bool> nodePrims;
    for (const SchemaInstance* s : world.instancesOf("ThermalAPI_1")) {
        nodePrims[s->prim.index] = true;
        const double area = s->real("thermal:radiativeArea").value_or(0.0);
        if (!(area > 0.0)) {
            continue;
        }
        Target t;
        t.name = std::string(world.signalInstanceName(s->prim));
        bool ownPrim = false;
        const auto [body, pose] = bodyAndPose(world, s->prim, ownPrim);
        t.body = body;
        t.inBody = pose.position;
        t.colliders = collidersOf(world, body, pose, ownPrim);
        t.emissivity = s->real("thermal:emissivity").value_or(0.9);
        t.area = area;
        for (std::size_t k = 0; k < specs_.size(); ++k) {
            if (specs_[k].prim == s->prim) {
                t.fire = static_cast<int>(k);
            }
        }
        t.power = d.output(
            spec("fire_radiation/" + t.name + "/loss_power", SignalType::Scalar, units::W));
        need(body);
        targets_.push_back(std::move(t));
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const FireSpec& f = specs_[k];
        need(f.body);
        const std::string base = "fire/" + f.name + "/";
        Out& o = out_[k];
        o.flameHeight = d.output(spec(base + "flame_height", SignalType::Scalar, units::m));
        o.tilt = d.output(spec(base + "flame_tilt", SignalType::Vec3, units::one, "world_local"));
        o.base = d.output(spec(base + "base", SignalType::Vec3, units::m, "world_local"));
        o.radiative = d.output(spec(base + "radiative_power", SignalType::Scalar, units::W));
        o.fuel = d.output(spec(base + "fuel_remaining", SignalType::Scalar, units::kg));
        o.soot = d.output(spec(base + "soot_rate", SignalType::Scalar, kKgPerS));
        o.waterRate = d.output(spec(base + "water_rate", SignalType::Scalar, kKgPerM2S));
        o.cooling = d.output(spec(base + "water_cooling", SignalType::Scalar, units::W));
        o.steam = d.output(spec(base + "steam_rate", SignalType::Scalar, kKgPerS));
        o.foam = d.output(spec(base + "foam_coverage", SignalType::Scalar, units::one));
        o.hazard = d.output(spec(base + "hazard", SignalType::Bool, units::one));
        o.ignite = d.input(optionalInput(base + "ignite", SignalType::Bool, units::one, 0.0));
        o.downwash = d.input(
            optionalInput(base + "downwash", SignalType::Vec3, units::m_per_s, 0.0, "world_local"));
        o.waterSupply =
            d.input(optionalInput(base + "water_supply", SignalType::Scalar, kKgPerM2S, 0.0));
        o.foamSupply =
            d.input(optionalInput(base + "foam_supply", SignalType::Scalar, units::m_per_s, 0.0));
        if (f.ignitionTemperature && nodePrims.contains(f.prim.index)) {
            o.nodeTemperature = d.input(optionalInput("thermal/" + f.name + "/temperature",
                                                      SignalType::Scalar, units::K, 0.0));
            o.hasNodeTemperature = true;
        }
    }
    wind_ = d.input(
        optionalInput("env/wind/velocity", SignalType::Vec3, units::m_per_s, 0.0, "world_local"));
    airDensity_ = d.input(
        optionalInput("env/air_density", SignalType::Scalar, units::kg_per_m3, siteDensity_));
    airTemperature_ = d.input(
        optionalInput("env/air_temperature", SignalType::Scalar, units::K, siteTemperature_));
    rain_ = d.input(optionalInput("env/rain_rate", SignalType::Scalar, units::m_per_s, 0.0));
    warned_.assign(specs_.size(), std::vector<bool>(targets_.size(), false));
    std::vector<Vec3> bases;
    for (const FireSpec& f : specs_) {
        bases.push_back(pointIn(
            f.body == kWorldBody || f.body.index >= rest_.size() ? Pose{} : rest_[f.body.index],
            f.baseInBody));
    }
    smoke_.configure(smoke, bases);
    return {};
}

Status Fires::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    burners_.clear();
    flames_.assign(specs_.size(), Flame{});
    plumes_.assign(specs_.size(), PlumeSource{});
    for (const FireSpec& f : specs_) {
        burners_.emplace_back(f, extinguishFraction_);
        if (mode_ == Mode::Constant) {
            burners_.back().holdFull();
        }
    }
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        bus.write(out_[k].hrr, burners_[k].hrr());
        bus.write(out_[k].state, static_cast<double>(burners_[k].state()));
        if (mode_ != Mode::Zero) {
            const Vec3 base = pointIn(bodyPose(specs_[k].body, bus), specs_[k].baseInBody);
            writeOutputs(
                k, base, {0.0, 0.0, 1.0},
                flameHeight(burners_[k].hrr(), equivalentDiameter(burners_[k].burningArea())), bus);
        }
    }
    for (const Target& t : targets_) {
        bus.write(t.power, 0.0);
    }
    if (mode_ != Mode::Zero) {
        smoke_.clear();
    }
    bus.publishBuffer(smokeField_, fields::fieldBuffer(nullptr));
    bus.write(smokeMass_, 0.0);
    return {};
}

Pose Fires::bodyPose(BodyId body, const SignalBus& bus) const {
    if (body == kWorldBody) {
        return Pose{};
    }
    if (body.index < movingIndex_.size() && movingIndex_[body.index] >= 0) {
        const std::span<const double> p =
            bus.readN(poses_[static_cast<std::size_t>(movingIndex_[body.index])]);
        if (p.size() == 7) {
            return Pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
        }
    }
    return body.index < rest_.size() ? rest_[body.index] : Pose{};
}

bool Fires::blocked(const Vec3& from, const Vec3& to, const FireSpec& fire,
                    const Target& target) const {
    if (!scenery_) {
        return false;
    }
    const Vec3 d = sub(to, from);
    const double length = norm(d);
    if (!(length > 0.0)) {
        return false;
    }
    const Vec3 dir = scale(d, 1.0 / length);
    constexpr double kStepPast = 1e-4;  // m: past the face of a shape that does not count
    constexpr int kMostShapes = 16;
    double travelled = 0.0;
    for (int i = 0; i < kMostShapes; ++i) {
        const Vec3 origin = add(from, scale(dir, travelled));
        const services::RayHit hit = scenery_->ray(origin, dir, length - travelled);
        if (!hit.hit() || travelled + hit.distance >= length - 1e-6) {
            return false;
        }
        const std::uint32_t collider =
            hit.shape < sceneryColliders_.size() ? sceneryColliders_[hit.shape] : hit.shape;
        const auto own = [collider](const std::vector<std::uint32_t>& list) {
            return std::ranges::find(list, collider) != list.end();
        };
        if (!own(fire.colliders) && !own(target.colliders)) {
            return true;
        }
        travelled += hit.distance + kStepPast;
    }
    return false;
}

void Fires::writeOutputs(std::size_t k, const Vec3& base, const Vec3& axis, double height,
                         SignalBus& bus) const {
    const FireSpec& f = specs_[k];
    const Burner& b = burners_[k];
    const Out& o = out_[k];
    bus.write(o.hrr, b.hrr());
    bus.write(o.state, static_cast<double>(b.state()));
    bus.write(o.flameHeight, height);
    bus.writeN(o.tilt, axis);
    bus.writeN(o.base, base);
    bus.write(o.radiative, f.radiativeFraction * b.hrr());
    bus.write(o.fuel, b.fuelRemaining());
    bus.write(o.soot, f.sootYield * b.hrr() / f.heatOfCombustion);
    bus.write(o.waterRate, b.waterRate());
    bus.write(o.cooling, b.waterCooling());
    bus.write(o.steam, b.steamRate());
    bus.write(o.foam, b.foamCoverage());
    bus.write(o.hazard, b.hazard() ? 1.0 : 0.0);
}

Status Fires::step(const StepContext& context, SignalBus& bus) {
    if (mode_ == Mode::Zero) {
        for (const Out& o : out_) {
            bus.write(o.hrr, 0.0);
            bus.write(o.state, 0.0);
        }
        bus.write(smokeMass_, 0.0);
        return {};
    }
    const double t = context.t.seconds();
    const double dt = context.dt.seconds();
    const std::span<const double> windIn = bus.readN(wind_);
    const Vec3 wind =
        windIn.size() == 3 ? Vec3{windIn[0], windIn[1], windIn[2]} : Vec3{0.0, 0.0, 0.0};
    const double density = bus.read(airDensity_);
    const double airTemperature = bus.read(airTemperature_);
    const double rain = std::max(bus.read(rain_), 0.0) * kWaterDensity;
    const double g = norm(gravity_);
    // Flame source points this step, for the radiation below.
    std::vector<Flame>& flames = flames_;
    for (std::size_t k = 0; k < specs_.size(); ++k) {
        const FireSpec& f = specs_[k];
        Burner& b = burners_[k];
        const Out& o = out_[k];
        const Vec3 base = pointIn(bodyPose(f.body, bus), f.baseInBody);
        const std::span<const double> dw = bus.readN(o.downwash);
        const Vec3 air = dw.size() == 3 ? add(wind, Vec3{dw[0], dw[1], dw[2]}) : wind;
        if (mode_ == Mode::Simulate) {
            Exposure e;
            e.waterRate = bus.read(o.waterSupply) + rain;
            for (const Schedule& s : schedule_) {
                if (s.fire == k && t + 1e-9 >= s.start) {
                    e.waterRate += s.rate;
                }
            }
            e.foamSupply = bus.read(o.foamSupply);
            e.airSpeed = norm(air);
            e.ignite = bus.read(o.ignite) != 0.0;
            e.nodeTemperature = o.hasNodeTemperature ? bus.read(o.nodeTemperature) : 0.0;
            e.waterTemperature = airTemperature;
            b.step(t, dt, e);
        }
        // The flame: Heskestad's height over its equivalent diameter, leaning downwind (AGA).
        const double diameter = equivalentDiameter(b.burningArea());
        const double height = flameHeight(b.hrr(), diameter);
        const double horizontal = std::hypot(air[0], air[1]);
        Vec3 axis{0.0, 0.0, 1.0};
        if (mode_ == Mode::Simulate && horizontal > 0.0 && b.hrr() > 0.0) {
            const double c = tiltCosine(horizontal, b.massFlux(), diameter, g, density);
            const double sn = std::sqrt(std::max(1.0 - c * c, 0.0));
            axis = {sn * air[0] / horizontal, sn * air[1] / horizontal, c};
        }
        flames[k] = {add(base, scale(axis, 0.5 * height)), b.hrr(), diameter};
        writeOutputs(k, base, axis, height, bus);
        if (smokeOn_) {
            PlumeSource& plume = plumes_[k];
            plume.base = base;
            plume.tip = add(base, scale(axis, height));
            plume.hrr = b.hrr();
            plume.radiativeFraction = f.radiativeFraction;
            plume.virtualOriginHeight = virtualOrigin(b.hrr(), diameter);
            plume.sootRate = f.sootYield * b.hrr() / f.heatOfCombustion;
        }
    }
    if (smokeOn_) {
        stepSmoke(t, dt, wind, density, airTemperature, g, bus);
    }
    // Radiation into the thermal nodes in sight (point source, §3).
    for (std::size_t j = 0; j < targets_.size(); ++j) {
        const Target& target = targets_[j];
        const Vec3 at = pointIn(bodyPose(target.body, bus), target.inBody);
        double power = 0.0;
        for (std::size_t k = 0; k < specs_.size(); ++k) {
            const Flame& flame = flames[k];
            if (!(flame.hrr > 0.0) || target.fire == static_cast<int>(k)) {
                continue;
            }
            const double distance = norm(sub(at, flame.source));
            if (blocked(flame.source, at, specs_[k], target)) {
                continue;
            }
            if (distance < kNearFieldDiameters * flame.diameter && !warned_[k][j]) {
                warned_[k][j] = true;
                services::log(services::LogLevel::Warning, "fire",
                              "fire " + specs_[k].name + " is within two diameters of node " +
                                  target.name +
                                  ": the point-source model over-predicts its heat there");
            }
            power += absorbedByNode(
                target.emissivity,
                pointSourceFlux(specs_[k].radiativeFraction, flame.hrr, distance), target.area);
        }
        bus.write(target.power, power);
    }
    return {};
}

void Fires::stepSmoke(double t, double dt, const Vec3& wind, double density, double temperature,
                      double gravity, SignalBus& bus) {
    SmokeAir air;
    air.wind = wind;
    air.gravity = gravity;
    air.density = density;
    air.temperature = temperature;
    if (!(density > 0.0 && temperature > 0.0) && !smokeWarned_) {
        smokeWarned_ = true;
        services::log(services::LogLevel::Warning, "fire",
                      "no air density or ambient temperature in the run (--air-density, "
                      "--ambient-temperature or the weather): the smoke drifts but does not rise");
    }
    smoke_.step(dt, plumes_, air, scenery_.get());
    bus.write(smokeMass_, smoke_.mass());
    // The field at its own rate: when this step crosses a multiple of the period.
    constexpr double kTimeSlack = 1e-9;  // s, against rounding of the step's end
    if (std::floor((t + dt + kTimeSlack) / smokePeriod_) >
        std::floor((t + kTimeSlack) / smokePeriod_)) {
        const std::shared_ptr<const fields::Field> field = smoke_.raster();
        bus.publishBuffer(smokeField_, fields::fieldBuffer(field.get()));
    }
}

Status Fires::saveState(StateWriter& writer) const {
    std::vector<double> state;
    for (const Burner& b : burners_) {
        b.save(state);
    }
    writer.putDoubles(state);
    std::vector<double> smoke;
    smoke_.save(smoke);
    writer.put<std::uint64_t>(smoke.size());
    writer.putDoubles(smoke);
    return {};
}

Status Fires::restoreState(StateReader& reader) {
    std::vector<double> state(burners_.size() * 11);
    if (!reader.getDoubles(state)) {
        return Status::error(StatusCode::Internal, "fire: short state");
    }
    for (std::size_t k = 0; k < burners_.size(); ++k) {
        if (!burners_[k].restore(std::span<const double>(state).subspan(11 * k, 11))) {
            return Status::error(StatusCode::Internal, "fire: state does not fit");
        }
    }
    std::uint64_t size = 0;
    constexpr std::uint64_t kMostSmokeValues = 1ULL << 28;
    if (!reader.get(size) || size > kMostSmokeValues) {
        return Status::error(StatusCode::Internal, "fire: short state");
    }
    std::vector<double> smoke(size);
    if (!reader.getDoubles(smoke) || !smoke_.restore(smoke)) {
        return Status::error(StatusCode::Internal, "fire: smoke state does not fit");
    }
    return {};
}

}  // namespace lagrange::fire
