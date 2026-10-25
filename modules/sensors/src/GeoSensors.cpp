// GNSS receiver, magnetometer and barometer (docs/design/domains/sensors.md, v0.4). They place
// the sensor on the Earth through the world's geo origin, so they need one; createGeoSensors
// leaves them out, with a log line, when the site has none.
#include "GeoSensors.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Wmm.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>

namespace lagrange::sensors {

namespace {

using Vec3 = std::array<double, 3>;

// ICAO standard atmosphere, troposphere (ICAO Doc 7488/3, "Manual of the ICAO Standard
// Atmosphere", 1993, table A and eqs. 7 and 12): pressure and temperature at sea level, the
// lapse rate, and the exponent g0·M/(R*·L).
constexpr double kSeaLevelPressurePa = 101325.0;
constexpr double kSeaLevelTemperatureK = 288.15;
constexpr double kLapseRateKPerM = 0.0065;
constexpr double kPressureExponent = 5.25588;  // 9.80665 · 0.0289644 / (8.31432 · 0.0065)
constexpr double kNanoTesla = 1e-9;            // T per nT

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

double setting(const World::Sensors& sensors, std::uint32_t sensor, const char* key) {
    const auto& map = sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? 0.0 : it->second;
}

/// A sensor that reads its body's pose (and twist) and knows where it is on the Earth.
class GeoSensor : public IModule {
public:
    GeoSensor(const World& world, std::uint32_t sensor, const char* kind, bool groundTruth)
        : groundTruth_(groundTruth)
        , sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , label_(std::string(kind) + "_" + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(Duration{static_cast<std::int64_t>(
              std::llround(1e9 / std::max(world.sensors.rateHz[sensor], 1e-3)))}) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", label_, 0, 1, groundTruth_, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

protected:
    /// Declares the body's pose (and twist) inputs; refuses a sensor on a body that never moves.
    Status declareBody(const WorldView& world, SignalDeclarer& d, bool twist) {
        const World& raw = world.raw();
        if (!movingBodies(raw)[body_]) {
            return Status::error(
                StatusCode::InvalidConfig,
                name_ + " is on '" + raw.bodies.names[body_] + "', which never moves");
        }
        if (!raw.geoOrigin) {
            return Status::error(StatusCode::InvalidConfig,
                                 name_ + " needs the site's geo origin (world.geo_origin)");
        }
        origin_.emplace(*raw.geoOrigin);
        region_ = raw.region.worldEnu;
        const std::string body = "body/" + raw.bodies.names[body_];
        pose_ = d.input(spec(body + "/pose", SignalType::Pose, units::m, "world_local"));
        if (twist) {
            twist_ =
                d.input(spec(body + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        }
        return {};
    }

    /// The sensor's world pose: position in world ENU, orientation (world from sensor).
    [[nodiscard]] Pose sensorPose(const SignalBus& bus) const {
        const std::span<const double> p = bus.readN(pose_);
        const Pose body{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
        Pose out = compose(body, mount_);
        for (std::size_t i = 0; i < 3; ++i) {
            out.position[i] += region_[i];
        }
        return out;
    }

    [[nodiscard]] coords::Geodetic geodetic(const Vec3& enu) const {
        return origin_->toGeodetic({enu[0], enu[1], enu[2]});
    }

    bool groundTruth_;
    std::uint32_t sensor_;
    std::string name_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
    std::optional<coords::GeoOrigin> origin_;
    Vec3 region_{};
    SignalId pose_, twist_;
};

/// GNSS receiver: WGS84 latitude, longitude and ellipsoidal height of the antenna, its velocity
/// in ENU, and a fix type (3: 3D). White noise per fix: "horizontal_noise" and "vertical_noise"
/// (m), "velocity_noise" (m/s). No multipath, outages or latency yet.
class Gnss final : public GeoSensor {
public:
    Gnss(const World& world, std::uint32_t sensor, bool groundTruth)
        : GeoSensor(world, sensor, "gnss", groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        if (Status s = declareBody(world, d, true); !s.ok()) {
            return s;
        }
        if (!groundTruth_) {
            const World::Sensors& sensors = world.sensors();
            horizontal_ = setting(sensors, sensor_, "horizontal_noise");
            vertical_ = setting(sensors, sensor_, "vertical_noise");
            velocityNoise_ = setting(sensors, sensor_, "velocity_noise");
        }
        const std::string base = "sensor/" + name_ + "/";
        latitude_ = d.output(spec(base + "latitude", SignalType::Scalar, units::rad));
        longitude_ = d.output(spec(base + "longitude", SignalType::Scalar, units::rad));
        altitude_ = d.output(spec(base + "altitude", SignalType::Scalar, units::m));
        velocity_ = d.output(spec(base + "velocity", SignalType::Vec3, units::m_per_s, "enu"));
        fix_ = d.output(spec(base + "fix", SignalType::Scalar, units::one));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, sensorPose(bus).position, Vec3{});
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose pose = sensorPose(bus);
        const std::span<const double> t = bus.readN(twist_);
        // The antenna's velocity: the body's plus ω × lever arm.
        const std::span<const double> p = bus.readN(pose_);
        const Vec3 arm = rotate({p[3], p[4], p[5], p[6]}, mount_.position);
        Vec3 velocity{t[0] + t[4] * arm[2] - t[5] * arm[1], t[1] + t[5] * arm[0] - t[3] * arm[2],
                      t[2] + t[3] * arm[1] - t[4] * arm[0]};
        Vec3 position = pose.position;
        random::Stream rng(context.seed, context.stepIndex, 0);
        position[0] += horizontal_ * rng.normal();
        position[1] += horizontal_ * rng.normal();
        position[2] += vertical_ * rng.normal();
        for (double& v : velocity) {
            v += velocityNoise_ * rng.normal();
        }
        write(bus, position, velocity);
        return {};
    }

private:
    void write(SignalBus& bus, const Vec3& enu, const Vec3& velocity) const {
        const coords::Geodetic where = geodetic(enu);
        bus.write(latitude_, where.latitudeRad);
        bus.write(longitude_, where.longitudeRad);
        bus.write(altitude_, where.heightM);
        bus.writeN(velocity_, velocity);
        bus.write(fix_, 3.0);
    }

    double horizontal_{0.0}, vertical_{0.0}, velocityNoise_{0.0};
    SignalId latitude_, longitude_, altitude_, velocity_, fix_;
};

/// Magnetometer: the World Magnetic Model 2025 main field at the sensor (Wmm.hpp), in the
/// sensor frame, T, with white noise "noise" (T) per axis, plus the EMI domain's disturbance
/// field when the sensor is a magnetometer victim (emi/<p>/b_field, sampled at the sensor's
/// rate, so a mains field aliases as it would). The date is the sensors config's
/// "decimal_year" (default 2025.0, the model's epoch). No crustal field or hard- and soft-iron
/// effects yet.
class Magnetometer final : public GeoSensor {
public:
    Magnetometer(const World& world, std::uint32_t sensor, bool groundTruth)
        : GeoSensor(world, sensor, "magnetometer", groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) override {
        if (Status s = declareBody(world, d, false); !s.ok()) {
            return s;
        }
        year_ = config.get<double>("decimal_year").value_or(kWmmValidFrom);
        if (year_ < kWmmValidFrom || year_ > kWmmValidUntil) {
            return Status::error(StatusCode::InvalidConfig,
                                 "the World Magnetic Model 2025 is valid from 2025.0 to 2030.0");
        }
        noise_ = groundTruth_ ? 0.0 : setting(world.sensors(), sensor_, "noise");
        if (const std::string victim = emiVictim(world, sensor_, "magnetometer");
            !victim.empty() && !groundTruth_) {
            disturbance_ =
                d.input(optionalSpec(victim + "b_field", SignalType::Vec3, units::T, "sensor"));
        }
        field_ = d.output(spec("sensor/" + name_ + "/field", SignalType::Vec3, units::T, "sensor"));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.writeN(field_, measure(sensorPose(bus), nullptr));
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        random::Stream rng(context.seed, context.stepIndex, 0);
        Vec3 field = measure(sensorPose(bus), &rng);
        if (disturbance_.valid()) {
            const std::span<const double> b = bus.readN(disturbance_);
            for (std::size_t i = 0; i < 3; ++i) {
                field[i] += b[i];
            }
        }
        bus.writeN(field_, field);
        return {};
    }

private:
    Vec3 measure(const Pose& pose, random::Stream* rng) const {
        // North, east, down (nT) to ENU (T), then into the sensor frame.
        const Vec3 northEastDown = wmmField(geodetic(pose.position), year_);
        const Vec3 enu{northEastDown[1] * kNanoTesla, northEastDown[0] * kNanoTesla,
                       -northEastDown[2] * kNanoTesla};
        Vec3 field = rotate(conjugate(pose.orientation), enu);
        if (rng != nullptr) {
            for (double& b : field) {
                b += noise_ * rng->normal();
            }
        }
        return field;
    }

    double year_{kWmmValidFrom};
    double noise_{0.0};
    SignalId field_, disturbance_;
};

/// Barometer: static pressure and air temperature of the ICAO standard atmosphere at the
/// sensor's height (troposphere, below 11 km), with white noise "noise" (Pa). The height above
/// the ellipsoid stands for the altitude above mean sea level (the geoid's undulation, tens of
/// metres, is not modelled), and the standard day for the weather until the weather domain
/// gives the site's (v0.5).
class Barometer final : public GeoSensor {
public:
    Barometer(const World& world, std::uint32_t sensor, bool groundTruth)
        : GeoSensor(world, sensor, "barometer", groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        if (Status s = declareBody(world, d, false); !s.ok()) {
            return s;
        }
        noise_ = groundTruth_ ? 0.0 : setting(world.sensors(), sensor_, "noise");
        const std::string base = "sensor/" + name_ + "/";
        pressure_ = d.output(spec(base + "pressure", SignalType::Scalar, units::Pa));
        temperature_ = d.output(spec(base + "temperature", SignalType::Scalar, units::K));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        random::Stream rng(context.seed, context.stepIndex, 0);
        write(bus, noise_ * rng.normal());
        return {};
    }

private:
    void write(SignalBus& bus, double noise) const {
        const double height = geodetic(sensorPose(bus).position).heightM;
        const Atmosphere air = standardAtmosphere(height);
        bus.write(pressure_, air.pressurePa + noise);
        bus.write(temperature_, air.temperatureK);
    }

    double noise_{0.0};
    SignalId pressure_, temperature_;
};

}  // namespace

Atmosphere standardAtmosphere(double altitudeM) {
    const double temperature = kSeaLevelTemperatureK - kLapseRateKPerM * altitudeM;
    return {kSeaLevelPressurePa * std::pow(temperature / kSeaLevelTemperatureK, kPressureExponent),
            temperature};
}

SignalSpec optionalSpec(std::string name, SignalType type, Unit unit, std::string frame) {
    SignalSpec s = spec(std::move(name), type, unit, std::move(frame));
    s.optional = true;
    return s;
}

std::string emiVictim(const WorldView& world, std::uint32_t sensor, std::string_view victimType) {
    const PrimId prim = world.sensors().prim[sensor];
    for (const SchemaInstance* s : world.instancesOf("EmiSusceptibilityAPI_1")) {
        if (s->prim == prim && s->token("emi:victimType") == victimType) {
            return "emi/" + std::string(world.signalInstanceName(prim)) + "/";
        }
    }
    return {};
}

void createGeoSensors(const World& world, bool groundTruth,
                      std::vector<std::unique_ptr<IModule>>& modules) {
    bool skipped = false;
    for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
        const SensorType type = world.sensors.type[s];
        if (type != SensorType::Gnss && type != SensorType::Magnetometer &&
            type != SensorType::Barometer) {
            continue;
        }
        if (!world.geoOrigin) {
            skipped = true;
            continue;
        }
        if (type == SensorType::Gnss) {
            modules.push_back(std::make_unique<Gnss>(world, s, groundTruth));
        } else if (type == SensorType::Magnetometer) {
            modules.push_back(std::make_unique<Magnetometer>(world, s, groundTruth));
        } else {
            modules.push_back(std::make_unique<Barometer>(world, s, groundTruth));
        }
    }
    if (skipped) {
        services::log(services::LogLevel::Warning, "sensors",
                      "GNSS, magnetometers and barometers do not run without the site's geo "
                      "origin: the profile's world.geo_origin or --geo-origin LAT,LON,HEIGHT");
    }
}

}  // namespace lagrange::sensors
