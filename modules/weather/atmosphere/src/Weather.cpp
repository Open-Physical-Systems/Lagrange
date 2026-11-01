// The weather domain: the atmosphere and its extinction (Weather.hpp, domains/weather.md).
#include "lagrange/weather/Weather.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::weather {

using namespace literals;

namespace {

constexpr double kCelsiusZero = 273.15;  // K
constexpr double kDryAirGas = 287.05;    // J/(kg·K), dry air
constexpr double kVapourGas = 461.5;     // J/(kg·K), water vapour
constexpr double kKoschmieder = 3.912;   // −ln(0.02): Koschmieder's 2 % contrast threshold
constexpr double kMetresPerKm = 1000.0;
// Marshall–Palmer's slope: Λ = 4.1 mm⁻¹ · R^−0.21, R in mm/h.
constexpr double kSlopePerMm = 4.1;
constexpr double kSlopeExponent = -0.21;
constexpr double kMpsPerMmph = 1e-3 / 3600.0;  // m/s per mm/h

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

}  // namespace

double saturationVapourPressure(double kelvin) {
    const double t = kelvin - kCelsiusZero;
    return 610.94 * std::exp(17.625 * t / (t + 243.04));
}

double moistAirDensity(double kelvin, double pressurePa, double relativeHumidity) {
    const double vapour = std::clamp(relativeHumidity, 0.0, 1.0) * saturationVapourPressure(kelvin);
    return (pressurePa - vapour) / (kDryAirGas * kelvin) + vapour / (kVapourGas * kelvin);
}

double fogExtinctionVisible(double visibilityM) {
    return visibilityM > 0.0 ? kKoschmieder / visibilityM : 0.0;
}

double kimExponent(double visibilityM) {
    const double v = visibilityM / kMetresPerKm;  // km
    if (v > 50.0) {
        return 1.6;
    }
    if (v > 6.0) {
        return 1.3;
    }
    if (v > 1.0) {
        return 0.16 * v + 0.34;
    }
    if (v > 0.5) {
        return v - 0.5;
    }
    return 0.0;
}

double fogExtinction(double visibilityM, double wavelengthM) {
    if (visibilityM <= 0.0) {
        return 0.0;
    }
    return fogExtinctionVisible(visibilityM) *
           std::pow(wavelengthM / kVisibleWavelength, -kimExponent(visibilityM));
}

double marshallPalmerSlope(double rainRateMps) {
    const double mmph = rainRateMps / kMpsPerMmph;
    return kSlopePerMm * std::pow(mmph, kSlopeExponent) * 1000.0;  // 1/mm → 1/m
}

double rainExtinction(double rainRateMps) {
    if (rainRateMps <= 0.0) {
        return 0.0;
    }
    const double slope = marshallPalmerSlope(rainRateMps);
    return std::numbers::pi * kMarshallPalmerN0 / (slope * slope * slope);
}

double snowExtinction(double snowRateMps) {
    return snowRateMps <= 0.0 ? 0.0 : kSnowExtinctionPerMmph * snowRateMps / kMpsPerMmph;
}

double frictionScale(double wetness, double snowCover, double temperatureK) {
    const double wet = temperatureK <= kFreezing ? kIceFrictionScale : kWetFrictionScale;
    const double bare = 1.0 - (wetness * (1.0 - wet));
    return bare + (snowCover * (std::min(kSnowFrictionScale, bare) - bare));
}

double dustExtinction(double concentrationKgPerM3, double medianDiameterM) {
    if (concentrationKgPerM3 <= 0.0 || medianDiameterM <= 0.0) {
        return 0.0;
    }
    return 3.0 * concentrationKgPerM3 / (kDustParticleDensity * medianDiameterM);
}

namespace {

const SchemaInstance* weatherPrim(const WorldView& world) {
    const auto instances = world.instancesOf("WeatherAPI_1");
    return instances.empty() ? nullptr : instances.front();
}

std::optional<double> site(const WorldView& world, const ModuleConfig& config, const char* key,
                           const char* attribute) {
    if (const auto v = config.get<double>(key)) {
        return v;
    }
    if (const SchemaInstance* w = weatherPrim(world)) {
        return w->real(attribute);
    }
    return std::nullopt;
}

}  // namespace

bool Weather::hasSite(const WorldView& world, const ModuleConfig& config) {
    return site(world, config, "temperature", "weather:ambientTemperature") &&
           site(world, config, "pressure", "weather:pressure") &&
           site(world, config, "relative_humidity", "weather:relativeHumidity");
}

ModuleInfo Weather::info() const {
    return clear_ ? ModuleInfo{"weather", "clear", 0, 1, true, true, CapabilityTier::Tier0}
                  : ModuleInfo{"weather", "atmosphere", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Weather::preferredStep() const {
    return 50_ms;
}

Status Weather::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    const auto temperature = site(world, config, "temperature", "weather:ambientTemperature");
    const auto pressure = site(world, config, "pressure", "weather:pressure");
    const auto humidity = site(world, config, "relative_humidity", "weather:relativeHumidity");
    if (!temperature || !pressure || !humidity || *temperature <= 0.0 || *pressure <= 0.0 ||
        *humidity < 0.0 || *humidity > 1.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the weather needs the site's air temperature (K), pressure (Pa) and "
                             "relative humidity (0 to 1): the profile's weather keys or a "
                             "WeatherAPI_1 prim; they have no defaults");
    }
    s_ = State{};
    s_.temperature = *temperature;
    s_.pressure = *pressure;
    s_.humidity = *humidity;
    s_.skyTemperature =
        site(world, config, "sky_temperature", "weather:skyTemperature").value_or(s_.temperature);
    const SchemaInstance* prim = weatherPrim(world);
    const auto value = [&](const char* key, const char* attribute, double fallback) {
        if (const auto v = config.get<double>(key)) {
            return *v;
        }
        return prim != nullptr ? prim->real(attribute).value_or(fallback) : fallback;
    };
    if (!clear_) {
        s_.visibility = value("visibility", "weather:visibility", 0.0);
        s_.rainRate = std::max(0.0, value("rain_rate", "weather:rainRate", 0.0));
        snowRate_ = std::max(0.0, config.get<double>("snow_rate").value_or(0.0));
        startCover_ = std::clamp(config.get<double>("snow_cover").value_or(0.0), 0.0, 1.0);
        s_.dustConcentration =
            std::max(0.0, value("dust_concentration", "weather:dustConcentration", 0.0));
        s_.dustDiameter = value("dust_median_diameter", "weather:dustMedianDiameter", 1e-5);
        if (prim != nullptr) {
            if (const auto w = prim->realArray("weather:meanWind"); w.size() == 3) {
                s_.wind = {w[0], w[1], w[2]};
            }
        }
    }
    s_.irradiance = value("solar_irradiance", "weather:solarIrradiance", 0.0);
    if (prim != nullptr) {
        if (const auto v = prim->realArray("weather:sunDirection"); v.size() == 3) {
            s_.sun = {v[0], v[1], v[2]};
        }
    }
    if (const auto effects = config.get<std::vector<std::string>>("effects")) {
        const auto has = [&](const char* e) { return std::ranges::count(*effects, e) > 0; };
        fog_ = has("fog");
        rain_ = has("rain");
        snow_ = has("snow");
        dust_ = has("dust");
        wetFriction_ = has("wet_friction");
        lens_ = has("lens_contamination");
    }

    temperatureOut_ = d.output(spec("env/air_temperature", SignalType::Scalar, units::K));
    skyOut_ = d.output(spec("env/sky_temperature", SignalType::Scalar, units::K));
    pressureOut_ = d.output(spec("env/pressure", SignalType::Scalar, units::Pa));
    humidityOut_ = d.output(spec("env/relative_humidity", SignalType::Scalar, units::one));
    densityOut_ = d.output(spec("env/air_density", SignalType::Scalar, units::kg_per_m3));
    windOut_ = d.output(spec("env/wind/mean", SignalType::Vec3, units::m_per_s, "world_local"));
    irradianceOut_ = d.output(spec("env/solar_irradiance", SignalType::Scalar, units::W_per_m2));
    sunOut_ = d.output(spec("env/sun_direction", SignalType::Vec3, units::one, "world_local"));
    visibleOut_ = d.output(spec("env/extinction/visible", SignalType::Scalar, units::per_m));
    nir905Out_ = d.output(spec("env/extinction/nir_905", SignalType::Scalar, units::per_m));
    nir1550Out_ = d.output(spec("env/extinction/nir_1550", SignalType::Scalar, units::per_m));
    fog905Out_ = d.output(spec("env/extinction/fog_905", SignalType::Scalar, units::per_m));
    fog1550Out_ = d.output(spec("env/extinction/fog_1550", SignalType::Scalar, units::per_m));
    rainOut_ = d.output(spec("env/rain_rate", SignalType::Scalar, units::m_per_s));
    frictionOut_ = d.output(spec("env/friction_scale", SignalType::Scalar, units::one));
    snowRateOut_ = d.output(spec("env/snow_rate", SignalType::Scalar, units::m_per_s));
    snowCoverOut_ = d.output(spec("env/snow_cover", SignalType::Scalar, units::one));
    // Fields: a fog grid (extinction, 1/m, scalar) and a wind grid (m/s, vector).
    const auto loadField = [&](const char* key, const char* attribute, fields::Field::Kind kind,
                               std::shared_ptr<const fields::Field>& out) -> Status {
        std::string path = config.get<std::string>(key).value_or("");
        if (path.empty() && prim != nullptr) {
            path = std::string(prim->assetPath(attribute).value_or(""));
        }
        out.reset();
        if (path.empty() || clear_) {
            return {};
        }
        auto loaded = fields::Field::load(path, world.raw().region.worldEnu);
        if (!loaded) {
            return Status::error(StatusCode::InvalidConfig, loaded.error());
        }
        if ((*loaded)->kind() != kind) {
            return Status::error(StatusCode::InvalidConfig,
                                 path + (kind == fields::Field::Kind::Scalar
                                             ? ": a fog field holds one value per voxel"
                                             : ": a wind field holds a vector per voxel"));
        }
        out = *loaded;
        return {};
    };
    if (Status s =
            loadField("fog_field", "weather:fogField", fields::Field::Kind::Scalar, fogField_);
        !s.ok()) {
        return s;
    }
    if (Status s =
            loadField("wind_field", "weather:windField", fields::Field::Kind::Vector, windField_);
        !s.ok()) {
        return s;
    }
    fogFieldOut_ = d.output(spec("env/extinction/field", SignalType::Field3D, units::per_m));
    windFieldOut_ = d.output(
        spec("env/wind/velocity_field", SignalType::Field3D, units::m_per_s, "world_local"));
    contaminationOut_.clear();
    for (const std::string& name : world.sensors().names) {
        contaminationOut_.push_back(
            d.output(spec("sensor/" + name + "/contamination", SignalType::Scalar, units::one)));
    }
    return {};
}

Status Weather::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    wetness_ = 0.0;
    snowCover_ = snow_ ? startCover_ : 0.0;
    contamination_.assign(contaminationOut_.size(), 0.0);
    bus.publishBuffer(fogFieldOut_, fields::fieldBuffer(fogField_.get()));
    bus.publishBuffer(windFieldOut_, fields::fieldBuffer(windField_.get()));
    writeAll(bus);
    return {};
}

Status Weather::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    if (wetFriction_ && s_.rainRate > 0.0) {
        // Wetting towards 1, faster in heavier rain up to kWettingRainRate.
        const double rate = std::min(s_.rainRate / kWettingRainRate, 1.0) / kWettingTimeS;
        wetness_ = 1.0 - (1.0 - wetness_) * std::exp(-rate * dt);
    } else {
        wetness_ *= std::exp(-dt / kDryingTimeS);
    }
    // Snow builds up below freezing, about covered after 10 minutes at 1 mm/h, and melts above it
    // in half an hour (behavioural, illustrative).
    constexpr double kCoverTimeS = 600.0;
    constexpr double kMeltTimeS = 1800.0;
    const double snowing = snow_ ? snowRate_ : 0.0;
    if (s_.temperature <= kFreezing) {
        snowCover_ =
            1.0 - ((1.0 - snowCover_) * std::exp(-(snowing / kMpsPerMmph) / kCoverTimeS * dt));
    } else {
        snowCover_ *= std::exp(-dt / kMeltTimeS);
    }
    if (lens_) {
        const double rate =
            ((rain_ ? s_.rainRate : 0.0) + snowing) / kLensRainReference / kLensRainTimeS +
            (dust_ ? s_.dustConcentration / kLensDustReference / kLensDustTimeS : 0.0);
        for (double& c : contamination_) {
            c = 1.0 - (1.0 - c) * std::exp(-rate * dt);
        }
    }
    writeAll(bus);
    return {};
}

void Weather::writeAll(SignalBus& bus) const {
    bus.write(temperatureOut_, s_.temperature);
    bus.write(skyOut_, s_.skyTemperature);
    bus.write(pressureOut_, s_.pressure);
    bus.write(humidityOut_, s_.humidity);
    bus.write(densityOut_, moistAirDensity(s_.temperature, s_.pressure, s_.humidity));
    bus.writeN(windOut_, s_.wind);
    bus.write(irradianceOut_, s_.irradiance);
    bus.writeN(sunOut_, s_.sun);
    const double rain = rain_ ? rainExtinction(s_.rainRate) : 0.0;
    const double dust = dust_ ? dustExtinction(s_.dustConcentration, s_.dustDiameter) : 0.0;
    const auto total = [&](double wavelength) {
        return (fog_ ? fogExtinction(s_.visibility, wavelength) : 0.0) + rain + dust +
               (snow_ ? snowExtinction(snowRate_) : 0.0);
    };
    bus.write(visibleOut_, total(kVisibleWavelength));
    bus.write(nir905Out_, total(kLidar905));
    bus.write(nir1550Out_, total(kLidar1550));
    bus.write(fog905Out_, fog_ ? fogExtinction(s_.visibility, kLidar905) : 0.0);
    bus.write(fog1550Out_, fog_ ? fogExtinction(s_.visibility, kLidar1550) : 0.0);
    bus.write(rainOut_, rain_ ? s_.rainRate : 0.0);
    bus.write(frictionOut_, frictionScale(wetness_, snowCover_, s_.temperature));
    bus.write(snowRateOut_, snow_ ? snowRate_ : 0.0);
    bus.write(snowCoverOut_, snowCover_);
    for (std::size_t i = 0; i < contaminationOut_.size(); ++i) {
        bus.write(contaminationOut_[i], contamination_[i]);
    }
}

Status Weather::saveState(StateWriter& writer) const {
    writer.put(wetness_);
    writer.put(snowCover_);
    for (const double c : contamination_) {
        writer.put(c);
    }
    return {};
}

Status Weather::restoreState(StateReader& reader) {
    bool ok = reader.get(wetness_) && reader.get(snowCover_);
    for (double& c : contamination_) {
        ok = ok && reader.get(c);
    }
    return ok ? Status{} : Status::error(StatusCode::InvalidConfig, "weather state truncated");
}

}  // namespace lagrange::weather
