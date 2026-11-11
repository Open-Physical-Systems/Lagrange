#pragma once
// Private to lagrange_sensors: where the sun is and how much daylight reaches a sensor, for the
// light_sensor and sun_sensor kinds (sensors.md §9).

#include "EnvCommon.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <cmath>
#include <numbers>
#include <optional>
#include <string>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace sun_common {

/// Illustrative: the depth of a fog layer the direct beam crosses, per air mass (m). The weather
/// gives the extinction where the robot is, not the fog's height.
inline constexpr double kFogLayerDepth = 100.0;

/// The sun for one sensor. Its direction comes from the NOAA algorithm when the site's geo origin
/// and the sensor's `start_utc` (the UTC date and time at t = 0) are known, else from the weather's
/// env/sun_direction when it runs, else nowhere (outputs NaN, valid 0). The daylight is the IES
/// model: overcast while it rains or snows (env/rain_rate, env/snow_rate) or with `sky` overcast,
/// the direct beam dimmed by fog (env/extinction/visible over kFogLayerDepth per air mass).
class SunSource {
public:
    Status declare(const World& w, std::uint32_t sensor, const std::string& who,
                   SignalDeclarer& d) {
        const std::string start = kindText(w, sensor, "start_utc");
        if (!start.empty()) {
            start_ = parseUtc(start);
            if (!start_) {
                return Status::error(
                    StatusCode::InvalidConfig,
                    who + ": start_utc '" + start + "' is not YYYY-MM-DDTHH:MM:SSZ");
            }
        }
        if (start_ && w.geoOrigin) {
            origin_.emplace(*w.geoOrigin);
            region_ = w.region.worldEnu;
        }
        const std::string sky = kindText(w, sensor, "sky", "clear");
        if (sky != "clear" && sky != "overcast") {
            return Status::error(StatusCode::InvalidConfig,
                                 who + ": sky is clear or overcast, not '" + sky + "'");
        }
        overcast_ = sky == "overcast";
        fogDepth_ = kindSetting(w, sensor, "fog_depth", kFogLayerDepth);
        if (!origin_) {
            sunIn_ = d.input(
                optionalNaN("env/sun_direction", SignalType::Vec3, units::one, "world_local"));
        }
        rain_ = d.input(optionalNaN("env/rain_rate", SignalType::Scalar, units::m_per_s));
        snow_ = d.input(optionalNaN("env/snow_rate", SignalType::Scalar, units::m_per_s));
        fog_ = d.input(optionalNaN("env/extinction/visible", SignalType::Scalar, units::per_m));
        return {};
    }

    /// Unit vector to the sun (world_local, ENU axes) at `t` for a sensor at `position`, or NaN.
    [[nodiscard]] Vec3 direction(const SignalBus& bus, SimTime t, const Vec3& position) {
        if (origin_) {
            const coords::Geodetic g = origin_->toGeodetic(
                {position[0] + region_[0], position[1] + region_[1], position[2] + region_[2]});
            const double jd = *start_ + t.seconds() / kSecondsPerDay;
            const SolarPosition sun = solarPosition(jd, g.latitudeRad, g.longitudeRad);
            dayOfYear_ = dayOfYear(jd);
            return sun.enu();
        }
        if (!sunIn_.valid()) {
            return {kNaN, kNaN, kNaN};
        }
        const Vec3 v = readVec3(bus, sunIn_);
        const double n = norm(v);
        dayOfYear_ = kEquinoxDay;  // the weather gives no date
        return n > 0.0 && std::isfinite(n) ? Vec3{v[0] / n, v[1] / n, v[2] / n}
                                           : Vec3{kNaN, kNaN, kNaN};
    }

    /// The daylight for the sun at unit `sun` (z up), and the direct beam's transmission through
    /// fog.
    [[nodiscard]] Daylight daylight(const SignalBus& bus, const Vec3& sun,
                                    double& transmission) const {
        const double rain = bus.read(rain_);
        const double snow = bus.read(snow_);
        const bool overcast =
            overcast_ || (std::isfinite(rain) && rain > 0.0) || (std::isfinite(snow) && snow > 0.0);
        const double elevation = std::asin(std::clamp(sun[2], -1.0, 1.0));
        const Daylight light = iesDaylight(elevation, dayOfYear_, overcast);
        const double sigma = bus.read(fog_);
        transmission = 1.0;
        if (std::isfinite(sigma) && sigma > 0.0 && elevation > 0.0) {
            transmission =
                std::exp(-sigma * fogDepth_ * airMass(std::numbers::pi / 2.0 - elevation));
        }
        return light;
    }

private:
    static constexpr double kSecondsPerDay = 86400.0;
    static constexpr int kEquinoxDay = 80;  // about 21 March

    /// Day of the year (1 = 1 January) of Julian day `jd`, by Meeus ch. 7's inverse.
    static int dayOfYear(double jd) {
        const double z = std::floor(jd + 0.5);
        const double alpha = std::floor((z - 1867216.25) / 36524.25);
        const double a = z + 1.0 + alpha - std::floor(alpha / 4.0);
        const double b = a + 1524.0;
        const double c = std::floor((b - 122.1) / 365.25);
        const double dd = std::floor(365.25 * c);
        const double e = std::floor((b - dd) / 30.6001);
        const int day = static_cast<int>(b - dd - std::floor(30.6001 * e));
        const int month = static_cast<int>(e < 14.0 ? e - 1.0 : e - 13.0);
        const int year = static_cast<int>(month > 2 ? c - 4716.0 : c - 4715.0);
        const double first = julianDay(year, 1, 1, 0.0, 0.0, 0.0);
        const double that = julianDay(year, month, day, 0.0, 0.0, 0.0);
        return static_cast<int>(that - first) + 1;
    }

    std::optional<double> start_;
    std::optional<coords::GeoOrigin> origin_;
    Vec3 region_{};
    bool overcast_{false};
    double fogDepth_{kFogLayerDepth};
    int dayOfYear_{kEquinoxDay};
    SignalId sunIn_, rain_, snow_, fog_;
};

}  // namespace sun_common
}  // namespace lagrange::sensors::kinds
