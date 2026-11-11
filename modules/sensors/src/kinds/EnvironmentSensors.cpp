// The closed forms behind the environment sensor kinds (EnvironmentSensors.hpp).
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

namespace lagrange::sensors {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;
constexpr double kJ2000 = 2451545.0;         // Julian day of J2000.0
constexpr double kDaysPerCentury = 36525.0;  // Julian century
constexpr double kMinutesPerDay = 1440.0;
constexpr double kDaysPerYear = 365.0;       // IES 1984's eccentricity term
constexpr double kEccentricityTerm = 0.033;  // IES 1984: 1 + 0.033·cos(2π·day/365)

double wrapDegrees(double x) {
    x = std::fmod(x, 360.0);
    return x < 0.0 ? x + 360.0 : x;
}

/// NOAA's atmospheric refraction (degrees) at geometric elevation `e` (degrees), the Solar
/// Calculator's piecewise fit (after Meeus and the Explanatory Supplement), for a standard
/// atmosphere.
double noaaRefraction(double e) {
    if (e > 85.0) {
        return 0.0;
    }
    const double te = std::tan(e * kDeg);
    double arcsec = 0.0;
    if (e > 5.0) {
        arcsec = 58.1 / te - 0.07 / (te * te * te) + 0.000086 / std::pow(te, 5);
    } else if (e > -0.575) {
        arcsec = 1735.0 + e * (-518.2 + e * (103.4 + e * (-12.79 + e * 0.711)));
    } else {
        arcsec = -20.772 / te;
    }
    return arcsec / 3600.0;
}

bool number(std::string_view text, std::size_t at, std::size_t length, int& out) {
    if (at + length > text.size()) {
        return false;
    }
    const char* first = text.data() + at;
    const auto [end, error] = std::from_chars(first, first + length, out);
    return error == std::errc{} && end == first + length;
}

}  // namespace

double pitotPressure(double density, double flowAlongProbe) {
    return 0.5 * density * flowAlongProbe * std::abs(flowAlongProbe);
}

double airspeedFromPressure(double q, double density) {
    if (!(density > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::copysign(std::sqrt(2.0 * std::abs(q) / density), q);
}

double windFromDirection(double vx, double vy) {
    if (vx == 0.0 && vy == 0.0) {
        return 0.0;
    }
    const double a = std::atan2(vy, -vx);
    return a < 0.0 ? a + 2.0 * kPi : a;
}

double meteorologicalOpticalRange(double extinction) {
    if (!(extinction > 0.0)) {
        return std::numeric_limits<double>::infinity();
    }
    return -std::log(kMorContrast) / extinction;
}

double firstOrderStep(double state, double input, double dt, double tau) {
    if (!(tau > 0.0) || !std::isfinite(state)) {
        return input;
    }
    return state + (1.0 - std::exp(-dt / tau)) * (input - state);
}

double julianDay(int year, int month, int day, double hour, double minute, double second) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    const int a = year / 100;
    const int b = 2 - a + a / 4;  // Gregorian calendar
    const double dayFraction = day + (hour + (minute + second / 60.0) / 60.0) / 24.0;
    return std::floor(365.25 * (year + 4716)) + std::floor(30.6001 * (month + 1)) + dayFraction +
           b - 1524.5;
}

std::optional<double> parseUtc(std::string_view text) {
    // YYYY-MM-DDTHH:MM[:SS[.s]][Z]
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, whole = 0;
    if (text.size() < 16 || text[4] != '-' || text[7] != '-' ||
        (text[10] != 'T' && text[10] != ' ') || text[13] != ':' || !number(text, 0, 4, year) ||
        !number(text, 5, 2, month) || !number(text, 8, 2, day) || !number(text, 11, 2, hour) ||
        !number(text, 14, 2, minute)) {
        return std::nullopt;
    }
    std::string_view rest = text.substr(16);
    double second = 0.0;
    if (!rest.empty() && rest.front() == ':') {
        if (!number(rest, 1, 2, whole)) {
            return std::nullopt;
        }
        second = whole;
        rest = rest.substr(3);
        if (!rest.empty() && rest.front() == '.') {
            std::size_t digits = 1;
            while (digits < rest.size() && rest[digits] >= '0' && rest[digits] <= '9') {
                ++digits;
            }
            const std::string fraction("0" + std::string(rest.substr(0, digits)));
            second += std::stod(fraction);
            rest = rest.substr(digits);
        }
    }
    if (rest == "Z") {
        rest = {};
    }
    if (!rest.empty() || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 ||
        minute > 59 || second >= 61.0) {
        return std::nullopt;
    }
    return julianDay(year, month, day, hour, minute, second);
}

std::array<double, 3> SolarPosition::enu() const {
    const double c = std::cos(elevation);
    return {c * std::sin(azimuth), c * std::cos(azimuth), std::sin(elevation)};
}

SolarPosition solarPosition(double jd, double latitudeRad, double longitudeRad) {
    // NOAA Solar Calculator ("General Solar Position Calculations", NOAA GML), in degrees.
    const double t = (jd - kJ2000) / kDaysPerCentury;
    const double l0 = wrapDegrees(280.46646 + t * (36000.76983 + t * 0.0003032));
    const double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
    const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
    const double mr = m * kDeg;
    const double centre = std::sin(mr) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                          std::sin(2.0 * mr) * (0.019993 - 0.000101 * t) +
                          std::sin(3.0 * mr) * 0.000289;
    const double trueLongitude = l0 + centre;
    const double omega = 125.04 - 1934.136 * t;
    const double lambda = trueLongitude - 0.00569 - 0.00478 * std::sin(omega * kDeg);
    const double obliquityMean =
        23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
    const double obliquity = obliquityMean + 0.00256 * std::cos(omega * kDeg);
    const double declination = std::asin(std::sin(obliquity * kDeg) * std::sin(lambda * kDeg));
    const double y = std::pow(std::tan(obliquity * kDeg / 2.0), 2);
    const double l0r = l0 * kDeg;
    const double eot = 4.0 / kDeg *
                       (y * std::sin(2.0 * l0r) - 2.0 * e * std::sin(mr) +
                        4.0 * e * y * std::sin(mr) * std::cos(2.0 * l0r) -
                        0.5 * y * y * std::sin(4.0 * l0r) - 1.25 * e * e * std::sin(2.0 * mr));
    // Minutes of the UT day, then true solar time and the hour angle.
    const double dayFraction = (jd + 0.5) - std::floor(jd + 0.5);
    const double trueSolarTime =
        std::fmod(dayFraction * kMinutesPerDay + eot + 4.0 * longitudeRad / kDeg, kMinutesPerDay);
    const double hourAngle = (trueSolarTime / 4.0 - 180.0) * kDeg;
    const double cosZenith =
        std::clamp(std::sin(latitudeRad) * std::sin(declination) +
                       std::cos(latitudeRad) * std::cos(declination) * std::cos(hourAngle),
                   -1.0, 1.0);
    const double elevation = kPi / 2.0 - std::acos(cosZenith);
    double azimuth =
        std::atan2(std::sin(hourAngle), std::cos(hourAngle) * std::sin(latitudeRad) -
                                            std::tan(declination) * std::cos(latitudeRad)) +
        kPi;
    azimuth = std::fmod(azimuth, 2.0 * kPi);
    SolarPosition out;
    out.elevation = elevation + noaaRefraction(elevation / kDeg) * kDeg;
    out.azimuth = azimuth;
    out.declination = declination;
    out.equationOfTime = eot;
    return out;
}

double airMass(double zenith) {
    // Kasten and Young (1989): a = 0.50572, b = 6.07995°, c = 1.6364.
    const double z = std::min(zenith / kDeg, 90.0);
    return 1.0 / (std::cos(z * kDeg) + 0.50572 * std::pow(96.07995 - z, -1.6364));
}

Daylight iesDaylight(double elevation, int dayOfYear, bool overcast) {
    Daylight out;
    if (!(elevation > 0.0)) {
        return out;
    }
    const double s = std::sin(elevation);
    if (overcast) {
        out.diffuseHorizontal = (0.3 + 21.0 * s) * 1e3;  // IES 1984, overcast sky
        return out;
    }
    const double extraterrestrial =
        kExtraterrestrialIlluminance *
        (1.0 + kEccentricityTerm * std::cos(2.0 * kPi * dayOfYear / kDaysPerYear));
    out.directNormal =
        extraterrestrial * std::exp(-kClearSkyExtinction * airMass(kPi / 2.0 - elevation));
    out.diffuseHorizontal = (0.8 + 15.5 * std::sqrt(s)) * 1e3;  // IES 1984, clear sky
    return out;
}

double surfaceIlluminance(const Daylight& light, const std::array<double, 3>& sun,
                          const std::array<double, 3>& normal, bool sunBlocked,
                          double beamTransmission, double groundReflectance) {
    const double incidence =
        std::max(0.0, sun[0] * normal[0] + sun[1] * normal[1] + sun[2] * normal[2]);
    const double beam = light.directNormal * beamTransmission;
    const double direct = sunBlocked ? 0.0 : beam * incidence;
    const double sky = light.diffuseHorizontal * (1.0 + normal[2]) / 2.0;
    const double globalHorizontal = light.diffuseHorizontal + beam * std::max(0.0, sun[2]);
    const double ground = groundReflectance * globalHorizontal * (1.0 - normal[2]) / 2.0;
    return direct + sky + ground;
}

}  // namespace lagrange::sensors
