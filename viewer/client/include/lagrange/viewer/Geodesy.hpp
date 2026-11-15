#pragma once
// WGS84 <-> local ENU for the World workspace's Geodesy tab, header only.
//
// The viewer's client and panels do not link the simulation core (invariant 14), so this is the
// same math as core/world/coords (Frames.cpp), written again: the server sends the geo origin and
// positions in world ENU, and the viewer turns them into latitude, longitude and height.
// test_geodesy checks it against core's coords to the last digits.

#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

namespace lagrange::viewer::geodesy {

/// WGS84 ellipsoid (NIMA TR8350.2, "Department of Defense World Geodetic System 1984",
/// third edition, table 3.1); the same constants as lagrange::coords::wgs84.
inline constexpr double kSemiMajorAxisM = 6378137.0;         // a, m
inline constexpr double kInverseFlattening = 298.257223563;  // 1/f
inline constexpr double kFlattening = 1.0 / kInverseFlattening;
inline constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);  // e^2

inline constexpr double kDegree = std::numbers::pi / 180.0;  // rad per degree

/// Latitude and longitude in degrees, height above the ellipsoid (not the geoid) in metres.
struct Geodetic {
    double latitudeDeg{0.0};
    double longitudeDeg{0.0};
    double heightM{0.0};
};

/// Earth-centred, Earth-fixed, metres.
struct Ecef {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/// East, north, up from the geo origin, metres: the world ENU frame of WorldState.
struct Enu {
    double e{0.0};
    double n{0.0};
    double u{0.0};
};

[[nodiscard]] inline Ecef toEcef(const Geodetic& point) {
    const double latitude = point.latitudeDeg * kDegree;
    const double longitude = point.longitudeDeg * kDegree;
    const double sinLat = std::sin(latitude);
    const double cosLat = std::cos(latitude);
    // Prime vertical radius of curvature.
    const double n = kSemiMajorAxisM / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);
    const double r = (n + point.heightM) * cosLat;
    return {r * std::cos(longitude), r * std::sin(longitude),
            (n * (1.0 - kEccentricitySquared) + point.heightM) * sinLat};
}

/// Exact closed form of Vermeille, "Direct transformation from geocentric coordinates to
/// geodetic coordinates", Journal of Geodesy 76 (2002) 451-454, section 2. Valid farther than
/// about 43 km from the Earth's centre.
[[nodiscard]] inline Geodetic toGeodetic(const Ecef& point) {
    const double e2 = kEccentricitySquared;
    const double e4 = e2 * e2;
    const double a2 = kSemiMajorAxisM * kSemiMajorAxisM;
    const double horizontal2 = point.x * point.x + point.y * point.y;
    const double p = horizontal2 / a2;
    const double q = (1.0 - e2) / a2 * point.z * point.z;
    const double r = (p + q - e4) / 6.0;
    const double s = e4 * p * q / (4.0 * r * r * r);
    const double t = std::cbrt(1.0 + s + std::sqrt(s * (2.0 + s)));
    const double u = r * (1.0 + t + 1.0 / t);
    const double v = std::sqrt(u * u + e4 * q);
    const double w = e2 * (u + v - q) / (2.0 * v);
    const double k = std::sqrt(u + v + w * w) - w;
    const double horizontal = std::sqrt(horizontal2);
    const double d = k * horizontal / (k + e2);
    const double dz = std::sqrt(d * d + point.z * point.z);
    return {2.0 * std::atan2(point.z, d + dz) / kDegree, std::atan2(point.y, point.x) / kDegree,
            (k + e2 - 1.0) / k * dz};
}

/// The tangent plane a world is anchored to (WorldState.geo_origin).
class Origin {
public:
    explicit Origin(const Geodetic& origin) : origin_(origin), ecef_(geodesy::toEcef(origin)) {
        const double sinLat = std::sin(origin.latitudeDeg * kDegree);
        const double cosLat = std::cos(origin.latitudeDeg * kDegree);
        const double sinLon = std::sin(origin.longitudeDeg * kDegree);
        const double cosLon = std::cos(origin.longitudeDeg * kDegree);
        rotation_ = {-sinLon,          cosLon,           0.0,      // east
                     -sinLat * cosLon, -sinLat * sinLon, cosLat,   // north
                     cosLat * cosLon,  cosLat * sinLon,  sinLat};  // up
    }

    [[nodiscard]] const Geodetic& geodetic() const { return origin_; }
    [[nodiscard]] const Ecef& ecef() const { return ecef_; }

    [[nodiscard]] Ecef toEcef(const Enu& point) const {
        const std::array<double, 9>& m = rotation_;
        return {ecef_.x + m[0] * point.e + m[3] * point.n + m[6] * point.u,
                ecef_.y + m[1] * point.e + m[4] * point.n + m[7] * point.u,
                ecef_.z + m[2] * point.e + m[5] * point.n + m[8] * point.u};
    }

    [[nodiscard]] Enu fromEcef(const Ecef& point) const {
        const std::array<double, 9>& m = rotation_;
        const double dx = point.x - ecef_.x;
        const double dy = point.y - ecef_.y;
        const double dz = point.z - ecef_.z;
        return {m[0] * dx + m[1] * dy + m[2] * dz, m[3] * dx + m[4] * dy + m[5] * dz,
                m[6] * dx + m[7] * dy + m[8] * dz};
    }

    [[nodiscard]] Geodetic toGeodetic(const Enu& point) const {
        return geodesy::toGeodetic(toEcef(point));
    }
    [[nodiscard]] Enu fromGeodetic(const Geodetic& point) const {
        return fromEcef(geodesy::toEcef(point));
    }

    /// How far the flat world (the tangent plane at the origin's height) is above the
    /// ellipsoid surface at the origin's height, `e` east and `n` north of the origin: what a
    /// flat world gets wrong about the Earth's curve there, metres (about d^2 / 2R).
    [[nodiscard]] double planeRiseM(double e, double n) const {
        return toGeodetic(Enu{e, n, 0.0}).heightM - origin_.heightM;
    }

    /// Metres along the ground per degree of latitude (meridian radius of curvature M) and of
    /// longitude (N cos(latitude)), at the origin.
    [[nodiscard]] std::array<double, 2> metresPerDegree() const {
        const double sinLat = std::sin(origin_.latitudeDeg * kDegree);
        const double w = 1.0 - kEccentricitySquared * sinLat * sinLat;
        const double meridian = kSemiMajorAxisM * (1.0 - kEccentricitySquared) / (w * std::sqrt(w));
        const double prime = kSemiMajorAxisM / std::sqrt(w);
        return {(meridian + origin_.heightM) * kDegree,
                (prime + origin_.heightM) * std::cos(origin_.latitudeDeg * kDegree) * kDegree};
    }

private:
    Geodetic origin_;
    Ecef ecef_;
    std::array<double, 9> rotation_{};
};

/// The UTM zone of a position, "32T": the 6° zone number with the Norway (32V) and Svalbard
/// (31X, 33X, 35X, 37X) exceptions, and the 8° latitude band letter. Empty outside UTM's
/// latitudes (south of 80° S, north of 84° N), where UPS applies.
[[nodiscard]] inline std::string utmZone(double latitudeDeg, double longitudeDeg) {
    if (latitudeDeg < -80.0 || latitudeDeg > 84.0) {
        return {};
    }
    double lon = std::fmod(longitudeDeg + 180.0, 360.0);
    if (lon < 0.0) {
        lon += 360.0;
    }
    int zone = static_cast<int>(lon / 6.0) + 1;
    zone = zone > 60 ? 60 : zone;
    const char* bands = "CDEFGHJKLMNPQRSTUVWX";
    int band = static_cast<int>(std::floor((latitudeDeg + 80.0) / 8.0));
    band = band > 19 ? 19 : band;  // X runs 72° to 84° N
    const char letter = bands[band];
    lon -= 180.0;  // back to -180 .. 180
    if (letter == 'V' && lon >= 3.0 && lon < 12.0) {
        zone = 32;
    } else if (letter == 'X' && lon >= 0.0 && lon < 42.0) {
        zone = lon < 9.0 ? 31 : (lon < 21.0 ? 33 : (lon < 33.0 ? 35 : 37));
    }
    char text[16];
    std::snprintf(text, sizeof(text), "%d%c", zone, letter);
    return text;
}

/// An angle as degrees, minutes and seconds with its hemisphere: 47°23'51.87" N.
[[nodiscard]] inline std::string dms(double degrees, char positive, char negative) {
    const char hemisphere = degrees < 0.0 ? negative : positive;
    // Round to the shown hundredth of a second first, so 59.999" does not print as 60.00".
    const double hundredths = std::round(std::abs(degrees) * 360000.0);
    const auto whole = static_cast<long long>(hundredths);
    const long long d = whole / 360000;
    const long long m = (whole / 6000) % 60;
    const double s = static_cast<double>(whole % 6000) / 100.0;
    char text[40];
    std::snprintf(text, sizeof(text), "%lld°%02lld'%05.2f\" %c", d, m, s, hemisphere);
    return text;
}

}  // namespace lagrange::viewer::geodesy
