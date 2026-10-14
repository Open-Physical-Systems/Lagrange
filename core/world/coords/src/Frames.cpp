#include "lagrange/world/coords/Frames.hpp"

#include <cmath>

namespace lagrange::coords {

namespace {

using wgs84::kEccentricitySquared;
using wgs84::kSemiMajorAxisM;

}  // namespace

Ecef toEcef(const Geodetic& point) {
    const double sinLat = std::sin(point.latitudeRad);
    const double cosLat = std::cos(point.latitudeRad);
    // Prime vertical radius of curvature.
    const double n = kSemiMajorAxisM / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);
    const double r = (n + point.heightM) * cosLat;
    return {r * std::cos(point.longitudeRad), r * std::sin(point.longitudeRad),
            (n * (1.0 - kEccentricitySquared) + point.heightM) * sinLat};
}

Geodetic toGeodetic(const Ecef& point) {
    // Vermeille (2002), section 2, with e^4 = (e^2)^2.
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
    return {2.0 * std::atan2(point.z, d + dz), std::atan2(point.y, point.x),
            (k + e2 - 1.0) / k * dz};
}

GeoOrigin::GeoOrigin(const Geodetic& origin)
    : origin_(origin), ecef_(coords::toEcef(origin)), rotation_{} {
    const double sinLat = std::sin(origin.latitudeRad);
    const double cosLat = std::cos(origin.latitudeRad);
    const double sinLon = std::sin(origin.longitudeRad);
    const double cosLon = std::cos(origin.longitudeRad);
    rotation_ = {-sinLon,          cosLon,           0.0,      // east
                 -sinLat * cosLon, -sinLat * sinLon, cosLat,   // north
                 cosLat * cosLon,  cosLat * sinLon,  sinLat};  // up
}

Ecef GeoOrigin::toEcef(const Enu& point) const {
    const std::array<double, 9>& m = rotation_;
    // The transpose of the ENU-from-ECEF rotation takes ENU vectors back to ECEF.
    return {ecef_.x + m[0] * point.e + m[3] * point.n + m[6] * point.u,
            ecef_.y + m[1] * point.e + m[4] * point.n + m[7] * point.u,
            ecef_.z + m[2] * point.e + m[5] * point.n + m[8] * point.u};
}

Enu GeoOrigin::fromEcef(const Ecef& point) const {
    const std::array<double, 9>& m = rotation_;
    const double dx = point.x - ecef_.x;
    const double dy = point.y - ecef_.y;
    const double dz = point.z - ecef_.z;
    return {m[0] * dx + m[1] * dy + m[2] * dz, m[3] * dx + m[4] * dy + m[5] * dz,
            m[6] * dx + m[7] * dy + m[8] * dz};
}

Geodetic GeoOrigin::toGeodetic(const Enu& point) const {
    return coords::toGeodetic(toEcef(point));
}

Enu GeoOrigin::fromGeodetic(const Geodetic& point) const {
    return fromEcef(coords::toEcef(point));
}

std::array<double, 3> rebaseShift(const std::array<double, 3>& focusLocal, double rebaseDistanceM) {
    const double distance =
        std::sqrt(focusLocal[0] * focusLocal[0] + focusLocal[1] * focusLocal[1] +
                  focusLocal[2] * focusLocal[2]);
    if (!(distance > rebaseDistanceM)) {
        return {0.0, 0.0, 0.0};
    }
    return {std::round(focusLocal[0]), std::round(focusLocal[1]), std::round(focusLocal[2])};
}

void applyRebase(RegionOrigin& origin, const std::array<double, 3>& shift) {
    for (std::size_t i = 0; i < 3; ++i) {
        origin.worldEnu.at(i) += shift.at(i);
    }
    ++origin.rebaseCount;
}

}  // namespace lagrange::coords
