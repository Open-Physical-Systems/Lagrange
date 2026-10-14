#pragma once

#include <array>
#include <cstdint>

namespace lagrange::coords {

/// WGS84 ellipsoid (NIMA TR8350.2, "Department of Defense World Geodetic System 1984",
/// third edition, table 3.1).
namespace wgs84 {
inline constexpr double kSemiMajorAxisM = 6378137.0;         // a, m
inline constexpr double kInverseFlattening = 298.257223563;  // 1/f
inline constexpr double kFlattening = 1.0 / kInverseFlattening;
inline constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);  // e^2
}  // namespace wgs84

/// A position on the WGS84 ellipsoid: latitude and longitude in radians, height above the
/// ellipsoid (not above the geoid) in metres.
struct Geodetic {
    double latitudeRad{0.0};
    double longitudeRad{0.0};
    double heightM{0.0};
};

/// Earth-centred, Earth-fixed position in metres. An anchor only; never used for dynamics.
struct Ecef {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/// East, north, up in metres, relative to a GeoOrigin. USD stages are authored in this frame.
struct Enu {
    double e{0.0};
    double n{0.0};
    double u{0.0};
};

[[nodiscard]] Ecef toEcef(const Geodetic& point);

/// Exact closed form of Vermeille, "Direct transformation from geocentric coordinates to
/// geodetic coordinates", Journal of Geodesy 76 (2002) 451-454. Valid for points farther than
/// about 43 km from the Earth's centre, i.e. everywhere a robot can be.
[[nodiscard]] Geodetic toGeodetic(const Ecef& point);

/// The tangent plane a world is anchored to (profile `world.geo_origin`).
class GeoOrigin {
public:
    explicit GeoOrigin(const Geodetic& origin);

    [[nodiscard]] const Geodetic& geodetic() const { return origin_; }
    [[nodiscard]] const Ecef& ecef() const { return ecef_; }

    [[nodiscard]] Ecef toEcef(const Enu& point) const;
    [[nodiscard]] Enu fromEcef(const Ecef& point) const;
    [[nodiscard]] Geodetic toGeodetic(const Enu& point) const;
    [[nodiscard]] Enu fromGeodetic(const Geodetic& point) const;

    /// Row-major rotation taking ECEF vectors to ENU vectors (rows: east, north, up).
    [[nodiscard]] const std::array<double, 9>& enuFromEcefRotation() const { return rotation_; }

private:
    Geodetic origin_;
    Ecef ecef_;
    std::array<double, 9> rotation_;
};

/// Origin of the region-local frame that physics, `Pose` signals and the protocol use.
/// Region-local = world ENU - worldEnu.
struct RegionOrigin {
    std::array<double, 3> worldEnu{0.0, 0.0, 0.0};
    std::uint64_t rebaseCount{0};  // increments on every rebase
};

/// The shift to apply when the focus entity, at `focusLocal` in region-local metres, is more
/// than `rebaseDistanceM` from the region origin; all zeros otherwise.
///
/// The shift is rounded to whole metres, so subtracting it from region-local positions is exact
/// in double for any position below 2^52 m and does not disturb determinism.
[[nodiscard]] std::array<double, 3> rebaseShift(const std::array<double, 3>& focusLocal,
                                                double rebaseDistanceM);

/// Moves the region origin by `shift` (from rebaseShift) and counts the rebase. Positions in the
/// region-local frame must be reduced by the same shift.
void applyRebase(RegionOrigin& origin, const std::array<double, 3>& shift);

}  // namespace lagrange::coords
