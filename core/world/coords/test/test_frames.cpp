#include "lagrange/random/Philox.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::coords {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
// Acceptance criterion of docs/design/huge-worlds.md §6: agreement within 1e-6 m.
constexpr double kToleranceM = 1e-6;

double distance(const Ecef& a, double x, double y, double z) {
    return std::hypot(a.x - x, a.y - y, a.z - z);
}

/// Origins spread over the globe, including the poles' neighbourhood and the antimeridian.
std::vector<Geodetic> origins() {
    return {{0.0, 0.0, 0.0},
            {33.5902 * kDegree, 130.4017 * kDegree, 12.0},   // Fukuoka
            {-33.8688 * kDegree, 151.2093 * kDegree, 58.0},  // Sydney
            {52.3676 * kDegree, 4.9041 * kDegree, -2.0},     // Amsterdam, below the ellipsoid
            {27.9881 * kDegree, 86.9250 * kDegree, 8849.0},  // Everest
            {89.9 * kDegree, -45.0 * kDegree, 2800.0},       // near the North Pole
            {-12.0 * kDegree, 179.999 * kDegree, 100.0}};    // antimeridian
}

TEST(Frames, GeodeticToEcefMatchesGeographicLib) {
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    for (const Geodetic& point : origins()) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        earth.Forward(point.latitudeRad / kDegree, point.longitudeRad / kDegree, point.heightM, x,
                      y, z);
        EXPECT_LT(distance(toEcef(point), x, y, z), kToleranceM);
    }
}

TEST(Frames, EcefToGeodeticMatchesGeographicLib) {
    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    random::Stream rng(random::moduleSeed(2026, 1), 0, 0);
    for (int i = 0; i < 2000; ++i) {
        // Heights from a mine shaft to an airliner's cruise altitude.
        const Geodetic point{(rng.uniform() * 179.8 - 89.9) * kDegree,
                             (rng.uniform() * 360.0 - 180.0) * kDegree,
                             rng.uniform() * 13000.0 - 3000.0};
        const Ecef ecef = toEcef(point);
        double lat = 0.0;
        double lon = 0.0;
        double h = 0.0;
        earth.Reverse(ecef.x, ecef.y, ecef.z, lat, lon, h);
        const Geodetic ours = toGeodetic(ecef);
        // Compare the positions the angles describe, in metres, not the angles themselves.
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        earth.Forward(ours.latitudeRad / kDegree, ours.longitudeRad / kDegree, ours.heightM, x, y,
                      z);
        ASSERT_LT(distance(ecef, x, y, z), kToleranceM) << "point " << i;
        ASSERT_NEAR(ours.heightM, h, kToleranceM) << "point " << i;
        ASSERT_NEAR(ours.latitudeRad / kDegree, lat, 1e-11) << "point " << i;
    }
}

TEST(Frames, EnuMatchesGeographicLibWithin100Km) {
    random::Stream rng(random::moduleSeed(2026, 2), 0, 0);
    for (const Geodetic& anchor : origins()) {
        const GeoOrigin origin(anchor);
        const GeographicLib::LocalCartesian reference(
            anchor.latitudeRad / kDegree, anchor.longitudeRad / kDegree, anchor.heightM);
        for (int i = 0; i < 500; ++i) {
            const Enu point{(rng.uniform() * 2.0 - 1.0) * 70'000.0,
                            (rng.uniform() * 2.0 - 1.0) * 70'000.0, rng.uniform() * 3000.0 - 500.0};
            // ENU -> geodetic against GeographicLib's reverse transform.
            double lat = 0.0;
            double lon = 0.0;
            double h = 0.0;
            reference.Reverse(point.e, point.n, point.u, lat, lon, h);
            const Geodetic ours = origin.toGeodetic(point);
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            GeographicLib::Geocentric::WGS84().Forward(lat, lon, h, x, y, z);
            ASSERT_LT(distance(toEcef(ours), x, y, z), kToleranceM);

            // Geodetic -> ENU, and the round trip.
            double e = 0.0;
            double n = 0.0;
            double u = 0.0;
            reference.Forward(lat, lon, h, e, n, u);
            const Enu back = origin.fromGeodetic(ours);
            ASSERT_LT(std::hypot(back.e - e, back.n - n, back.u - u), kToleranceM);
            ASSERT_LT(std::hypot(back.e - point.e, back.n - point.n, back.u - point.u),
                      kToleranceM);
        }
    }
}

TEST(Frames, RotationIsOrthonormal) {
    for (const Geodetic& anchor : origins()) {
        const GeoOrigin origin(anchor);
        const std::array<double, 9>& m = origin.enuFromEcefRotation();
        for (int row = 0; row < 3; ++row) {
            for (int other = 0; other < 3; ++other) {
                double dot = 0.0;
                for (int col = 0; col < 3; ++col) {
                    dot += m.at(static_cast<std::size_t>(row * 3 + col)) *
                           m.at(static_cast<std::size_t>(other * 3 + col));
                }
                EXPECT_NEAR(dot, row == other ? 1.0 : 0.0, 1e-15);
            }
        }
    }
}

TEST(Rebase, NoShiftInsideTheDistance) {
    EXPECT_EQ(rebaseShift({600.0, 700.0, 0.0}, 1000.0), (std::array<double, 3>{0.0, 0.0, 0.0}));
    EXPECT_EQ(rebaseShift({1000.0, 0.0, 0.0}, 1000.0), (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(Rebase, ShiftIsInWholeMetres) {
    const std::array<double, 3> shift = rebaseShift({1200.4, -35.6, 2.5}, 1000.0);
    EXPECT_EQ(shift, (std::array<double, 3>{1200.0, -36.0, 3.0}));
}

TEST(Rebase, SubtractingTheShiftIsExact) {
    // A position 500 km away keeps every bit of its offset from the focus after the rebase.
    const std::array<double, 3> focus{500'000.123456789, -250'000.987654321, 12.000001};
    const std::array<double, 3> body{500'003.5, -249'998.25, 13.75};
    const std::array<double, 3> shift = rebaseShift(focus, 1000.0);
    RegionOrigin origin;
    applyRebase(origin, shift);
    EXPECT_EQ(origin.rebaseCount, 1U);
    EXPECT_EQ(origin.worldEnu, shift);
    for (std::size_t i = 0; i < 3; ++i) {
        const double local = body.at(i) - shift.at(i);
        EXPECT_EQ(local + shift.at(i), body.at(i));
        EXPECT_EQ((body.at(i) - focus.at(i)), (local - (focus.at(i) - shift.at(i))));
    }
}

}  // namespace
}  // namespace lagrange::coords
