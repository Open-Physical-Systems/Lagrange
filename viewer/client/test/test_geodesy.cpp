// The viewer's header-only WGS84 <-> ENU (Geodesy.hpp) against the simulation core's
// lagrange::coords, which the server uses: both must give the same latitude, longitude and
// height for the same world position. Linked with lagrange_core here only, in a test.
#include "lagrange/viewer/Geodesy.hpp"
#include "lagrange/world/coords/Frames.hpp"

#include <array>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace geo = lagrange::viewer::geodesy;
namespace core = lagrange::coords;

namespace {

constexpr double kRad = std::numbers::pi / 180.0;

core::Geodetic toCore(const geo::Geodetic& g) {
    return {g.latitudeDeg * kRad, g.longitudeDeg * kRad, g.heightM};
}

// Origins: the samples' Zurich and Tokyo, the equator on the antimeridian, the south, near a
// pole, high above the ellipsoid.
const std::array<geo::Geodetic, 6> kOrigins{{{47.397742, 8.545594, 488.0},
                                             {35.689568, 139.691717, 68.5},
                                             {0.0, 180.0, 0.0},
                                             {-33.8688, 151.2093, 20.0},
                                             {89.5, -45.0, 2800.0},
                                             {-12.0, -77.0, 10000.0}}};
// ENU points: on the origin, a street away, the 40 km sample's corner, high up and below.
const std::array<geo::Enu, 6> kPoints{{{0.0, 0.0, 0.0},
                                       {12.5, -3.25, 1.5},
                                       {1036.0, 1036.0, 0.0},
                                       {-20000.0, 20000.0, 0.0},
                                       {5000.0, -15000.0, 3000.0},
                                       {-250.0, 400.0, -120.0}}};

TEST(Geodesy, MatchesTheCoreForEveryOriginAndPoint) {
    for (const geo::Geodetic& origin : kOrigins) {
        const geo::Origin viewer(origin);
        const core::GeoOrigin reference(toCore(origin));
        for (const geo::Enu& point : kPoints) {
            const geo::Geodetic a = viewer.toGeodetic(point);
            const core::Geodetic b = reference.toGeodetic({point.e, point.n, point.u});
            // 1e-10 degree is about 11 µm on the ground.
            EXPECT_NEAR(a.latitudeDeg, b.latitudeRad / kRad, 1e-10);
            double dLon = a.longitudeDeg - b.longitudeRad / kRad;
            dLon -= 360.0 * std::round(dLon / 360.0);  // ±180 are the same meridian
            EXPECT_NEAR(dLon, 0.0, 1e-10);
            EXPECT_NEAR(a.heightM, b.heightM, 1e-6);
            const geo::Ecef e = viewer.toEcef(point);
            const core::Ecef f = reference.toEcef({point.e, point.n, point.u});
            EXPECT_NEAR(e.x, f.x, 1e-6);
            EXPECT_NEAR(e.y, f.y, 1e-6);
            EXPECT_NEAR(e.z, f.z, 1e-6);
        }
    }
}

TEST(Geodesy, RoundTripsThroughLatitudeAndLongitude) {
    for (const geo::Geodetic& origin : kOrigins) {
        const geo::Origin viewer(origin);
        for (const geo::Enu& point : kPoints) {
            const geo::Enu back = viewer.fromGeodetic(viewer.toGeodetic(point));
            EXPECT_NEAR(back.e, point.e, 1e-6);
            EXPECT_NEAR(back.n, point.n, 1e-6);
            EXPECT_NEAR(back.u, point.u, 1e-6);
        }
        const geo::Geodetic self = viewer.toGeodetic({});
        EXPECT_NEAR(self.latitudeDeg, origin.latitudeDeg, 1e-10);
        EXPECT_NEAR(self.heightM, origin.heightM, 1e-6);
    }
}

TEST(Geodesy, MetresPerDegreeMatchesASmallStepThroughTheCore) {
    for (const geo::Geodetic& origin : kOrigins) {
        if (std::abs(origin.latitudeDeg) > 89.0) {
            continue;  // a degree of longitude is a few hundred metres there: no flat step
        }
        const geo::Origin viewer(origin);
        const core::GeoOrigin reference(toCore(origin));
        constexpr double kStepDeg = 1e-5;
        const core::Enu north = reference.fromGeodetic(
            {(origin.latitudeDeg + kStepDeg) * kRad, origin.longitudeDeg * kRad, origin.heightM});
        const core::Enu east = reference.fromGeodetic(
            {origin.latitudeDeg * kRad, (origin.longitudeDeg + kStepDeg) * kRad, origin.heightM});
        const std::array<double, 2> perDegree = viewer.metresPerDegree();
        EXPECT_NEAR(perDegree[0], north.n / kStepDeg, 0.05);
        EXPECT_NEAR(perDegree[1], east.e / kStepDeg, 0.05);
    }
}

TEST(Geodesy, TheFlatWorldRisesAboveTheEllipsoidAsDSquaredOverTwoR) {
    const geo::Origin zurich(kOrigins[0]);
    EXPECT_NEAR(zurich.planeRiseM(0.0, 0.0), 0.0, 1e-6);
    // 1 km: about 8 cm; 20 km: about 31 m (R about 6 371 km).
    for (const double d : {1000.0, 20000.0}) {
        const double rise = zurich.planeRiseM(d, 0.0);
        EXPECT_NEAR(rise, d * d / (2.0 * 6.371e6), 0.01 * rise) << d;
    }
}

TEST(Geodesy, UtmZoneWithItsExceptions) {
    EXPECT_EQ(geo::utmZone(47.397742, 8.545594), "32T");    // Zurich
    EXPECT_EQ(geo::utmZone(35.689568, 139.691717), "54S");  // Tokyo
    EXPECT_EQ(geo::utmZone(-33.8688, 151.2093), "56H");     // Sydney
    EXPECT_EQ(geo::utmZone(60.39, 5.32), "32V");            // Bergen: the Norway exception
    EXPECT_EQ(geo::utmZone(78.22, 15.65), "33X");           // Longyearbyen: Svalbard
    EXPECT_EQ(geo::utmZone(0.0, 180.0), "1N");
    EXPECT_EQ(geo::utmZone(85.0, 0.0), "");  // UPS
}

TEST(Geodesy, DegreesMinutesSeconds) {
    EXPECT_EQ(geo::dms(47.397742, 'N', 'S'), "47°23'51.87\" N");
    EXPECT_EQ(geo::dms(-77.0, 'E', 'W'), "77°00'00.00\" W");
    EXPECT_EQ(geo::dms(8.9999999999, 'E', 'W'), "9°00'00.00\" E");  // no 60.00"
}

}  // namespace
