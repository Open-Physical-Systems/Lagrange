// World Magnetic Model 2025 (Wmm.hpp). Equation numbers are those of the WMM2025 technical
// report, section 1.2.
#include "lagrange/sensors/Wmm.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors {

namespace {

#include "generated/Wmm2025Coefficients.inc"

constexpr double kReferenceRadiusM = 6371200.0;  // a, geomagnetic reference radius (eq. 4)
constexpr int kSize = kWmmMaxDegree + 1;

}  // namespace

std::array<double, 3> wmmField(const coords::Geodetic& where, double decimalYear) {
    // Geodetic to geocentric spherical coordinates on WGS84 (eqs. 7-8).
    const double sinLat = std::sin(where.latitudeRad);
    const double cosLat = std::cos(where.latitudeRad);
    const double rc = coords::wgs84::kSemiMajorAxisM /
                      std::sqrt(1.0 - coords::wgs84::kEccentricitySquared * sinLat * sinLat);
    const double p = (rc + where.heightM) * cosLat;
    const double z = (rc * (1.0 - coords::wgs84::kEccentricitySquared) + where.heightM) * sinLat;
    const double r = std::hypot(p, z);
    const double geocentric = std::asin(z / r);

    // Gauss coefficients at the date (eq. 9), Schmidt semi-normalised.
    std::array<std::array<double, kSize>, kSize> g{}, h{};
    const double years = decimalYear - kWmmEpoch;
    for (const auto& row : kWmmCoefficients) {
        const auto n = static_cast<std::size_t>(row[0]);
        const auto m = static_cast<std::size_t>(row[1]);
        g[n][m] = row[2] + years * row[4];
        h[n][m] = row[3] + years * row[5];
    }

    // Associated Legendre functions of cos(colatitude) = sin(geocentric latitude) and their
    // derivatives with respect to the colatitude, Gauss-normalised by recursion, then scaled
    // to Schmidt semi-normalisation (eq. 5).
    const double cosT = std::sin(geocentric);
    const double sinT = std::cos(geocentric);
    std::array<std::array<double, kSize>, kSize> pnm{}, dpnm{}, schmidt{};
    pnm[0][0] = 1.0;
    schmidt[0][0] = 1.0;
    for (std::size_t n = 1; n < kSize; ++n) {
        schmidt[n][0] = schmidt[n - 1][0] * static_cast<double>(2 * n - 1) / static_cast<double>(n);
        for (std::size_t m = 1; m <= n; ++m) {
            const double factor =
                static_cast<double>((n - m + 1) * (m == 1 ? 2 : 1)) / static_cast<double>(n + m);
            schmidt[n][m] = schmidt[n][m - 1] * std::sqrt(factor);
        }
        for (std::size_t m = 0; m <= n; ++m) {
            if (n == m) {
                pnm[n][m] = sinT * pnm[n - 1][m - 1];
                dpnm[n][m] = sinT * dpnm[n - 1][m - 1] + cosT * pnm[n - 1][m - 1];
            } else if (n == 1) {
                pnm[n][m] = cosT * pnm[n - 1][m];
                dpnm[n][m] = cosT * dpnm[n - 1][m] - sinT * pnm[n - 1][m];
            } else {
                const double k = static_cast<double>((n - 1) * (n - 1) - m * m) /
                                 static_cast<double>((2 * n - 1) * (2 * n - 3));
                pnm[n][m] = cosT * pnm[n - 1][m] - k * pnm[n - 2][m];
                dpnm[n][m] = cosT * dpnm[n - 1][m] - sinT * pnm[n - 1][m] - k * dpnm[n - 2][m];
            }
        }
    }

    // The field in geocentric spherical coordinates (eq. 10).
    double north = 0.0;
    double east = 0.0;
    double down = 0.0;
    const double ratio = kReferenceRadiusM / r;
    double power = ratio * ratio;  // (a/r)^(n+2) for n = 0
    for (std::size_t n = 1; n < kSize; ++n) {
        power *= ratio;
        for (std::size_t m = 0; m <= n; ++m) {
            const double ml = static_cast<double>(m) * where.longitudeRad;
            const double gs = g[n][m] * schmidt[n][m];
            const double hs = h[n][m] * schmidt[n][m];
            const double cosine = gs * std::cos(ml) + hs * std::sin(ml);
            north += power * cosine * dpnm[n][m];
            east += power * static_cast<double>(m) * (gs * std::sin(ml) - hs * std::cos(ml)) *
                    pnm[n][m];
            down -= power * static_cast<double>(n + 1) * cosine * pnm[n][m];
        }
    }
    east /= sinT;

    // Back to the ellipsoidal frame (eq. 17).
    const double tilt = geocentric - where.latitudeRad;
    return {north * std::cos(tilt) - down * std::sin(tilt), east,
            north * std::sin(tilt) + down * std::cos(tilt)};
}

}  // namespace lagrange::sensors
