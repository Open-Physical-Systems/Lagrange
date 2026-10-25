// Writes the sample fields of assets/fields (illustrative, made up from analytic shapes):
//   lagrange_make_fields fog_bank OUT.nvdb    fog across the warehouse's aisle at x 2 to 6 m
//   lagrange_make_fields wind_shear OUT.nvdb  wind over the open field, with the shed's wake
#include "lagrange/fields/Field.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

using lagrange::fields::DenseBox;
using lagrange::fields::Vec3;

/// 0 below a, 1 above b, smooth between.
double smoothstep(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// A dense bank of fog, 10 m visibility (σ = 3.912 / 10) inside, soft over half a metre.
double fogBank(const Vec3& p) {
    constexpr double kInside = 3.912 / 10.0;  // 1/m
    const double x = smoothstep(1.5, 2.5, p[0]) * (1.0 - smoothstep(5.5, 6.5, p[0]));
    const double y = smoothstep(-5.0, -4.0, p[1]) * (1.0 - smoothstep(4.0, 5.0, p[1]));
    const double z = 1.0 - smoothstep(2.5, 3.5, p[2]);
    return kInside * x * y * z;
}

/// Wind from the west, 5 m/s at 10 m over open terrain (logarithmic, z0 = 0.03 m), slowed
/// behind the shed at (−18, −2) in an illustrative wake.
Vec3 windShear(const Vec3& p) {
    constexpr double kRoughness = 0.03;  // m, open terrain
    const double z = std::max(p[2], 2.0 * kRoughness);
    double u = 5.0 * std::log(z / kRoughness) / std::log(10.0 / kRoughness);
    if (p[0] > -16.0 && p[2] < 6.0) {
        const double across = (p[1] + 2.0) / 2.5;
        u *= 1.0 - 0.6 * std::exp(-across * across) * std::exp(-(p[0] + 16.0) / 10.0) *
                       (1.0 - smoothstep(3.0, 6.0, p[2]));
    }
    return {u, 0.0, 0.0};
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: lagrange_make_fields fog_bank|wind_shear OUT.nvdb\n");
        return 2;
    }
    const std::string what = argv[1];
    lagrange::Expected<void, std::string> written =
        lagrange::Unexpected{std::string("unknown field")};
    if (what == "fog_bank") {
        written = lagrange::fields::writeScalarField(
            argv[2], "extinction", DenseBox{{0.0, -6.0, 0.0}, 0.25, {33, 49, 17}}, fogBank);
    } else if (what == "wind_shear") {
        written = lagrange::fields::writeVectorField(
            argv[2], "wind", DenseBox{{-25.0, -25.0, 0.0}, 2.0, {26, 26, 11}}, windShear);
    }
    if (!written) {
        std::fprintf(stderr, "%s\n", written.error().c_str());
        return 1;
    }
    return 0;
}
