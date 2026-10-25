// The World Magnetic Model 2025 against NOAA's published test values (data/wmm2025,
// WMM2025_TestValues.txt): X, Y, Z for points from pole to pole, 0 to 100 km, 2025 to 2030.
#include "lagrange/sensors/Wmm.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr double kToleranceNt = 0.5;  // the test values are printed to 1e-6 nT; the model's
                                      // own arithmetic agrees far closer than this

TEST(Wmm, MatchesNoaaTestValues) {
    std::ifstream in(LAGRANGE_TEST_WMM_DIR "/WMM2025_TestValues.txt");
    ASSERT_TRUE(in) << "test values not found";
    int rows = 0;
    double worst = 0.0;
    for (std::string line; std::getline(in, line);) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream f(line);
        double year = 0, altitudeKm = 0, lat = 0, lon = 0, d = 0, i = 0, hz = 0, x = 0, y = 0,
               z = 0;
        f >> year >> altitudeKm >> lat >> lon >> d >> i >> hz >> x >> y >> z;
        const double deg = std::numbers::pi / 180.0;
        const std::array<double, 3> b = wmmField({lat * deg, lon * deg, altitudeKm * 1e3}, year);
        for (const auto& [got, want] : {std::pair{b[0], x}, {b[1], y}, {b[2], z}}) {
            worst = std::max(worst, std::abs(got - want));
            EXPECT_NEAR(got, want, kToleranceNt) << line;
        }
        ++rows;
    }
    EXPECT_GE(rows, 12);
    std::printf("WMM2025: %d test points, worst component error %.2e nT\n", rows, worst);
}

}  // namespace
}  // namespace lagrange::sensors
