// Tyre models (domains/vehicle_dynamics.md §7): the Magic Formula's analytic properties, the
// MF-Tyre 5.2 load dependence and combined slip, .tir reading, the linear tyre's ellipse.
#include "lagrange/vehicle_dynamics/Tyre.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::vehicle_dynamics {
namespace {

/// The largest value of f over [0, xMax], by a fine scan refined by golden section.
template <class F>
double peak(F f, double xMax) {
    double best = 0.0;
    double at = 0.0;
    constexpr int kScan = 20000;
    for (int i = 0; i <= kScan; ++i) {
        const double x = xMax * i / kScan;
        if (f(x) > best) {
            best = f(x);
            at = x;
        }
    }
    double lo = std::max(0.0, at - xMax / kScan);
    double hi = at + xMax / kScan;
    for (int i = 0; i < 200; ++i) {
        const double m1 = lo + (hi - lo) * 0.381966;
        const double m2 = lo + (hi - lo) * 0.618034;
        (f(m1) < f(m2) ? lo : hi) = f(m1) < f(m2) ? m1 : m2;
    }
    return f(0.5 * (lo + hi));
}

TEST(MagicFormula, PeakSlopeAndAsymptote) {
    for (const double e : {0.0, 0.5, -1.0}) {
        const double b = 10.0, c = 1.65, d = 4000.0;
        const auto f = [&](double x) { return magicFormula(x, b, c, d, e); };
        // The peak is D, where C·atan(·) reaches π/2.
        EXPECT_NEAR(peak(f, 2.0), d, 1e-6 * d) << e;
        // The slope at zero is B·C·D, whatever E.
        const double h = 1e-7;
        EXPECT_NEAR((f(h) - f(-h)) / (2.0 * h), b * c * d, 1e-4 * b * c * d) << e;
        // Far out the curve tends to D·sin(C·π/2).
        EXPECT_NEAR(f(1e9), d * std::sin(c * std::numbers::pi / 2.0), 1e-3 * d) << e;
    }
}

MagicFormula sample() {
    const std::filesystem::path tir =
        std::filesystem::path(LAGRANGE_TEST_ASSETS) / "tyres/generic_205_55R16.tir";
    auto mf = MagicFormula::fromTir(tir.string());
    EXPECT_TRUE(mf) << (mf ? "" : mf.error());
    return *mf;
}

TEST(Pacejka, PureSlipFollowsTheFileAndOpposesTheSlip) {
    const MagicFormula mf = sample();
    EXPECT_EQ(mf.fz0, 4000.0);
    // At the nominal load: peak Fx = PDX1·Fz, initial slope Kx = PKX1·Fz.
    const auto fx = [&](double k) { return pacejka(mf, k, 0.0, 4000.0, 1.0).fx; };
    EXPECT_NEAR(peak(fx, 1.0), 1.05 * 4000.0, 1e-3);
    EXPECT_NEAR(fx(1e-6) / 1e-6, 21.0 * 4000.0, 1.0);
    EXPECT_LT(pacejka(mf, -0.1, 0.0, 4000.0, 1.0).fx, 0.0);  // braking: backwards
    // Lateral: the force is against the slip, its stiffness PKY1·Fz0·sin(2·atan(Fz/(PKY2·Fz0))).
    const double ky = 17.5 * 4000.0 * std::sin(2.0 * std::atan(1.0 / 1.9));
    EXPECT_NEAR(pacejka(mf, 0.0, 1e-6, 4000.0, 1.0).fy / -1e-6, ky, 1.0);
    EXPECT_GT(pacejka(mf, 0.0, -0.05, 4000.0, 1.0).fy, 0.0);
    // Load sensitivity: twice the load, less than twice the peak (PDY2 < 0).
    const auto fy8 = [&](double a) { return -pacejka(mf, 0.0, a, 8000.0, 1.0).fy; };
    EXPECT_NEAR(peak(fy8, 0.5), (1.00 - 0.10) * 8000.0, 1e-3);
    // A wet road: the friction scale scales the peak.
    const auto wet = [&](double k) { return pacejka(mf, k, 0.0, 4000.0, 0.6).fx; };
    EXPECT_NEAR(peak(wet, 1.0), 0.6 * 1.05 * 4000.0, 1e-3);
}

TEST(Pacejka, CombinedSlipShrinksEachForce) {
    const MagicFormula mf = sample();
    const double fyPure = pacejka(mf, 0.0, 0.05, 4000.0, 1.0).fy;
    const double fyBraking = pacejka(mf, -0.1, 0.05, 4000.0, 1.0).fy;
    EXPECT_LT(std::abs(fyBraking), std::abs(fyPure));
    const double fxPure = pacejka(mf, 0.1, 0.0, 4000.0, 1.0).fx;
    const double fxCornering = pacejka(mf, 0.1, 0.08, 4000.0, 1.0).fx;
    EXPECT_LT(fxCornering, fxPure);
    EXPECT_GT(fxCornering, 0.0);
    // The resultant stays within what the tyre can give.
    const TyreForce both = pacejka(mf, 0.1, 0.08, 4000.0, 1.0);
    EXPECT_LT(std::hypot(both.fx, both.fy), 1.05 * 4000.0 * 1.0001);
}

TEST(Pacejka, ReadsTirFilesAndNamesWhatIsMissing) {
    const std::string path = testing::TempDir() + "/small.tir";
    std::ofstream(path) << "[VERTICAL]\nfnomin = 3000 $nominal\n[LONGITUDINAL_COEFFICIENTS]\n"
                           "PCX1 = 1.6\nPDX1=1\nPKX1 = 20 ! slip stiffness\n"
                           "[LATERAL_COEFFICIENTS]\nPCY1 = 1.3\nPDY1 = 0.9\nPKY1 = -15\n";
    const auto missing = MagicFormula::fromTir(path);
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("PKY2"), std::string::npos);
    std::ofstream(path, std::ios::app) << "PKY2 = 2\n";
    const auto mf = MagicFormula::fromTir(path);
    ASSERT_TRUE(mf) << mf.error();
    EXPECT_EQ(mf->fz0, 3000.0);
    EXPECT_EQ(mf->pkx1, 20.0);
}

TEST(LinearTyre, ForcesOnTheFrictionEllipse) {
    const TyreForce small = linearTyre(80000.0, 70000.0, 1.0, 0.01, 0.01, 4000.0);
    EXPECT_DOUBLE_EQ(small.fx, 800.0);
    EXPECT_DOUBLE_EQ(small.fy, -700.0);
    const TyreForce big = linearTyre(80000.0, 70000.0, 1.0, 0.2, 0.2, 4000.0);
    EXPECT_NEAR(std::hypot(big.fx, big.fy), 4000.0, 1e-9);
    EXPECT_NEAR(big.fx / big.fy, -80.0 / 70.0, 1e-12);
}

}  // namespace
}  // namespace lagrange::vehicle_dynamics
