// The lumped thermal network against closed forms (docs/design/domains/thermal.md §7).
#include "lagrange/thermal/Lumped.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::thermal {
namespace {

TEST(LumpedNetwork, ASingleNodeCoolsExponentially) {
    // C·dT/dt = −hA·(T − T_air): T = T_air + (T0 − T_air)·e^(−t·hA/C). Required within 0.1 % of
    // the initial difference, at the default 100 ms step.
    constexpr double kCapacity = 1000.0;  // J/K
    constexpr double kConvection = 2.0;   // W/K
    LumpedNetwork net({{kCapacity, kConvection, 0.0, 0.0}}, {});
    std::array<double, 1> t{350.0};
    const std::array<double, 1> q{0.0};
    std::array<double, 1> out{};
    const Surroundings air{300.0, 300.0, 0.0};
    for (int n = 1; n <= 20000; ++n) {  // 2000 s, four time constants
        net.step(t, q, air, 0.1, out);
        if (n % 1000 == 0) {
            const double expected = 300.0 + 50.0 * std::exp(-n * 0.1 * kConvection / kCapacity);
            EXPECT_NEAR(t[0], expected, 1e-3 * 50.0) << "at " << n * 0.1 << " s";
        }
    }
    EXPECT_NEAR(out[0], kConvection * (t[0] - 300.0), 1e-9);
}

TEST(LumpedNetwork, TwoNodesFollowTheTwoPoleStepResponse) {
    // Node 1 (a motor, heated by Q, cooled by convection) linked to node 2 (the chassis, no
    // other exchange): x' = A·x + b, solved exactly by eigen-decomposing A. Within 0.5 % of the
    // rise.
    constexpr double c1 = 450.0, c2 = 9000.0, g = 0.5, h = 0.3, q = 40.0, air = 298.15;
    LumpedNetwork net({{c1, h, 0.0, 0.0}, {c2, 0.0, 0.0, 0.0}}, {{0, 1, g}});
    std::vector<double> t{air, air};
    const std::vector<double> sources{q, 0.0};
    std::vector<double> out(2);
    const Surroundings around{air, air, 0.0};
    // Deviations from the air: A = [[−(g+h)/c1, g/c1], [g/c2, −g/c2]], b = [q/c1, 0].
    const double a11 = -(g + h) / c1, a12 = g / c1, a21 = g / c2, a22 = -g / c2;
    const double tr = a11 + a22;
    const double det = a11 * a22 - a12 * a21;
    const double l1 = 0.5 * (tr + std::sqrt(tr * tr - 4.0 * det));
    const double l2 = 0.5 * (tr - std::sqrt(tr * tr - 4.0 * det));
    // Steady state: x∞ = −A⁻¹·b, both nodes at q/h above the air.
    const double inf1 = q / h;
    const double inf2 = q / h;
    // x(t) = x∞ + k1·v1·e^(l1·t) + k2·v2·e^(l2·t), v = (a12, l − a11), x(0) = 0.
    const double v11 = a12, v12 = l1 - a11, v21 = a12, v22 = l2 - a11;
    const double k2 = (-inf2 * v11 + inf1 * v12) / (v11 * v22 - v12 * v21);
    const double k1 = (-inf1 - k2 * v21) / v11;
    double worst = 0.0;
    for (int n = 1; n <= 200000; ++n) {  // 20000 s
        net.step(t, sources, around, 0.1, out);
        if (n % 5000 == 0) {
            const double s = n * 0.1;
            const double x1 = inf1 + k1 * v11 * std::exp(l1 * s) + k2 * v21 * std::exp(l2 * s);
            const double x2 = inf2 + k1 * v12 * std::exp(l1 * s) + k2 * v22 * std::exp(l2 * s);
            worst = std::max({worst, std::abs(t[0] - air - x1), std::abs(t[1] - air - x2)});
        }
    }
    EXPECT_LT(worst, 5e-3 * inf1);
}

TEST(LumpedNetwork, ASunlitPlateReachesItsRadiativeEquilibrium) {
    // α·A·E = ε·σ·A·(T⁴ − T_sky⁴) + h·A·(T − T_air), solved by bisection; required within 0.5 K.
    constexpr double area = 0.5, alpha = 0.7, eps = 0.9, hc = 5.0, e = 800.0;
    constexpr double air = 293.15, sky = 273.15;
    LumpedNetwork net({{2000.0, hc * area, eps * kStefanBoltzmann * area, alpha * area}}, {});
    std::array<double, 1> t{air};
    const std::array<double, 1> q{0.0};
    std::array<double, 1> out{};
    const Surroundings around{air, sky, e};
    for (int n = 0; n < 200000; ++n) {
        net.step(t, q, around, 0.1, out);
    }
    double lo = 200.0, hi = 500.0;
    for (int k = 0; k < 200; ++k) {
        const double mid = 0.5 * (lo + hi);
        const double net_in =
            alpha * area * e -
            eps * kStefanBoltzmann * area * (std::pow(mid, 4) - std::pow(sky, 4)) -
            hc * area * (mid - air);
        (net_in > 0.0 ? lo : hi) = mid;
    }
    EXPECT_NEAR(t[0], lo, 0.5);
    EXPECT_GT(t[0], air + 10.0);  // the sun did warm it
}

TEST(LumpedNetwork, EnergyIsConserved) {
    // Σ C·ΔT = Σ (Q + solar − heat out)·dt each step: links move heat without making any.
    LumpedNetwork net({{450.0, 0.2, 1e-9, 0.01}, {9000.0, 9.0, 5e-8, 0.3}, {8000.0, 1.0, 0.0, 0.0}},
                      {{0, 1, 0.5}, {2, 1, 2.0}, {0, 2, 0.1}});
    const std::vector<double> c{450.0, 9000.0, 8000.0};
    std::vector<double> t{310.0, 300.0, 305.0};
    const std::vector<double> q{60.0, 0.0, 8.0};
    std::vector<double> out(3);
    const Surroundings around{295.0, 270.0, 600.0};
    const std::vector<double> solar{0.01, 0.3, 0.0};
    for (int n = 0; n < 1000; ++n) {
        const std::vector<double> before = t;
        net.step(t, q, around, 0.1, out);
        double stored = 0.0;
        double flowed = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            stored += c[i] * (t[i] - before[i]);
            flowed += (q[i] + solar[i] * around.irradiance - out[i]) * 0.1;
        }
        ASSERT_NEAR(stored, flowed, 1e-9 * (std::abs(flowed) + 1.0)) << "step " << n;
    }
}

TEST(Derating, FallsLinearlyBetweenItsLimits) {
    EXPECT_EQ(derating(300.0, 393.15, 428.15), 1.0);
    EXPECT_NEAR(derating(410.65, 393.15, 428.15), 0.5, 1e-12);
    EXPECT_EQ(derating(430.0, 393.15, 428.15), 0.0);
    EXPECT_EQ(derating(1000.0, 0.0, 0.0), 1.0);  // no limits
}

}  // namespace
}  // namespace lagrange::thermal
