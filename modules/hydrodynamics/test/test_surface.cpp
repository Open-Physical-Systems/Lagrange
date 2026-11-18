// The moving water surface against linear wave theory (Surface.hpp; water.md "The surface under a
// rotor"): the hydrostatic dimple under a steady pressure, the capillary-gravity dispersion
// relation, and the patch that follows a drone.
#include "lagrange/hydrodynamics/Surface.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::hydrodynamics {
namespace {

constexpr double kG = 9.80665;
constexpr double kWater = 1000.0;  // kg/m³
constexpr double kAir = 1.2;       // kg/m³, the air of the dimple's reference numbers

/// The patch's cell centre nearest (x, y).
std::size_t cellAt(const SurfacePatch& s, double x, double y) {
    const int n = s.cells();
    const int col = std::clamp(static_cast<int>((x - s.origin()[0]) / s.params().cell), 0, n - 1);
    const int row = std::clamp(static_cast<int>((y - s.origin()[1]) / s.params().cell), 0, n - 1);
    return static_cast<std::size_t>(row * n + col);
}

/// A round pressure p0·exp(−r²/b²) about (cx, cy), per cell.
std::vector<double> bump(const SurfacePatch& s, double p0, double b, double cx, double cy) {
    const int n = s.cells();
    std::vector<double> p(static_cast<std::size_t>(n * n));
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const double dx = s.origin()[0] + (x + 0.5) * s.params().cell - cx;
            const double dy = s.origin()[1] + (y + 0.5) * s.params().cell - cy;
            p[static_cast<std::size_t>(y * n + x)] = p0 * std::exp(-(dx * dx + dy * dy) / (b * b));
        }
    }
    return p;
}

SurfaceParams pond(int cells = 128, double cell = 0.03, double depth = 1.2) {
    SurfaceParams p;
    p.cells = cells;
    p.cell = cell;
    p.depth = depth;
    p.density = kWater;
    p.gravity = kG;
    return p;
}

TEST(SurfacePatch, StartsFlatAndStillAndStaysSoUnloaded) {
    SurfacePatch s(pond(32, 0.05));
    for (int i = 0; i < 100; ++i) {
        s.step(0.02, {});
    }
    EXPECT_TRUE(std::ranges::all_of(s.heights(), [](double h) { return h == 0.0; }));
    EXPECT_TRUE(std::ranges::all_of(s.rates(), [](double v) { return v == 0.0; }));
}

TEST(SurfacePatch, TheDimpleUnderASteadyJetIsHydrostatic) {
    // A steady stagnation pressure p = ½ρ_air U² presses a dimple ρ_air U²/(2ρ_w g) into the
    // water once the waves it started have left: about 2, 4, 6, 12 and 17 mm for jets of 5.7,
    // 8.1, 10.1, 13.7 and 16.6 m/s. Surface tension changes it by σk²/(ρg), under 0.1 % this wide.
    const std::vector<std::pair<double, double>> cases{
        {5.7, 2.0e-3}, {8.1, 4.0e-3}, {10.1, 6.2e-3}, {13.7, 11.5e-3}, {16.6, 16.9e-3}};
    for (const auto& [speed, rounded] : cases) {
        SurfacePatch s(pond());
        const double half = s.side() / 2.0;
        const double p0 = 0.5 * kAir * speed * speed;
        const std::vector<double> p = bump(s, p0, 0.25, half, half);
        for (int i = 0; i < 1500; ++i) {  // 30 s
            s.step(0.02, p);
        }
        // The dimple's depth below the water around it (a periodic patch keeps its volume, so
        // its far water stands a little higher than the still level).
        const double centre = s.heights()[cellAt(s, half, half)];
        const double far = s.heights()[cellAt(s, half + 1.2, half)];
        const double expected = p0 / (kWater * kG);
        EXPECT_NEAR((far - centre) / expected, 1.0, 0.02) << speed;
        EXPECT_NEAR(far - centre, rounded, 0.4e-3) << speed;
        // Settled: the water hardly moves any more.
        double fastest = 0.0;
        for (const double v : s.rates()) {
            fastest = std::max(fastest, std::abs(v));
        }
        EXPECT_LT(fastest, 0.02 * expected) << speed;  // under 2 % of the depth per second
    }
}

/// Group velocity of a wave packet with carrier k0, measured from its energy's centroid moving
/// along x, against dω/dk of ω² = (gk + σk³/ρ)·tanh(kd).
void checkGroupVelocity(double k0, double cell, double width, double seconds, double depth) {
    SurfaceParams params = pond(256, cell, depth);
    params.spongeShare = 0.05;
    SurfacePatch s(params);
    const int n = s.cells();
    const double side = s.side();
    const double x0 = 0.3 * side;
    const double w0 = SurfacePatch::omega(k0, kG, kSurfaceTension, kWater, depth);
    std::vector<double> eta(static_cast<std::size_t>(n * n));
    std::vector<double> rate(eta.size());
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const double dx = (x + 0.5) * cell - x0;
            const double envelope = 1e-4 * std::exp(-dx * dx / (width * width));
            eta[static_cast<std::size_t>(y * n + x)] = envelope * std::cos(k0 * dx);
            rate[static_cast<std::size_t>(y * n + x)] = envelope * w0 * std::sin(k0 * dx);
        }
    }
    s.setState({0.0, 0.0}, eta, rate);
    const auto centroid = [&] {
        double sum = 0.0;
        double moment = 0.0;
        const int y = n / 2;
        for (int x = 0; x < n; ++x) {
            const auto i = static_cast<std::size_t>(y * n + x);
            const double e = s.heights()[i] * s.heights()[i] +
                             s.rates()[i] * s.rates()[i] / (w0 * w0);  // energy, as amplitude²
            sum += e;
            moment += e * (x + 0.5) * cell;
        }
        return moment / sum;
    };
    const double start = centroid();
    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        s.step(seconds / steps, {});
    }
    const double measured = (centroid() - start) / seconds;
    const double dk = 1e-4 * k0;
    const double expected = (SurfacePatch::omega(k0 + dk, kG, kSurfaceTension, kWater, depth) -
                             SurfacePatch::omega(k0 - dk, kG, kSurfaceTension, kWater, depth)) /
                            (2.0 * dk);
    EXPECT_NEAR(measured / expected, 1.0, 0.03) << k0;
    // And the phase speed it carries: c = ω/k.
    const double phase = w0 / k0;
    if (k0 * depth > 3.0 && k0 < 50.0) {
        EXPECT_NEAR(expected / phase, 0.5, 0.02) << "deep-water gravity waves: c_g = c/2";
    }
    if (k0 > 2000.0) {
        EXPECT_NEAR(expected / phase, 1.5, 0.05) << "capillary waves: c_g = 3c/2";
    }
}

TEST(SurfacePatch, RipplesTravelAtTheCapillaryGravityGroupVelocity) {
    // Deep-water gravity waves (λ = 0.63 m), a capillary-gravity ripple near the slowest phase
    // speed (λ = 1.7 cm, k = √(ρg/σ)), and capillary waves (λ = 2 mm).
    checkGroupVelocity(10.0, 0.04, 1.0, 4.0, 5.0);
    checkGroupVelocity(std::sqrt(kWater * kG / kSurfaceTension), 0.002, 0.04, 0.4, 1.0);
    checkGroupVelocity(3000.0, 0.0002, 0.006, 0.02, 1.0);
}

TEST(SurfacePatch, ItsSlowestRippleIsTheKnownOne) {
    // The phase speed's minimum, √2·(gσ/ρ)^¼ ≈ 0.23 m/s at λ ≈ 1.7 cm (Lamb §267).
    const double km = std::sqrt(kWater * kG / kSurfaceTension);
    const double cMin = SurfacePatch::omega(km, kG, kSurfaceTension, kWater, 10.0) / km;
    EXPECT_NEAR(cMin, std::sqrt(2.0) * std::pow(kG * kSurfaceTension / kWater, 0.25), 1e-9);
    EXPECT_NEAR(cMin, 0.231, 0.002);
    EXPECT_NEAR(2.0 * std::numbers::pi / km, 0.0172, 0.0002);
    for (const double f : {0.8, 1.25}) {
        EXPECT_GT(SurfacePatch::omega(f * km, kG, kSurfaceTension, kWater, 10.0) / (f * km), cMin);
    }
}

TEST(SurfacePatch, ViscosityDampsRipplesAtTwoNuKSquared) {
    // A standing ripple of k = 300 /m decays as exp(−2νk²t) (Lamb §348).
    SurfaceParams params = pond(64, 2.0 * std::numbers::pi * 6.0 / 300.0 / 64.0, 1.0);
    params.spongeRate = 0.0;  // the standing wave fills the patch: no sponge
    SurfacePatch s(params);
    const int n = s.cells();
    const double k = 2.0 * std::numbers::pi * 6.0 / (n * params.cell);  // 6 waves on the patch
    std::vector<double> eta(static_cast<std::size_t>(n * n));
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            eta[static_cast<std::size_t>(y * n + x)] = 1e-4 * std::cos(k * (x + 0.5) * params.cell);
        }
    }
    s.setState({0.0, 0.0}, eta, std::vector<double>(eta.size(), 0.0));
    const double w = SurfacePatch::omega(k, kG, kSurfaceTension, kWater, 1.0);
    const double period = 2.0 * std::numbers::pi / w;
    const int periods = 40;
    for (int i = 0; i < periods * 10; ++i) {
        s.step(period / 10.0, {});
    }
    // Measure the amplitude by the energy over the middle row's centre half.
    double amplitude = 0.0;
    for (int x = n / 4; x < 3 * n / 4; ++x) {
        const auto i = static_cast<std::size_t>((n / 2) * n + x);
        amplitude = std::max(amplitude, std::hypot(s.heights()[i], s.rates()[i] / w));
    }
    const double expected = 1e-4 * std::exp(-2.0 * kWaterViscosity * k * k * periods * period);
    EXPECT_NEAR(amplitude / expected, 1.0, 0.05);
}

TEST(SurfacePatch, ThePatchFollowsByWholeCellsAndKeepsTheWaterUnderIt) {
    SurfacePatch s(pond(32, 0.1));
    const std::vector<double> p = bump(s, 50.0, 0.3, 1.6, 1.6);
    for (int i = 0; i < 50; ++i) {
        s.step(0.02, p);
    }
    const double before = s.heightAt(1.6, 1.6);
    EXPECT_LT(before, -1e-3);
    s.moveTo({0.52, -0.29});  // rounds to 5 and −3 cells
    EXPECT_NEAR(s.origin()[0], 0.5, 1e-12);
    EXPECT_NEAR(s.origin()[1], -0.3, 1e-12);
    EXPECT_NEAR(s.heightAt(1.6, 1.6), before, 1e-12);  // the same water, a new grid
    EXPECT_EQ(s.heightAt(-5.0, 0.0), 0.0);             // outside it: the still level
}

TEST(SurfacePatch, GridHeightIsTheBilinearOne) {
    const std::vector<double> h{0.0, 1.0, 2.0, 3.0};  // 2 × 2
    EXPECT_NEAR(gridHeight(h, 2, 1.0, {0.0, 0.0}, 0.5, 0.5), 0.0, 1e-12);
    EXPECT_NEAR(gridHeight(h, 2, 1.0, {0.0, 0.0}, 1.0, 1.0), 1.5, 1e-12);
    EXPECT_NEAR(gridHeight(h, 2, 1.0, {0.0, 0.0}, 1.5, 0.5), 1.0, 1e-12);
    EXPECT_EQ(gridHeight(h, 2, 1.0, {0.0, 0.0}, 3.0, 0.5), 0.0);
    EXPECT_EQ(gridHeight({}, 2, 1.0, {0.0, 0.0}, 1.0, 1.0), 0.0);
}

}  // namespace
}  // namespace lagrange::hydrodynamics
