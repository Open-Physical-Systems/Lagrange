// Heat conduction in solids (RFC 0012, Conduction.hpp) against closed-form solutions:
// the lumped limit, a fin with a convecting tip, a semi-infinite solid under a constant surface
// flux, two blocks across a contact conductance, an orthotropic fin, grid convergence, and
// bit-reproducibility.
#include "lagrange/thermal/Conduction.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::thermal {
namespace {

constexpr double kAir = 300.0;  // K

SolidShape box(double hx, double hy, double hz, std::array<double, 3> at = {0.0, 0.0, 0.0}) {
    SolidShape s;
    s.type = ShapeType::Box;
    s.size = {hx, hy, hz};
    s.pose.position = at;
    return s;
}

ThermalMaterial material(double k, double rho, double cp) {
    return {k, rho, cp, 0.0, std::nullopt};  // emissivity 0: convection only
}

ConductionModel build(std::vector<SolidPart> parts, std::size_t maxCells = 400000) {
    std::vector<std::string> notes;
    std::string error;
    auto model = ConductionModel::build(std::move(parts), maxCells, notes, error);
    EXPECT_TRUE(model) << error;
    for (const std::string& note : notes) {
        std::printf("note: %s\n", note.c_str());
    }
    return model ? std::move(*model) : ConductionModel{};
}

/// Runs to `seconds` in steps of `dt` with every part still and the given source heats.
void run(ConductionModel& m, std::vector<double>& t, std::vector<double> heats, double seconds,
         double dt, double h) {
    std::vector<PartInput> in(m.parts().size());
    const Surroundings around{kAir, kAir, 0.0};
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int i = 0; i < steps; ++i) {
        m.step(t, in, heats, around, dt, h);
    }
}

// A small copper cube (Biot ≪ 0.1) cooling in air: exp(−hA·t/(ρcV)).
TEST(Conduction, ASmallCopperCubeCoolsLikeALumpedBody) {
    SolidPart cube;
    cube.name = "cube";
    cube.shapes = {box(0.005, 0.005, 0.005)};
    cube.material = material(401.0, 8933.0, 385.0);
    cube.initialTemperature = 350.0;
    cube.cell = 0.002;
    ConductionModel m = build({cube});
    std::vector<double> t = m.initialTemperatures();
    constexpr double kH = 10.0;  // W/(m²·K)
    run(m, t, {}, 600.0, 0.5, kH);
    const double side = 0.01;
    const double tau = 8933.0 * 385.0 * side * side * side / (kH * 6.0 * side * side);
    const double expect = kAir + 50.0 * std::exp(-600.0 / tau);
    std::printf("cube after 600 s: %.4f K, lumped %.4f K\n", m.mean(t, 0), expect);
    EXPECT_NEAR(m.mean(t, 0), expect, 0.005 * (expect - kAir));
    EXPECT_LT(m.maximum(t, 0) - m.minimum(t, 0), 0.01);  // isothermal
}

struct FinErrors {
    double shape{0.0};  // largest error of θ(x)/θ(base) along the rod
    double base{0.0};   // relative error of θ(base) against 1 W into the end face
};

/// A square aluminium rod along x, 1 W into its x = 0 end, steady: the fin with a convecting
/// tip (Incropera Table 3.4, case A), its end face convecting too.
FinErrors finErrors(double cell, double kAcross, std::optional<double> kAlong,
                    std::array<double, 3> axis = {1.0, 0.0, 0.0}) {
    constexpr double kW = 0.004;  // m, side
    constexpr double kL = 0.1;    // m, length
    constexpr double kH = 25.0;   // W/(m²·K)
    SolidPart rod;
    rod.name = "rod";
    rod.shapes = {box(kL / 2.0, kW / 2.0, kW / 2.0, {kL / 2.0, 0.0, 0.0})};
    rod.material = {kAcross, 2700.0, 896.0, 0.0, kAlong};
    rod.axis = axis;
    rod.initialTemperature = kAir;
    rod.cell = cell;
    rod.sources.push_back({{0.0, 0.0, 0.0}, {cell, kW, kW}});  // the first layer of cells
    ConductionModel m = build({rod});
    std::vector<double> t = m.initialTemperatures();
    run(m, t, {1.0}, 2e5, 1e4, kH);  // steady
    const double k = kAlong.value_or(kAcross);
    const double mm = std::sqrt(kH * 4.0 * kW / (k * kW * kW));
    const double tip = kH / (mm * k);
    // θ along the rod, from the first layer's centre (the base) to the tip face.
    const double base = cell / 2.0;
    const double length = kL - base;
    const auto& g = m.grid(0);
    double worst = 0.0;
    const auto at = [&](int x) {  // mean over a cross-section layer
        double sum = 0.0;
        int n = 0;
        for (int z = 0; z < g.dims[2]; ++z) {
            for (int y = 0; y < g.dims[1]; ++y) {
                const auto c =
                    g.cellAt[(static_cast<std::size_t>(z) * g.dims[1] + y) * g.dims[0] + x];
                if (c >= 0) {
                    sum += t[g.first + static_cast<std::size_t>(c)] - kAir;
                    ++n;
                }
            }
        }
        return sum / n;
    };
    const double thetaBase = at(0);
    for (int x = 1; x < g.dims[0]; ++x) {
        const double s = (x + 0.5) * g.cell - base;
        const double expect = (std::cosh(mm * (length - s)) + tip * std::sinh(mm * (length - s))) /
                              (std::cosh(mm * length) + tip * std::sinh(mm * length));
        worst = std::max(worst, std::abs(at(x) / thetaBase - expect));
    }
    // Energy: at steady state all of the source leaves through the faces.
    EXPECT_NEAR(m.heatOut(0), 1.0, 1e-6);
    // θ at x = 0 for 1 W in: through the end face h·Ac·θ, and into the fin of length L.
    const double fin = std::sqrt(kH * 4.0 * kW * k * kW * kW) *
                       (std::sinh(mm * kL) + tip * std::cosh(mm * kL)) /
                       (std::cosh(mm * kL) + tip * std::sinh(mm * kL));
    const double expectBase = 1.0 / (kH * kW * kW + fin);
    const FinErrors e{worst, std::abs(thetaBase - expectBase) / expectBase};
    std::printf("fin, cells of %.2f mm: θ(base) %.3f K (closed form %.3f), errors %.5f, %.5f\n",
                g.cell * 1000.0, thetaBase, expectBase, e.shape, e.base);
    return e;
}

double finError(double cell, double kAcross, std::optional<double> kAlong,
                std::array<double, 3> axis = {1.0, 0.0, 0.0}) {
    return finErrors(cell, kAcross, kAlong, axis).shape;
}

TEST(Conduction, ARodMatchesTheFinSolution) {
    EXPECT_LT(finError(0.001, 167.0, std::nullopt), 0.01);
}

// The base temperature converges to the closed form at first order: the heat goes into the
// first layer of cells, half a cell from the face, rather than through the face itself.
TEST(Conduction, ErrorFallsWithTheCellSize) {
    const double coarse = finErrors(0.002, 167.0, std::nullopt).base;
    const double fine = finErrors(0.001, 167.0, std::nullopt).base;
    const double finer = finErrors(0.0005, 167.0, std::nullopt).base;
    EXPECT_LT(fine, coarse * 0.6);
    EXPECT_LT(finer, fine * 0.6);
    EXPECT_LT(finer, 0.01);
}

// Orthotropic: 1 W/(m·K) across, 167 along x: the same fin as isotropic 167.
TEST(Conduction, AnOrthotropicRodConductsAlongItsAxis) {
    EXPECT_LT(finError(0.001, 1.0, 167.0), 0.02);
    // Turned so the axis is across: the rod hardly conducts along its length.
    const double across = finError(0.001, 1.0, 167.0, {0.0, 0.0, 1.0});
    EXPECT_GT(across, 0.1);
}

// A long steel bar with a constant flux into one end, insulated elsewhere: the semi-infinite
// solid (Carslaw and Jaeger, Conduction of Heat in Solids, §2.9; Incropera Eq. 5.59).
TEST(Conduction, ASemiInfiniteSolidUnderAConstantFlux) {
    constexpr double kK = 14.9;
    constexpr double kRho = 7900.0;
    constexpr double kCp = 477.0;
    constexpr double kCell = 0.0005;
    constexpr double kSide = 0.004;
    constexpr double kFlux = 1e5;  // W/m²
    SolidPart bar;
    bar.name = "bar";
    bar.shapes = {box(0.03, kSide / 2.0, kSide / 2.0, {0.03, 0.0, 0.0})};
    bar.material = material(kK, kRho, kCp);
    bar.initialTemperature = kAir;
    bar.cell = kCell;
    bar.sources.push_back({{0.0, 0.0, 0.0}, {kCell, kSide, kSide}});
    ConductionModel m = build({bar});
    std::vector<double> t = m.initialTemperatures();
    constexpr double kTime = 10.0;
    run(m, t, {kFlux * kSide * kSide}, kTime, 0.005, 0.0);
    const double alpha = kK / (kRho * kCp);
    const auto expect = [&](double x) {
        const double r = std::sqrt(alpha * kTime);
        return 2.0 * kFlux / kK * r / std::sqrt(std::numbers::pi) *
                   std::exp(-x * x / (4.0 * alpha * kTime)) -
               kFlux * x / kK * std::erfc(x / (2.0 * r));
    };
    const auto& g = m.grid(0);
    for (const int x : {2, 6, 12}) {
        const double at =
            t[g.first +
              static_cast<std::size_t>(
                  g.cellAt[(static_cast<std::size_t>(g.dims[2] / 2) * g.dims[1] + g.dims[1] / 2) *
                               g.dims[0] +
                           x])] -
            kAir;
        // The flux goes into the first layer of cells, [0, cell]: deeper than it, the same as
        // a flux through the face at 0.
        const double want = expect((x + 0.5) * kCell);
        std::printf("x %.2f mm: %.3f K, closed form %.3f K\n", (x + 0.5) * kCell * 1000.0, at,
                    want);
        EXPECT_NEAR(at, want, 0.03 * want + 0.02);
    }
}

// Two copper cubes, stacked, across a contact conductance: the two-node network in steady state.
TEST(Conduction, TwoBlocksExchangeHeatThroughTheirContact) {
    constexpr double kSide = 0.01;
    constexpr double kContact = 2000.0;  // W/(m²·K)
    constexpr double kH = 10.0;
    SolidPart lower;
    lower.name = "lower";
    lower.shapes = {box(kSide / 2.0, kSide / 2.0, kSide / 2.0)};
    lower.material = material(401.0, 8933.0, 385.0);
    lower.initialTemperature = kAir;
    lower.cell = 0.001;
    SolidPart upper = lower;
    upper.name = "upper";
    upper.rest.position = {0.0, 0.0, kSide};
    upper.parent = 0;
    upper.contactConductance = kContact;
    upper.sources.push_back({{0.0, 0.0, 0.0}, {kSide, kSide, kSide}});
    ConductionModel m = build({lower, upper});
    std::vector<double> t = m.initialTemperatures();
    run(m, t, {1.0}, 1e5, 2e3, kH);
    const double area = kSide * kSide;
    const double loss = kH * 5.0 * area;  // each loses through its five free faces
    const double g = 1.0 / (1.0 / (kContact * area) + kSide / (401.0 * area));
    // q = loss·θu + g·(θu − θl), g·(θu − θl) = loss·θl.
    const double thetaLower = 1.0 / (loss * (2.0 + loss / g));
    const double thetaUpper = thetaLower * (1.0 + loss / g);
    std::printf("upper %.3f K (expect %.3f), lower %.3f K (expect %.3f)\n", m.mean(t, 1) - kAir,
                thetaUpper, m.mean(t, 0) - kAir, thetaLower);
    EXPECT_NEAR(m.mean(t, 1) - kAir, thetaUpper, 0.02 * thetaUpper);
    EXPECT_NEAR(m.mean(t, 0) - kAir, thetaLower, 0.02 * thetaLower);
    EXPECT_NEAR(m.heatOut(0) + m.heatOut(1), 1.0, 1e-6);
}

TEST(Conduction, TouchingWithoutAContactConductanceIsNoted) {
    SolidPart lower;
    lower.name = "lower";
    lower.shapes = {box(0.005, 0.005, 0.005)};
    lower.material = material(401.0, 8933.0, 385.0);
    lower.initialTemperature = kAir;
    SolidPart upper = lower;
    upper.name = "upper";
    upper.rest.position = {0.0, 0.0, 0.01};
    upper.parent = 0;
    std::vector<std::string> notes;
    std::string error;
    const auto m = ConductionModel::build({lower, upper}, 400000, notes, error);
    ASSERT_TRUE(m) << error;
    ASSERT_EQ(notes.size(), 1U);
    EXPECT_NE(notes[0].find("contactConductance"), std::string::npos);
}

// Natural convection: the hot side up loses more than the hot side down, and a 10 cm plate 50 K
// over the air is within the textbook's few W/(m²·K).
TEST(Conduction, NaturalConvectionCoefficients) {
    const double vertical = naturalConvection(350.0, 300.0, 0.0, 0.1, 0.025);
    const double up = naturalConvection(350.0, 300.0, 1.0, 0.1, 0.025);
    const double down = naturalConvection(350.0, 300.0, -1.0, 0.1, 0.025);
    std::printf("h: vertical %.2f, up %.2f, down %.2f W/(m²·K)\n", vertical, up, down);
    EXPECT_GT(up, down);
    EXPECT_GT(vertical, 3.0);
    EXPECT_LT(vertical, 10.0);
    EXPECT_EQ(naturalConvection(300.0, 300.0, 1.0, 0.1, 0.025), 0.0);
    // Forced flow at 5 m/s over 10 cm: laminar, about 20 W/(m²·K).
    const double forced = forcedConvection(5.0, 0.1);
    EXPECT_GT(forced, 15.0);
    EXPECT_LT(forced, 30.0);
}

// A convex part alone sees all of its surroundings; two plates 2 mm apart face each other on
// their inner sides, which see only the strip between them at the edges.
TEST(Conduction, FacesThatSeeEachOtherRadiateLess) {
    SolidPart plate;
    plate.name = "plate";
    plate.shapes = {box(0.02, 0.02, 0.001)};
    plate.material = {167.0, 2700.0, 896.0, 0.9, std::nullopt};
    plate.initialTemperature = kAir;
    plate.cell = 0.001;
    const ConductionModel alone = build({plate});
    EXPECT_NEAR(alone.skyView(0), 1.0, 1e-12);
    SolidPart other = plate;
    other.name = "other";
    other.rest.position = {0.0, 0.0, 0.004};  // 2 mm gap
    const ConductionModel pair = build({plate, other});
    // Of each plate's area, about half faces the other (its inner side), which sees a sliver:
    // the share it sees falls from 1 to a little over a half.
    std::printf("sky view: alone %.3f, facing another %.3f\n", alone.skyView(0), pair.skyView(0));
    EXPECT_LT(pair.skyView(0), 0.65);
    EXPECT_GT(pair.skyView(0), 0.5);
}

TEST(Conduction, IsBitReproducible) {
    const auto once = [] {
        SolidPart plate;
        plate.name = "plate";
        plate.shapes = {box(0.02, 0.02, 0.001)};
        plate.material = {0.81, 1850.0, 1100.0, 0.9, 0.29};
        plate.initialTemperature = kAir;
        plate.sources.push_back({{0.01, 0.0, 0.0}, {0.003, 0.003, 0.002}});
        ConductionModel m = build({plate});
        std::vector<double> t = m.initialTemperatures();
        std::vector<PartInput> in(1);
        for (int i = 0; i < 50; ++i) {
            m.step(t, in, std::vector<double>{2.0}, Surroundings{kAir, 280.0, 0.0}, 0.1);
        }
        return t;
    };
    EXPECT_EQ(once(), once());
}

}  // namespace
}  // namespace lagrange::thermal
