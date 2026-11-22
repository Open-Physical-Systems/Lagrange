// RFC 0022 §Validation, the checks that need no sensors: each correlation against its closed
// form or the published value, and one fire's life (growth, water, foam, wind, burn-out) through
// Burner. Tolerances and sources are stated with each check.
#include "lagrange/fire/Correlations.hpp"
#include "lagrange/fire/Fires.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::fire {
namespace {

constexpr double kBtuPerSecond = 1055.056;  // W, International Table Btu per second

// t² growth: the four classes reach 1000 Btu/s (1055 kW) at 600, 300, 150 and 75 s. α for slow,
// medium and fast as NBSIR 85-3167 (Evans and Stroup 1985, the DETACT-T2 listing) sets them;
// ultrafast from its definition (TODO(verify) against NFPA 72's annex). Within 0.1 %.
TEST(Correlations, TSquaredClassesReach1055KilowattsOnTime) {
    const double target = 1000.0 * kBtuPerSecond;
    EXPECT_NEAR(tSquaredHrr(2.9306, 600.0), target, 1e-3 * target);
    EXPECT_NEAR(tSquaredHrr(11.722, 300.0), target, 1e-3 * target);
    EXPECT_NEAR(tSquaredHrr(46.889, 150.0), target, 1e-3 * target);
    EXPECT_NEAR(tSquaredHrr(target / (75.0 * 75.0), 75.0), target, 1e-3 * target);
    EXPECT_EQ(tSquaredHrr(46.889, -3.0), 0.0);  // before ignition
}

// Babrauskas' pool fire for a 1 m pool: Q = ṁ''(1 − e^(−kβD))ΔH_c·A. The inputs are examples
// (the presets ship empty, RFC 0022 Q3); the check is the closed form, 0.1 %:
// 0.101·(1 − e^(−1.1))·44.6e6·π/4 = 2.36024 MW.
TEST(Correlations, PoolFireIsBabrauskasClosedForm) {
    const double area = std::numbers::pi / 4.0;
    EXPECT_NEAR(equivalentDiameter(area), 1.0, 1e-12);
    EXPECT_NEAR(poolHrr(0.101, 1.1, 44.6e6, area), 2.360238e6, 1e-3 * 2.360238e6);
}

// Heskestad (SFPE Handbook 5th ed. ch. 13): at Q = 1 MW and D = 1 m, L = 0.235·1000^0.4 − 1.02 =
// 2.70450 m, z0 = 0.083·1000^0.4 − 1.02 = 0.29546 m, and with g = 9.81, c_p = 1005, T = 293.15,
// ρ = 1.2 and χ_r = 0.3 the centreline velocity at 5 m is 5.45444 m/s. 0.1 %.
TEST(Correlations, HeskestadFlameHeightOriginAndPlume) {
    EXPECT_NEAR(flameHeight(1e6, 1.0), 2.704499, 1e-3 * 2.704499);
    EXPECT_NEAR(virtualOrigin(1e6, 1.0), 0.295461, 1e-3 * 0.295461);
    EXPECT_NEAR(plumeVelocity(1e6, 0.3, 5.0, virtualOrigin(1e6, 1.0), 9.81, 1005.0, 293.15, 1.2),
                5.454436, 1e-3 * 5.454436);
    EXPECT_EQ(flameHeight(1e3, 2.0), 0.0);  // a low flame over a wide area: no negative height
    EXPECT_EQ(plumeVelocity(1e6, 0.3, 0.1, 0.3, 9.81, 1005.0, 293.15, 1.2), 0.0);
}

// The AGA tilt (Mudan and Croce, SFPE Handbook): cos θ = 1 for u* ≤ 1, u*^(−1/2) above; at
// ṁ'' = 0.05 kg/(m²·s), D = 1 m, ρ_a = 1.2, g = 9.81 the velocity scale is 0.742140 m/s.
TEST(Correlations, AgaTiltAtThreeWindSpeeds) {
    const double scale = 0.7421401;
    EXPECT_DOUBLE_EQ(tiltCosine(0.5 * scale, 0.05, 1.0, 9.81, 1.2), 1.0);
    EXPECT_NEAR(tiltCosine(2.0 * scale, 0.05, 1.0, 9.81, 1.2), 1.0 / std::sqrt(2.0), 1e-6);
    EXPECT_NEAR(tiltCosine(4.0 * scale, 0.05, 1.0, 9.81, 1.2), 0.5, 1e-6);
    EXPECT_EQ(tiltCosine(5.0, 0.0, 1.0, 9.81, 1.2), 1.0);  // nothing burning: upright
}

// The point-source model: q'' = χ_r·Q·cos β/(4πR²), and a lumped node absorbs ε·q''·A/4.
TEST(Correlations, PointSourceAndALumpedNode) {
    EXPECT_NEAR(pointSourceFlux(0.3, 1e6, 5.0), 0.3e6 / (4.0 * std::numbers::pi * 25.0), 1e-9);
    EXPECT_NEAR(pointSourceFlux(0.3, 1e6, 5.0, 0.5), 0.15e6 / (4.0 * std::numbers::pi * 25.0),
                1e-9);
    EXPECT_EQ(pointSourceFlux(0.3, 1e6, 5.0, -0.2), 0.0);  // facing away
    EXPECT_NEAR(absorbedByNode(0.8, 1000.0, 2.0), 400.0, 1e-12);
}

// Evans (NISTIR 5254, 1993, eq. 3): τ = 3.0·w''^(−1.85), 410 s at 0.07 mm/s as the report states
// for 0.61 m wood cribs (the correlation's own scatter is ±50 %; 0.5 % here is its arithmetic).
TEST(Correlations, EvansTauIs410SecondsAt007MillimetresPerSecond) {
    EXPECT_NEAR(evansTau(0.07), 410.0, 0.005 * 410.0);
    EXPECT_GT(evansTau(kEvansMinRate), evansTau(kEvansMaxRate));
}

// Water to steam from 20 °C: c_w·80 K + h_fg (2.256 MJ/kg at 100 °C, NIST WebBook), about
// 2.6 MJ/kg (RFC 0022 §5.1).
TEST(Correlations, WaterTakesUpAbout26MegajoulesPerKilogram) {
    EXPECT_NEAR(waterHeatPerKg(293.15), 2.6e6, 0.01 * 2.6e6);
    EXPECT_EQ(waterHeatPerKg(0.0), kLatentHeatWater);
}

TEST(Correlations, BurnOutAndCurve) {
    EXPECT_EQ(burnOutFactor(5.0, 10.0, 0.2), 1.0);
    EXPECT_NEAR(burnOutFactor(1.0, 10.0, 0.2), 0.5, 1e-12);
    EXPECT_EQ(burnOutFactor(0.0, 10.0, 0.2), 0.0);
    const std::vector<double> curve{0.0, 0.0, 10.0, 1000.0, 30.0, 0.0};
    EXPECT_EQ(curveHrr(curve, 5.0), 500.0);
    EXPECT_EQ(curveHrr(curve, 20.0), 500.0);
    EXPECT_EQ(curveHrr(curve, 40.0), 0.0);
}

/// A wood-crib-like class A item: fast growth, 500 kW/m² over 1 m², 50 kg. The values are
/// examples for the laws, not a fuel's data.
FireSpec crib() {
    FireSpec f;
    f.name = "crib";
    f.alpha = 46.889;
    f.hrrPerArea = 5e5;
    f.area = 1.0;
    f.fuelMass = 50.0;
    f.heatOfCombustion = 1.2e7;
    f.radiativeFraction = 0.3;
    f.sootYield = 0.015;
    f.criticalWaterRate = 0.03;
    f.ignitionTime = 0.0;
    return f;
}

constexpr double kDt = 0.1;  // s, the domain's step

/// Steps `b` from `t` until `end`, with `e` throughout.
double run(Burner& b, double t, double end, const Exposure& e) {
    for (; t < end - 1e-9; t += kDt) {
        b.step(t, kDt, e);
    }
    return t;
}

// The t² growth through Burner: Q = α·t² exactly at 60 s, capped at q''·A, then water at 0.1 mm/s
// (over the critical rate) decays it as exp(−Δt/τ), 0.1 %, and the water takes up no more than it
// can boil (RFC 0022 Validation "Water energy").
TEST(Burner, GrowsThenWaterDecaysItExponentially) {
    const FireSpec f = crib();
    Burner b(f, 0.01);
    double t = run(b, 0.0, 60.0, {});
    EXPECT_NEAR(b.hrr(), 46.889 * 3600.0, 1e-9 * b.hrr());
    EXPECT_EQ(b.state(), FireState::Growing);
    t = run(b, t, 120.0, {});
    EXPECT_EQ(b.hrr(), 5e5);  // full involvement: 46.889·120² would be 675 kW
    EXPECT_EQ(b.state(), FireState::FullyInvolved);
    const double before = b.hrr();
    Exposure water;
    water.waterRate = 0.1;
    water.waterTemperature = 293.15;
    const double tau = evansTau(0.1);
    double end = t + 100.0;
    for (; t < end - 1e-9; t += kDt) {
        b.step(t, kDt, water);
        const double mdot = water.waterRate * b.burningArea();
        EXPECT_LE(b.waterCooling(), mdot * waterHeatPerKg(293.15) + 1e-9);
        EXPECT_LE(b.waterCooling(), b.hrr() + 1e-9);
    }
    // Burnt meanwhile: 17 MJ of t² growth, 8.5 MJ at full, under 50 MJ decaying, so under
    // 6.5 kg of 50 and far from the decay fraction: only Evans' law acts.
    EXPECT_NEAR(b.hrr(), before * std::exp(-100.0 / tau), 1e-3 * b.hrr());
    EXPECT_EQ(b.state(), FireState::Decaying);
    EXPECT_GT(b.steamRate(), 0.0);
    // Without the water it grows again from where it is, along the t² law.
    const double low = b.hrr();
    b.step(t, kDt, {});
    const double clock = std::sqrt(low / 46.889) + kDt;
    EXPECT_NEAR(b.hrr(), 46.889 * clock * clock, 1e-9 * b.hrr());
}

// Below the critical rate the fire still grows (RFC 0022 §5.1); at the critical rate it goes out
// once it falls under 1 % of its HRR at the water's arrival, at t = τ·ln 100.
TEST(Burner, CriticalRateAndGoingOut) {
    const FireSpec f = crib();
    Burner b(f, 0.01);
    double t = run(b, 0.0, 30.0, {});
    Exposure weak;
    weak.waterRate = 0.02;  // under 0.03
    t = run(b, t, 40.0, weak);
    EXPECT_NEAR(b.hrr(), 46.889 * 1600.0, 1e-6 * b.hrr());
    Exposure enough;
    enough.waterRate = 0.133;
    const double tau = evansTau(0.133);
    const double start = t;
    while (!isOut(b.state()) && t < start + 2000.0) {
        b.step(t, kDt, enough);
        t += kDt;
    }
    EXPECT_EQ(b.state(), FireState::OutWater);
    EXPECT_NEAR(t - start, tau * std::log(100.0), kDt + 1e-9);
    EXPECT_EQ(b.hrr(), 0.0);
}

// Class B: water at any rate leaves a pool burning; foam at full coverage puts it out, after
// thickness/supply seconds, its burning area shrinking meanwhile (RFC 0022 §5.1, §5.3).
TEST(Burner, WaterLeavesAPoolBurningAndFoamSmothersIt) {
    FireSpec f;
    f.name = "tray";
    f.fireClass = FireClass::B;
    f.growth = Growth::Pool;
    f.poolBurningRate = 0.101;
    f.poolExtinction = 1.1;
    f.heatOfCombustion = 44.6e6;
    f.area = std::numbers::pi / 4.0;
    f.fuelMass = 200.0;
    f.radiativeFraction = 0.3;
    f.ignitionTime = 0.0;
    f.foamBlanketThickness = 0.1;
    Burner b(f, 0.01);
    double t = run(b, 0.0, 1.0, {});
    const double full = b.hrr();
    EXPECT_NEAR(full, 2.360238e6, 1e-3 * full);
    for (const double rate : {0.05, 0.5, 5.0}) {
        Exposure water;
        water.waterRate = rate;
        t = run(b, t, t + 20.0, water);
        EXPECT_EQ(b.state(), FireState::FullyInvolved) << rate;
    }
    Exposure foam;
    foam.foamSupply = 0.01;  // m/s: 0.1 m of foam in 10 s
    t = run(b, t, t + 5.0, foam);
    EXPECT_NEAR(b.foamCoverage(), 0.5, 1e-9);
    EXPECT_LT(b.hrr(), full);
    t = run(b, t, t + 5.0 + kDt, foam);
    EXPECT_EQ(b.state(), FireState::OutSmothered);
}

// Blow-off: a flame with a blow-off speed goes out at the step the air at it passes that speed,
// and not while its HRR is over blowOffMaxHrr (RFC 0022 Validation "Blow-off").
TEST(Burner, BlowsOffAtTheStepTheAirPassesItsSpeed) {
    FireSpec f = crib();
    f.blowOffSpeed = 6.0;
    f.blowOffMaxHrr = 50e3;
    Burner b(f, 0.01);
    double t = run(b, 0.0, 10.0, {});
    Exposure wind;
    wind.airSpeed = 5.9;
    t = run(b, t, 12.0, wind);
    EXPECT_FALSE(isOut(b.state()));
    wind.airSpeed = 6.1;
    b.step(t, kDt, wind);
    EXPECT_EQ(b.state(), FireState::OutBlownOff);
    // A fire over blowOffMaxHrr leans and burns on.
    Burner big(f, 0.01);
    t = run(big, 0.0, 60.0, {});  // 169 kW
    big.step(t, kDt, wind);
    EXPECT_FALSE(isOut(big.state()));
}

// Burn-out: the fuel burnt is ∫Q dt/ΔH_c; Q falls with the remaining mass over the last decay
// fraction (exponentially in time) and the fire is out at 1 % of that tail, having released
// fuelMass·ΔH_c less 0.2 % (0.5 %).
TEST(Burner, BurnsOutWithItsFuel) {
    FireSpec f = crib();
    f.fuelMass = 10.0;
    Burner b(f, 0.01);
    double t = 0.0;
    double released = 0.0;
    bool decayed = false;
    while (!isOut(b.state()) && t < 3600.0) {
        b.step(t, kDt, {});
        released += b.hrr() * kDt;
        decayed = decayed || b.state() == FireState::Decaying;
        t += kDt;
    }
    EXPECT_TRUE(decayed);
    EXPECT_EQ(b.state(), FireState::OutBurnt);
    EXPECT_NEAR(released, 10.0 * 1.2e7, 0.005 * 10.0 * 1.2e7);
    EXPECT_LE(b.fuelRemaining(), 0.01 * 0.2 * 10.0 + 1e-9);
}

// Class C: water on energised equipment raises the hazard flag.
TEST(Burner, WaterOnClassCIsAHazard) {
    FireSpec f = crib();
    f.fireClass = FireClass::C;
    Burner b(f, 0.01);
    double t = run(b, 0.0, 10.0, {});
    EXPECT_FALSE(b.hazard());
    Exposure water;
    water.waterRate = 0.05;
    b.step(t, kDt, water);
    EXPECT_TRUE(b.hazard());
}

// A saved burner restored elsewhere continues identically (determinism, invariant 7).
TEST(Burner, SaveAndRestoreContinueIdentically) {
    const FireSpec f = crib();
    Burner a(f, 0.01);
    double t = run(a, 0.0, 70.0, {});
    std::vector<double> state;
    a.save(state);
    Burner b(f, 0.01);
    ASSERT_TRUE(b.restore(state));
    Exposure water;
    water.waterRate = 0.1;
    for (int i = 0; i < 100; ++i, t += kDt) {
        a.step(t, kDt, water);
        b.step(t, kDt, water);
        ASSERT_EQ(a.hrr(), b.hrr());
    }
}

// Two discs: the lens of two unit circles a radius apart is 2π/3 − √3/2; one inside the other
// shares the smaller; apart, nothing. The spray's share is that over its own disc.
TEST(Spray, TheOverlapOfTwoDiscsIsTheLens) {
    EXPECT_NEAR(discOverlapArea(1.0, 1.0, 1.0), 2.0 * std::numbers::pi / 3.0 - std::sqrt(3.0) / 2.0,
                1e-12);
    EXPECT_NEAR(discOverlapArea(0.2, 0.1, 1.0), std::numbers::pi * 0.01, 1e-15);
    EXPECT_EQ(discOverlapArea(2.1, 1.0, 1.0), 0.0);
    EXPECT_NEAR(discOverlapArea(0.0, 0.3, 0.5), std::numbers::pi * 0.09, 1e-15);
    // Symmetric in the radii, and continuous where one disc starts to leave the other.
    EXPECT_NEAR(discOverlapArea(0.7, 0.3, 1.0), discOverlapArea(0.7, 1.0, 0.3), 1e-15);
    EXPECT_NEAR(discOverlapArea(0.7 + 1e-9, 0.3, 1.0), std::numbers::pi * 0.09, 1e-6);
    // Half a small disc on the edge of a large one: close to ½ (the edge curves away).
    EXPECT_NEAR(sprayHitFraction(10.0, 0.1, 10.0), 0.5, 0.005);
    EXPECT_EQ(sprayHitFraction(0.0, 0.0, 0.5), 1.0);
    EXPECT_EQ(sprayHitFraction(0.6, 0.0, 0.5), 0.0);
    EXPECT_EQ(sprayHitFraction(0.2, 0.1, 0.5), 1.0);
    EXPECT_EQ(sprayHitFraction(5.0, 0.1, 0.5), 0.0);
}

}  // namespace
}  // namespace lagrange::fire
