// Kind tension_sensor (sensors.md §9): a load cell's rated output into an HX711 (24 bits over
// ±0.5·AVDD/128), its bow of nonlinearity × capacity at half load, hysteresis × capacity between
// loading and unloading there, the anchor end, NaN with the tether domain off, ground truth.
#include "KindRig.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

constexpr double kCapacity = 500.0;  // N

void tether(Rig& rig) {
    rig.addSchema("leash", "TetherAPI_1");
    rig.source("tether/leash/tension_attach", units::N, 0.0);
    rig.source("tether/leash/tension_anchor", units::N, 0.0);
}

TEST(TensionSensor, ReadsInTheAdcsSteps) {
    Rig rig;
    tether(rig);
    rig.set("tether/leash/tension_attach", 123.456);
    rig.sensor("cell", "tension_sensor", 10.0, {{"adc/noise", 0.0}, {"adc/bits", 12.0}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.25));
    // 2 mV/V × 5 V at 500 N; 12 bits over ±19.53 mV (0.5 × 5 V / 128).
    const double voltsPerN = 2e-3 * 5.0 / kCapacity;
    const double lsb = 2.0 * (0.5 * 5.0 / 128.0) / 4096.0;
    EXPECT_NEAR(rig.read("sensor/cell/force"),
                std::round(123.456 * voltsPerN / lsb) * lsb / voltsPerN, 1e-9);
    EXPECT_NEAR(rig.read("sensor/cell/strain"), rig.read("sensor/cell/force") / kCapacity * 1e-3,
                1e-15);
    EXPECT_EQ(rig.read("sensor/cell/valid"), 1.0);
}

TEST(TensionSensor, Hx711NoiseAtTheBridge) {
    Rig rig;
    tether(rig);
    rig.set("tether/leash/tension_attach", 200.0);
    rig.sensor("cell", "tension_sensor", 10.0, {});
    ASSERT_TRUE(rig.wire());
    const auto [mean, sigma] = statistics(rig, "sensor/cell/force", 3000, 0.1);
    const double expected = 50e-9 / (2e-3 * 5.0 / kCapacity);  // 50 nV rms in N
    EXPECT_NEAR(mean, 200.0, 5.0 * expected / std::sqrt(3000.0) + 1e-6);
    EXPECT_NEAR(sigma, expected, 0.06 * expected);
}

TEST(TensionSensor, NonlinearityAndHysteresisAtHalfLoad) {
    Rig rig;
    tether(rig);
    rig.sensor("cell", "tension_sensor", 10.0,
               {{"adc/noise", 0.0}, {"nonlinearity", 2e-3}, {"hysteresis", 1e-3}});
    ASSERT_TRUE(rig.wire());
    rig.set("tether/leash/tension_attach", 0.5 * kCapacity);  // loading
    ASSERT_TRUE(rig.runFor(0.25));
    const double up = rig.read("sensor/cell/force");
    rig.set("tether/leash/tension_attach", kCapacity);
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_NEAR(rig.read("sensor/cell/force"), kCapacity, 1e-3);  // no bow at full load
    rig.set("tether/leash/tension_attach", 0.5 * kCapacity);      // unloading
    ASSERT_TRUE(rig.runFor(0.25));
    const double down = rig.read("sensor/cell/force");
    EXPECT_NEAR(down - up, 1e-3 * kCapacity, 1e-3);
    EXPECT_NEAR(0.5 * (up + down) - 0.5 * kCapacity, 2e-3 * kCapacity, 1e-3);
}

TEST(TensionSensor, TheAnchorEnd) {
    Rig rig;
    tether(rig);
    rig.set("tether/leash/tension_attach", 10.0);
    rig.set("tether/leash/tension_anchor", 30.0);
    rig.sensor("cell", "tension_sensor", 10.0, {}, {{"end", "anchor"}});
    ASSERT_TRUE(rig.wire(true));
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_EQ(rig.read("sensor/cell/force"), 30.0);
}

TEST(TensionSensor, WithTheTetherDomainOffAndWithoutATether) {
    {
        Rig rig;
        rig.addSchema("leash", "TetherAPI_1");
        rig.sensor("cell", "tension_sensor", 10.0, {});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.25));
        EXPECT_TRUE(std::isnan(rig.read("sensor/cell/force")));
        EXPECT_EQ(rig.read("sensor/cell/valid"), 0.0);
    }
    {
        Rig rig;
        rig.sensor("cell", "tension_sensor", 10.0, {});
        EXPECT_FALSE(rig.wire());
    }
}

TEST(TensionSensor, GroundTruthIsExact) {
    Rig rig;
    tether(rig);
    rig.set("tether/leash/tension_attach", 123.456);
    rig.sensor("cell", "tension_sensor", 10.0, {{"noise", 5.0}, {"nonlinearity", 0.01}});
    ASSERT_TRUE(rig.wire(true));
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_EQ(rig.read("sensor/cell/force"), 123.456);
}

}  // namespace
}  // namespace lagrange::sensors::test
