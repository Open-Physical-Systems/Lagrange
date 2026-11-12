// Kind esc_telemetry (sensors.md §9): KISS frames in 100 eRPM, 0.01 V, 0.01 A, 1 °C and mAh steps
// (Betaflight esc_sensor.c) at most one frame per 10 bytes at 115200 baud per ESC on the wire;
// bidirectional DShot's eRPM period in µs as a 9-bit mantissa shifted by a 3-bit exponent, and
// its extended frames (0.25 V, 1 A, 1 °C) once a second; eRPM = RPM × pole pairs.
#include "KindRig.hpp"

#include <cmath>
#include <numbers>

namespace lagrange::sensors::test {
namespace {

constexpr double kPolePairs = 7.0;
constexpr double kSpeed = 1000.0;  // rad/s at the joint (and the motor: no gear)

double erpmOf(double radPerS) {
    return radPerS * 60.0 / (2.0 * std::numbers::pi) * kPolePairs;
}
double speedOf(double erpm) {
    return erpm / kPolePairs * 2.0 * std::numbers::pi / 60.0;
}

void motor(Rig& rig) {
    rig.world.addPrim("/robot/rotor1", "rotor1");
    const PrimId joint{static_cast<std::uint32_t>(rig.world.prims.paths.size() - 1)};
    rig.addSchema("main", "ElectricalBusAPI_1");
    const PrimId bus{static_cast<std::uint32_t>(rig.world.prims.paths.size() - 1)};
    SchemaInstance& m = rig.addSchema("m1", "ElectricalMotorAPI_1");
    m.integers["elec:polePairs"] = static_cast<std::int64_t>(kPolePairs);
    m.relationships["elec:joint"] = {joint};
    m.relationships["elec:powerPort"] = {bus};
}

void signals(Rig& rig) {
    rig.source("joint/rotor1/velocity", units::rad_per_s, kSpeed);
    rig.source("motor/m1/bus_current", units::A, 12.347);
    rig.source("bus/main/voltage", units::V, 16.2);
    rig.source("thermal/m1/temperature", units::K, 330.4);
}

TEST(EscTelemetry, KissFramesAndTheirSteps) {
    Rig rig;
    motor(rig);
    signals(rig);
    rig.sensor("esc1", "esc_telemetry", 100.0, {}, {{"joint", "rotor1"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.1));
    // 66 845 eRPM → 668 × 100 eRPM.
    EXPECT_NEAR(rig.read("sensor/esc1/speed"), speedOf(66800.0), 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/voltage"), 16.20, 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/current"), 12.35, 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/temperature"), 57.0 + 273.15, 1e-9);  // 57.25 °C
    EXPECT_EQ(rig.read("sensor/esc1/valid"), 1.0);
}

TEST(EscTelemetry, KissConsumptionCountsWholeMilliampHours) {
    Rig rig;
    motor(rig);
    signals(rig);
    rig.set("motor/m1/bus_current", 36.0);  // 10 mAh a second
    rig.sensor("esc1", "esc_telemetry", 100.0, {}, {{"joint", "rotor1"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(1.0));
    const double c = rig.read("sensor/esc1/consumption");
    EXPECT_NEAR(c, 36.0, 3.6 + 1e-9);
    EXPECT_NEAR(std::round(c / 3.6) * 3.6, c, 1e-9);  // whole mAh
}

TEST(EscTelemetry, KissFramesAreLimitedByTheWire) {
    Rig rig;
    motor(rig);
    rig.sensor("esc1", "esc_telemetry", 1000.0, {{"escs", 4.0}}, {{"joint", "rotor1"}});
    auto module = createBuiltinKind(rig.world, 0, false, nullptr, nullptr);
    ASSERT_TRUE(module);
    // 4 ESCs × 100 bit times at 115 200 baud = 3.47 ms, rounded up to the kinds' whole
    // milliseconds (never faster than the wire).
    EXPECT_EQ(module->preferredStep().ns, 4'000'000);
}

TEST(EscTelemetry, DshotCarriesThePeriodAsMantissaAndExponent) {
    Rig rig;
    motor(rig);
    signals(rig);
    rig.sensor("esc1", "esc_telemetry", 1000.0, {}, {{"joint", "rotor1"}, {"protocol", "dshot"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.01));
    // 66 845 eRPM: a period of 897.6 µs, sent as 898 µs = 449 << 1.
    const double period = std::round(60e6 / erpmOf(kSpeed));
    ASSERT_EQ(period, 898.0);
    EXPECT_NEAR(rig.read("sensor/esc1/speed"), speedOf(60e6 / (449.0 * 2.0)), 1e-9);
    // Extended frames: 0.25 V, 1 A, 1 °C.
    EXPECT_NEAR(rig.read("sensor/esc1/voltage"), 16.25, 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/current"), 12.0, 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/temperature"), 57.0 + 273.15, 1e-9);
    EXPECT_TRUE(std::isnan(rig.read("sensor/esc1/consumption")));
    // They come once a second: a new voltage shows only after it.
    rig.set("bus/main/voltage", 14.0);
    ASSERT_TRUE(rig.runFor(0.5));
    EXPECT_NEAR(rig.read("sensor/esc1/voltage"), 16.25, 1e-9);
    ASSERT_TRUE(rig.runFor(0.6));
    EXPECT_NEAR(rig.read("sensor/esc1/voltage"), 14.0, 1e-9);
    // Slower than 511 << 7 µs per electrical turn (≈ 917 eRPM) reads stopped.
    rig.set("joint/rotor1/velocity", 10.0);  // 668 eRPM
    ASSERT_TRUE(rig.runFor(0.01));
    EXPECT_EQ(rig.read("sensor/esc1/speed"), 0.0);
}

TEST(EscTelemetry, WithoutPhysicsOrMotorSignals) {
    {
        Rig rig;
        motor(rig);
        rig.sensor("esc1", "esc_telemetry", 100.0, {}, {{"joint", "rotor1"}});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.05));
        EXPECT_TRUE(std::isnan(rig.read("sensor/esc1/speed")));
        EXPECT_TRUE(std::isnan(rig.read("sensor/esc1/current")));
        EXPECT_EQ(rig.read("sensor/esc1/valid"), 0.0);
    }
    {
        Rig rig;  // no motor prim: pole pairs must be given
        rig.sensor("esc1", "esc_telemetry", 100.0, {}, {{"joint", "rotor1"}});
        EXPECT_FALSE(rig.wire());
    }
    {
        Rig rig;
        rig.source("joint/rotor1/velocity", units::rad_per_s, kSpeed);
        rig.sensor("esc1", "esc_telemetry", 100.0, {{"pole_pairs", kPolePairs}},
                   {{"joint", "rotor1"}});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.05));
        EXPECT_NEAR(rig.read("sensor/esc1/speed"), speedOf(66800.0), 1e-9);
        EXPECT_TRUE(std::isnan(rig.read("sensor/esc1/voltage")));
    }
}

TEST(EscTelemetry, GroundTruthIsExact) {
    Rig rig;
    motor(rig);
    signals(rig);
    rig.sensor("esc1", "esc_telemetry", 100.0, {}, {{"joint", "rotor1"}});
    ASSERT_TRUE(rig.wire(true));
    ASSERT_TRUE(rig.runFor(0.05));
    EXPECT_NEAR(rig.read("sensor/esc1/speed"), kSpeed, 1e-9);
    EXPECT_NEAR(rig.read("sensor/esc1/current"), 12.347, 1e-12);
    EXPECT_NEAR(rig.read("sensor/esc1/temperature"), 330.4, 1e-9);
}

TEST(EscTelemetry, AGearRatioOfZeroIsRefused) {
    // gear_ratio 0 or below read a speed of 0 (or of the wrong sign) with valid 1.
    for (const double ratio : {0.0, -1.0}) {
        Rig rig;
        motor(rig);
        rig.sensor("esc1", "esc_telemetry", 100.0, {{"gear_ratio", ratio}}, {{"joint", "rotor1"}});
        std::string error;
        EXPECT_FALSE(rig.wire(false, &error)) << ratio;
        EXPECT_NE(error.find("gear_ratio"), std::string::npos) << error;
    }
}

}  // namespace
}  // namespace lagrange::sensors::test
