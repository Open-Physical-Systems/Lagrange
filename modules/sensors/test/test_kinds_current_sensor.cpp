// Kind current_sensor (sensors.md §9): the INA226's 2.5 µV shunt step and ±81.92 mV range
// (SBOS547C §5.5), noise falling as √N with its averaging, an ACS712 read by a 12-bit ADC, NaN
// when nothing writes the signal, units checked, ground truth.
#include "KindRig.hpp"
#include "lagrange/services/Log.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::test {
namespace {

constexpr double kShunt = 0.002;       // Ω
constexpr double kShuntStep = 2.5e-6;  // V, INA226 1 LSB

double inaReads(double amps) {
    return std::round(amps * kShunt / kShuntStep) * kShuntStep / kShunt;
}

TEST(CurrentSensor, Ina226ReadsInShuntVoltageSteps) {
    Rig rig;
    rig.source("motor/m1/bus_current", units::A, 1.2345);
    rig.sensor("ina", "current_sensor", 10.0, {{"shunt_resistance", kShunt}},
               {{"signal", "motor/m1/bus_current"}});
    ASSERT_TRUE(rig.wire());
    for (const double amps : {1.2345, -0.5, 0.0013, 12.0}) {
        rig.set("motor/m1/bus_current", amps);
        ASSERT_TRUE(rig.runFor(0.0044));  // two readings of 2.2 ms (1.1 ms × shunt and bus)
        EXPECT_NEAR(rig.read("sensor/ina/current"), inaReads(amps), 1e-12) << amps;
    }
    EXPECT_NEAR(inaReads(1.2345), 1.235, 1e-12);
    rig.set("motor/m1/bus_current", 50.0);  // 100 mV across the shunt: over ±81.92 mV
    ASSERT_TRUE(rig.runFor(0.0044));
    EXPECT_NEAR(rig.read("sensor/ina/current"), 81.92e-3 / kShunt, 1e-9);
    EXPECT_EQ(rig.read("sensor/ina/valid"), 1.0);
}

TEST(CurrentSensor, AveragingNConversionsCutsTheNoiseByRootN) {
    const auto sigma = [](double averages) {
        Rig rig;
        rig.source("branch/b/current", units::A, 2.0);
        rig.sensor("ina", "current_sensor", 10.0,
                   {{"shunt_resistance", kShunt},
                    {"noise", 25e-6},  // V at the shunt, 10 steps
                    {"averages", averages},
                    {"conversion_time", 140e-6},
                    {"channels", 1.0}},
                   {{"signal", "branch/b/current"}});
        EXPECT_TRUE(rig.wire());
        return statistics(rig, "sensor/ina/current", 3000, 140e-6 * averages).second;
    };
    const double one = sigma(1.0);
    const double sixteen = sigma(16.0);
    EXPECT_NEAR(one, 25e-6 / kShunt, 0.06 * 25e-6 / kShunt);
    EXPECT_NEAR(one / sixteen, 4.0, 0.3);
}

TEST(CurrentSensor, Ina226ReadsTheMeanOverItsConversions) {
    Rig rig;
    rig.source("branch/b/current", units::A, 1.0);
    // 1.1 ms × 2 channels × 4 averages = 8.8 ms per reading, on the kinds' whole milliseconds
    // (9 ms) so that it lines up with the other modules.
    rig.sensor("ina", "current_sensor", 10.0, {{"shunt_resistance", kShunt}, {"averages", 4.0}},
               {{"signal", "branch/b/current"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.009 * 2));
    rig.set("branch/b/current", 3.0);
    ASSERT_TRUE(rig.runFor(0.0045));  // half the window at 3 A
    rig.set("branch/b/current", 1.0);
    ASSERT_TRUE(rig.runFor(0.0045));
    EXPECT_NEAR(rig.read("sensor/ina/current"), 2.0, 0.03);
}

TEST(CurrentSensor, Acs712ThroughATwelveBitAdc) {
    Rig rig;
    rig.source("motor/m1/bus_current", units::A, 2.0);
    rig.sensor("hall", "current_sensor", 100.0, {{"noise", 0.0}},
               {{"signal", "motor/m1/bus_current"}, {"model", "hall"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.05));
    // VCC/2 + 185 mV/A × 2 A = 2.87 V; 12 bits over 5 V.
    const double lsb = 5.0 / 4096.0;
    EXPECT_NEAR(rig.read("sensor/hall/current"), (std::round(2.87 / lsb) * lsb - 2.5) / 0.185,
                1e-9);
}

TEST(CurrentSensor, Acs712DefaultNoiseIsItsPeakToPeakOverSix) {
    Rig rig;
    rig.source("motor/m1/bus_current", units::A, 1.0);
    rig.sensor("hall", "current_sensor", 100.0, {{"adc/bits", 24.0}},
               {{"signal", "motor/m1/bus_current"}, {"model", "hall"}});
    ASSERT_TRUE(rig.wire());
    const auto [mean, sigma] = statistics(rig, "sensor/hall/current", 3000, 0.01);
    const double expected = 0.021 / 6.0 / 0.185;  // A
    EXPECT_NEAR(mean, 1.0, 4.0 * expected / std::sqrt(3000.0));
    EXPECT_NEAR(sigma, expected, 0.06 * expected);
}

TEST(CurrentSensor, MissingSignalReadsNaNAndUnitsAreChecked) {
    {
        Rig rig;
        rig.sensor("ina", "current_sensor", 10.0, {}, {{"signal", "motor/m1/bus_current"}});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.01));
        EXPECT_TRUE(std::isnan(rig.read("sensor/ina/current")));
        EXPECT_EQ(rig.read("sensor/ina/valid"), 0.0);
    }
    {
        Rig rig;
        rig.source("bus/main/voltage", units::V, 12.0);
        rig.sensor("ina", "current_sensor", 10.0, {}, {{"signal", "bus/main/voltage"}});
        EXPECT_FALSE(rig.wire());  // V into a current sensor: refused at wiring
    }
    {
        Rig rig;
        rig.sensor("ina", "current_sensor", 10.0, {});
        EXPECT_FALSE(rig.wire());  // no signal named
    }
}

TEST(CurrentSensor, GroundTruthIsExact) {
    Rig rig;
    rig.source("motor/m1/bus_current", units::A, 1.2345);
    rig.sensor("ina", "current_sensor", 10.0, {{"noise", 1e-3}, {"offset", 1e-4}},
               {{"signal", "motor/m1/bus_current"}});
    ASSERT_TRUE(rig.wire(true));
    ASSERT_TRUE(rig.runFor(0.01));
    EXPECT_NEAR(rig.read("sensor/ina/current"), 1.2345, 1e-12);
}

TEST(CurrentSensor, ADeviceThatCannotMeasureIsRefused) {
    // A shunt of 0 Ω read NaN, a Hall sensitivity of 0 read inf, a negative ADC reference broke
    // std::clamp's precondition, all with valid 1. Each is refused by name.
    using Numbers = std::map<std::string, double, std::less<>>;
    using Text = std::map<std::string, std::string, std::less<>>;
    const std::vector<std::pair<Numbers, Text>> cases{
        {{{"shunt_resistance", 0.0}}, {}},
        {{{"shunt_resistance", -0.002}}, {}},
        {{{"sensitivity", 0.0}}, {{"model", "hall"}}},
        {{{"adc/reference", -1.0}}, {{"model", "hall"}}},
        {{{"adc/bits", 0.0}}, {{"model", "hall"}}},
        {{{"gain_error", -1.0}}, {}},
    };
    for (const auto& [numbers, text] : cases) {
        Rig rig;
        rig.source("motor/m1/bus_current", units::A, 2.0);
        Text t = text;
        t.emplace("signal", "motor/m1/bus_current");
        rig.sensor("ina", "current_sensor", 10.0, numbers, t);
        std::string error;
        EXPECT_FALSE(rig.wire(false, &error)) << numbers.begin()->first;
        EXPECT_NE(error.find("current_sensor ina"), std::string::npos) << error;
    }
}

// QA 2026-11-12 #13: a misspelt <signal> reads NaN (the input is optional, its domain may be
// off), and says so at the start instead of nothing.
TEST(CurrentSensor, ASignalNoModuleWritesIsReportedAtTheStart) {
    std::vector<std::string> warnings;
    services::setLogSink([&](services::LogLevel level, std::string_view, std::string_view text) {
        if (level == services::LogLevel::Warning) {
            warnings.emplace_back(text);
        }
    });
    Rig rig;
    rig.source("motor/m1/bus_current", units::A, 1.0);
    rig.sensor("ina", "current_sensor", 10.0, {}, {{"signal", "motor/m1/bus_curent"}});
    const bool wired = rig.wire();
    services::setLogSink({});
    ASSERT_TRUE(wired);
    ASSERT_TRUE(rig.runFor(0.01));
    EXPECT_TRUE(std::isnan(rig.read("sensor/ina/current")));
    ASSERT_EQ(warnings.size(), 1U);
    EXPECT_NE(warnings[0].find("motor/m1/bus_curent"), std::string::npos) << warnings[0];
    EXPECT_NE(warnings[0].find("ina"), std::string::npos) << warnings[0];
}

}  // namespace
}  // namespace lagrange::sensors::test
