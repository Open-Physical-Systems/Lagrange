// Kind battery_monitor (sensors.md §9): SBS 1.1 register steps, an ADC's step from bits and full
// scale, remaining capacity = SOC × capacity, NaN with the electrical domain off, ground truth.
#include "KindRig.hpp"
#include "lagrange/services/Log.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lagrange::sensors::test {
namespace {

constexpr double kCapacityC = 18000.0;  // 5 A·h

void battery(Rig& rig, const std::string& name = "pack") {
    rig.addSchema(name, "BatteryAPI_1").reals["elec:battery:capacity"] = kCapacityC;
}

void electrical(Rig& rig) {
    rig.source("battery/pack/terminal_voltage", units::V, 16.12345);
    rig.source("battery/pack/current", units::A, 3.21987);
    rig.source("battery/pack/soc", units::one, 0.73456);
    rig.source("thermal/pack/temperature", units::K, 301.234);
}

TEST(BatteryMonitor, ReadsInSmartBatteryRegisterSteps) {
    Rig rig;
    battery(rig);
    electrical(rig);
    rig.sensor("gauge", "battery_monitor", 10.0, {});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_NEAR(rig.read("sensor/gauge/voltage"), 16.123, 1e-9);     // mV
    EXPECT_NEAR(rig.read("sensor/gauge/current"), 3.220, 1e-9);      // mA
    EXPECT_NEAR(rig.read("sensor/gauge/soc"), 0.73, 1e-9);           // %
    EXPECT_NEAR(rig.read("sensor/gauge/temperature"), 301.2, 1e-9);  // 0.1 K
    // mAh: round(0.73456 × 5000 mAh) = 3673 mAh.
    EXPECT_NEAR(rig.read("sensor/gauge/remaining_capacity"), 3673 * 3.6, 1e-6);
    EXPECT_EQ(rig.read("sensor/gauge/valid"), 1.0);
}

TEST(BatteryMonitor, AnAdcStepIsFullScaleOverTwoToTheBits) {
    Rig rig;
    battery(rig);
    electrical(rig);
    rig.sensor("gauge", "battery_monitor", 10.0,
               {{"voltage/bits", 12.0}, {"voltage/full_scale", 25.2}});
    ASSERT_TRUE(rig.wire());
    const double lsb = 25.2 / 4096.0;
    for (const double v : {16.12345, 3.3, 0.001, 25.0}) {
        rig.set("battery/pack/terminal_voltage", v);
        ASSERT_TRUE(rig.runFor(0.1));
        const double read = rig.read("sensor/gauge/voltage");
        EXPECT_NEAR(read, std::round(v / lsb) * lsb, 1e-12) << v;
        EXPECT_LE(std::abs(read - v), lsb / 2.0 + 1e-12) << v;
    }
    rig.set("battery/pack/terminal_voltage", 30.0);  // over the range: saturates
    ASSERT_TRUE(rig.runFor(0.1));
    EXPECT_NEAR(rig.read("sensor/gauge/voltage"), 25.2, 1e-12);
}

TEST(BatteryMonitor, NoiseHasItsSigmaAndRepeatsWithTheSeed) {
    const auto run = [] {
        Rig rig;
        battery(rig);
        electrical(rig);
        rig.sensor("gauge", "battery_monitor", 100.0,
                   {{"voltage/noise", 0.01}, {"voltage/resolution", 0.0}});
        EXPECT_TRUE(rig.wire());
        return statistics(rig, "sensor/gauge/voltage", 4000, 0.01);
    };
    const auto [mean, sigma] = run();
    EXPECT_NEAR(mean, 16.12345, 4.0 * 0.01 / std::sqrt(4000.0));
    EXPECT_NEAR(sigma, 0.01, 0.1 * 0.01);
    const auto [mean2, sigma2] = run();
    EXPECT_EQ(mean, mean2);  // the CPU reference is bit-reproducible
    EXPECT_EQ(sigma, sigma2);
}

TEST(BatteryMonitor, ReadsNothingWithTheElectricalDomainOff) {
    Rig rig;
    battery(rig);
    rig.sensor("gauge", "battery_monitor", 10.0, {});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_TRUE(std::isnan(rig.read("sensor/gauge/voltage")));
    EXPECT_TRUE(std::isnan(rig.read("sensor/gauge/remaining_capacity")));
    EXPECT_EQ(rig.read("sensor/gauge/valid"), 0.0);
}

TEST(BatteryMonitor, WithoutAThermalNodeTheTemperatureIsNaN) {
    Rig rig;
    battery(rig);
    rig.source("battery/pack/terminal_voltage", units::V, 16.0);
    rig.sensor("gauge", "battery_monitor", 10.0, {});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_NEAR(rig.read("sensor/gauge/voltage"), 16.0, 1e-9);
    EXPECT_TRUE(std::isnan(rig.read("sensor/gauge/temperature")));
    EXPECT_EQ(rig.read("sensor/gauge/valid"), 1.0);
}

TEST(BatteryMonitor, SeveralBatteriesNeedANameAndNoneIsAnError) {
    {
        Rig rig;
        battery(rig, "pack");
        battery(rig, "spare");
        rig.sensor("gauge", "battery_monitor", 10.0, {});
        std::string error;
        EXPECT_FALSE(rig.wire(false, &error));
        EXPECT_NE(error.find("several batteries"), std::string::npos) << error;
    }
    {
        Rig rig;
        battery(rig, "pack");
        battery(rig, "spare");
        electrical(rig);
        rig.sensor("gauge", "battery_monitor", 10.0, {}, {{"battery", "pack"}});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.25));
        EXPECT_NEAR(rig.read("sensor/gauge/voltage"), 16.123, 1e-9);
    }
    {
        Rig rig;
        rig.sensor("gauge", "battery_monitor", 10.0, {});
        EXPECT_FALSE(rig.wire());
    }
}

TEST(BatteryMonitor, GroundTruthIsExact) {
    Rig rig;
    battery(rig);
    electrical(rig);
    rig.sensor("gauge", "battery_monitor", 10.0, {{"voltage/noise", 0.5}, {"soc/offset", 0.1}});
    ASSERT_TRUE(rig.wire(true));
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_EQ(rig.read("sensor/gauge/voltage"), 16.12345);
    EXPECT_EQ(rig.read("sensor/gauge/soc"), 0.73456);
    EXPECT_EQ(rig.read("sensor/gauge/remaining_capacity"), 0.73456 * kCapacityC);
}

// QA 2026-11-12 #13: a misspelt <battery> reads NaN and says so at the start.
TEST(BatteryMonitor, ABatteryNoModuleWritesIsReportedAtTheStart) {
    std::vector<std::string> warnings;
    services::setLogSink([&](services::LogLevel level, std::string_view, std::string_view text) {
        if (level == services::LogLevel::Warning) {
            warnings.emplace_back(text);
        }
    });
    Rig rig;
    battery(rig);
    electrical(rig);
    rig.sensor("gauge", "battery_monitor", 10.0, {}, {{"battery", "pakc"}});
    const bool wired = rig.wire();
    services::setLogSink({});
    ASSERT_TRUE(wired);
    ASSERT_TRUE(rig.runFor(0.25));
    EXPECT_EQ(rig.read("sensor/gauge/valid"), 0.0);
    ASSERT_EQ(warnings.size(), 1U);
    EXPECT_NE(warnings[0].find("pakc"), std::string::npos) << warnings[0];
}

}  // namespace
}  // namespace lagrange::sensors::test
