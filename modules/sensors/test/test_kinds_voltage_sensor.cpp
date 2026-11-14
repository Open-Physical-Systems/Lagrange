// Kind voltage_sensor (sensors.md §9): the INA226's 1.25 mV bus step and 40.96 V full scale
// (SBOS547C §5.5), a divider into a 12-bit ADC and its range, NaN when nothing writes, ground
// truth.
#include "KindRig.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

TEST(VoltageSensor, Ina226ReadsInBusVoltageSteps) {
    Rig rig;
    rig.source("bus/main/voltage", units::V, 12.3456);
    rig.sensor("vbus", "voltage_sensor", 10.0, {}, {{"signal", "bus/main/voltage"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.0044));
    EXPECT_NEAR(rig.read("sensor/vbus/voltage"), 9876 * 1.25e-3, 1e-12);  // 12.345 V
    rig.set("bus/main/voltage", 45.0);
    ASSERT_TRUE(rig.runFor(0.0044));
    EXPECT_NEAR(rig.read("sensor/vbus/voltage"), 40.96, 1e-12);
}

TEST(VoltageSensor, ADividerIntoATwelveBitAdc) {
    Rig rig;
    rig.source("battery/pack/terminal_voltage", units::V, 16.8);
    rig.sensor("vbat", "voltage_sensor", 50.0, {{"ratio", 1.0 / 11.0}},
               {{"signal", "battery/pack/terminal_voltage"}, {"model", "divider"}});
    ASSERT_TRUE(rig.wire());
    ASSERT_TRUE(rig.runFor(0.05));
    const double lsb = 3.3 / 4096.0;
    EXPECT_NEAR(rig.read("sensor/vbat/voltage"), std::round(16.8 / 11.0 / lsb) * lsb * 11.0, 1e-9);
    rig.set("battery/pack/terminal_voltage", 50.0);  // 4.5 V at a 3.3 V ADC
    ASSERT_TRUE(rig.runFor(0.05));
    EXPECT_NEAR(rig.read("sensor/vbat/voltage"), 3.3 * 11.0, 1e-9);
}

TEST(VoltageSensor, MissingSignalReadsNaNAndGroundTruthIsExact) {
    {
        Rig rig;
        rig.sensor("vbus", "voltage_sensor", 10.0, {}, {{"signal", "bus/main/voltage"}});
        ASSERT_TRUE(rig.wire());
        ASSERT_TRUE(rig.runFor(0.01));
        EXPECT_TRUE(std::isnan(rig.read("sensor/vbus/voltage")));
        EXPECT_EQ(rig.read("sensor/vbus/valid"), 0.0);
    }
    {
        Rig rig;
        rig.source("bus/main/voltage", units::V, 12.3456);
        rig.sensor("vbus", "voltage_sensor", 10.0, {{"noise", 0.1}},
                   {{"signal", "bus/main/voltage"}});
        ASSERT_TRUE(rig.wire(true));
        ASSERT_TRUE(rig.runFor(0.01));
        EXPECT_NEAR(rig.read("sensor/vbus/voltage"), 12.3456, 1e-12);
    }
    {
        Rig rig;
        rig.sensor("vbus", "voltage_sensor", 10.0, {},
                   {{"signal", "bus/main/voltage"}, {"model", "hall"}});
        EXPECT_FALSE(rig.wire());  // hall is for currents
    }
}

}  // namespace
}  // namespace lagrange::sensors::test
