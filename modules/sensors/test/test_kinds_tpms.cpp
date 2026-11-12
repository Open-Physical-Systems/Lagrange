// tpms (sensors.md §9): Gay-Lussac on a cold morning, a slow leak reaching the FMVSS No. 138
// warning at 75 % of the placard, and nothing without an air temperature.
#include "more_kinds_util.hpp"

#include <cmath>

namespace lagrange::sensors::test {
namespace {

/// A car body with four WheelAPI_1 wheel joints, a TPMS on it; another car's wheel apart.
KindRun car(std::map<std::string, double, std::less<>> settings, std::function<void(Writer&)> air) {
    KindRun r;
    World& w = r.world;
    const BodyId chassis = movingBody(w, "chassis");
    for (const char* name : {"fl", "fr", "rl", "rr"}) {
        const BodyId wheel = w.addBody(std::string(name) + "_wheel", chassis, Pose{}, 10.0, {},
                                       {0.5, 0.5, 0.5, 0, 0, 0});
        const JointId j = w.addJoint(name, JointType::Revolute, wheel, {0.0, 1.0, 0.0}, true);
        SchemaInstance s;
        s.schema = "WheelAPI_1";
        s.prim = w.joints.prim[j.index];
        w.schemaInstances.push_back(s);
    }
    const BodyId other = movingBody(w, "other_car");
    const BodyId otherWheel = w.addBody("ow", other, Pose{}, 10.0, {}, {0.5, 0.5, 0.5, 0, 0, 0});
    const JointId oj = w.addJoint("other_wheel", JointType::Revolute, otherWheel, {0, 1, 0}, true);
    SchemaInstance s;
    s.schema = "WheelAPI_1";
    s.prim = w.joints.prim[oj.index];
    w.schemaInstances.push_back(s);
    w.addCustomSensor("tpms", "tpms", chassis, Pose{}, 1.0, std::move(settings), {});
    auto out = std::make_unique<Writer>();
    if (air) {
        air(*out);
    }
    r.wire(std::move(out));
    return r;
}

void coldMorning(Writer& w) {
    w.add("env/air_temperature", SignalType::Scalar, units::K,
          [](double) { return std::vector<double>{273.15}; });
    w.add("env/pressure", SignalType::Scalar, units::Pa,
          [](double) { return std::vector<double>{101325.0}; });
}

TEST(Tpms, ColdAirLowersThePressure) {
    // Filled to 230 kPa at 20 °C; at 0 °C: (230 + 101.325)·273.15/293.15 − 101.325 kPa.
    KindRun r = car({}, coldMorning);
    ASSERT_TRUE(r.runTo(2.5));
    const auto p = r.readN("sensor/tpms/pressure");
    ASSERT_EQ(p.size(), 4U);  // its own four wheels, not the other car's
    const double expected = (230e3 + 101325.0) * 273.15 / 293.15 - 101325.0;
    for (const double v : p) {
        EXPECT_NEAR(v, expected, 1e-6);
    }
    EXPECT_NEAR(expected, 207.4e3, 0.1e3);  // 22.6 kPa down: a cold-morning warning light's cause
    EXPECT_EQ(r.readN("sensor/tpms/temperature")[0], 273.15);
    EXPECT_EQ(r.read("sensor/tpms/warning"), 0.0);
    EXPECT_EQ(r.read("sensor/tpms/valid"), 1.0);
}

TEST(Tpms, ALeakWarnsAtAQuarterBelowThePlacard) {
    // At 20 °C, 1 kPa/s: the gauge reads 230 − t kPa; the warning at 172.5 kPa, t = 57.5 s.
    KindRun r = car({{"leak_rate", 1000.0}, {"temperature", 293.15}}, nullptr);
    ASSERT_TRUE(r.runTo(56.5));
    EXPECT_NEAR(r.readN("sensor/tpms/pressure")[0], 230e3 - 57e3, 1e-6);  // 57 steps of 1 s
    EXPECT_EQ(r.read("sensor/tpms/warning"), 0.0);
    ASSERT_TRUE(r.runTo(58.5));
    EXPECT_EQ(r.read("sensor/tpms/warning"), 1.0);
}

TEST(Tpms, NothingWithoutAnAirTemperature) {
    KindRun r = car({}, nullptr);
    ASSERT_TRUE(r.runTo(2.5));
    EXPECT_TRUE(std::isnan(r.readN("sensor/tpms/pressure")[0]));
    EXPECT_EQ(r.read("sensor/tpms/valid"), 0.0);
}

}  // namespace
}  // namespace lagrange::sensors::test
