// Kind "airspeed" (sensors.md §9): a pitot-static probe reads q = ½ρV² along its axis, true and
// indicated airspeed from it, the weather's density before the site's.
#include "kinds_test_util.hpp"
#include "lagrange/sensors/EnvironmentSensors.hpp"

#include <cmath>

namespace lagrange::sensors {
namespace {

using namespace test;

TEST(Airspeed, Bernoulli) {
    EXPECT_NEAR(pitotPressure(1.225, 30.0), 0.5 * 1.225 * 900.0, 1e-9);
    EXPECT_NEAR(pitotPressure(1.225, -30.0), -0.5 * 1.225 * 900.0, 1e-9);
    EXPECT_NEAR(airspeedFromPressure(0.5 * 1.1 * 400.0, 1.1), 20.0, 1e-12);
    EXPECT_TRUE(std::isnan(airspeedFromPressure(100.0, 0.0)));
}

ModuleConfigData site(double density) {
    ModuleConfigData c;
    c.values["air_density"] = density;
    return c;
}

TEST(Airspeed, ForwardFlightInStillAirAtTheSiteDensity) {
    KindRun r;
    rig(r.world, "pitot", "airspeed", Pose{}, 50.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {20, 0, 0, 0, 0, 0}), true, site(1.2)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_NEAR(r.read("sensor/pitot/differential_pressure"), 0.5 * 1.2 * 400.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/pitot/airspeed"), 20.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/pitot/indicated_airspeed"), std::sqrt(480.0 / 1.225), 1e-9);
    EXPECT_EQ(r.read("sensor/pitot/valid"), 1.0);
}

TEST(Airspeed, TheWeathersDensityAndAHeadwind) {
    KindRun r;
    rig(r.world, "pitot", "airspeed", Pose{}, 50.0);
    std::vector<Feed> f = bodyFeeds(Pose{}, {20, 0, 0, 0, 0, 0});
    f.push_back(constant("env/air_density", SignalType::Scalar, units::kg_per_m3, {1.0}));
    f.push_back(constant("env/air_temperature", SignalType::Scalar, units::K, {280.0}));
    f.push_back(
        constant("env/wind/velocity", SignalType::Vec3, units::m_per_s, {-5, 3, 0}, "world_local"));
    ASSERT_TRUE(start(r, std::move(f), true, site(1.2)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    // 25 m/s into the probe; the crosswind is not along it.
    EXPECT_NEAR(r.read("sensor/pitot/differential_pressure"), 0.5 * 1.0 * 625.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/pitot/airspeed"), 25.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/pitot/temperature"), 280.0, 1e-12);
}

TEST(Airspeed, NoDensityAnywhereIsNotValid) {
    KindRun r;
    rig(r.world, "pitot", "airspeed", Pose{}, 50.0);
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {20, 0, 0, 0, 0, 0})));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    EXPECT_EQ(r.read("sensor/pitot/valid"), 0.0);
    EXPECT_TRUE(std::isnan(r.read("sensor/pitot/differential_pressure")));
}

TEST(Airspeed, NoiseAndCountsOfTheMS4525DO) {
    // One count of a ±1 psi MS4525DO is 2·6894.757 / (0.8·16383) Pa.
    const double count = 2.0 * 6894.757 / (0.8 * 16383.0);
    KindRun r;
    rig(r.world, "pitot", "airspeed", Pose{}, 100.0, {{"noise/stddev", 2.0}});
    ASSERT_TRUE(start(r, bodyFeeds(Pose{}, {15, 0, 0, 0, 0, 0}), false, site(1.225)));
    const double q = 0.5 * 1.225 * 225.0;
    double sum = 0.0;
    for (int k = 1; k <= 200; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 10'000'000LL}).ok());
        const double p = r.read("sensor/pitot/differential_pressure");
        EXPECT_NEAR(std::remainder(p, count), 0.0, 1e-9);
        EXPECT_NEAR(p, q, 5.0 * 2.0 + count);
        sum += p;
    }
    EXPECT_NEAR(sum / 200.0, q, 1.0);
}

TEST(Airspeed, AirRotorMustNameARotor) {
    // `air_rotor` reads rotor/<j>/air_velocity, an optional input: a name that is no rotor read
    // NaN for ever, taken as still air, and the probe went on valid with the craft's own motion
    // only. It is refused at start; a rotor's name is taken and read.
    for (const std::string rotor : {"rotor_9", "rotor_0"}) {
        KindRun r;
        const BodyId frame =
            rig(r.world, "pitot", "airspeed", Pose{}, 50.0, {}, {{"air_rotor", rotor}});
        const BodyId prop =
            r.world.addBody("prop", frame, Pose{}, 0.01, {}, {1e-4, 1e-4, 1e-4, 0, 0, 0});
        r.world.addJoint("rotor_0", JointType::Revolute, prop, {0, 0, 1}, true);
        SchemaInstance api;
        api.schema = "RotorAPI_1";
        api.prim = r.world.joints.prim.back();
        r.world.schemaInstances.push_back(api);
        std::vector<Feed> f = bodyFeeds(Pose{}, {});
        f.push_back(constant("rotor/rotor_0/air_velocity", SignalType::Vec3, units::m_per_s,
                             {-12, 0, 0}, "world_local"));
        f.push_back(constant("body/prop/pose", SignalType::Pose, units::m, {0, 0, 0, 1, 0, 0, 0},
                             "world_local"));
        if (rotor == "rotor_9") {
            r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
            r.scheduler->add(createBuiltinKind(r.world, 0, true, nullptr, nullptr));
            auto wired = r.scheduler->wire(WorldView(r.world));
            ASSERT_FALSE(wired);
            EXPECT_NE(to_string(wired.error().front()).find("air_rotor 'rotor_9'"),
                      std::string::npos);
            continue;
        }
        ASSERT_TRUE(start(r, std::move(f), true, site(1.2)));
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
        EXPECT_NEAR(r.read("sensor/pitot/airspeed"), 12.0, 1e-9);
    }
}

}  // namespace
}  // namespace lagrange::sensors
