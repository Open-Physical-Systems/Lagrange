// GNSS, magnetometer and barometer (v0.4) against their references: the geo origin's own
// conversion, the World Magnetic Model rotated into the sensor, and the ICAO standard atmosphere.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/Wmm.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <memory>
#include <numbers>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

constexpr double kDegree = std::numbers::pi / 180.0;
const coords::Geodetic kZurich{47.397742 * kDegree, 8.545594 * kDegree, 488.0};

/// Writes a body's pose and twist, as physics would.
class Plant final : public IModule {
public:
    Pose pose{{100.0, -50.0, 20.0}, {1.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> twist{3.0, 4.0, 0.5, 0.0, 0.0, 0.0};
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "plant", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec p;
        p.name = "body/frame/pose";
        p.type = SignalType::Pose;
        p.unit = units::m;
        p.frame = "world_local";
        pose_ = d.output(p);
        SignalSpec t;
        t.name = "body/frame/twist";
        t.type = SignalType::Wrench;
        t.unit = units::m_per_s;
        t.frame = "world_local";
        twist_ = d.output(t);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus) const {
        bus.writeN(pose_,
                   std::array<double, 7>{pose.position[0], pose.position[1], pose.position[2],
                                         pose.orientation[0], pose.orientation[1],
                                         pose.orientation[2], pose.orientation[3]});
        bus.writeN(twist_, twist);
    }
    SignalId pose_, twist_;
};

/// A free frame with the three sensors, mounted 10 cm above its origin.
World rig(bool withOrigin = true) {
    World w;
    const BodyId frame = w.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("frame_free", JointType::Free, frame, {0, 0, 1}, false);
    const Pose mount{{0.0, 0.0, 0.1}};
    w.addSensor("gnss", SensorType::Gnss, frame, mount, 10.0, {});
    w.addSensor("magnetometer", SensorType::Magnetometer, frame, mount, 100.0, {});
    w.addSensor("barometer", SensorType::Barometer, frame, mount, 50.0, {});
    w.region.worldEnu = {1000.0, 2000.0, 0.0};  // a rebased region: positions add it back
    if (withOrigin) {
        w.geoOrigin = kZurich;
    }
    return w;
}

struct Rig {
    std::unique_ptr<Scheduler> scheduler;
    Plant* plant{nullptr};
    [[nodiscard]] double read(const std::string& name) const {
        return scheduler->bus().read(scheduler->bus().find(name));
    }
    [[nodiscard]] std::array<double, 3> read3(const std::string& name) const {
        const auto v = scheduler->bus().readN(scheduler->bus().find(name));
        return {v[0], v[1], v[2]};
    }
};

Rig makeRig(const World& world) {
    Rig r;
    r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    auto plant = std::make_unique<Plant>();
    r.plant = plant.get();
    r.scheduler->add(std::move(plant));
    for (auto& sensor : createSensorModules(world, nullptr)) {
        r.scheduler->add(std::move(sensor));
    }
    return r;
}

TEST(GeoSensors, GnssPlacesTheAntennaThroughTheGeoOrigin) {
    const World world = rig();
    Rig r = makeRig(world);
    auto wired = r.scheduler->wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{200'000'000}).ok());
    // Region-local (100, -50, 20) + region (1000, 2000, 0) + mount (0, 0, 0.1).
    const coords::Geodetic want = coords::GeoOrigin(kZurich).toGeodetic({1100.0, 1950.0, 20.1});
    EXPECT_NEAR(r.read("sensor/gnss/latitude"), want.latitudeRad, 1e-12);
    EXPECT_NEAR(r.read("sensor/gnss/longitude"), want.longitudeRad, 1e-12);
    EXPECT_NEAR(r.read("sensor/gnss/altitude"), want.heightM, 1e-6);
    const std::array<double, 3> v = r.read3("sensor/gnss/velocity");
    EXPECT_NEAR(v[0], 3.0, 1e-12);
    EXPECT_NEAR(v[1], 4.0, 1e-12);
    EXPECT_EQ(r.read("sensor/gnss/fix"), 3.0);
}

TEST(GeoSensors, TheMagnetometerReadsTheWorldMagneticModelInItsFrame) {
    const World world = rig();
    Rig r = makeRig(world);
    // Yawed 90° to the left and rolled 30°.
    const double yaw = std::numbers::pi / 2.0;
    const double roll = std::numbers::pi / 6.0;
    r.plant->pose.orientation = multiply({std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)},
                                         {std::cos(roll / 2), std::sin(roll / 2), 0.0, 0.0});
    ASSERT_TRUE(r.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const Pose sensor = compose(r.plant->pose, {{0.0, 0.0, 0.1}});
    const coords::Geodetic where = coords::GeoOrigin(kZurich).toGeodetic(
        {sensor.position[0] + 1000.0, sensor.position[1] + 2000.0, sensor.position[2]});
    const std::array<double, 3> northEastDown = wmmField(where, kWmmValidFrom);
    const std::array<double, 3> want =
        rotate(conjugate(sensor.orientation),
               {northEastDown[1] * 1e-9, northEastDown[0] * 1e-9, -northEastDown[2] * 1e-9});
    const std::array<double, 3> got = r.read3("sensor/magnetometer/field");
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(got[i], want[i], 1e-15) << i;
    }
    // Zurich: about 21.6 µT horizontal, 43 µT down (WMM2025).
    EXPECT_NEAR(std::hypot(northEastDown[0], northEastDown[1]), 21600.0, 300.0);
    EXPECT_NEAR(northEastDown[2], 43300.0, 500.0);
}

TEST(GeoSensors, TheBarometerFollowsTheIcaoStandardAtmosphere) {
    // ICAO Doc 7488/3 table values: 1013.25 hPa at sea level, 898.76 hPa and 281.65 K at
    // 1000 m, 226.32 hPa at 11 000 m.
    EXPECT_NEAR(standardAtmosphere(0.0).pressurePa, 101325.0, 1e-9);
    EXPECT_NEAR(standardAtmosphere(1000.0).pressurePa, 89876.0, 2.0);
    EXPECT_NEAR(standardAtmosphere(1000.0).temperatureK, 281.65, 1e-9);
    EXPECT_NEAR(standardAtmosphere(11000.0).pressurePa, 22632.0, 3.0);

    const World world = rig();
    Rig r = makeRig(world);
    ASSERT_TRUE(r.scheduler->wire(WorldView(world)));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{100'000'000}).ok());
    const double height = coords::GeoOrigin(kZurich).toGeodetic({1100.0, 1950.0, 20.1}).heightM;
    EXPECT_NEAR(r.read("sensor/barometer/pressure"), standardAtmosphere(height).pressurePa, 1e-6);
}

TEST(GeoSensors, AreLeftOutWithoutAGeoOrigin) {
    const World world = rig(false);
    EXPECT_TRUE(createSensorModules(world, nullptr).empty());
}

}  // namespace
}  // namespace lagrange::sensors
