// Kind "sonar" (sensors.md §9): an echosounder and a multibeam fan under a box of water, against
// closed forms: ranges to the bottom and a wall, the device's sound speed against Mackenzie's,
// two-way spreading, absorption and Lambert backscatter, the ping interval, nothing above water.
#include "kinds_test_util.hpp"

#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::sensors::kinds {
inline namespace extra2 {
double mackenzieSoundSpeed(double t, double s, double d);
double seawaterAbsorption(double fKHz, double t, double s, double zKm, double ph);
}  // namespace extra2
}  // namespace lagrange::sensors::kinds

namespace lagrange::sensors {
namespace {

using namespace test;
using kinds::mackenzieSoundSpeed;
using kinds::seawaterAbsorption;

constexpr double kPi = std::numbers::pi;

/// A 100 m × 100 m box of water 20 m deep, its surface at z = 0.
void addWater(World& w) {
    const PrimId prim = w.addPrim("/pond", "pond");
    SchemaInstance s;
    s.schema = "WaterAPI_1";
    s.prim = prim;
    s.reals = {{"water:sizeX", 100.0}, {"water:sizeY", 100.0}, {"water:depth", 20.0}};
    w.schemaInstances.push_back(s);
}

Pose down() {  // the sensor's +x straight down
    Pose p;
    p.orientation = {std::cos(kPi / 4.0), 0.0, std::sin(kPi / 4.0), 0.0};
    return p;
}

Pose at(double x, double y, double z) {
    Pose p;
    p.position = {x, y, z};
    return p;
}

using Settings = std::map<std::string, double, std::less<>>;

Settings exact(Settings s) {
    s.emplace("noise/stddev", 0.0);
    s.emplace("resolution", 0.0);
    s.emplace("intensity_noise", 0.0);
    return s;
}

double alpha750(double depth) {  // dB/m, 10 °C, 35 PSU, pH 8
    return seawaterAbsorption(750.0, 10.0, 35.0, depth / 1000.0, 8.0) / 1000.0;
}

TEST(Sonar, IsBuiltin) {
    EXPECT_TRUE(isBuiltinKind("sonar"));
}

TEST(Sonar, MackenzieAndAinslieMcColm) {
    // Mackenzie (1981) at 10 °C, 35 PSU, the surface: 1489.80 m/s by the equation's constant
    // and temperature terms; 1000 m deeper adds 16.30 + 0.17 m/s.
    EXPECT_NEAR(mackenzieSoundSpeed(10.0, 35.0, 0.0), 1489.80, 0.01);
    EXPECT_NEAR(mackenzieSoundSpeed(10.0, 35.0, 1000.0) - mackenzieSoundSpeed(10.0, 35.0, 0.0),
                16.30 + 0.1675 - 7.139e-13 * 10.0 * 1e9, 1e-9);
    // About 1 dB/km at 10 kHz in cold sea water (Urick 1983, fig. 5.5).
    const double at10k = seawaterAbsorption(10.0, 4.0, 35.0, 0.0, 8.0);
    EXPECT_GT(at10k, 0.9);
    EXPECT_LT(at10k, 1.3);
    // At 750 kHz pure water's viscosity dominates: a few tenths of a dB per metre.
    const double at750k = seawaterAbsorption(750.0, 10.0, 35.0, 0.0, 8.0);
    EXPECT_GT(at750k, 200.0);
    EXPECT_LT(at750k, 280.0);
}

TEST(Sonar, AltimeterSeesTheBottom) {
    KindRun r;
    rig(r.world, "alt", "sonar", down(), 10.0, {{"beams", 1}, {"vertical_rays", 1}});
    addWater(r.world);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, -10), {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/alt/valid"), 1.0);
    EXPECT_NEAR(r.readN("sensor/alt/ranges")[0], 10.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/alt/depth"), 10.0, 1e-9);
    // −40·log10(10) − 2·α·10 − 27 dB, at normal incidence.
    EXPECT_NEAR(r.readN("sensor/alt/intensities")[0], -40.0 - 20.0 * alpha750(10.0) - 27.0, 1e-9);
    EXPECT_NEAR(r.read("sensor/alt/ping_interval"), 2.0 * 120.0 / 1500.0, 1e-12);
}

TEST(Sonar, ItsSoundSpeedAgainstTheWaters) {
    // Set to 1500 m/s in water of 1489.96 m/s: the range reads long by 1500/c.
    KindRun r;
    rig(r.world, "alt", "sonar", down(), 10.0,
        exact({{"beams", 1}, {"vertical_rays", 1}, {"water_temperature", 10.0}}));
    addWater(r.world);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, -10), {}), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    const double c = mackenzieSoundSpeed(10.0, 35.0, 10.0);
    EXPECT_NEAR(r.read("sensor/alt/sound_speed"), c, 1e-9);
    EXPECT_NEAR(r.readN("sensor/alt/ranges")[0], 10.0 * 1500.0 / c, 1e-9);
}

TEST(Sonar, TheSurfaceFromBelow) {
    Pose up;  // +x straight up
    up.orientation = {std::cos(kPi / 4.0), 0.0, -std::sin(kPi / 4.0), 0.0};
    KindRun r;
    rig(r.world, "up", "sonar", up, 10.0, {{"beams", 1}, {"vertical_rays", 1}});
    addWater(r.world);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, -4), {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_NEAR(r.readN("sensor/up/ranges")[0], 4.0, 1e-9);
}

TEST(Sonar, NothingAboveWater) {
    KindRun r;
    rig(r.world, "alt", "sonar", down(), 10.0, {{"beams", 1}});
    addWater(r.world);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, 1.0), {}), false));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    EXPECT_EQ(r.read("sensor/alt/valid"), 0.0);
    EXPECT_EQ(r.readN("sensor/alt/ranges")[0], std::numeric_limits<double>::infinity());
    EXPECT_TRUE(std::isnan(r.read("sensor/alt/depth")));
}

TEST(Sonar, AFanOnAWall) {
    // Three beams over 90°, level, 10 m down, a wall 5 m ahead: the middle beam 5 m at normal
    // incidence, the sides 5·√2 m at 45°, 12.04 dB of spreading and 3.01 dB of Lambert's cos²
    // weaker, less the absorption over the extra path.
    KindRun r;
    rig(r.world, "fan", "sonar", Pose{}, 10.0,
        {{"beams", 3},
         {"horizontal_fov", kPi / 2.0},
         {"vertical_rays", 1},
         {"image_bins", 10},
         {"range/max", 10.0},
         {"range/min", 0.0}});
    addWater(r.world);
    r.world.addCollider(kWorldBody, ShapeType::Box, {0.5, 20.0, 20.0}, at(5.5, 0.0, -10.0), 1.0);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, -10), {}), true));
    ASSERT_TRUE(r.scheduler->stepUntil(SimTime{500'000'000}).ok());
    const std::vector<double> ranges = r.readN("sensor/fan/ranges");
    const std::vector<double> levels = r.readN("sensor/fan/intensities");
    ASSERT_EQ(ranges.size(), 3U);
    EXPECT_NEAR(ranges[1], 5.0, 1e-9);
    EXPECT_NEAR(ranges[0], 5.0 * std::sqrt(2.0), 1e-9);
    EXPECT_NEAR(ranges[2], 5.0 * std::sqrt(2.0), 1e-9);
    const double a = alpha750(10.0);
    const double spread = 40.0 * std::log10(std::sqrt(2.0));
    const double lambert = 10.0 * std::log10(0.5);
    EXPECT_NEAR(levels[0] - levels[1], -spread + lambert - 2.0 * a * 5.0 * (std::sqrt(2.0) - 1.0),
                1e-3);
    // The polar image: beam 1's echo in bin 5 of 10 over 0 to 10 m.
    const std::vector<double> image = r.readN("sensor/fan/image");
    ASSERT_EQ(image.size(), 30U);
    EXPECT_GT(image[1 * 10 + 5], 0.0);
    EXPECT_EQ(image[1 * 10 + 4], 0.0);
}

TEST(Sonar, NoiseAndResolution) {
    KindRun r;
    rig(r.world, "alt", "sonar", down(), 5.0, {{"beams", 1}, {"vertical_rays", 1}});
    addWater(r.world);
    ASSERT_TRUE(start(r, bodyFeeds(at(0, 0, -10), {}), false));
    double sum = 0.0;
    const int n = 100;
    for (int k = 1; k <= n; ++k) {
        ASSERT_TRUE(r.scheduler->stepUntil(SimTime{k * 200'000'000LL}).ok());
        const double d = r.readN("sensor/alt/ranges")[0];
        EXPECT_NEAR(std::remainder(d, 0.004), 0.0, 1e-9);  // 4 mm steps
        EXPECT_NEAR(d, 10.0, 0.03);
        sum += d;
    }
    EXPECT_NEAR(sum / n, 10.0, 0.003);
}

TEST(Sonar, AWaterTemperatureInKelvinIsRefused) {
    // water_temperature is in °C: 283.15 (10 °C in K, §9's SI habit) made Mackenzie's equation
    // read 3886 m/s, every range 2.6 times short. It is refused, as a salinity of 100.
    for (const Settings& settings : {Settings{{"water_temperature", 283.15}},
                                     Settings{{"water_temperature", 10.0}, {"salinity", 100.0}}}) {
        KindRun r;
        rig(r.world, "alt", "sonar", down(), 10.0, settings);
        addWater(r.world);
        r.scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
        r.scheduler->add(createBuiltinKind(r.world, 0, false, nullptr, nullptr));
        auto wired = r.scheduler->wire(WorldView(r.world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("water_temperature is in"),
                  std::string::npos);
    }
}

}  // namespace
}  // namespace lagrange::sensors
