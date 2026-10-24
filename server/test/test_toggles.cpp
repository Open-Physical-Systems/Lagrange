// v0.2 acceptance criterion 4: disabling any domain that has a module, in favour of its
// substitute, keeps the simulation running. Every combination of the domains with modules is
// wired (so each substitute provides what the other modules consume) and run for 2 s. The
// sample AMR carries its power-system overlay and an ambient temperature is given, so electrical
// and thermal have modules too.
#include "lagrange/server/Server.hpp"

#include <filesystem>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

TEST(DomainToggles, EveryCombinationOfSubstitutesRuns) {
    const std::vector<std::string> domains{"rigid_body", "electrical", "thermal", "sensors"};
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_toggles";
    std::filesystem::create_directories(dir);
    for (unsigned mask = 0; mask < (1U << domains.size()); ++mask) {
        ServerOptions options;
        options.endpoint =
            proto::Endpoint::unixSocket((dir / ("s" + std::to_string(mask) + ".sock")).string());
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;
        options.ambientTemperatureK = 298.15;  // a warehouse at 25 °C: thermal has a module
        options.realTimeFactor = 0.0;
        std::string label;
        for (std::size_t d = 0; d < domains.size(); ++d) {
            if ((mask & (1U << d)) != 0) {
                options.substituted.insert(domains[d]);
                label += domains[d] + " off ";
            }
        }
        SCOPED_TRACE(label.empty() ? "all on" : label);
        Server server(options);
        const Status started = server.start();
        ASSERT_TRUE(started.ok()) << started.message;
        while (server.simTime().ns < 2'000'000'000) {
            server.runOnce(0);
            ASSERT_TRUE(server.running()) << "the run stopped at t = " << server.simTime().ns;
        }
    }
}

TEST(DomainToggles, TheSampleAmrRunsOnItsBatteryAndTheProfileSwitchesItToIdeal) {
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_toggles";
    std::filesystem::create_directories(dir);
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "power.sock").string());
    options.robotPath = LAGRANGE_TEST_AMR_URDF;
    options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
    options.overlayPath = LAGRANGE_TEST_AMR_OVERLAY;
    options.ambientTemperatureK = 298.15;
    options.realTimeFactor = 0.0;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;
    EXPECT_NE(server.wiringDot().find("electrical/averaged_network\\n100 µs"), std::string::npos)
        << server.wiringDot();
    while (server.simTime().ns < 3'000'000'000) {  // the demo drive
        server.runOnce(0);
    }
    const double soc = server.read("battery/battery/soc").at(0);
    EXPECT_LT(soc, 1.0);
    EXPECT_GT(soc, 0.99);
    const double volts = server.read("bus/dc_48v/voltage").at(0);
    EXPECT_GT(volts, 50.0);
    EXPECT_LT(volts, 16 * 3.45);
    EXPECT_EQ(server.read("load/computer/powered").at(0), 1.0);
    EXPECT_NE(server.wiringDot().find("thermal/lumped\\n100 ms"), std::string::npos);
    // The motors' copper losses warm them, a little, in three seconds.
    const double motor = server.read("thermal/left_drive_motor/temperature").at(0);
    EXPECT_GT(motor, 298.15);
    EXPECT_LT(motor, 299.15);
    EXPECT_EQ(server.read("thermal/left_drive_motor/derating").at(0), 1.0);
    EXPECT_GT(std::abs(server.read("joint/left_wheel_joint/torque").at(0)) +
                  std::abs(server.read("motor/left_drive_motor/current_q").at(0)),
              0.0);

    const Status off = server.applyProfile(R"(schema_version: 1
stage: )" + std::string(LAGRANGE_TEST_AMR_URDF) +
                                           R"(
domains:
  rigid_body: {enabled: true}
  electrical: {enabled: false, substitute: ideal}
  thermal: {enabled: false, substitute: ambient}
  sensors: {enabled: true}
)");
    ASSERT_TRUE(off.ok()) << off.message;
    EXPECT_NE(server.wiringDot().find("electrical/ideal_torque_source"), std::string::npos);
    EXPECT_EQ(server.wiringDot().find("averaged_network"), std::string::npos);
    EXPECT_NE(server.wiringDot().find("thermal/ambient"), std::string::npos);
    EXPECT_EQ(server.read("thermal/left_drive_motor/temperature").at(0), 298.15);
}

}  // namespace
}  // namespace lagrange::server
