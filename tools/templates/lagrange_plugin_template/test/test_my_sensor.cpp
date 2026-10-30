// The plugin in a headless simulator: the sample AMR drives its demo loop and the plugin's
// output follows its base.
#include "lagrange/server/Server.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_path.hpp>

TEST(MySensor, MeasuresTheBaseFromTheOrigin) {
    lagrange::server::ServerOptions options;
    options.robotPath = (ament_index_cpp::get_package_share_path("lagrange_assets") /
                         "robots/amr_diffdrive/amr_diffdrive.urdf")
                            .string();
    options.ros = false;
    options.realTimeFactor = 0.0;
    options.plugins = {MY_PLUGIN};
    lagrange::server::Server server(options);
    const lagrange::Status loaded = server.load();
    ASSERT_TRUE(loaded.ok()) << loaded.message;
    ASSERT_TRUE(server.stepUntil(lagrange::SimTime{3'000'000'000}).ok());
    const std::vector<double> pose = server.read("body/base_link/pose");
    const std::vector<double> distance = server.read("my_sensor/distance");
    ASSERT_EQ(distance.size(), 1U);
    EXPECT_NEAR(distance[0], std::hypot(pose[0], pose[1]), 0.05);
    EXPECT_GT(distance[0], 0.3) << "the demo drive moved it";
}
