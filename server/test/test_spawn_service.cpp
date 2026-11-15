// simulation_interfaces' /spawn_entity on the server: a robot given as text is named by its own
// <robot name>, renamed on a clash, as the viewer's ImportRobot names it (docs/guide/cli.md
// "Spawning robots").
#include "lagrange/server/Server.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace lagrange::server {
namespace {

using namespace std::chrono_literals;
namespace si = simulation_interfaces;

constexpr const char* kCrate = R"(<?xml version="1.0"?>
<robot name="crate">
  <link name="crate_body">
    <inertial><mass value="2"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision><geometry><box size="0.4 0.4 0.4"/></geometry></collision>
  </link>
</robot>
)";

TEST(SpawnService, ARobotGivenAsTextIsNamedByItsRobotName) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_spawn_service";
    std::filesystem::create_directories(dir);
    ServerOptions options;
    options.endpoint = proto::Endpoint::unixSocket((dir / "s.sock").string());
    options.robotPath.clear();  // --robot none
    options.worldPath.clear();
    options.overlayPath.clear();
    options.ros = true;
    options.realTimeFactor = 0.0;
    Server server(options);
    const Status started = server.start();
    ASSERT_TRUE(started.ok()) << started.message;

    auto node = std::make_shared<rclcpp::Node>("spawn_service_test");
    auto client = node->create_client<si::srv::SpawnEntity>("/spawn_entity");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto call = [&](const std::string& name) {
        auto request = std::make_shared<si::srv::SpawnEntity::Request>();
        request->name = name;
        request->entity_resource.resource_string = kCrate;
        request->initial_pose.pose.orientation.w = 1.0;
        auto future = client->async_send_request(request);
        const auto deadline = std::chrono::steady_clock::now() + 60s;
        while (future.wait_for(0s) != std::future_status::ready &&
               std::chrono::steady_clock::now() < deadline) {
            server.runOnce(1);
            executor.spin_some(10ms);
        }
        EXPECT_EQ(future.wait_for(0s), std::future_status::ready) << "no answer";
        return future.get();
    };
    const auto deadline = std::chrono::steady_clock::now() + 60s;
    while (!client->service_is_ready() && std::chrono::steady_clock::now() < deadline) {
        server.runOnce(1);
        executor.spin_some(10ms);
    }
    ASSERT_TRUE(client->service_is_ready());

    const auto first = call("");
    EXPECT_EQ(first->result.result, si::msg::Result::RESULT_OK) << first->result.error_message;
    EXPECT_EQ(first->entity_name, "/crate") << "named by <robot name>, not its temporary file";
    const auto second = call("");
    EXPECT_EQ(second->result.result, si::msg::Result::RESULT_OK) << second->result.error_message;
    EXPECT_EQ(second->entity_name, "/crate_2") << "renamed on a clash";
    const auto named = call("pallet");
    EXPECT_EQ(named->result.result, si::msg::Result::RESULT_OK) << named->result.error_message;
    EXPECT_EQ(named->entity_name, "/pallet") << "a name in the request wins";
}

}  // namespace
}  // namespace lagrange::server
