// simulation_interfaces' /spawn_entity on the server: a robot given as text is named by its own
// <robot name>, as the viewer's ImportRobot names it, renamed on a clash when allow_renaming
// says so (docs/guide/cli.md "Spawning robots").
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
    const auto call = [&](const std::string& name, bool allowRenaming,
                          const std::string& text = kCrate) {
        auto request = std::make_shared<si::srv::SpawnEntity::Request>();
        request->name = name;
        request->allow_renaming = allowRenaming;
        request->entity_resource.resource_string = text;
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

    const auto first = call("", false);
    EXPECT_EQ(first->result.result, si::msg::Result::RESULT_OK) << first->result.error_message;
    EXPECT_EQ(first->entity_name, "/crate") << "named by <robot name>, not its temporary file";
    const auto second = call("", true);
    EXPECT_EQ(second->result.result, si::msg::Result::RESULT_OK) << second->result.error_message;
    EXPECT_EQ(second->entity_name, "/crate_2") << "renamed on a clash";
    // allow_renaming false: the file's own name taken is NAME_NOT_UNIQUE, given or not (QA3 F4).
    const auto clash = call("", false);
    EXPECT_EQ(clash->result.result, si::srv::SpawnEntity::Response::NAME_NOT_UNIQUE)
        << clash->result.error_message;
    const auto clashNamed = call("crate", false);
    EXPECT_EQ(clashNamed->result.result, si::srv::SpawnEntity::Response::NAME_NOT_UNIQUE);
    const auto named = call("pallet", false);
    EXPECT_EQ(named->result.result, si::msg::Result::RESULT_OK) << named->result.error_message;
    EXPECT_EQ(named->entity_name, "/pallet") << "a name in the request wins";

    // Text that names no robot: NAME_INVALID unless renaming is allowed; then the parser's own
    // message comes back, not a temporary file's path.
    const std::string nameless = R"(<?xml version="1.0"?>
<robot>
  <link name="lonely"/>
</robot>
)";
    const auto unnamed = call("", false, nameless);
    EXPECT_EQ(unnamed->result.result, si::srv::SpawnEntity::Response::NAME_INVALID)
        << unnamed->result.error_message;
    const auto parsed = call("", true, nameless);
    EXPECT_EQ(parsed->result.result, si::srv::SpawnEntity::Response::RESOURCE_PARSE_ERROR)
        << parsed->result.error_message;
    const std::string& message = parsed->result.error_message;
    EXPECT_NE(message.find("resource_string is not a valid URDF: "), std::string::npos) << message;
    EXPECT_NE(message.find("name"), std::string::npos) << "urdfdom's own reason: " << message;
    EXPECT_EQ(message.find("console"), std::string::npos) << message;
    EXPECT_EQ(message.find("/lagrange"), std::string::npos) << "no temporary path: " << message;
}

}  // namespace
}  // namespace lagrange::server
