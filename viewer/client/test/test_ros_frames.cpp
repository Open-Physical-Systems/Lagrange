#include "lagrange/viewer/RosFrames.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::isHiddenRosName;
using lagrange::viewer::kNone;
using lagrange::viewer::PrimData;
using lagrange::viewer::RosGraphInfo;
using lagrange::viewer::SceneStatics;
using lagrange::viewer::TfFrame;

PrimData prim(std::uint32_t parent, std::string name, std::vector<std::string> schemas = {}) {
    PrimData out;
    out.parent = parent;
    out.name = std::move(name);
    out.apiSchemas = std::move(schemas);
    return out;
}

const std::vector<std::string> kBody{"PhysicsRigidBodyAPI", "PhysicsMassAPI"};

// The sample AMR's shape: a scenery box first, then the robot with nested bodies, a lidar on
// a mast, an IMU on the base and a camera.
SceneStatics amr() {
    SceneStatics scene;
    auto& p = scene.prims;
    p.push_back(prim(kNone, "warehouse"));                             // 0
    p.push_back(prim(0, "crate", kBody));                              // 1: scenery, no children
    p.push_back(prim(kNone, "amr", {"PhysicsArticulationRootAPI"}));   // 2
    p.push_back(prim(2, "base_link", kBody));                          // 3
    p.push_back(prim(3, "visual"));                                    // 4: a mesh
    p.push_back(prim(3, "left_wheel", kBody));                         // 5
    p.push_back(prim(3, "lidar_mast", kBody));                         // 6
    p.push_back(prim(6, "top_lidar", {"SensorAPI_1", "LidarAPI_1"}));  // 7
    p.push_back(prim(3, "imu", {"SensorAPI_1", "ImuAPI_1"}));          // 8
    p.push_back(prim(3, "front_camera", {"SensorAPI_1", "CameraSensorAPI_1"}));  // 9
    p.push_back(prim(2, "joints"));                                              // 10
    return scene;
}

RosGraphInfo graph(bool withStatePublisher, double tfRate) {
    RosGraphInfo g;
    g.topics.push_back({"/odom", "nav_msgs/msg/Odometry", 1, 0, 50.0, 1.0, true, ""});
    g.topics.push_back({"/tf", "tf2_msgs/msg/TFMessage", 1, 0, tfRate, 1.0, true, ""});
    g.topics.push_back({"/tf_static", "tf2_msgs/msg/TFMessage", 1, 0, 0.0, 0.0, true, ""});
    RosGraphInfo::Node bridge;
    bridge.name = "/lagrange_ros2_bridge";
    bridge.publishes = {"/odom", "/tf", "/tf_static"};
    g.nodes.push_back(bridge);
    if (withStatePublisher) {
        RosGraphInfo::Node rsp;
        rsp.name = "/robot_state_publisher";
        rsp.publishes = {"/tf", "/tf_static"};
        rsp.external = true;
        g.nodes.push_back(rsp);
    }
    return g;
}

const TfFrame& named(const std::vector<TfFrame>& frames, const std::string& name) {
    for (const TfFrame& frame : frames) {
        if (frame.name == name) {
            return frame;
        }
    }
    static const TfFrame none;
    ADD_FAILURE() << "no frame " << name;
    return none;
}

TEST(RosFrames, HiddenNames) {
    EXPECT_TRUE(isHiddenRosName("/navigate_to_pose/_action/status"));
    EXPECT_TRUE(isHiddenRosName("/_ros2cli_daemon"));
    EXPECT_FALSE(isHiddenRosName("/cmd_vel"));
    EXPECT_FALSE(isHiddenRosName("/lidar_front/points"));
    EXPECT_FALSE(isHiddenRosName("/"));
}

TEST(RosFrames, ActionsFromTheirHiddenTopics) {
    RosGraphInfo g;
    g.topics.push_back({"/navigate_to_pose/_action/status", "action_msgs/msg/GoalStatusArray", 1, 1,
                        0.0, 0.0, false, ""});
    g.topics.push_back({"/navigate_to_pose/_action/feedback",
                        "nav2_msgs/action/NavigateToPose_FeedbackMessage", 1, 1, 0.0, 0.0, false,
                        ""});
    g.topics.push_back({"/cmd_vel", "geometry_msgs/msg/TwistStamped", 1, 1, 0.0, 0.0, true, ""});
    RosGraphInfo::Node server;
    server.name = "/bt_navigator";
    server.publishes = {"/navigate_to_pose/_action/status", "/navigate_to_pose/_action/feedback"};
    RosGraphInfo::Node client;
    client.name = "/rviz2";
    client.subscribes = {"/navigate_to_pose/_action/status", "/navigate_to_pose/_action/feedback"};
    g.nodes = {server, client};
    const auto actions = lagrange::viewer::rosActions(g);
    ASSERT_EQ(actions.size(), 1U);
    EXPECT_EQ(actions[0].name, "/navigate_to_pose");
    EXPECT_EQ(actions[0].type, "nav2_msgs/action/NavigateToPose");
    EXPECT_EQ(actions[0].servers, std::vector<std::string>{"/bt_navigator"});
    EXPECT_EQ(actions[0].clients, std::vector<std::string>{"/rviz2"});
    EXPECT_TRUE(lagrange::viewer::rosActions(RosGraphInfo{}).empty());
}

TEST(RosFrames, TreeOfTheRobotUnderOdom) {
    const auto frames = lagrange::viewer::tfFrames(amr(), graph(true, 50.0), true);
    // odom, base_link, left_wheel, lidar_mast, top_lidar, imu, front_camera(+optical).
    ASSERT_EQ(frames.size(), 8U);
    EXPECT_EQ(frames[0].name, "odom");
    EXPECT_EQ(frames[1].name, "base_link");
    EXPECT_EQ(frames[1].parent, 0);
    EXPECT_TRUE(frames[1].rateKnown);
    EXPECT_DOUBLE_EQ(frames[1].rateHz, 50.0);
    EXPECT_FALSE(frames[1].stale);
    EXPECT_EQ(frames[1].publisher, "/lagrange_ros2_bridge");
    const TfFrame& lidar = named(frames, "top_lidar");
    EXPECT_EQ(lidar.kind, TfFrame::Kind::Sensor);
    EXPECT_EQ(frames[static_cast<std::size_t>(lidar.parent)].name, "lidar_mast");
    EXPECT_EQ(lidar.depth, 3);
    EXPECT_EQ(lidar.topic, "/tf_static");
    EXPECT_EQ(named(frames, "left_wheel").publisher, "/robot_state_publisher");
    const TfFrame& optical = named(frames, "front_camera_optical");
    EXPECT_EQ(optical.kind, TfFrame::Kind::Optical);
    EXPECT_EQ(frames[static_cast<std::size_t>(optical.parent)].name, "front_camera");
    for (const TfFrame& frame : frames) {
        EXPECT_NE(frame.name, "crate");  // scenery is not on TF
    }
}

TEST(RosFrames, StaleAndUnpublishedFramesSayWhy) {
    const auto frames = lagrange::viewer::tfFrames(amr(), graph(false, 0.0), true);
    EXPECT_TRUE(named(frames, "base_link").stale);
    EXPECT_FALSE(named(frames, "base_link").note.empty());
    // Without robot_state_publisher the bodies below the base are on no topic.
    EXPECT_TRUE(named(frames, "left_wheel").topic.empty());
    EXPECT_FALSE(named(frames, "left_wheel").note.empty());
    // A paused run is not stale.
    const auto paused = lagrange::viewer::tfFrames(amr(), graph(false, 0.0), false);
    EXPECT_FALSE(named(paused, "base_link").stale);
}

TEST(RosFrames, NoOdomNoRoot) {
    RosGraphInfo g = graph(true, 50.0);
    g.topics.erase(g.topics.begin());  // no /odom
    const auto frames = lagrange::viewer::tfFrames(amr(), g, true);
    ASSERT_FALSE(frames.empty());
    EXPECT_EQ(frames[0].name, "base_link");
    EXPECT_EQ(frames[0].parent, -1);
    EXPECT_TRUE(lagrange::viewer::tfFrames(SceneStatics{}, g, true).empty());
}

}  // namespace
