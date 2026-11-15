#pragma once
// What the ROS 2 workspace can work out from the protocol it already has (protocol v1):
// the actions of the ROS graph from their hidden topics, and the TF tree from the scene's
// frames and the graph's /tf publishers. Pure functions, so they are tested without a window.
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// A hidden name by ROS convention: one of its tokens starts with '_' ("/nav/_action/status").
/// `ros2 topic list` leaves these out unless asked; so does the Topics table.
[[nodiscard]] bool isHiddenRosName(std::string_view name);

/// An action of the graph, as its hidden topics show it: `<name>/_action/status` and
/// `<name>/_action/feedback` (rcl's naming). Services are not in the graph the server reports.
struct RosAction {
    std::string name;                  ///< "/navigate_to_pose"
    std::string type;                  ///< "nav2_msgs/action/NavigateToPose"; empty if unknown
    std::vector<std::string> servers;  ///< nodes publishing its status
    std::vector<std::string> clients;  ///< nodes subscribed to its feedback
    double feedbackHz{0.0};            ///< measured only for the bridge's own topics
};
[[nodiscard]] std::vector<RosAction> rosActions(const RosGraphInfo& graph);

/// One frame of the TF tree that the run's robot gives on /tf and /tf_static.
struct TfFrame {
    enum class Kind : std::uint8_t {
        Odom,     ///< the odometry frame, the tree's root
        Link,     ///< a body of the robot (a rigid body prim)
        Sensor,   ///< a sensor's mount, on /tf_static from the bridge
        Optical,  ///< a camera's optical frame, on /tf_static from the bridge
    };
    std::string name;
    int parent{-1};  ///< index into the list; -1 for a root
    int depth{0};
    Kind kind{Kind::Link};
    std::uint32_t prim{kNone};  ///< the scene prim, kNone for odom and optical frames
    bool dynamic{false};        ///< moves: on /tf; otherwise /tf_static or fixed
    std::string topic;          ///< "/tf", "/tf_static", or "" when nobody publishes it
    std::string publisher;      ///< node name(s), "" when nobody publishes it
    double rateHz{0.0};         ///< /tf's measured rate for the bridge's frames
    bool rateKnown{false};      ///< the rate was measured (the bridge's own topics only)
    bool stale{false};          ///< should move but no message arrives
    std::string note;           ///< why it is stale or not published; empty when fine
};

/// The TF tree: odom (when the graph has /odom) over the robot's base body, the bodies by the
/// scene's prim hierarchy, sensors on their body, and cameras' optical frames. `running` is
/// whether simulated time advances (a paused run has no stale frames). Frames come in
/// depth-first order, so a list drawn in order is the tree.
[[nodiscard]] std::vector<TfFrame> tfFrames(const SceneStatics& scene, const RosGraphInfo& graph,
                                            bool running);

}  // namespace lagrange::viewer
