#include "lagrange/viewer/RosFrames.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <map>
#include <string_view>

namespace lagrange::viewer {

namespace {

constexpr std::string_view kActionStatus = "/_action/status";
constexpr std::string_view kActionFeedback = "/_action/feedback";
// rosidl's names for an action's feedback topic type: "<pkg>/action/<Name>_FeedbackMessage".
constexpr std::string_view kFeedbackSuffix = "_FeedbackMessage";

bool hasSchema(const PrimData& prim, std::string_view schema) {
    return std::ranges::find(prim.apiSchemas, schema) != prim.apiSchemas.end();
}

bool isBody(const PrimData& prim) {
    return hasSchema(prim, "PhysicsRigidBodyAPI");
}

bool isSensor(const PrimData& prim) {
    return hasSchema(prim, "SensorAPI_1");
}

bool isCamera(const PrimData& prim) {
    return hasSchema(prim, "CameraSensorAPI_1") || hasSchema(prim, "ThermalCameraAPI_1");
}

const RosGraphInfo::Topic* findTopic(const RosGraphInfo& graph, std::string_view name) {
    for (const RosGraphInfo::Topic& topic : graph.topics) {
        if (topic.name == name) {
            return &topic;
        }
    }
    return nullptr;
}

/// The nodes that publish `topic`, joined by ", ", leaving out `except`.
std::string publishersOf(const RosGraphInfo& graph, std::string_view topic,
                         std::string_view except = {}) {
    std::string out;
    for (const RosGraphInfo::Node& node : graph.nodes) {
        if (node.name == except ||
            std::ranges::find(node.publishes, topic) == node.publishes.end()) {
            continue;
        }
        out += (out.empty() ? "" : ", ") + node.name;
    }
    return out;
}

/// The simulator's own node: the one node the graph does not mark external.
std::string bridgeNode(const RosGraphInfo& graph) {
    for (const RosGraphInfo::Node& node : graph.nodes) {
        if (!node.external) {
            return node.name;
        }
    }
    return {};
}

}  // namespace

bool isHiddenRosName(std::string_view name) {
    std::size_t start = 0;
    while (start < name.size()) {
        if (name[start] == '/') {
            ++start;
            continue;
        }
        if (name[start] == '_') {
            return true;
        }
        const std::size_t slash = name.find('/', start);
        if (slash == std::string_view::npos) {
            break;
        }
        start = slash + 1;
    }
    return false;
}

std::vector<RosAction> rosActions(const RosGraphInfo& graph) {
    std::map<std::string, RosAction, std::less<>> found;
    for (const RosGraphInfo::Topic& topic : graph.topics) {
        const std::string_view name = topic.name;
        if (name.ends_with(kActionStatus)) {
            const std::string action(name.substr(0, name.size() - kActionStatus.size()));
            found[action].name = action;
        } else if (name.ends_with(kActionFeedback)) {
            const std::string action(name.substr(0, name.size() - kActionFeedback.size()));
            RosAction& entry = found[action];
            entry.name = action;
            entry.feedbackHz = topic.fromBridge ? topic.rateHz : 0.0;
            std::string_view type = topic.type;
            if (type.ends_with(kFeedbackSuffix)) {
                type.remove_suffix(kFeedbackSuffix.size());
                entry.type = std::string(type);
            }
        }
    }
    for (auto& [name, action] : found) {
        const std::string status = name + std::string(kActionStatus);
        const std::string feedback = name + std::string(kActionFeedback);
        for (const RosGraphInfo::Node& node : graph.nodes) {
            if (std::ranges::find(node.publishes, status) != node.publishes.end()) {
                action.servers.push_back(node.name);
            }
            if (std::ranges::find(node.subscribes, feedback) != node.subscribes.end()) {
                action.clients.push_back(node.name);
            }
        }
    }
    std::vector<RosAction> out;
    out.reserve(found.size());
    for (auto& [name, action] : found) {
        out.push_back(std::move(action));
    }
    return out;
}

std::vector<TfFrame> tfFrames(const SceneStatics& scene, const RosGraphInfo& graph, bool running) {
    const std::vector<PrimData>& prims = scene.prims;
    // The nearest body above each prim (kNone: none).
    const auto bodyAbove = [&prims](std::uint32_t prim) {
        std::uint32_t at = prims[prim].parent;
        while (at != kNone && at < prims.size()) {
            if (isBody(prims[at])) {
                return at;
            }
            at = prims[at].parent;
        }
        return kNone;
    };
    const auto underArticulation = [&prims](std::uint32_t prim) {
        for (std::uint32_t at = prim; at != kNone && at < prims.size(); at = prims[at].parent) {
            if (hasSchema(prims[at], "PhysicsArticulationRootAPI")) {
                return true;
            }
        }
        return false;
    };
    // The robot's base: the first body with no body above it under an articulation root; else
    // the first such body that carries other bodies or sensors (a scenery box carries none).
    std::uint32_t base = kNone;
    std::uint32_t carrying = kNone;
    for (std::uint32_t i = 0; i < prims.size() && base == kNone; ++i) {
        if (!isBody(prims[i]) || bodyAbove(i) != kNone) {
            continue;
        }
        if (underArticulation(i)) {
            base = i;
        } else if (carrying == kNone) {
            for (std::uint32_t j = i + 1; j < prims.size(); ++j) {
                if ((isBody(prims[j]) || isSensor(prims[j])) && bodyAbove(j) == i) {
                    carrying = i;
                    break;
                }
            }
        }
    }
    if (base == kNone) {
        base = carrying;
    }
    std::vector<TfFrame> out;
    if (base == kNone) {
        return out;
    }
    // Children by the nearest body above, in prim order.
    std::vector<std::vector<std::uint32_t>> children(prims.size());
    for (std::uint32_t i = 0; i < prims.size(); ++i) {
        if (i == base || !(isBody(prims[i]) || isSensor(prims[i]))) {
            continue;
        }
        const std::uint32_t above = bodyAbove(i);
        if (above != kNone) {
            children[above].push_back(i);
        }
    }

    const std::string bridge = bridgeNode(graph);
    const RosGraphInfo::Topic* tf = findTopic(graph, "/tf");
    const RosGraphInfo::Topic* odom = findTopic(graph, "/odom");
    // The links between bodies are the user's robot_state_publisher's (from /joint_states and
    // the bridge's /robot_description): any node but the bridge that publishes /tf.
    const std::string linkPublishers = publishersOf(graph, "/tf", bridge);
    const std::string staticPublisher = publishersOf(graph, "/tf_static");

    int parent = -1;
    if (odom != nullptr) {
        TfFrame frame;
        frame.name = "odom";
        frame.kind = TfFrame::Kind::Odom;
        out.push_back(std::move(frame));
        parent = 0;
    }
    std::function<void(std::uint32_t, int)> add = [&](std::uint32_t prim, int above) {
        TfFrame frame;
        frame.name = prims[prim].name;
        frame.prim = prim;
        frame.parent = above;
        frame.depth = above < 0 ? 0 : out[static_cast<std::size_t>(above)].depth + 1;
        if (isSensor(prims[prim])) {
            frame.kind = TfFrame::Kind::Sensor;
            frame.topic = bridge.empty() ? "" : "/tf_static";
            frame.publisher = bridge;
        } else if (prim == base) {
            frame.kind = TfFrame::Kind::Link;
            if (odom != nullptr) {
                // odom -> base: from whoever publishes /odom (the bridge, or the controllers
                // of ros2_control), on /tf.
                frame.dynamic = true;
                frame.publisher = publishersOf(graph, "/odom");
                frame.topic = frame.publisher.empty() ? "" : "/tf";
                if (odom->fromBridge && tf != nullptr && tf->fromBridge) {
                    frame.rateHz = tf->rateHz;
                    frame.rateKnown = true;
                    if (running && tf->rateHz <= 0.0) {
                        frame.stale = true;
                        frame.note = "no /tf message from the bridge while the run goes on";
                    }
                }
                if (frame.publisher.empty()) {
                    frame.stale = running;
                    frame.note = "nobody publishes /odom, so nothing places it in odom";
                }
            }
        } else {
            frame.kind = TfFrame::Kind::Link;
            frame.dynamic = true;
            frame.publisher = linkPublishers;
            // Which of the two is not in the scene: a moving joint's child is on /tf, a fixed
            // joint's on /tf_static (robot_state_publisher's split).
            frame.topic = linkPublishers.empty() ? "" : "/tf · /tf_static";
            if (!linkPublishers.empty()) {
                frame.note = "from /joint_states: moving joints on /tf, fixed joints on /tf_static";
            } else {
                frame.note =
                    "not on /tf: no node turns /joint_states into transforms (run "
                    "robot_state_publisher on /robot_description)";
            }
        }
        out.push_back(std::move(frame));
        const int self = static_cast<int>(out.size()) - 1;
        if (isCamera(prims[prim]) && !bridge.empty()) {
            TfFrame optical;
            optical.name = prims[prim].name + "_optical";
            optical.kind = TfFrame::Kind::Optical;
            optical.parent = self;
            optical.depth = out.back().depth + 1;
            optical.topic = "/tf_static";
            optical.publisher = bridge;
            out.push_back(std::move(optical));
        }
        for (const std::uint32_t child : children[prim]) {
            add(child, self);
        }
    };
    add(base, parent);
    // The bridge's static frames are on /tf_static only if it has that topic.
    if (staticPublisher.find(bridge) == std::string::npos) {
        for (TfFrame& frame : out) {
            if (frame.kind == TfFrame::Kind::Sensor || frame.kind == TfFrame::Kind::Optical) {
                frame.topic.clear();
                frame.publisher.clear();
                frame.note = "no /tf_static from the bridge";
            }
        }
    }
    return out;
}

}  // namespace lagrange::viewer
