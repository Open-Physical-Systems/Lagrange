#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/ros2_control/JointTable.hpp"

#include <thread>
#include <tinyxml2.h>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace lagrange::ros2_control {

struct ControllerHost::Impl {
    std::shared_ptr<rclcpp::Executor> executor;
    std::shared_ptr<controller_manager::ControllerManager> manager;
    std::thread spinner;
};

ControllerHost::ControllerHost(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

ControllerHost::~ControllerHost() {
    impl_->executor->cancel();
    if (impl_->spinner.joinable()) {
        impl_->spinner.join();
    }
    impl_->executor->remove_node(impl_->manager);
    impl_->manager.reset();
}

Expected<std::unique_ptr<ControllerHost>, std::string> ControllerHost::start(
    const std::string& urdf, const std::string& paramsFile) {
    auto joints = controlJoints(urdf);
    if (!joints) {
        return Unexpected{joints.error()};
    }
    if (joints->empty()) {
        return Unexpected{std::string(
            "the robot has no <ros2_control> block with lagrange_ros2_control/LagrangeSystem")};
    }
    {
        JointTable& table = JointTable::instance();
        const std::lock_guard<std::mutex> lock(table.mutex());
        table.clear();
        for (const ControlJoint& joint : *joints) {
            (void)table[joint.name];  // every slot exists before the plugin looks
        }
    }
    auto impl = std::make_unique<Impl>();
    impl->executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
    std::vector<std::string> arguments{"--ros-args"};
    if (!paramsFile.empty()) {
        arguments.insert(arguments.end(), {"--params-file", paramsFile});
    }
    arguments.insert(arguments.end(), {"-p", "use_sim_time:=true"});
    options.arguments(arguments);
    try {
        impl->manager = std::make_shared<controller_manager::ControllerManager>(
            impl->executor, urdf, true, "controller_manager", "", options);
    } catch (const std::exception& e) {
        return Unexpected{std::string("the controller manager did not start: ") + e.what()};
    }
    impl->executor->add_node(impl->manager);
    rclcpp::Executor* executor = impl->executor.get();
    impl->spinner = std::thread([executor] { executor->spin(); });
    return std::unique_ptr<ControllerHost>(new ControllerHost(std::move(impl)));
}

unsigned int ControllerHost::updateRateHz() const {
    return impl_->manager->get_update_rate();
}

void ControllerHost::cycle(std::int64_t nowNs, std::int64_t periodNs) {
    const rclcpp::Time now(nowNs, RCL_ROS_TIME);
    const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(periodNs);
    controller_manager::ControllerManager& cm = *impl_->manager;
    cm.read(now, period);
    cm.update(now, period);
    cm.write(now, period);
}

Expected<std::vector<ControlJoint>, std::string> controlJoints(const std::string& urdf) {
    tinyxml2::XMLDocument document;
    if (document.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{std::string("the robot description is not valid XML")};
    }
    std::vector<ControlJoint> out;
    const tinyxml2::XMLElement* robot = document.FirstChildElement("robot");
    for (const tinyxml2::XMLElement* block =
             robot != nullptr ? robot->FirstChildElement("ros2_control") : nullptr;
         block != nullptr; block = block->NextSiblingElement("ros2_control")) {
        const tinyxml2::XMLElement* hardware = block->FirstChildElement("hardware");
        const tinyxml2::XMLElement* plugin =
            hardware != nullptr ? hardware->FirstChildElement("plugin") : nullptr;
        if (plugin == nullptr || plugin->GetText() == nullptr ||
            std::string(plugin->GetText()) != "lagrange_ros2_control/LagrangeSystem") {
            continue;
        }
        for (const tinyxml2::XMLElement* joint = block->FirstChildElement("joint");
             joint != nullptr; joint = joint->NextSiblingElement("joint")) {
            ControlJoint control;
            control.name = joint->Attribute("name") != nullptr ? joint->Attribute("name") : "";
            for (const tinyxml2::XMLElement* param = joint->FirstChildElement("param");
                 param != nullptr; param = param->NextSiblingElement("param")) {
                const char* name = param->Attribute("name");
                const char* text = param->GetText();
                if (name == nullptr || text == nullptr) {
                    continue;
                }
                const double value = std::strtod(text, nullptr);
                const std::string key = name;
                if (key == "velocity_p") {
                    control.gains.velocityP = value;
                } else if (key == "velocity_i") {
                    control.gains.velocityI = value;
                } else if (key == "position_p") {
                    control.gains.positionP = value;
                } else if (key == "position_d") {
                    control.gains.positionD = value;
                } else if (key == "position_i") {
                    control.gains.positionI = value;
                }
            }
            out.push_back(std::move(control));
        }
    }
    return out;
}

}  // namespace lagrange::ros2_control
