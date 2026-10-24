// lagrange_ros2_control/LagrangeSystem: the hardware_interface::SystemInterface that a URDF's
// <ros2_control> block names. Like gz_ros2_control's GazeboSimSystem, it is loaded by the
// controller manager that runs inside the simulator, and exchanges joint values with the
// simulation through the JointTable (docs/design/robot-classes.md §2.2).
#include "lagrange/ros2_control/Encoder.hpp"
#include "lagrange/ros2_control/JointTable.hpp"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace lagrange::ros2_control {

class LagrangeSystem final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            joints_.push_back(joint.name);
            std::vector<std::string> states;
            for (const hardware_interface::InterfaceInfo& state : joint.state_interfaces) {
                states.push_back(state.name);
            }
            std::vector<std::string> commands;
            for (const hardware_interface::InterfaceInfo& command : joint.command_interfaces) {
                commands.push_back(command.name);
            }
            states_.push_back(states);
            commands_.push_back(commands);
            // <param name="encoder_resolution">counts per revolution</param>: whole counts.
            std::optional<Encoder> encoder;
            if (const auto it = joint.parameters.find("encoder_resolution");
                it != joint.parameters.end()) {
                char* end = nullptr;
                const double counts = std::strtod(it->second.c_str(), &end);
                if (end == it->second.c_str() || *end != '\0' || !(counts >= 1.0)) {
                    RCLCPP_ERROR(get_logger(),
                                 "joint %s: encoder_resolution must be a count per revolution "
                                 ">= 1, not '%s'",
                                 joint.name.c_str(), it->second.c_str());
                    return CallbackReturn::ERROR;
                }
                encoder.emplace(counts);
            }
            encoders_.push_back(encoder);
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>&, const std::vector<std::string>&) override {
        return hardware_interface::return_type::OK;
    }

    /// The claimed interface ("left_wheel_joint/velocity") sets the joint's mode; stopping a
    /// controller returns the joint to no command (zero torque).
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start, const std::vector<std::string>& stop) override {
        JointTable& table = JointTable::instance();
        const std::lock_guard<std::mutex> lock(table.mutex());
        for (const std::string& name : stop) {
            const std::size_t slash = name.rfind('/');
            if (slash != std::string::npos && table.contains(name.substr(0, slash))) {
                table[name.substr(0, slash)].mode = CommandMode::None;
            }
        }
        for (const std::string& name : start) {
            const std::size_t slash = name.rfind('/');
            if (slash == std::string::npos) {
                continue;
            }
            const std::string joint = name.substr(0, slash);
            const std::string kind = name.substr(slash + 1);
            JointSlot& slot = table[joint];
            if (kind == hardware_interface::HW_IF_EFFORT) {
                slot.mode = CommandMode::Effort;
            } else if (kind == hardware_interface::HW_IF_VELOCITY) {
                slot.mode = CommandMode::Velocity;
            } else if (kind == hardware_interface::HW_IF_POSITION) {
                slot.mode = CommandMode::Position;
                slot.commandPosition = slot.position;  // hold where it is until commanded
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type read(const rclcpp::Time&,
                                         const rclcpp::Duration& period) override {
        JointTable& table = JointTable::instance();
        const std::lock_guard<std::mutex> lock(table.mutex());
        for (std::size_t j = 0; j < joints_.size(); ++j) {
            const JointSlot& slot = table[joints_[j]];
            std::optional<Encoder>& encoder = encoders_[j];
            // With an encoder, position and velocity are what its counter and driver report.
            const double position = encoder ? encoder->position(slot.position) : slot.position;
            const double velocity =
                encoder ? encoder->velocity(slot.position, period.seconds()) : slot.velocity;
            for (const std::string& state : states_[j]) {
                const std::string name = joints_[j] + "/" + state;
                if (state == hardware_interface::HW_IF_POSITION) {
                    set_state(name, position);
                } else if (state == hardware_interface::HW_IF_VELOCITY) {
                    set_state(name, velocity);
                } else if (state == hardware_interface::HW_IF_EFFORT) {
                    set_state(name, slot.effort);
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
        JointTable& table = JointTable::instance();
        const std::lock_guard<std::mutex> lock(table.mutex());
        for (std::size_t j = 0; j < joints_.size(); ++j) {
            JointSlot& slot = table[joints_[j]];
            for (const std::string& command : commands_[j]) {
                const std::string name = joints_[j] + "/" + command;
                const double value = get_command<double>(name);
                if (std::isnan(value)) {
                    continue;  // never set by a controller
                }
                if (command == hardware_interface::HW_IF_EFFORT) {
                    slot.commandEffort = value;
                } else if (command == hardware_interface::HW_IF_VELOCITY) {
                    slot.commandVelocity = value;
                } else if (command == hardware_interface::HW_IF_POSITION) {
                    slot.commandPosition = value;
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

private:
    std::vector<std::string> joints_;
    std::vector<std::vector<std::string>> states_;
    std::vector<std::vector<std::string>> commands_;
    std::vector<std::optional<Encoder>> encoders_;  // per joint; none: exact values
};

}  // namespace lagrange::ros2_control

PLUGINLIB_EXPORT_CLASS(lagrange::ros2_control::LagrangeSystem, hardware_interface::SystemInterface)
