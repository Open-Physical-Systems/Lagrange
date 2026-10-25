// lagrange_autoware_interface: Autoware's vehicle interface for a Lagrange road vehicle
// (AutowareInterface.hpp). Run it beside lagrange_server, or through
// `ros2 launch lagrange_autoware sedan_autoware.launch.py`.
#include "lagrange/autoware/AutowareInterface.hpp"

#include <cstdio>
#include <exception>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int code = 0;
    try {
        rclcpp::spin(std::make_shared<lagrange::autoware::AutowareInterface>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}
