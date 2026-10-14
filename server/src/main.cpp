// lagrange_server: simulate a robot and serve it to viewers.
//
//   lagrange_server [--robot FILE.urdf] [--listen ENDPOINT] [--rtf X] [--paused]
//                   [--ros-args ...]
//
// Defaults: the sample AMR of lagrange_assets, and the default Unix socket that the viewer
// connects to when started without --connect.
#include "lagrange/server/MapExport.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

std::atomic<bool> gStop{false};

void onSignal(int /*signal*/) {
    gStop = true;
}

std::string shareFile(const char* package, const char* relative) {
    try {
        return (ament_index_cpp::get_package_share_path(package) / relative).string();
    } catch (const std::exception&) {
        return {};
    }
}

int usage(const char* error) {
    if (error != nullptr) {
        std::fprintf(stderr, "lagrange_server: %s\n", error);
    }
    std::fprintf(stderr,
                 "usage: lagrange_server [--robot FILE.urdf] [--world FILE.urdf|none] [--listen "
                 "ENDPOINT] [--rtf X] "
                 "[--paused]\n"
                 "  --robot   URDF to simulate (default: the sample AMR from lagrange_assets)\n"
                 "  --world   URDF of fixed scenery, or none (default: the sample warehouse)\n"
                 "  --listen  unix:/path.sock or tcp://host:port (default: %s)\n"
                 "  --rtf     real-time factor, 0 = as fast as possible (default 1)\n"
                 "  --paused  start paused; press Play in the viewer\n"
                 "  --no-ros  run without the ROS 2 bridge\n"
                 "  --record FILE.mcap  record the run (signals, scene, state hashes) until the\n"
                 "            server stops or the run is reset; the mock server replays it\n"
                 "  --record-max-components N  leave larger signals out (default 1024: the\n"
                 "            3D lidar's ranges)\n"
                 "  --domain D=off  replace domain D (rigid_body, sensors) by its substitute\n"
                 "  --cmd-vel-unstamped  take /cmd_vel as geometry_msgs/Twist instead of\n"
                 "            TwistStamped\n"
                 "  --export-map FILE.yaml  write an occupancy map of the fixed scenery for\n"
                 "            nav2_map_server (PGM next to it) and exit\n",
                 lagrange::proto::defaultUnixSocketPath().c_str());
    return error != nullptr ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    lagrange::server::ServerOptions options;
    options.ros = true;
    std::string exportMap;
    options.endpoint =
        lagrange::proto::Endpoint::unixSocket(lagrange::proto::defaultUnixSocketPath());
    options.robotPath = shareFile("lagrange_assets", "robots/amr_diffdrive/amr_diffdrive.urdf");
    options.worldPath = shareFile("lagrange_assets", "worlds/warehouse_small/warehouse_small.urdf");
    options.profileSchemaPath = shareFile("lagrange_profiles", "schema.json");

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        const auto value = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        if (arg == "--help" || arg == "-h") {
            return usage(nullptr);
        }
        if (arg == "--ros-args") {
            break;  // the rest is for rclcpp (ros2 launch passes remappings and parameters)
        }
        if (arg == "--robot") {
            const char* path = value();
            if (path == nullptr) {
                return usage("--robot needs a file");
            }
            options.robotPath = path;
        } else if (arg == "--world") {
            const char* path = value();
            if (path == nullptr) {
                return usage("--world needs a file, or none");
            }
            options.worldPath = std::string_view(path) == "none" ? "" : path;
        } else if (arg == "--listen") {
            const char* text = value();
            const auto endpoint =
                text != nullptr ? lagrange::proto::parseEndpoint(text) : std::nullopt;
            if (!endpoint) {
                return usage(
                    "--listen needs an endpoint such as unix:/tmp/l.sock or tcp://127.0.0.1:47610");
            }
            options.endpoint = *endpoint;
        } else if (arg == "--rtf") {
            const char* text = value();
            char* end = nullptr;
            const double rtf = text != nullptr ? std::strtod(text, &end) : -1.0;
            if (text == nullptr || end == text || *end != '\0' || !(rtf >= 0.0)) {
                return usage("--rtf needs a number >= 0");
            }
            options.realTimeFactor = rtf;
        } else if (arg == "--paused") {
            options.startPaused = true;
        } else if (arg == "--no-ros") {
            options.ros = false;
        } else if (arg == "--record") {
            const char* path = value();
            if (path == nullptr) {
                return usage("--record needs an .mcap path");
            }
            options.recordPath = path;
        } else if (arg == "--record-max-components") {
            const char* text = value();
            char* end = nullptr;
            const unsigned long n = text != nullptr ? std::strtoul(text, &end, 10) : 0;
            if (text == nullptr || end == text || *end != '\0' || n == 0) {
                return usage("--record-max-components needs a positive number");
            }
            options.recordMaxComponents = static_cast<std::uint32_t>(n);
        } else if (arg == "--domain") {
            // --domain rigid_body=off: the domain's substitute stands in (fixed robot, ground
            // truth sensors). Profiles will do this once the server applies them (0.2-E5).
            const char* text = value();
            const std::string_view setting = text != nullptr ? text : "";
            const std::size_t equals = setting.find('=');
            const std::string domain(setting.substr(0, equals));
            const std::string_view state =
                equals == std::string_view::npos ? "" : setting.substr(equals + 1);
            if ((domain != "rigid_body" && domain != "sensors") ||
                (state != "on" && state != "off")) {
                return usage("--domain takes rigid_body=on|off or sensors=on|off");
            }
            if (state == "off") {
                options.substituted.insert(domain);
            } else {
                options.substituted.erase(domain);
            }
        } else if (arg == "--cmd-vel-unstamped") {
            options.stampedCmdVel = false;
        } else if (arg == "--export-map") {
            const char* path = value();
            if (path == nullptr) {
                return usage("--export-map needs a .yaml path");
            }
            exportMap = path;
        } else {
            return usage(("unknown option " + std::string(arg)).c_str());
        }
    }
    if (options.robotPath.empty()) {
        return usage("no --robot given and lagrange_assets is not installed");
    }
    if (!exportMap.empty()) {
        // Only the fixed scenery is mapped; the robot is loaded for its frame of reference.
        auto robot = lagrange::server::loadUrdf(options.robotPath);
        if (robot && !options.worldPath.empty()) {
            if (auto added = lagrange::server::addUrdf(*robot, options.worldPath); !added) {
                std::fprintf(stderr, "lagrange_server: %s\n", added.error().c_str());
                return 1;
            }
        }
        if (!robot) {
            std::fprintf(stderr, "lagrange_server: %s\n", robot.error().c_str());
            return 1;
        }
        if (auto written = lagrange::server::exportOccupancyMap(robot->world, exportMap);
            !written) {
            std::fprintf(stderr, "lagrange_server: %s\n", written.error().c_str());
            return 1;
        }
        std::printf("wrote %s\n", exportMap.c_str());
        return 0;
    }

    if (options.ros) {
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    lagrange::server::Server server(options);
    if (const lagrange::Status status = server.start(); !status.ok()) {
        std::fprintf(stderr, "lagrange_server: %s\n", status.message.c_str());
        return 1;
    }
    while (!gStop && !server.quitRequested()) {
        server.runOnce(5);
    }
    if (options.ros) {
        rclcpp::shutdown();
    }
    return 0;
}
