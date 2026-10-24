#include "CommandLine.hpp"
#include "lagrange/server/MapExport.hpp"
#include "lagrange/server/Robot.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lagrange::server {
namespace {

std::atomic<bool> gStop{false};

void onSignal(int /*signal*/) {
    gStop = true;
}

/// "X,Y[,Z[,YAW]]": metres in world ENU and a heading in radians about z.
std::optional<Pose> parseSpawn(const char* text) {
    if (text == nullptr) {
        return std::nullopt;
    }
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    int count = 0;
    const char* at = text;
    while (count < 4) {
        char* end = nullptr;
        values[count++] = std::strtod(at, &end);
        if (end == at || !std::isfinite(values[count - 1])) {
            return std::nullopt;
        }
        if (*end == '\0') {
            break;
        }
        if (*end != ',') {
            return std::nullopt;
        }
        at = end + 1;
    }
    if (count < 2 || *at == '\0') {
        return std::nullopt;
    }
    Pose pose;
    pose.position = {values[0], values[1], values[2]};
    pose.orientation = {std::cos(values[3] / 2.0), 0.0, 0.0, std::sin(values[3] / 2.0)};
    return pose;
}

}  // namespace

std::string shareFile(const char* package, const char* relative) {
    try {
        return (ament_index_cpp::get_package_share_path(package) / relative).string();
    } catch (const std::exception&) {
        return {};
    }
}

CommandLine defaultCommandLine() {
    CommandLine line;
    line.options.ros = true;
    line.options.endpoint = proto::Endpoint::unixSocket(proto::defaultUnixSocketPath());
    line.options.robotPath =
        shareFile("lagrange_assets", "robots/amr_diffdrive/amr_diffdrive.urdf");
    line.options.worldPath =
        shareFile("lagrange_assets", "worlds/warehouse_small/warehouse_small.urdf");
    line.options.profileSchemaPath = shareFile("lagrange_profiles", "schema.json");
    return line;
}

void printServerOptions(std::FILE* out) {
    std::fprintf(out,
                 "  --robot   URDF to simulate (default: the sample AMR from lagrange_assets)\n"
                 "  --world   URDF of fixed scenery, or none (default: the sample warehouse)\n"
                 "  --listen  unix:/path.sock or tcp://host:port (default: %s)\n"
                 "  --spawn X,Y[,Z[,YAW]]  start the robot there (m, world ENU; heading in rad)\n"
                 "  --lidar auto|cpu|gpu  where lidars cast: auto (default) uses a hardware GPU\n"
                 "            when the run takes ROS input, else the CPU reference\n"
                 "  --gpu DEVICE  Vulkan device for the lidar (index or part of its name)\n"
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
                 proto::defaultUnixSocketPath().c_str());
}

std::optional<std::string> parseCommandLine(int argc, char** argv, int first, bool allowPositional,
                                            CommandLine& out) {
    ServerOptions& options = out.options;
    for (int i = first; i < argc; ++i) {
        const std::string_view arg = argv[i];
        const auto value = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        if (arg == "--help" || arg == "-h") {
            out.help = true;
            return std::nullopt;
        }
        if (arg == "--ros-args") {
            break;  // the rest is for rclcpp (ros2 launch passes remappings and parameters)
        }
        if (allowPositional && !arg.starts_with("--")) {
            out.positional.emplace_back(arg);
        } else if (arg == "--robot") {
            const char* path = value();
            if (path == nullptr) {
                return "--robot needs a file";
            }
            options.robotPath = path;
            out.robotGiven = true;
        } else if (arg == "--world") {
            const char* path = value();
            if (path == nullptr) {
                return "--world needs a file, or none";
            }
            options.worldPath = std::string_view(path) == "none" ? "" : path;
        } else if (arg == "--listen") {
            const char* text = value();
            const auto endpoint = text != nullptr ? proto::parseEndpoint(text) : std::nullopt;
            if (!endpoint) {
                return "--listen needs an endpoint such as unix:/tmp/l.sock or "
                       "tcp://127.0.0.1:47610";
            }
            options.endpoint = *endpoint;
        } else if (arg == "--spawn") {
            const auto spawn = parseSpawn(value());
            if (!spawn) {
                return "--spawn needs X,Y[,Z[,YAW]]";
            }
            options.spawn = *spawn;
        } else if (arg == "--lidar") {
            const char* text = value();
            const std::string_view where = text != nullptr ? text : "";
            if (where == "auto") {
                options.lidar = ServerOptions::Lidar::Auto;
            } else if (where == "cpu") {
                options.lidar = ServerOptions::Lidar::Cpu;
            } else if (where == "gpu") {
                options.lidar = ServerOptions::Lidar::Gpu;
            } else {
                return "--lidar needs auto, cpu or gpu";
            }
        } else if (arg == "--gpu") {
            const char* device = value();
            if (device == nullptr) {
                return "--gpu needs a device index or part of its name";
            }
            options.gpu = device;
        } else if (arg == "--rtf") {
            const char* text = value();
            char* end = nullptr;
            const double rtf = text != nullptr ? std::strtod(text, &end) : -1.0;
            if (text == nullptr || end == text || *end != '\0' || !(rtf >= 0.0)) {
                return "--rtf needs a number >= 0";
            }
            options.realTimeFactor = rtf;
        } else if (arg == "--paused") {
            options.startPaused = true;
        } else if (arg == "--no-ros") {
            options.ros = false;
        } else if (arg == "--record") {
            const char* path = value();
            if (path == nullptr) {
                return "--record needs an .mcap path";
            }
            options.recordPath = path;
        } else if (arg == "--record-max-components") {
            const char* text = value();
            char* end = nullptr;
            const unsigned long n = text != nullptr ? std::strtoul(text, &end, 10) : 0;
            if (text == nullptr || end == text || *end != '\0' || n == 0) {
                return "--record-max-components needs a positive number";
            }
            options.recordMaxComponents = static_cast<std::uint32_t>(n);
        } else if (arg == "--domain") {
            // --domain rigid_body=off: the domain's substitute stands in (fixed robot, ground
            // truth sensors). A profile does the same for every domain.
            const char* text = value();
            const std::string_view setting = text != nullptr ? text : "";
            const std::size_t equals = setting.find('=');
            const std::string domain(setting.substr(0, equals));
            const std::string_view state =
                equals == std::string_view::npos ? "" : setting.substr(equals + 1);
            if ((domain != "rigid_body" && domain != "sensors") ||
                (state != "on" && state != "off")) {
                return "--domain takes rigid_body=on|off or sensors=on|off";
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
                return "--export-map needs a .yaml path";
            }
            out.exportMap = path;
        } else {
            return "unknown option " + std::string(arg);
        }
    }
    if (options.robotPath.empty()) {
        return "no --robot given and lagrange_assets is not installed";
    }
    return std::nullopt;
}

int serve(CommandLine& line, int argc, char** argv, const std::string* profileYaml,
          const char* program) {
    ServerOptions& options = line.options;
    if (!line.exportMap.empty()) {
        // Only the fixed scenery is mapped; the robot is loaded for its frame of reference.
        auto robot = loadUrdf(options.robotPath, {.spawn = options.spawn});
        if (robot && !options.worldPath.empty()) {
            if (auto added = addUrdf(*robot, options.worldPath); !added) {
                std::fprintf(stderr, "%s: %s\n", program, added.error().c_str());
                return 1;
            }
        }
        if (!robot) {
            std::fprintf(stderr, "%s: %s\n", program, robot.error().c_str());
            return 1;
        }
        // The map is in the frame /odom and /tf use: region-local.
        recentre(*robot);
        if (auto written = exportOccupancyMap(robot->world, line.exportMap); !written) {
            std::fprintf(stderr, "%s: %s\n", program, written.error().c_str());
            return 1;
        }
        std::printf("wrote %s\n", line.exportMap.c_str());
        return 0;
    }

    if (options.ros) {
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    int code = 0;
    {
        Server server(options);
        Status status = server.load();
        if (status.ok() && profileYaml != nullptr) {
            status = server.applyProfile(*profileYaml);
        }
        if (status.ok()) {
            status = server.start();
        }
        if (!status.ok()) {
            std::fprintf(stderr, "%s: %s\n", program, status.message.c_str());
            code = 1;
        } else {
            while (!gStop && !server.quitRequested()) {
                server.runOnce(5);
            }
        }
    }
    if (options.ros) {
        rclcpp::shutdown();
    }
    return code;
}

}  // namespace lagrange::server
