#include "CommandLine.hpp"
#include "lagrange/server/MapExport.hpp"
#include "lagrange/server/Robot.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <set>
#include <string_view>

#include <yaml-cpp/yaml.h>

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
    line.options.overlayPath =
        shareFile("lagrange_assets", "robots/amr_diffdrive/amr_diffdrive.overlay.yaml");
    line.options.profileSchemaPath = shareFile("lagrange_profiles", "schema.json");
    return line;
}

void printServerOptions(std::FILE* out) {
    std::fprintf(
        out,
        "  --robot   URDF, SDFormat, MJCF or USD to simulate, or none to start without a\n"
        "            robot and spawn one later (default: the sample AMR)\n"
        "  --world   fixed scenery in any of those formats, or none (default: the sample\n"
        "            warehouse)\n"
        "  --overlay FILE.yaml  apply schemas (motors, battery, bus) to the robot, or\n"
        "            none (default: the sample AMR's power system, for the sample only)\n"
        "  --listen  unix:/path.sock or tcp://host:port (default: %s)\n"
        "  --tls-cert FILE --tls-key FILE  certificate chain and key (PEM) for a tls:// --listen\n"
        "  --auth-token-file FILE  TCP clients must send this token (first line of FILE);\n"
        "                         required to listen beyond loopback\n"
        "  --spawn X,Y[,Z[,YAW]]  start the robot there (m, world ENU; heading in rad)\n"
        "  --ros-topics FILE.yaml  publish signals on ROS topics and write topics into signals\n"
        "            (publish: [{signal, topic, type, frame, params}], subscribe: [...])\n"
        "  --plugin LIB  load a plugin library (RFC 0008): a path, or NAME for lib<NAME>.so in\n"
        "            $LAGRANGE_PLUGIN_PATH or the lagrange_plugins resource index (repeatable)\n"
        "  --add-robot NAME=FILE@X,Y[,Z[,YAW]]  one more robot in the run, its names prefixed\n"
        "                         with NAME, passive (repeatable)\n"
        "  --lidar auto|cpu|gpu  where lidars cast: auto (default) and cpu use the\n"
        "            exact CPU caster; gpu the compute kernel on a hardware GPU\n"
        "  --gpu DEVICE  Vulkan device for the lidar and cameras (index or part of its name)\n"
        "  --physics mujoco|jolt|NAME  rigid-body backend (default mujoco), or a plugin's\n"
        "  --replay FILE.mcap  replay a recording instead of simulating (the viewer can seek;\n"
        "            no ROS bridge)\n"
        "  --session FILE.usda  where the viewer's edits are saved when it asks (default:\n"
        "            $XDG_STATE_HOME/lagrange/<robot>.session.usda)\n"
        "  --rtf     real-time factor, 0 = as fast as possible (default 1)\n"
        "  --paused  start paused; press Play in the viewer\n"
        "  --no-ros  run without the ROS 2 bridge\n"
        "  --ros2-control PARAMS.yaml  run a ros2_control controller manager in lockstep\n"
        "            (lagrange_ros2_control/LagrangeSystem); spawn its controllers\n"
        "  --record FILE.mcap  record the run (signals, scene, state hashes) until the\n"
        "            server stops or the run is reset; the mock server replays it\n"
        "  --record-max-components N  leave larger signals out (default 1024: the\n"
        "            3D lidar's ranges)\n"
        "  --bench RUN.mcap  bench mode: physics replaced by the run's recorded joint\n"
        "            and body motion, open loop (electrical, thermal, sensors live)\n"
        "  --ambient-temperature K  the site's air temperature: runs the thermal domain\n"
        "            (no default; a profile's thermal.ambient_temperature also sets it)\n"
        "  --mavlink px4[:PORT]|ardupilot[:PORT]  fly a multicopter with PX4 SITL (TCP 4560;\n"
        "            tools/ci/px4_mission.sh) or ArduPilot SITL's JSON interface (UDP 9002;\n"
        "            tools/ci/ardupilot_mission.sh), in lockstep\n"
        "  --geo-origin LAT,LON,HEIGHT  where the world's origin is on the Earth (degrees,\n"
        "            degrees, m above WGS84), for GNSS, magnetometers and barometers (no default;\n"
        "            a profile's world.geo_origin also sets it)\n"
        "  --air-density KG_M3  the site's air density, for rotors (ISA sea level: 1.225; no\n"
        "            default; a profile's rotor_aero.air_density also sets it)\n"
        "  --weather K=V,...  the weather: temperature (K), pressure (Pa) and relative_humidity\n"
        "            (0-1), the site, no defaults; visibility (m, fog), rain_rate (m/s),\n"
        "            dust_concentration (kg/m³), dust_median_diameter (m), solar_irradiance\n"
        "            (W/m²), sky_temperature (K), fog_field and wind_field (.nvdb files);\n"
        "            a profile's weather keys also set them\n"
        "  --domain D=off  replace domain D (rigid_body, electrical, thermal, weather, emi,\n"
        "            comms, vehicle_dynamics, traffic, sensors) by its substitute\n"
        "  --scenario FILE.xosc  run an OpenSCENARIO scenario (a subset of 1.x); its roads are "
        "the\n"
        "            world unless --world is given; its entity Ego (--scenario-ego) is the robot\n"
        "  --traffic vehicles=N,pedestrians=M,seed=S  the builtin traffic on a world with road\n"
        "            lanes (default 8, 6, 1; 0 and 0: none)\n"
        "  --cmd-vel-unstamped  take /cmd_vel as geometry_msgs/Twist instead of\n"
        "            TwistStamped\n"
        "  --no-demo  stand still until the first /cmd_vel instead of driving the demo\n"
        "            figure-eight\n"
        "  --link-topics FILE.yaml  ROS topics that ride on radio links (LINK: [/topic, ...]):\n"
        "            lost and delayed as the comms domain's link quality says\n"
        "  --export-map FILE.yaml  write an occupancy map of the fixed scenery for\n"
        "            nav2_map_server (PGM next to it) and exit\n",
        proto::defaultUnixSocketPath().c_str());
}

std::optional<std::string> parseCommandLine(int argc, char** argv, int first, bool allowPositional,
                                            CommandLine& out) {
    ServerOptions& options = out.options;
    bool thermalAsked = false;  // --domain thermal=on
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
                return "--robot needs a file, or none";
            }
            options.robotPath = std::string_view(path) == "none" ? "" : path;
            out.robotGiven = true;
        } else if (arg == "--world") {
            const char* path = value();
            if (path == nullptr) {
                return "--world needs a file, or none";
            }
            options.worldPath = std::string_view(path) == "none" ? "" : path;
            out.worldGiven = true;
        } else if (arg == "--overlay") {
            const char* path = value();
            if (path == nullptr) {
                return "--overlay needs a file, or none";
            }
            options.overlayPath = std::string_view(path) == "none" ? "" : path;
            out.overlayGiven = true;
        } else if (arg == "--listen") {
            const char* text = value();
            const auto endpoint = text != nullptr ? proto::parseEndpoint(text) : std::nullopt;
            if (!endpoint) {
                return "--listen needs an endpoint such as unix:/tmp/l.sock, "
                       "tcp://127.0.0.1:47610 or tls://0.0.0.0:47610";
            }
            options.endpoint = *endpoint;
        } else if (arg == "--tls-cert") {
            const char* text = value();
            if (text == nullptr) {
                return "--tls-cert needs a PEM file (the certificate chain)";
            }
            options.tlsCertificate = text;
        } else if (arg == "--tls-key") {
            const char* text = value();
            if (text == nullptr) {
                return "--tls-key needs a PEM file (the private key)";
            }
            options.tlsKey = text;
        } else if (arg == "--auth-token-file") {
            const char* text = value();
            std::ifstream in(text != nullptr ? text : "");
            std::string token;
            if (!in || !std::getline(in, token) || token.empty()) {
                return "--auth-token-file needs a readable file whose first line is the token";
            }
            options.authToken = token;
        } else if (arg == "--spawn") {
            const auto spawn = parseSpawn(value());
            if (!spawn) {
                return "--spawn needs X,Y[,Z[,YAW]]";
            }
            options.spawn = *spawn;
        } else if (arg == "--add-robot") {
            const char* raw = value();
            const std::string text = raw != nullptr ? raw : "";
            const std::size_t equals = text.find('=');
            const std::size_t at = text.rfind('@');
            const auto spawn =
                at == std::string::npos ? std::nullopt : parseSpawn(text.c_str() + at + 1);
            if (equals == std::string::npos || equals == 0 || at == std::string::npos ||
                at < equals + 2 || !spawn) {
                return "--add-robot needs NAME=FILE@X,Y[,Z[,YAW]]";
            }
            options.extraRobots.push_back(
                {text.substr(0, equals), text.substr(equals + 1, at - equals - 1), *spawn});
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
        } else if (arg == "--replay") {
            const char* path = value();
            if (path == nullptr) {
                return "--replay needs a recording (FILE.mcap from --record)";
            }
            options.replayPath = path;
            options.ros = false;  // a replay has no robot to drive
        } else if (arg == "--session") {
            const char* path = value();
            if (path == nullptr) {
                return "--session needs a file";
            }
            options.sessionPath = path;
        } else if (arg == "--physics") {
            const char* text = value();
            const std::string_view backend = text != nullptr ? text : "";
            if (backend.empty()) {
                return "--physics needs mujoco, jolt or a plugin's backend";
            }
            options.physics = std::string(backend);
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
        } else if (arg == "--bench") {
            const char* path = value();
            if (path == nullptr) {
                return "--bench needs a recording (.mcap)";
            }
            options.benchReplayPath = path;
            options.substituted.insert("rigid_body");
        } else if (arg == "--ambient-temperature") {
            const char* text = value();
            char* end = nullptr;
            const double kelvin = text != nullptr ? std::strtod(text, &end) : -1.0;
            if (text == nullptr || end == text || *end != '\0' || !(kelvin > 0.0)) {
                return "--ambient-temperature needs a temperature in K";
            }
            options.ambientTemperatureK = kelvin;
        } else if (arg == "--mavlink") {
            const char* text = value();
            const std::string_view which = text != nullptr ? text : "";
            const bool px4 = which == "px4" || which.starts_with("px4:");
            const bool ardupilot = which == "ardupilot" || which.starts_with("ardupilot:");
            unsigned port = px4 ? 4560 : 9002;
            const std::size_t colon = which.find(':');
            if ((!px4 && !ardupilot) ||
                (colon != std::string_view::npos &&
                 (std::sscanf(text + colon + 1, "%u", &port) != 1 || port == 0 || port > 65535))) {
                return "--mavlink takes px4[:PORT] (PX4 SITL connects to TCP 4560) or "
                       "ardupilot[:PORT] (ArduPilot's JSON frames on UDP 9002)";
            }
            (px4 ? options.px4Port : options.ardupilotPort) = static_cast<std::uint16_t>(port);
        } else if (arg == "--geo-origin") {
            const char* text = value();
            double lat = 0.0, lon = 0.0, height = 0.0;
            char tail = 0;
            if (text == nullptr ||
                std::sscanf(text, "%lf,%lf,%lf%c", &lat, &lon, &height, &tail) != 3 ||
                std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
                return "--geo-origin needs LAT,LON,HEIGHT (degrees, degrees, m above WGS84)";
            }
            constexpr double kDegree = std::numbers::pi / 180.0;
            options.geoOrigin = coords::Geodetic{lat * kDegree, lon * kDegree, height};
        } else if (arg == "--air-density") {
            const char* text = value();
            char* end = nullptr;
            const double rho = text != nullptr ? std::strtod(text, &end) : -1.0;
            if (text == nullptr || end == text || *end != '\0' || !(rho > 0.0)) {
                return "--air-density needs a density in kg/m³";
            }
            options.airDensityKgM3 = rho;
        } else if (arg == "--scenario") {
            const char* text = value();
            if (text == nullptr) {
                return "--scenario needs an OpenSCENARIO file (.xosc)";
            }
            options.scenarioPath = text;
        } else if (arg == "--scenario-ego") {
            const char* text = value();
            if (text == nullptr) {
                return "--scenario-ego needs the scenario entity that is the robot";
            }
            options.scenarioEgo = text;
        } else if (arg == "--traffic") {
            const char* text = value();
            if (text == nullptr) {
                return "--traffic needs vehicles=N,pedestrians=M,seed=S";
            }
            std::string_view rest = text;
            while (!rest.empty()) {
                const std::size_t comma = rest.find(',');
                const std::string_view item = rest.substr(0, comma);
                rest = comma == std::string_view::npos ? "" : rest.substr(comma + 1);
                const std::size_t equals = item.find('=');
                const std::string key(item.substr(0, equals));
                const std::string number =
                    equals == std::string_view::npos ? "" : std::string(item.substr(equals + 1));
                char* end = nullptr;
                const long long n = std::strtoll(number.c_str(), &end, 10);
                if (number.empty() || *end != '\0' || n < 0) {
                    return "--traffic: " + std::string(item) + " is not KEY=N (N ≥ 0)";
                }
                if (key == "vehicles") {
                    options.trafficVehicles = static_cast<int>(n);
                } else if (key == "pedestrians") {
                    options.trafficPedestrians = static_cast<int>(n);
                } else if (key == "seed") {
                    options.trafficSeed = static_cast<std::uint64_t>(n);
                } else {
                    return "--traffic takes vehicles, pedestrians and seed, not " + key;
                }
            }
        } else if (arg == "--link-topics") {
            // A YAML map from a link (the receiving radio's prim) to the topics on it.
            const char* text = value();
            if (text == nullptr) {
                return "--link-topics needs a YAML file: LINK: [/topic, ...]";
            }
            try {
                const YAML::Node file = YAML::LoadFile(text);
                if (!file.IsMap()) {
                    return std::string(text) + ": a map from links to lists of topics";
                }
                for (const auto& entry : file) {
                    const std::string link = entry.first.as<std::string>();
                    if (!entry.second.IsSequence()) {
                        return std::string(text) + ": " + link + " needs a list of topics";
                    }
                    for (const auto& topic : entry.second) {
                        options.linkTopics[topic.as<std::string>()] = link;
                    }
                }
            } catch (const YAML::Exception& e) {
                return std::string(text) + ": " + e.what();
            }
        } else if (arg == "--weather") {
            const char* text = value();
            if (text == nullptr) {
                return "--weather needs KEY=VALUE,... (temperature=288.15,pressure=101325,...)";
            }
            static const std::set<std::string, std::less<>> kKeys{
                "temperature",          "pressure",         "relative_humidity",
                "visibility",           "rain_rate",        "dust_concentration",
                "dust_median_diameter", "solar_irradiance", "sky_temperature"};
            std::string list(text);
            std::size_t at = 0;
            while (at <= list.size()) {
                const std::size_t comma = std::min(list.find(',', at), list.size());
                const std::string item = list.substr(at, comma - at);
                const std::size_t equals = item.find('=');
                if (equals != std::string::npos && (item.substr(0, equals) == "fog_field" ||
                                                    item.substr(0, equals) == "wind_field")) {
                    options.weatherFiles[item.substr(0, equals)] = item.substr(equals + 1);
                    at = comma + 1;
                    continue;
                }
                char* end = nullptr;
                const double v = equals == std::string::npos
                                     ? 0.0
                                     : std::strtod(item.c_str() + equals + 1, &end);
                if (equals == std::string::npos || !kKeys.contains(item.substr(0, equals)) ||
                    end == item.c_str() + equals + 1 || *end != '\0') {
                    return "--weather: \"" + item +
                           "\" is not KEY=VALUE with KEY one of temperature, pressure, "
                           "relative_humidity, visibility, rain_rate, dust_concentration, "
                           "dust_median_diameter, solar_irradiance, sky_temperature, fog_field, "
                           "wind_field";
                }
                options.weather[item.substr(0, equals)] = v;
                at = comma + 1;
            }
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
            // Any other name is a plugin's domain (RFC 0008), checked once the plugins load.
            const bool named = !domain.empty() && std::ranges::all_of(domain, [](char c) {
                return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
            });
            if (!named || (state != "on" && state != "off")) {
                return "--domain takes rigid_body, electrical, thermal, weather, emi, comms, "
                       "vehicle_dynamics, traffic, sensors or a plugin's domain =on|off";
            }
            if (state == "off") {
                options.substituted.insert(domain);
            } else {
                options.substituted.erase(domain);
            }
            thermalAsked = domain == "thermal" ? state == "on" : thermalAsked;
        } else if (arg == "--ros-topics") {
            const char* path = value();
            if (path == nullptr) {
                return "--ros-topics needs a YAML file: publish: [{signal, topic, type}], "
                       "subscribe: [...]";
            }
            options.rosTopicsPath = path;
        } else if (arg == "--plugin") {
            const char* library = value();
            if (library == nullptr) {
                return "--plugin needs a library: a path, or a name in LAGRANGE_PLUGIN_PATH or "
                       "the lagrange_plugins index";
            }
            options.plugins.emplace_back(library);
        } else if (arg == "--cmd-vel-unstamped") {
            options.stampedCmdVel = false;
        } else if (arg == "--ros2-control") {
            const char* path = value();
            if (path == nullptr) {
                return "--ros2-control needs the controllers' parameter file";
            }
            options.ros2ControlParams = path;
        } else if (arg == "--no-demo") {
            options.demoDrive = false;
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
    if (options.robotPath.empty() && !out.robotGiven) {
        return "no --robot given and lagrange_assets is not installed";
    }
    if (thermalAsked && !options.ambientTemperatureK) {
        return "--domain thermal=on needs the site's ambient temperature: --ambient-temperature K";
    }
    if (!options.scenarioPath.empty() && !out.worldGiven) {
        options.worldPath.clear();  // the scenario's own road network is the world
    }
    if (out.robotGiven && !out.overlayGiven &&
        std::filesystem::path(options.robotPath).filename() != "amr_diffdrive.urdf") {
        options.overlayPath.clear();  // the default overlay fits the sample robot only
    }
    return std::nullopt;
}

int serve(CommandLine& line, int argc, char** argv, const std::string* profileYaml,
          const char* program) {
    ServerOptions& options = line.options;
    if (!line.exportMap.empty()) {
        // Only the fixed scenery is mapped; the robot is loaded for its frame of reference.
        auto robot = options.robotPath.empty()
                         ? Expected<Robot, std::string>(emptyModel())
                         : loadModel(options.robotPath, {.spawn = options.spawn});
        if (robot && !options.worldPath.empty()) {
            if (auto added = addModel(*robot, options.worldPath); !added) {
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
