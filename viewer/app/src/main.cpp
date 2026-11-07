#include "App.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace {

void usage() {
    std::puts(
        "Usage: lagrange_viewer [options]\n"
        "\n"
        "  --connect <endpoint>   unix:/path/to.sock or tcp://host:port\n"
        "  --tls-ca <file>        CA certificates (PEM) for a tls:// server (default: the "
        "system's)\n"
        "  --auth-token-file <f>  the token a TCP server asks for (first line of the file)\n"
        "                         (default: the Unix socket in $XDG_RUNTIME_DIR/lagrange)\n"
        "  --gpu <index|name>     Vulkan device to use (see lagrange_gpu_info)\n"
        "  --validation           enable Vulkan validation layers\n"
        "  --size <W>x<H>         window or offscreen size in pixels (default 1600x900)\n"
        "  --open <file>          ask the server to load a robot (.urdf, .xacro) or recording "
        "(.mcap)\n"
        "  --scope <a,b,...>      open these signals in the scope at start-up\n"
        "  --workspace <name>     workspace to show at start-up: scene, domains or signals\n"
        "  --shading <mode>       viewport shading at start-up: lit, unlit, wireframe, collision\n"
        "                         or thermal\n"
        "  --follow <m>[,<deg>]   start with the follow camera, this far from the robot (and this\n"
        "                         far above the horizon)\n"
        "  --camera <x,y,z,m,yaw,pitch>  start the camera looking at x,y,z from m metres away, at\n"
        "                         these angles in degrees\n"
        "  --season <name>        how the realistic view draws the year: summer, fall, winter, "
        "rainy\n"
        "  --shading <mode>       realistic (the default on a GPU), lit (without one), unlit, ...\n"
        "  --overlays <list>      show these from the start: grid,axes,sensors,airflow\n"
        "  --slice <field>[:xy|xz|yz]  the Slice 2D panel's field and plane, e.g.\n"
        "                         aero/wind_speed:xz\n"
        "  --launcher             start on the launcher (the default when no --connect is given)\n"
        "  --start-local          press the launcher's Start local simulator: run lagrange_server\n"
        "                         on the --connect endpoint (or the default socket) and connect\n"
        "  --cursors              show the scope's A and B cursors from the start\n"
        "  --export-netlist <f>   write the stage's power system to this netlist file\n"
        "  --input <script>       headless mouse and keyboard, by frame: "
        "50:click:400,200;60:text:12;61:key:enter (keys: enter, tab, escape)\n"
        "  --panel <title>        bring this panel in front of its tabs, e.g. Circuit\n"
        "  --select <prim path>   select this prim once the scene arrived, e.g. /amr/base_link\n"
        "  --set-domain <d>=on|off  switch a domain in the profile being edited (repeatable)\n"
        "  --apply-profile        apply the edited profile once the server calls it valid\n"
        "  --headless             no window: render offscreen and exit\n"
        "  --frames <N>           headless: frames to render once the scene arrived (default "
        "120);\n"
        "                         with a window: close after N frames\n"
        "  --wait-ms <N>          headless: how long to wait for the scene (default 5000)\n"
        "  --screenshot <file>    headless: write the last frame as a PNG\n"
        "  --record-frames <dir>  headless: write a frame sequence (frame_00000.png, ...) to dir\n"
        "  --record-fps <N>       headless: frames per second written by --record-frames "
        "(default 15)\n"
        "  --help\n");
}

}  // namespace

int main(int argc, char** argv) {
    lagrange::viewer::Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "error: %s needs a value\n", arg.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        }
        if (arg == "--ros-args") {
            break;  // the viewer is not a ROS node; ros2 run and ros2 launch may append these
        }
        if (arg == "--connect") {
            options.endpoint = value();
        } else if (arg == "--tls-ca") {
            options.tlsCa = value();
        } else if (arg == "--auth-token-file") {
            std::ifstream in(value());
            if (!in || !std::getline(in, options.authToken) || options.authToken.empty()) {
                std::fputs("error: --auth-token-file needs a file whose first line is the token\n",
                           stderr);
                return 2;
            }
        } else if (arg == "--gpu") {
            options.gpu = value();
        } else if (arg == "--validation") {
            options.validation = true;
        } else if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--frames") {
            options.frames = static_cast<std::uint32_t>(std::strtoul(value(), nullptr, 10));
        } else if (arg == "--wait-ms") {
            options.waitMs = static_cast<std::uint32_t>(std::strtoul(value(), nullptr, 10));
        } else if (arg == "--screenshot") {
            options.screenshot = value();
        } else if (arg == "--follow") {
            char* rest = nullptr;
            options.followDistance = std::strtod(value(), &rest);
            if (rest != nullptr && *rest == ',') {
                options.followPitchDeg = std::strtod(rest + 1, nullptr);
            }
            if (!(options.followDistance > 0.0)) {
                std::fputs("error: --follow expects a distance in metres\n", stderr);
                return 2;
            }
        } else if (arg == "--camera") {
            std::array<double, 6> view{};
            if (std::sscanf(value(), "%lf,%lf,%lf,%lf,%lf,%lf", &view[0], &view[1], &view[2],
                            &view[3], &view[4], &view[5]) != 6 ||
                !(view[3] > 0.0)) {
                std::fputs("error: --camera expects x,y,z,distance,yaw,pitch\n", stderr);
                return 2;
            }
            options.camera = view;
        } else if (arg == "--record-frames") {
            options.recordFrames = value();
        } else if (arg == "--record-fps") {
            options.recordFps = static_cast<std::uint32_t>(std::strtoul(value(), nullptr, 10));
            if (options.recordFps == 0) {
                std::fputs("error: --record-fps must be at least 1\n", stderr);
                return 2;
            }
        } else if (arg == "--set-domain") {
            const std::string edit = value();
            const std::size_t equals = edit.find('=');
            const std::string state = equals == std::string::npos ? "" : edit.substr(equals + 1);
            if (equals == 0 || (state != "on" && state != "off")) {
                std::fputs("error: --set-domain expects <domain>=on or <domain>=off\n", stderr);
                return 2;
            }
            options.domainEdits.emplace_back(edit.substr(0, equals), state == "on");
        } else if (arg == "--launcher") {
            options.launcher = true;
        } else if (arg == "--start-local") {
            options.startLocal = true;
        } else if (arg == "--cursors") {
            options.cursors = true;
        } else if (arg == "--export-netlist") {
            options.exportNetlist = value();
        } else if (arg == "--input") {
            options.input = value();
        } else if (arg == "--panel") {
            options.focusPanel = value();
        } else if (arg == "--select") {
            options.selectPath = value();
        } else if (arg == "--apply-profile") {
            options.applyProfile = true;
        } else if (arg == "--workspace") {
            options.workspace = value();
        } else if (arg == "--shading") {
            options.shading = value();
        } else if (arg == "--season") {
            options.season = value();
        } else if (arg == "--overlays") {
            options.overlays = value();
        } else if (arg == "--slice") {
            options.slice = value();
        } else if (arg == "--open") {
            options.openPath = value();
        } else if (arg == "--scope") {
            const std::string list = value();
            for (std::size_t start = 0; start <= list.size();) {
                const std::size_t comma = std::min(list.find(',', start), list.size());
                if (comma > start) {
                    options.scopeSignals.push_back(list.substr(start, comma - start));
                }
                start = comma + 1;
            }
        } else if (arg == "--size") {
            unsigned width = 0;
            unsigned height = 0;
            if (std::sscanf(value(), "%ux%u", &width, &height) != 2 || width < 64 || height < 64 ||
                width > 16384 || height > 16384) {
                std::fputs("error: --size expects <W>x<H>, each between 64 and 16384\n", stderr);
                return 2;
            }
            options.width = width;
            options.height = height;
        } else {
            std::fprintf(stderr, "error: unknown option %s\n\n", arg.c_str());
            usage();
            return 2;
        }
    }
    return lagrange::viewer::runViewer(options);
}
