#include "App.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

void usage() {
    std::puts(
        "Usage: lagrange_viewer [options]\n"
        "\n"
        "  --connect <endpoint>   unix:/path/to.sock or tcp://host:port\n"
        "                         (default: the Unix socket in $XDG_RUNTIME_DIR/lagrange)\n"
        "  --gpu <index|name>     Vulkan device to use (see lagrange_gpu_info)\n"
        "  --validation           enable Vulkan validation layers\n"
        "  --size <W>x<H>         window or offscreen size in pixels (default 1600x900)\n"
        "  --headless             no window: render offscreen and exit\n"
        "  --frames <N>           headless: frames to render once the scene arrived (default "
        "120);\n"
        "                         with a window: close after N frames\n"
        "  --wait-ms <N>          headless: how long to wait for the scene (default 5000)\n"
        "  --screenshot <file>    headless: write the last frame as a PNG\n"
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
        if (arg == "--connect") {
            options.endpoint = value();
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
