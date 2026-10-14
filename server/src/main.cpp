// lagrange_server: simulate a robot and serve it to viewers.
//
//   lagrange_server [--robot FILE.urdf] [--listen ENDPOINT] [--rtf X] [--paused]
//
// Defaults: the sample AMR of lagrange_assets, and the default Unix socket that the viewer
// connects to when started without --connect.
#include "lagrange/server/Server.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>

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
                 "usage: lagrange_server [--robot FILE.urdf] [--listen ENDPOINT] [--rtf X] "
                 "[--paused]\n"
                 "  --robot   URDF to simulate (default: the sample AMR from lagrange_assets)\n"
                 "  --listen  unix:/path.sock or tcp://host:port (default: %s)\n"
                 "  --rtf     real-time factor, 0 = as fast as possible (default 1)\n"
                 "  --paused  start paused; press Play in the viewer\n",
                 lagrange::proto::defaultUnixSocketPath().c_str());
    return error != nullptr ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    lagrange::server::ServerOptions options;
    options.endpoint =
        lagrange::proto::Endpoint::unixSocket(lagrange::proto::defaultUnixSocketPath());
    options.robotPath = shareFile("lagrange_assets", "robots/amr_diffdrive/amr_diffdrive.urdf");
    options.profileSchemaPath = shareFile("lagrange_profiles", "schema.json");

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        const auto value = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        if (arg == "--help" || arg == "-h") {
            return usage(nullptr);
        }
        if (arg == "--robot") {
            const char* path = value();
            if (path == nullptr) {
                return usage("--robot needs a file");
            }
            options.robotPath = path;
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
        } else {
            return usage(("unknown option " + std::string(arg)).c_str());
        }
    }
    if (options.robotPath.empty()) {
        return usage("no --robot given and lagrange_assets is not installed");
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    lagrange::server::Server server(options);
    if (const lagrange::Status status = server.start(); !status.ok()) {
        std::fprintf(stderr, "lagrange_server: %s\n", status.message.c_str());
        return 1;
    }
    while (!gStop) {
        server.runOnce(5);
    }
    return 0;
}
