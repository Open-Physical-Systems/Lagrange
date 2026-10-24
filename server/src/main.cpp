// lagrange_server: simulate a robot and serve it to viewers.
//
//   lagrange_server [--robot FILE.urdf] [--listen ENDPOINT] [--rtf X] [--paused]
//                   [--ros-args ...]
//
// Defaults: the sample AMR of lagrange_assets, and the default Unix socket that the viewer
// connects to when started without --connect. `lagrange run PROFILE` is the same with a
// profile applied.
#include "CommandLine.hpp"

#include <cstdio>

int main(int argc, char** argv) {
    lagrange::server::CommandLine line = lagrange::server::defaultCommandLine();
    const auto error = lagrange::server::parseCommandLine(argc, argv, 1, false, line);
    if (error || line.help) {
        if (error) {
            std::fprintf(stderr, "lagrange_server: %s\n", error->c_str());
        }
        std::fprintf(stderr, "usage: lagrange_server [options] [--ros-args ...]\n");
        lagrange::server::printServerOptions(stderr);
        return error ? 2 : 0;
    }
    return lagrange::server::serve(line, argc, argv, nullptr, "lagrange_server");
}
