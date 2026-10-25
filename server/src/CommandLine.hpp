#pragma once

// The command line that lagrange_server and `lagrange run|validate|graph` share.

#include "lagrange/server/Server.hpp"

#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::server {

struct CommandLine {
    ServerOptions options;
    std::string exportMap;  // --export-map
    bool robotGiven{false};
    bool worldGiven{false};  // --world, even `none`
    bool overlayGiven{false};
    bool help{false};
    std::vector<std::string> positional;  // when allowed
};

/// Defaults: the sample AMR in the sample warehouse, the default Unix socket, the installed
/// profile schema, the ROS 2 bridge on.
[[nodiscard]] CommandLine defaultCommandLine();

/// Parses argv[first..] up to --ros-args into `out`. Arguments not starting with "--" go to
/// `positional` when `allowPositional`. Returns the error, if any.
[[nodiscard]] std::optional<std::string> parseCommandLine(int argc, char** argv, int first,
                                                          bool allowPositional, CommandLine& out);

/// One line per server option, for usage texts.
void printServerOptions(std::FILE* out);

/// Runs the server until SIGINT, SIGTERM or a quit request; with `profileYaml`, applies it
/// before the run starts. Returns the exit code.
int serve(CommandLine& line, int argc, char** argv, const std::string* profileYaml,
          const char* program);

/// A share file of an installed package, or "" when the package is not installed.
[[nodiscard]] std::string shareFile(const char* package, const char* relative);

}  // namespace lagrange::server
