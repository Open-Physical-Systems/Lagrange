#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/plugin_abi/PluginLibrary.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::server {

/// One module of a loaded plugin library (RFC 0008 §1).
struct PluginModule {
    std::string library;  // as the command line or profile gave it
    std::string path;     // the file loaded
    std::string domain;   // "rigid_body", or a domain of its own ("acme_sonar")
    std::string name;     // its module name: a backend or substitute token
    bool substitute{false};
    bool deterministic{true};
    PluginLibrary file;
    std::size_t index{0};

    /// A new instance for a run.
    [[nodiscard]] std::unique_ptr<IModule> create() const { return file.create(index); }
};

/// The file a plugin library name stands for: a path as given (it has a '/' or ends in .so), or
/// lib<name>.so in the directories of $LAGRANGE_PLUGIN_PATH, then the library the
/// `lagrange_plugins` ament resource of that name points to (a path relative to its prefix).
[[nodiscard]] Expected<std::string, std::string> resolvePlugin(const std::string& library);

/// Loads every library and lists its modules. Fails, naming the library, for a file that is not
/// found, another ABI major, no modules, a module that cannot be created, or two modules with
/// one domain/name.
[[nodiscard]] Expected<std::vector<PluginModule>, std::string> loadPlugins(
    const std::vector<std::string>& libraries);

/// The domains the server builds in: a plugin module of one of these is a backend or substitute
/// of it; any other domain is the plugin's own.
[[nodiscard]] bool isBuiltInDomain(std::string_view domain);

}  // namespace lagrange::server
