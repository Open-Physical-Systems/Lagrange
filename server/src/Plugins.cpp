#include "lagrange/server/Plugins.hpp"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>

#include <ament_index_cpp/get_resource.hpp>

namespace lagrange::server {

namespace fs = std::filesystem;

Expected<std::string, std::string> resolvePlugin(const std::string& library) {
    if (library.find('/') != std::string::npos || library.ends_with(".so")) {
        if (!fs::is_regular_file(library)) {
            return Unexpected{"plugin " + library + ": no such file"};
        }
        return fs::absolute(library).string();
    }
    const std::string file = "lib" + library + ".so";
    if (const char* paths = std::getenv("LAGRANGE_PLUGIN_PATH"); paths != nullptr) {
        std::istringstream list(paths);
        for (std::string directory; std::getline(list, directory, ':');) {
            if (!directory.empty() && fs::is_regular_file(fs::path(directory) / file)) {
                return fs::absolute(fs::path(directory) / file).string();
            }
        }
    }
    const ament_index_cpp::PathWithResource indexed =
        ament_index_cpp::get_resource("lagrange_plugins", library);
    if (indexed.resourcePath) {
        std::string relative = indexed.contents;
        while (!relative.empty() && (relative.back() == '\n' || relative.back() == ' ')) {
            relative.pop_back();
        }
        const fs::path found = *indexed.resourcePath / relative;
        if (fs::is_regular_file(found)) {
            return found.string();
        }
        return Unexpected{"plugin " + library + ": the lagrange_plugins index names " +
                          found.string() + ", which does not exist"};
    }
    return Unexpected{"plugin " + library + ": no " + file +
                      " in LAGRANGE_PLUGIN_PATH and no lagrange_plugins resource of that name"};
}

bool isBuiltInDomain(std::string_view domain) {
    static constexpr std::array<std::string_view, 14> kDomains{
        "rigid_body", "electrical", "thermal",          "aerodynamics", "weather",       "emi",
        "radiation",  "sensors",    "vehicle_dynamics", "rotor_aero",   "hydrodynamics", "traffic",
        "comms",      "humans"};
    return std::ranges::find(kDomains, domain) != kDomains.end();
}

Expected<std::vector<PluginModule>, std::string> loadPlugins(
    const std::vector<std::string>& libraries) {
    std::vector<PluginModule> modules;
    std::set<std::string> labels;
    for (const std::string& library : libraries) {
        auto path = resolvePlugin(library);
        if (!path) {
            return Unexpected{path.error()};
        }
        auto file = PluginLibrary::open(*path);
        if (!file) {
            return Unexpected{"plugin " + library + ": " + file.error()};
        }
        for (std::size_t index = 0; index < file->size(); ++index) {
            const std::unique_ptr<IModule> probe = file->create(index);
            if (!probe) {
                return Unexpected{"plugin " + library + ": module " + file->label(index) +
                                  " could not be created"};
            }
            const ModuleInfo info = probe->info();
            PluginModule module{library,
                                *path,
                                std::string(info.domain),
                                std::string(info.name),
                                info.isSubstitute,
                                info.deterministic,
                                *file,
                                index};
            const std::string label = module.domain + "/" + module.name;
            if (!labels.insert(label).second) {
                return Unexpected{"plugin " + library + ": a module " + label +
                                  " is already loaded"};
            }
            modules.push_back(std::move(module));
        }
    }
    return modules;
}

}  // namespace lagrange::server
