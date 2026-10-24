#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/IModule.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

struct lagrange_module_api;

namespace lagrange {

/// A shared object holding modules built against the C ABI (lagrange_plugin.h, invariant 10).
/// Each module it holds becomes an IModule through an adapter, so the scheduler wires and steps
/// it like a built-in one. The library stays loaded as long as this object or any module made
/// from it exists.
class PluginLibrary {
public:
    /// dlopen()s `path` and asks it for its modules with this host's ABI version. Fails with the
    /// reason for a missing file, a missing lagrange_plugin_get_module, another ABI major, or no
    /// modules.
    [[nodiscard]] static Expected<PluginLibrary, std::string> open(const std::string& path);

    /// Number of modules in the library.
    [[nodiscard]] std::size_t size() const;
    /// "domain/name" of module `index`.
    [[nodiscard]] std::string label(std::size_t index) const;
    /// A new instance of module `index`; nullptr when its create() fails.
    [[nodiscard]] std::unique_ptr<IModule> create(std::size_t index) const;

    struct Handle;

private:
    std::shared_ptr<Handle> handle_;
};

}  // namespace lagrange
