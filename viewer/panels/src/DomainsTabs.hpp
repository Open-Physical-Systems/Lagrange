#pragma once

#include "lagrange/viewer/Panel.hpp"

#include <memory>
#include <vector>

namespace lagrange::viewer {

/// The Domains workspace's secondary tabs: Modules, Coupling, Diff and Schema
/// (docs/design/ui/03-domains/SPEC.md).
[[nodiscard]] std::vector<std::unique_ptr<Panel>> createDomainsTabPanels();

}  // namespace lagrange::viewer
