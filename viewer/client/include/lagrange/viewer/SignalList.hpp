#pragma once

// The rows of the Signals browser (docs/design/ui/05-signals/SPEC.md: one row per signal path).
// A signal with several components (a pose, a lidar's 1080 ranges) is one row that opens to
// list them; a search that names a component ("ranges.10") shows its signal open on just the
// components that match. No ImGui here.

#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

struct SignalFilter {
    std::string search;  ///< In the name, the unit, or "name.component"; case does not matter.
    std::string unit;    ///< Only this unit; empty: any.
    bool fastOnly{false};
    std::int64_t fastDtNs{10'000'000};  ///< "rate ≥ 100 Hz": a sample period this short or less.
};

struct SignalRow {
    /// The `component` of a signal's own row when it has several (an array, a pose).
    static constexpr std::uint32_t kWhole = 0xFFFFFFFFU;
    std::uint32_t signal{0};     ///< Index in SignalCatalog::signals.
    std::uint32_t component{0};  ///< kWhole, or the component this row plots.
    bool expanded{false};        ///< A kWhole row whose components follow it.
};

/// The domain a signal is listed under: the first part of the module that writes it.
[[nodiscard]] std::string signalDomain(const SignalInfo& info);

/// The rows of one domain's group, in catalog order. `open` holds the names of the signals the
/// user opened. `listed` (when given) is increased by the number of signals listed.
[[nodiscard]] std::vector<SignalRow> signalRows(const SignalCatalog& catalog,
                                                std::string_view domain, const SignalFilter& filter,
                                                const std::set<std::string, std::less<>>& open,
                                                std::size_t* listed = nullptr);

/// How a component is named in its row: its name ("x", "qw"), or "[i]" for a numbered one.
[[nodiscard]] std::string componentLabel(const SignalInfo& info, std::uint32_t component);

/// The full name of one component as a scope plots it: "name.x", or the name of a scalar.
[[nodiscard]] std::string componentPath(const SignalInfo& info, std::uint32_t component);

}  // namespace lagrange::viewer
