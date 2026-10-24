#include "lagrange/services/DeterminismCheck.hpp"

#include <algorithm>

namespace lagrange::services {

DeterminismReport checkDeterminism(
    const std::function<Expected<std::unique_ptr<CheckedRun>, std::string>()>& make, SimTime end,
    Duration every) {
    DeterminismReport report;
    auto a = make();
    auto b = make();
    if (!a || !b) {
        report.error = !a ? a.error() : b.error();
        return report;
    }
    const std::int64_t step = std::max<std::int64_t>(every.ns, 1);
    for (std::int64_t t = 0; t <= end.ns; t += step) {
        const SimTime at{t};
        if (Status s = (*a)->stepUntil(at); !s.ok()) {
            report.error = s.message;
            return report;
        }
        if (Status s = (*b)->stepUntil(at); !s.ok()) {
            report.error = s.message;
            return report;
        }
        const auto ha = (*a)->stateHash();
        const auto hb = (*b)->stateHash();
        if (!ha || !hb) {
            report.error = !ha ? ha.error().message : hb.error().message;
            return report;
        }
        ++report.comparisons;
        if (*ha != *hb) {
            report.firstDivergence = at;
            return report;
        }
    }
    report.identical = true;
    return report;
}

}  // namespace lagrange::services
