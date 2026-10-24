#pragma once

// The determinism check (roadmap 0.2-H2, invariant 7): the same run twice must give
// bit-identical state at every point. It catches what a reviewer cannot see, such as a sum
// whose result depends on thread order.

#include "lagrange/module/Expected.hpp"
#include "lagrange/module/Status.hpp"
#include "lagrange/time/SimTime.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace lagrange::services {

/// One run under test: advances to a time and hashes its whole state (Scheduler::stateHash).
class CheckedRun {
public:
    CheckedRun() = default;
    CheckedRun(const CheckedRun&) = delete;
    CheckedRun& operator=(const CheckedRun&) = delete;
    CheckedRun(CheckedRun&&) = delete;
    CheckedRun& operator=(CheckedRun&&) = delete;
    virtual ~CheckedRun() = default;
    virtual Status stepUntil(SimTime t) = 0;
    [[nodiscard]] virtual Expected<std::uint64_t, Status> stateHash() const = 0;
};

struct DeterminismReport {
    bool identical{false};
    std::uint32_t comparisons{0};
    std::optional<SimTime> firstDivergence;  // the first compared time whose hashes differ
    std::string error;                       // a run that failed to build or step
};

/// Builds two runs with `make`, steps them side by side to `end`, and compares their state
/// hashes every `every` of simulated time. Stops at the first divergence.
[[nodiscard]] DeterminismReport checkDeterminism(
    const std::function<Expected<std::unique_ptr<CheckedRun>, std::string>()>& make, SimTime end,
    Duration every);

}  // namespace lagrange::services
