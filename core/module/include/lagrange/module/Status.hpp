#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace lagrange {

/// Outcome of a module call. Modules never throw: exceptions cannot cross the plugin C ABI
/// (RFC 0001).
enum class StatusCode : std::uint8_t {
    Ok,
    InvalidConfig,
    UnitMismatch,
    MissingInput,
    Diverged,  // the scheduler also dumps the run state for debugging
    Unsupported,
    Internal,
};

struct Status {
    StatusCode code{StatusCode::Ok};
    std::string message;

    [[nodiscard]] bool ok() const { return code == StatusCode::Ok; }

    static Status success() { return {}; }
    static Status error(StatusCode code, std::string message) { return {code, std::move(message)}; }
};

[[nodiscard]] std::string_view to_string(StatusCode code);

}  // namespace lagrange
