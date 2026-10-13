#pragma once

#include "lagrange/viewer/ProfileSession.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

/// Text formatting shared by panels. Everything writes into a caller's buffer: panels do not
/// allocate while drawing.
namespace lagrange::viewer::format {

/// "00:03.413" from simulation nanoseconds.
void clock(std::int64_t simTimeNs, char* buffer, std::size_t size);
/// A step in the unit a person would pick: "100 µs", "1 ms", "0.1 s".
void step(std::int64_t dtNs, char* buffer, std::size_t size);
/// The mono text on a domain chip: "mujoco · 1 ms", "→ zero", with "· pending" when edited.
void domainDetail(const DomainView& view, char* buffer, std::size_t size);
/// One of the eight physics domains (as opposed to an additional module such as comms).
[[nodiscard]] bool isPhysicsDomain(std::string_view name);

}  // namespace lagrange::viewer::format
