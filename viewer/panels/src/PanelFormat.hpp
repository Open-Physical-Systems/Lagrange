#pragma once

#include "lagrange/viewer/ProfileSession.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include <imgui.h>

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

/// Text cut to a width with an ellipsis (lagrange/viewer/TextFit.hpp, measured in an ImGui font):
/// at the end for titles; `middle` for paths and signal names (fitPath: the last part whole).
namespace lagrange::viewer::fit {

/// `text` cut to `width` in `font` at `size` (pixels).
[[nodiscard]] std::string cut(ImFont* font, float size, std::string_view text, float width,
                              bool middle = false);

/// `text` cut to `width` so that it reads differently from the other names of `group`
/// (lagrange/viewer/TextFit.hpp fitDistinct).
[[nodiscard]] std::string cutDistinct(ImFont* font, float size, std::string_view text,
                                      std::span<const std::string> group, float width);

/// Draws `text` at `at`, cut to `width`. Returns true when it was cut.
bool draw(ImDrawList* draw, ImFont* font, float size, const ImVec2& at, ImU32 color,
          std::string_view text, float width, bool middle = false);

/// TextUnformatted in the current font and colour, cut to `width`; hovering a cut text shows
/// all of it in a tooltip.
void text(std::string_view text, float width, bool middle = false);

}  // namespace lagrange::viewer::fit
