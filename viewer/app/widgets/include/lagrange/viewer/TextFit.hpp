#pragma once

// Text that must fit a width: cut with an ellipsis at the end (a title) or in the middle (a
// signal path, whose start and end both say what it is). The width of a string comes from the
// caller (the font), so this has no ImGui in it.

#include <functional>
#include <string>
#include <string_view>

namespace lagrange::viewer {

/// The width of a string as it would be drawn.
using TextWidth = std::function<float(std::string_view)>;

/// `text` itself when it fits `width`; otherwise its start and "…" that fit. Cuts only between
/// UTF-8 characters. Returns "…" alone when not even one character fits.
[[nodiscard]] std::string fitEnd(std::string_view text, float width, const TextWidth& measure);

/// `text` itself when it fits `width`; otherwise its start, "…" and its end that fit, the end
/// keeping at least as many characters as the start. Cuts only between UTF-8 characters.
[[nodiscard]] std::string fitMiddle(std::string_view text, float width, const TextWidth& measure);

/// For a path or a signal name ("joint/front_caster_swivel_joint/position"): its last part
/// stays whole and the rest is cut at its end ("joint/front_caster…/position"); when even the
/// last part does not fit, fitMiddle.
[[nodiscard]] std::string fitPath(std::string_view text, float width, const TextWidth& measure);

}  // namespace lagrange::viewer
