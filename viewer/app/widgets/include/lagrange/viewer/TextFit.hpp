#pragma once

// Text that must fit a width: cut with an ellipsis at the end (a title) or in the middle (a
// signal path, whose start and end both say what it is). The width of a string comes from the
// caller (the font), so this has no ImGui in it.

#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

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

/// `text` cut to `width` so that it reads differently from every other name of `group` (texts
/// cut the same way by fitPath, such as "joint/front_caster_swivel_joint/position" and
/// "joint/front_caster_wheel_joint/position", both "joint/front_cas…/position"): the first of
/// fitPath, fitPath with its start cut in the middle ("joint/fro…wheel_joint/position"), the end
/// alone ("…wheel_joint/position"), fitMiddle and fitEnd that tells all of `group` apart; fitPath
/// when none does. `group` holds `text` too.
[[nodiscard]] std::string fitDistinct(std::string_view text, std::span<const std::string> group,
                                      float width, const TextWidth& measure);

/// The cells of a row of options that must fit `available` (the Segmented control at a narrow
/// panel width, the joint type's "fixed revolute continuous prismatic"): each label's width plus
/// `padding` on both sides when that fits; else the padding shrinks, down to `minPadding`, and the
/// cells fill `available`; else the longest labels are cut to one common width (the shorter ones
/// stay whole) so that the row is `available` wide. `labels` are the labels' widths; the result
/// has one width per label. Labels whose cell is narrower than label + 2 × minPadding are cut
/// (fitEnd) by the caller.
[[nodiscard]] std::vector<float> fitCells(std::span<const float> labels, float available,
                                          float padding, float minPadding);

}  // namespace lagrange::viewer
