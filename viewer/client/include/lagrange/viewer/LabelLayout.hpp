#pragma once

// Labels over an image that must not cover each other (the Electrical 3D overlay's parts): each
// is placed in a column beside the points, as near the height of its point as the others let
// it, and joined to it by a leader line. No ImGui here: positions in, positions out.

#include <vector>

namespace lagrange::viewer {

/// Where each label goes, in the order of `anchors`: the top of each label, so that labels of
/// `height` are at least `gap` apart, each as close as it can be to having its middle at its
/// anchor, and all within [top, bottom] when they fit there (from `top` down when they do not).
[[nodiscard]] std::vector<float> stackLabels(const std::vector<float>& anchors, float height,
                                             float gap, float top, float bottom);

}  // namespace lagrange::viewer
