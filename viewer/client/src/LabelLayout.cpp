#include "lagrange/viewer/LabelLayout.hpp"

#include <algorithm>
#include <numeric>

namespace lagrange::viewer {

std::vector<float> stackLabels(const std::vector<float>& anchors, float height, float gap,
                               float top, float bottom) {
    const std::size_t n = anchors.size();
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::ranges::stable_sort(order,
                             [&](std::size_t a, std::size_t b) { return anchors[a] < anchors[b]; });
    std::vector<float> placed(n, top);
    if (n == 0) {
        return placed;
    }
    const float step = height + gap;
    // Down from the top: each where it wants to be, or under the one above.
    float next = top;
    for (const std::size_t i : order) {
        const float wanted = std::max(anchors[i] - height * 0.5F, top);
        placed[i] = std::max(wanted, next);
        next = placed[i] + step;
    }
    // Up from the bottom: what ran past it moves up, as long as the column fits.
    float limit = bottom - height;
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        placed[*it] = std::max(std::min(placed[*it], limit), top);
        limit = placed[*it] - step;
    }
    // When the column is taller than the room, the top wins: keep them apart from the top down.
    next = top;
    for (const std::size_t i : order) {
        placed[i] = std::max(placed[i], next);
        next = placed[i] + step;
    }
    return placed;
}

}  // namespace lagrange::viewer
