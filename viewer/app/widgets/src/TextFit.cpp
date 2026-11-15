#include "lagrange/viewer/TextFit.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <vector>

namespace lagrange::viewer {

namespace {

constexpr std::string_view kEllipsis = "…";
constexpr std::size_t kMinimumHead = 9;  // characters fitPath keeps of a path's start

/// The byte offsets where the characters of `text` start, and its size at the end.
std::vector<std::size_t> characterStarts(std::string_view text) {
    std::vector<std::size_t> starts;
    for (std::size_t i = 0; i < text.size(); ++i) {
        // A UTF-8 continuation byte is 10xxxxxx.
        if ((static_cast<unsigned char>(text[i]) & 0xC0U) != 0x80U) {
            starts.push_back(i);
        }
    }
    starts.push_back(text.size());
    return starts;
}

}  // namespace

std::string fitEnd(std::string_view text, float width, const TextWidth& measure) {
    if (measure(text) <= width) {
        return std::string(text);
    }
    const std::vector<std::size_t> starts = characterStarts(text);
    // The most characters k (of starts.size() - 1) whose start and the ellipsis fit.
    std::size_t low = 0;
    std::size_t high = starts.size() - 1;
    while (low < high) {
        const std::size_t mid = (low + high + 1) / 2;
        std::string candidate(text.substr(0, starts[mid]));
        candidate += kEllipsis;
        if (measure(candidate) <= width) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }
    std::string result(text.substr(0, starts[low]));
    result += kEllipsis;
    return result;
}

std::string fitMiddle(std::string_view text, float width, const TextWidth& measure) {
    if (measure(text) <= width) {
        return std::string(text);
    }
    const std::vector<std::size_t> starts = characterStarts(text);
    const std::size_t count = starts.size() - 1;
    const auto build = [&](std::size_t head, std::size_t tail) {
        std::string candidate(text.substr(0, starts[head]));
        candidate += kEllipsis;
        candidate += text.substr(starts[count - tail]);
        return candidate;
    };
    // Grow the kept characters one at a time, alternating end and start (the end first: a
    // signal's last part names the quantity), while the result fits.
    std::size_t head = 0;
    std::size_t tail = 0;
    while (head + tail < count) {
        const bool growTail = tail <= head;
        const std::size_t nextHead = growTail ? head : head + 1;
        const std::size_t nextTail = growTail ? tail + 1 : tail;
        if (measure(build(nextHead, nextTail)) > width) {
            // The other side may still take one (a narrow character).
            const std::size_t otherHead = growTail ? head + 1 : head;
            const std::size_t otherTail = growTail ? tail : tail + 1;
            if (otherHead + otherTail < count && measure(build(otherHead, otherTail)) <= width) {
                head = otherHead;
                tail = otherTail;
                continue;
            }
            break;
        }
        head = nextHead;
        tail = nextTail;
    }
    return build(head, tail);
}

std::string fitPath(std::string_view text, float width, const TextWidth& measure) {
    if (measure(text) <= width) {
        return std::string(text);
    }
    const std::size_t slash = text.rfind('/');
    if (slash == std::string_view::npos || slash == 0) {
        return fitMiddle(text, width, measure);
    }
    const std::string_view last = text.substr(slash);  // with its '/'
    const float lastWidth = measure(last);
    // At least kMinimumHead characters of the start ("joint/fro"), or the start says nothing:
    // then the middle goes instead.
    const std::string head = fitEnd(text.substr(0, slash), width - lastWidth, measure);
    if (characterStarts(head).size() - 1 < kMinimumHead + 1) {
        return fitMiddle(text, width, measure);
    }
    std::string result = head;
    result += last;
    return result;
}

namespace {

/// The path's last part whole and its start cut in the middle: "joint/fro…wheel_joint/position".
std::string fitPathMiddle(std::string_view text, float width, const TextWidth& measure) {
    if (measure(text) <= width) {
        return std::string(text);
    }
    const std::size_t slash = text.rfind('/');
    if (slash == std::string_view::npos || slash == 0) {
        return fitMiddle(text, width, measure);
    }
    const std::string_view last = text.substr(slash);
    std::string result = fitMiddle(text.substr(0, slash), width - measure(last), measure);
    result += last;
    return measure(result) <= width ? result : fitMiddle(text, width, measure);
}

/// "…" and the longest end of `text` that fits.
std::string fitStart(std::string_view text, float width, const TextWidth& measure) {
    if (measure(text) <= width) {
        return std::string(text);
    }
    const std::vector<std::size_t> starts = characterStarts(text);
    // The fewest dropped characters k whose ellipsis and the rest fit.
    std::size_t low = 1;
    std::size_t high = starts.size() - 1;
    while (low < high) {
        const std::size_t mid = (low + high) / 2;
        std::string candidate(kEllipsis);
        candidate += text.substr(starts[mid]);
        if (measure(candidate) <= width) {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    std::string result(kEllipsis);
    result += text.substr(starts[low]);
    return result;
}

}  // namespace

std::string fitDistinct(std::string_view text, std::span<const std::string> group, float width,
                        const TextWidth& measure) {
    using Fit = std::string (*)(std::string_view, float, const TextWidth&);
    constexpr std::array<Fit, 5> kFits{&fitPath, &fitPathMiddle, &fitStart, &fitMiddle, &fitEnd};
    std::set<std::string_view> names(group.begin(), group.end());
    names.insert(text);
    for (const Fit fit : kFits) {
        std::set<std::string> seen;
        bool distinct = true;
        for (const std::string_view name : names) {
            distinct = seen.insert(fit(name, width, measure)).second;
            if (!distinct) {
                break;
            }
        }
        if (distinct) {
            return fit(text, width, measure);
        }
    }
    return fitPath(text, width, measure);
}

}  // namespace lagrange::viewer
