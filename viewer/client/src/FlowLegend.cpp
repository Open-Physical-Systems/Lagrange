#include "lagrange/viewer/FlowLegend.hpp"

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <map>
#include <optional>
#include <string_view>

namespace lagrange::viewer {

namespace {

struct Band {
    int index{0};
    double from{0.0}, to{0.0};
    std::string unit;
};

/// "P11p40" or "M0p25".
std::optional<double> numberOf(std::string_view text) {
    if (text.size() < 2 || (text[0] != 'P' && text[0] != 'M')) {
        return std::nullopt;
    }
    std::string plain(text.substr(1));
    std::ranges::replace(plain, 'p', '.');
    double value = 0.0;
    const auto [end, error] = std::from_chars(plain.data(), plain.data() + plain.size(), value);
    if (error != std::errc{} || end != plain.data() + plain.size()) {
        return std::nullopt;
    }
    return text[0] == 'M' ? -value : value;
}

std::optional<Band> bandOf(std::string_view name) {
    Band band;
    const std::size_t a = name.find('_');
    const std::size_t to = name.find("_to_");
    if (name.size() < 4 || name[0] != 'b' || a == std::string_view::npos ||
        to == std::string_view::npos || to <= a) {
        return std::nullopt;
    }
    const std::size_t last = name.rfind('_');
    if (last <= to + 3) {
        return std::nullopt;
    }
    const auto [end, error] = std::from_chars(name.data() + 1, name.data() + a, band.index);
    const auto from = numberOf(name.substr(a + 1, to - a - 1));
    const auto until = numberOf(name.substr(to + 4, last - to - 4));
    if (error != std::errc{} || end != name.data() + a || !from || !until) {
        return std::nullopt;
    }
    band.from = *from;
    band.to = *until;
    band.unit = std::string(name.substr(last + 1));
    return band;
}

}  // namespace

std::vector<FlowLegend> flowLegends(const SceneStatics& statics) {
    struct Seen {
        std::map<int, std::array<float, 4>> colours;
        double low{1e300}, high{-1e300};
    };
    std::map<std::string, Seen> byTitle;
    for (const PrimData& prim : statics.prims) {
        if (prim.material == kNone || prim.parent == kNone) {
            continue;
        }
        const auto band = bandOf(prim.name);
        if (!band) {
            continue;
        }
        const std::string title = band->unit == "mps"  ? "speed · m/s"
                                  : band->unit == "cp" ? "pressure coefficient"
                                                       : band->unit;
        Seen& seen = byTitle[title];
        seen.colours[band->index] = statics.materials[prim.material].baseColor;
        seen.low = std::min(seen.low, band->from);
        seen.high = std::max(seen.high, band->to);
    }
    std::vector<FlowLegend> out;
    for (const auto& [title, seen] : byTitle) {
        FlowLegend legend;
        legend.title = title;
        legend.low = seen.low;
        legend.high = seen.high;
        for (const auto& [index, colour] : seen.colours) {
            legend.colours.push_back(colour);
        }
        out.push_back(std::move(legend));
    }
    return out;
}

}  // namespace lagrange::viewer
