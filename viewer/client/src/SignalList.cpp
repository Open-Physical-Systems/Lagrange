#include "lagrange/viewer/SignalList.hpp"

#include <algorithm>
#include <cctype>

namespace lagrange::viewer {

namespace {

bool containsNoCase(std::string_view text, std::string_view needle) {
    const auto it =
        std::search(text.begin(), text.end(), needle.begin(), needle.end(), [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a)) ==
                   std::tolower(static_cast<unsigned char>(b));
        });
    return it != text.end();
}

bool numbered(std::string_view name) {
    return !name.empty() && std::ranges::all_of(name, [](char ch) {
        return std::isdigit(static_cast<unsigned char>(ch)) != 0;
    });
}

}  // namespace

std::string signalDomain(const SignalInfo& info) {
    return info.writer.substr(0, info.writer.find('/'));
}

std::string componentLabel(const SignalInfo& info, std::uint32_t component) {
    const std::string name =
        component < info.componentNames.size() ? info.componentNames[component] : std::string();
    if (name.empty() || numbered(name)) {
        return "[" + std::to_string(component) + "]";
    }
    return name;
}

std::string componentPath(const SignalInfo& info, std::uint32_t component) {
    if (info.components <= 1) {
        return info.name;
    }
    const std::string name = component < info.componentNames.size() ? info.componentNames[component]
                                                                    : std::to_string(component);
    return info.name + "." + name;
}

std::vector<SignalRow> signalRows(const SignalCatalog& catalog, std::string_view domain,
                                  const SignalFilter& filter,
                                  const std::set<std::string, std::less<>>& open,
                                  std::size_t* listed) {
    std::vector<SignalRow> rows;
    const std::string_view search = filter.search;
    for (std::uint32_t s = 0; s < catalog.signals.size(); ++s) {
        const SignalInfo& info = catalog.signals[s];
        if (info.components == 0 || signalDomain(info) != domain ||
            (filter.fastOnly && info.dtNs > filter.fastDtNs) ||
            (!filter.unit.empty() && info.unit != filter.unit)) {
            continue;
        }
        const bool named = search.empty() || containsNoCase(info.name, search) ||
                           containsNoCase(info.unit, search);
        if (info.components == 1) {
            if (named) {
                rows.push_back({s, 0, false});
                if (listed != nullptr) {
                    ++*listed;
                }
            }
            continue;
        }
        if (named) {
            const bool expanded = open.contains(info.name);
            rows.push_back({s, SignalRow::kWhole, expanded});
            if (expanded) {
                for (std::uint32_t k = 0; k < info.components; ++k) {
                    rows.push_back({s, k, false});
                }
            }
        } else {
            // Only the components the search names: "ranges.10", or "[10]" as the row shows it.
            const std::size_t header = rows.size();
            for (std::uint32_t k = 0; k < info.components; ++k) {
                if (containsNoCase(componentPath(info, k), search) ||
                    containsNoCase(info.name + componentLabel(info, k), search)) {
                    if (rows.size() == header) {
                        rows.push_back({s, SignalRow::kWhole, true});
                    }
                    rows.push_back({s, k, false});
                }
            }
            if (rows.size() == header) {
                continue;
            }
        }
        if (listed != nullptr) {
            ++*listed;
        }
    }
    return rows;
}

}  // namespace lagrange::viewer
