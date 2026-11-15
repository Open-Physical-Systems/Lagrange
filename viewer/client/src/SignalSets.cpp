// See SignalSets.hpp.
#include "lagrange/viewer/SignalSets.hpp"

#include <algorithm>
#include <exception>
#include <fstream>
#include <sstream>

namespace lagrange::viewer {

namespace {

std::string clean(std::string text) {
    std::ranges::replace_if(text, [](char c) { return c == '\t' || c == '\n' || c == '\r'; }, ' ');
    return text;
}

}  // namespace

std::vector<SignalSet> loadSignalSets(const std::string& path) {
    std::vector<SignalSet> sets;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
        std::vector<std::string> fields;
        std::istringstream row(line);
        for (std::string field; std::getline(row, field, '\t');) {
            fields.push_back(field);
        }
        if (fields.size() != 3 || fields[0].empty() || fields[1].empty()) {
            continue;
        }
        std::uint32_t component = 0;
        try {
            const unsigned long value = std::stoul(fields[2]);
            component = static_cast<std::uint32_t>(value);
        } catch (const std::exception&) {
            continue;  // a damaged line is left out
        }
        auto it = std::ranges::find(sets, fields[0], &SignalSet::name);
        if (it == sets.end()) {
            sets.push_back({fields[0], {}});
            it = sets.end() - 1;
        }
        it->signals.emplace_back(fields[1], component);
    }
    return sets;
}

bool saveSignalSets(const std::string& path, const std::vector<SignalSet>& sets) {
    std::ofstream out(path);
    for (const SignalSet& set : sets) {
        const std::string name = clean(set.name);
        for (const auto& [signal, component] : set.signals) {
            out << name << '\t' << clean(signal) << '\t' << component << '\n';
        }
    }
    out.close();
    return static_cast<bool>(out);
}

std::size_t storeSignalSet(std::vector<SignalSet>& sets, SignalSet set) {
    const auto it = std::ranges::find(sets, set.name, &SignalSet::name);
    if (it != sets.end()) {
        *it = std::move(set);
        return static_cast<std::size_t>(it - sets.begin());
    }
    sets.push_back(std::move(set));
    return sets.size() - 1;
}

}  // namespace lagrange::viewer
