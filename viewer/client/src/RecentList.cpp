#include "lagrange/viewer/RecentList.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>
#include <string_view>
#include <utility>

namespace lagrange::viewer {

namespace {

struct Field {
    std::string_view key;
    std::string RecentEntry::* member;
};

constexpr std::array<Field, 5> kFields{{
    {"stage", &RecentEntry::stage},
    {"robot", &RecentEntry::robot},
    {"profile", &RecentEntry::profile},
    {"endpoint", &RecentEntry::endpoint},
    {"opened", &RecentEntry::opened},
}};

void writeString(std::ostream& out, std::string_view text) {
    out << '"';
    for (const char ch : text) {
        switch (ch) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(ch) >= 0x20) {
                    out << ch;  // UTF-8 passes through; other control characters are dropped
                }
        }
    }
    out << '"';
}

/// Reads a JSON string starting at the opening quote `text[at]`. Advances `at` past it.
bool readString(std::string_view text, std::size_t& at, std::string& out) {
    if (at >= text.size() || text[at] != '"') {
        return false;
    }
    out.clear();
    for (++at; at < text.size(); ++at) {
        const char ch = text[at];
        if (ch == '"') {
            ++at;
            return true;
        }
        if (ch == '\\' && at + 1 < text.size()) {
            const char escaped = text[++at];
            out += escaped == 'n' ? '\n' : (escaped == 't' ? '\t' : escaped);
        } else {
            out += ch;
        }
    }
    return false;  // no closing quote
}

void skipSpace(std::string_view text, std::size_t& at) {
    while (at < text.size() &&
           (text[at] == ' ' || text[at] == '\n' || text[at] == '\r' || text[at] == '\t')) {
        ++at;
    }
}

}  // namespace

void RecentList::add(RecentEntry entry) {
    std::erase_if(entries_, [&](const RecentEntry& other) {
        return other.stage == entry.stage && other.endpoint == entry.endpoint;
    });
    entries_.insert(entries_.begin(), std::move(entry));
    if (entries_.size() > kCapacity) {
        entries_.resize(kCapacity);
    }
}

void RecentList::write(std::ostream& out) const {
    out << "[\n";
    for (std::size_t i = 0; i < entries_.size(); ++i) {
        out << "  {";
        for (std::size_t f = 0; f < kFields.size(); ++f) {
            out << (f == 0 ? "" : ", ");
            writeString(out, kFields[f].key);
            out << ": ";
            writeString(out, entries_[i].*kFields[f].member);
        }
        out << (i + 1 == entries_.size() ? "}\n" : "},\n");
    }
    out << "]\n";
}

void RecentList::read(std::istream& in) {
    entries_.clear();
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::size_t at = 0;
    while (at < text.size() && entries_.size() < kCapacity) {
        at = text.find('{', at);
        if (at == std::string::npos) {
            break;
        }
        ++at;
        RecentEntry entry;
        bool closed = false;
        while (at < text.size()) {
            skipSpace(text, at);
            if (at < text.size() && text[at] == '}') {
                ++at;
                closed = true;
                break;
            }
            std::string key;
            std::string value;
            if (!readString(text, at, key)) {
                break;
            }
            skipSpace(text, at);
            if (at >= text.size() || text[at] != ':') {
                break;
            }
            ++at;
            skipSpace(text, at);
            if (!readString(text, at, value)) {
                break;  // only string members are written; anything else ends this object
            }
            for (const Field& field : kFields) {
                if (field.key == key) {
                    entry.*field.member = std::move(value);
                    break;
                }
            }
            skipSpace(text, at);
            if (at < text.size() && text[at] == ',') {
                ++at;
            }
        }
        if (closed && !entry.stage.empty()) {
            entries_.push_back(std::move(entry));
        }
    }
}

bool RecentList::save(const std::string& path) const {
    std::ofstream file(path, std::ios::trunc);
    if (!file) {
        return false;
    }
    write(file);
    return static_cast<bool>(file);
}

bool RecentList::load(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        entries_.clear();
        return false;
    }
    read(file);
    return true;
}

}  // namespace lagrange::viewer
