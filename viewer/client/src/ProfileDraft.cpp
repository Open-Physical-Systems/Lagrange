#include "lagrange/viewer/ProfileDraft.hpp"

#include <algorithm>
#include <charconv>

namespace lagrange::viewer {

namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
constexpr std::size_t kNone = static_cast<std::size_t>(-1);

std::string_view trim(std::string_view text) {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) {
        text.remove_prefix(1);
    }
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\r')) {
        text.remove_suffix(1);
    }
    return text;
}

std::vector<std::string> splitLines(std::string_view text) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = std::min(text.find('\n', start), text.size());
        lines.emplace_back(text.substr(start, end - start));
        start = end + 1;
    }
    return lines;
}

/// The position of `separator` outside brackets and quotes, from `from` on.
std::size_t findTopLevel(std::string_view text, char separator, std::size_t from) {
    int depth = 0;
    char quote = '\0';
    for (std::size_t i = from; i < text.size(); ++i) {
        const char c = text[i];
        if (quote != '\0') {
            if (c == quote) {
                quote = '\0';
            }
        } else if (c == '"' || c == '\'') {
            quote = c;
        } else if (c == '[' || c == '{') {
            ++depth;
        } else if (c == ']' || c == '}') {
            --depth;
        } else if (c == separator && depth == 0) {
            return i;
        }
    }
    return kNone;
}

/// Where the lines of the `domains:` block are: [first, last).
struct Block {
    std::size_t first{kNone};
    std::size_t last{kNone};
};

Block domainsBlock(const std::vector<std::string>& lines) {
    Block block;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (block.first == kNone) {
            if (trim(lines[i]) == "domains:" && !lines[i].empty() && lines[i].front() == 'd') {
                block.first = i + 1;
                block.last = i + 1;
            }
            continue;
        }
        const std::string& line = lines[i];
        if (!line.empty() && line.front() != ' ' && line.front() != '\t' && line.front() != '#') {
            break;  // the next top-level key
        }
        if (!trim(line).empty()) {
            block.last = i + 1;  // blank lines after the last entry are not part of the block
        }
    }
    return block;
}

/// If `line` is "  <name>: {...}", the text between the braces.
std::optional<std::string_view> domainBody(std::string_view line, std::string_view name) {
    std::string_view rest = trim(line);
    if (rest.size() <= name.size() || rest.substr(0, name.size()) != name ||
        rest[name.size()] != ':') {
        return std::nullopt;
    }
    rest = trim(rest.substr(name.size() + 1));
    if (rest.empty() || rest.front() != '{') {
        return std::nullopt;
    }
    // A comment may follow the mapping; the mapping ends at its matching brace.
    int depth = 0;
    char quote = '\0';
    for (std::size_t i = 0; i < rest.size(); ++i) {
        const char c = rest[i];
        if (quote != '\0') {
            if (c == quote) {
                quote = '\0';
            }
        } else if (c == '"' || c == '\'') {
            quote = c;
        } else if (c == '{' || c == '[') {
            ++depth;
        } else if (c == '}' || c == ']') {
            if (--depth == 0) {
                return rest.substr(1, i - 1);
            }
        }
    }
    return std::nullopt;
}

std::size_t findDomainLine(const std::vector<std::string>& lines, std::string_view name) {
    const Block block = domainsBlock(lines);
    if (block.first == kNone) {
        return kNone;
    }
    for (std::size_t i = block.first; i < block.last; ++i) {
        if (domainBody(lines[i], name)) {
            return i;
        }
    }
    return kNone;
}

std::optional<std::vector<ProfileEntry>> readDomain(const std::vector<std::string>& lines,
                                                    std::string_view name) {
    const std::size_t index = findDomainLine(lines, name);
    if (index == kNone) {
        return std::nullopt;
    }
    const std::string_view body = *domainBody(lines[index], name);
    std::vector<ProfileEntry> entries;
    std::size_t start = 0;
    while (start < body.size()) {
        const std::size_t comma = std::min(findTopLevel(body, ',', start), body.size());
        const std::string_view item = body.substr(start, comma - start);
        const std::size_t colon = findTopLevel(item, ':', 0);
        if (colon != kNone) {
            entries.push_back({std::string(trim(item.substr(0, colon))),
                               std::string(trim(item.substr(colon + 1)))});
        } else if (!trim(item).empty()) {
            return std::nullopt;  // not a mapping this editor understands
        }
        start = comma + 1;
    }
    return entries;
}

std::string writeDomain(std::string_view name, const std::vector<ProfileEntry>& entries) {
    std::string line = "  ";
    line += name;
    line += ": {";
    for (std::size_t i = 0; i < entries.size(); ++i) {
        if (i != 0) {
            line += ", ";
        }
        line += entries[i].key;
        line += ": ";
        line += entries[i].value;
    }
    line += "}";
    return line;
}

}  // namespace

const std::string* findEntry(const std::vector<ProfileEntry>& entries, std::string_view key) {
    for (const ProfileEntry& entry : entries) {
        if (entry.key == key) {
            return &entry.value;
        }
    }
    return nullptr;
}

void setEntry(std::vector<ProfileEntry>& entries, std::string_view key, std::string value) {
    for (ProfileEntry& entry : entries) {
        if (entry.key == key) {
            entry.value = std::move(value);
            return;
        }
    }
    entries.push_back({std::string(key), std::move(value)});
}

void eraseEntry(std::vector<ProfileEntry>& entries, std::string_view key) {
    std::erase_if(entries, [key](const ProfileEntry& entry) { return entry.key == key; });
}

std::string formatSeconds(std::int64_t nanoseconds) {
    if (nanoseconds <= 0) {
        return "0";
    }
    std::string text = std::to_string(nanoseconds / kNanosecondsPerSecond);
    std::int64_t fraction = nanoseconds % kNanosecondsPerSecond;
    if (fraction == 0) {
        return text;
    }
    std::string digits = std::to_string(fraction);
    digits.insert(0, 9 - digits.size(), '0');
    while (digits.back() == '0') {
        digits.pop_back();
    }
    return text + "." + digits;
}

std::optional<std::int64_t> parseSeconds(std::string_view text) {
    text = trim(text);
    const std::size_t dot = std::min(text.find('.'), text.size());
    const std::string_view whole = text.substr(0, dot);
    std::string_view fraction = dot < text.size() ? text.substr(dot + 1) : std::string_view{};
    const auto digits = [](std::string_view part) {
        return std::ranges::all_of(part, [](char c) { return c >= '0' && c <= '9'; });
    };
    if ((whole.empty() && fraction.empty()) || whole.size() > 9 || fraction.size() > 9 ||
        !digits(whole) || !digits(fraction)) {
        return std::nullopt;
    }
    std::int64_t seconds = 0;
    if (!whole.empty()) {
        const auto [end, error] =
            std::from_chars(whole.data(), whole.data() + whole.size(), seconds);
        if (error != std::errc{} || end != whole.data() + whole.size()) {
            return std::nullopt;
        }
    }
    std::int64_t nanoseconds = 0;
    if (!fraction.empty()) {
        const auto [end, error] =
            std::from_chars(fraction.data(), fraction.data() + fraction.size(), nanoseconds);
        if (error != std::errc{} || end != fraction.data() + fraction.size()) {
            return std::nullopt;
        }
        for (std::size_t i = fraction.size(); i < 9; ++i) {
            nanoseconds *= 10;
        }
    }
    const std::int64_t total = seconds * kNanosecondsPerSecond + nanoseconds;
    if (total <= 0) {
        return std::nullopt;
    }
    return total;
}

void ProfileDraft::reset(std::string_view appliedYaml) {
    applied_ = splitLines(appliedYaml);
    revert();
}

void ProfileDraft::revert() {
    lines_ = applied_;
    origin_.resize(lines_.size());
    for (std::size_t i = 0; i < origin_.size(); ++i) {
        origin_[i] = i;
    }
    ++revision_;
}

std::string ProfileDraft::text() const {
    std::string text;
    for (const std::string& line : lines_) {
        text += line;
        text += '\n';
    }
    return text;
}

bool ProfileDraft::lineChanged(std::size_t index) const {
    return index < lines_.size() &&
           (origin_[index] == kAdded || lines_[index] != applied_[origin_[index]]);
}

const std::string* ProfileDraft::appliedLine(std::size_t index) const {
    if (index >= lines_.size() || origin_[index] == kAdded) {
        return nullptr;
    }
    return &applied_[origin_[index]];
}

std::size_t ProfileDraft::changedLineCount() const {
    std::size_t count = 0;
    for (std::size_t i = 0; i < lines_.size(); ++i) {
        count += lineChanged(i) ? 1U : 0U;
    }
    return count;
}

std::optional<std::vector<ProfileEntry>> ProfileDraft::domain(std::string_view name) const {
    return readDomain(lines_, name);
}

std::optional<std::vector<ProfileEntry>> ProfileDraft::appliedDomain(std::string_view name) const {
    return readDomain(applied_, name);
}

bool ProfileDraft::domainChanged(std::string_view name) const {
    const std::size_t index = findDomainLine(lines_, name);
    return index != kNone && lineChanged(index);
}

bool ProfileDraft::setDomain(std::string_view name, const std::vector<ProfileEntry>& entries) {
    std::string line = writeDomain(name, entries);
    const std::size_t index = findDomainLine(lines_, name);
    if (index != kNone) {
        if (lines_[index] != line) {
            lines_[index] = std::move(line);
            ++revision_;
        }
        return true;
    }
    const Block block = domainsBlock(lines_);
    if (block.first == kNone) {
        return false;
    }
    const auto offset = static_cast<std::ptrdiff_t>(block.last);
    lines_.insert(lines_.begin() + offset, std::move(line));
    origin_.insert(origin_.begin() + offset, kAdded);
    ++revision_;
    return true;
}

}  // namespace lagrange::viewer
