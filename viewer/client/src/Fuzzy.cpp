#include "lagrange/viewer/Fuzzy.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace lagrange::viewer {

namespace {

char lower(char ch) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
}

bool wordStart(std::string_view text, std::size_t i) {
    if (i == 0) {
        return true;
    }
    const char before = text[i - 1];
    return before == '/' || before == '_' || before == ' ' || before == '.' || before == '-' ||
           before == ':' ||
           (std::islower(static_cast<unsigned char>(before)) != 0 &&
            std::isupper(static_cast<unsigned char>(text[i])) != 0);
}

// Score weights: what makes one match better than another.
constexpr int kMatched = 1;            // each character found
constexpr int kConsecutive = 5;        // right after the previous one
constexpr int kAtWordStart = 8;        // at the start of a word
constexpr int kSubstring = 40;         // the query as one piece
constexpr int kSubstringAtStart = 20;  // ... at the very start
constexpr int kGapCost = 1;            // per character skipped, up to kMaxGapCost
constexpr int kMaxGapCost = 20;
constexpr int kLastPart = 4;  // matched by the last part of a path alone

int scoreOf(std::string_view text, std::string_view query) {
    int score = 0;
    std::size_t at = 0;
    std::size_t previous = std::string_view::npos;
    int gaps = 0;
    for (const char wanted : query) {
        const char q = lower(wanted);
        // The first place it is found after the previous character.
        std::size_t found = std::string_view::npos;
        for (std::size_t i = at; i < text.size(); ++i) {
            if (lower(text[i]) == q) {
                found = i;
                break;
            }
        }
        if (found == std::string_view::npos) {
            return -1;
        }
        score += kMatched;
        if (previous != std::string_view::npos && found == previous + 1) {
            score += kConsecutive;
        } else if (previous != std::string_view::npos) {
            gaps += static_cast<int>(found - previous - 1);
        }
        if (wordStart(text, found)) {
            score += kAtWordStart;
        }
        previous = found;
        at = found + 1;
    }
    score -= std::min(gaps * kGapCost, kMaxGapCost);
    // A substring match, case ignored.
    const auto hit = std::search(text.begin(), text.end(), query.begin(), query.end(),
                                 [](char a, char b) { return lower(a) == lower(b); });
    if (hit != text.end()) {
        score += kSubstring;
        if (hit == text.begin()) {
            score += kSubstringAtStart;
        }
    }
    // Shorter texts first among equals: up to 10 points for brevity.
    score += std::max(0, 10 - static_cast<int>(text.size() / 8));
    return std::max(score, 0);
}

}  // namespace

int fuzzyScore(std::string_view text, std::string_view query) {
    if (query.empty()) {
        return 0;
    }
    // A path is also matched by its last part alone ("left_drive_motor" of
    // "/amr/base_link/left_drive_motor"), which is what a person names it by.
    int score = scoreOf(text, query);
    const std::size_t slash = text.rfind('/');
    if (slash != std::string_view::npos && slash + 1 < text.size()) {
        const int last = scoreOf(text.substr(slash + 1), query);
        if (last >= 0) {
            score = std::max(score, last + kLastPart);
        }
    }
    return score;
}

std::vector<std::size_t> fuzzyRank(const std::vector<std::string_view>& texts,
                                   std::string_view query, std::size_t most) {
    std::vector<std::pair<int, std::size_t>> scored;
    for (std::size_t i = 0; i < texts.size(); ++i) {
        const int score = fuzzyScore(texts[i], query);
        if (score >= 0) {
            scored.emplace_back(score, i);
        }
    }
    std::ranges::stable_sort(scored,
                             [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<std::size_t> result;
    for (std::size_t i = 0; i < scored.size() && i < most; ++i) {
        result.push_back(scored[i].second);
    }
    return result;
}

}  // namespace lagrange::viewer
