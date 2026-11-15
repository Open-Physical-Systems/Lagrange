#pragma once

// Fuzzy matching for the command palette (ui-conventions.md §3, Ctrl K: prims, signals, topics,
// commands): the query's characters in order, anywhere in the text, case ignored.

#include <cstddef>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// How well `text` matches `query`: below zero when it does not (some character of the query is
/// not found in order), higher for a better match: characters one after another, at the start of
/// a word ("left_drive" for "ld"), a whole substring, and shorter texts. An empty query matches
/// everything with 0.
[[nodiscard]] int fuzzyScore(std::string_view text, std::string_view query);

/// The indices of `texts` that match `query`, best first (ties in their order), at most `most`.
[[nodiscard]] std::vector<std::size_t> fuzzyRank(const std::vector<std::string_view>& texts,
                                                 std::string_view query, std::size_t most);

}  // namespace lagrange::viewer
