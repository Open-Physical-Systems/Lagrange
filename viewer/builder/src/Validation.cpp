// Stub (RFC 0016 WP0): WP-A replaces validate() with the rules of RFC 0016 §8.1. validName is
// the server's rule and complete.
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>

namespace lagrange::viewer::builder {

std::vector<Problem> validate(const Document& /*document*/, const SensorCatalog* /*catalog*/,
                              const std::filesystem::path& /*meshBase*/) {
    // A hint, not an error: an unchecked robot may still be tested, the server's loader judges.
    return {Problem{Level::Hint, "",
                    "the builder's own checks are not implemented yet (RFC 0016 WP-A); the "
                    "server's loader still checks the robot on Test"}};
}

bool validName(std::string_view name) {
    const auto letter = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    };
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    return !name.empty() && letter(name.front()) &&
           std::ranges::all_of(name, [&](char c) { return letter(c) || digit(c); });
}

std::string_view toString(Level level) {
    switch (level) {
        case Level::Error:
            return "error";
        case Level::Warning:
            return "warning";
        case Level::Hint:
            return "hint";
    }
    return "error";
}

}  // namespace lagrange::viewer::builder
