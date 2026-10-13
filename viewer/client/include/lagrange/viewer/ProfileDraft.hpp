#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// One `key: value` of a domain's settings, with the value as profile text.
struct ProfileEntry {
    std::string key;
    std::string value;
};

[[nodiscard]] const std::string* findEntry(const std::vector<ProfileEntry>& entries,
                                           std::string_view key);
/// Replaces the value of `key`, or appends it.
void setEntry(std::vector<ProfileEntry>& entries, std::string_view key, std::string value);
void eraseEntry(std::vector<ProfileEntry>& entries, std::string_view key);

/// A step size as profile text: whole nanoseconds as decimal seconds, exactly ("0.0001").
[[nodiscard]] std::string formatSeconds(std::int64_t nanoseconds);
/// The reverse. Empty when the text is not a positive decimal number of whole nanoseconds.
[[nodiscard]] std::optional<std::int64_t> parseSeconds(std::string_view text);

/// The profile being edited in the viewer, next to the one the server runs.
///
/// The server sends profiles in one layout: under `domains:`, one line per domain holding a
/// flow mapping. An edit replaces (or adds) exactly one such line, so everything else in the
/// file, comments included, is kept as it is. Whether the result is a valid profile is for
/// the server to say; this class only edits text.
class ProfileDraft {
public:
    /// Starts again from the profile the server runs. Drops every edit.
    void reset(std::string_view appliedYaml);
    /// Drops every edit.
    void revert();

    [[nodiscard]] const std::vector<std::string>& lines() const { return lines_; }
    [[nodiscard]] const std::vector<std::string>& appliedLines() const { return applied_; }
    [[nodiscard]] std::string text() const;

    /// Increases with every change to the text, including reset() and revert().
    [[nodiscard]] std::uint64_t revision() const { return revision_; }
    [[nodiscard]] bool dirty() const { return changedLineCount() != 0; }
    [[nodiscard]] std::size_t changedLineCount() const;
    /// True when draft line `index` is new or differs from the applied profile.
    [[nodiscard]] bool lineChanged(std::size_t index) const;
    /// The applied line that draft line `index` replaces, or nullptr for an added line.
    [[nodiscard]] const std::string* appliedLine(std::size_t index) const;

    /// The settings of a domain in the draft / in the applied profile. Empty when the
    /// profile has no line for it, or the line is not in the expected layout.
    [[nodiscard]] std::optional<std::vector<ProfileEntry>> domain(std::string_view name) const;
    [[nodiscard]] std::optional<std::vector<ProfileEntry>> appliedDomain(
        std::string_view name) const;
    [[nodiscard]] bool domainChanged(std::string_view name) const;

    /// Writes a domain's settings. Returns false when the profile has no `domains:` block.
    bool setDomain(std::string_view name, const std::vector<ProfileEntry>& entries);

private:
    static constexpr std::size_t kAdded = static_cast<std::size_t>(-1);

    std::vector<std::string> applied_;
    std::vector<std::string> lines_;
    std::vector<std::size_t> origin_;  // per draft line: index into applied_, or kAdded
    std::uint64_t revision_{0};
};

}  // namespace lagrange::viewer
