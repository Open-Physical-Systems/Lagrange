#pragma once

#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

namespace lagrange::viewer {

/// One line of the launcher's "Recent" table.
struct RecentEntry {
    std::string stage;     ///< Stage or recording that was shown.
    std::string robot;     ///< Its first root prim.
    std::string profile;   ///< "applied", or how the profile was named.
    std::string endpoint;  ///< Server it came from, as the viewer connects to it.
    std::string opened;    ///< Local date and time, "2026-10-14 23:12".

    bool operator==(const RecentEntry&) const = default;
};

/// What the viewer opened lately, newest first (~/.config/lagrange/recent.json).
class RecentList {
public:
    static constexpr std::size_t kCapacity = 12;

    [[nodiscard]] const std::vector<RecentEntry>& entries() const { return entries_; }
    /// Puts `entry` first. An earlier entry for the same stage and server is replaced.
    void add(RecentEntry entry);

    /// A JSON array of objects with string members. read() accepts what write() produces and
    /// skips anything else, so a damaged file gives an empty or shorter list, never an error.
    void write(std::ostream& out) const;
    void read(std::istream& in);

    /// Convenience over write() and read(). False if the file could not be opened.
    bool save(const std::string& path) const;
    bool load(const std::string& path);

private:
    std::vector<RecentEntry> entries_;
};

}  // namespace lagrange::viewer
