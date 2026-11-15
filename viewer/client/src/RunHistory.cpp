#include "lagrange/viewer/RunHistory.hpp"

#include <algorithm>
#include <sstream>
#include <string_view>

namespace lagrange::viewer {

namespace {

bool startsWith(std::string_view text, std::string_view prefix) {
    return text.substr(0, prefix.size()) == prefix;
}

/// "spawned NAME from PATH (why)" / "robot NAME from PATH (why)" after the first word: the
/// name, the file and what is in the brackets (without them).
bool nameFromPath(std::string_view rest, std::string& name, std::string& path, std::string& why) {
    const std::size_t from = rest.find(" from ");
    if (from == std::string_view::npos || from == 0) {
        return false;
    }
    name = std::string(rest.substr(0, from));
    std::string_view tail = rest.substr(from + 6);
    why.clear();
    if (const std::size_t open = tail.rfind(" (");
        open != std::string_view::npos && !tail.empty() && tail.back() == ')') {
        why = std::string(tail.substr(open + 2, tail.size() - open - 3));
        tail = tail.substr(0, open);
    }
    path = std::string(tail);
    return !path.empty();
}

std::string fileName(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

std::optional<HistoryItem> historyItemOf(const LogEntry& entry) {
    // The lines lagrange_server writes for these changes (server/src/Server.cpp): "edit" for
    // SetAttribute and the session layer, "stage" for spawn and delete, "profile", "site" and
    // the scheduler's reset.
    const std::string_view text = entry.text;
    HistoryItem item;
    item.simTimeNs = entry.simTimeNs;
    if (entry.source == "edit") {
        static constexpr std::string_view kSaved = "session layer saved to ";
        if (startsWith(text, kSaved)) {
            item.kind = HistoryItem::Kind::SessionSaved;
            item.target = std::string(text.substr(kSaved.size()));
            return item;
        }
        // "/amr/left_drive_motor elec:windingResistance 0.12 -> 0.15 ohm (from t = 3.413 s)"
        std::string_view body = text;
        if (const std::size_t from = body.rfind(" (from t = "); from != std::string_view::npos) {
            body = body.substr(0, from);
        }
        std::istringstream words{std::string(body)};
        std::string arrow;
        if (!(words >> item.target >> item.name >> item.before >> arrow >> item.after) ||
            arrow != "->") {
            return std::nullopt;
        }
        std::string unit;
        while (words >> unit) {
            item.unit += (item.unit.empty() ? "" : " ") + unit;
        }
        item.kind = HistoryItem::Kind::Attribute;
        return item;
    }
    if (entry.source == "stage") {
        static constexpr std::string_view kSpawned = "spawned ";
        static constexpr std::string_view kDeleted = "deleted ";
        if (startsWith(text, kSpawned) &&
            nameFromPath(text.substr(kSpawned.size()), item.target, item.detail, item.after)) {
            item.kind = HistoryItem::Kind::Spawn;
            return item;
        }
        if (startsWith(text, kDeleted) && text.size() > kDeleted.size()) {
            item.kind = HistoryItem::Kind::Delete;
            item.target = std::string(text.substr(kDeleted.size()));
            return item;
        }
        return std::nullopt;
    }
    if (entry.source == "profile" && startsWith(text, "applied")) {
        item.kind = HistoryItem::Kind::Profile;
        item.detail = std::string(text);
        return item;
    }
    if (entry.source == "scheduler" && startsWith(text, "reset")) {
        item.kind = HistoryItem::Kind::Reset;
        item.detail = std::string(text);
        return item;
    }
    if (entry.source == "site") {
        static constexpr std::string_view kTogether = "set together: ";
        if (startsWith(text, kTogether)) {
            item.kind = HistoryItem::Kind::Site;
            item.name = std::string(text.substr(kTogether.size()));
            return item;
        }
        // "gravity = 1.62"; not the notes about what a value rebuilt or blows on.
        const std::size_t equals = text.find(" = ");
        if (equals == std::string_view::npos || equals == 0 ||
            text.substr(0, equals).find(' ') != std::string_view::npos) {
            return std::nullopt;
        }
        item.kind = HistoryItem::Kind::Site;
        item.name = std::string(text.substr(0, equals));
        item.after = std::string(text.substr(equals + 3));
        return item;
    }
    return std::nullopt;
}

void RunHistory::take(const LogEntry& entry) {
    if (entry.source == "stage" && startsWith(entry.text, "robot ")) {
        // A robot added at load (--add-robot): a layer, not a change.
        RunRobot robot;
        std::string why;
        if (nameFromPath(std::string_view(entry.text).substr(6), robot.name, robot.path, why)) {
            robots_.push_back(std::move(robot));
            ++generation_;
        }
        return;
    }
    std::optional<HistoryItem> item = historyItemOf(entry);
    if (!item) {
        return;
    }
    switch (item->kind) {
        case HistoryItem::Kind::SessionSaved:
            sessionLayer_ = item->target;
            break;
        case HistoryItem::Kind::Spawn:
            robots_.push_back({item->target, item->detail, true});
            break;
        case HistoryItem::Kind::Delete:
            std::erase_if(robots_,
                          [&item](const RunRobot& robot) { return robot.name == item->target; });
            break;
        default:
            break;
    }
    items_.push_back(std::move(*item));
    ++generation_;
}

bool RunHistory::update(const LogBook& book, std::uint64_t connects) {
    const std::uint64_t before = generation_;
    if (connects != connects_ || book.total < seen_) {
        connects_ = connects;
        seen_ = 0;
        items_.clear();
        robots_.clear();
        sessionLayer_.clear();
        ++generation_;
    }
    if (book.total > seen_) {
        const std::uint64_t fresh = book.total - seen_;
        const std::size_t held = book.entries.size();
        const std::size_t first =
            held - static_cast<std::size_t>(std::min<std::uint64_t>(fresh, held));
        for (std::size_t i = first; i < held; ++i) {
            take(book.entries[i]);
        }
        seen_ = book.total;
    }
    return generation_ != before;
}

std::vector<LayerRow> layerRows(const std::string& stagePath, const std::string& environmentLayer,
                                const RunHistory& history) {
    std::vector<LayerRow> rows;
    LayerRow session;
    session.role = LayerRow::Role::Session;
    if (history.sessionLayer().empty()) {
        session.label = "session (not saved)";
        session.detail = "edits apply to the running model only";
    } else {
        session.path = history.sessionLayer();
        session.label = fileName(session.path);
        session.detail = "the edited model, saved by the server";
    }
    rows.push_back(std::move(session));
    for (auto robot = history.robots().rbegin(); robot != history.robots().rend(); ++robot) {
        rows.push_back({LayerRow::Role::Robot, fileName(robot->path), robot->path,
                        robot->name + (robot->spawned ? " · spawned" : " · added at load")});
    }
    if (!stagePath.empty()) {
        rows.push_back({LayerRow::Role::Stage, fileName(stagePath), stagePath, "root layer"});
    }
    if (!environmentLayer.empty()) {
        rows.push_back({LayerRow::Role::Environment, fileName(environmentLayer), environmentLayer,
                        "environment"});
    }
    return rows;
}

}  // namespace lagrange::viewer
