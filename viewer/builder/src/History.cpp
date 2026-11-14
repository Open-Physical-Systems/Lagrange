// Undo and redo by snapshots of the whole document (RFC 0016 §9): one state per committed edit,
// a label each, edits to the same field within one second merged, the oldest dropped beyond the
// depth.
#include "lagrange/viewer/builder/History.hpp"

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace lagrange::viewer::builder {

namespace {

/// Edits to the same field closer together than this merge into one step (RFC 0016 §9), s.
constexpr double kMergeWindowS = 1.0;

}  // namespace

struct History::State {
    std::vector<Document> documents;  // oldest first
    std::vector<std::string> labels;  // one per document
    std::vector<std::string> keys;    // merge key per document ("" never merges)
    std::vector<double> times;        // ImGui time of the last commit into each state, s
    std::size_t position{0};
    std::size_t depth{256};
    std::optional<std::size_t> saved;  // the state on disk, while it is kept
    bool mergeable{false};             // the current state was the last commit's
    std::uint64_t revision{0};
};

History::History(Document initial, std::size_t depth)
    : History(std::move(initial), "Start", depth) {}

History::History(Document initial, std::string label, std::size_t depth)
    : state_(std::make_unique<State>()) {
    state_->documents.push_back(std::move(initial));
    state_->labels.push_back(std::move(label));
    state_->keys.emplace_back();
    state_->times.push_back(0.0);
    state_->depth = std::max<std::size_t>(depth, 1);
}

History::~History() = default;
History::History(History&&) noexcept = default;
History& History::operator=(History&&) noexcept = default;

void History::commit(Document next, std::string label, std::string mergeKey, double nowS) {
    State& s = *state_;
    // Redo is cleared: everything after the current state goes.
    const std::size_t keep = s.position + 1;
    s.documents.resize(keep);
    s.labels.resize(keep);
    s.keys.resize(keep);
    s.times.resize(keep);
    if (s.saved && *s.saved >= keep) {
        s.saved.reset();
    }
    const bool merge = s.mergeable && !mergeKey.empty() && s.position > 0 &&
                       s.keys[s.position] == mergeKey &&
                       nowS - s.times[s.position] < kMergeWindowS && s.saved != s.position;
    if (merge) {
        s.documents[s.position] = std::move(next);
        s.labels[s.position] = std::move(label);
        s.times[s.position] = nowS;
    } else {
        s.documents.push_back(std::move(next));
        s.labels.push_back(std::move(label));
        s.keys.push_back(std::move(mergeKey));
        s.times.push_back(nowS);
        s.position = s.documents.size() - 1;
        if (s.documents.size() > s.depth) {
            const auto drop = static_cast<std::ptrdiff_t>(s.documents.size() - s.depth);
            s.documents.erase(s.documents.begin(), s.documents.begin() + drop);
            s.labels.erase(s.labels.begin(), s.labels.begin() + drop);
            s.keys.erase(s.keys.begin(), s.keys.begin() + drop);
            s.times.erase(s.times.begin(), s.times.begin() + drop);
            s.position -= static_cast<std::size_t>(drop);
            if (s.saved) {
                if (*s.saved < static_cast<std::size_t>(drop)) {
                    s.saved.reset();
                } else {
                    *s.saved -= static_cast<std::size_t>(drop);
                }
            }
        }
    }
    s.mergeable = true;
    ++s.revision;
}

bool History::undo() {
    if (!canUndo()) {
        return false;
    }
    return goTo(state_->position - 1);
}

bool History::redo() {
    if (!canRedo()) {
        return false;
    }
    return goTo(state_->position + 1);
}

bool History::goTo(std::size_t step) {
    State& s = *state_;
    if (step >= s.documents.size()) {
        return false;
    }
    if (step != s.position) {
        s.position = step;
        ++s.revision;
    }
    s.mergeable = false;  // an edit after undo or a jump is a step of its own
    return true;
}

const Document& History::current() const {
    return state_->documents[state_->position];
}

std::span<const std::string> History::labels() const {
    return state_->labels;
}

std::size_t History::position() const {
    return state_->position;
}

bool History::canUndo() const {
    return state_->position > 0;
}

bool History::canRedo() const {
    return state_->position + 1 < state_->documents.size();
}

std::uint64_t History::revision() const {
    return state_->revision;
}

History::Checkpoint::Checkpoint() = default;
History::Checkpoint::~Checkpoint() = default;
History::Checkpoint::Checkpoint(Checkpoint&&) noexcept = default;
History::Checkpoint& History::Checkpoint::operator=(Checkpoint&&) noexcept = default;

History::Checkpoint History::checkpoint() const {
    Checkpoint out;
    out.state_ = std::make_unique<State>(*state_);
    return out;
}

void History::restore(const Checkpoint& checkpoint) {
    if (!checkpoint.state_) {
        return;
    }
    const std::uint64_t revision = state_->revision + 1;
    *state_ = *checkpoint.state_;
    state_->revision = revision;  // current() may differ from what was last seen
    state_->mergeable = false;
}

void History::markSaved() {
    state_->saved = state_->position;
    state_->mergeable = false;  // the saved state stays as it was written
}

int History::editsSinceSave() const {
    const State& s = *state_;
    if (!s.saved) {
        return static_cast<int>(s.position);
    }
    const std::size_t a = *s.saved;
    const std::size_t b = s.position;
    return static_cast<int>(a > b ? a - b : b - a);
}

}  // namespace lagrange::viewer::builder
