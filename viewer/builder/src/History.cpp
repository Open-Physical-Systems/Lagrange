// Stub (RFC 0016 WP0): WP-A replaces this file with the snapshot stack of RFC 0016 §9. Until
// then there is no undo: a commit replaces the one state, and undo and redo do nothing.
#include "lagrange/viewer/builder/History.hpp"

#include <utility>
#include <vector>

namespace lagrange::viewer::builder {

struct History::State {
    Document document;
    std::vector<std::string> labels;  // one: the current state's
    std::uint64_t revision{0};
    int editsSinceSave{0};
};

History::History(Document initial, std::size_t depth)
    : History(std::move(initial), "Start", depth) {}

History::History(Document initial, std::string label, std::size_t /*depth*/)
    : state_(std::make_unique<State>()) {
    state_->document = std::move(initial);
    state_->labels.push_back(std::move(label));
}

History::~History() = default;
History::History(History&&) noexcept = default;
History& History::operator=(History&&) noexcept = default;

void History::commit(Document next, std::string label, std::string /*mergeKey*/, double /*nowS*/) {
    state_->document = std::move(next);
    state_->labels.front() = std::move(label);
    ++state_->revision;
    ++state_->editsSinceSave;
}

bool History::undo() {
    return false;  // not implemented yet (WP-A)
}

bool History::redo() {
    return false;  // not implemented yet (WP-A)
}

bool History::goTo(std::size_t step) {
    return step == 0;  // the one state there is
}

const Document& History::current() const {
    return state_->document;
}

std::span<const std::string> History::labels() const {
    return state_->labels;
}

std::size_t History::position() const {
    return 0;
}

bool History::canUndo() const {
    return false;
}

bool History::canRedo() const {
    return false;
}

std::uint64_t History::revision() const {
    return state_->revision;
}

void History::markSaved() {
    state_->editsSinceSave = 0;
}

int History::editsSinceSave() const {
    return state_->editsSinceSave;
}

}  // namespace lagrange::viewer::builder
