#pragma once

// Undo and redo by snapshots of the whole document (RFC 0016 §9). Owner: WP-A.

#include "lagrange/viewer/builder/Document.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace lagrange::viewer::builder {

/// States are numbered 0 (the oldest kept) to labels().size() - 1 (the newest). position() is
/// the state current() shows. State 0's label is the one given at construction.
class History {
public:
    explicit History(Document initial, std::size_t depth = 256);
    History(Document initial, std::string label, std::size_t depth = 256);
    ~History();
    History(History&&) noexcept;
    History& operator=(History&&) noexcept;
    History(const History&) = delete;
    History& operator=(const History&) = delete;

    /// Makes `next` the current state, labelled `label` ("Set range/max 12 → 20 m"). Drops
    /// every state after position() (redo is cleared). When `mergeKey` is not empty, equals the
    /// current state's key and `nowS - its time < 1 s`, the current state is replaced instead of
    /// a new one being added (typing in one field is one step). `nowS` is ImGui::GetTime() in
    /// seconds (plan decision 5: headless runs are then reproducible). Beyond `depth` states the
    /// oldest is dropped.
    void commit(Document next, std::string label, std::string mergeKey, double nowS);
    bool undo();                  ///< false at state 0
    bool redo();                  ///< false at the newest state
    bool goTo(std::size_t step);  ///< false when out of range; the History tab's click

    [[nodiscard]] const Document& current() const;
    [[nodiscard]] std::span<const std::string> labels() const;  // newest last
    [[nodiscard]] std::size_t position() const;
    [[nodiscard]] bool canUndo() const;
    [[nodiscard]] bool canRedo() const;
    /// Changes whenever current() changes (commit, undo, redo, goTo): the scene adapter
    /// rebuilds its statics and validation runs again when it does.
    [[nodiscard]] std::uint64_t revision() const;

    /// The current state is what is on disk now.
    void markSaved();
    /// Steps between the saved state and the current one (the bar's "● unsaved · N"): 0 when
    /// current() is what was saved; when never saved (or the saved state was dropped by the
    /// depth), the edits since state 0, position(). Whether a file exists at all is the
    /// session's (BuilderSession::path), not the history's.
    [[nodiscard]] int editsSinceSave() const;

private:
    struct State;  // WP-A's to change: the layout of the stack is not part of the interface
    std::unique_ptr<State> state_;
};

}  // namespace lagrange::viewer::builder
