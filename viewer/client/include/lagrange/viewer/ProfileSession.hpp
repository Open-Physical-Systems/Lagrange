#pragma once

#include "lagrange/viewer/ProfileDraft.hpp"
#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

/// One domain as the profile being edited describes it.
struct DomainView {
    std::string name;
    bool inProfile{false};  ///< The draft has a line for it.
    bool enabled{false};
    bool pending{false};      ///< Differs from the profile the server runs.
    std::string mode;         ///< Enabled: backend or fidelity. Disabled: the substitute.
    std::string source;       ///< Recording a replay substitute reads; may be empty.
    std::int64_t dtNs{0};     ///< Step when enabled; 0 when the domain has none.
    bool needsSource{false};  ///< The chosen substitute needs a recording.
};

/// Editing the server's profile: the draft, its validation by the server, and the edits the
/// domain panel makes. No drawing here, so it can be tested without a window.
class ProfileSession {
public:
    enum class Validity : std::uint8_t {
        None,      ///< No profile from the server yet.
        Checking,  ///< Waiting for the server's answer for the current text.
        Valid,
        Invalid,
    };

    /// Call once per frame. Follows the server's profile and asks the server to validate the
    /// draft a moment after the last edit. `now` is in seconds, from any steady clock.
    void update(ViewerClient& client, bool connected, std::uint64_t connects, double now);
    /// Adopts a profile from the server if it is newer than the one the draft is based on.
    /// Edits are dropped: the applied profile changed under them.
    void sync(const std::shared_ptr<const ProfileInfo>& profile);

    [[nodiscard]] const std::shared_ptr<const ProfileInfo>& profile() const { return profile_; }
    [[nodiscard]] const ProfileDraft& draft() const { return draft_; }
    [[nodiscard]] Validity validity() const;
    /// Issues for the current text; empty unless validity() is Invalid or Valid.
    [[nodiscard]] const std::vector<ProfileIssue>& issues() const;
    [[nodiscard]] bool domainHasError(std::string_view name) const;
    /// The wiring the server would build from the draft. While the server is still checking
    /// an edit, this is the wiring of the text it last answered for, so the graph does not
    /// blink; wiringIsCurrent() says which it is. Empty before the first answer.
    [[nodiscard]] const WiringInfo& wiring() const { return wiring_; }
    [[nodiscard]] bool wiringIsCurrent() const {
        return validity() == Validity::Valid || validity() == Validity::Invalid;
    }
    /// Wires whose writer and reader disagree on the unit. Any of them blocks Apply.
    [[nodiscard]] std::size_t unitConflicts() const;

    /// Every domain the server listed, in its order, as the draft describes it.
    [[nodiscard]] const std::vector<DomainView>& domains() const { return views_; }
    [[nodiscard]] const DomainView* find(std::string_view name) const;

    // --- edits; each returns false when it cannot be made -----------------------------------
    bool setEnabled(std::string_view name, bool enabled);
    /// Enabled domain: its backend or fidelity. Disabled domain: its substitute.
    bool setMode(std::string_view name, std::string_view value);
    bool setStep(std::string_view name, std::int64_t dtNs);
    bool setSource(std::string_view name, std::string_view path);
    void revert();
    /// Sends the draft to the server to be applied (restarts the run at t = 0).
    void apply(ViewerClient& client) const;
    [[nodiscard]] bool canApply() const {
        return draft_.dirty() && validity() == Validity::Valid && unitConflicts() == 0;
    }

    std::string selectedDomain;  ///< Row selected in the Domains panel; may be empty.

private:
    void rebuildViews();
    bool write(std::string_view name, const std::vector<ProfileEntry>& entries);

    std::shared_ptr<const ProfileInfo> profile_;
    std::shared_ptr<const ProfileCheck> check_;
    WiringInfo wiring_;
    ProfileDraft draft_;
    std::vector<DomainView> views_;
    std::uint64_t connects_{0};
    std::uint64_t seenRevision_{0};
    std::uint64_t sentRevision_{0};
    std::uint64_t requestId_{0};
    double editTime_{0.0};
};

}  // namespace lagrange::viewer
