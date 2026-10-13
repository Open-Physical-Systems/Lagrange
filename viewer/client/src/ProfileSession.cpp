#include "lagrange/viewer/ProfileSession.hpp"

#include <algorithm>

namespace lagrange::viewer {

namespace {

// Long enough that typing a number does not send a request per key, short enough that the
// wiring preview follows a click without a visible wait (design: within 200 ms).
constexpr double kValidationDelaySeconds = 0.12;

const std::vector<ProfileIssue> kNoIssues;

bool isPlain(std::string_view text) {
    return !text.empty() && std::ranges::all_of(text, [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
               c == '_' || c == '.' || c == '/' || c == '-';
    });
}

std::string quote(std::string_view text) {
    if (isPlain(text)) {
        return std::string(text);
    }
    std::string quoted = "\"";
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            quoted += '\\';
        }
        quoted += c;
    }
    quoted += '"';
    return quoted;
}

std::string unquote(std::string_view text) {
    if (text.size() < 2 || text.front() != '"' || text.back() != '"') {
        return std::string(text);
    }
    std::string plain;
    for (std::size_t i = 1; i + 1 < text.size(); ++i) {
        if (text[i] == '\\' && i + 2 < text.size()) {
            ++i;
        }
        plain += text[i];
    }
    return plain;
}

const SubstituteOption* findSubstitute(const DomainInfo& info, std::string_view token) {
    for (const SubstituteOption& option : info.substitutes) {
        if (option.token == token) {
            return &option;
        }
    }
    return nullptr;
}

const char* modeKey(const DomainInfo& info) {
    return info.backends.empty() ? "fidelity" : "backend";
}

}  // namespace

void ProfileSession::sync(const std::shared_ptr<const ProfileInfo>& profile) {
    if (!profile || (profile_ && profile_->serial == profile->serial)) {
        return;
    }
    profile_ = profile;
    draft_.reset(profile->yaml);
    check_.reset();
    sentRevision_ = 0;
    rebuildViews();
}

void ProfileSession::update(ViewerClient& client, bool connected, std::uint64_t connects,
                            double now) {
    sync(client.profile());
    if (connects != connects_) {
        connects_ = connects;
        sentRevision_ = 0;  // a new connection never saw the earlier request
    }
    if (draft_.revision() != seenRevision_) {
        seenRevision_ = draft_.revision();
        editTime_ = now;
    }
    if (connected && profile_ && sentRevision_ != draft_.revision() &&
        now - editTime_ >= kValidationDelaySeconds) {
        requestId_ = client.validateProfile(draft_.text());
        sentRevision_ = draft_.revision();
    }
    if (const auto check = client.profileCheck(); check && check->requestId == requestId_) {
        check_ = check;
    }
}

ProfileSession::Validity ProfileSession::validity() const {
    if (!profile_) {
        return Validity::None;
    }
    if (!check_ || check_->requestId != requestId_ || sentRevision_ != draft_.revision()) {
        return Validity::Checking;
    }
    return check_->valid ? Validity::Valid : Validity::Invalid;
}

const std::vector<ProfileIssue>& ProfileSession::issues() const {
    const Validity state = validity();
    return state == Validity::Valid || state == Validity::Invalid ? check_->issues : kNoIssues;
}

bool ProfileSession::domainHasError(std::string_view name) const {
    return std::ranges::any_of(issues(), [name](const ProfileIssue& issue) {
        constexpr std::string_view kPrefix = "/domains/";
        if (!issue.error || !issue.pointer.starts_with(kPrefix)) {
            return false;
        }
        const std::string_view rest = std::string_view(issue.pointer).substr(kPrefix.size());
        return rest.starts_with(name) && (rest.size() == name.size() || rest[name.size()] == '/');
    });
}

const DomainView* ProfileSession::find(std::string_view name) const {
    for (const DomainView& view : views_) {
        if (view.name == name) {
            return &view;
        }
    }
    return nullptr;
}

void ProfileSession::rebuildViews() {
    views_.clear();
    if (!profile_) {
        return;
    }
    views_.reserve(profile_->domains.size());
    for (const DomainInfo& info : profile_->domains) {
        DomainView view;
        view.name = info.name;
        const auto entries = draft_.domain(info.name);
        view.inProfile = entries.has_value();
        view.pending = draft_.domainChanged(info.name);
        if (entries) {
            const std::string* enabled = findEntry(*entries, "enabled");
            view.enabled = enabled != nullptr && *enabled == "true";
            if (view.enabled) {
                if (const std::string* mode = findEntry(*entries, modeKey(info))) {
                    view.mode = *mode;
                } else if (!info.backends.empty()) {
                    view.mode = info.backends.front();
                } else if (!info.fidelities.empty()) {
                    view.mode = info.fidelities.front();
                }
                const std::string* step = findEntry(*entries, "dt");
                view.dtNs = step != nullptr ? parseSeconds(*step).value_or(0) : info.dtNs;
            } else {
                if (const std::string* substitute = findEntry(*entries, "substitute")) {
                    view.mode = *substitute;
                }
                if (const std::string* source = findEntry(*entries, "source")) {
                    view.source = unquote(*source);
                }
                const SubstituteOption* option = findSubstitute(info, view.mode);
                view.needsSource = option != nullptr && option->needsSource;
            }
        } else if (!info.substitutes.empty()) {
            view.mode = info.substitutes.front().token;
        }
        views_.push_back(std::move(view));
    }
}

bool ProfileSession::write(std::string_view name, const std::vector<ProfileEntry>& entries) {
    if (!draft_.setDomain(name, entries)) {
        return false;
    }
    rebuildViews();
    return true;
}

bool ProfileSession::setEnabled(std::string_view name, bool enabled) {
    const DomainInfo* info = profile_ ? profile_->find(name) : nullptr;
    if (info == nullptr || !info->available) {
        return false;
    }
    const auto applied = draft_.appliedDomain(name);
    const std::string* appliedEnabled = applied ? findEntry(*applied, "enabled") : nullptr;
    const bool wasEnabled = appliedEnabled != nullptr && *appliedEnabled == "true";

    std::vector<ProfileEntry> entries;
    if (applied && wasEnabled == enabled) {
        entries = *applied;  // back to what the server runs
    } else if (enabled) {
        entries.push_back({"enabled", "true"});
        if (!info->backends.empty()) {
            entries.push_back({"backend", info->backends.front()});
        } else if (!info->fidelities.empty()) {
            entries.push_back({"fidelity", info->fidelities.front()});
        }
        if (info->dtNs > 0) {
            entries.push_back({"dt", formatSeconds(info->dtNs)});
        }
    } else {
        // A disabled domain always names what replaces it (CLAUDE.md §11.1).
        if (info->substitutes.empty()) {
            return false;
        }
        entries.push_back({"enabled", "false"});
        entries.push_back({"substitute", info->substitutes.front().token});
    }
    return write(name, entries);
}

bool ProfileSession::setMode(std::string_view name, std::string_view value) {
    const DomainInfo* info = profile_ ? profile_->find(name) : nullptr;
    auto entries = draft_.domain(name);
    if (info == nullptr || !entries) {
        return false;
    }
    const std::string* enabled = findEntry(*entries, "enabled");
    if (enabled != nullptr && *enabled == "true") {
        setEntry(*entries, modeKey(*info), std::string(value));
    } else {
        const SubstituteOption* option = findSubstitute(*info, value);
        if (option == nullptr) {
            return false;
        }
        setEntry(*entries, "substitute", std::string(value));
        if (!option->needsSource) {
            eraseEntry(*entries, "source");
        }
    }
    return write(name, *entries);
}

bool ProfileSession::setStep(std::string_view name, std::int64_t dtNs) {
    auto entries = draft_.domain(name);
    if (!entries || dtNs <= 0) {
        return false;
    }
    setEntry(*entries, "dt", formatSeconds(dtNs));
    return write(name, *entries);
}

bool ProfileSession::setSource(std::string_view name, std::string_view path) {
    auto entries = draft_.domain(name);
    if (!entries) {
        return false;
    }
    if (path.empty()) {
        eraseEntry(*entries, "source");
    } else {
        setEntry(*entries, "source", quote(path));
    }
    return write(name, *entries);
}

void ProfileSession::revert() {
    draft_.revert();
    rebuildViews();
}

void ProfileSession::apply(ViewerClient& client) const {
    client.applyProfile(draft_.text());
}

}  // namespace lagrange::viewer
