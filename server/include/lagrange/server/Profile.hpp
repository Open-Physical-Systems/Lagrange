#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace lagrange::server {

/// What the running server can do with a profile (RFC 0005).
struct ProfileContext {
    std::string stage;              // the stage the server runs; profiles must name it
    std::set<std::string> modules;  // domains with a module in this server and robot
    /// Per domain, the substitute tokens that can stand in for it here.
    std::map<std::string, std::set<std::string>> substitutes;
};

/// The settings of a valid profile that the server applies.
struct ProfileSettings {
    std::set<std::string> substituted;          // domains with modules that are switched off
    std::optional<std::int64_t> rigidBodyDtNs;  // domains.rigid_body.dt
    std::optional<double> realTimeFactor;       // time.real_time_factor (0: as fast as possible)
};

struct ProfileIssue {
    bool error{true};
    std::string pointer;  // JSON pointer into the profile: "/domains/rigid_body/dt"
    std::string message;
};

struct ProfileCheck {
    bool valid{false};
    std::vector<ProfileIssue> issues;
    std::optional<ProfileSettings> settings;  // when valid
};

/// Checks profile YAML against profiles/schema.json (JSON Schema 2020-12, with jsoncons) and
/// against what this server can run: every enabled domain needs a module here, every disabled
/// one a substitute available here, the rigid body backend is MuJoCo, steps are whole
/// nanoseconds within 0.1 ms to 10 ms, and the stage is the one the server runs. Settings it
/// does not apply yet (seed, record, ...) are warnings. Never throws for bad input.
[[nodiscard]] ProfileCheck checkProfile(const std::string& yaml, const std::string& schemaJson,
                                        const ProfileContext& context);

}  // namespace lagrange::server
