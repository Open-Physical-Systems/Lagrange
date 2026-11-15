#pragma once

// What the Domains workspace's secondary tabs show, worked out without drawing so it can be
// tested without a window (docs/design/ui/03-domains/SPEC.md): the line diff of the applied
// and the edited profile (Diff), the profile schema as rows (Schema), and the coupling of the
// domains from the wiring the server sent (Coupling).

#include "lagrange/viewer/ViewerClient.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

// --- Diff ----------------------------------------------------------------------------------

struct DiffLine {
    enum class Kind : std::uint8_t { Same, Removed, Added };
    Kind kind{Kind::Same};
    int beforeLine{0};  ///< 1-based line in the applied text; 0 for an added line.
    int afterLine{0};   ///< 1-based line in the edited text; 0 for a removed line.
    std::string text;
};

/// The shortest line diff of `before` and `after` (longest common subsequence), in order,
/// removed lines before the added lines that replace them.
[[nodiscard]] std::vector<DiffLine> diffLines(std::string_view before, std::string_view after);
/// Runs of consecutive changed lines (hunks) in a diff.
[[nodiscard]] std::size_t diffHunks(const std::vector<DiffLine>& diff);

// --- Schema --------------------------------------------------------------------------------

/// One key of the profile schema.
struct SchemaField {
    std::string key;  ///< "dt"
    int depth{0};     ///< 0 for the section's own keys, 1 for keys of a nested object, ...
    bool required{false};
    std::string type;         ///< "number", "boolean", "one of: lumped | conduction", ...
    std::string constraints;  ///< "> 0, ≤ 10", "≥ 1"; empty when none.
    std::string defaultValue;
    std::string description;
};

/// A group of keys: the top level, or one domain's entry under `domains`.
struct SchemaSection {
    std::string name;  ///< "profile" for the top level, else the domain ("thermal").
    std::string note;  ///< A rule that spans keys, from the schema's descriptions.
    std::vector<SchemaField> fields;
};

struct ProfileSchemaDoc {
    std::filesystem::path path;  ///< The file read; empty when none was found.
    std::string error;           ///< Why it could not be read; empty when it was.
    std::string title;
    std::string dialect;       ///< "$schema": "https://json-schema.org/draft/2020-12/schema"
    std::uint32_t version{0};  ///< The const of schema_version.
    SchemaSection top;         ///< Top-level keys, with `domains` itself left out.
    std::vector<SchemaSection> domains;  ///< In the schema's order.
    std::vector<std::string> requiredDomains;

    [[nodiscard]] const SchemaSection* domain(std::string_view name) const;
};

/// The profile schema shipped with this install: the viewer's own copy
/// (share/lagrange_viewer/profiles/schema.json), else lagrange_profiles' file, found under
/// AMENT_PREFIX_PATH. Empty when neither is there.
[[nodiscard]] std::filesystem::path findProfileSchema();
/// Reads a profile schema (JSON) into sections. Never throws: a file that cannot be read
/// gives a document with `error` set.
[[nodiscard]] ProfileSchemaDoc loadProfileSchema(const std::filesystem::path& path);

// --- Coupling ------------------------------------------------------------------------------

/// Every wire from one module to another, gathered: what the two exchange and how often.
struct Coupling {
    std::string from;  ///< Writer module.
    std::string to;    ///< Reader module.
    std::string fromDomain;
    std::string toDomain;
    std::int64_t fromDtNs{0};
    std::int64_t toDtNs{0};
    std::vector<std::string> symbols;  ///< "τ · N·m", each once, in wire order.
    std::size_t wires{0};
    std::size_t unitConflicts{0};
    bool stiff{false};
    bool fromSubstitute{false};
    /// The reader steps faster than the writer: it reads the same value `holdSteps` times
    /// (zero-order hold). 1 when it reads each new value once.
    std::int64_t holdSteps{1};
    /// Both directions are wired (the pair is coupled both ways).
    bool twoWay{false};
};

[[nodiscard]] std::vector<Coupling> couplings(const WiringInfo& wiring);

}  // namespace lagrange::viewer
