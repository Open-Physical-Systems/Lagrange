#pragma once

#include "lagrange/module/IModule.hpp"
#include "lagrange/signals/Signal.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange {

/// One output a domain promises to every other domain (docs/design/architecture.md §7).
///
/// With an empty `schema` the rule applies once per world; otherwise once per prim with that
/// applied schema, and "{instance}" in the pattern becomes the prim's signal instance name:
/// {"ElectricalMotorAPI_1", "joint/{instance}/torque", Scalar, N·m}.
struct ContractRule {
    std::string schema;
    std::string signalPattern;
    SignalType type{SignalType::Scalar};
    Unit unit{};
};

struct DomainContract {
    std::string domain;
    std::vector<ContractRule> provides;
};

enum class SubstituteKind : std::uint8_t { Ideal, Constant, Replay };

[[nodiscard]] std::string_view to_string(SubstituteKind kind);

/// The (kind, preset) a profile substitute token names: "zero" → (Constant, "zero"),
/// "ideal" → (Ideal, "ideal_torque"), "replay" → (Replay, ""). std::nullopt if unknown.
struct SubstituteChoice {
    SubstituteKind kind;
    std::string preset;
};
[[nodiscard]] std::optional<SubstituteChoice> substituteForToken(std::string_view token);

struct SubstituteDescriptor {
    std::string domain;  // "electrical"
    SubstituteKind kind{SubstituteKind::Constant};
    std::string preset;  // "ideal_torque", "zero", "clear", "ground_truth", "ambient"
    std::function<std::unique_ptr<IModule>()> create;
};

/// Domain contracts and the substitutes that can stand in for each domain.
class SubstituteRegistry {
public:
    /// The process-wide registry that plugins register into when loaded.
    static SubstituteRegistry& instance();

    void add(SubstituteDescriptor descriptor);
    void addContract(DomainContract contract);

    [[nodiscard]] const DomainContract* contract(std::string_view domain) const;
    /// nullptr if the domain has no substitute of that kind and preset.
    [[nodiscard]] std::unique_ptr<IModule> create(std::string_view domain, SubstituteKind kind,
                                                  std::string_view preset) const;
    [[nodiscard]] std::vector<const SubstituteDescriptor*> list(std::string_view domain) const;

private:
    std::vector<SubstituteDescriptor> substitutes_;
    std::vector<DomainContract> contracts_;
};

}  // namespace lagrange
