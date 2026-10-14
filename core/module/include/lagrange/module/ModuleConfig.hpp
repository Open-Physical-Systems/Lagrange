#pragma once

#include "lagrange/time/SimTime.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace lagrange {

/// Capability tier of the run (RFC 0004): 0 CPU only, 1 raster GPU, 2 hardware ray tracing.
enum class CapabilityTier : std::uint8_t { Tier0 = 0, Tier1 = 1, Tier2 = 2 };

/// A domain-specific profile value ("coupling", "ambient_temperature", ...).
using ConfigValue = std::variant<bool, std::int64_t, double, std::string, std::vector<std::string>>;

/// The fields of one domain's profile entry, already validated against profiles/schema.json.
struct ModuleConfigData {
    bool enabled{true};
    std::string backend{};           // "mujoco", "physx", "" if not applicable
    std::string fidelity{};          // "averaged", "drag_plus_field", ...
    std::optional<Duration> dt{};    // overrides IModule::preferredStep when present
    std::string substituteKind{};    // "ideal", "constant", "replay"; "" when enabled
    std::string substitutePreset{};  // "zero", "clear", "ground_truth", ...
    std::string replaySource{};      // MCAP path for replay substitutes
    CapabilityTier tier{CapabilityTier::Tier0};
    bool deterministic{false};
    std::map<std::string, ConfigValue, std::less<>> values{};
};

/// The profile entry for one domain, as a module sees it (docs/design/architecture.md §5).
class ModuleConfig {
public:
    ModuleConfig() = default;
    explicit ModuleConfig(ModuleConfigData data) : data_(std::move(data)) {}

    [[nodiscard]] bool enabled() const { return data_.enabled; }
    [[nodiscard]] std::string_view backend() const { return data_.backend; }
    [[nodiscard]] std::string_view fidelity() const { return data_.fidelity; }
    [[nodiscard]] std::optional<Duration> dt() const { return data_.dt; }
    [[nodiscard]] std::string_view substituteKind() const { return data_.substituteKind; }
    [[nodiscard]] std::string_view substitutePreset() const { return data_.substitutePreset; }
    [[nodiscard]] std::string_view replaySource() const { return data_.replaySource; }
    [[nodiscard]] CapabilityTier tier() const { return data_.tier; }
    [[nodiscard]] bool deterministic() const { return data_.deterministic; }

    [[nodiscard]] bool has(std::string_view key) const { return data_.values.contains(key); }

    /// A domain-specific value of type T. An integer is also returned as a double.
    template <class T>
    [[nodiscard]] std::optional<T> get(std::string_view key) const {
        const auto it = data_.values.find(key);
        if (it == data_.values.end()) {
            return std::nullopt;
        }
        if (const T* value = std::get_if<T>(&it->second)) {
            return *value;
        }
        if constexpr (std::is_same_v<T, double>) {
            if (const auto* integer = std::get_if<std::int64_t>(&it->second)) {
                return static_cast<double>(*integer);
            }
        }
        return std::nullopt;
    }

private:
    ModuleConfigData data_;
};

}  // namespace lagrange
