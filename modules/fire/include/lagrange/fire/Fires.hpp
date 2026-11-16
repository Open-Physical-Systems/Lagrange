#pragma once

// The fire domain (RFC 0022 steps 2 and 3): every FireAPI_1 prim is a fuel item. Once ignited
// (at fire:ignitionTime, by fire/<n>/ignite, or when the thermal node on its prim reaches
// fire:ignitionTemperature) its heat release rate grows as t², burns as a pool or follows a
// measured curve, and burns out with its fuel. Its flame (Heskestad) leans in the wind (AGA) and
// radiates (point source) into every thermal node in sight, through fire_radiation/<node>/
// loss_power, which the thermal network adds to that node: robots near it heat, and fuel near
// it ignites. Water at or above the fuel's critical rate stops its growth and decays it (Evans,
// NISTIR 5254), except on burning liquids; foam smothers it; a wind over its blow-off speed blows
// a small one out. Step 100 ms, deterministic, no random numbers.
//
// Substitutes: zero (no fires: hrr 0, nothing heated) and constant (every fire at its full
// involvement from the start, unaffected by water, foam and wind: a steady fire for tests).

#include "lagrange/fire/Correlations.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/services/SceneQuery.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::fire {

using Vec3 = std::array<double, 3>;

enum class FireClass : std::uint8_t { A, B, C };
enum class Growth : std::uint8_t { TSquared, Pool, Curve };

/// fire/<n>/state.
enum class FireState : std::uint8_t {
    NotIgnited = 0,
    Growing = 1,
    FullyInvolved = 2,
    Decaying = 3,
    OutWater = 4,
    OutBurnt = 5,
    OutBlownOff = 6,
    OutSmothered = 7,
    OutOxygen = 8,
};
[[nodiscard]] inline bool isOut(FireState s) {
    return s >= FireState::OutWater;
}

/// One FireAPI_1 instance, checked, with its fallbacks.
struct FireSpec {
    std::string name;
    PrimId prim{};
    BodyId body{kWorldBody};
    Vec3 baseInBody{};  // m: the top centre of its collider, or the prim's origin
    FireClass fireClass{FireClass::A};
    Growth growth{Growth::TSquared};
    double alpha{0.0};             // W/s²
    double hrrPerArea{0.0};        // W/m²
    double area{0.0};              // m²
    double fuelMass{0.0};          // kg
    double heatOfCombustion{0.0};  // J/kg
    double radiativeFraction{0.0};
    double sootYield{0.0};        // kg/kg
    double poolBurningRate{0.0};  // kg/(m²·s)
    double poolExtinction{0.0};   // 1/m
    double decayFraction{0.2};
    double ignitionTime{-1.0};                   // s
    std::optional<double> ignitionTemperature;   // K
    std::optional<double> blowOffSpeed;          // m/s
    std::optional<double> blowOffMaxHrr;         // W
    std::optional<double> criticalWaterRate;     // kg/(m²·s)
    std::optional<double> foamBlanketThickness;  // m
    std::vector<double> curve;                   // t, Q pairs
    std::vector<std::uint32_t> colliders;        // its own colliders (World::Colliders order)

    /// Its heat release rate at full involvement over `burningArea` (W).
    [[nodiscard]] double fullHrr(double burningArea) const;
};

/// Every FireAPI_1 of the world; an error names what is wrong (a required value missing, a
/// preset that does not exist, a number out of range).
[[nodiscard]] std::vector<FireSpec> firesOf(const WorldView& world, std::string& error);

/// What reaches one fire during a step.
struct Exposure {
    double waterRate{0.0};   // kg/(m²·s) over its burning area
    double foamSupply{0.0};  // m/s: finished foam depth laid down per second
    double airSpeed{0.0};    // m/s at the flame: wind and downwash
    bool ignite{false};
    double nodeTemperature{0.0};   // K: the thermal node on its prim, 0 when none
    double waterTemperature{0.0};  // K, 0 when not known: latent heat only
};

/// One fire's heat release rate through ignition, growth, suppression and burn-out (RFC 0022
/// §2, §5). Pure: no bus, so its laws are tested alone.
class Burner {
public:
    Burner() = default;
    /// `extinguishFraction`: water has put the fire out when Q falls below this share of Q at the
    /// water's arrival (a modelling choice: Evans' decay never reaches 0).
    Burner(const FireSpec& spec, double extinguishFraction);

    /// Advances from `t` (s, the step's start) by `dt` (s).
    void step(double t, double dt, const Exposure& exposure);
    /// The constant substitute: full involvement from the start.
    void holdFull();

    [[nodiscard]] FireState state() const { return state_; }
    [[nodiscard]] double hrr() const { return hrr_; }               // W
    [[nodiscard]] double burningArea() const;                       // m²
    [[nodiscard]] double fuelRemaining() const;                     // kg
    [[nodiscard]] double foamCoverage() const { return foam_; }     // 1
    [[nodiscard]] double waterRate() const { return waterRate_; }   // kg/(m²·s)
    [[nodiscard]] double waterCooling() const { return cooling_; }  // W
    [[nodiscard]] double steamRate() const { return steam_; }       // kg/s
    [[nodiscard]] bool hazard() const { return hazard_; }
    /// The mass burning flux (kg/(m²·s)), for the flame's tilt.
    [[nodiscard]] double massFlux() const;

    void save(std::vector<double>& out) const;
    [[nodiscard]] bool restore(std::span<const double> in);

private:
    const FireSpec* spec_{nullptr};
    double extinguish_{0.01};
    FireState state_{FireState::NotIgnited};
    double clock_{0.0};    // s of growth since ignition (held while water decays the fire)
    double free_{0.0};     // W before the burn-out factor
    double hrr_{0.0};      // W
    double burnt_{0.0};    // kg
    double atWater_{0.0};  // W when water above the critical rate arrived; 0 when none is on
    double foam_{0.0};
    double waterRate_{0.0}, cooling_{0.0}, steam_{0.0};
    bool hazard_{false};
};

class Fires final : public IModule {
public:
    enum class Mode : std::uint8_t { Simulate, Zero, Constant };

    /// `scenery`: the fixed scenery that blocks radiation (nullptr: nothing blocks), with the
    /// World::Colliders index of each of its shapes (so a fire's or a target's own box does not
    /// hide it).
    explicit Fires(Mode mode = Mode::Simulate,
                   std::shared_ptr<const services::SceneQuery> scenery = nullptr,
                   std::vector<std::uint32_t> sceneryColliders = {});

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    [[nodiscard]] std::span<const FireSpec> fires() const { return specs_; }
    [[nodiscard]] std::span<const Burner> burners() const { return burners_; }

private:
    struct Out {
        SignalId hrr{}, state{}, flameHeight{}, tilt{}, radiative{}, fuel{}, soot{}, waterRate{},
            cooling{}, steam{}, foam{}, hazard{}, base{};
        SignalId ignite{}, downwash{}, waterSupply{}, foamSupply{}, nodeTemperature{};
        bool hasNodeTemperature{false};
    };
    struct Target {  // a thermal node the fires heat
        std::string name;
        BodyId body{kWorldBody};
        Vec3 inBody{};
        double emissivity{0.9};
        double area{0.0};  // m², thermal:radiativeArea
        std::vector<std::uint32_t> colliders;
        int fire{-1};  // the fire on the same prim, which does not heat it
        SignalId power{};
    };
    struct Flame {  // this step's point source
        Vec3 source{};
        double hrr{0.0};       // W
        double diameter{0.0};  // m
    };
    struct Schedule {  // water from the run's options: a sprinkler's rate from a time on
        std::size_t fire{};
        double rate{0.0};   // kg/(m²·s)
        double start{0.0};  // s
    };

    [[nodiscard]] Pose bodyPose(BodyId body, const SignalBus& bus) const;
    [[nodiscard]] bool blocked(const Vec3& from, const Vec3& to, const FireSpec& fire,
                               const Target& target) const;
    void writeOutputs(std::size_t k, const Vec3& base, const Vec3& axis, double height,
                      SignalBus& bus) const;

    Mode mode_;
    std::shared_ptr<const services::SceneQuery> scenery_;
    std::vector<std::uint32_t> sceneryColliders_;
    std::vector<FireSpec> specs_;
    std::vector<Burner> burners_;
    std::vector<Out> out_;
    std::vector<Target> targets_;
    std::vector<Schedule> schedule_;
    std::vector<int> movingIndex_;  // per world body: index into poses_, or -1
    std::vector<SignalId> poses_;
    std::vector<Pose> rest_;
    std::vector<Flame> flames_;
    std::vector<std::vector<bool>> warned_;  // per fire and target: the near-field warning given
    SignalId wind_{}, airDensity_{}, airTemperature_{}, rain_{};
    Vec3 gravity_{};
    double extinguishFraction_{0.01};
    double siteDensity_{0.0}, siteTemperature_{0.0};
};

}  // namespace lagrange::fire
