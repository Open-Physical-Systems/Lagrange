#pragma once

// The actuators domain's nozzle module (RFC 0022 §8): every NozzleAPI_1 prim and the TankAPI_1
// tanks they draw from, stepped at 10 ms. Each nozzle reads act/<z>/command (the valve, 0 to 1)
// and act/<z>/pressure_command (Pa gauge, over 0 to replace act:nozzle:pressure), sprays at
// v0 = C_d √(2Δp/ρ) and ṁ = ρ A v0 times the opening while its tank holds agent, pushes its body
// back with ṁ v0 (body/<b>/wrench/nozzle), and traces its jet, drag-free, through the scene's
// colliders at the bodies' poses until it lands: act/<z>/impact, flow, footprint and path, which
// the fire domain reads to wet the fire the jet lands on (modules never call each other). Each
// tank's agent is a weight at its prim, m g (body/<b>/wrench/tank), shrinking as it empties.
//
// Substitute: `constant` (and `--domain actuators=off`): every valve shut, the tanks full.

#include "lagrange/actuators/Grippers.hpp"
#include "lagrange/actuators/Nozzle.hpp"
#include "lagrange/module/IModule.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lagrange::actuators {

/// One TankAPI_1 prim.
struct TankSpec {
    std::string name;
    PrimId prim{};
    BodyId body{kWorldBody};
    Pose inBody{};
    double capacity{0.0};  // m³
    double initial{1.0};   // 1
    double density{kWaterDensity};

    [[nodiscard]] double fullMass() const { return capacity * density; }  // kg
};

/// One NozzleAPI_1 prim.
struct NozzleSpec {
    std::string name;
    PrimId prim{};
    BodyId body{kWorldBody};
    Pose inBody{};  // the exit and its frame in the body
    NozzleParams params;
    int tank{-1};            // into NozzleSet::tanks; -1: a hose
    bool signalLoad{false};  // a signal-model ElectricalLoadAPI_1 on its prim
};

struct NozzleSet {
    std::vector<NozzleSpec> nozzles;
    std::vector<TankSpec> tanks;  // those the nozzles name, each once
};
/// Every nozzle of the world with its tank; an error names what is wrong (a required attribute
/// missing, a value out of range, a tank relation to a prim with no TankAPI_1).
[[nodiscard]] NozzleSet nozzlesOf(const WorldView& world, std::string& error);

/// Points of act/<z>/path (3 doubles each, NaN past the jet's end).
inline constexpr std::uint32_t kJetPathPoints = 32;

class Nozzles final : public IModule {
public:
    enum class Mode : std::uint8_t { Full, Shut };

    explicit Nozzles(Mode mode = Mode::Full, SceneProvider scene = {},
                     std::shared_ptr<Commands> commands = {})
        : mode_(mode), scene_(std::move(scene)), commands_(std::move(commands)) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The last step's traced jets, one per nozzle (for tests and drawing).
    [[nodiscard]] std::span<const JetPath> jets() const { return jets_; }

private:
    struct Body {
        SignalId pose{}, wrenchNozzle{}, wrenchTank{};
        bool moving{false};
        Vec3 com{};
        std::uint32_t robot{0};  // the first body of its group of jointed bodies
    };
    struct State {
        double command{0.0};
        double busCommand{0.0};
    };
    struct Out {
        SignalId command{}, pressureCommand{}, flow{}, velocity{}, impact{}, footprint{}, path{},
            tankMass{}, pumpPower{}, reaction{}, landed{}, loadDemand{};
    };

    [[nodiscard]] Pose bodyPose(BodyId body, const SignalBus& bus) const;
    void push(BodyId body, bool tank, const Vec3& force, const Vec3& at, const Pose& pose);

    Mode mode_;
    SceneProvider scene_;
    std::shared_ptr<Commands> commands_;
    std::uint64_t seenGeneration_{0};
    NozzleSet set_;
    std::vector<State> state_;
    std::vector<Out> out_;
    std::vector<double> tankMass_;  // kg, per tank
    std::vector<JetPath> jets_;
    std::vector<Body> bodies_;
    std::vector<Pose> rest_;
    std::vector<BodyId> shapeBody_;
    std::vector<std::array<double, 6>> nozzleWrench_, tankWrench_;  // per world body
    Vec3 gravity_{0.0, 0.0, -9.80665};
};

}  // namespace lagrange::actuators
