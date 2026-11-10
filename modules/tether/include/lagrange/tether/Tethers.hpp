#pragma once

// The tether domain (RFC 0011): every TetherAPI_1 prim is a cable from its anchor to a point on a
// body, simulated by tether::Cable every 1 ms against every collider at its body's pose this step.
// It writes each cable's tensions, length, winch speed, contacts and node positions, and pulls
// and pushes bodies through body/<b>/wrench/tether. Zero substitute (`--domain tether=off`):
// tensions 0 and nothing else.

#include "lagrange/module/IModule.hpp"
#include "lagrange/tether/Cable.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::tether {

/// The parameters of one TetherAPI_1 instance, with its fallbacks; an error names what is wrong.
struct TetherSpec {
    std::string name;
    PrimId prim{};
    BodyId anchorBody{kWorldBody};
    Vec3 anchorInBody{};  // m
    BodyId attachBody{kWorldBody};
    Vec3 attachInBody{};  // m
    CableParams params;
};
[[nodiscard]] std::vector<TetherSpec> tethersOf(const WorldView& world, std::string& error);

class Tethers final : public IModule {
public:
    /// `zero`: the substitute.
    explicit Tethers(bool zero = false) : zero_(zero) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;
    void shiftOrigin(const std::array<double, 3>& shift) override;

    /// The cables, for the server's drawing (nodes and diameter).
    [[nodiscard]] std::span<const Cable> cables() const { return cables_; }

private:
    struct Body {  // a moving body the cables can touch, pull or hang from
        BodyId id{};
        Vec3 com{};  // m, in its frame
        SignalId pose{}, twist{}, wrench{};
        bool hasWrench{false};
    };
    struct Out {
        SignalId tensionAttach{}, tensionAnchor{}, length{}, winchSpeed{}, contacts{}, caught{},
            nodes{};
    };

    [[nodiscard]] Pose bodyPose(BodyId body, const SignalBus& bus) const;
    [[nodiscard]] int movingIndex(BodyId body) const;

    bool zero_{false};
    std::vector<TetherSpec> specs_;
    std::vector<Cable> cables_;
    std::vector<Out> out_;
    std::vector<Body> bodies_;
    std::vector<int> bodyIndex_;  // per world body: index in bodies_, or -1
    std::vector<Pose> rest_;      // per world body, every joint at zero
    std::vector<std::uint32_t> robotOfBody_;
    // Colliders: where each is in its body, and the obstacles placed this step.
    std::vector<Obstacle> obstacles_;
    std::vector<Pose> colliderInBody_;
    std::vector<BodyId> colliderBody_;
    SignalId wind_{};
    bool hasWind_{false};
    double airDensity_{0.0};
    Vec3 gravity_{};
    std::vector<double> nodeBuffer_;
    std::vector<std::array<double, 6>> wrenches_;  // per moving body, summed over the cables
};

}  // namespace lagrange::tether
