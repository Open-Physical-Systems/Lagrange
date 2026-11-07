#pragma once

#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/signals/SignalBus.hpp"
#include "lagrange/world/World.hpp"

#include <cstdint>
#include <mutex>
#include <vector>

namespace lagrange::sensors {

/// Every collider of a world placed at the body poses on a bus: the scene of the plugins' ray
/// and overlap queries (RFC 0008 §2). Shape i is collider i. Built on first use and refitted
/// only when a body pose was committed since the last call, so a run whose plugins never ask
/// pays nothing.
class BodyScene {
public:
    /// `world` must outlive the scene (the run's world tables do).
    explicit BodyScene(const World& world);

    /// The scene at the poses committed on `bus` now. Safe from several threads at once; the
    /// returned query is valid until the next call that sees a pose move.
    [[nodiscard]] const services::SceneQuery& at(const SignalBus& bus);

private:
    const World& world_;
    std::vector<Pose> rest_;       // per body, for bodies without a pose signal
    std::vector<SignalId> poses_;  // per body, body/<b>/pose when it has one
    std::vector<SimTime> stamps_;  // per body, the pose's stamp at the last refit
    std::vector<ShapeCaster::Shape> shapes_;
    ShapeCaster caster_;
    std::mutex mutex_;
    bool resolved_{false};
    bool placed_{false};
};

}  // namespace lagrange::sensors
