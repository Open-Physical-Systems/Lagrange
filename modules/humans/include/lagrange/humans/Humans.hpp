#pragma once

// The humans domain (RFC 0009): moves the people addHuman made. At 100 Hz the limbs by the walk
// cycle; at 20 Hz where each person is going: waypoints or wandering, a social force from other
// people, robots and walls (Helbing and Molnár 1995).

#include "lagrange/humans/Body.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/services/SceneQuery.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::humans {

/// Signals, per person n: outputs body/human_<n>_<segment>/pose_command, human/<n>/pose (the
/// pelvis), human/<n>/speed (m/s), human/<n>/gait_phase (1), human/<n>/nearest_robot (m, +inf
/// without robots); input human/<n>/goal (optional: x, y in m in the region-local frame, then
/// active: above 0.5 the person goes there, and carries on when arrived and it drops).
/// config: robot_bodies (the moving robots' root bodies, as a list of names), seed.
///
/// With `zero` it is the zero substitute: nobody (they wait 100 m under the ground). `scenery` is
/// what walls are (fixed colliders, for rays at chest height) and `ground` what is under foot;
/// either may be null (no walls; the ground at z of the start).
class Humans final : public IModule {
public:
    Humans(bool zero, std::shared_ptr<const services::SceneQuery> scenery,
           std::shared_ptr<const services::SceneQuery> ground);

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;
    void shiftOrigin(const std::array<double, 3>& shift) override;

private:
    struct Person {
        Spec spec;
        std::array<SignalId, kSegments> commands;
        SignalId pose, speedOut, phaseOut, nearest, goal;
        std::uint32_t index{0};
        // State.
        double x{0.0}, y{0.0}, z{0.0}, heading{0.0};
        double vx{0.0}, vy{0.0};
        double phase{0.0};
        std::uint64_t waypoint{0};
        double dwellLeft{0.0};
        double blocked{0.0};
        double goalX{0.0}, goalY{0.0};
        std::int64_t hasGoal{0};  // 1: heading for (goalX, goalY)
        std::uint64_t picks{0};   // wander targets drawn so far
        double homeX{0.0}, homeY{0.0};
    };

    void steer(Person& person, std::size_t self, const StepContext& context, SignalBus& bus);
    void place(Person& person, SignalBus& bus) const;

    bool zero_;
    std::shared_ptr<const services::SceneQuery> scenery_;
    std::shared_ptr<const services::SceneQuery> ground_;
    std::vector<Person> people_;
    std::vector<SignalId> robots_;
    std::uint64_t ticks_{0};
};

}  // namespace lagrange::humans
