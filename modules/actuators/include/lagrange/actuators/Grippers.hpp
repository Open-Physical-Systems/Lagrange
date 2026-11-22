#pragma once

// The actuators domain's gripper module (RFC 0020 §4.1–§4.2): every SuctionGripperAPI_1 and
// MagnetGripperAPI_1 prim, stepped at 1 ms. Each reads act/<g>/command, finds what is in front of
// it with the scene's ray queries at the bodies' poses, and holds it by an equal and opposite
// pair of forces, body/<part>/wrench/suction (or /magnet) toward the gripper and
// body/<gripper's body>/wrench/... back, both at the gripper's face. The contact between the
// gripper and the part carries the reaction and the friction, so the part slips and falls when
// the physics says so. It never welds.
//
// Substitutes (§3.3): `ideal` holds when commanded and in contact, with no pressure or coil
// dynamics (the cup at its ultimate vacuum, the coil at its rated current); `constant` (and
// `--domain actuators=off`) never holds.

#include "lagrange/actuators/Gripper.hpp"
#include "lagrange/module/IModule.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lagrange::actuators {

/// Commands given outside the bus (Server::commandActuator, Python): the latest per gripper,
/// taken by the module at its next step. Thread-safe.
class Commands {
public:
    void set(const std::string& name, double value);
    /// The value set for `name` since the last take, if any.
    [[nodiscard]] bool take(const std::string& name, double& value);
    /// Changes whenever set() is called: a cheap test before take().
    [[nodiscard]] std::uint64_t generation() const { return generation_.load(); }

private:
    std::mutex mutex_;
    std::map<std::string, double, std::less<>> pending_;
    std::atomic<std::uint64_t> generation_{0};
};

/// One gripper of the stage, with its attributes and fallbacks.
struct GripperSpec {
    enum class Kind : std::uint8_t { Suction, Magnet };
    Kind kind{Kind::Suction};
    std::string name;  // the prim's signal instance name
    PrimId prim{};
    BodyId body{kWorldBody};
    Pose inBody{};  // the face's centre and frame in the body
    SuctionParams suction;
    MagnetParams magnet;
};
/// Every gripper of the world; an error names what is wrong (a required attribute missing, a
/// value out of range).
[[nodiscard]] std::vector<GripperSpec> grippersOf(const WorldView& world, std::string& error);

/// The scene the rays are cast in, at the bus's body poses (sensors::BodyScene in the server):
/// shape i is World::Colliders entry i. nullptr: nothing can be found, nothing held.
using SceneProvider = std::function<const services::SceneQuery*(const SignalBus& bus)>;

class Grippers final : public IModule {
public:
    enum class Mode : std::uint8_t { Full, Ideal, Never };

    explicit Grippers(Mode mode = Mode::Full, SceneProvider scene = {},
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

    /// Config keys: the site's atmosphere (Pa) and air temperature (K) when the weather does not
    /// run (env/pressure, env/air_temperature win when it does).
    static constexpr const char* kPressureKey = "pressure";
    static constexpr const char* kTemperatureKey = "temperature";

private:
    struct Body {  // a body a gripper can be on or hold
        SignalId pose{}, wrenchSuction{}, wrenchMagnet{};
        bool moving{false}, pushable{false};
        Vec3 com{};
        double airPermeability{0.0};       // m/(Pa·s)
        double relativePermeability{1.0};  // of its iron, for magnets
    };
    struct State {
        double command{0.0};
        double busCommand{0.0};  // last value seen on act/<g>/command
        double pressure{kStandardPressure};
        double current{0.0};
        bool latched{false};
    };
    struct Out {
        SignalId command{}, pressure{}, sealed{}, holding{}, held{}, force{}, current{}, gap{},
            power{};
    };

    [[nodiscard]] Pose bodyPose(BodyId body, const SignalBus& bus) const;
    void push(BodyId body, bool magnet, const Vec3& force, const Vec3& at, const Pose& pose);

    Mode mode_;
    SceneProvider scene_;
    std::shared_ptr<Commands> commands_;
    std::uint64_t seenGeneration_{0};
    std::vector<GripperSpec> specs_;
    std::vector<State> state_;
    std::vector<Out> out_;
    std::vector<Body> bodies_;                                         // per world body
    std::vector<Pose> rest_;                                           // per world body
    std::vector<BodyId> shapeBody_;                                    // per collider
    std::vector<std::array<double, 6>> suctionWrench_, magnetWrench_;  // per world body
    bool anySuction_{false}, anyMagnet_{false};
    SignalId envPressure_{}, envTemperature_{};
    double sitePressure_{kStandardPressure};
    double siteTemperature_{kReferenceTemperature};
};

}  // namespace lagrange::actuators
