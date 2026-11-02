#pragma once

// The live air round one multicopter (aerodynamics.downwash_model: lbm; rotor_aero.md §8): a
// lattice Boltzmann box (LbmBox) that follows the drone, its rotors as actuator discs, the
// scenery's boxes as walls and the ground as a floor. AeroDrag owns one.

#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/aerodynamics/LbmBox.hpp"

#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace lagrange::gpu {
class Device;
}

namespace lagrange::fields {
class Field;
}

namespace lagrange::aerodynamics {

class LiveAir {
public:
    using Vec3 = std::array<double, 3>;

    struct Settings {
        double cell{0.04};      ///< m
        double width{3.0};      ///< m, across x and y, centred on the drone
        double below{2.0};      ///< m of air under the drone (down to the ground when near it)
        double above{1.0};      ///< m over it
        double maxSpeed{15.0};  ///< m/s, sets the time step (LbmBoxConfig)
        double airDensity{1.225};
        bool cpu{false};  ///< the deterministic CPU reference instead of the GPU (slow)
    };

    LiveAir();
    ~LiveAir();
    LiveAir(const LiveAir&) = delete;
    LiveAir& operator=(const LiveAir&) = delete;

    /// Opens a hardware GPU unless `settings.cpu`; false, with the reason, when there is none
    /// (Tier 0, or only a software device): the fitted model then stays.
    bool start(const Settings& settings, std::string& why);
    void setGround(bool has, double z);
    void setObstacles(std::vector<Downwash::Box> boxes);
    /// The wind round the box (m/s, world), from the next advance: it blows through the box, so
    /// the wake leans with it. What the box gives back (velocityAt, airAtRotor, field) is the
    /// air's departure from this wind, so the wind is added once, by the caller.
    void setAmbient(const Vec3& wind) { ambient_ = wind; }

    /// Moves the box with `centre` (the drone's, world) when it has drifted, pushes the rotors
    /// (world frame) and advances `dt` seconds of air.
    void advance(double dt, const Vec3& centre, std::span<const LbmBox::Rotor> rotors);

    [[nodiscard]] bool ready() const { return box_ != nullptr; }
    [[nodiscard]] bool contains(const Vec3& world) const;
    /// m/s, world, at a point in the box: the air's velocity less the wind's.
    [[nodiscard]] Vec3 velocityAt(const Vec3& world) const;
    /// The box's air over a rotor's disc, less what the same disc makes alone in such a box: the
    /// air the rotor flies in from everything else (walls, ground, the drone's other rotors).
    [[nodiscard]] Vec3 airAtRotor(const LbmBox::Rotor& rotor) const;
    /// The box's flow, every second cell, as a vector field (world) for the viewer; rebuilt at
    /// most every `interval` s of air.
    [[nodiscard]] std::shared_ptr<const fields::Field> field(double interval);
    /// The share of momentum theory's induced velocity the box makes through a lone disc
    /// (calibrated at the first rotors).
    [[nodiscard]] double isolatedShare() const { return isolatedShare_; }

private:
    void rebuild(const Vec3& centre);
    void calibrate(const LbmBox::Rotor& rotor);
    [[nodiscard]] Vec3 discMean(const LbmBox::Rotor& rotor) const;
    struct Placement {
        Vec3 origin{};
        bool floor{false};
        double height{0.0};  // m
    };
    [[nodiscard]] Placement placementFor(const Vec3& centre) const;
    [[nodiscard]] LbmBoxConfig configAt(bool floor, double height) const;

    Settings settings_;
    std::unique_ptr<gpu::Device> device_;
    std::unique_ptr<LbmBox> box_;
    Vec3 origin_{};  // world position of the box's corner
    bool hasGround_{false};
    double groundZ_{0.0};
    std::vector<Downwash::Box> obstacles_;
    Vec3 ambient_{};
    double pending_{0.0};     // s of air not yet stepped
    double sinceSync_{0.0};   // s since the velocity was copied back
    double sinceField_{1e9};  // s since the field was built
    double isolatedShare_{-1.0};
    std::shared_ptr<const fields::Field> field_;
};

}  // namespace lagrange::aerodynamics
