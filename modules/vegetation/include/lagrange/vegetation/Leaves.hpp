#pragma once

// Flying leaves (docs/physics/ground.md "Flying leaves"): leaves and twigs a propeller cuts from a
// crown, or the wind and the rotors' wake strip from it, fly as particles until they settle. The
// vegetation domain owns them (Vegetation) and publishes them as lagrange/world/Leaves.hpp lays
// out. Everything here is free of the bus, so the tests drive it directly.
//
// Each particle is a small plate falling at its own terminal speed v_t, carried by the air:
//
//     u = v − air(x),   a = g + a_glide − k |u| u,   k = |g| / v_t²
//
// so in still air it falls at v_t and in a steady wind it drifts at the wind's speed. The drag is
// stepped implicitly (v' = (v + dt (g + a_glide + k|u| air)) / (1 + dt k|u|), |u| at the start of
// the step), stable at any step and exact at the terminal speed. The tumbling is drawing level: the
// blade tilts by A sin φ about a yaw that turns at a constant rate, φ advancing at the flutter
// frequency, and it glides towards the side it tilts to, a_glide = G |g| sin φ (cos yaw, sin yaw,
// 0), mean zero. A particle that reaches the ground (the highest fixed plane, height field, upright
// box or upright cylinder top under it that it was above) stops there and lies flat.
//
// The air at a particle is the site wind (env/wind/velocity, uniform) plus every rotor's wake
// (rotorWake): the vortex cylinder of an actuator disc on its axis (Leishman, Principles of
// Helicopter Aerodynamics, 2nd ed., 2006, §2.7, as aerodynamics::Downwash), w(s) = v_i (1 +
// s/√(s² + R²)), a top hat of radius R √(v_i/w) below the disc (R above it), and past 4 R a round
// jet that keeps its momentum flux, w ∝ 1/s, radius ∝ s. The vegetation domain cannot ask the
// aerodynamics domain for its wake at each particle (architecture invariant 1), so it rebuilds this
// simpler one from rotor/<j>/induced_velocity and thrust_axis.

#include "lagrange/world/Leaves.hpp"
#include "lagrange/world/Trees.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace lagrange::vegetation {

/// The flying leaves' numbers. Every default is TODO(verify) unless its comment says otherwise
/// (docs/physics/ground.md "Flying leaves", "Numbers and sources").
struct LeafParams {
    /// m/s, a broad leaf's terminal fall speed in still air: about 1-2 m/s. TODO(verify) against
    /// measured falling-leaf speeds.
    double terminalSpeed{1.5};
    /// ± share each particle's terminal speed and size are drawn within (uniform).
    double spread{0.2};
    /// G, the glide's peak acceleration over |g| (drawing level, TODO(verify)); 0 falls straight.
    double glide{0.3};
    /// Hz, the tumbling's flutter frequency (drawing level, TODO(verify)).
    double flutterHz{1.5};
    /// rad, the blade's tilt amplitude while it tumbles (drawing level).
    double tiltAmplitude{0.6};
    /// rad/s, the most the yaw turns while it falls (drawn in ±, drawing level).
    double yawRate{2.0};
    /// The share of the blade's velocity at the cut a cut leaf leaves with. TODO(verify).
    double restitution{0.3};
    /// m/s: air over this speed at a crown strips leaves from it. TODO(verify).
    double stripSpeed{10.0};
    /// 1/s: the share of a crown's leaves the air strips per second at (u/u0)² − 1 = 1.
    /// TODO(verify).
    double stripRate{1e-3};
    /// The most particles one step starts (the rest of a big cut is not drawn).
    std::uint32_t maxSpawnPerStep{256};
    /// m/s², the gravity (the site's, 9.80665 standard by default).
    double gravity{9.80665};
};

/// One particle. Plain doubles, so its state round-trips exactly.
struct Leaf {
    std::array<double, 3> position{};  // m, world_local
    std::array<double, 3> velocity{};  // m/s
    double yaw{0.0};                   // rad, the tilt's azimuth
    double phase{0.0};                 // rad, the flutter's
    double size{0.0};                  // m, the blade's length
    double drag{0.0};                  // k = |g| / v_t², 1/m
    double spin{0.0};                  // rad/s, the yaw's rate
    double tree{0.0};                  // the crown it came from (index into treesOf())
    double settled{0.0};               // 1 once on the ground
    double id{0.0};                    // the running count of particles: its random key

    [[nodiscard]] double terminalSpeed(double gravity) const;
    /// The blade's (yaw, pitch) as Leaves.hpp lays them out: flat (pitch π/2) once settled.
    [[nodiscard]] std::array<double, 2> orientation(double tiltAmplitude) const;
};
inline constexpr std::size_t kLeafDoubles = sizeof(Leaf) / sizeof(double);
static_assert(std::is_trivially_copyable_v<Leaf> && kLeafDoubles * sizeof(double) == sizeof(Leaf));

/// A rotor's wake source: its hub, the unit direction its thrust points (the wake goes the other
/// way), its radius and the momentum-theory induced velocity at the disc.
struct RotorWake {
    std::array<double, 3> hub{};
    std::array<double, 3> thrustAxis{};
    double radius{0.0};   // m
    double induced{0.0};  // m/s
};
/// The near wake ends here, in radii below the disc; past it the round jet (TODO(verify)).
inline constexpr double kNearWakeRadii = 4.0;
/// Above the disc the inflow is left out beyond this many radii.
inline constexpr double kInflowRadii = 2.0;
/// The air a rotor's wake moves at `point` (m/s, world).
[[nodiscard]] std::array<double, 3> rotorWake(const RotorWake& rotor,
                                              const std::array<double, 3>& point);

/// Where falling things land: the tops of the world's fixed planes, height fields and upright
/// boxes and cylinders, bucketed on a horizontal grid.
class LandingSurfaces {
public:
    void build(const World& world);
    /// The highest surface under (x, y) at or below z; -inf when there is none.
    [[nodiscard]] double below(double x, double y, double z) const;

private:
    struct Surface {
        ShapeType type{ShapeType::Plane};
        Pose pose{};
        std::array<double, 3> size{};
        std::uint32_t field{0};  // index into fields_
    };
    [[nodiscard]] double top(const Surface& s, double x, double y, bool& over) const;
    [[nodiscard]] static std::uint64_t bucketKey(std::int64_t x, std::int64_t y);
    std::vector<Surface> surfaces_;
    std::vector<std::uint32_t> everywhere_;  // planes, height fields, big boxes
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> buckets_;
    std::vector<HeightField> fields_;  // copies: the world may go
};

/// A cut cell a leaf may fly from: its centre (world) and the blade's velocity there (m/s).
struct CutCell {
    std::array<double, 3> centre{};
    std::array<double, 3> bladeVelocity{};
};

/// The particles.
class LeafField {
public:
    explicit LeafField(LeafParams params = {}) : params_(params) {}
    void setParams(const LeafParams& params) { params_ = params; }
    [[nodiscard]] const LeafParams& params() const { return params_; }
    /// Forgets every particle and carry; `trees` crowns.
    void reset(std::size_t trees);

    /// Leaves cut off tree `tree` from `cells` (kTreeCutCell cubes): cell volume × LAD / leafArea
    /// each, the fraction carried to the next cell, at the cell's centre jittered within it, at
    /// restitution × the blade's velocity plus `air` there. Returns the particles started.
    std::uint32_t spawnFromCut(std::uint32_t tree, const TreeSpec& spec,
                               std::span<const CutCell> cells, std::span<const RotorWake> rotors,
                               const std::array<double, 3>& wind, std::uint64_t seed,
                               std::uint64_t step);
    /// Leaves the air strips from tree `tree` in `dt`: where the air at the crown `air` is faster
    /// than stripSpeed, stripRate × ((|air|/stripSpeed)² − 1) of its remaining leaves per second,
    /// from random points of its surface at sway `sway`. Returns the particles started.
    std::uint32_t strip(std::uint32_t tree, const TreeSpec& spec, double cutVolume,
                        const std::array<double, 3>& air, const std::array<double, 3>& sway,
                        std::span<const RotorWake> rotors, const std::array<double, 3>& wind,
                        double dt, std::uint64_t seed, std::uint64_t step);
    /// Starts one particle at `position` moved by up to ±jitter (m) along each axis (the tests'
    /// way in; spawnFromCut and strip use it). Its terminal speed, size, yaw, flutter phase and
    /// spin are drawn from the counter-based stream (seed, step, its id).
    void add(std::uint32_t tree, const std::array<double, 3>& position,
             const std::array<double, 3>& velocity, double leafArea, double jitter,
             std::uint64_t seed, std::uint64_t step);
    /// Moves every particle in the air by dt; returns whether any moved.
    bool step(double dt, const std::array<double, 3>& wind, std::span<const RotorWake> rotors,
              const LandingSurfaces& ground);

    [[nodiscard]] const std::vector<Leaf>& leaves() const { return leaves_; }
    /// Particles in the air from tree `tree`.
    [[nodiscard]] std::uint32_t flying(std::uint32_t tree) const;
    /// The kLeavesSignal value, version `version`.
    void write(std::vector<double>& out, double version) const;
    /// The state as doubles, and back (false when it does not fit).
    [[nodiscard]] std::vector<double> state() const;
    bool assign(std::span<const double> state);

private:
    LeafParams params_;
    std::vector<Leaf> leaves_;
    std::vector<double> cutCarry_;    // leaves owed per tree, under 1
    std::vector<double> stripCarry_;  // the same for the air's stripping
    std::vector<double> stripped_;    // leaves stripped so far per tree
    double nextId_{0.0};
    std::uint32_t spawnedThisStep_{0};
    std::uint64_t spawnStep_{~std::uint64_t{0}};
};

}  // namespace lagrange::vegetation
