#pragma once

// Vegetation (docs/physics/ground.md "Reeds and shrubs"): reeds, shrubs and tall grass that a
// moving body pushes through. A canopy is a fixed collider whose surface material has a
// canopyHeight (presets `reeds`, `shrub`, `tall_grass`): the box's top face extruded by that
// height, or a slab of it over a plane, as the lidars see it (sensors::canopyVolumeSpan).
//
// Each moving body is cut into cells; a cell inside a canopy is slowed by the stems' drag per
// volume of the canopy it fills, the quadratic drag of an array of emergent stems (Nepf 1999,
// "Drag, turbulence, and diffusion in flow through emergent vegetation", Water Resources
// Research 35(2), 479-489, doi:10.1029/1998WR900069, eq. 1: F/V = 1/2 rho Cd a U^2):
//
//     F = -1/2 rho Cd a V |v| v
//
// with a the frontal area of the stems and leaves per volume, taken as G LAI / h (G = 0.5, the
// projection of a spherical leaf angle distribution, Ross 1981, as the lidars' gap model), V the
// cell's volume in the canopy and v its velocity. Nepf's form is for water flowing past fixed
// stems; here the fluid is the air (the site's air density) unless the run gives an effective
// density (config "density") for the stems' own resistance to bending, which no measurement
// behind this module gives (TODO(verify) against a push-through-vegetation test). At rest there
// is no force.
//
// Cutting (ground.md "Cutting the plants"): a spinning rotor (RotorAPI_1) whose disc reaches into
// a canopy cuts the plants down to where its blades went. The cut is kept per cell of the canopy
// (lagrange/world/CanopyCut.hpp) and published for the lidars; the plants do not grow back. Each
// cut takes the energy of severing the stems on that ground, kCutEnergyPerArea per m² each time a
// cell is cut lower, paid by the rotor as a torque against its spin.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/CanopyCut.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

namespace lagrange::vegetation {

/// G: the leaf projection of a spherical leaf angle distribution (Ross 1981), as the lidars'.
inline constexpr double kLeafProjection = 0.5;
/// Cd of the stems. Nepf 1999 (Fig. 2) reports Cd near 1 for sparse arrays of cylinders at stem
/// Reynolds numbers of hundreds, falling as the array gets denser. TODO(verify) for reeds and
/// shrubs, whose leaves are not cylinders.
inline constexpr double kStemDragCoefficient = 1.0;

/// A canopy volume, in the frame of a fixed collider (z up).
struct Canopy {
    ShapeType type{ShapeType::Box};
    std::array<double, 3> size{};  // the collider's (half extents for a box)
    Pose pose{};                   // region-local
    double height{0.0};            // m
    double leafAreaIndex{0.0};     // m²/m²
    std::uint32_t collider{0};     // index into World::Colliders: the cut map's key
};

/// The energy of cutting the stems standing on 1 m² of ground once (J/m²). Grass stems take of the
/// order of 10-60 mJ per mm² of stem cut (forage-harvester measurements, Persson 1987, "Principles,
/// Design and Testing of Reaper Forage Harvesters", ASAE) and a lawn stands some 4000-10000 mm² of
/// stem per m²: 40-600 J/m². TODO(verify) the value and its spread between grass, reeds and shrubs.
inline constexpr double kCutEnergyPerArea = 100.0;
/// A stub shorter than this is bent aside by a blade rather than cut (m). TODO(verify).
inline constexpr double kMinCutPiece = 0.01;
/// A rotor whose blade tips move slower than this cuts nothing: the plants bend away (m/s).
/// TODO(verify).
inline constexpr double kMinCutTipSpeed = 5.0;

/// The map of cut plants over the canopies (lagrange/world/CanopyCut.hpp): the plants' top over
/// the soil per cell, lowered by sweeps, never raised.
class CutMap {
public:
    /// Sweeps a rotor's disc (centre and unit normal, world; radius) through a canopy: every cell
    /// the disc passes over with its blades under the plants' top is cut down to the blades.
    /// Returns the ground area cut lower by at least kMinCutPiece (m²). Cells over the capacity are
    /// lost.
    double sweep(const Canopy& canopy, const std::array<double, 3>& centre,
                 const std::array<double, 3>& normal, double radius);
    /// The plants' top over the soil at (x, y) in the canopy collider's frame; +inf where not cut.
    [[nodiscard]] double top(std::uint32_t collider, double x, double y) const;
    /// The signal's value (kCanopyCutLength doubles).
    [[nodiscard]] const std::vector<double>& values() const { return values_; }
    /// Puts back a value from values() (saved state); false when it does not fit.
    bool assign(std::span<const double> values);
    [[nodiscard]] std::size_t cells() const { return index_.size(); }
    [[nodiscard]] bool full() const { return index_.size() >= kCanopyCutCapacity; }

private:
    std::vector<double> values_ = std::vector<double>(kCanopyCutLength, 0.0);
    std::unordered_map<std::uint64_t, std::uint32_t> index_;  // key -> slot
};

/// The canopies of a world: fixed boxes and planes with a canopy height over 0.
[[nodiscard]] std::vector<Canopy> canopiesOf(const World& world);

/// Whether a point (world) lies in the canopy volume, where the plants stand (none above a cut).
[[nodiscard]] bool insideCanopy(const Canopy& canopy, const std::array<double, 3>& point,
                                const CutMap* cuts = nullptr);

/// a, the frontal area per volume: G LAI / h (1/m); 0 without a canopy.
[[nodiscard]] double frontalAreaDensity(double height, double leafAreaIndex);

/// The stems' drag on a volume V moving at v through them (N): -1/2 rho Cd a V |v| v.
[[nodiscard]] std::array<double, 3> stemDrag(double density, double dragCoefficient,
                                             double frontalArea, double volume,
                                             const std::array<double, 3>& velocity);

/// Writes body/<b>/wrench/vegetation (world_local, about the centre of mass) and
/// body/<b>/in_vegetation (the share of its colliders' volume inside a canopy) for every moving,
/// non-kinematic body with colliders, at 1 ms. Config: "density" (kg/m³; the site's air density
/// when the server has one, else 0: no drag), "drag_coefficient" (default kStemDragCoefficient),
/// "cut_energy" (J/m², default kCutEnergyPerArea; 0 cuts without load), "cutting" (default true).
///
/// With rotors (RotorAPI_1 joints) and cutting on, it also writes vegetation/canopy_cut (the cut
/// map), rotor/<j>/cutting_power (W, the power the cuts take) and body/<link>/wrench/cutting (the
/// torque that takes it, against the spin, on the propeller's link).
class Vegetation final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    [[nodiscard]] const CutMap& cuts() const { return cuts_; }

private:
    struct Cell {
        std::array<double, 3> at;  // in the body's frame
        double volume;             // m³
    };
    struct Body {
        SignalId pose, twist, wrench, inside;
        std::array<double, 3> com{};
        std::vector<Cell> cells;
        double volume{0.0};
    };
    struct Rotor {
        SignalId pose, velocity, wrench, power;
        std::array<double, 3> axis{};  // the joint axis in the link's frame
        double radius{0.0};            // m
        double inertia{0.0};           // kg·m² of the propeller's link about the axis
        double owed{0.0};              // J of cutting not yet paid by the rotor
    };
    std::vector<Canopy> canopies_;
    std::vector<Body> bodies_;
    std::vector<Rotor> rotors_;
    CutMap cuts_;
    SignalId cutOut_;
    double cutEnergy_{kCutEnergyPerArea};
    double density_{0.0};
    double dragCoefficient_{kStemDragCoefficient};
};

}  // namespace lagrange::vegetation
