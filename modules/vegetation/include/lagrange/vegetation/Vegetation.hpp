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

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <array>
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
};

/// The canopies of a world: fixed boxes and planes with a canopy height over 0.
[[nodiscard]] std::vector<Canopy> canopiesOf(const World& world);

/// Whether a point (world) lies in the canopy volume.
[[nodiscard]] bool insideCanopy(const Canopy& canopy, const std::array<double, 3>& point);

/// a, the frontal area per volume: G LAI / h (1/m); 0 without a canopy.
[[nodiscard]] double frontalAreaDensity(double height, double leafAreaIndex);

/// The stems' drag on a volume V moving at v through them (N): -1/2 rho Cd a V |v| v.
[[nodiscard]] std::array<double, 3> stemDrag(double density, double dragCoefficient,
                                             double frontalArea, double volume,
                                             const std::array<double, 3>& velocity);

/// Writes body/<b>/wrench/vegetation (world_local, about the centre of mass) and
/// body/<b>/in_vegetation (the share of its colliders' volume inside a canopy) for every moving,
/// non-kinematic body with colliders, at 1 ms. Config: "density" (kg/m³; the site's air density
/// when the server has one, else 0: no drag), "drag_coefficient" (default kStemDragCoefficient).
class Vegetation final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter&) const override { return {}; }  // stateless
    Status restoreState(StateReader&) override { return {}; }

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
    std::vector<Canopy> canopies_;
    std::vector<Body> bodies_;
    double density_{0.0};
    double dragCoefficient_{kStemDragCoefficient};
};

}  // namespace lagrange::vegetation
