#pragma once

// Water (docs/design/domains/hydrodynamics.md §9): a box of still water. Buoyancy and quadratic
// drag on whatever moving bodies enter it, from cells sampled in their colliders. Not Fossen's
// manoeuvring model: no added mass and no waves.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/Model.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lagrange::hydrodynamics {

struct WaterBody {
    std::string name{"water"};
    std::array<double, 3> surface{};              // the middle of the surface, world ENU
    double sizeX{10.0}, sizeY{10.0}, depth{2.0};  // m
    double density{1000.0};                       // kg/m³
    double drag{1.0};                             // 1, illustrative
    double reflectivity{0.02};                    // 1
};

/// The WaterAPI_1 prims of `model`, out of its schema instances (addWater puts them back).
[[nodiscard]] std::vector<WaterBody> takeWater(Model& model);

/// Adds a body of water: a drawn surface and a WaterAPI_1 instance that the module and the lidars
/// read. It has no collider: nothing stands on water, and a robot on the ground under it stays on
/// the ground.
void addWater(Model& model, const WaterBody& water);

/// The water bodies a world holds (for the module, and for the lidars' surface).
[[nodiscard]] std::vector<WaterBody> waterOf(const WorldView& world);

/// Writes body/<b>/wrench/water (world_local, about the centre of mass) for every moving,
/// non-kinematic body with colliders, and body/<b>/submerged (the fraction of its colliders'
/// volume under water), at 1 ms: buoyancy ρ g V at each submerged cell, and drag
/// ½ ρ Cd A |v| v from the cell's velocity. Each collider is cut into cells (kCells a side, kept
/// when their centre is inside it, scaled to the shape's true volume).
class Water final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

private:
    struct Cell {
        std::array<double, 3> at;  // in the body's frame
        double volume;             // m³
        double area{0.0};          // m², its share of the frontal area, all under water
        double height;             // m, of the cell, for how much of it is under the surface
    };
    struct Body {
        SignalId pose, twist, wrench, submerged;
        std::array<double, 3> com{};
        std::vector<Cell> cells;
        double volume{0.0};
        double reach{0.0};  // m, of its farthest cell from its origin
    };
    std::vector<WaterBody> waters_;
    std::vector<Body> bodies_;
    std::array<double, 3> gravity_{};
};

}  // namespace lagrange::hydrodynamics
