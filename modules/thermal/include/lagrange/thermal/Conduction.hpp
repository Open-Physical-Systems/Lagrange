#pragma once

#include "lagrange/thermal/Lumped.hpp"
#include "lagrange/thermal/Materials.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace lagrange::thermal {

/// Air near 300 K and 1 atm (Incropera et al., Fundamentals of Heat and Mass Transfer, 7th ed.,
/// Table A.4): what the natural and forced convection correlations use.
inline constexpr double kAirKinematicViscosity = 15.89e-6;  // m²/s
inline constexpr double kAirConductivity = 26.3e-3;         // W/(m·K)
inline constexpr double kAirDiffusivity = 22.5e-6;          // m²/s
inline constexpr double kAirPrandtl = 0.707;                // 1
inline constexpr double kStandardGravity = 9.80665;         // m/s², CGPM 1901

/// Natural convection coefficient (W/(m²·K)) of a flat surface at `surface` K in still air at
/// `air` K, whose outward normal has world z component `normalZ`: vertical faces by Churchill
/// and Chu (all Rayleigh numbers, Incropera Eq. 9.26) on `height`; horizontal ones by the
/// McAdams / Lloyd–Moran correlations (Incropera Eqs. 9.30–9.32) on `horizontalLength`
/// (area / perimeter): upper surface of a hot plate (or lower of a cold one)
/// 0.54 Ra^1/4 (Ra ≤ 1e7), 0.15 Ra^1/3 above; the other side 0.52 Ra^1/5. Inclined faces blend
/// the vertical and horizontal coefficients by |normalZ|.
[[nodiscard]] double naturalConvection(double surface, double air, double normalZ, double height,
                                       double horizontalLength);

/// Forced convection coefficient (W/(m²·K)) of a flat plate of length `length` in air at
/// `speed` m/s, averaged over it: laminar 0.664 Re^1/2 Pr^1/3 below Re 5e5, mixed
/// (0.037 Re^4/5 − 871) Pr^1/3 above (Incropera Eqs. 7.30 and 7.38).
[[nodiscard]] double forcedConvection(double speed, double length);

/// A collision shape of a part, in its body's frame.
struct SolidShape {
    ShapeType type{ShapeType::Box};
    std::array<double, 3> size{0.0, 0.0, 0.0};  // as World::Colliders::size
    Pose pose;                                  // in the body frame
    std::vector<std::array<double, 3>> points;  // a hull's vertices, shape frame
    std::vector<std::array<double, 4>> faces;   // a hull's planes, n·p ≤ d inside
};

/// A box of a part's solid that a heat source heats.
struct HeatBox {
    std::array<double, 3> centre{0.0, 0.0, 0.0};  // m, body frame
    std::array<double, 3> size{0.0, 0.0, 0.0};    // m, full size
};

/// One link with a thermal material (RFC 0012): its shapes, material and heat sources.
struct SolidPart {
    std::string name;  // the link's signal name
    Pose rest;         // its body's world pose with every joint at zero
    std::vector<SolidShape> shapes;
    ThermalMaterial material;
    std::array<double, 3> axis{0.0, 0.0, 1.0};  // unit, body frame: conductivityAlong's
    std::optional<double> contactConductance;   // W/(m²·K), to its parent part
    std::optional<std::size_t> parent;          // index of the nearest ancestor part
    double initialTemperature{0.0};             // K
    double cell{0.002};                         // m, the largest cell wanted
    std::vector<HeatBox> sources;
};

/// What a part receives each step.
struct PartInput {
    Pose pose;             // its body's world pose now (for the faces' orientation)
    double airspeed{0.0};  // m/s
    double heat{0.0};      // W spread over all its cells (loss_power signals)
};

/// Heat conduction in the solids of a robot (RFC 0012, docs/physics/heat-conduction.md): every
/// part cut into cubic cells in its body's frame, finite volumes between neighbours (orthotropic
/// k), contact conductance between a part and its parent where their cells touch, convection and
/// radiation from every face that touches neither, heat sources in boxes. Backward Euler, solved
/// by Jacobi-preconditioned conjugate gradients from the last step's temperatures; the same
/// operations in the same order every step, so a run is bit-reproducible (invariant 7).
class ConductionModel {
public:
    struct Grid {
        std::array<double, 3> origin{0.0, 0.0, 0.0};  // m, body frame: the first cell's corner
        double cell{0.0};                             // m
        std::array<int, 3> dims{0, 0, 0};
        std::vector<std::int32_t> cellAt;  // x fastest; −1 outside the solid, else cell index
        std::size_t first{0}, count{0};    // this part's cells in the model's arrays
    };

    ConductionModel() = default;

    /// Cuts the parts into cells: at most `cell` (each part's own), at most half the thinnest
    /// shape, and at most `maxCellsPerPart` cells per part (coarser when it would be more).
    /// Notes what it changed or found (touching parts without a contact conductance). An error
    /// for a part with no cells.
    [[nodiscard]] static std::optional<ConductionModel> build(std::vector<SolidPart> parts,
                                                              std::size_t maxCellsPerPart,
                                                              std::vector<std::string>& notes,
                                                              std::string& error);

    [[nodiscard]] std::size_t cellCount() const { return capacity_.size(); }
    [[nodiscard]] std::span<const SolidPart> parts() const { return parts_; }
    [[nodiscard]] const Grid& grid(std::size_t part) const { return grids_.at(part); }
    /// A cell's centre in its part's body frame.
    [[nodiscard]] std::array<double, 3> centre(std::size_t cell) const;
    [[nodiscard]] std::span<const std::size_t> sourceCells(std::size_t part,
                                                           std::size_t source) const;

    /// Starting temperatures, each part at its initial temperature.
    [[nodiscard]] std::vector<double> initialTemperatures() const;

    /// One backward Euler step of `dt` s. `sourceHeat` holds each part's sources' heat in order
    /// (W, after the light that left). `fixedCoefficient` replaces the correlations by one film
    /// coefficient (W/(m²·K)) for every face: for checks against closed-form solutions.
    void step(std::span<double> temperatures, std::span<const PartInput> inputs,
              std::span<const double> sourceHeat, const Surroundings& around, double dt,
              std::optional<double> fixedCoefficient = std::nullopt);

    /// Over the last step: each part's heat to the air and the sky, W.
    [[nodiscard]] double heatOut(std::size_t part) const { return heatOut_.at(part); }
    [[nodiscard]] double mean(std::span<const double> t, std::size_t part) const;
    [[nodiscard]] double maximum(std::span<const double> t, std::size_t part) const;
    [[nodiscard]] double minimum(std::span<const double> t, std::size_t part) const;
    [[nodiscard]] double sourceMean(std::span<const double> t, std::size_t part,
                                    std::size_t source) const;
    /// Conjugate gradient iterations of the last step.
    [[nodiscard]] int iterations() const { return iterations_; }

private:
    struct Link {
        std::uint32_t a, b;
        double conductance;  // W/K
    };
    struct Face {
        std::uint32_t cell;
        std::uint8_t direction;  // 0..5: −x, +x, −y, +y, −z, +z of the body frame
        double area;             // m²
    };

    std::vector<SolidPart> parts_;
    std::vector<Grid> grids_;
    std::vector<std::uint32_t> partOf_;           // per cell
    std::vector<std::array<double, 3>> centres_;  // per cell, body frame
    std::vector<double> capacity_;                // J/K per cell
    std::vector<Link> links_;
    std::vector<Face> faces_;
    std::vector<std::size_t> facesFirst_;  // per part, into faces_ (faces are grouped by part)
    std::vector<std::vector<std::vector<std::size_t>>> sourceCells_;  // [part][source]
    std::vector<double> heatOut_;
    int iterations_{0};
    // Work arrays, kept between steps.
    std::vector<double> diagonal_, rhs_, residual_, direction_, product_, preconditioned_;
    std::vector<double> faceCoefficient_, faceSky_;
};

}  // namespace lagrange::thermal
