#pragma once

// A patch of a water surface that moves (docs/physics/water.md "The surface under a rotor"):
// small, linear capillary-gravity waves driven by the air's pressure and shear on the surface,
// on a square grid that can follow a drone across a bigger body of water.

#include "lagrange/hydrodynamics/Water.hpp"
#include "lagrange/module/IModule.hpp"

#include <array>
#include <complex>
#include <cstddef>
#include <span>
#include <vector>

namespace lagrange::hydrodynamics {

/// Water's properties at 20 °C, for the surface.
/// Surface tension against air (Vargaftik, Volkov and Voljak, "International tables of the
/// surface tension of water", J. Phys. Chem. Ref. Data 12(3), 1983: 72.75 mN/m at 20 °C).
inline constexpr double kSurfaceTension = 0.07275;  // N/m
/// Kinematic viscosity (1.002 mPa·s over 998.2 kg/m³ at 20 °C; TODO(verify) the table cited).
inline constexpr double kWaterViscosity = 1.004e-6;  // m²/s

struct SurfaceParams {
    int cells{128};          // a side, a power of two
    double cell{0.03};       // m
    double depth{2.0};       // m, the water's
    double density{1000.0};  // kg/m³
    double gravity{9.80665};
    double surfaceTension{kSurfaceTension};
    double viscosity{kWaterViscosity};
    /// The outer band (a share of the side) where outgoing waves are absorbed: the water's
    /// velocity is damped there, at most kSpongeRate, so they leave the patch and do not wrap.
    double spongeShare{0.125};
    double spongeRate{6.0};  // 1/s
};

/// Linear waves on water of depth d (Lamb, Hydrodynamics, 6th ed., 1932, §267 and §348):
/// ω² = (g·k + σ·k³/ρ)·tanh(k·d), each amplitude decaying at γ = 2·ν·k² (viscous, for a clean
/// surface, §348). Each Fourier mode η_k of the surface obeys
///   η̈_k + 2·γ·η̇_k + ω²·η_k = −(k·tanh(k·d)/ρ)·p̂_k − (tanh(k·d)/(k·d))·i·(k·τ̂_k)/ρ,
/// the linearised potential-flow response to a surface pressure p (its steady answer is
/// η_k = −p̂_k/(ρ·g + σ·k²): for anything wider than a few centimetres, the hydrostatic dimple
/// −p/(ρ·g)) and to a tangential stress τ taken as a force spread over the depth (the shallow
/// water response, ∂²η/∂t² = g·d·∇²η − ∇·τ/ρ, with the pressure term's depth factor; a modelling
/// choice: its set-up is under 1 % of the dimple a rotor makes). The modes are integrated
/// exactly over each step, the forcing held over it: no stability limit on the step.
///
/// The patch is periodic in the transform; its outer band damps the water's velocity, so waves
/// leaving the middle are absorbed there and do not come back round. Cells marked dry (outside
/// the body of water) are damped as hard: the shore.
class SurfacePatch {
public:
    explicit SurfacePatch(const SurfaceParams& params);

    [[nodiscard]] const SurfaceParams& params() const { return p_; }
    [[nodiscard]] int cells() const { return p_.cells; }
    /// The patch's lower corner (m, world x and y) and its side.
    [[nodiscard]] std::array<double, 2> origin() const { return origin_; }
    [[nodiscard]] double side() const { return p_.cells * p_.cell; }

    /// Puts the patch's lower corner at `origin`, rounded to whole cells; the surface moves with
    /// the world under it (shifted by whole cells), and the cells that come in are still.
    void moveTo(const std::array<double, 2>& origin);
    /// Marks the cells that are water (x, y inside [lo, hi]); the rest is shore.
    void setWater(const std::array<double, 2>& lo, const std::array<double, 2>& hi);

    /// Advances by `dt` (s) under the pressure (Pa) and the stress (Pa, x then y), one value
    /// per cell in row order (y rows of x), held over the step. Empty spans: no load.
    void step(double dt, std::span<const double> pressure, std::span<const double> stressX = {},
              std::span<const double> stressY = {});

    /// The surface's height over the still level (m), per cell in row order, and its rate.
    [[nodiscard]] std::span<const double> heights() const { return eta_; }
    [[nodiscard]] std::span<const double> rates() const { return rate_; }
    /// The height at a world point (bilinear between cell centres); 0 outside the patch.
    [[nodiscard]] double heightAt(double x, double y) const;
    /// Back to still and flat.
    void reset();
    /// The state for a save: the corner, the heights, the rates.
    void setState(const std::array<double, 2>& origin, std::span<const double> heights,
                  std::span<const double> rates);

    /// The dispersion relation of the class comment: ω (rad/s) at wavenumber k (rad/m).
    [[nodiscard]] static double omega(double k, double gravity, double tension, double density,
                                      double depth);

private:
    SurfaceParams p_;
    std::array<double, 2> origin_{};
    std::vector<double> eta_, rate_;
    std::vector<double> damping_;  // 1/s, per cell: the sponge and the shore
    std::vector<bool> wet_;
    std::array<double, 2> waterLo_{-1e300, -1e300}, waterHi_{1e300, 1e300};
    // Per mode: |k|, ω, γ, the pressure and stress gains.
    std::vector<double> k_, kx_, ky_, omega_, gamma_, pressureGain_, stressGain_;
    std::vector<std::complex<double>> a_, b_, f_, g_;  // work: η̂, η̇̂, forcing
    void buildDamping();
};

/// The height (m) at world (x, y) of a surface grid of `cells` a side, `cell` m, its lower corner
/// at `origin`, heights in row order (bilinear between cell centres); 0 outside it or without one.
[[nodiscard]] double gridHeight(std::span<const double> heights, int cells, double cell,
                                const std::array<double, 2>& origin, double x, double y);

/// The water's moving surface as a module (domain hydrodynamics, at config "surface_step", s,
/// default 0.04): a SurfacePatch per body of water (config "surface_cells", a power of two,
/// default 128, and "surface_cell", m, default 0.03), forced by what the air does on it.
///
/// Per body of water w it reads water/<w>/patch (m, 2: the lower corner the patch should have,
/// NaN to stay), water/<w>/air_pressure (Pa, one per cell) and water/<w>/air_stress (Pa, x for
/// every cell then y), all from the aerodynamics domain's downwash (feedback, optional: without
/// them the surface stays still), and writes water/<w>/surface (m over the still level, one per
/// cell), water/<w>/surface_origin (m, 2: the grid's lower corner), and water/<w>/surface_min and
/// surface_max (m). It starts flat and still, centred on the water.
class SurfaceWaves final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// The patch of body of water `w`, after declare.
    [[nodiscard]] const SurfacePatch& patch(std::size_t w) const { return pools_.at(w).patch; }

private:
    struct Pool {
        WaterBody water;
        SurfacePatch patch;
        SignalId patchIn, pressureIn, stressIn;
        SignalId surfaceOut, originOut, minOut, maxOut;
        bool quiet{true};  // flat, still and unloaded: nothing to step
    };
    void write(Pool& pool, SignalBus& bus) const;
    std::vector<Pool> pools_;
    Duration step_{40'000'000};
};

/// In-place complex FFT of a square n × n grid in row order (n a power of two); `inverse`
/// divides by n².
void fft2(std::vector<std::complex<double>>& data, int n, bool inverse);

}  // namespace lagrange::hydrodynamics
