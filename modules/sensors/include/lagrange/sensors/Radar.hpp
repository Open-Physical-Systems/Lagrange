#pragma once

// An automotive radar by ray launching (domains/sensors.md): rays over the field of view, their
// echoes grouped into a detection per object and resolution cell, kept when the radar equation's
// signal-to-noise ratio beats the threshold.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <memory>

namespace lagrange::gpu {
class RayContext;
}

namespace lagrange::sensors {

/// The radar equation's signal-to-noise ratio for a target of cross-section `rcs` (m²) at
/// `range` (m): P·G²·λ²·σ / ((4π)³·R⁴·N), with λ = c / f.
[[nodiscard]] double radarSnr(double txPower, double gain, double frequency, double rcs,
                              double range, double noiseFloor);

/// One detection, as sensor/<n>/detections carries it (6 numbers): range (m), azimuth and
/// elevation (rad, in the sensor frame, x forward, z up), radial velocity (m/s, positive
/// receding), radar cross-section (m²) and signal-to-noise ratio (linear).
inline constexpr std::size_t kRadarFields = 6;
inline constexpr std::size_t kRadarMaxDetections = 64;

/// A radar module for sensor `sensor` of `world` (RadarAPI_1 settings: max_range, azimuth_fov,
/// elevation_fov, azimuth_resolution, range_resolution, velocity_resolution, tx_power,
/// antenna_gain, noise_floor, detection_threshold, carrier_frequency).
///
/// Each ray's echo is σ = ρ·r²·ΔΩ·cos θ: a Lambertian back-scatterer of the hit surface's
/// sensmat:radarCrossSectionDensity ρ (SensorMaterialAPI_1 on the body, 0.1 without one)
/// illuminated over the ray's solid angle ΔΩ at incidence θ (from the surface normal), so
/// surfaces at grazing angles return little. Hits on the ground plane are dropped as clutter (a
/// real radar's clutter map removes the road surface; not simulated). Echoes of one object in one
/// range cell add up to one detection (a clustered detection, as automotive radars report; its
/// angles are the echoes' power-weighted mean); its radial velocity is the relative velocity of the
/// hit points along the rays, power weighted. Range, angle and velocity get Gaussian noise of
/// resolution / √(2·SNR). A fixed threshold over the noise floor stands in for CFAR; no multipath,
/// ghosts or raw ADC data. Writes sensor/<n>/detections (kRadarMaxDetections × kRadarFields, NaN
/// beyond the count) and sensor/<n>/count. With `rayContext` the rays are cast on the GPU (the
/// lidar's kernel, gpu/kernels/lidar/bvh_traverse.slang; float ranges, so not bit-reproducible,
/// Tier 1); without, on the CPU in parallel, the deterministic reference.
[[nodiscard]] std::unique_ptr<IModule> createRadar(
    const World& world, std::uint32_t sensor, bool groundTruth,
    std::shared_ptr<gpu::RayContext> rayContext = nullptr);

/// A 4D imaging radar (kind `imaging_radar`, sensors.md §2 "4D imaging radar"): the same rays and
/// echoes as createRadar, resolved by the radar's own cells instead of by object. The settings
/// above (same names) plus, all optional:
/// - `azimuth_elements`, `elevation_elements`: virtual array channels (N_TX × N_RX laid out in
///   azimuth and elevation); with `element_spacing` d (wavelengths, 0.5) each angle cell is
///   1/(N·d) wide in direction sine (u = cos el · sin az, w = sin el): λ/(N·d) rad at boresight.
///   Without them, `azimuth_resolution` and `elevation_resolution` (rad, at boresight).
/// - `bandwidth` (Hz): range_resolution = c/(2B); `chirp_period` T_c (s): the unambiguous
///   velocity ±λ/(4·T_c); with `chirps` N_c: velocity_resolution = λ/(2·N_c·T_c).
/// - `max_velocity` (m/s): the unambiguous velocity given directly; radial velocities fold into
///   [−v_max, v_max). 0 or absent: no folding.
/// - `unambiguous_range` (m): ranges beyond it fold back (r mod R_u); absent: they do not.
/// - `ghost_reflection`: the ground's power reflection coefficient |Γ|² (0..1, 0: no ghosts);
///   each echo also returns by the ground both ways (the target's mirror image below the ground,
///   power × |Γ|⁴).
/// - `peak_grouping` (1): each cell climbs to its strongest neighbour (±1 cell in the four
///   dimensions) and a local peak takes in the cells that climb to it; 0: every cell is its own.
/// - `max_detections` (256), `ray_spacing` (rad; half the finer angle cell at boresight).
/// Every echo falls in a cell of (range, Doppler, azimuth sine, elevation sine); a peak whose
/// summed Σσ/R⁴ beats the threshold by the radar equation is one detection (signal-weighted
/// means, σ summed), so two targets nearer than a cell merge and one large object gives several
/// detections. Writes
/// sensor/<n>/detections (max_detections × kRadarFields, NaN beyond the count; velocity as
/// measured, folded) and sensor/<n>/count.
[[nodiscard]] std::unique_ptr<IModule> createImagingRadar(
    const World& world, std::uint32_t sensor, bool groundTruth,
    std::shared_ptr<gpu::RayContext> rayContext = nullptr);

/// The width of a uniform virtual array's angle cell in direction sine, 1/(N·d) for N elements
/// d wavelengths apart: the Rayleigh resolution λ/(N·d) at boresight (S. Rao, "Introduction
/// to mmWave Sensing: FMCW Radars", Texas Instruments mmWave training, 2017: angle resolution
/// θ_res = λ/(N·d·cos θ); TODO(verify): its TI literature number).
[[nodiscard]] double arrayAngleCell(double elements, double spacingWavelengths);

/// A radial velocity `v` (m/s) as a radar of unambiguous velocity ±`maxVelocity` measures it:
/// folded into [−maxVelocity, maxVelocity). `maxVelocity` ≤ 0: unchanged.
[[nodiscard]] double foldVelocity(double v, double maxVelocity);

}  // namespace lagrange::sensors
