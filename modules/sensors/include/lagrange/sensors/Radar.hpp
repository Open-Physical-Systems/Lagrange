#pragma once

// An automotive radar by ray launching (domains/sensors.md): rays over the field of view, their
// echoes grouped into a detection per object and resolution cell, kept when the radar equation's
// signal-to-noise ratio beats the threshold.

#include "lagrange/module/IModule.hpp"
#include "lagrange/world/World.hpp"

#include <memory>

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
/// beyond the count) and sensor/<n>/count. CPU on every tier.
[[nodiscard]] std::unique_ptr<IModule> createRadar(const World& world, std::uint32_t sensor,
                                                   bool groundTruth);

}  // namespace lagrange::sensors
