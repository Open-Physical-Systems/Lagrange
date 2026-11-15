#pragma once

// How the viewer draws a radar's detections (design 06-sensors, sensor views; the viewport's
// sensor overlays): colour by radial velocity on a diverging scale, size by radar cross-section.
// Pure functions, no ImGui: the viewport, the Radar panel and the tests share them.
//
// A radar frame (SensorFrame points with fields x, y, z, velocity, rcs, snr) carries per
// detection its position in the sensor frame (m), its radial velocity (m/s, positive receding,
// as sensor/<n>/detections), its radar cross-section (m²) and its signal-to-noise ratio (linear).

#include <array>
#include <cstdint>

namespace lagrange::viewer {

/// Floats per radar detection in a PointCloud: x, y, z, velocity, rcs, snr.
inline constexpr std::uint32_t kRadarFloats = 6;

/// The colour of radial velocity `velocity` (m/s) on a scale of ±`span`: the design tokens'
/// diverging colour map (colormap.error) from approaching (#2F6FED, -span) to receding
/// (#E8772E, +span), clamped beyond. Its middle is the token t3 (#8190A6) rather than the map's
/// #16243A: a target at rest must stay visible on the dark viewport and panels. sRGB, alpha 1.
/// A non-finite velocity or span ≤ 0 gives the middle.
[[nodiscard]] std::array<float, 4> radarVelocityColour(float velocity, float span);

/// The index of `velocity`'s band when ±`span` is cut into `bins` equal bands (the viewport
/// draws one batch of lines per band): 0 most approaching, bins − 1 most receding, clamped.
/// The middle band holds 0 when `bins` is odd.
[[nodiscard]] int radarVelocityBin(float velocity, float span, int bins);

/// The velocity at the centre of band `bin` of `bins` over ±`span` (what the band is drawn in).
[[nodiscard]] float radarBinVelocity(int bin, int bins, float span);

/// The colour scale's half span for a frame whose fastest detection moves `fastest` m/s
/// (absolute): rounded up to 1, 2 or 5 × 10ⁿ, never under kRadarMinimumSpan, so the scale
/// changes only now and then and a still scene does not paint noise in full colour.
inline constexpr float kRadarMinimumSpan = 2.0F;  // m/s
[[nodiscard]] float radarVelocitySpan(float fastest);

/// Where radar cross-section `rcs` (m²) lies on the size scale, 0..1: linear in dBsm from
/// −20 dBsm (0.01 m², a person's order at 77 GHz) to +20 dBsm (100 m², a car's strongest
/// returns), clamped; non-finite or ≤ 0 gives 0. TODO(verify): typical 77 GHz RCS of a
/// pedestrian and a car against a published measurement.
[[nodiscard]] float radarRcsFraction(float rcs);

}  // namespace lagrange::viewer
