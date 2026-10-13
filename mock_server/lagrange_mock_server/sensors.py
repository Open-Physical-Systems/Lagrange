"""Synthetic sensor frames for the sample scene.

A real server ray-traces the stage. The mock server has no renderer, so it scans a small
fixed surrounding (an aisle with racks and a pallet) that exists only here. The frames are
deterministic for a given pose and time, and they are marked as synthetic in the console.
"""

from __future__ import annotations

import numpy as np

LIDAR_RINGS = 4  # channels, stacked vertically
LIDAR_BEAMS = 720  # per revolution and ring: 0.5 degree steps
LIDAR_MAX_RANGE = 25.0  # m
LIDAR_RANGE_NOISE = 0.02  # m, one sigma
LIDAR_DROPOUT = 0.004  # fraction of returns lost
LIDAR_RATE_HZ = 10.0
POINT_FIELDS = ("x", "y", "z", "intensity")
POINT_STRIDE = 16  # four float32

# The surrounding, as wall segments in the region frame: (x0, y0, x1, y1, reflectivity).
_SEGMENTS = np.array(
    [
        (-9.0, -4.5, 9.0, -4.5, 0.55),  # long walls
        (-9.0, 4.5, 9.0, 4.5, 0.55),
        (-9.0, -4.5, -9.0, 4.5, 0.55),  # end walls
        (9.0, -4.5, 9.0, 4.5, 0.55),
        (-6.0, 3.0, 6.0, 3.0, 0.85),  # rack faces
        (-6.0, -3.0, 6.0, -3.0, 0.85),
        (3.2, 0.9, 4.0, 0.9, 0.30),  # a pallet
        (4.0, 0.9, 4.0, 1.7, 0.30),
        (4.0, 1.7, 3.2, 1.7, 0.30),
        (3.2, 1.7, 3.2, 0.9, 0.30),
    ],
    dtype=np.float64,
)


def lidar_scan(x: float, y: float, yaw: float, time_ns: int) -> np.ndarray:
    """One revolution seen from (x, y, yaw). Returns float32 [n, 4]: x, y, z, intensity.

    Points are in the sensor frame (x forward, z up). Beams that hit nothing in range, and a
    small random fraction, give no point.
    """
    rng = np.random.default_rng(time_ns & 0xFFFFFFFF)
    angles = np.linspace(-np.pi, np.pi, LIDAR_BEAMS, endpoint=False)
    world = angles + yaw
    dx, dy = np.cos(world), np.sin(world)

    # Ray against every segment: origin + t d = a + u (b - a).
    ax, ay, bx, by, reflectivity = _SEGMENTS.T
    ex, ey = bx - ax, by - ay
    denominator = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((ax - x)[None, :] * ey[None, :] - (ay - y)[None, :] * ex[None, :]) / denominator
        u = ((ax - x)[None, :] * dy[:, None] - (ay - y)[None, :] * dx[:, None]) / denominator
    hit = (np.abs(denominator) > 1e-12) & (t > 0.05) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    nearest = np.argmin(t, axis=1)
    distance = t[np.arange(LIDAR_BEAMS), nearest]
    surface = reflectivity[nearest]

    rings = []
    for ring in range(LIDAR_RINGS):
        elevation = np.radians(-3.0 + 2.0 * ring)
        slant = distance / np.cos(elevation)
        slant = slant + rng.normal(0.0, LIDAR_RANGE_NOISE, LIDAR_BEAMS)
        keep = (slant < LIDAR_MAX_RANGE) & (rng.random(LIDAR_BEAMS) > LIDAR_DROPOUT)
        # Returned power falls with the square of the range.
        intensity = surface / (1.0 + 0.02 * slant * slant)
        points = np.stack(
            [
                slant * np.cos(elevation) * np.cos(angles),
                slant * np.cos(elevation) * np.sin(angles),
                slant * np.sin(elevation),
                intensity,
            ],
            axis=1,
        )
        rings.append(points[keep])
    return np.concatenate(rings).astype(np.float32)
