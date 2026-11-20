#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::viewer {

/// How the viewport colours meshes (design 02-scene: the shading menu).
enum class Shading : std::uint8_t {
    Realistic,  ///< Sky and sun with shadows, materials that reflect, water (RealisticPass).
    Lit,        ///< Material colour, one directional light and sky ambient.
    Unlit,      ///< Material colour as it is.
    Thermal,    ///< Temperature on the design's thermal colour map; unlit.
    Current,    ///< Electrical's 3D overlay: meshes tinted by their motor's current; unlit.
    Wireframe,  ///< Triangle edges in the material colour, unlit (when the device can).
    Collision,  ///< The collision shapes the server sends, lit, in place of the visuals.
};

/// What the viewport draws besides the scene's meshes.
/// What the realistic view shows of the year. Drawn only; the weather a season brings to physics
/// and sensors is the site's (`lagrange --season`).
enum class Season : int { Summer = 0, Fall = 1, Winter = 2, Rainy = 3 };

struct RenderOptions {
    bool grid{true};
    bool axes{true};     ///< The frame's X, Y and Z axes at its origin.
    bool sensors{true};  ///< Sensor frustums: each lidar's coverage fan and latest returns.
    /// Radar detections: each radar's latest scan as boxes coloured by radial velocity and sized
    /// by RCS, and its field of view as a faint wedge (RadarScale.hpp).
    bool radar{true};
    /// m/s, the radar colour scale's half span (set by the app; 0 while no detection is drawn).
    float radarVelocitySpan{0.0F};
    bool follow{false};  ///< The camera keeps looking at the robot, or at the selection.

    Shading shading{Shading::Lit};
    bool wireframeAvailable{false};  ///< Set from SceneRenderer::wireframeSupported().
    bool realisticAvailable{false};  ///< Set on a GPU: a software device would crawl.
    Season season{Season::Summer};
    float windSpeed{0.0F};  ///< m/s, the site's: how rough the water is drawn.
    /// The run's weather, drawn in the realistic view: rain (mm/h) and snow (mm/h of water)
    /// falling through the world with the wind (m/s, world x and y, where it blows to), and the
    /// snow on the ground (0-1). With Summer picked, falling weather brings its look: snow lying
    /// or falling the winter's, rain the rainy season's.
    float rainMmPerH{0.0F};
    float snowMmPerH{0.0F};
    float snowCover{0.0F};
    std::array<float, 2> wind{};
    /// Thermal shading: per scene prim, the temperature (K) its meshes take (NaN: none, drawn
    /// dim grey), and the range the colour map spans.
    std::vector<float> temperature;
    float temperatureLow{0.0F};
    float temperatureHigh{1.0F};
    /// Thermal shading of links with a thermal material (RFC 0012): each link's outside cell
    /// faces as triangles in its prim's frame, coloured by the cell's temperature, drawn instead
    /// of the meshes of the prims marked in `heatCovered`.
    struct HeatPatch {
        std::uint32_t prim{0};        ///< whose transform places it
        std::vector<float> vertices;  ///< x, y, z, then linear r, g, b, per vertex; triangles
    };
    std::vector<HeatPatch> heat;
    /// Changed by whoever changes `heat`: the renderer copies the patches to the GPU only then.
    std::uint64_t heatVersion{0};
    std::vector<bool> heatCovered;  ///< per scene prim: drawn by a heat patch
    /// Current shading: per scene prim, its share of the largest current, -1..1, negative when
    /// reversed (NaN: no motor, its own colour dimmed).
    std::vector<float> current;
    /// What the Viewport goes back to after the Electrical overlay took the shading.
    Shading beforeCurrent{Shading::Lit};
    /// Airflow particles (design 07-environment: the Overlays checklist): moved through the
    /// server's aero/wind_speed volume around the robot and drawn as streaks coloured by speed.
    bool airflow{false};
    float airflowMax{0.0F};  ///< m/s, the speed the colour map tops out at (set by the app).
    /// Spray off water (downwash.md §5b): the server's aero/spray volume drawn as falling streaks.
    bool spray{false};
    /// The water's moving surface (water.md "The surface under a rotor"), from the server's
    /// water/surface volume, and the foam where its spray is torn off (aero/spray): the
    /// realistic view lifts and shades the first body of water by them. Null: the still level.
    struct WaterSurface {
        std::uint32_t cells{0};         ///< a side
        double cell{0.0};               ///< m
        std::array<double, 2> first{};  ///< region frame: the first cell's centre, m
        std::vector<float> heights;     ///< m over the still level, row order
        std::vector<float> foam;        ///< 0-1 per cell, or empty
        /// The air's friction velocity on the water (m/s; x for every cell, then y), from the
        /// server's water/friction_velocity volume, or empty.
        std::vector<float> friction;
    };
    std::shared_ptr<const WaterSurface> waterSurface;
    /// The spray's water per cell (aero/spray, kg/m³), drawn by the realistic view as mist.
    struct Spray {
        std::array<std::uint32_t, 3> cells{};  ///< nx, ny, nz
        double cell{0.0};                      ///< m
        std::array<double, 3> first{};         ///< region frame: the first cell's centre, m
        std::vector<float> water;              ///< kg/m³, x fastest
    };
    std::shared_ptr<const Spray> spray3d;
    /// How much the realistic view scales the moving surface's heights and slopes: 1 as the
    /// physics has them (--water-relief; more shows sub-millimetre ripples, a drawing choice).
    float waterRelief{1.0F};
    /// The water's coloured dissolved organic matter (CDOM): its absorption at 440 nm, 1/m
    /// (--water-cdom; 0: pure water). The realistic view absorbs by it as well (water.md).
    float waterCdom{0.0F};
    /// Lamps (RFC 0021 §4), lighting the realistic view: from the scene's LampAPI_1 prims, their
    /// attributes and their lamp/<n>/intensity signals (RFC 0021 §7).
    struct Lamp {
        std::array<double, 3> position{};           ///< region frame, m
        std::array<double, 3> axis{1.0, 0.0, 0.0};  ///< where the beam points, unit
        double intensity{0.0};                      ///< cd on the axis, now
        double beamAngle{0.0};                      ///< rad, full angle to 50 %
        double fieldAngle{0.0};                     ///< rad, full angle to 10 %
        bool point{false};                          ///< the same all round
        double colorTemperature{0.0};               ///< K; 0: color
        std::array<float, 3> color{1.0F, 1.0F, 1.0F};
        bool operator==(const Lamp&) const = default;
    };
    std::vector<Lamp> lamps;
    /// The sky's horizontal illuminance (lx, env/sky_illuminance); NaN: the drawn day.
    double skyIlluminance{std::numeric_limits<double>::quiet_NaN()};
};

/// The year the realistic view draws: the one picked, or with Summer (the default), the weather's:
/// snow falling or lying (a third of the ground or more) the winter's, rain the rainy season's.
[[nodiscard]] inline Season effectiveSeason(const RenderOptions& options) {
    if (options.season != Season::Summer) {
        return options.season;
    }
    constexpr float kLyingSnow = 0.3F;  // of the ground covered
    constexpr float kSomeRain = 0.1F;   // mm/h
    if (options.snowMmPerH > 0.0F || options.snowCover >= kLyingSnow) {
        return Season::Winter;
    }
    return options.rainMmPerH >= kSomeRain ? Season::Rainy : Season::Summer;
}

/// The Current shading's colour at `share` (-1..1), as linear RGB: the electrical
/// colour at 1, regeneration's blue at -1; a motor at rest is faintly tinted.
[[nodiscard]] std::array<float, 3> currentColor(float share);

/// The design's thermal colour map (design tokens colormap.thermal, "inferno") at `t` in
/// [0, 1], as linear RGB for the mesh shader.
[[nodiscard]] std::array<float, 3> thermalColor(float t);

/// A link's cells (thermal/cells: nx × ny × nz scalars, x fastest, NaN outside the solid, the
/// first cell's centre at `origin` in the prim's frame) as the triangles of their outside faces,
/// coloured by temperature between `low` and `high` (K) and a little darker on faces that look
/// down or sideways, so the shape reads without lighting. Appends to `out` (x, y, z, r, g, b per
/// vertex).
void heatFaces(std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
               const std::array<double, 3>& origin, double cell, std::span<const float> values,
               float low, float high, std::vector<float>& out);

}  // namespace lagrange::viewer
