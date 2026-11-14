#pragma once

#include "lagrange/fields/Field.hpp"
#include "lagrange/module/IModule.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::weather {

// --- The atmosphere (domains/weather.md §2) -------------------------------------------------

/// Saturation vapour pressure over water (Pa) at temperature `kelvin`: the Magnus form with the
/// coefficients of Alduchov and Eskridge (1996, J. Appl. Meteor. 35, 601–609, their AERK),
/// e_s = 610.94·exp(17.625·t / (t + 243.04)) with t in °C; within 0.4 % of the reference values
/// from −40 to 50 °C.
[[nodiscard]] double saturationVapourPressure(double kelvin);

/// Density of moist air (kg/m³): dry air and water vapour as ideal gases at partial pressures
/// p − e and e, e = relativeHumidity·e_s(T). R_d = 287.05 and R_v = 461.5 J/(kg·K).
[[nodiscard]] double moistAirDensity(double kelvin, double pressurePa, double relativeHumidity);

// --- Extinction (1/m) ------------------------------------------------------------------------

/// Fog: extinction at 550 nm from the visibility (m) by Koschmieder's law with a 2 % contrast
/// threshold, σ = −ln(0.02) / V = 3.912 / V.
[[nodiscard]] double fogExtinctionVisible(double visibilityM);

/// The Kim model's size-distribution exponent q for visibility `visibilityM` (Kim, McArthur and
/// Korevaar 2001, Proc. SPIE 4214: q = 1.6 above 50 km, 1.3 from 6 to 50 km, 0.16·V + 0.34 from
/// 1 to 6 km, V − 0.5 from 0.5 to 1 km, 0 below 0.5 km, V in km). TODO(verify): the table
/// against the paper; it is the form widely quoted from it.
[[nodiscard]] double kimExponent(double visibilityM);

/// Fog or haze extinction at wavelength `wavelengthM` from the visibility: σ = (3.912/V)·
/// (λ / 550 nm)^(−q), q from kimExponent. In dense fog (V < 500 m) q = 0: the droplets are much
/// larger than the wavelength and extinction no longer depends on it.
[[nodiscard]] double fogExtinction(double visibilityM, double wavelengthM);

/// Marshall–Palmer drop-size distribution (Marshall and Palmer 1948, J. Meteor. 5, 165–166):
/// N(D) = N0·exp(−Λ·D) with N0 = 8000 m⁻³·mm⁻¹ and Λ = 4.1·R^−0.21 mm⁻¹, R in mm/h; here in SI:
/// N0 = 8e6 m⁻⁴ and Λ in 1/m for a rain rate in m/s.
inline constexpr double kMarshallPalmerN0 = 8.0e6;  // m⁻⁴ (8000 m⁻³·mm⁻¹)
[[nodiscard]] double marshallPalmerSlope(double rainRateMps);

/// Rain extinction for wavelengths much shorter than the drops (light, lidar): geometric optics,
/// each drop removing twice its cross-section (the extinction paradox, van de Hulst 1957),
/// integrated over Marshall–Palmer: σ = ∫ 2·(π D²/4)·N(D) dD = π·N0 / Λ³.
[[nodiscard]] double rainExtinction(double rainRateMps);

/// Mineral dust: monodisperse spheres of diameter `medianDiameterM` and density
/// kDustParticleDensity, each removing twice its cross-section (diameter ≫ wavelength):
/// σ = 3·C / (ρ_p·D).
inline constexpr double kDustParticleDensity = 2650.0;  // kg/m³, quartz, the main mineral of dust
[[nodiscard]] double dustExtinction(double concentrationKgPerM3, double medianDiameterM);

/// Falling snow's extinction (1/m) at a rate in m/s of melted water, wavelength-independent like
/// rain's. Illustrative: 1 mm/h gives a visibility of 1 km by Koschmieder, the order of what
/// weather services report; Rasmussen et al. (1999, J. Appl. Meteor. 38) find it varies 3 to 10
/// times from storm to storm at one rate, with the crystals.
inline constexpr double kSnowExtinctionPerMmph = 3.912 / 1000.0;  // 1/m per mm/h
[[nodiscard]] double snowExtinction(double snowRateMps);

/// How much grip a surface has left, of dry asphalt's (env/friction_scale). Range checks, not
/// measurements: hard-packed snow is about 0.20 to 0.35 and ice about 0.10 against about 0.85 dry
/// (engineering tables; 0.35 for cars braking on packed snow at -6 degC, TRB SR 115).
inline constexpr double kFreezing = 273.15;        // K
inline constexpr double kSnowFrictionScale = 0.3;  // of dry: about 0.25
inline constexpr double kIceFrictionScale = 0.13;  // of dry: about 0.11
/// `wetness` 0 to 1, `snowCover` 0 to 1, air temperature in K; wet below freezing is ice.
[[nodiscard]] double frictionScale(double wetness, double snowCover, double temperatureK);

/// Wavelengths of the extinction outputs.
inline constexpr double kVisibleWavelength = 550e-9;  // m
inline constexpr double kLidar905 = 905e-9;           // m
inline constexpr double kLidar1550 = 1550e-9;         // m

// --- The module --------------------------------------------------------------------------------

/// The weather domain (domains/weather.md): the site's atmosphere and what it does to sensors and
/// surfaces, from the world's WeatherAPI_1 prim and the module config ("temperature",
/// "pressure", "relative_humidity" (site parameters, no defaults: without them the module does
/// not start); "visibility", "rain_rate", "snow_rate" (m/s of melted water), "snow_cover" (0 to 1,
/// on the ground at the start), "dust_concentration", "dust_median_diameter", "solar_irradiance",
/// "sky_temperature"; the config wins over the prim). Config "effects" (fog, rain, snow, dust,
/// wet_friction, lens_contamination) selects effects; all by default. Snow lies below freezing and
/// melts above it; env/snow_rate and env/snow_cover say how much.
///
/// Writes env/air_temperature, env/sky_temperature, env/pressure, env/relative_humidity,
/// env/air_density, env/wind/mean (at the reference height, from the prim), env/solar_irradiance,
/// env/sun_direction, env/extinction/{visible,nir_905,nir_1550} (fog, rain and dust together),
/// env/extinction/{fog_905,fog_1550} (the fog's share, which backscatters into lidar clutter),
/// env/extinction/field and env/wind/velocity_field (Field3D: the fog grid, extinction in 1/m,
/// and the wind grid, m/s, from config "fog_field" and "wind_field" or the prim's
/// weather:fogField and weather:windField, NanoVDB files; empty buffers without),
/// env/rain_rate, env/friction_scale, env/dust/concentration (kg/m³, the air's own dust; 0
/// without the dust effect) and env/dust/median_diameter (m, its grains), and
/// sensor/<s>/contamination for every sensor. Default
/// step 50 ms. With config "downwash_dust" it reads env/dust/downwash (kg/m³, the dust rotor
/// outwash lifts, of grains "ground_dust_diameter") and adds it to the air's dust.
///
/// Wet friction and lens contamination are behavioural models (domains/weather.md §7: range
/// checks, not point validations): surfaces wet with time constant kWettingTimeS once it rains
/// at kWettingRainRate or more and dry with kDryingTimeS; friction falls to kWetFrictionScale of
/// dry when wet. A lens clouds as 1 − exp(−∫(R/R_ref/τ_rain + C/C_ref/τ_dust)dt).
///
/// `clear` makes the substitute (domains/weather.md §5): the same atmosphere, no fog, rain,
/// dust or wind, friction 1, clean lenses.
class Weather final : public IModule {
public:
    explicit Weather(bool clear = false) : clear_(clear) {}

    [[nodiscard]] ModuleInfo info() const override;
    [[nodiscard]] Duration preferredStep() const override;
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override;
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override;
    Status step(const StepContext& context, SignalBus& bus) override;
    Status saveState(StateWriter& writer) const override;
    Status restoreState(StateReader& reader) override;

    /// Whether `world` and `config` give the site parameters the module needs.
    [[nodiscard]] static bool hasSite(const WorldView& world, const ModuleConfig& config);

private:
    struct State {
        double temperature{0.0}, skyTemperature{0.0}, pressure{0.0}, humidity{0.0};
        double visibility{0.0};  // m; 0: no fog
        double rainRate{0.0}, dustConcentration{0.0}, dustDiameter{1e-5};
        double irradiance{0.0};
        std::array<double, 3> sun{0.0, 0.0, 1.0}, wind{};
    };
    void writeAll(SignalBus& bus) const;

    bool clear_;
    State s_;
    bool fog_{true}, rain_{true}, snow_{true}, dust_{true}, wetFriction_{true}, lens_{true};
    double wetness_{0.0};    // 0 dry … 1 wet
    double snowCover_{0.0};  // 0 bare … 1 covered
    double snowRate_{0.0};   // m/s of melted water
    double startCover_{0.0};
    SignalId snowRateOut_, snowCoverOut_;
    SignalId dustOut_, dustDiameterOut_;  // env/dust/concentration, env/dust/median_diameter
    std::vector<double> contamination_;   // per sensor, 0 … 1
    SignalId temperatureOut_, skyOut_, pressureOut_, humidityOut_, densityOut_, windOut_;
    SignalId irradianceOut_, sunOut_, visibleOut_, nir905Out_, nir1550Out_, rainOut_, frictionOut_;
    SignalId fog905Out_, fog1550Out_;
    SignalId fogFieldOut_, windFieldOut_;
    std::shared_ptr<const fields::Field> fogField_, windField_;
    std::vector<SignalId> contaminationOut_;
    SignalId downwashDustIn_;          // env/dust/downwash, with config "downwash_dust"
    double liftedDust_{0.0};           // kg/m³, this step's
    double groundDustDiameter_{1e-4};  // m, the lifted grains'
};

// Behavioural constants (see the class comment). Modelling choices, not measured values.
inline constexpr double kWettingTimeS = 120.0;        // s, dry to wet in steady rain
inline constexpr double kWettingRainRate = 2.778e-7;  // m/s (1 mm/h): rain that wets fully
inline constexpr double kDryingTimeS = 1800.0;        // s, wet to dry
// Wet over dry friction: sensmat:frictionWetScale's fallback, inside the published range for
// asphalt (dry 0.7–0.9, wet 0.4–0.7, so 0.45–1.0; a range check).
inline constexpr double kWetFrictionScale = 0.7;
inline constexpr double kLensRainReference = 2.778e-6;  // m/s (10 mm/h)
inline constexpr double kLensRainTimeS = 600.0;         // s to cloud at the reference rain
inline constexpr double kLensDustReference = 1e-6;      // kg/m³ (1 mg/m³: very dusty)
inline constexpr double kLensDustTimeS = 3600.0;        // s to cloud at the reference dust

}  // namespace lagrange::weather
