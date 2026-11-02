#pragma once

// The thermal camera's radiometry (docs/design/domains/sensors.md §2, "Thermal camera"): what a
// long-wave infrared camera reads of a surface, from Planck's law over its band.

#include <array>
#include <cstdint>
#include <vector>

namespace lagrange::sensors {

/// Planck's law, CODATA 2018 exact constants.
inline constexpr double kPlanck = 6.62607015e-34;           // J·s
inline constexpr double kLightSpeed = 299792458.0;          // m/s
inline constexpr double kBoltzmann = 1.380649e-23;          // J/K
inline constexpr double kStefanBoltzmann = 5.670374419e-8;  // W/(m²·K⁴), from the three above

/// Radiance of a black body at `temperature` (K) between the wavelengths `shortest` and
/// `longest` (m), W/(m²·sr): ∫ B_λ(T) dλ, B_λ = 2hc²/λ⁵ / (exp(hc/(λkT)) − 1). Exact, by the
/// series ∫_x^∞ t³/(eᵗ−1) dt = Σ e^(−nx)(x³/n + 3x²/n² + 6x/n³ + 6/n⁴) (Widger and Woodall,
/// Bull. Am. Meteorol. Soc. 57(10), 1976). Over all wavelengths it is σT⁴/π.
[[nodiscard]] double bandRadiance(double temperature, double shortest, double longest);

/// The fraction of a black body's emission at wavelengths below λ, F(0→λT) (Incropera et al.,
/// Fundamentals of Heat and Mass Transfer, Table 12.1), for λT in m·K.
[[nodiscard]] double blackBodyFraction(double wavelengthTimesTemperature);

/// The effective temperature of the clear night sky for long-wave exchange, from the air
/// temperature at screen height (K): T_sky = 0.0552·T_air^1.5 (Swinbank, Q. J. R. Meteorol. Soc.
/// 89, 1963). What a camera reads looking up; clouds would bring it nearer the air's.
[[nodiscard]] double clearSkyTemperature(double airTemperature);

/// Human skin, for people (HumanAPI_1): Fanger's mean skin temperature in comfort,
/// t_sk = 35.7 − 0.0275·M °C (Thermal Comfort, 1970), at M = 116 W/m² (2 met, walking slowly):
/// 32.5 °C. Emissivity 0.98 (Steketee, Phys. Med. Biol. 18(5), 1973). Clothing, cooler than
/// skin, is not told apart.
inline constexpr double kSkinTemperature = 273.15 + 35.7 - 0.0275 * 116.0;  // K
inline constexpr double kSkinEmissivity = 0.98;

/// A camera's band, to turn radiance back into the temperature a black body would need for it
/// (the apparent, or brightness, temperature, which radiometric cameras report).
class ThermalBand {
public:
    /// Tabulated from kLowest to kHighest every kStep.
    static constexpr double kLowest = 100.0;    // K
    static constexpr double kHighest = 2000.0;  // K
    static constexpr double kStep = 0.25;       // K

    ThermalBand(double shortest, double longest);
    [[nodiscard]] double radiance(double temperature) const;
    /// The apparent temperature of `radiance`, clamped to the table.
    [[nodiscard]] double temperatureOf(double radiance) const;
    /// What the camera reads of a surface at `surface` (K) with `emissivity`, opaque, in
    /// surroundings at `surroundings` (K): ε·B(T_s) + (1−ε)·B(T_surr), as an apparent temperature.
    [[nodiscard]] double apparent(double surface, double emissivity, double surroundings) const;

private:
    double shortest_, longest_;
    std::vector<double> table_;  // radiance at kLowest + k·kStep
};

/// How temperatures are drawn (sensor:thermal:palette).
enum class ThermalPalette : std::uint8_t { WhiteHot, BlackHot, Ironbow };
/// The colour of `x` in [0, 1] (0 coldest), display-encoded RGBA. Ironbow is the usual
/// black–purple–red–yellow–white ramp, by eye.
[[nodiscard]] std::array<std::uint8_t, 4> paletteColour(ThermalPalette palette, double x);

}  // namespace lagrange::sensors
