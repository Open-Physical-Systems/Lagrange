#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace lagrange {

/// Exponents of the seven SI base units plus the plane angle.
///
/// Angle is tracked as its own dimension so that rad/s and 1/s (Hz) are different units.
struct Dimension {
    std::int8_t length{0};       // m
    std::int8_t mass{0};         // kg
    std::int8_t time{0};         // s
    std::int8_t current{0};      // A
    std::int8_t temperature{0};  // K
    std::int8_t amount{0};       // mol
    std::int8_t luminous{0};     // cd
    std::int8_t angle{0};        // rad

    friend constexpr bool operator==(const Dimension&, const Dimension&) = default;
};

/// What a quantity is, beyond its dimension: torque and energy are both kg·m²·s⁻².
///
/// The numbers are stored in recordings and sent in the viewer protocol (`Unit.kind`), so
/// values are only ever appended, never renumbered.
enum class QuantityKind : std::uint16_t {
    Unspecified = 0,
    Torque = 1,
    Energy = 2,
    Force = 3,
    Power = 4,
    Voltage = 5,
    Current = 6,
    Charge = 7,
    Resistance = 8,
    Temperature = 9,
    HeatFlow = 10,
    AngularVelocity = 11,
    AngularPosition = 12,
    LinearVelocity = 13,
    Position = 14,
    Pressure = 15,
    Density = 16,
    ElectricField = 17,
    MagneticFluxDensity = 18,
    DoseRate = 19,
    AbsorbedDose = 20,
    Irradiance = 21,
    Extinction = 22,
    Probability = 23,
    Ratio = 24,
    Count = 25,
    LinearAcceleration = 26,
    Frequency = 27,
    Inductance = 28,
    Capacitance = 29,
    Area = 30,
    Mass = 31,
    Activity = 32,
    AngularAcceleration = 33,
    MomentOfInertia = 34,
};

/// A unit: dimension, factor to the coherent SI unit, and quantity kind.
///
/// Two units are wired together only if all three are equal (invariant 2). There is no
/// implicit conversion anywhere.
struct Unit {
    Dimension dim{};
    double scale{1.0};  // factor to the coherent SI unit; 1.0 for SI
    QuantityKind kind{QuantityKind::Unspecified};

    friend constexpr bool operator==(const Unit&, const Unit&) = default;
};

namespace units {  // SI, scale 1.0
inline constexpr Unit one{{}, 1.0, QuantityKind::Ratio};
inline constexpr Unit m{{.length = 1}, 1.0, QuantityKind::Position};
inline constexpr Unit m2{{.length = 2}, 1.0, QuantityKind::Area};
inline constexpr Unit kg{{.mass = 1}, 1.0, QuantityKind::Mass};
inline constexpr Unit m_per_s{{.length = 1, .time = -1}, 1.0, QuantityKind::LinearVelocity};
inline constexpr Unit m_per_s2{{.length = 1, .time = -2}, 1.0, QuantityKind::LinearAcceleration};
inline constexpr Unit rad{{.angle = 1}, 1.0, QuantityKind::AngularPosition};
inline constexpr Unit rad_per_s{{.time = -1, .angle = 1}, 1.0, QuantityKind::AngularVelocity};
inline constexpr Unit rad_per_s2{{.time = -2, .angle = 1}, 1.0, QuantityKind::AngularAcceleration};
inline constexpr Unit kg_m2{{.length = 2, .mass = 1}, 1.0, QuantityKind::MomentOfInertia};
inline constexpr Unit Hz{{.time = -1}, 1.0, QuantityKind::Frequency};
inline constexpr Unit N{{.length = 1, .mass = 1, .time = -2}, 1.0, QuantityKind::Force};
inline constexpr Unit N_m{{.length = 2, .mass = 1, .time = -2}, 1.0, QuantityKind::Torque};
inline constexpr Unit J{{.length = 2, .mass = 1, .time = -2}, 1.0, QuantityKind::Energy};
inline constexpr Unit W{{.length = 2, .mass = 1, .time = -3}, 1.0, QuantityKind::Power};
inline constexpr Unit V{
    {.length = 2, .mass = 1, .time = -3, .current = -1}, 1.0, QuantityKind::Voltage};
inline constexpr Unit A{{.current = 1}, 1.0, QuantityKind::Current};
inline constexpr Unit C{{.time = 1, .current = 1}, 1.0, QuantityKind::Charge};
inline constexpr Unit ohm{
    {.length = 2, .mass = 1, .time = -3, .current = -2}, 1.0, QuantityKind::Resistance};
inline constexpr Unit H{
    {.length = 2, .mass = 1, .time = -2, .current = -2}, 1.0, QuantityKind::Inductance};
inline constexpr Unit F{
    {.length = -2, .mass = -1, .time = 4, .current = 2}, 1.0, QuantityKind::Capacitance};
inline constexpr Unit K{{.temperature = 1}, 1.0, QuantityKind::Temperature};
inline constexpr Unit Pa{{.length = -1, .mass = 1, .time = -2}, 1.0, QuantityKind::Pressure};
inline constexpr Unit kg_per_m3{{.length = -3, .mass = 1}, 1.0, QuantityKind::Density};
inline constexpr Unit V_per_m{
    {.length = 1, .mass = 1, .time = -3, .current = -1}, 1.0, QuantityKind::ElectricField};
inline constexpr Unit T{
    {.mass = 1, .time = -2, .current = -1}, 1.0, QuantityKind::MagneticFluxDensity};
inline constexpr Unit Gy{{.length = 2, .time = -2}, 1.0, QuantityKind::AbsorbedDose};
inline constexpr Unit Gy_per_s{{.length = 2, .time = -3}, 1.0, QuantityKind::DoseRate};
inline constexpr Unit Bq{{.time = -1}, 1.0, QuantityKind::Activity};
inline constexpr Unit W_per_m2{{.mass = 1, .time = -3}, 1.0, QuantityKind::Irradiance};
inline constexpr Unit per_m{{.length = -1}, 1.0, QuantityKind::Extinction};
}  // namespace units

/// Name of a quantity kind, e.g. "Torque". "Unspecified" for kind 0, "Unknown" outside the enum.
[[nodiscard]] std::string_view to_string(QuantityKind kind);

/// The unit in SI symbols, e.g. "kg·m²·s⁻²"; "1" for a dimensionless unit. A scale other
/// than 1 is written in front ("3600 A·s").
[[nodiscard]] std::string symbol(const Unit& unit);

/// The unit as people read it, without the kind: "N·m", "Ω", "m/s²", or the base-unit form.
[[nodiscard]] std::string label(const Unit& unit);

/// The unit for messages, e.g. "N·m (Torque)": a named symbol where one matches, else the
/// base-unit form, then the quantity kind. Used in wiring errors, so both sides of a mismatch
/// read clearly.
[[nodiscard]] std::string to_string(const Unit& unit);

/// Result of parsing a unit string: the unit, or why the text is not one.
struct UnitParse {
    std::optional<Unit> unit;
    std::string error;  // empty when `unit` is set
};

/// Parses a unit string as written in schema metadata (the `lagrangeUnit` field).
///
/// Grammar: SI symbols (m kg s A K mol cd rad sr N J W V C ohm F H Hz Pa T Gy Bq S Wb),
/// products with '.', a quotient with '/', integer exponents written after the symbol ("m2",
/// "s-2"), parentheses, and "1" for dimensionless. Prefixes are not accepted: schema values are
/// SI. "a/b.c" is rejected as ambiguous; write "a/(b.c)" or "a.c/b".
///
/// The quantity kind is set when the text is one that names a kind unambiguously ("N.m" is
/// Torque, "J" is Energy, "Hz" Frequency, "Bq" Activity, ...). Otherwise it is Unspecified.
[[nodiscard]] UnitParse parseUnit(std::string_view text);

}  // namespace lagrange
