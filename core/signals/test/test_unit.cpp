#include "lagrange/signals/Unit.hpp"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange {
namespace {

TEST(Unit, TorqueAndEnergyShareADimensionButDiffer) {
    EXPECT_EQ(units::N_m.dim, units::J.dim);
    EXPECT_NE(units::N_m, units::J);
}

TEST(Unit, AngularVelocityAndFrequencyDiffer) {
    EXPECT_NE(units::rad_per_s.dim, units::Hz.dim);
    EXPECT_EQ(units::Hz.dim, units::Bq.dim);
    EXPECT_NE(units::Hz, units::Bq);
}

TEST(Unit, ToStringNamesTheSymbolAndTheKind) {
    EXPECT_EQ(to_string(units::N_m), "N·m (Torque)");
    EXPECT_EQ(to_string(units::J), "J (Energy)");
    EXPECT_EQ(to_string(units::ohm), "Ω (Resistance)");
    EXPECT_EQ(to_string(units::m_per_s2), "m/s² (LinearAcceleration)");
    EXPECT_EQ(to_string(units::one), "1 (Ratio)");
}

TEST(Unit, ToStringFallsBackToBaseUnits) {
    const Unit torqueConstant{{.length = 2, .mass = 1, .time = -2, .current = -1}};
    EXPECT_EQ(to_string(torqueConstant), "kg·m²·s⁻²·A⁻¹");
    const Unit ampereHour{{.time = 1, .current = 1}, 3600.0, QuantityKind::Charge};
    EXPECT_EQ(to_string(ampereHour), "3600 s·A (Charge)");
    EXPECT_EQ(to_string(Unit{}), "1");
}

TEST(Unit, KindNames) {
    EXPECT_EQ(to_string(QuantityKind::Unspecified), "Unspecified");
    EXPECT_EQ(to_string(QuantityKind::Activity), "Activity");
    EXPECT_EQ(to_string(static_cast<QuantityKind>(999)), "Unknown");
}

// Every unit string used in docs/design/formats.md and rfcs/0002-usd-schemas.md.
TEST(ParseUnit, EverySchemaUnitString) {
    const std::vector<std::pair<std::string, Unit>> cases{
        {"1", units::one},
        {"K", units::K},
        {"m2", units::m2},
        {"m", units::m},
        {"ohm", units::ohm},
        {"A", units::A},
        {"V", units::V},
        {"Hz", units::Hz},
        {"H", units::H},
        {"m/s", units::m_per_s},
        {"V/m", units::V_per_m},
        {"F", units::F},
        {"1/K", Unit{{.temperature = -1}}},
        {"rad/s", units::rad_per_s},
        {"kg/m3", units::kg_per_m3},
        {"kg.m2", units::kg_m2},
        {"W/m2", units::W_per_m2},
        {"W/K", Unit{{.length = 2, .mass = 1, .time = -3, .temperature = -1}}},
        {"W/(m2.K)", Unit{{.mass = 1, .time = -3, .temperature = -1}}},
        {"W", units::W},
        {"V/(V/m)", Unit{{.length = 1}}},
        {"V.s/rad", Unit{{.length = 2, .mass = 1, .time = -2, .current = -1, .angle = -1}}},
        {"S/m", Unit{{.length = -3, .mass = -1, .time = 3, .current = 2}}},
        {"Pa", units::Pa},
        {"N.m/A", Unit{{.length = 2, .mass = 1, .time = -2, .current = -1}}},
        {"N.m.s/rad", Unit{{.length = 2, .mass = 1, .time = -1, .angle = -1}}},
        {"J/K", Unit{{.length = 2, .mass = 1, .time = -2, .temperature = -1}}},
        {"J", units::J},
        {"Gy/s", units::Gy_per_s},
        {"Gy.m2", Unit{{.length = 4, .time = -2}}},
        {"Gy", units::Gy},
        {"C", units::C},
        {"Bq", units::Bq},
        {"1/rad", Unit{{.angle = -1}}},
        {"1/m", units::per_m},
        {"N.m", units::N_m},
        {"m/s2", units::m_per_s2},
        {"s-1", Unit{{.time = -1}}},
        {"Wb", Unit{{.length = 2, .mass = 1, .time = -2, .current = -1}}},
        {"sr", Unit{{.angle = 2}}},
    };
    for (const auto& [text, expected] : cases) {
        const UnitParse parsed = parseUnit(text);
        ASSERT_TRUE(parsed.unit.has_value()) << text << ": " << parsed.error;
        EXPECT_EQ(*parsed.unit, expected) << text << " gave " << to_string(*parsed.unit);
        EXPECT_TRUE(parsed.error.empty());
    }
}

TEST(ParseUnit, BaseUnitsGiveTheSameDimensionAsNamedOnes) {
    const UnitParse newtonMetre = parseUnit("kg.m2/s2");
    ASSERT_TRUE(newtonMetre.unit);
    EXPECT_EQ(newtonMetre.unit->dim, units::N_m.dim);
    // Written in base units, the kind is not implied: it could be torque or energy.
    EXPECT_EQ(newtonMetre.unit->kind, QuantityKind::Unspecified);
}

TEST(ParseUnit, RejectsWithAReason) {
    const std::vector<std::pair<std::string, std::string>> cases{
        {"", "expected a unit"},
        {"mm", "unknown unit symbol 'mm'"},
        {"degC", "unknown unit symbol 'degC'"},
        {"km", "unknown unit symbol 'km'"},
        {"W/m2.K", "ambiguous unit"},
        {"N.", "expected a unit"},
        {"(m", "expected ')'"},
        {"m0", "exponent 0"},
        {"m-", "expected digits after '-'"},
        {"m123", "exponent too large"},
        {"N m", "unexpected ' '"},
        {"N·m", "unexpected"},
    };
    for (const auto& [text, reason] : cases) {
        const UnitParse parsed = parseUnit(text);
        EXPECT_FALSE(parsed.unit.has_value()) << text;
        EXPECT_NE(parsed.error.find(reason), std::string::npos)
            << text << ": got \"" << parsed.error << "\"";
    }
}

}  // namespace
}  // namespace lagrange
