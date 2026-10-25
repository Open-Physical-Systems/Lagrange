#include "lagrange/signals/Unit.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>

namespace lagrange {

namespace {

constexpr std::array<std::string_view, 37> kKindNames{
    "Unspecified",
    "Torque",
    "Energy",
    "Force",
    "Power",
    "Voltage",
    "Current",
    "Charge",
    "Resistance",
    "Temperature",
    "HeatFlow",
    "AngularVelocity",
    "AngularPosition",
    "LinearVelocity",
    "Position",
    "Pressure",
    "Density",
    "ElectricField",
    "MagneticFluxDensity",
    "DoseRate",
    "AbsorbedDose",
    "Irradiance",
    "Extinction",
    "Probability",
    "Ratio",
    "Count",
    "LinearAcceleration",
    "Frequency",
    "Inductance",
    "Capacitance",
    "Area",
    "Mass",
    "Activity",
    "AngularAcceleration",
    "MomentOfInertia",
    "Duration",
    "DataRate",
};

/// A symbol the parser accepts, with its dimension.
struct Symbol {
    std::string_view text;
    Dimension dim;
};

// Coherent SI units only: schema values are SI (docs/design/formats.md §2), so no prefixes and
// no scaled units such as h or degC.
constexpr std::array<Symbol, 23> kSymbols{{
    {"m", {.length = 1}},
    {"kg", {.mass = 1}},
    {"s", {.time = 1}},
    {"A", {.current = 1}},
    {"K", {.temperature = 1}},
    {"mol", {.amount = 1}},
    {"cd", {.luminous = 1}},
    {"rad", {.angle = 1}},
    {"sr", {.angle = 2}},
    {"N", {.length = 1, .mass = 1, .time = -2}},
    {"J", {.length = 2, .mass = 1, .time = -2}},
    {"W", {.length = 2, .mass = 1, .time = -3}},
    {"V", {.length = 2, .mass = 1, .time = -3, .current = -1}},
    {"C", {.time = 1, .current = 1}},
    {"ohm", {.length = 2, .mass = 1, .time = -3, .current = -2}},
    {"F", {.length = -2, .mass = -1, .time = 4, .current = 2}},
    {"H", {.length = 2, .mass = 1, .time = -2, .current = -2}},
    {"Hz", {.time = -1}},
    {"Pa", {.length = -1, .mass = 1, .time = -2}},
    {"T", {.mass = 1, .time = -2, .current = -1}},
    {"Gy", {.length = 2, .time = -2}},
    {"Bq", {.time = -1}},
    {"S", {.length = -2, .mass = -1, .time = 3, .current = 2}},
}};

constexpr Symbol kWeber{"Wb", {.length = 2, .mass = 1, .time = -2, .current = -1}};

/// Unit strings that name one quantity kind, with the symbol used to display that unit.
struct Named {
    std::string_view text;     // as written in schemas
    std::string_view display;  // as shown to people
    Unit unit;
};

constexpr std::array<Named, 32> kNamed{{
    {"1", "1", units::one},
    {"m", "m", units::m},
    {"m2", "m²", units::m2},
    {"kg", "kg", units::kg},
    {"m/s", "m/s", units::m_per_s},
    {"m/s2", "m/s²", units::m_per_s2},
    {"rad", "rad", units::rad},
    {"rad/s", "rad/s", units::rad_per_s},
    {"rad/s2", "rad/s²", units::rad_per_s2},
    {"kg.m2", "kg·m²", units::kg_m2},
    {"Hz", "Hz", units::Hz},
    {"s", "s", units::s},
    {"N", "N", units::N},
    {"N.m", "N·m", units::N_m},
    {"J", "J", units::J},
    {"W", "W", units::W},
    {"V", "V", units::V},
    {"A", "A", units::A},
    {"C", "C", units::C},
    {"ohm", "Ω", units::ohm},
    {"H", "H", units::H},
    {"F", "F", units::F},
    {"K", "K", units::K},
    {"Pa", "Pa", units::Pa},
    {"kg/m3", "kg/m³", units::kg_per_m3},
    {"V/m", "V/m", units::V_per_m},
    {"T", "T", units::T},
    {"Gy", "Gy", units::Gy},
    {"Gy/s", "Gy/s", units::Gy_per_s},
    {"Bq", "Bq", units::Bq},
    {"W/m2", "W/m²", units::W_per_m2},
    {"1/m", "1/m", units::per_m},
}};

Dimension add(Dimension a, Dimension b, int sign) {
    const auto sum = [sign](std::int8_t x, std::int8_t y) {
        return static_cast<std::int8_t>(x + sign * y);
    };
    return {sum(a.length, b.length),
            sum(a.mass, b.mass),
            sum(a.time, b.time),
            sum(a.current, b.current),
            sum(a.temperature, b.temperature),
            sum(a.amount, b.amount),
            sum(a.luminous, b.luminous),
            sum(a.angle, b.angle)};
}

Dimension power(Dimension d, int exponent) {
    Dimension out{};
    for (int i = 0; i < std::abs(exponent); ++i) {
        out = add(out, d, exponent < 0 ? -1 : 1);
    }
    return out;
}

/// Recursive-descent parser over the grammar in Unit.hpp.
class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}

    UnitParse run() {
        Dimension dim{};
        if (!expression(dim)) {
            return {std::nullopt, error_};
        }
        if (pos_ != text_.size()) {
            return fail("unexpected '" + std::string(1, text_[pos_]) + "'");
        }
        return {Unit{dim, 1.0, QuantityKind::Unspecified}, {}};
    }

private:
    UnitParse fail(std::string message) {
        error_ =
            message + " at position " + std::to_string(pos_) + " in \"" + std::string(text_) + "\"";
        return {std::nullopt, error_};
    }
    bool failed(std::string message) {
        fail(std::move(message));
        return false;
    }

    // expression := factor (('.' factor) | ('/' factor))*, with no '.' after a '/'.
    bool expression(Dimension& out) {
        if (!factor(out)) {
            return false;
        }
        bool divided = false;
        while (pos_ < text_.size() && (text_[pos_] == '.' || text_[pos_] == '/')) {
            const char op = text_[pos_++];
            if (divided) {
                return failed("ambiguous unit: after '/', group the rest in parentheses");
            }
            Dimension next{};
            if (!factor(next)) {
                return false;
            }
            out = add(out, next, op == '/' ? -1 : 1);
            divided = op == '/';
        }
        return true;
    }

    // factor := '(' expression ')' exponent? | symbol exponent? | '1'
    bool factor(Dimension& out) {
        if (pos_ >= text_.size()) {
            return failed("expected a unit");
        }
        if (text_[pos_] == '(') {
            ++pos_;
            if (!expression(out)) {
                return false;
            }
            if (pos_ >= text_.size() || text_[pos_] != ')') {
                return failed("expected ')'");
            }
            ++pos_;
            return exponent(out);
        }
        if (text_[pos_] == '1') {
            ++pos_;
            out = {};
            return true;
        }
        const std::size_t start = pos_;
        while (pos_ < text_.size() && ((text_[pos_] >= 'a' && text_[pos_] <= 'z') ||
                                       (text_[pos_] >= 'A' && text_[pos_] <= 'Z'))) {
            ++pos_;
        }
        const std::string_view name = text_.substr(start, pos_ - start);
        if (name.empty()) {
            return failed("expected a unit symbol");
        }
        const Symbol* found = name == kWeber.text ? &kWeber : nullptr;
        for (const Symbol& symbol : kSymbols) {
            if (symbol.text == name) {
                found = &symbol;
            }
        }
        if (found == nullptr) {
            pos_ = start;
            return failed("unknown unit symbol '" + std::string(name) +
                          "' (schema values are SI, without prefixes)");
        }
        out = found->dim;
        return exponent(out);
    }

    // exponent := '-'? digit+, written directly after the symbol: "m2", "s-1".
    bool exponent(Dimension& out) {
        const std::size_t start = pos_;
        const bool negative = pos_ < text_.size() && text_[pos_] == '-';
        if (negative) {
            ++pos_;
        }
        int value = 0;
        std::size_t digits = 0;
        while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') {
            value = value * 10 + (text_[pos_] - '0');
            ++pos_;
            if (++digits > 2) {
                return failed("exponent too large");
            }
        }
        if (digits == 0) {
            if (negative) {
                pos_ = start;
                return failed("expected digits after '-'");
            }
            return true;
        }
        if (value == 0) {
            return failed("exponent 0");
        }
        out = power(out, negative ? -value : value);
        return true;
    }

    std::string_view text_;
    std::size_t pos_{0};
    std::string error_;
};

void appendExponent(std::string& out, int exponent) {
    static constexpr std::array<std::string_view, 10> kSuperscript{"⁰", "¹", "²", "³", "⁴",
                                                                   "⁵", "⁶", "⁷", "⁸", "⁹"};
    if (exponent == 1) {
        return;
    }
    if (exponent < 0) {
        out += "⁻";
        exponent = -exponent;
    }
    const std::string digits = std::to_string(exponent);
    for (const char digit : digits) {
        out += kSuperscript.at(static_cast<std::size_t>(digit - '0'));
    }
}

}  // namespace

std::string_view to_string(QuantityKind kind) {
    const auto index = static_cast<std::size_t>(kind);
    return index < kKindNames.size() ? kKindNames.at(index) : std::string_view("Unknown");
}

std::string symbol(const Unit& unit) {
    std::string out;
    if (unit.scale != 1.0) {
        char text[32];
        std::snprintf(text, sizeof(text), "%.17g ", unit.scale);
        out = text;
    }
    const std::array<std::pair<std::string_view, int>, 8> parts{{
        {"kg", unit.dim.mass},
        {"m", unit.dim.length},
        {"s", unit.dim.time},
        {"A", unit.dim.current},
        {"K", unit.dim.temperature},
        {"mol", unit.dim.amount},
        {"cd", unit.dim.luminous},
        {"rad", unit.dim.angle},
    }};
    bool any = false;
    for (const auto& [name, exponent] : parts) {
        if (exponent == 0) {
            continue;
        }
        if (any) {
            out += "·";
        }
        out += name;
        appendExponent(out, exponent);
        any = true;
    }
    if (!any) {
        out += "1";
    }
    return out;
}

std::string label(const Unit& unit) {
    for (const Named& named : kNamed) {
        if (named.unit.dim == unit.dim && named.unit.scale == unit.scale &&
            (named.unit.kind == unit.kind || unit.kind == QuantityKind::Unspecified)) {
            return std::string(named.display);
        }
    }
    return symbol(unit);
}

std::string to_string(const Unit& unit) {
    std::string out;
    for (const Named& named : kNamed) {
        if (named.unit == unit) {
            out = named.display;
        }
    }
    if (out.empty()) {
        out = symbol(unit);
    }
    if (unit.kind != QuantityKind::Unspecified) {
        out += " (";
        out += to_string(unit.kind);
        out += ")";
    }
    return out;
}

UnitParse parseUnit(std::string_view text) {
    UnitParse result = Parser(text).run();
    if (!result.unit) {
        return result;
    }
    for (const Named& named : kNamed) {
        if (named.text == text) {
            result.unit->kind = named.unit.kind;
        }
    }
    return result;
}

}  // namespace lagrange
