#include "lagrange/time/SimTime.hpp"

#include <limits>
#include <string>

namespace lagrange {

namespace {

constexpr int kNanosecondsExponent = 9;               // 1 s = 10^9 ns
constexpr std::int64_t kMaxExponentMagnitude = 1000;  // far beyond any representable value

constexpr bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

}  // namespace

std::optional<Duration> parseDecimalSeconds(std::string_view text) {
    std::size_t i = 0;
    const std::size_t n = text.size();

    if (i < n && text[i] == '+') {
        ++i;
    }

    // Collect the mantissa digits and remember how many of them follow the decimal point.
    std::string digits;
    std::int64_t fractionDigits = 0;
    while (i < n && isDigit(text[i])) {
        digits.push_back(text[i++]);
    }
    if (i < n && text[i] == '.') {
        ++i;
        while (i < n && isDigit(text[i])) {
            digits.push_back(text[i++]);
            ++fractionDigits;
        }
    }
    if (digits.empty()) {
        return std::nullopt;
    }

    std::int64_t exponent = 0;
    if (i < n && (text[i] == 'e' || text[i] == 'E')) {
        ++i;
        bool negative = false;
        if (i < n && (text[i] == '+' || text[i] == '-')) {
            negative = text[i] == '-';
            ++i;
        }
        if (i >= n || !isDigit(text[i])) {
            return std::nullopt;
        }
        while (i < n && isDigit(text[i])) {
            if (exponent > kMaxExponentMagnitude) {
                return std::nullopt;
            }
            exponent = exponent * 10 + (text[i++] - '0');
        }
        if (negative) {
            exponent = -exponent;
        }
    }
    if (i != n) {
        return std::nullopt;
    }

    // value in nanoseconds = (digits as an integer) * 10^shift
    std::int64_t shift = exponent - fractionDigits + kNanosecondsExponent;

    // A negative shift divides by ten: only allowed while the removed digit is zero.
    while (shift < 0 && !digits.empty()) {
        if (digits.back() != '0') {
            return std::nullopt;  // not a whole number of nanoseconds
        }
        digits.pop_back();
        ++shift;
    }

    constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
    std::int64_t value = 0;
    for (const char c : digits) {
        const std::int64_t digit = c - '0';
        if (value > (kMax - digit) / 10) {
            return std::nullopt;
        }
        value = value * 10 + digit;
    }
    for (; shift > 0 && value != 0; --shift) {
        if (value > kMax / 10) {
            return std::nullopt;
        }
        value *= 10;
    }
    return Duration{value};
}

}  // namespace lagrange
