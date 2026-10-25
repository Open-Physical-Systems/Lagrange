#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>

namespace lagrange::random {

namespace {

// Round multipliers and Weyl key increments of Philox4x32 (Salmon et al. 2011, Table 2;
// the same constants as Random123's philox.h).
constexpr std::uint32_t kMultiplier0 = 0xD2511F53U;
constexpr std::uint32_t kMultiplier1 = 0xCD9E8D57U;
constexpr std::uint32_t kWeyl0 = 0x9E3779B9U;  // golden ratio
constexpr std::uint32_t kWeyl1 = 0xBB67AE85U;  // sqrt(3) - 1
constexpr int kRounds = 10;

// Counter word that separates module-seed derivation from every stream: streams use a block
// index there, which never reaches this value for one entity in one step.
constexpr std::uint32_t kSeedDomain = 0xFFFFFFFFU;

void mulhilo(std::uint32_t a, std::uint32_t b, std::uint32_t& hi, std::uint32_t& lo) {
    const std::uint64_t product = static_cast<std::uint64_t>(a) * b;
    hi = static_cast<std::uint32_t>(product >> 32U);
    lo = static_cast<std::uint32_t>(product);
}

Key keyOf(std::uint64_t seed) {
    return {static_cast<std::uint32_t>(seed), static_cast<std::uint32_t>(seed >> 32U)};
}

}  // namespace

Counter philox4x32_10(Counter counter, Key key) {
    for (int round = 0; round < kRounds; ++round) {
        if (round > 0) {
            key[0] += kWeyl0;
            key[1] += kWeyl1;
        }
        std::uint32_t hi0 = 0;
        std::uint32_t lo0 = 0;
        std::uint32_t hi1 = 0;
        std::uint32_t lo1 = 0;
        mulhilo(kMultiplier0, counter[0], hi0, lo0);
        mulhilo(kMultiplier1, counter[2], hi1, lo1);
        counter = {hi1 ^ counter[1] ^ key[0], lo1, hi0 ^ counter[3] ^ key[1], lo0};
    }
    return counter;
}

std::uint64_t moduleSeed(std::uint64_t runSeed, std::uint32_t moduleId) {
    const Counter out = philox4x32_10({moduleId, 0, 0, kSeedDomain}, keyOf(runSeed));
    return (static_cast<std::uint64_t>(out[1]) << 32U) | out[0];
}

Stream::Stream(std::uint64_t moduleSeed, std::uint64_t stepIndex, std::uint32_t entity)
    : key_(keyOf(moduleSeed))
    , counter_{static_cast<std::uint32_t>(stepIndex), static_cast<std::uint32_t>(stepIndex >> 32U),
               entity, 0} {}

std::uint32_t Stream::nextU32() {
    if (used_ == 4) {
        block_ = philox4x32_10(counter_, key_);
        ++counter_[3];
        used_ = 0;
    }
    return block_.at(used_++);
}

double Stream::uniform() {
    // 27 + 26 bits fill a double's 53-bit significand exactly (the method of
    // std::generate_canonical and of Random123's u01 helpers).
    const std::uint64_t high = nextU32() >> 5U;
    const std::uint64_t low = nextU32() >> 6U;
    return static_cast<double>((high << 26U) | low) * 0x1.0p-53;
}

double Stream::normal() {
    // 1 - uniform() is in (0, 1], so the logarithm is finite.
    const double radius = std::sqrt(-2.0 * std::log(1.0 - uniform()));
    return radius * std::cos(2.0 * std::numbers::pi * uniform());
}

}  // namespace lagrange::random
