#pragma once

#include <array>
#include <cstdint>

namespace lagrange::random {

/// Philox 4x32 with 10 rounds: a counter-based random number generator.
///
/// The output is a pure function of (counter, key), so no module keeps sequential RNG state
/// and results do not depend on the order modules or entities are stepped in
/// (docs/design/architecture.md §12). Algorithm from Salmon, Moraes, Dror and Shaw,
/// "Parallel random numbers: as easy as 1, 2, 3", SC 2011; checked against the known-answer
/// vectors of the authors' Random123 library.
using Counter = std::array<std::uint32_t, 4>;
using Key = std::array<std::uint32_t, 2>;

[[nodiscard]] Counter philox4x32_10(Counter counter, Key key);

/// The seed a module receives for a run: a hash of the run seed and the module's id, so two
/// modules never draw the same numbers. It is what `StepContext::seed` carries.
[[nodiscard]] std::uint64_t moduleSeed(std::uint64_t runSeed, std::uint32_t moduleId);

/// Random numbers for one entity in one step of one module.
///
/// The counter is (stepIndex low, stepIndex high, entity, block) and the key is the module
/// seed. Each block gives four 32-bit values; a stream can draw up to 2^32 blocks.
///
///     Stream rng(ctx.seed, ctx.stepIndex, wheelIndex);
///     const double noise = sigma * rng.normal();
class Stream {
public:
    Stream(std::uint64_t moduleSeed, std::uint64_t stepIndex, std::uint32_t entity);

    /// The next 32 random bits.
    [[nodiscard]] std::uint32_t nextU32();
    /// Uniform in [0, 1) with 53 random bits.
    [[nodiscard]] double uniform();
    /// Standard normal (mean 0, variance 1), by the Box-Muller transform.
    [[nodiscard]] double normal();

private:
    Key key_;
    Counter counter_;
    Counter block_{};
    unsigned used_{4};  // values of block_ already returned
};

}  // namespace lagrange::random
