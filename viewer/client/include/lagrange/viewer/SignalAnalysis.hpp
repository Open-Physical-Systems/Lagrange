#pragma once

// What the Signals workspace's Measure and FFT tabs compute (docs/design/ui/05-signals): the
// numbers between the cursors, and the amplitude spectrum of a trace. No drawing, so it can be
// tested without a window.

#include "lagrange/viewer/ScopeBuffer.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace lagrange::viewer {

/// In-place radix-2 FFT (Cooley-Tukey, decimation in time). `data.size()` must be a power of
/// two (1 is fine); returns false and leaves `data` alone when it is not. The forward transform
/// is X_k = sum_n x_n e^{-2 pi i k n / N}, unscaled; `inverse` uses e^{+...} and divides by N.
bool fft(std::span<std::complex<double>> data, bool inverse = false);

/// The largest power of two that is at most `n` (0 for 0).
[[nodiscard]] std::size_t floorPowerOfTwo(std::size_t n);

/// The samples of one trace between two times (the cursors A and B, in either order).
struct Measurement {
    std::uint64_t count{0};  ///< Samples between A and B, gaps not counted.
    double seconds{0.0};     ///< |B - A|.
    double min{0.0};
    double max{0.0};
    double mean{0.0};
    double rms{0.0};
    double peakToPeak{0.0};  ///< max - min.
    double delta{0.0};       ///< value at B - value at A (NaN when either is not held).
    double slope{0.0};       ///< delta / (B - A), per second (NaN as delta, or when A = B).
};
/// Measures the raw samples of `buffer` in [min(A, B), max(A, B)] (both ends included).
[[nodiscard]] Measurement measure(const ScopeBuffer& buffer, std::int64_t aNs, std::int64_t bNs);

enum class SpectrumWindow { Rectangular, Hann };

/// A single-sided amplitude spectrum: a sine of amplitude a at a bin's frequency shows as a
/// peak of height a (the window's coherent gain is divided out).
struct Spectrum {
    std::size_t samples{0};         ///< N, a power of two; 0 when there were fewer than 8 samples.
    double sampleSeconds{0.0};      ///< dt.
    double binHz{0.0};              ///< 1 / (N dt).
    double dc{0.0};                 ///< Mean of the samples; it is taken out before the transform.
    std::vector<double> hz;         ///< N / 2 + 1 bins, 0 ... Nyquist.
    std::vector<double> amplitude;  ///< Same unit as the signal.
    std::size_t peak{0};            ///< Bin of the largest amplitude above 0 Hz.
    double peakHz{0.0};             ///< The peak's frequency, interpolated between bins.
    double peakAmplitude{0.0};
};

/// The spectrum of the newest power-of-two run of `samples` (at most `maxSamples`). A NaN
/// (a gap in the stream) counts as the mean.
[[nodiscard]] Spectrum spectrum(std::span<const double> samples, double dtSeconds,
                                SpectrumWindow window, std::size_t maxSamples = 65536);

/// The spectrum of what `buffer` holds in [tBeginNs, tEndNs): the raw samples, not decimated.
[[nodiscard]] Spectrum spectrumOf(const ScopeBuffer& buffer, std::int64_t tBeginNs,
                                  std::int64_t tEndNs, SpectrumWindow window,
                                  std::size_t maxSamples = 65536);

}  // namespace lagrange::viewer
