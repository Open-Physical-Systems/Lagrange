// See SignalAnalysis.hpp. The FFT is the textbook iterative radix-2 transform; written here
// rather than taken from a library because the viewer needs one small real spectrum at a few
// hertz, and the usual libraries are GPL (FFTW) or more than this needs.
#include "lagrange/viewer/SignalAnalysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace lagrange::viewer {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kMinSpectrumSamples = 8;

}  // namespace

std::size_t floorPowerOfTwo(std::size_t n) {
    if (n == 0) {
        return 0;
    }
    std::size_t p = 1;
    while (p <= n / 2) {
        p *= 2;
    }
    return p;
}

bool fft(std::span<std::complex<double>> data, bool inverse) {
    const std::size_t n = data.size();
    if (n == 0 || (n & (n - 1)) != 0) {
        return false;
    }
    // Bit-reversal permutation.
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; (j & bit) != 0; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(data[i], data[j]);
        }
    }
    // Butterflies; the twiddles of each stage from cos/sin directly (no recurrence drift).
    const double sign = inverse ? 1.0 : -1.0;
    for (std::size_t length = 2; length <= n; length *= 2) {
        const std::size_t half = length / 2;
        const double step = sign * 2.0 * std::numbers::pi / static_cast<double>(length);
        for (std::size_t k = 0; k < half; ++k) {
            const double angle = step * static_cast<double>(k);
            const std::complex<double> w(std::cos(angle), std::sin(angle));
            for (std::size_t start = 0; start < n; start += length) {
                const std::complex<double> u = data[start + k];
                const std::complex<double> v = data[start + k + half] * w;
                data[start + k] = u + v;
                data[start + k + half] = u - v;
            }
        }
    }
    if (inverse) {
        const double scale = 1.0 / static_cast<double>(n);
        for (std::complex<double>& value : data) {
            value *= scale;
        }
    }
    return true;
}

Measurement measure(const ScopeBuffer& buffer, std::int64_t aNs, std::int64_t bNs) {
    const std::int64_t from = std::min(aNs, bNs);
    const std::int64_t to = std::max(aNs, bNs);
    const ScopeBuffer::Statistics s = buffer.statistics(from, to + 1);
    Measurement m;
    m.count = s.count;
    m.seconds = static_cast<double>(to - from) * 1e-9;
    m.min = s.min;
    m.max = s.max;
    m.mean = s.mean;
    m.rms = s.rms;
    m.peakToPeak = s.max - s.min;
    const double a = buffer.valueAt(aNs);
    const double b = buffer.valueAt(bNs);
    m.delta = (std::isnan(a) || std::isnan(b)) ? kNaN : b - a;
    m.slope = (std::isnan(m.delta) || aNs == bNs)
                  ? kNaN
                  : m.delta / (static_cast<double>(bNs - aNs) * 1e-9);
    return m;
}

Spectrum spectrum(std::span<const double> samples, double dtSeconds, SpectrumWindow window,
                  std::size_t maxSamples) {
    Spectrum result;
    const std::size_t n = floorPowerOfTwo(std::min(samples.size(), maxSamples));
    if (n < kMinSpectrumSamples || !(dtSeconds > 0.0)) {
        return result;
    }
    const std::span<const double> used = samples.subspan(samples.size() - n);  // the newest
    double sum = 0.0;
    std::size_t valid = 0;
    for (const double value : used) {
        if (!std::isnan(value)) {
            sum += value;
            ++valid;
        }
    }
    const double mean = valid == 0 ? 0.0 : sum / static_cast<double>(valid);

    std::vector<std::complex<double>> data(n);
    double gain = 0.0;  // sum of the window: the coherent gain times N
    for (std::size_t i = 0; i < n; ++i) {
        const double w = window == SpectrumWindow::Hann
                             ? 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi *
                                                    static_cast<double>(i) / static_cast<double>(n))
                             : 1.0;
        const double value = std::isnan(used[i]) ? mean : used[i];
        data[i] = {(value - mean) * w, 0.0};
        gain += w;
    }
    fft(data);

    result.samples = n;
    result.sampleSeconds = dtSeconds;
    result.binHz = 1.0 / (static_cast<double>(n) * dtSeconds);
    result.dc = mean;
    const std::size_t bins = n / 2 + 1;
    result.hz.resize(bins);
    result.amplitude.resize(bins);
    for (std::size_t k = 0; k < bins; ++k) {
        result.hz[k] = static_cast<double>(k) * result.binHz;
        // Single-sided: every bin but 0 and Nyquist holds half of its sine.
        const double sides = (k == 0 || k == n / 2) ? 1.0 : 2.0;
        result.amplitude[k] = sides * std::abs(data[k]) / gain;
    }
    result.peak = 1;
    for (std::size_t k = 2; k < bins; ++k) {
        if (result.amplitude[k] > result.amplitude[result.peak]) {
            result.peak = k;
        }
    }
    // Between bins: the vertex of a parabola through the log amplitudes of the peak and its
    // neighbours (exact for a Gaussian, close for the Hann window's main lobe).
    result.peakHz = result.hz[result.peak];
    result.peakAmplitude = result.amplitude[result.peak];
    if (result.peak + 1 < bins) {
        const double left = result.amplitude[result.peak - 1];
        const double mid = result.amplitude[result.peak];
        const double right = result.amplitude[result.peak + 1];
        // Neighbours at rounding noise (a rectangular window on a bin) say nothing: keep the bin.
        const double meaningful = mid * 1e-9;
        if (left > meaningful && mid > 0.0 && right > meaningful) {
            const double a = std::log(left);
            const double b = std::log(mid);
            const double c = std::log(right);
            const double denominator = a - 2.0 * b + c;
            if (denominator < 0.0) {
                const double offset = 0.5 * (a - c) / denominator;
                if (std::abs(offset) <= 0.5) {
                    result.peakHz += offset * result.binHz;
                }
            }
        }
    }
    return result;
}

Spectrum spectrumOf(const ScopeBuffer& buffer, std::int64_t tBeginNs, std::int64_t tEndNs,
                    SpectrumWindow window, std::size_t maxSamples) {
    const std::int64_t dt = buffer.dtNs();
    if (dt <= 0 || tEndNs <= tBeginNs) {
        return {};
    }
    // Only the newest maxSamples are used: read no more than those.
    const std::int64_t span = static_cast<std::int64_t>(maxSamples) * dt;
    const std::int64_t begin = std::max(tBeginNs, tEndNs - span);
    const auto count = static_cast<std::size_t>(std::min<std::int64_t>(
        (tEndNs - begin) / dt + 2, static_cast<std::int64_t>(maxSamples) + 2));
    std::vector<double> times(count);
    std::vector<double> values(count);
    const std::size_t n = buffer.readRaw(begin, tEndNs, times, values);
    values.resize(n);
    return spectrum(values, static_cast<double>(dt) * 1e-9, window, maxSamples);
}

}  // namespace lagrange::viewer
