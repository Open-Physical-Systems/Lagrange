// The Signals workspace's Measure, FFT and Saved sets (docs/design/ui/05-signals).
#include "lagrange/viewer/ScopeBuffer.hpp"
#include "lagrange/viewer/SignalAnalysis.hpp"
#include "lagrange/viewer/SignalSets.hpp"

#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <vector>

#include <gtest/gtest.h>

namespace {

using namespace lagrange::viewer;
constexpr std::int64_t kDt = 100'000;  // 10 kHz, as the motor currents
constexpr double kDtSeconds = 1e-4;

void push(ScopeBuffer& buffer, std::uint64_t first, const std::vector<double>& values) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    buffer.push(first, reinterpret_cast<const std::byte*>(values.data()), values.size(),
                sizeof(double));
}

std::vector<double> sine(std::size_t n, double amplitude, double hz, double offset = 0.0) {
    std::vector<double> values(n);
    for (std::size_t i = 0; i < n; ++i) {
        values[i] = offset + amplitude * std::sin(2.0 * std::numbers::pi * hz *
                                                  static_cast<double>(i) * kDtSeconds);
    }
    return values;
}

// --- FFT ----------------------------------------------------------------------------------

TEST(Fft, MatchesTheDefinitionOnRandomData) {
    std::mt19937 random(7);
    std::normal_distribution<double> normal;
    for (const std::size_t n : {1U, 2U, 4U, 8U, 64U, 256U}) {
        std::vector<std::complex<double>> x(n);
        for (auto& value : x) {
            value = {normal(random), normal(random)};
        }
        std::vector<std::complex<double>> fast = x;
        ASSERT_TRUE(fft(fast));
        for (std::size_t k = 0; k < n; ++k) {
            std::complex<double> slow;
            for (std::size_t i = 0; i < n; ++i) {
                const double angle =
                    -2.0 * std::numbers::pi * static_cast<double>(k * i) / static_cast<double>(n);
                slow += x[i] * std::complex<double>(std::cos(angle), std::sin(angle));
            }
            EXPECT_NEAR(fast[k].real(), slow.real(), 1e-9 * static_cast<double>(n))
                << n << " " << k;
            EXPECT_NEAR(fast[k].imag(), slow.imag(), 1e-9 * static_cast<double>(n))
                << n << " " << k;
        }
        ASSERT_TRUE(fft(fast, true));  // and back
        for (std::size_t i = 0; i < n; ++i) {
            EXPECT_NEAR(std::abs(fast[i] - x[i]), 0.0, 1e-12);
        }
    }
}

TEST(Fft, RefusesSizesThatAreNotPowersOfTwo) {
    std::vector<std::complex<double>> x(12, {1.0, 0.0});
    EXPECT_FALSE(fft(x));
    EXPECT_EQ(x[0], std::complex<double>(1.0, 0.0));
    EXPECT_EQ(floorPowerOfTwo(0), 0U);
    EXPECT_EQ(floorPowerOfTwo(1), 1U);
    EXPECT_EQ(floorPowerOfTwo(100000), 65536U);
}

TEST(Fft, KeepsEnergyParseval) {
    std::mt19937 random(11);
    std::normal_distribution<double> normal(0.3, 2.0);
    const std::size_t n = 4096;
    std::vector<std::complex<double>> x(n);
    double time = 0.0;
    for (auto& value : x) {
        value = {normal(random), 0.0};
        time += std::norm(value);
    }
    ASSERT_TRUE(fft(x));
    double frequency = 0.0;
    for (const auto& value : x) {
        frequency += std::norm(value);
    }
    EXPECT_NEAR(frequency / static_cast<double>(n), time, 1e-9 * time);
}

// --- spectrum -----------------------------------------------------------------------------

TEST(Spectrum, ASinePeaksAtItsFrequencyWithItsAmplitude) {
    // 8192 samples at 10 kHz: bins of 1.2207 Hz. 250 Hz is between bins, 1220.703125 on one.
    for (const SpectrumWindow window : {SpectrumWindow::Hann, SpectrumWindow::Rectangular}) {
        const double onBin = 1000.0 * 10000.0 / 8192.0;
        const Spectrum s = spectrum(sine(8192, 3.0, onBin, 5.0), kDtSeconds, window);
        ASSERT_EQ(s.samples, 8192U);
        EXPECT_NEAR(s.binHz, 10000.0 / 8192.0, 1e-12);
        EXPECT_NEAR(s.dc, 5.0, 1e-9);
        EXPECT_EQ(s.peak, 1000U);
        EXPECT_NEAR(s.peakHz, onBin, 1e-6 * s.binHz);
        EXPECT_NEAR(s.peakAmplitude, 3.0, 1e-9);
        EXPECT_NEAR(s.hz.back(), 5000.0, 1e-9);  // Nyquist
    }
    const Spectrum between = spectrum(sine(8192, 1.0, 250.0), kDtSeconds, SpectrumWindow::Hann);
    EXPECT_NEAR(between.peakHz, 250.0, 0.1 * between.binHz);
    EXPECT_GT(between.peakAmplitude, 0.84);  // Hann scalloping loss is at most 1.42 dB
}

TEST(Spectrum, AmplitudesKeepTheEnergyOfTheSignal) {
    // Rectangular window: the sum of the bins' powers (a^2 / 2 each) is the variance.
    std::mt19937 random(3);
    std::normal_distribution<double> normal(1.0, 0.5);
    std::vector<double> values(2048);
    for (double& value : values) {
        value = normal(random);
    }
    const Spectrum s = spectrum(values, kDtSeconds, SpectrumWindow::Rectangular);
    double variance = 0.0;
    for (const double value : values) {
        variance += (value - s.dc) * (value - s.dc);
    }
    variance /= static_cast<double>(values.size());
    double power = 0.0;
    for (std::size_t k = 1; k < s.amplitude.size(); ++k) {
        const double a = s.amplitude[k];
        power += (k == s.amplitude.size() - 1) ? a * a : a * a / 2.0;
    }
    EXPECT_NEAR(power, variance, 1e-9 * variance);
}

TEST(Spectrum, TakesTheNewestPowerOfTwoAndFillsGaps) {
    std::vector<double> values = sine(10000, 2.0, 50.0);
    values[9000] = std::nan("");
    const Spectrum s = spectrum(values, kDtSeconds, SpectrumWindow::Hann);
    EXPECT_EQ(s.samples, 8192U);
    EXPECT_NEAR(s.peakHz, 50.0, 0.2);
    EXPECT_NEAR(s.peakAmplitude, 2.0, 0.05);
    EXPECT_EQ(spectrum(std::vector<double>(5, 1.0), kDtSeconds, SpectrumWindow::Hann).samples, 0U);
}

TEST(Spectrum, OfABufferReadsTheRawSamplesOfTheWindow) {
    ScopeBuffer buffer(kDt);
    // 2 s of a 10 kHz current: 120 Hz at 4 A on a 1 A offset.
    push(buffer, 0, sine(20000, 4.0, 120.0, 1.0));
    const Spectrum s = spectrumOf(buffer, 0, 2'000'000'000, SpectrumWindow::Hann, 16384);
    EXPECT_EQ(s.samples, 16384U);
    // 1.6384 s is not a whole number of periods: the mean is near the offset, and 120 Hz falls
    // between bins (196.6), so the Hann window shows it at most 1.42 dB low.
    EXPECT_NEAR(s.dc, 1.0, 0.01);
    EXPECT_NEAR(s.peakHz, 120.0, 0.1 * s.binHz);
    EXPECT_GT(s.peakAmplitude, 4.0 * 0.84);
    EXPECT_LE(s.peakAmplitude, 4.0);
    EXPECT_EQ(spectrumOf(buffer, 5, 5, SpectrumWindow::Hann).samples, 0U);
}

// --- measure ------------------------------------------------------------------------------

TEST(Measure, GivesTheStatisticsBetweenTheCursors) {
    ScopeBuffer buffer(kDt);
    // 1 s of a 10 Hz sine of amplitude 2 on an offset of 1, then a ramp.
    std::vector<double> values = sine(10000, 2.0, 10.0, 1.0);
    for (std::size_t i = 0; i < 1000; ++i) {
        values.push_back(static_cast<double>(i) * 0.001);  // 10 per second
    }
    push(buffer, 0, values);
    // Whole periods: mean 1, rms sqrt(1 + 2^2 / 2), p-p 4.
    const Measurement m = measure(buffer, 0, 999'900'000);
    EXPECT_EQ(m.count, 10000U);
    EXPECT_NEAR(m.mean, 1.0, 1e-9);
    EXPECT_NEAR(m.rms, std::sqrt(3.0), 1e-9);
    EXPECT_NEAR(m.min, -1.0, 1e-6);
    EXPECT_NEAR(m.max, 3.0, 1e-6);
    EXPECT_NEAR(m.peakToPeak, 4.0, 1e-6);
    EXPECT_NEAR(m.seconds, 0.9999, 1e-12);
    // On the ramp, from B back to A: the slope is the ramp's, whichever cursor is first.
    const Measurement ramp = measure(buffer, 1'050'000'000, 1'010'000'000);
    EXPECT_EQ(ramp.count, 401U);
    EXPECT_NEAR(ramp.delta, -0.4, 1e-9);
    EXPECT_NEAR(ramp.slope, 10.0, 1e-9);
    EXPECT_NEAR(ramp.mean, 0.3, 1e-9);
    // A cursor outside what is held: no delta, but the samples there are measured.
    const Measurement outside = measure(buffer, 1'050'000'000, 5'000'000'000);
    EXPECT_TRUE(std::isnan(outside.delta));
    EXPECT_TRUE(std::isnan(outside.slope));
    EXPECT_EQ(outside.count, 500U);
}

// --- saved sets ---------------------------------------------------------------------------

TEST(SignalSets, RoundTripThroughTheFile) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("lagrange_signal_sets_" +
         std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + ".tsv");
    std::vector<SignalSet> sets;
    storeSignalSet(sets, {"drive train", {{"joint/left_wheel/velocity", 0}, {"motor/left/iq", 0}}});
    storeSignalSet(sets, {"pose\tz", {{"body/base_link/pose", 2}}});
    EXPECT_EQ(storeSignalSet(sets, {"drive train", {{"motor/right/iq", 0}}}), 0U);  // replaced
    ASSERT_TRUE(saveSignalSets(path.string(), sets));
    {
        std::ofstream damaged(path, std::ios::app);
        damaged << "broken line\n" << "x\ty\tnot a number\n";
    }
    const std::vector<SignalSet> read = loadSignalSets(path.string());
    ASSERT_EQ(read.size(), 2U);
    EXPECT_EQ(read[0].name, "drive train");
    ASSERT_EQ(read[0].signals.size(), 1U);
    EXPECT_EQ(read[0].signals[0].first, "motor/right/iq");
    EXPECT_EQ(read[1].name, "pose z");
    EXPECT_EQ(read[1].signals[0].second, 2U);
    std::filesystem::remove(path);
    EXPECT_TRUE(loadSignalSets(path.string()).empty());
}

}  // namespace
