// The camera's sensor model (sensors.md §2): lens models against OpenCV's definitions, the
// noise against the linear photon-transfer model of EMVA 1288, vignetting, the rolling shutter's
// skew and auto exposure, on synthetic renders (no GPU).
#include "lagrange/sensors/CameraModel.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

TEST(Lens, PlumbBobMatchesTheBrownConradyFormula) {
    // A point worked by hand from OpenCV's definition (calib3d, "Detailed Description").
    const Lens lens{Lens::Model::PlumbBob, {-0.3, 0.1, 0.001, -0.002, 0.01}};
    const double x = 0.4;
    const double y = -0.25;
    const double r2 = x * x + y * y;
    const double radial = 1 - 0.3 * r2 + 0.1 * r2 * r2 + 0.01 * r2 * r2 * r2;
    const auto d = distort(lens, x, y);
    EXPECT_NEAR(d[0], x * radial + 2 * 0.001 * x * y - 0.002 * (r2 + 2 * x * x), 1e-15);
    EXPECT_NEAR(d[1], y * radial + 0.001 * (r2 + 2 * y * y) + 2 * -0.002 * x * y, 1e-15);
}

TEST(Lens, UndistortInvertsBothModels) {
    const Lens plumb{Lens::Model::PlumbBob, {-0.28, 0.07, 0.0005, -0.0003, 0.0}};
    const Lens fisheye{Lens::Model::Equidistant, {0.05, -0.01, 0.002, -0.0005, 0.0}};
    for (const Lens& lens : {plumb, fisheye}) {
        for (double x = -0.6; x <= 0.6; x += 0.15) {
            for (double y = -0.45; y <= 0.45; y += 0.15) {
                const auto d = distort(lens, x, y);
                const auto u = undistort(lens, d[0], d[1]);
                ASSERT_TRUE(u);
                EXPECT_NEAR((*u)[0], x, 1e-8);
                EXPECT_NEAR((*u)[1], y, 1e-8);
            }
        }
    }
    // The equidistant model maps the angle: θ_d = θ for zero coefficients, so r_d = atan(r).
    const Lens ideal{Lens::Model::Equidistant, {}};
    EXPECT_NEAR(distort(ideal, 1.0, 0.0)[0], std::atan(1.0), 1e-15);
}

TEST(Noise, FollowsThePhotonTransferCurve) {
    // EMVA 1288's linear model: variance in e⁻ = mean (shot noise) + read noise².
    Exposure e;
    e.fullWell = 10000.0;
    e.readNoise = 5.0;
    e.bitDepth = 16;
    const double fullScale = 65535.0;
    for (const double electrons : {100.0, 2500.0}) {
        random::Stream rng(7, 1, 0);
        constexpr int kSamples = 40000;
        double sum = 0.0;
        double squares = 0.0;
        for (int i = 0; i < kSamples; ++i) {
            const double v = digitise(electrons, e, rng) * e.fullWell / fullScale;
            sum += v;
            squares += v * v;
        }
        const double mean = sum / kSamples;
        const double variance = squares / kSamples - mean * mean;
        EXPECT_NEAR(mean, electrons, 0.5);
        EXPECT_NEAR(variance / (electrons + 25.0), 1.0, 0.03) << electrons;
    }
    // The full well clips; the gain scales.
    random::Stream rng(7, 2, 0);
    e.readNoise = 0.0;
    EXPECT_EQ(digitise(1e6, e, rng), 65535U);
    e.gain = 2.0;
    EXPECT_EQ(digitise(6000.0, e, rng), 65535U);
}

/// A flat render of one display value, with a depth and an instance id.
gpu::RasterImages flat(std::uint32_t w, std::uint32_t h, std::uint8_t value) {
    gpu::RasterImages images;
    images.rgba.assign(static_cast<std::size_t>(w) * h * 4, value);
    images.depth.assign(static_cast<std::size_t>(w) * h, 5.0F);
    images.instance.assign(static_cast<std::size_t>(w) * h, 3U);
    return images;
}

CameraModelConfig pinhole(std::uint32_t w, std::uint32_t h, double fx) {
    CameraModelConfig c;
    c.width = w;
    c.height = h;
    c.fx = fx;
    c.fy = fx;
    c.cx = (w - 1.0) / 2.0;
    c.cy = (h - 1.0) / 2.0;
    return c;
}

TEST(CameraModel, VignettingFollowsCosineToTheFourth) {
    CameraModelConfig c = pinhole(101, 101, 50.0);
    c.vignetting = 1.0;
    CameraModel model(c);
    ASSERT_TRUE(model.active());
    gpu::RasterImages out;
    model.apply(flat(101, 101, 255), {}, 1, 0, 0, out);
    const auto value = [&](std::uint32_t u, std::uint32_t v) {
        return std::pow(out.rgba[(static_cast<std::size_t>(v) * 101 + u) * 4] / 255.0, 2.2);
    };
    EXPECT_NEAR(value(50, 50), 1.0, 1e-9);
    // The corner ray: x = y = 1 in normalised units, cos²θ = 1/3.
    EXPECT_NEAR(value(100, 100), 1.0 / 9.0, 0.01);
    EXPECT_FLOAT_EQ(out.depth[50 * 101 + 50], 5.0F);
    EXPECT_EQ(out.instance[0], 3U);
}

TEST(CameraModel, BarrelLensRendersWiderAndKeepsTheCentre) {
    CameraModelConfig c = pinhole(320, 240, 200.0);
    c.lens = {Lens::Model::PlumbBob, {-0.3, 0.08, 0.0, 0.0, 0.0}};
    CameraModel model(c);
    EXPECT_GT(model.renderCamera().width, 320U);
    EXPECT_GT(model.renderCamera().height, 240U);
    EXPECT_LE(model.renderCamera().width, 640U);
    // A render with a bright vertical line through the principal point: it stays straight
    // through the centre (radial distortion keeps lines through the centre straight).
    const gpu::PinholeCamera& r = model.renderCamera();
    gpu::RasterImages render = flat(r.width, r.height, 0);
    const auto centre = static_cast<std::uint32_t>(std::lround(r.cx));
    for (std::uint32_t v = 0; v < r.height; ++v) {
        for (int ch = 0; ch < 3; ++ch) {
            render.rgba[(static_cast<std::size_t>(v) * r.width + centre) * 4 + ch] = 255;
        }
    }
    gpu::RasterImages out;
    model.apply(render, {}, 1, 0, 0, out);
    for (std::uint32_t v = 0; v < 240; v += 20) {
        std::uint32_t brightest = 0;
        std::uint8_t best = 0;
        for (std::uint32_t u = 0; u < 320; ++u) {
            const std::uint8_t value = out.rgba[(static_cast<std::size_t>(v) * 320 + u) * 4];
            if (value > best) {
                best = value;
                brightest = u;
            }
        }
        EXPECT_NEAR(brightest, 159.5 - (r.cx - centre), 0.6) << v;
    }
}

TEST(CameraModel, RollingShutterSkewsAVerticalLineByThePanDuringReadout) {
    CameraModelConfig c = pinhole(200, 100, 100.0);
    c.rollingShutterTime = 0.03;  // s
    CameraModel model(c);
    const gpu::PinholeCamera& r = model.renderCamera();
    gpu::RasterImages render = flat(r.width, r.height, 0);
    const auto centre = static_cast<std::uint32_t>(std::lround(r.cx));
    for (std::uint32_t v = 0; v < r.height; ++v) {
        render.rgba[(static_cast<std::size_t>(v) * r.width + centre) * 4] = 255;
    }
    const double line = 99.5 - (r.cx - centre);  // the line in output pixels
    // Panning right (about the optical frame's +y, which points down) at 2 rad/s.
    const double omega = 2.0;
    gpu::RasterImages out;
    model.apply(render, {0.0, omega, 0.0}, 1, 0, 0, out);
    const auto lineAt = [&](std::uint32_t v) {
        double weighted = 0.0;
        double total = 0.0;
        for (std::uint32_t u = 0; u < 200; ++u) {
            const double value =
                std::pow(out.rgba[(static_cast<std::size_t>(v) * 200 + u) * 4] / 255.0, 2.2);
            weighted += value * u;
            total += value;
        }
        return weighted / total;
    };
    // The last row is exposed at the frame's time; the first 30 ms earlier, when the camera
    // pointed left of the line by ωτ, so it sees the line to the right by fx·tan(ωτ).
    EXPECT_NEAR(lineAt(99), line, 0.2);
    EXPECT_NEAR(lineAt(0), line + 100.0 * std::tan(omega * 0.03), 0.3);
    EXPECT_NEAR(lineAt(50), line + 100.0 * std::tan(omega * 0.03 * 49.0 / 99.0), 0.3);
}

TEST(CameraModel, ExposureBrightnessAndAutoExposure) {
    CameraModelConfig c = pinhole(64, 48, 50.0);
    Exposure e;
    e.time = 0.01;
    e.electronRate = 1e6;  // 10 000 e⁻ at linear 1: the full well
    e.readNoise = 0.0;
    e.bitDepth = 12;
    c.exposure = e;
    CameraModel model(c);
    gpu::RasterImages out;
    // A mid-grey surface (display 128, linear 0.2195) keeps its brightness on average.
    ASSERT_EQ(model.renderCamera().width, 64U);  // no lens, no rolling shutter: no margin
    model.apply(flat(64, 48, 128), {}, 1, 0, 0, out);
    double mean = 0.0;
    for (std::size_t i = 0; i < out.rgba.size(); i += 4) {
        mean += out.rgba[i];
    }
    mean /= static_cast<double>(out.rgba.size() / 4);
    EXPECT_NEAR(mean, 128.0, 1.0);
    // Auto exposure: a dim scene (display 40) lengthens the exposure until the mean is 18 %.
    c.exposure->autoExposure = true;
    c.exposure->time = 1.0;  // the longest
    CameraModel autoModel(c);
    autoModel.setExposureTime(0.001);
    for (int frame = 0; frame < 4; ++frame) {
        autoModel.apply(flat(64, 48, 40), {}, 1, frame, 0, out);
    }
    const double linear = std::pow(40.0 / 255.0, 2.2);
    EXPECT_NEAR(autoModel.exposureTime() * 1e6 * linear / 10000.0, kAutoExposureTarget, 0.01);
}

}  // namespace
}  // namespace lagrange::sensors
