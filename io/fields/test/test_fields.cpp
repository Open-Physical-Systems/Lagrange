// Fields against analytic ones (roadmap §6, fields infrastructure: "analytic-field tests"):
// written to .nvdb, read back and sampled; trilinear interpolation is exact for linear fields.
#include "lagrange/fields/Field.hpp"

#include <cmath>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::fields {
namespace {

std::string temp(const std::string& name) {
    return (std::filesystem::path(testing::TempDir()) / name).string();
}

double linear(const Vec3& p) {
    return 0.05 + 0.002 * p[0] - 0.001 * p[1] + 0.003 * p[2];
}

TEST(Fields, AScalarFieldRoundTripsAndInterpolatesExactly) {
    const DenseBox box{{-2.0, -2.0, 0.0}, 0.5, {9, 9, 7}};  // -2..2, -2..2, 0..3 m
    ASSERT_TRUE(writeScalarField(temp("fog.nvdb"), "extinction", box, linear));
    const auto loaded = Field::load(temp("fog.nvdb"));
    ASSERT_TRUE(loaded) << loaded.error();
    const Field& f = **loaded;
    EXPECT_EQ(f.kind(), Field::Kind::Scalar);
    EXPECT_EQ(f.name(), "extinction");
    EXPECT_DOUBLE_EQ(f.voxelSize(), 0.5);
    EXPECT_NEAR(f.bounds()[0][0], -2.0, 1e-9);
    EXPECT_NEAR(f.bounds()[1][2], 3.0, 1e-9);
    for (const Vec3& p : {Vec3{0.13, -0.71, 1.37}, Vec3{-1.9, 1.2, 0.05}, Vec3{1.5, 1.5, 2.9}}) {
        EXPECT_NEAR(f.sampleScalar(p), linear(p), 1e-6) << p[0] << " " << p[1] << " " << p[2];
    }
    EXPECT_EQ(f.sampleScalar({10.0, 0.0, 1.0}), 0.0);  // outside: the background
    // Optical depth along a segment: the integral of a linear field is exact by the midpoint rule.
    const Vec3 a{-1.0, 0.0, 1.0};
    const Vec3 b{1.0, 1.0, 2.0};
    const double length = std::sqrt(4.0 + 1.0 + 1.0);
    const Vec3 mid{0.0, 0.5, 1.5};
    EXPECT_NEAR(f.integrate(a, b), linear(mid) * length, 1e-5);
    // The bytes are the grid: a field made from them is the same field.
    const auto copy = Field::fromBytes(f.bytes());
    ASSERT_TRUE(copy);
    EXPECT_NEAR(copy->sampleScalar({0.13, -0.71, 1.37}), f.sampleScalar({0.13, -0.71, 1.37}), 0.0);
}

TEST(Fields, AVectorFieldIsAWindShear) {
    const auto wind = [](const Vec3& p) { return Vec3{2.0 + 0.5 * p[2], -0.3 * p[0], 0.1}; };
    const DenseBox box{{-5.0, -5.0, 0.0}, 1.0, {11, 11, 11}};
    ASSERT_TRUE(writeVectorField(temp("wind.nvdb"), "wind", box, wind));
    const auto loaded = Field::load(temp("wind.nvdb"));
    ASSERT_TRUE(loaded) << loaded.error();
    EXPECT_EQ((*loaded)->kind(), Field::Kind::Vector);
    const Vec3 p{1.25, -3.5, 4.75};
    const Vec3 v = (*loaded)->sampleVector(p);
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(v[k], wind(p)[k], 1e-5) << k;
    }
    EXPECT_NEAR((*loaded)->sampleScalar(p), std::hypot(v[0], v[1], v[2]), 1e-12);
}

TEST(Fields, AMissingFileIsAnError) {
    const auto loaded = Field::load(temp("missing.nvdb"));
    ASSERT_FALSE(loaded);
    EXPECT_NE(loaded.error().find("missing.nvdb"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::fields
