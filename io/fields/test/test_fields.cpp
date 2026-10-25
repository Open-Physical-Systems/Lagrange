// Fields against analytic ones (roadmap §6, fields infrastructure: "analytic-field tests"):
// written to .nvdb, read back and sampled; trilinear interpolation is exact for linear fields.
#include "lagrange/fields/Field.hpp"
#include "lagrange/fields/Foam.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

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

/// A linear wind: trilinear sampling and voxel means reproduce it exactly.
Vec3 linearWind(const Vec3& p) {
    return {4.0 + 0.1 * p[0] - 0.05 * p[1], 0.2 * p[1], 0.3 + 0.02 * p[2]};
}

/// Writes an ASCII volVectorField as OpenFOAM does: header, comments, the internal field, an
/// (ignored) boundary field.
void writeFoam(const std::string& path, const char* object, const std::vector<Vec3>& values) {
    std::ofstream out(path);
    out << "/*--------------------------------*- C++ -*----------------------------------*\\\n"
           "  =========                 |\n"
           "\\*---------------------------------------------------------------------------*/\n"
           "FoamFile\n{\n    version     2.0;\n    format      ascii;\n"
           "    class       volVectorField;\n    location    \"100\";\n    object      "
        << object << ";\n}\n// * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n"
        << "dimensions      [0 1 -1 0 0 0 0];\n\ninternalField   nonuniform List<vector> \n"
        << values.size() << "\n(\n";
    out.precision(17);
    for (const Vec3& v : values) {
        out << "(" << v[0] << " " << v[1] << " " << v[2] << ")\n";
    }
    out << ")\n;\n\nboundaryField\n{\n    inlet\n    {\n        type            fixedValue;\n"
           "        value           uniform (4 0 0);\n    }\n}\n";
}

/// Cell centres of a block of cubic cells of size h, the first centre at `first`.
std::vector<Vec3> cellCentres(const Vec3& first, double h, int nx, int ny, int nz) {
    std::vector<Vec3> out;
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                out.push_back({first[0] + i * h, first[1] + j * h, first[2] + k * h});
            }
        }
    }
    return out;
}

TEST(Foam, AnAnalyticWindRoundTripsThroughOpenFoamFiles) {
    // Cells of 0.5 m on the voxel lattice: one cell per voxel.
    const std::vector<Vec3> centres = cellCentres({-3.0, -2.0, 0.5}, 0.5, 13, 9, 7);
    std::vector<Vec3> u;
    for (const Vec3& c : centres) {
        u.push_back(linearWind(c));
    }
    writeFoam(temp("C"), "C", centres);
    writeFoam(temp("U"), "U", u);
    const auto readC = readFoamVectorField(temp("C"));
    const auto readU = readFoamVectorField(temp("U"));
    ASSERT_TRUE(readC) << readC.error();
    ASSERT_TRUE(readU) << readU.error();
    ASSERT_EQ(readC->size(), centres.size());
    EXPECT_EQ((*readU)[17], u[17]);
    const auto report = foamToWindField(*readC, *readU, temp("foam.nvdb"), "wind", 0.5);
    ASSERT_TRUE(report) << report.error();
    EXPECT_EQ(report->voxels, centres.size());
    const auto field = Field::load(temp("foam.nvdb"));
    ASSERT_TRUE(field) << field.error();
    EXPECT_EQ((*field)->kind(), Field::Kind::Vector);
    for (const Vec3& p : {Vec3{0.0, 0.0, 1.0}, Vec3{-1.3, 1.1, 2.2}, Vec3{2.6, -1.7, 3.4}}) {
        const Vec3 got = (*field)->sampleVector(p);
        const Vec3 want = linearWind(p);
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(got[k], want[k], 1e-5) << p[0] << " " << p[1] << " " << p[2];
        }
    }
}

TEST(Foam, CellsAreAveragedIntoCoarserVoxelsAndShifted) {
    // Cells of 0.5 m centred between voxel centres: eight per 1 m voxel, whose mean is the
    // value at the voxel's centre for a linear field; the offset moves the case into the region.
    const std::vector<Vec3> centres = cellCentres({0.25, 0.25, 0.25}, 0.5, 12, 12, 8);
    std::vector<Vec3> u;
    for (const Vec3& c : centres) {
        u.push_back(linearWind(c));
    }
    const Vec3 offset{-10.0, 5.0, 0.0};
    const auto report = foamToWindField(centres, u, temp("coarse.nvdb"), "wind", 1.0, offset);
    ASSERT_TRUE(report) << report.error();
    const auto field = Field::load(temp("coarse.nvdb"));
    ASSERT_TRUE(field);
    for (const Vec3& voxel : {Vec3{2.0, 3.0, 2.0}, Vec3{4.0, 1.0, 3.0}}) {  // interior voxels
        const Vec3 got = (*field)->sampleVector(
            {voxel[0] + offset[0], voxel[1] + offset[1], voxel[2] + offset[2]});
        const Vec3 want = linearWind(voxel);
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_NEAR(got[k], want[k], 1e-5);
        }
    }
}

TEST(Foam, ReadsUniformAndCompactListsAndRefusesBinary) {
    const auto write = [](const std::string& path, const std::string& body,
                          const char* format = "ascii") {
        std::ofstream(path) << "FoamFile { version 2.0; format " << format
                            << "; class volVectorField; object U; }\n"
                            << body << "\nboundaryField { }\n";
    };
    write(temp("U_uniform"), "internalField uniform (3 0 -1);");
    const auto uniform = readFoamVectorField(temp("U_uniform"));
    ASSERT_TRUE(uniform) << uniform.error();
    ASSERT_EQ(uniform->size(), 1U);
    EXPECT_EQ((*uniform)[0], (Vec3{3.0, 0.0, -1.0}));
    write(temp("U_compact"), "internalField nonuniform List<vector> 4{(1 2 3)};");
    const auto compact = readFoamVectorField(temp("U_compact"));
    ASSERT_TRUE(compact) << compact.error();
    EXPECT_EQ(compact->size(), 4U);
    write(temp("U_binary"), "internalField nonuniform List<vector> 0();", "binary");
    const auto binary = readFoamVectorField(temp("U_binary"));
    ASSERT_FALSE(binary);
    EXPECT_NE(binary.error().find("writeFormat ascii"), std::string::npos);
    write(temp("U_short"), "internalField nonuniform List<vector> 3 ( (1 2 3) (4 5 6) );");
    EXPECT_FALSE(readFoamVectorField(temp("U_short")));
}

}  // namespace
}  // namespace lagrange::fields
