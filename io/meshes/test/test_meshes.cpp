#include "lagrange/meshes/MeshImport.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::meshes {
namespace {

std::string fixture(const char* name) {
    return std::string(LAGRANGE_TEST_MESH_DIR) + "/" + name;
}

struct Bounds {
    std::array<float, 3> lo{1e30F, 1e30F, 1e30F}, hi{-1e30F, -1e30F, -1e30F};
};

Bounds boundsOf(const ImportedMesh& mesh) {
    Bounds b;
    for (const MeshPart& part : mesh.parts) {
        for (std::size_t v = 0; v + 2 < part.positions.size(); v += 3) {
            for (std::size_t i = 0; i < 3; ++i) {
                b.lo.at(i) = std::min(b.lo.at(i), part.positions[v + i]);
                b.hi.at(i) = std::max(b.hi.at(i), part.positions[v + i]);
            }
        }
    }
    return b;
}

void expectWellFormed(const MeshPart& part) {
    EXPECT_EQ(part.positions.size(), part.normals.size());
    EXPECT_EQ(part.indices.size() % 3, 0U);
    for (const std::uint32_t index : part.indices) {
        EXPECT_LT(index, part.positions.size() / 3);
    }
}

TEST(MeshImport, ObjGivesOnePartPerMaterialWithItsColour) {
    const auto mesh = importMesh(fixture("cube.obj"));
    ASSERT_TRUE(mesh) << mesh.error();
    ASSERT_EQ(mesh->parts.size(), 2U);
    for (const MeshPart& part : mesh->parts) {
        expectWellFormed(part);
        ASSERT_TRUE(part.hasColor) << part.material;
        if (part.material == "top") {
            EXPECT_EQ(part.indices.size(), 2U * 3U);  // one quad, triangulated
            EXPECT_EQ(part.rgba, (std::array<float, 4>{1.0F, 0.0F, 0.0F, 1.0F}));
        } else {
            EXPECT_EQ(part.material, "side");
            EXPECT_EQ(part.indices.size(), 10U * 3U);
            EXPECT_FLOAT_EQ(part.rgba[0], 0.5F);
        }
    }
    const Bounds b = boundsOf(*mesh);
    EXPECT_EQ(b.lo, (std::array<float, 3>{-0.5F, -0.5F, -0.5F}));
    EXPECT_EQ(b.hi, (std::array<float, 3>{0.5F, 0.5F, 0.5F}));
}

TEST(MeshImport, StlTrianglesKeepTheirUnits) {
    const auto mesh = importMesh(fixture("tetra.stl"));
    ASSERT_TRUE(mesh) << mesh.error();
    ASSERT_EQ(mesh->parts.size(), 1U);
    expectWellFormed(mesh->parts[0]);
    EXPECT_EQ(mesh->parts[0].indices.size(), 4U * 3U);
    // STL has no unit: millimetres stay 10, and the URDF's scale makes them metres.
    EXPECT_EQ(boundsOf(*mesh).hi, (std::array<float, 3>{10.0F, 10.0F, 10.0F}));
}

TEST(MeshImport, GltfBakesTheNodeTransformAndUsesTheBaseColour) {
    const auto mesh = importMesh(fixture("triangle.gltf"));
    ASSERT_TRUE(mesh) << mesh.error();
    ASSERT_EQ(mesh->parts.size(), 1U);
    const MeshPart& part = mesh->parts[0];
    expectWellFormed(part);
    EXPECT_EQ(part.indices.size(), 3U);
    EXPECT_TRUE(part.hasColor);
    EXPECT_NEAR(part.rgba[1], 0.8F, 1e-6F);
    EXPECT_NEAR(part.rgba[2], 0.2F, 1e-6F);
    const Bounds b = boundsOf(*mesh);
    EXPECT_FLOAT_EQ(b.lo[0], 2.0F);  // the node's translation
    EXPECT_FLOAT_EQ(b.hi[0], 3.0F);
    // Normals were generated: the triangle lies in z = 0.
    EXPECT_NEAR(std::abs(part.normals[2]), 1.0F, 1e-6F);
}

TEST(MeshImport, ColladaUnitsBecomeMetresAndZUpStaysUp) {
    const auto mesh = importMesh(fixture("plate.dae"));
    ASSERT_TRUE(mesh) << mesh.error();
    ASSERT_EQ(mesh->parts.size(), 1U);
    expectWellFormed(mesh->parts[0]);
    EXPECT_TRUE(mesh->parts[0].hasColor);
    EXPECT_FLOAT_EQ(mesh->parts[0].rgba[2], 1.0F);
    const Bounds b = boundsOf(*mesh);
    EXPECT_NEAR(b.hi[0], 1.0F, 1e-6F);  // 100 cm
    EXPECT_NEAR(b.hi[1], 0.0F, 1e-6F);
    EXPECT_NEAR(b.hi[2], 0.5F, 1e-6F);  // 50 cm, still along z
}

// A textured part is split by its texture's colours: the triangle on the red texel and the one
// on the blue texel become two parts of those colours (PNG and JPEG textures).
TEST(MeshImport, TexturesBakeIntoPartsOfTheirColours) {
    for (const char* file : {"textured_quad.obj", "textured_quad_jpeg.obj"}) {
        SCOPED_TRACE(file);
        const auto mesh = importMesh(fixture(file));
        ASSERT_TRUE(mesh.has_value()) << mesh.error();
        ASSERT_EQ(mesh->parts.size(), 2U);
        bool red = false;
        bool blue = false;
        for (const MeshPart& part : mesh->parts) {
            expectWellFormed(part);
            EXPECT_EQ(part.indices.size(), 3U);
            EXPECT_TRUE(part.hasColor);
            red = red || (part.rgba[0] > 0.8F && part.rgba[2] < 0.2F);
            blue = blue || (part.rgba[2] > 0.8F && part.rgba[0] < 0.2F);
        }
        EXPECT_TRUE(red);
        EXPECT_TRUE(blue);
    }
}

TEST(MeshImport, MissingAndBrokenFilesAreErrors) {
    const auto missing = importMesh(fixture("nothing.stl"));
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("cannot read"), std::string::npos);
    const auto broken = importMesh(fixture("cube.mtl"));  // not a mesh
    ASSERT_FALSE(broken);
}

TEST(MeshImport, FetchedAssetsResolveInTheAssetDirectory) {
    const std::string dir = LAGRANGE_TEST_MESH_DIR;
    ASSERT_EQ(setenv("LAGRANGE_ASSET_DIR", dir.c_str(), 1), 0);
    const auto found = resolveMeshUri("lagrange-asset://cube.obj", "/elsewhere");
    ASSERT_TRUE(found.has_value()) << found.error();
    EXPECT_EQ(*found, dir + "/cube.obj");
    const auto missing = resolveMeshUri("lagrange-asset://not_fetched/body.dae", "/elsewhere");
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().find("fetch_assets"), std::string::npos);
    unsetenv("LAGRANGE_ASSET_DIR");
}

TEST(MeshImport, UrisResolveRelativeFileAndPackage) {
    EXPECT_EQ(*resolveMeshUri("meshes/a.stl", "/robots/arm"), "/robots/arm/meshes/a.stl");
    EXPECT_EQ(*resolveMeshUri("../b.dae", "/robots/arm"), "/robots/b.dae");
    EXPECT_EQ(*resolveMeshUri("file:///abs/c.obj", "/robots/arm"), "/abs/c.obj");
    const auto unknown = resolveMeshUri("package://no_such_package_here/m.stl", "/x");
    ASSERT_FALSE(unknown);
    EXPECT_NE(unknown.error().find("no_such_package_here"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::meshes
