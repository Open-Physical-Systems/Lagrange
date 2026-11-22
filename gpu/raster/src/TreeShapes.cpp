#include "lagrange/gpu/TreeShapes.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace lagrange::gpu {
namespace {

// The shader's hash and random numbers (realistic.slang vegHash, vegRandom), bit for bit.
std::uint32_t vegHash(std::uint32_t x) {
    x ^= x >> 16U;
    x *= 0x7feb352dU;
    x ^= x >> 15U;
    x *= 0x846ca68bU;
    x ^= x >> 16U;
    return x;
}

float vegRandom(std::uint32_t& state) {
    state = vegHash(state + 0x9e3779b9U);
    return static_cast<float>(state & 0xffffffU) / 16777216.0F;
}

constexpr std::uint32_t kTreeBranches = 7;  // realistic.slang kTreeBranches
constexpr float kGoldenAngle = 2.39996F;    // rad

// realistic.slang treeShape: the numbers vsTree draws a tree with.
struct Shape {
    float height, radius, base, half, trunk, trunkTop;
    std::array<float, 2> lean;
};

Shape shapeOf(const RealisticTree& tree) {
    Shape t{};
    t.height = static_cast<float>(tree.height);
    t.radius = static_cast<float>(tree.crownRadius);
    t.base = static_cast<float>(tree.crownBase);
    t.half = 0.5F * (t.height - t.base);
    t.trunk = 0.5F * std::max(static_cast<float>(tree.trunkDiameter), 0.02F);
    t.trunkTop = t.base + (1.3F * t.half);
    std::uint32_t s = vegHash(tree.seed ^ 0x51ed27U);
    const float a = vegRandom(s);
    const float b = vegRandom(s);
    t.lean = {(a - 0.5F) * 0.12F * t.radius, (b - 0.5F) * 0.12F * t.radius};
    return t;
}

std::array<double, 3> turn(const std::array<float, 4>& q, const std::array<double, 3>& v) {
    const double w = q[0];
    const std::array<double, 3> u{q[1], q[2], q[3]};
    const auto cross = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return std::array<double, 3>{(a[1] * b[2]) - (a[2] * b[1]), (a[2] * b[0]) - (a[0] * b[2]),
                                     (a[0] * b[1]) - (a[1] * b[0])};
    };
    std::array<double, 3> t = cross(u, v);
    for (std::size_t k = 0; k < 3; ++k) {
        t.at(k) += w * v.at(k);
    }
    const std::array<double, 3> c = cross(u, t);
    return {v[0] + (2.0 * c[0]), v[1] + (2.0 * c[1]), v[2] + (2.0 * c[2])};
}

// A point of the tree (its own frame over the foot) in the region frame, swayed as vsTree does.
std::array<double, 3> place(const RealisticTree& tree, const Shape& t,
                            const std::array<double, 3>& local) {
    const double bend = std::pow(std::max(local[2], 0.0) / (t.base + t.half), 1.5);
    const std::array<double, 3> r = turn(tree.rotation, local);
    return {tree.foot[0] + r[0] + (tree.sway[0] * bend),
            tree.foot[1] + r[1] + (tree.sway[1] * bend),
            tree.foot[2] + r[2] + (tree.sway[2] * bend)};
}

struct Meshes {
    std::vector<float> spherePositions, sphereNormals;
    std::vector<std::uint32_t> sphereIndices;
    std::vector<float> trunkPositions, trunkNormals;
    std::vector<std::uint32_t> trunkIndices;
    std::array<MeshSource, kTreeMeshCount> sources;

    Meshes() {
        // A UV sphere of radius 1: low-poly, it is a clump among 28.
        constexpr std::uint32_t kRings = 8;
        constexpr std::uint32_t kSegments = 12;
        for (std::uint32_t i = 0; i <= kRings; ++i) {
            const double theta = std::numbers::pi * i / kRings;
            for (std::uint32_t j = 0; j <= kSegments; ++j) {
                const double phi = 2.0 * std::numbers::pi * j / kSegments;
                const auto x = static_cast<float>(std::sin(theta) * std::cos(phi));
                const auto y = static_cast<float>(std::sin(theta) * std::sin(phi));
                const auto z = static_cast<float>(std::cos(theta));
                spherePositions.insert(spherePositions.end(), {x, y, z});
                sphereNormals.insert(sphereNormals.end(), {x, y, z});
            }
        }
        for (std::uint32_t i = 0; i < kRings; ++i) {
            for (std::uint32_t j = 0; j < kSegments; ++j) {
                const std::uint32_t a = (i * (kSegments + 1)) + j;
                const std::uint32_t b = a + kSegments + 1;
                sphereIndices.insert(sphereIndices.end(), {a, b, a + 1, a + 1, b, b + 1});
            }
        }
        // The trunk: a frustum from radius 1 at z = 0 to kTreeTrunkTaper at z = 1, open ends.
        constexpr std::uint32_t kSides = 10;
        const double slope = 1.0 - kTreeTrunkTaper;  // radius lost per unit of height
        const double norm = std::sqrt(1.0 + (slope * slope));
        for (std::uint32_t j = 0; j <= kSides; ++j) {
            const double phi = 2.0 * std::numbers::pi * j / kSides;
            const auto c = static_cast<float>(std::cos(phi));
            const auto s = static_cast<float>(std::sin(phi));
            const auto nx = static_cast<float>(std::cos(phi) / norm);
            const auto ny = static_cast<float>(std::sin(phi) / norm);
            const auto nz = static_cast<float>(slope / norm);
            trunkPositions.insert(trunkPositions.end(),
                                  {c, s, 0.0F, c * kTreeTrunkTaper, s * kTreeTrunkTaper, 1.0F});
            trunkNormals.insert(trunkNormals.end(), {nx, ny, nz, nx, ny, nz});
        }
        for (std::uint32_t j = 0; j < kSides; ++j) {
            const std::uint32_t a = 2 * j;
            trunkIndices.insert(trunkIndices.end(), {a, a + 2, a + 1, a + 1, a + 2, a + 3});
        }
        sources = {MeshSource{spherePositions, sphereNormals, sphereIndices},
                   MeshSource{trunkPositions, trunkNormals, trunkIndices}};
    }
};

// Drawing choices, linear RGB: TODO(verify) against photographs of broadleaf crowns and bark.
constexpr std::array<float, 3> kLeafGreen{0.075F, 0.16F, 0.045F};
constexpr std::array<float, 3> kBark{0.16F, 0.10F, 0.06F};

}  // namespace

std::span<const MeshSource> treeMeshSources() {
    static const Meshes meshes;
    return meshes.sources;
}

std::array<float, 4> treeClump(const RealisticTree& tree, std::uint32_t k) {
    const Shape t = shapeOf(tree);
    std::uint32_t s = vegHash((tree.seed * 2654435761U) + (k * 0x9e3779b9U) + 17U);
    float az = 0.0F;
    float up = 0.0F;
    float reach = 0.0F;
    if (k < kTreeBranches) {
        az = (static_cast<float>(k) + (0.35F * vegRandom(s))) * kGoldenAngle;
        up = -0.35F +
             (1.05F * (static_cast<float>(k) + vegRandom(s)) / static_cast<float>(kTreeBranches));
        reach = 0.6F + (0.15F * vegRandom(s));
    } else {
        const auto i = static_cast<float>(k - kTreeBranches);
        const auto n = static_cast<float>(kTreeClumpCount - kTreeBranches);
        az = (i * kGoldenAngle) + (vegRandom(s) * 0.8F);
        const float f = (i + vegRandom(s)) / n;
        up = -0.55F + ((0.92F + 0.55F) * f);
        reach = 0.55F + (0.3F * vegRandom(s));
    }
    const float horizontal = std::sqrt(std::max(1.0F - (up * up), 0.0F));
    const float r = (0.27F + (0.2F * vegRandom(s))) * std::min(t.radius, t.half * 1.3F);
    return {horizontal * std::cos(az) * reach * t.radius,
            horizontal * std::sin(az) * reach * t.radius, up * reach * t.half, r};
}

void appendTreeParts(const RealisticTree& tree, std::uint32_t index,
                     std::span<const RealisticTreeCut> cuts, std::uint32_t firstMesh,
                     std::uint32_t id, std::vector<RasterInstance>& out) {
    if (!(tree.height > 0.0) || !(tree.crownRadius > 0.0) || !(tree.crownBase < tree.height)) {
        return;
    }
    const Shape t = shapeOf(tree);
    // The trunk: from the foot to its top, leaning and bent by the sway as vsTree's.
    const std::array<double, 3> foot = place(tree, t, {0.0, 0.0, 0.0});
    const std::array<double, 3> top = place(tree, t, {t.lean[0], t.lean[1], t.trunkTop});
    const std::array<double, 3> axis{top[0] - foot[0], top[1] - foot[1], top[2] - foot[2]};
    const double length =
        std::sqrt((axis[0] * axis[0]) + (axis[1] * axis[1]) + (axis[2] * axis[2]));
    RasterInstance& trunk = out.emplace_back();
    trunk.mesh = firstMesh + 1;
    trunk.position = foot;
    // The turn of +z onto the axis: q = (1 + cos, z × a) normalised.
    const std::array<double, 3> a{axis[0] / length, axis[1] / length, axis[2] / length};
    std::array<double, 4> q{1.0 + a[2], -a[1], a[0], 0.0};
    const double qn = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]));
    trunk.rotation = qn > 1e-9 ? std::array<double, 4>{q[0] / qn, q[1] / qn, q[2] / qn, 0.0}
                               : std::array<double, 4>{0.0, 1.0, 0.0, 0.0};
    trunk.scale = {t.trunk, t.trunk, static_cast<float>(length)};
    trunk.color = {kBark[0], kBark[1], kBark[2], 1.0F};
    trunk.id = id;
    trunk.roughness = 0.9F;
    // The clumps, each less the share of its volume cut away.
    std::array<float, kTreeClumpCount> cut{};
    std::array<std::array<float, 4>, kTreeClumpCount> clumps{};
    for (std::uint32_t k = 0; k < kTreeClumpCount; ++k) {
        clumps.at(k) = treeClump(tree, k);
    }
    constexpr double kCell = 0.1;  // m, Trees.hpp kTreeCutCell
    for (const RealisticTreeCut& c : cuts) {
        if (c.tree != index) {
            continue;
        }
        const std::array<double, 3> p{(c.cell[0] + 0.5) * kCell, (c.cell[1] + 0.5) * kCell,
                                      (c.cell[2] + 0.5) * kCell};
        for (std::uint32_t k = 0; k < kTreeClumpCount; ++k) {
            const std::array<float, 4>& m = clumps.at(k);
            const double x = (p[0] - m[0]) / m[3];
            const double y = (p[1] - m[1]) / m[3];
            const double z = (p[2] - m[2]) / (0.8 * m[3]);
            if ((x * x) + (y * y) + (z * z) <= 1.0) {
                cut.at(k) += static_cast<float>(kCell * kCell * kCell);
            }
        }
    }
    const double centre = t.base + t.half;
    for (std::uint32_t k = 0; k < kTreeClumpCount; ++k) {
        const std::array<float, 4>& m = clumps.at(k);
        const double volume = 4.0 / 3.0 * std::numbers::pi * m[3] * m[3] * (0.8 * m[3]);
        const double left = 1.0 - (cut.at(k) / volume);
        constexpr double kGone = 0.3;  // under this share left, the clump is not drawn
        if (left < kGone) {
            continue;
        }
        const auto shrink = static_cast<float>(std::cbrt(left));
        RasterInstance& clump = out.emplace_back();
        clump.mesh = firstMesh;
        clump.position = place(tree, t, {m[0], m[1], centre + m[2]});
        clump.rotation = {tree.rotation[0], tree.rotation[1], tree.rotation[2], tree.rotation[3]};
        clump.scale = {m[3] * shrink, m[3] * shrink, 0.8F * m[3] * shrink};
        // Each clump its own tone, darker low in the crown (as the realistic view's leaves).
        std::uint32_t s = vegHash((tree.seed * 747796405U) + (k * 2891336453U) + 5U);
        const float tone = 0.8F + (0.4F * vegRandom(s));
        const float below = 0.75F + (0.25F * std::clamp((m[2] / t.half) + 0.5F, 0.0F, 1.0F));
        clump.color = {kLeafGreen[0] * tone * below, kLeafGreen[1] * tone * below,
                       kLeafGreen[2] * tone * below, 1.0F};
        clump.id = id;
        clump.roughness = 0.8F;
    }
}

}  // namespace lagrange::gpu
