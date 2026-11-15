// OpenDRIVE (roadmap §7, "alignment tests"): the reference-line geometries against closed forms
// and an independent clothoid integral, lane borders, and the sample city block's lanes, which
// must meet their successors end to start.
#include "lagrange/opendrive/OpenDrive.hpp"

#include <cmath>
#include <map>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::opendrive {
namespace {

constexpr double kPi = std::numbers::pi;

Geometry geometry(Geometry::Kind kind, double length) {
    Geometry g;
    g.kind = kind;
    g.length = length;
    return g;
}

TEST(Geometry, AQuarterArcEndsAtItsClosedForm) {
    Geometry g = geometry(Geometry::Kind::Arc, kPi / 2.0 * 10.0);
    g.curvature = 0.1;  // R = 10 m, turning left
    const Frame f = evaluate(g, g.length);
    EXPECT_NEAR(f.x, 10.0, 1e-12);
    EXPECT_NEAR(f.y, 10.0, 1e-12);
    EXPECT_NEAR(f.hdg, kPi / 2.0, 1e-12);
}

TEST(Geometry, ASpiralIsTheClothoidIntegral) {
    // Curvature from 0.3 to 0.8 1/m over 10 m: heading 0.3·s + 0.025·s²; the end point by an
    // independent composite Simpson integration (200 000 panels).
    Geometry g = geometry(Geometry::Kind::Spiral, 10.0);
    g.curvStart = 0.3;
    g.curvEnd = 0.8;
    const Frame f = evaluate(g, 10.0);
    EXPECT_NEAR(f.x, -0.133400675293952, 1e-9);
    EXPECT_NEAR(f.y, 1.828598728484163, 1e-9);
    EXPECT_NEAR(f.hdg, 0.3 * 10.0 + 0.025 * 100.0, 1e-12);
    // A spiral of constant curvature is an arc.
    Geometry s = geometry(Geometry::Kind::Spiral, 7.0);
    s.curvStart = s.curvEnd = 0.2;
    Geometry a = geometry(Geometry::Kind::Arc, 7.0);
    a.curvature = 0.2;
    for (const double d : {1.3, 4.0, 7.0}) {
        EXPECT_NEAR(evaluate(s, d).x, evaluate(a, d).x, 1e-12);
        EXPECT_NEAR(evaluate(s, d).y, evaluate(a, d).y, 1e-12);
    }
}

TEST(Geometry, PolynomialsByArcLength) {
    // A paramPoly3 that is a straight line at 45°, by arc length: s maps to the point exactly.
    const std::string xml =
        R"(<OpenDRIVE><road id="7" length="10" junction="-1"><planView>
        <geometry s="0" x="1" y="2" hdg="0" length="10">
        <paramPoly3 aU="0" bU="0.7071067811865476" cU="0" dU="0" aV="0" bV="0.7071067811865476"
            cV="0" dV="0" pRange="arcLength"/></geometry>
        </planView><lanes><laneSection s="0"><center><lane id="0" type="none"/></center>
        </laneSection></lanes></road></OpenDRIVE>)";
    const auto n = parse(xml);
    ASSERT_TRUE(n) << n.error();
    const Frame f = referenceAt(n->roads[0], 4.0);
    EXPECT_NEAR(f.x, 1.0 + 4.0 / std::numbers::sqrt2, 1e-9);
    EXPECT_NEAR(f.y, 2.0 + 4.0 / std::numbers::sqrt2, 1e-9);
    EXPECT_NEAR(f.hdg, kPi / 4.0, 1e-12);
    // A poly3 v = 0.1·u²: at arc length s, u solves ∫√(1 + (0.2u)²) du = s.
    Geometry p = geometry(Geometry::Kind::Poly3, 5.0);
    p.v = {0.0, 0.0, 0.1, 0.0};
    const std::string poly = R"(<OpenDRIVE><road id="1" length="5" junction="-1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="5"><poly3 a="0" b="0" c="0.1" d="0"/>
        </geometry></planView><lanes><laneSection s="0"><center><lane id="0"/></center>
        </laneSection></lanes></road></OpenDRIVE>)";
    const auto q = parse(poly);
    ASSERT_TRUE(q) << q.error();
    const Frame end = referenceAt(q->roads[0], 5.0);
    // u at s = 5: (u√(1+0.04u²) + asinh(0.2u)/0.2)/2 = 5 → u = 4.7046...; check the arc length.
    const double u = end.x;
    const double arc = 0.5 * (u * std::sqrt(1.0 + 0.04 * u * u) + std::asinh(0.2 * u) / 0.2);
    EXPECT_NEAR(arc, 5.0, 1e-6);
    EXPECT_NEAR(end.y, 0.1 * u * u, 1e-9);
    EXPECT_NEAR(end.hdg, std::atan(0.2 * u), 1e-9);
}

TEST(Lanes, BordersStackOutwardsFromTheOffset) {
    const std::string xml = R"(<OpenDRIVE><road id="1" length="100" junction="-1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="100"><line/></geometry></planView>
        <lanes><laneOffset s="0" a="0.5" b="0" c="0" d="0"/><laneSection s="0">
        <left><lane id="1" type="driving"><width sOffset="0" a="3" b="0.01" c="0" d="0"/></lane>
        </left><center><lane id="0" type="none"/></center>
        <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
        <lane id="-2" type="sidewalk"><width sOffset="0" a="2" b="0" c="0" d="0"/></lane></right>
        </laneSection></lanes></road></OpenDRIVE>)";
    const auto n = parse(xml);
    ASSERT_TRUE(n) << n.error();
    const Road& r = n->roads[0];
    const auto [i1, o1] = laneBorders(r, r.sections[0], 1, 10.0);
    EXPECT_DOUBLE_EQ(i1, 0.5);
    EXPECT_DOUBLE_EQ(o1, 0.5 + 3.1);
    const auto [i2, o2] = laneBorders(r, r.sections[0], -2, 10.0);
    EXPECT_DOUBLE_EQ(i2, 0.5 - 3.5);
    EXPECT_DOUBLE_EQ(o2, 0.5 - 5.5);
    const auto p = pointAt(r, 10.0, o2);
    EXPECT_DOUBLE_EQ(p[1], -5.0);
}

/// A lane's centre-line points.
std::vector<std::array<double, 3>> points(const SchemaInstance& s) {
    const std::span<const double> c = s.realArray("lane:centreLine");
    std::vector<std::array<double, 3>> out;
    for (std::size_t i = 0; i + 2 < c.size(); i += 3) {
        out.push_back({c[i], c[i + 1], c[i + 2]});
    }
    return out;
}

TEST(CityBlock, LanesMeetTheirSuccessorsEndToStart) {
    const auto n = read(LAGRANGE_TEST_CITY_XODR);
    ASSERT_TRUE(n) << n.error();
    EXPECT_EQ(n->roads.size(), 16U);
    const auto m = toModel(*n);
    ASSERT_TRUE(m) << m.error();
    const World& w = m->world;
    std::map<std::uint32_t, const SchemaInstance*> byPrim;
    int driving = 0;
    for (const SchemaInstance& s : w.schemaInstances) {
        ASSERT_EQ(s.schema, "RoadLaneAPI_1");
        byPrim[s.prim.index] = &s;
        driving += s.token("lane:type") == "driving" ? 1 : 0;
    }
    // Four arms with two driving lanes each way, and 4 × (2 + 1 + 1) connecting lanes.
    EXPECT_EQ(driving, 4 * 4 + 16);
    int links = 0;
    for (const SchemaInstance& s : w.schemaInstances) {
        const auto from = points(s);
        ASSERT_GE(from.size(), 2U);
        for (const PrimId next : s.rel("lane:successors")) {
            const auto to = points(*byPrim.at(next.index));
            const double gap =
                std::hypot(from.back()[0] - to.front()[0], from.back()[1] - to.front()[1]);
            EXPECT_LT(gap, 1e-6) << w.prims.paths[s.prim.index] << " → "
                                 << w.prims.paths[next.index];
            // Travel goes on in about the same direction.
            const double h0 = std::atan2(from.back()[1] - from[from.size() - 2][1],
                                         from.back()[0] - from[from.size() - 2][0]);
            const double h1 = std::atan2(to[1][1] - to[0][1], to[1][0] - to[0][0]);
            EXPECT_LT(std::abs(std::remainder(h1 - h0, 2.0 * kPi)), 0.1);
            ++links;
        }
    }
    // Into the junction: 2 incoming lanes × 2 movements per arm; out: one per connecting lane.
    EXPECT_EQ(links, 4 * 4 + 16);
    // An incoming lane of the east arm (lane 1, inner) travels west, on the north side.
    for (const SchemaInstance& s : w.schemaInstances) {
        if (s.integer("lane:road") == 1 && s.integer("lane:lane") == 1) {
            const auto p = points(s);
            EXPECT_GT(p.front()[0], p.back()[0]);
            EXPECT_NEAR(p.front()[1], 1.75, 1e-9);
            EXPECT_EQ(s.rel("lane:successors").size(), 2U);  // straight on, left
            EXPECT_NEAR(s.real("lane:speedLimit").value_or(0.0), 50.0 / 3.6, 1e-9);
        }
    }
    EXPECT_FALSE(m->scene.meshes.empty());
}

// Superelevation and lane heights (ASAM OpenDRIVE 1.8 §10.5.2, §11.6.3) against closed forms, on
// the roads of test/data/banked.xodr.
class Banked : public ::testing::Test {
protected:
    void SetUp() override {
        auto n = read(LAGRANGE_TEST_BANKED_XODR);
        ASSERT_TRUE(n) << n.error();
        network_ = std::move(*n);
    }
    const Road& road(int id) const { return *network_.road(id); }
    const SchemaInstance* lane(const Model& m, int road, int id) const {
        for (const SchemaInstance& s : m.world.schemaInstances) {
            if (s.integer("lane:road") == road && s.integer("lane:lane") == id) {
                return &s;
            }
        }
        return nullptr;
    }
    static const TriMesh* mesh(const Model& m, const std::string& name) {
        for (const ScenePrim& p : m.scene.prims) {
            if (p.name == name && p.mesh != kNoIndex) {
                return &m.scene.meshes.at(p.mesh);
            }
        }
        return nullptr;
    }
    Network network_;
};

constexpr double kBank = 0.0872664626;  // rad, 5°

TEST_F(Banked, AConstantBankRollsTheCrossSectionAboutTheReferenceLine) {
    const Road& r = road(1);
    EXPECT_NEAR(superelevationAt(r, 13.0), kBank, 1e-15);
    // t is measured along the banked t axis: t·cos φ to the side, t·sin φ up; positive
    // superelevation falls to the right (t < 0).
    for (const double t : {3.5, -3.5, -1.2}) {
        const auto p = pointAt(r, 10.0, t);
        EXPECT_NEAR(p[0], 10.0, 1e-12);
        EXPECT_NEAR(p[1], t * std::cos(kBank), 1e-12);
        EXPECT_NEAR(p[2], 2.0 + t * std::sin(kBank), 1e-12);
    }
    EXPECT_LT(pointAt(r, 10.0, -3.5)[2], pointAt(r, 10.0, 3.5)[2]);
    // The lanes' centre lines lie on the banked surface: lane -1 at t = -1.75, lane 1 at 1.75
    // (its points against s).
    const auto m = toModel(network_);
    ASSERT_TRUE(m) << m.error();
    const auto right = points(*lane(*m, 1, -1));
    const auto left = points(*lane(*m, 1, 1));
    for (const auto& p : right) {
        EXPECT_NEAR(p[1], -1.75 * std::cos(kBank), 1e-12);
        EXPECT_NEAR(p[2], 2.0 - 1.75 * std::sin(kBank), 1e-12);
    }
    EXPECT_NEAR(left.front()[0], 40.0, 1e-12);
    EXPECT_NEAR(left.front()[2], 2.0 + 1.75 * std::sin(kBank), 1e-12);
    // The asphalt's normals are the banked plane's: (0, -sin φ, cos φ).
    const TriMesh* asphalt = mesh(*m, "road_1_asphalt");
    ASSERT_NE(asphalt, nullptr);
    for (std::size_t i = 0; i < asphalt->normals.size(); i += 3) {
        EXPECT_NEAR(asphalt->normals[i], 0.0, 1e-6);
        EXPECT_NEAR(asphalt->normals[i + 1], -std::sin(kBank), 1e-6);
        EXPECT_NEAR(asphalt->normals[i + 2], std::cos(kBank), 1e-6);
    }
    // Every triangle is wound counter-clockwise seen from its normal's side.
    for (std::size_t t = 0; t < asphalt->indices.size(); t += 3) {
        const auto v = [&](std::size_t k, std::size_t c) {
            return static_cast<double>(asphalt->positions[3 * asphalt->indices[t + k] + c]);
        };
        const double ux = v(1, 0) - v(0, 0);
        const double uy = v(1, 1) - v(0, 1);
        const double uz = v(1, 2) - v(0, 2);
        const double wx = v(2, 0) - v(0, 0);
        const double wy = v(2, 1) - v(0, 1);
        const double wz = v(2, 2) - v(0, 2);
        const double ny = uz * wx - ux * wz;
        const double nz = ux * wy - uy * wx;
        EXPECT_GT(ny * -std::sin(kBank) + nz * std::cos(kBank), 0.0);
    }
}

TEST_F(Banked, ALinearRampOfSuperelevation) {
    const Road& r = road(2);
    // 0.01 rad/m from 0 for 20 m, then 0.2 rad.
    for (const double s : {0.0, 5.0, 12.5, 19.9}) {
        EXPECT_NEAR(superelevationAt(r, s), 0.01 * s, 1e-12);
        const auto p = pointAt(r, s, -3.5);
        EXPECT_NEAR(p[1], 50.0 - 3.5 * std::cos(0.01 * s), 1e-12);
        EXPECT_NEAR(p[2], -3.5 * std::sin(0.01 * s), 1e-12);
    }
    EXPECT_NEAR(superelevationAt(r, 30.0), 0.2, 1e-15);
    EXPECT_NEAR(pointAt(r, 30.0, -3.5)[2], -3.5 * std::sin(0.2), 1e-12);
}

TEST_F(Banked, ALevelLaneStaysAtTheHeightOfItsInnerBorder) {
    // Road 2's shoulder (@level="true") outside lane -1 at s = 30 (φ = 0.2): level from the
    // border at t = -3.5, reaching 1 m further out horizontally.
    const Road& r = road(2);
    const LaneSection& sec = r.sections.front();
    const auto border = pointAt(r, 30.0, -3.5);
    const auto outer = lanePoint(r, sec, -2, 30.0, -4.5);
    EXPECT_NEAR(outer[2], border[2], 1e-12);
    EXPECT_NEAR(outer[1], border[1] - 1.0, 1e-12);
    EXPECT_NEAR(lanePoint(r, sec, -1, 30.0, -2.0)[2], -2.0 * std::sin(0.2), 1e-12);
}

TEST_F(Banked, AKerbLaneIsRaisedAlongTheBankedNormal) {
    // Road 1's sidewalk (lane -2, t from -3.5 to -5.5) is 0.15 m up along h, the normal of the
    // banked cross section: (−h sin φ) to the side, h cos φ up.
    const Road& r = road(1);
    const LaneSection& sec = r.sections.front();
    constexpr double h = 0.15;
    for (const double t : {-3.5, -4.5, -5.5}) {
        const auto p = lanePoint(r, sec, -2, 20.0, t);
        EXPECT_NEAR(p[1], t * std::cos(kBank) - h * std::sin(kBank), 1e-12);
        EXPECT_NEAR(p[2], 2.0 + t * std::sin(kBank) + h * std::cos(kBank), 1e-12);
    }
    const auto m = toModel(network_);
    ASSERT_TRUE(m) << m.error();
    // The sidewalk's centre line carries the kerb in its height.
    for (const auto& p : points(*lane(*m, 1, -2))) {
        EXPECT_NEAR(p[2], 2.0 - 4.5 * std::sin(kBank) + h * std::cos(kBank), 1e-12);
    }
    // The kerb's face is drawn: pavement normals that point across the road, towards lane -1.
    const TriMesh* pavement = mesh(*m, "road_1_pavement");
    ASSERT_NE(pavement, nullptr);
    int facing = 0;
    for (std::size_t i = 0; i < pavement->normals.size(); i += 3) {
        if (std::abs(pavement->normals[i + 2]) < 0.2F) {
            EXPECT_GT(pavement->normals[i + 1], 0.9F);  // +y: towards the road's centre
            ++facing;
        }
    }
    EXPECT_GT(facing, 0);
}

TEST_F(Banked, RaisedRoadsCollideFlatOnesDoNot) {
    EXPECT_TRUE(raised(road(1)));
    EXPECT_TRUE(raised(road(2)));
    EXPECT_FALSE(raised(road(3)));
    ConvertOptions options;
    const auto m = toModel(network_, options);
    ASSERT_TRUE(m) << m.error();
    const World::Colliders& c = m->world.colliders;
    // Road 1: three drawn lanes, road 2: two, each in 40 m / 2 m = 20 slabs; road 3 none.
    ASSERT_EQ(c.body.size(), (3U + 2U) * 20U);
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        EXPECT_EQ(c.body[i], kWorldBody);
        EXPECT_EQ(c.shape[i], ShapeType::ConvexHull);
        EXPECT_LT(c.poseInBody[i].position[1], 60.0);  // none on road 3 (y = 100)
    }
    // The first slab (road 1, lane 1, s from 0 to 2): its top is the drawn surface.
    const ConvexHull& hull = m->world.hulls.at(c.hull[0]);
    const auto& at = c.poseInBody[0].position;
    double top = -INFINITY;
    double bottom = INFINITY;
    for (const auto& p : hull.points) {
        top = std::max(top, p[2] + at[2]);
        bottom = std::min(bottom, p[2] + at[2]);
    }
    EXPECT_NEAR(top, 2.0 + 3.5 * std::sin(kBank) + options.surfaceHeight, 1e-5);
    EXPECT_NEAR(bottom, 2.0 + options.surfaceHeight - options.colliderDepth, 1e-5);
}

TEST(Parse, NamesWhatIsWrong) {
    EXPECT_FALSE(parse("<notodr/>"));
    const auto bad = parse(R"(<OpenDRIVE><road id="3" length="1"><planView>
        <geometry s="0" x="0" y="0" hdg="0" length="1"><clothoid/></geometry></planView>
        <lanes><laneSection s="0"/></lanes></road></OpenDRIVE>)");
    ASSERT_FALSE(bad);
    EXPECT_NE(bad.error().find("road 3"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::opendrive
