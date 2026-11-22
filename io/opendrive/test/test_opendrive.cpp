// OpenDRIVE (roadmap §7, "alignment tests"): the reference-line geometries against closed forms
// and an independent clothoid integral, lane borders, and the sample city block's lanes, which
// must meet their successors end to start.
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/RoadLanes.hpp"

#include <cmath>
#include <map>
#include <numbers>
#include <stdexcept>
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

// Objects and signals (ASAM OpenDRIVE 1.8 §13, §14) on test/data/objects.xodr.
class Objects : public ::testing::Test {
protected:
    void SetUp() override {
        auto n = read(LAGRANGE_TEST_OBJECTS_XODR);
        ASSERT_TRUE(n) << n.error();
        network_ = std::move(*n);
    }
    const Road& road(int id) const { return *network_.road(id); }
    const Object& object(int r, const std::string& id) const {
        for (const Object& o : road(r).objects) {
            if (o.id == id) {
                return o;
            }
        }
        throw std::runtime_error("no object " + id);
    }
    /// The unit x and z axes of a pose.
    static std::array<double, 3> axis(const Pose& p, int k) {
        std::array<double, 3> v{};
        v[static_cast<std::size_t>(k)] = 1.0;
        return rotate(p.orientation, v);
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

TEST_F(Objects, AreReadWithTheirRepeatsOutlinesAndSignals) {
    EXPECT_EQ(road(1).objects.size(), 7U);
    EXPECT_EQ(road(1).signals.size(), 2U);
    EXPECT_EQ(road(2).objects.size(), 1U);
    const Object& rail = object(1, "rail");
    ASSERT_EQ(rail.repeats.size(), 1U);
    EXPECT_EQ(rail.repeats[0].distance, 0.0);
    EXPECT_EQ(object(1, "house").outlines.size(), 1U);  // under <outlines>
    EXPECT_TRUE(object(1, "house").outlines[0].local);
    EXPECT_EQ(object(1, "ell").outlines.size(), 1U);  // 1.4's single <outline>
    EXPECT_EQ(object(1, "ell").outlines[0].corners.size(), 6U);
    EXPECT_TRUE(road(1).signals[1].dynamic);
    EXPECT_EQ(road(1).signals[0].orientation, "+");
}

TEST_F(Objects, APoleStandsAtItsSAndTTurnedByItsHeading) {
    const auto p = placements(road(1), object(1, "pole1"));
    ASSERT_EQ(p.size(), 1U);
    EXPECT_NEAR(p[0].pose.position[0], 20.0, 1e-12);  // x = 10 + s
    EXPECT_NEAR(p[0].pose.position[1], 1.0, 1e-12);   // y = 5 + t
    EXPECT_NEAR(p[0].pose.position[2], 0.0, 1e-12);
    const auto x = axis(p[0].pose, 0);
    EXPECT_NEAR(std::atan2(x[1], x[0]), 0.3, 1e-12);
    EXPECT_EQ(p[0].height, 3.0);
}

TEST_F(Objects, OnABankedRoadTheOriginIsOnTheSurface) {
    // Road 2 runs along +y, 2 m up, banked 0.1 rad (falling to the right, +x): t = -3 lies
    // 3·cos 0.1 to the right and 3·sin 0.1 down, zOffset 0.5 above; heading from the road's.
    constexpr double kTilt = 0.1;
    const Object& crate = object(2, "crate");
    const auto p = placements(road(2), crate);
    ASSERT_EQ(p.size(), 1U);
    const Pose& pose = p[0].pose;
    EXPECT_NEAR(pose.position[0], 3.0 * std::cos(kTilt), 1e-9);
    EXPECT_NEAR(pose.position[1], 5.0, 1e-9);
    EXPECT_NEAR(pose.position[2], 2.0 - 3.0 * std::sin(kTilt) + 0.5, 1e-9);
    // z-y'-x'': yaw ψ = π/2 + 0.25, pitch θ = 0.05, roll φ = −0.1 (closed forms of R's columns).
    const double psi = kPi / 2.0 + 0.25;
    const double th = 0.05;
    const double ph = -0.1;
    const auto x = axis(pose, 0);
    EXPECT_NEAR(x[0], std::cos(psi) * std::cos(th), 1e-12);
    EXPECT_NEAR(x[1], std::sin(psi) * std::cos(th), 1e-12);
    EXPECT_NEAR(x[2], -std::sin(th), 1e-12);
    const auto z = axis(pose, 2);
    EXPECT_NEAR(z[0], std::cos(psi) * std::sin(th) * std::cos(ph) + std::sin(psi) * std::sin(ph),
                1e-12);
    EXPECT_NEAR(z[1], std::sin(psi) * std::sin(th) * std::cos(ph) - std::cos(psi) * std::sin(ph),
                1e-12);
    EXPECT_NEAR(z[2], std::cos(th) * std::cos(ph), 1e-12);
    // Its collider: a box of half its size, centred half its height up its own z.
    const auto m = toModel(network_);
    ASSERT_TRUE(m) << m.error();
    const World::Colliders& c = m->world.colliders;
    const std::size_t last = c.body.size() - 1;
    EXPECT_EQ(c.shape[last], ShapeType::Box);
    EXPECT_NEAR(c.size[last][0], 0.5, 1e-12);
    EXPECT_NEAR(c.size[last][1], 0.4, 1e-12);
    EXPECT_NEAR(c.size[last][2], 0.3, 1e-12);
    for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_NEAR(c.poseInBody[last].position[k], pose.position[k] + 0.3 * z[k], 1e-9);
    }
}

TEST_F(Objects, ARepeatPlacesACopyEveryDistanceWithItsValuesLinear) {
    const auto p = placements(road(1), object(1, "bollards"));
    ASSERT_EQ(p.size(), 5U);  // s = 20, 22.5, …, 30
    for (std::size_t i = 0; i < p.size(); ++i) {
        const double f = static_cast<double>(i) / 4.0;
        EXPECT_NEAR(p[i].pose.position[0], 30.0 + 2.5 * static_cast<double>(i), 1e-12);
        EXPECT_NEAR(p[i].pose.position[1], 5.0 - 5.0 - f, 1e-12);  // t from -5 to -6
        EXPECT_NEAR(p[i].height, 1.0 + 0.4 * f, 1e-12);
        EXPECT_EQ(p[i].radius, 0.1);
    }
    // A continuous repeat (distance 0) places no copies: it is one object along its path.
    EXPECT_TRUE(placements(road(1), object(1, "rail")).empty());
}

TEST_F(Objects, SignalsFaceTheTrafficTheyApplyTo) {
    // "+": for traffic along s, so its face looks back against s.
    const Pose stop = signalPose(road(1), road(1).signals[0]);
    EXPECT_NEAR(stop.position[0], 40.0, 1e-12);
    EXPECT_NEAR(stop.position[1], 0.0, 1e-12);
    EXPECT_NEAR(stop.position[2], 2.0, 1e-12);
    const auto face = axis(stop, 0);
    EXPECT_NEAR(face[0], -1.0, 1e-12);
    EXPECT_NEAR(face[1], 0.0, 1e-12);
    // "-": for traffic against s, facing along s, turned counter-clockwise by hOffset.
    const Pose light = signalPose(road(1), road(1).signals[1]);
    EXPECT_NEAR(light.position[0], 58.0, 1e-12);
    EXPECT_NEAR(light.position[1], 10.0, 1e-12);
    EXPECT_NEAR(light.position[2], 3.0, 1e-12);
    const auto lf = axis(light, 0);
    EXPECT_NEAR(std::atan2(lf[1], lf[0]), 0.2, 1e-12);
}

TEST_F(Objects, StandingObjectsCollideAndMarksDoNot) {
    ConvertOptions options;
    const auto m = toModel(network_, options);
    ASSERT_TRUE(m) << m.error();
    const World::Colliders& c = m->world.colliders;
    // Road 1 (flat, no slabs): pole 1 box, 5 bollard cylinders, the rail's 20 m in 2 m hulls
    // (10), the square house 1 hull, the L 4 (a hull per triangle of its footprint), the sign
    // and the light a box and a pole each; crosswalk and parking space none. Road 2: its lane's
    // 10 slabs, then the crate's box.
    ASSERT_EQ(c.body.size(), 1U + 5U + 10U + 1U + 4U + 4U + 10U + 1U);
    std::map<ShapeType, int> shapes;
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        EXPECT_EQ(c.body[i], kWorldBody);
        ++shapes[c.shape[i]];
    }
    EXPECT_EQ(shapes[ShapeType::Box], 1 + 2 + 1);
    EXPECT_EQ(shapes[ShapeType::Cylinder], 5 + 2);
    EXPECT_EQ(shapes[ShapeType::ConvexHull], 10 + 1 + 4 + 10);
    // The house's hull: its footprint at (50 ± 3, 17 ± 3), 5 m tall.
    const std::size_t house = 1 + 5 + 10;
    ASSERT_EQ(c.shape[house], ShapeType::ConvexHull);
    const ConvexHull& hull = m->world.hulls.at(c.hull[house]);
    const auto& at = c.poseInBody[house].position;
    std::array<double, 3> lo{INFINITY, INFINITY, INFINITY};
    std::array<double, 3> hi{-INFINITY, -INFINITY, -INFINITY};
    for (const auto& p : hull.points) {
        for (std::size_t k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k] + at[k]);
            hi[k] = std::max(hi[k], p[k] + at[k]);
        }
    }
    EXPECT_NEAR(lo[0], 47.0, 1e-5);
    EXPECT_NEAR(hi[0], 53.0, 1e-5);
    EXPECT_NEAR(lo[1], 14.0, 1e-5);
    EXPECT_NEAR(hi[1], 20.0, 1e-5);
    EXPECT_NEAR(lo[2], 0.0, 1e-5);
    EXPECT_NEAR(hi[2], 5.0, 1e-5);
    // The L's four hulls cover its 48 m² footprint, not its 64 m² bounding box.
    double area = 0.0;
    for (std::size_t i = house + 1; i < house + 5; ++i) {
        const ConvexHull& h = m->world.hulls.at(c.hull[i]);
        double xs[3];
        double ys[3];
        std::size_t n = 0;
        for (const auto& p : h.points) {
            if (p[2] + c.poseInBody[i].position[2] < 1.0 && n < 3) {
                xs[n] = p[0];
                ys[n] = p[1];
                ++n;
            }
        }
        ASSERT_EQ(n, 3U);
        area +=
            0.5 * std::abs((xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0]));
    }
    EXPECT_NEAR(area, 48.0, 1e-4);
    // Drawn: meshes per material under the road; the crosswalk's stripes and the parking
    // space's border are paint just above the road surface.
    const TriMesh* paint = mesh(*m, "road_1_objects_mark_white");
    ASSERT_NE(paint, nullptr);
    for (std::size_t i = 2; i < paint->positions.size(); i += 3) {
        EXPECT_NEAR(paint->positions[i], options.surfaceHeight + 0.003, 1e-6);
    }
    EXPECT_NE(mesh(*m, "road_1_objects_object_building_concrete"), nullptr);
    EXPECT_NE(mesh(*m, "road_1_objects_signal_head_housing"), nullptr);
    // A traffic light's lamps are the traffic module's: none drawn here.
    EXPECT_EQ(mesh(*m, "road_1_objects_signal_lamp_red"), nullptr);
    EXPECT_NE(mesh(*m, "road_1_objects_sign_board"), nullptr);
    EXPECT_NE(mesh(*m, "road_2_objects_object_concrete"), nullptr);
}

// Traffic lights (§14.3 validity, §14.6 controllers) on test/data/signals.xodr: a four-way
// junction with a light on each arm, east-west on controller 1, north-south on controller 2,
// which the junction lists first.
TEST(Signals, AreReadWithTheirValidityAndControllers) {
    const auto n = read(LAGRANGE_TEST_SIGNALS_XODR);
    ASSERT_TRUE(n) << n.error();
    const Signal& east = n->road(1)->signals.at(0);
    EXPECT_EQ(east.id, "11");
    EXPECT_TRUE(east.dynamic);
    ASSERT_EQ(east.validity.size(), 1U);
    EXPECT_EQ(east.validity[0], (std::pair{1, 1}));
    EXPECT_TRUE(n->road(4)->signals.at(0).validity.empty());
    ASSERT_EQ(n->controllers.size(), 2U);
    EXPECT_EQ(n->controllers[0].id, "1");
    EXPECT_EQ(n->controllers[0].name, "east_west");
    EXPECT_EQ(n->controllers[0].signals, (std::vector<std::string>{"11", "13"}));
    EXPECT_FALSE(n->controllers[0].sequence);
    ASSERT_EQ(n->junctions.size(), 1U);
    const auto& listed = n->junctions[0].controllers;
    ASSERT_EQ(listed.size(), 2U);
    EXPECT_EQ(listed[0].id, "2");
    EXPECT_EQ(listed[0].sequence, 1);
    EXPECT_EQ(listed[1].id, "1");
    EXPECT_EQ(listed[1].sequence, 2);
}

TEST(Signals, BecomeRoadSignalPrimsAtTheirFaces) {
    const auto n = read(LAGRANGE_TEST_SIGNALS_XODR);
    ASSERT_TRUE(n) << n.error();
    const auto m = toModel(*n);
    ASSERT_TRUE(m) << m.error();
    const WorldView world(m->world);
    const auto instances = world.instancesOf("RoadSignalAPI_1");
    ASSERT_EQ(instances.size(), 4U);
    std::map<std::string, const SchemaInstance*> byId;
    for (const SchemaInstance* s : instances) {
        byId[std::string(s->token("signal:id").value_or(""))] = s;
    }
    const SchemaInstance& east = *byId.at("11");
    EXPECT_EQ(world.primPath(east.prim), "/roads/signals/signal_11");
    EXPECT_EQ(east.integer("signal:road"), 1);
    EXPECT_EQ(east.real("signal:s"), 3.0);
    EXPECT_EQ(east.token("signal:orientation"), "-");
    EXPECT_EQ(east.token("signal:type"), "1000001");
    EXPECT_EQ(east.integer("signal:fromLane"), 1);
    EXPECT_EQ(east.integer("signal:toLane"), 1);
    EXPECT_EQ(east.token("signal:controller"), "1");
    EXPECT_EQ(east.integer("signal:junction"), 100);
    EXPECT_EQ(east.integer("signal:sequence"), 2);
    EXPECT_EQ(east.real("signal:height"), 0.9);
    const SchemaInstance& south = *byId.at("14");
    EXPECT_FALSE(south.integer("signal:fromLane"));  // no validity: the side it faces
    EXPECT_EQ(south.token("signal:controller"), "2");
    EXPECT_EQ(south.integer("signal:sequence"), 1);
    // The face's middle: the board's bottom centre (13, 4.5, 2.5) on the east arm, facing along
    // s (+x, towards the traffic coming in), half its depth (0.3 m) out and half its height up.
    const Pose& face = world.primPoseInBody(east.prim);
    EXPECT_NEAR(face.position[0], 13.15, 1e-9);
    EXPECT_NEAR(face.position[1], 4.5, 1e-9);
    EXPECT_NEAR(face.position[2], 2.95, 1e-9);
    const auto out = rotate(face.orientation, {1.0, 0.0, 0.0});
    EXPECT_NEAR(out[0], 1.0, 1e-12);
    // A scene prim of the same path carries the pose into a written stage.
    bool found = false;
    for (const ScenePrim& p : m->scene.prims) {
        if (p.path == "/roads/signals/signal_11") {
            found = true;
            EXPECT_NEAR(p.offset.position[0], 13.15, 1e-9);
        }
    }
    EXPECT_TRUE(found);
    // Resolved to the lanes they govern: each arm's lane 1, which comes in against s, stopping
    // 3 m before its end (s = 3 of 50 is 47 m along it).
    const RoadLanes lanes = roadLanesOf(world);
    const std::vector<RoadSignal> signals = roadSignalsOf(world, lanes);
    ASSERT_EQ(signals.size(), 4U);
    for (const RoadSignal& g : signals) {
        ASSERT_EQ(g.lanes.size(), 1U) << g.id;
        const RoadLane& lane = lanes.lanes[g.lanes[0].first];
        EXPECT_EQ(lane.road, g.road) << g.id;
        EXPECT_EQ(lane.lane, 1) << g.id;
        EXPECT_NEAR(g.lanes[0].second, lane.length() - 3.0, 1e-6) << g.id;
    }
}

TEST(ObjectsNone, ARoadWithoutObjectsGainsNothing) {
    const auto n = read(LAGRANGE_TEST_BANKED_XODR);
    ASSERT_TRUE(n) << n.error();
    const auto m = toModel(*n);
    ASSERT_TRUE(m) << m.error();
    for (const ScenePrim& p : m->scene.prims) {
        EXPECT_EQ(p.name.find("_objects_"), std::string::npos) << p.name;
    }
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
