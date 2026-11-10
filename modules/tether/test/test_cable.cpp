// The cable model of RFC 0011 against what is known of cables: the catenary, a taut string's
// fundamental, the capstan equation, contact that pushes back equally, no interpenetration when it
// piles up, the winch holding its tension, and the same numbers from the same run.
#include "lagrange/tether/Cable.hpp"

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::tether {
namespace {

constexpr double kG = 9.80665;

double sagOf(std::span<const Vec3> nodes) {
    double lowest = 0.0;
    for (const Vec3& p : nodes) {
        lowest = std::min(lowest, p[2]);
    }
    return -lowest;
}

/// Solves a·sinh(span / 2a) = length / 2 for the catenary's parameter a, by bisection.
double catenaryParameter(double span, double length) {
    double lo = 1e-3;
    double hi = 1e6;
    for (int i = 0; i < 200; ++i) {
        const double mid = std::sqrt(lo * hi);
        (2.0 * mid * std::sinh(span / (2.0 * mid)) > length ? lo : hi) = mid;
    }
    return std::sqrt(lo * hi);
}

TEST(Cable, HangsAsACatenary) {
    CableParams p;
    p.length = 12.0;
    p.segmentLength = 0.1;
    p.linearDensity = 0.02;
    p.axialStiffness = 2e4;
    p.settle = 2.0;  // still it, to compare with the static shape
    Cable cable(p, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, 0);
    Air air;
    for (int i = 0; i < 10000; ++i) {
        cable.step(1e-3, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {}, air);
    }
    const double a = catenaryParameter(10.0, 12.0);
    const double sag = a * (std::cosh(5.0 / a) - 1.0);
    const double end = p.linearDensity * kG * a * std::cosh(5.0 / a);  // T = μ g a cosh(x/a)
    std::printf("sag %.4f m (catenary %.4f), end tension %.4f N (catenary %.4f)\n",
                sagOf(cable.nodes()), sag, cable.tensionAttach(), end);
    EXPECT_NEAR(sagOf(cable.nodes()), sag, 0.005 * sag);
    EXPECT_NEAR(cable.tensionAttach(), end, 0.02 * end);
    EXPECT_NEAR(cable.tensionAnchor(), end, 0.02 * end);
    // The pull on the attach point is along the cable there, towards the anchor and up.
    EXPECT_LT(cable.forceOnAttach()[0], 0.0);
    EXPECT_LT(cable.forceOnAttach()[2], 0.0);  // the cable hangs below: it pulls down
}

TEST(Cable, ATautStringRingsAtItsFundamental) {
    // No gravity: a 4 m cable stretched to 4.02 m rings at f₁ = sqrt(T/μ) / (2L).
    CableParams p;
    p.length = 4.0;
    p.segmentLength = 0.05;
    p.linearDensity = 0.02;
    p.axialStiffness = 2e3;
    p.dampingRatio = 0.0;
    const double span = 4.02;
    Cable cable(p, {0.0, 0.0, 0.0}, {span, 0.0, 0.0}, 0);
    Air air;
    air.gravity = {};
    for (int i = 0; i < 500; ++i) {  // let it take up its stretch
        cable.step(1e-3, {0.0, 0.0, 0.0}, {span, 0.0, 0.0}, {}, air);
    }
    // Pluck it into its first mode, 2 cm at the middle.
    std::vector<double> state;
    cable.save(state);
    const auto count = static_cast<std::size_t>(state[0]);
    for (std::size_t i = 0; i < count; ++i) {
        const double x = state[1 + i * 6];
        state[1 + i * 6 + 1] = 0.02 * std::sin(std::numbers::pi * x / span);
        state[1 + i * 6 + 3] = state[1 + i * 6 + 4] = state[1 + i * 6 + 5] = 0.0;
    }
    ASSERT_TRUE(cable.restore(state));
    const double tension = p.axialStiffness * (span - p.length) / p.length;
    const double expected = std::sqrt(tension / p.linearDensity) / (2.0 * span);
    // Time the middle's crossings of zero.
    const std::size_t middle = count / 2;
    double last = cable.nodes()[middle][1];
    std::vector<double> crossings;
    for (int i = 1; i <= 4000; ++i) {
        cable.step(1e-3, {0.0, 0.0, 0.0}, {span, 0.0, 0.0}, {}, air);
        const double y = cable.nodes()[middle][1];
        if ((last > 0.0) != (y > 0.0)) {
            crossings.push_back((i - 1 + last / (last - y)) * 1e-3);
        }
        last = y;
    }
    ASSERT_GE(crossings.size(), 4U);
    const double period =
        2.0 * (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
    std::printf("taut string: %.3f Hz, expected %.3f Hz at %.2f N\n", 1.0 / period, expected,
                tension);
    EXPECT_NEAR(1.0 / period, expected, 0.03 * expected);
}

TEST(Cable, SlidesOverACylinderAsTheCapstanEquationSays) {
    // No gravity. The cable wraps half a fixed cylinder (θ = π); its anchor end comes off a free
    // drum, its attach end is pulled down steadily, so it slides over the cylinder:
    // T_attach / T_anchor = e^{μθ}.
    CableParams p;
    p.length = 6.5;
    p.segmentLength = 0.02;
    p.diameter = 0.004;
    p.linearDensity = 0.02;
    p.axialStiffness = 5e3;
    p.friction = 0.3;
    p.winch = Winch::Free;
    p.winchSpeed = 2.0;
    p.maxLength = 20.0;
    // It starts straight just over the drum, then both ends come down beside it (2 s), and the
    // attach end goes on down at 0.2 m/s.
    const Vec3 anchorStart{-3.0, 0.0, 0.52};
    const Vec3 anchorEnd{-0.5, 0.0, -2.0};
    Vec3 attach{3.0, 0.0, 0.52};
    p.length = 6.0;
    Obstacle drum;
    drum.type = ShapeType::Cylinder;
    drum.size = {0.498, 1.0, 0.0};  // its surface under the cable's centre line
    drum.pose.orientation = {std::cos(std::numbers::pi / 4.0), std::sin(std::numbers::pi / 4.0),
                             0.0, 0.0};  // axis along y
    drum.reach = reachOf(drum.type, drum.size, nullptr);
    const std::array<Obstacle, 1> obstacles{drum};
    Cable cable(p, anchorStart, attach, 0);
    Air air;
    air.gravity = {};
    air.density = 0.0;
    double ratio = 0.0;
    int samples = 0;
    for (int i = 0; i < 14000; ++i) {
        Vec3 anchor = anchorEnd;
        if (i < 2000) {
            const double f = (i + 1) / 2000.0;
            for (std::size_t a = 0; a < 3; ++a) {
                anchor.at(a) = anchorStart.at(a) + (anchorEnd.at(a) - anchorStart.at(a)) * f;
            }
            attach = {3.0 - 2.5 * f, 0.0, 0.52 - 2.52 * f};
        } else {
            attach[2] -= 0.2e-3;  // 0.2 m/s
        }
        cable.step(1e-3, anchor, attach, obstacles, air);
        if (i > 10000) {
            ratio += cable.tensionAttach() / cable.tensionAnchor();
            ++samples;
        }
    }
    ratio /= samples;
    const double expected = std::exp(p.friction * std::numbers::pi);
    std::printf("capstan: T_attach/T_anchor %.3f, e^(μπ) %.3f (anchor %.3f N, paid out %.2f m)\n",
                ratio, expected, cable.tensionAnchor(), cable.length() - p.length);
    EXPECT_NEAR(ratio, expected, 0.05 * expected);
    EXPECT_GT(cable.length(), p.length + 0.3);  // the free drum paid out as it slid
}

TEST(Cable, PushesABodyAsMuchAsTheBodyHoldsItUp) {
    // A slack cable draped over a box that belongs to a body (index 0): in equilibrium the
    // box's push, the two ends' pulls and the cable's weight balance.
    CableParams p;
    p.length = 5.0;
    p.segmentLength = 0.05;
    p.linearDensity = 0.03;
    p.axialStiffness = 2e4;
    p.friction = 0.0;
    p.settle = 3.0;
    // It starts hanging well above the box, and both ends come down beside it (1 s).
    p.length = 3.6;
    const Vec3 anchor{-1.5, 0.0, 0.0};
    const Vec3 attach{1.5, 0.0, 0.0};
    const double lift = 1.5;
    Obstacle box;
    box.type = ShapeType::Box;
    box.size = {0.5, 0.5, 0.5};
    box.pose.position = {0.0, 0.0, 0.0};
    box.body = 0;
    box.reach = reachOf(box.type, box.size, nullptr);
    const std::array<Obstacle, 1> obstacles{box};
    Cable cable(p, {anchor[0], 0.0, lift}, {attach[0], 0.0, lift}, 1);
    Air air;
    for (int i = 0; i < 8000; ++i) {
        const double z = i < 1000 ? lift * (1.0 - (i + 1) / 1000.0) : 0.0;
        cable.step(1e-3, {anchor[0], 0.0, z}, {attach[0], 0.0, z}, obstacles, air);
    }
    const auto nodes = cable.nodes();
    const Vec3& n0 = nodes[0];
    const Vec3& n1 = nodes[1];
    const double l = std::hypot(n1[0] - n0[0], n1[1] - n0[1], n1[2] - n0[2]);
    // The anchor holds the cable up along its first segment, as the attach point does along the
    // last: forceOnAttach is the cable's pull on the attach point, so the cable feels minus it.
    const double anchorUp = cable.tensionAnchor() * (n0[2] - n1[2]) / l;
    const double attachUp = -cable.forceOnAttach()[2];
    const double boxUp = -cable.pushes()[0].force[2];  // pushes() is what the cable did to the box
    const double weight = p.linearDensity * cable.length() * kG;
    std::printf("weight %.4f N: box %.4f + anchor %.4f + attach %.4f = %.4f N, contacts %d\n",
                weight, boxUp, anchorUp, attachUp, boxUp + anchorUp + attachUp, cable.contacts());
    EXPECT_GT(cable.contacts(), 0);
    EXPECT_GT(boxUp, 0.3 * weight);
    EXPECT_NEAR(boxUp + anchorUp + attachUp, weight, 0.02 * weight);
}

TEST(Cable, PilesUpWithoutPassingThroughItself) {
    // 6 m dropped onto the ground between ends 0.5 m apart: it folds and piles, and no two
    // segments that are not neighbours come closer than about its diameter.
    CableParams p;
    p.length = 6.0;
    p.segmentLength = 0.05;
    p.diameter = 0.01;
    p.linearDensity = 0.02;
    p.axialStiffness = 2e4;
    p.friction = 0.4;
    p.bendStiffness = 1e-4;
    Obstacle ground;
    ground.type = ShapeType::Plane;
    ground.reach = reachOf(ground.type, {}, nullptr);
    const std::array<Obstacle, 1> obstacles{ground};
    const Vec3 anchor{0.0, 0.0, 0.3};
    const Vec3 attach{0.5, 0.0, 0.3};
    Cable cable(p, anchor, attach, 0);
    Air air;
    air.density = 1.2;
    for (int i = 0; i < 4000; ++i) {
        cable.step(1e-3, anchor, attach, obstacles, air);
    }
    const auto nodes = cable.nodes();
    double closest = 1e9;
    double lowest = 1e9;
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        lowest = std::min(lowest, nodes[i][2]);
        for (std::size_t j = i + 3; j + 1 < nodes.size(); ++j) {
            // Midpoint distances: a coarse check of the segment contacts.
            const Vec3 a{(nodes[i][0] + nodes[i + 1][0]) / 2, (nodes[i][1] + nodes[i + 1][1]) / 2,
                         (nodes[i][2] + nodes[i + 1][2]) / 2};
            const Vec3 b{(nodes[j][0] + nodes[j + 1][0]) / 2, (nodes[j][1] + nodes[j + 1][1]) / 2,
                         (nodes[j][2] + nodes[j + 1][2]) / 2};
            closest = std::min(closest, std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]));
        }
    }
    std::printf("pile: closest segments %.4f m (diameter %.3f), lowest %.4f m, contacts %d\n",
                closest, p.diameter, lowest, cable.contacts());
    EXPECT_GT(closest, 0.8 * p.diameter);
    EXPECT_GT(lowest, 0.8 * p.diameter / 2.0);  // on the ground, not in it
    EXPECT_GT(cable.contacts(), 20);
}

TEST(Cable, ATensionWinchHoldsItsTension) {
    // The far end climbs 3 m at 0.5 m/s from beside the drum; the winch pays out and keeps
    // about its set tension at the anchor.
    CableParams p;
    p.length = 1.0;
    p.maxLength = 30.0;
    p.segmentLength = 0.2;
    p.linearDensity = 0.03;
    p.diameter = 0.005;
    p.winch = Winch::Tension;
    p.winchTension = 2.0;
    p.winchSpeed = 1.5;
    Vec3 attach{0.5, 0.0, 0.8};
    const Vec3 anchor{0.0, 0.0, 0.0};
    Cable cable(p, anchor, attach, 0);
    Air air;
    // A nearly straight tether is stiff along itself: each payout sends a pulse of tension along
    // it, so the winch is judged on its mean.
    double climbing = 0.0;
    double holding = 0.0;
    for (int i = 0; i < 9000; ++i) {
        if (attach[2] < 3.8) {
            attach[2] += 0.5e-3;
        }
        cable.step(1e-3, anchor, attach, {}, air);
        if (i >= 2000 && i < 6000) {
            climbing += cable.tensionAnchor() / 4000.0;
        } else if (i >= 7000) {
            holding += cable.tensionAnchor() / 2000.0;
        }
    }
    std::printf(
        "winch: %.2f m paid out; anchor tension %.2f N climbing, %.2f N holding (set %.2f)\n",
        cable.length(), climbing, holding, p.winchTension);
    EXPECT_GT(cable.length(), 3.7);
    EXPECT_NEAR(climbing, p.winchTension, 0.5);
    EXPECT_NEAR(holding, p.winchTension, 0.5);
}

// Laid out through a box (its arch through a slab above the ends): it comes out at a few m/s,
// instead of being shot out with thousands of newtons.
TEST(Cable, LaidThroughABoxItComesOutSlowly) {
    CableParams p;
    p.length = 4.5;
    Obstacle slab;
    slab.type = ShapeType::Box;
    slab.size = {3.0, 3.0, 0.5};
    slab.pose.position = {1.25, 0.0, 1.0};  // 0.5 to 1.5 m up, over the middle of the arch
    slab.reach = reachOf(slab.type, slab.size, nullptr);
    const std::array<Obstacle, 1> obstacles{slab};
    const Vec3 anchor{2.5, 0.0, 0.05};
    const Vec3 attach{0.0, 0.0, 0.13};
    Cable cable(p, anchor, attach, 0);
    Air air;
    air.gravity = {};
    double highest = 0.0;
    for (const Vec3& n : cable.nodes()) {
        highest = std::max(highest, n[2]);
    }
    ASSERT_GT(highest, 1.0);  // laid through the slab
    double most = 0.0;
    for (int i = 0; i < 100; ++i) {
        cable.step(1e-3, anchor, attach, obstacles, air);
        most = std::max(most, cable.tensionAttach());
    }
    double now = 0.0;
    for (const Vec3& n : cable.nodes()) {
        now = std::max(now, n[2]);
    }
    std::printf("through a slab: highest %.3f m, after 0.1 s %.3f m, most tension %.3f N\n",
                highest, now, most);
    // Out at a few m/s (each mass and each segment's contact leaves at 1 m/s at most), not the
    // tens of m/s of an uncapped push.
    EXPECT_LT(highest - now, 0.3);
    // Lifted over the slab's top, the cable's path grows: some tension, not the thousands of
    // newtons of a mass thrown out in one substep.
    EXPECT_LT(most, 30.0);
}

// Laid from a winch beside a thin pad that rests on a slab, to a drone sitting on the pad: when
// the drone lifts the end, the winch pays out and the tension stays near its set point, instead
// of a mass trapped between the pad and the slab holding the cable.
TEST(Cable, LiftedOffAPadTheWinchPaysOut) {
    CableParams p;
    p.length = 1.2;
    p.maxLength = 30.0;
    p.diameter = 0.005;
    p.linearDensity = 0.03;
    p.winch = Winch::Tension;
    p.winchTension = 2.5;
    p.winchSpeed = 3.0;
    const auto box = [](const Vec3& centre, const Vec3& half) {
        Obstacle o;
        o.type = ShapeType::Box;
        o.size = half;
        o.pose.position = centre;
        o.reach = reachOf(o.type, o.size, nullptr);
        return o;
    };
    // A slab (top 1 m up), a 4 cm pad on it, and the winch's housing beside the pad.
    const std::array<Obstacle, 3> obstacles{box({0.0, 0.0, 0.8}, {4.0, 4.0, 0.2}),
                                            box({0.5, 0.0, 1.02}, {0.5, 0.5, 0.02}),
                                            box({-0.6, 0.0, 1.2}, {0.25, 0.2, 0.2})};
    const Vec3 anchor{-0.33, 0.0, 1.25};  // the housing's outlet, facing the pad
    Vec3 attach{0.5, 0.0, 1.08};          // on the pad
    Cable cable(p, anchor, attach, 0);
    Air air;
    air.density = 1.225;
    double most = 0.0;
    double mean = 0.0;
    for (int i = 1; i <= 6000; ++i) {
        if (i > 1000 && attach[2] < 2.7) {
            attach[2] += 0.5e-3;  // up 1.6 m at 0.5 m/s
        }
        if (i > 4000) {
            attach[1] += 1.0e-3;  // then 2 m sideways at 1 m/s
        }
        cable.step(1e-3, anchor, attach, obstacles, air);
        if (i > 1500) {
            most = std::max(most, cable.tensionAttach());
            mean += cable.tensionAttach() / 4500.0;
        }
    }
    std::printf("off the pad: paid out %.2f m, tension at the drone mean %.2f N, most %.2f N\n",
                cable.length(), mean, most);
    EXPECT_GT(cable.length(), 2.5);
    EXPECT_LT(mean, 6.0);
    EXPECT_LT(most, 30.0);
}

// Over a slab's edge, out, down and back in under it: the straight segments between the masses
// do not cut into the slab where the cable bends over the edge, which with contacts at the
// masses alone went centimetres in.
TEST(Cable, BendsOverAnEdgeWithoutCuttingIntoIt) {
    CableParams p;
    p.length = 1.2;
    p.maxLength = 30.0;
    p.diameter = 0.005;
    p.linearDensity = 0.03;
    p.winch = Winch::Tension;
    p.winchTension = 2.5;
    p.winchSpeed = 3.0;
    Obstacle slab;  // 8 m square, 0.4 m thick, its top 1.5 m up and its edge at y = 4
    slab.type = ShapeType::Box;
    slab.size = {4.0, 4.0, 0.2};
    slab.pose.position = {0.0, 0.0, 1.3};
    slab.reach = reachOf(slab.type, slab.size, nullptr);
    const std::array<Obstacle, 1> obstacles{slab};
    const Vec3 anchor{-2.0, 0.0, 1.7};
    Vec3 attach{-1.2, 0.0, 1.8};
    Cable cable(p, anchor, attach, 0);
    Air air;
    air.density = 1.225;
    double deepest = 1e9;  // the centre line's signed distance from the slab, along every segment
    for (int i = 1; i <= 12000; ++i) {
        if (i < 4500 && attach[1] < 6.5) {
            attach[1] += 1.5e-3;  // out past the edge
        } else if (i >= 4500 && i < 7000 && attach[2] > 0.6) {
            attach[2] -= 1.0e-3;  // down below the slab
        } else if (i >= 7000 && attach[1] > 2.5) {
            attach[1] -= 1.5e-3;  // back in under it
        }
        cable.step(1e-3, anchor, attach, obstacles, air);
        const auto n = cable.nodes();
        for (std::size_t j = 0; j + 1 < n.size(); ++j) {
            for (int k = 0; k <= 20; ++k) {
                const double t = k / 20.0;
                const Vec3 q{n[j][0] + t * (n[j + 1][0] - n[j][0]),
                             n[j][1] + t * (n[j + 1][1] - n[j][1]),
                             n[j][2] + t * (n[j + 1][2] - n[j][2])};
                deepest = std::min(deepest, surfacePoint(slab, q).distance);
            }
        }
    }
    std::printf("over the edge: the centre line at worst %+.4f m from the slab (radius %.4f)\n",
                deepest, 0.5 * p.diameter);
    EXPECT_GT(deepest, -p.diameter);  // within a diameter: not through the slab
    EXPECT_GT(cable.length(), 5.0);   // paid out round the edge
}

TEST(Cable, TheSameRunGivesTheSameNumbers) {
    const auto run = [] {
        CableParams p;
        p.length = 3.0;
        p.segmentLength = 0.05;
        p.friction = 0.5;
        Obstacle ground;
        ground.type = ShapeType::Plane;
        ground.reach = reachOf(ground.type, {}, nullptr);
        const std::array<Obstacle, 1> obstacles{ground};
        Cable cable(p, {0.0, 0.0, 0.2}, {1.0, 0.0, 1.0}, 0);
        Air air;
        air.density = 1.2;
        air.wind = {3.0, 1.0, 0.0};
        for (int i = 0; i < 1500; ++i) {
            const double t = i * 1e-3;
            cable.step(1e-3, {0.0, 0.0, 0.2}, {1.0 + 0.5 * std::sin(3.0 * t), 0.0, 1.0}, obstacles,
                       air);
        }
        std::vector<double> state;
        cable.save(state);
        return state;
    };
    EXPECT_EQ(run(), run());
}

}  // namespace
}  // namespace lagrange::tether
