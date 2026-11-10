#pragma once

// A cable between two points (RFC 0011): point masses about a segment apart, solved by extended
// position-based dynamics with small steps (Macklin, Storey, Lu, Terdiman, Chentanez, Jeschke
// and Müller 2019, "Small Steps in Physics Simulation"). Each step is split into substeps; each
// substep predicts the masses under gravity and air drag, makes one Gauss-Seidel pass over the
// length constraints (forward, then back), the bend constraints, the contacts with obstacles
// and with itself, and updates the velocities. A cable takes tension only: a slack segment pulls
// nothing. The tension of a segment is its constraint's multiplier over the substep squared.
// Deterministic: a fixed order, no threads, no randomness.

#include "lagrange/world/World.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace lagrange::tether {

using Vec3 = std::array<double, 3>;

enum class Winch : std::uint8_t {
    Fixed,    // the length stays
    Tension,  // pays out or reels in to keep a tension at the anchor
    Free,     // pays out whatever is pulled; reels nothing in
};

struct CableParams {
    double length{1.0};          // m paid out at the start
    double diameter{0.005};      // m
    double linearDensity{0.02};  // kg/m
    double axialStiffness{2e4};  // N, EA
    double dampingRatio{0.05};   // 1, axial, of one segment
    double bendStiffness{0.0};   // N·m², EI
    double friction{0.4};        // 1, Coulomb
    double dragCoeff{1.2};       // 1, normal; tangential is a fiftieth of it
    double segmentLength{0.2};   // m, the target spacing
    Winch winch{Winch::Fixed};
    double winchTension{5.0};  // N
    double winchSpeed{1.0};    // m/s
    double maxLength{0.0};     // m, the drum's end; 0: length
    int substeps{20};          // 10 rings a taut string 3.5 % flat; 20, 0.3 %
    double settle{0.0};        // 1/s: velocity damping for tests that want a still cable
};

/// A collider where it is now. `body` is the caller's index of a moving body, or -1 for scenery.
struct Obstacle {
    ShapeType type{ShapeType::Box};
    Vec3 size{};
    Pose pose;
    const ConvexHull* hull{nullptr};
    const HeightField* field{nullptr};
    int body{-1};
    /// The robot it belongs to (its moving bodies share one), -1 for scenery: near an end the
    /// cable does not touch the robot that end is tied to.
    int robot{-1};
    Vec3 velocity{};    // m/s of the body's centre of mass
    Vec3 angular{};     // rad/s of the body
    Vec3 com{};         // m, the body's centre of mass, world
    double reach{0.0};  // m, a sphere about pose.position that holds the shape (∞ for a plane)
};

/// Where a point is against a shape's surface: the signed distance (negative inside), the unit
/// normal out of the shape and the nearest surface point (exact but for hulls, where the
/// farthest face plane stands in near edges, and height fields, measured along the slope).
struct SurfacePoint {
    double distance{0.0};
    Vec3 normal{0.0, 0.0, 1.0};
    Vec3 point{};
};
[[nodiscard]] SurfacePoint surfacePoint(const Obstacle& shape, const Vec3& at);
/// The radius of a sphere about the shape's origin that holds it (∞ for a plane).
[[nodiscard]] double reachOf(ShapeType type, const Vec3& size, const ConvexHull* hull);

struct Air {
    Vec3 gravity{0.0, 0.0, -9.80665};  // m/s²
    double density{0.0};               // kg/m³; 0: no drag
    Vec3 wind{};                       // m/s
};

/// What the cable did to a moving body over the last step: a force at its centre of mass and a
/// torque about it.
struct Push {
    Vec3 force{};
    Vec3 torque{};
};

class Cable {
public:
    /// A cable from `anchor` to `attach`, laid in a sag in the vertical plane through them so that
    /// its length is params.length (straight if that is not longer than their distance).
    /// `bodies` is how many moving bodies obstacles may name.
    Cable(const CableParams& params, const Vec3& anchor, const Vec3& attach, std::size_t bodies);

    /// One step of `dt`: the ends move in a straight line from where they were to these.
    void step(double dt, const Vec3& anchor, const Vec3& attach,
              std::span<const Obstacle> obstacles, const Air& air);
    /// The robots the ends are tied to (Obstacle::robot), -1 for none.
    void setEndRobots(int anchorRobot, int attachRobot) {
        anchorRobot_ = anchorRobot;
        attachRobot_ = attachRobot;
    }

    // What the last step found, averaged over its substeps.
    [[nodiscard]] double tensionAttach() const { return tensionAttach_; }  // N
    [[nodiscard]] double tensionAnchor() const { return tensionAnchor_; }  // N
    /// The cable's pull on the attach point (N, world): the last segment's tension along it, and
    /// the weight and drag of the half segment the end carries.
    [[nodiscard]] const Vec3& forceOnAttach() const { return forceOnAttach_; }
    /// The same at the anchor.
    [[nodiscard]] const Vec3& forceOnAnchor() const { return forceOnAnchor_; }
    [[nodiscard]] double length() const;                             // m paid out
    [[nodiscard]] double winchSpeed() const { return winchSpeed_; }  // m/s, + paying out
    [[nodiscard]] int contacts() const { return contacts_; }         // masses touching anything
    [[nodiscard]] bool caught() const { return caught_; }  // held somewhere by static friction
    /// Per moving body (the obstacles' index), what contacts did to it.
    [[nodiscard]] std::span<const Push> pushes() const { return pushes_; }

    [[nodiscard]] std::span<const Vec3> nodes() const { return {x_.data(), count_}; }
    /// The most masses the cable can have, at its drum's end.
    [[nodiscard]] std::size_t capacity() const { return x_.size(); }
    /// The tension of each segment (N), node i to i + 1.
    [[nodiscard]] std::span<const double> tensions() const { return {tension_.data(), count_ - 1}; }
    [[nodiscard]] const CableParams& params() const { return params_; }

    /// The state as numbers, and back (same parameters and capacity).
    void save(std::vector<double>& out) const;
    [[nodiscard]] bool restore(std::span<const double> in);
    void shift(const Vec3& by);

private:
    void layOut(const Vec3& anchor, const Vec3& attach);
    void updateMasses();
    void reelWinch(double dt);
    void splitOrMerge();
    void solveLengths(double h, std::span<double> lambda);
    void solveBends(double h);
    void solveObstacles(double h, std::span<const Obstacle> obstacles, double share);
    void findSelfPairs(double margin);
    void solveSelf();

    CableParams params_;
    double radius_{0.0};
    double maxLength_{0.0};
    std::size_t count_{0};  // masses in use, ends included
    std::vector<Vec3> x_, v_, p_, force_;
    std::vector<double> rest_;  // per segment, m
    std::vector<double> mass_;  // per mass, kg
    std::vector<double> w_;     // inverse mass, 0 at the ends
    std::vector<double> tension_;
    std::vector<double> lambda_;
    std::vector<Push> pushes_;
    std::vector<std::uint32_t> near_;  // obstacles near the cable this step
    std::vector<std::uint8_t>
        endInside_;  // per near obstacle: holds an end (bit 0 anchor, 1 attach)
    std::vector<std::array<std::int64_t, 2>> cells_;  // self contact: (cell, segment)
    std::vector<std::array<std::uint32_t, 2>> pairs_;
    Vec3 anchor_{}, attach_{};
    double tensionAttach_{0.0}, tensionAnchor_{0.0}, winchSpeed_{0.0}, anchorFilter_{0.0};
    double winchIntegral_{0.0};
    Vec3 forceOnAttach_{};
    Vec3 forceOnAnchor_{};
    int contacts_{0};
    bool caught_{false};
    int anchorRobot_{-1}, attachRobot_{-1};
    std::vector<std::uint8_t> touching_;
};

}  // namespace lagrange::tether
