// Stub (RFC 0016 WP0): WP-A replaces buildTree, linkFrames and robotMass. compose and toFrame
// are complete (the URDF convention), so WP-D can place shapes before WP-A lands.
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <cmath>

namespace lagrange::viewer::builder {

namespace {

using Quat = std::array<double, 4>;  // w x y z

Quat multiply(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

std::array<double, 3> rotate(const Quat& q, const std::array<double, 3>& v) {
    const Quat p{0.0, v[0], v[1], v[2]};
    const Quat conjugate{q[0], -q[1], -q[2], -q[3]};
    const Quat r = multiply(multiply(q, p), conjugate);
    return {r[1], r[2], r[3]};
}

}  // namespace

Tree buildTree(const Document& document) {
    Tree tree;
    tree.parentJoint.assign(document.links.size(), -1);
    tree.error = "the link tree is not implemented yet (RFC 0016 WP-A)";
    return tree;
}

std::vector<Frame> linkFrames(const Document& document, const Tree& /*tree*/,
                              const Preview& /*preview*/) {
    return std::vector<Frame>(document.links.size());  // identity until WP-A
}

MassProperties robotMass(const Document& /*document*/) {
    return {};  // not implemented yet (WP-A)
}

Frame compose(const Frame& a, const Frame& b) {
    Frame out;
    const std::array<double, 3> moved = rotate(a.rotation, b.position);
    for (std::size_t k = 0; k < 3; ++k) {
        out.position.at(k) = a.position.at(k) + moved.at(k);
    }
    out.rotation = multiply(a.rotation, b.rotation);
    return out;
}

Frame toFrame(const Pose& pose) {
    // URDF: R = Rz(yaw) · Ry(pitch) · Rx(roll).
    const double hr = pose.rpy[0] * 0.5;
    const double hp = pose.rpy[1] * 0.5;
    const double hy = pose.rpy[2] * 0.5;
    const double cr = std::cos(hr);
    const double sr = std::sin(hr);
    const double cp = std::cos(hp);
    const double sp = std::sin(hp);
    const double cy = std::cos(hy);
    const double sy = std::sin(hy);
    Frame frame;
    frame.position = pose.xyz;
    frame.rotation = {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
    return frame;
}

}  // namespace lagrange::viewer::builder
