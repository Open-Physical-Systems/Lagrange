// The link tree and forward kinematics of a document (RFC 0016 §3, SPEC "Joints").
#include "Math.hpp"
#include "TreeCheck.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

namespace {

using detail::multiply;
using detail::Quat;

std::array<double, 3> rotate(const Quat& q, const std::array<double, 3>& v) {
    const Quat p{0.0, v[0], v[1], v[2]};
    const Quat conjugate{q[0], -q[1], -q[2], -q[3]};
    const Quat r = multiply(multiply(q, p), conjugate);
    return {r[1], r[2], r[3]};
}

}  // namespace

namespace {

/// Index of the first link named `name`, or -1.
int linkIndex(const Document& document, std::string_view name) {
    for (std::size_t i = 0; i < document.links.size(); ++i) {
        if (document.links[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int jointIndex(const Document& document, std::string_view name) {
    for (std::size_t i = 0; i < document.joints.size(); ++i) {
        if (document.joints[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::string inQuotes(std::string_view name) {
    return "'" + std::string(name) + "'";
}

/// The joint's position: the preview's, or its mimic leader's through multiplier and offset.
double jointPosition(const Document& document, std::size_t joint, const Preview& preview,
                     int depth) {
    const Joint& j = document.joints[joint];
    if (j.type == Joint::Type::Fixed) {
        return 0.0;
    }
    if (j.mimic && depth < static_cast<int>(document.joints.size())) {
        const auto& [leader, multiplier, offset] = *j.mimic;
        const int found = jointIndex(document, leader);
        if (found >= 0 && static_cast<std::size_t>(found) != joint) {
            return multiplier * jointPosition(document, static_cast<std::size_t>(found), preview,
                                              depth + 1) +
                   offset;
        }
    }
    const auto it = preview.find(j.name);
    return it == preview.end() ? 0.0 : it->second;
}

/// The joint's motion at position `q`, in the joint frame.
Frame jointMotion(const Joint& joint, double q) {
    Frame motion;
    const auto& a = joint.axis;
    const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (norm <= 0.0 || !std::isfinite(norm) || q == 0.0) {
        return motion;  // a zero axis is a Problem; it does not move
    }
    const std::array<double, 3> unit{a[0] / norm, a[1] / norm, a[2] / norm};
    switch (joint.type) {
        case Joint::Type::Revolute:
        case Joint::Type::Continuous: {
            const double s = std::sin(q * 0.5);
            motion.rotation = {std::cos(q * 0.5), unit[0] * s, unit[1] * s, unit[2] * s};
            break;
        }
        case Joint::Type::Prismatic:
            motion.position = {unit[0] * q, unit[1] * q, unit[2] * q};
            break;
        case Joint::Type::Fixed:
            break;
    }
    return motion;
}

}  // namespace

namespace detail {

TreeAnalysis analyseTree(const Document& document) {
    TreeAnalysis out;
    const std::size_t links = document.links.size();
    out.parentJoint.assign(links, -1);
    std::vector<bool> parentMissing(links, false);  // the link's parent joint has no parent link
    for (std::size_t j = 0; j < document.joints.size(); ++j) {
        const Joint& joint = document.joints[j];
        const int parent = linkIndex(document, joint.parent);
        const int child = linkIndex(document, joint.child);
        if (parent < 0) {
            out.findings.push_back({joint.name, "joint " + inQuotes(joint.name) +
                                                    " names parent link " + inQuotes(joint.parent) +
                                                    ", which does not exist; pick a link"});
        }
        if (child < 0) {
            out.findings.push_back({joint.name, "joint " + inQuotes(joint.name) +
                                                    " names child link " + inQuotes(joint.child) +
                                                    ", which does not exist; pick a link"});
        }
        if (parent >= 0 && parent == child) {
            out.findings.push_back({joint.name, "joint " + inQuotes(joint.name) + " joins link " +
                                                    inQuotes(joint.parent) +
                                                    " to itself; pick another child"});
            continue;
        }
        if (child < 0) {
            continue;
        }
        auto& slot = out.parentJoint[static_cast<std::size_t>(child)];
        if (slot >= 0) {
            out.findings.push_back(
                {joint.child, "link " + inQuotes(joint.child) + " is the child of two joints, " +
                                  inQuotes(document.joints[static_cast<std::size_t>(slot)].name) +
                                  " and " + inQuotes(joint.name) +
                                  "; a link has one parent, remove one joint"});
            continue;
        }
        slot = static_cast<int>(j);
        parentMissing[static_cast<std::size_t>(child)] = parent < 0;
    }
    for (std::size_t i = 0; i < links; ++i) {
        if (out.parentJoint[i] < 0) {
            out.roots.push_back(static_cast<int>(i));
        }
    }
    if (links > 0 && out.roots.empty()) {
        out.findings.push_back({"",
                                "every link is the child of a joint, so the joints make a "
                                "loop; a robot needs one root link that no joint moves"});
    } else if (out.roots.size() > 1) {
        std::string names;
        for (const int root : out.roots) {
            names += (names.empty() ? "" : ", ") +
                     inQuotes(document.links[static_cast<std::size_t>(root)].name);
        }
        out.findings.push_back({document.links[static_cast<std::size_t>(out.roots[1])].name,
                                "links " + names +
                                    " have no parent joint; a robot has one root, so join the "
                                    "others to the tree with a joint"});
    }
    // Loops: walk up from every link; a walk that comes back to a link it passed is a loop.
    std::vector<int> state(links, 0);  // 0 unseen, 1 on the current walk, 2 done
    for (std::size_t start = 0; start < links; ++start) {
        std::vector<std::size_t> walk;
        std::size_t at = start;
        while (state[at] == 0) {
            state[at] = 1;
            walk.push_back(at);
            const int joint = out.parentJoint[at];
            if (joint < 0 || parentMissing[at]) {
                break;
            }
            at = static_cast<std::size_t>(
                linkIndex(document, document.joints[static_cast<std::size_t>(joint)].parent));
        }
        if (state[at] == 1 && out.parentJoint[at] >= 0 && !parentMissing[at]) {
            // `at` is on this walk and was reached again: the loop is from `at` to the end.
            std::string names;
            bool inLoop = false;
            for (const std::size_t link : walk) {
                inLoop = inLoop || link == at;
                if (inLoop) {
                    names += (names.empty() ? "" : " → ") + inQuotes(document.links[link].name);
                }
            }
            out.findings.push_back(
                {document.links[at].name, "links " + names +
                                              " form a loop of joints; a URDF robot is "
                                              "a tree, so remove one of these joints"});
        }
        for (const std::size_t link : walk) {
            state[link] = 2;
        }
    }
    return out;
}

}  // namespace detail

Tree buildTree(const Document& document) {
    const detail::TreeAnalysis analysis = detail::analyseTree(document);
    Tree tree;
    tree.parentJoint = analysis.parentJoint;
    if (analysis.roots.size() == 1) {
        tree.root = analysis.roots.front();
    }
    if (document.links.empty()) {
        tree.error = "the robot has no link";
    } else if (!analysis.findings.empty()) {
        tree.error = analysis.findings.front().message;
    }
    tree.ok = tree.error.empty() && tree.root >= 0;
    return tree;
}

std::vector<Frame> linkFrames(const Document& document, const Tree& tree, const Preview& preview) {
    std::vector<Frame> frames(document.links.size());
    if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= document.links.size() ||
        tree.parentJoint.size() != document.links.size()) {
        return frames;
    }
    std::vector<bool> placed(document.links.size(), false);
    placed[static_cast<std::size_t>(tree.root)] = true;
    std::deque<std::size_t> queue{static_cast<std::size_t>(tree.root)};
    while (!queue.empty()) {
        const std::size_t parent = queue.front();
        queue.pop_front();
        for (std::size_t j = 0; j < document.joints.size(); ++j) {
            const Joint& joint = document.joints[j];
            if (joint.parent != document.links[parent].name) {
                continue;
            }
            const int child = linkIndex(document, joint.child);
            if (child < 0 || placed[static_cast<std::size_t>(child)] ||
                tree.parentJoint[static_cast<std::size_t>(child)] != static_cast<int>(j)) {
                continue;
            }
            const double q = jointPosition(document, j, preview, 0);
            frames[static_cast<std::size_t>(child)] =
                compose(compose(frames[parent], toFrame(joint.origin)), jointMotion(joint, q));
            placed[static_cast<std::size_t>(child)] = true;
            queue.push_back(static_cast<std::size_t>(child));
        }
    }
    return frames;
}

MassProperties robotMass(const Document& document) {
    const Tree tree = buildTree(document);
    const std::vector<Frame> frames = linkFrames(document, tree, {});
    std::vector<MassProperties> parts;
    for (std::size_t i = 0; i < document.links.size(); ++i) {
        const std::optional<MassProperties> link = linkMass(document.links[i]);
        if (!link || link->mass <= 0.0) {
            continue;
        }
        const detail::Mat3 r = detail::matrixOf(frames[i].rotation);
        const detail::Vec3 moved = detail::apply(r, link->com);
        MassProperties part;
        part.mass = link->mass;
        for (std::size_t k = 0; k < 3; ++k) {
            part.com.at(k) = frames[i].position.at(k) + moved.at(k);
        }
        part.inertia = detail::rotateTensor(r, link->inertia);
        parts.push_back(part);
    }
    MassProperties out;
    for (const MassProperties& part : parts) {
        out.mass += part.mass;
    }
    if (out.mass <= 0.0) {
        return {};
    }
    for (const MassProperties& part : parts) {
        for (std::size_t k = 0; k < 3; ++k) {
            out.com.at(k) += part.mass * part.com.at(k) / out.mass;
        }
    }
    for (const MassProperties& part : parts) {
        const detail::Vec3 d{part.com[0] - out.com[0], part.com[1] - out.com[1],
                             part.com[2] - out.com[2]};
        const detail::Tensor6 shifted = detail::shiftTensor(part.inertia, part.mass, d);
        for (std::size_t k = 0; k < 6; ++k) {
            out.inertia.at(k) += shifted.at(k);
        }
    }
    return out;
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
