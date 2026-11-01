// See RobotCheck.hpp. A static balance: moments of the links' weights about each joint's axis.
#include "lagrange/server/RobotCheck.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace lagrange::server {

namespace {

using Vec3 = std::array<double, 3>;

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 minus(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double length(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

bool isMovable(JointType type) {
    return type == JointType::Revolute || type == JointType::Continuous ||
           type == JointType::Prismatic;
}

}  // namespace

std::vector<JointLoad> jointLoads(const World& world) {
    const World::Bodies& bodies = world.bodies;
    const World::Joints& joints = world.joints;
    std::vector<JointLoad> loads;
    for (const JointType type : joints.type) {
        if (type == JointType::Free) {
            return loads;
        }
    }
    const std::vector<Pose> poses = restPoses(world);
    const std::size_t count = bodies.names.size();
    // Whether body b hangs from body a, by walking up from b.
    const auto beyond = [&bodies](std::size_t b, std::size_t a) {
        for (BodyId at{static_cast<std::uint32_t>(b)}; at != kWorldBody;
             at = bodies.parent[at.index]) {
            if (at.index == a) {
                return true;
            }
        }
        return false;
    };
    const double g = length(world.gravityLocal);
    const Vec3 weight = world.gravityLocal;
    for (std::size_t j = 0; j < joints.names.size(); ++j) {
        if (!isMovable(joints.type[j]) || joints.child[j] == kWorldBody) {
            continue;
        }
        const std::size_t child = joints.child[j].index;
        const Pose& at = poses[child];
        const Vec3 axis = rotate(at.orientation, joints.axis[j]);
        const bool slide = joints.type[j] == JointType::Prismatic;
        JointLoad load;
        load.name = joints.names[j];
        load.effortLimit = joints.effortLimit[j];
        double rest = 0.0;
        double worst = 0.0;
        for (std::size_t b = 0; b < count; ++b) {
            if (!beyond(b, child)) {
                continue;
            }
            const double mass = bodies.mass[b];
            const Vec3 centre = compose(poses[b], Pose{bodies.comLocal[b], {1, 0, 0, 0}}).position;
            load.massCarried += mass;
            const Vec3 force{weight[0] * mass, weight[1] * mass, weight[2] * mass};
            rest += slide ? dot(force, axis) : dot(cross(minus(centre, at.position), force), axis);
            // Straight out: the links' lengths in a line from the joint, to this centre of mass.
            double reach = length(minus(centre, poses[b].position));
            for (BodyId up{static_cast<std::uint32_t>(b)}; up.index != child;
                 up = bodies.parent[up.index]) {
                reach += length(bodies.poseInParent[up.index].position);
            }
            worst += slide ? mass * g : mass * g * reach;
        }
        // A vertical axis that nothing above can tilt never carries the weight.
        bool tilts = false;
        for (BodyId up = joints.parent[j]; up != kWorldBody; up = bodies.parent[up.index]) {
            for (std::size_t k = 0; k < joints.names.size(); ++k) {
                tilts = tilts || (joints.child[k] == up && isMovable(joints.type[k]));
            }
        }
        const bool vertical =
            g > 0.0 && std::abs(dot(axis, weight)) / g > 1.0 - 1e-9;  // a unit axis along gravity
        load.atRest = std::abs(rest);
        load.worst = !slide && vertical && !tilts ? 0.0 : worst;
        loads.push_back(std::move(load));
    }
    for (JointLoad& load : loads) {
        for (const SchemaInstance& s : world.schemaInstances) {
            if (s.schema != "ElectricalMotorAPI_1") {
                continue;
            }
            const auto target = s.rel("elec:joint");
            if (target.empty()) {
                continue;
            }
            const auto named = std::find(joints.names.begin(), joints.names.end(), load.name);
            const auto j = static_cast<std::size_t>(named - joints.names.begin());
            if (joints.prim[j] == target[0]) {
                load.motorMost = s.real("elec:torqueConstant").value_or(0.0) *
                                 s.real("elec:maxCurrent").value_or(0.0) *
                                 s.real("elec:gearRatio").value_or(1.0) *
                                 s.real("elec:gearEfficiency").value_or(1.0);
            }
        }
    }
    return loads;
}

}  // namespace lagrange::server
