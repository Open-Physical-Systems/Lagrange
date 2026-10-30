#pragma once

// The plugin SDK (RFC 0008 §2): the world a plugin sees through the C ABI 1.1, read once into
// plain C++ containers. Header-only, on lagrange_plugin.h and the standard library alone, so a
// C++ plugin (an engine wrapper, a sensor) links nothing of Lagrange's and the ABI stays C.
//
//     lagrange_status declare(void* self, const lagrange_world* w, ...) {
//         const auto world = lagrange::plugin_sdk::World::read(*host, w);
//         if (!world) { return LAGRANGE_UNSUPPORTED; }  // a 1.0 host
//         for (const auto& body : world->bodies) { ... }
//     }

#include "lagrange_plugin.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::plugin_sdk {

/// x, y, z, then the unit quaternion w, x, y, z (m, region-local frame).
using Pose = std::array<double, 7>;

inline constexpr std::uint32_t kWorldBody = LAGRANGE_WORLD_BODY;

struct Body {
    std::string name;
    std::uint32_t parent{kWorldBody};
    std::uint32_t robot{0};
    double mass{0.0};                 // kg
    std::array<double, 3> com{};      // m, body frame
    std::array<double, 6> inertia{};  // kg·m² about the centre of mass: xx, yy, zz, xy, xz, yz
    Pose poseInParent{};              // with every joint at zero
    Pose restPose{};                  // in the world, with every joint at zero
    bool kinematic{false};
};

struct Joint {
    std::string name;
    lagrange_joint_type type{LAGRANGE_JOINT_FIXED};
    std::uint32_t parent{kWorldBody};
    std::uint32_t child{kWorldBody};
    std::array<double, 3> axis{};   // child body frame
    double lower{0.0}, upper{0.0};  // rad or m; equal: unlimited
    double effortLimit{0.0}, velocityLimit{0.0}, armature{0.0}, initialPosition{0.0};
    bool actuated{false};
};

struct Collider {
    std::uint32_t body{kWorldBody};
    lagrange_shape_type shape{LAGRANGE_SHAPE_BOX};
    std::array<double, 3> size{};
    Pose pose{};  // in the body, or in the world for fixed scenery
    double friction{1.0};
    std::vector<std::array<double, 3>> points;            // a convex hull
    std::vector<std::array<std::uint32_t, 3>> triangles;  // a convex hull, outward
    std::uint32_t columns{0}, rows{0};                    // a height field
    std::array<double, 2> cell{};
    std::vector<double> heights;
};

struct Sensor {
    std::string name;
    std::string kind;  // "lidar", "imu", ... or a custom sensor's kind
    std::uint32_t body{kWorldBody};
    Pose poseInBody{};
    double rate{0.0};  // Hz
};

/// Whether `host` has the 1.1 functions (a 1.0 host's table ends before them).
[[nodiscard]] inline bool hasWorldQueries(const lagrange_host_api& host) {
    return host.struct_size >=
           offsetof(lagrange_host_api, scene_overlap_sphere) + sizeof(host.scene_overlap_sphere);
}

struct World {
    std::vector<Body> bodies;
    std::vector<Joint> joints;
    std::vector<Collider> colliders;
    std::vector<Sensor> sensors;
    std::array<double, 3> gravity{};       // m/s², region-local
    std::array<double, 3> regionOrigin{};  // world ENU of the region-local frame, m

    /// The whole world, in declare() or init(); nothing from a 1.0 host.
    [[nodiscard]] static std::optional<World> read(const lagrange_host_api& host,
                                                   const lagrange_world* world) {
        if (!hasWorldQueries(host)) {
            return std::nullopt;
        }
        World out;
        for (std::uint32_t b = 0; b < host.world_body_count(world); ++b) {
            lagrange_body_info in{};
            in.struct_size = sizeof in;
            if (host.world_body(world, b, &in) == 0) {
                continue;
            }
            Body body;
            body.name = in.name != nullptr ? in.name : "";
            body.parent = in.parent;
            body.robot = in.robot;
            body.mass = in.mass;
            copy(in.com, body.com);
            copy(in.inertia, body.inertia);
            copy(in.pose_in_parent, body.poseInParent);
            copy(in.rest_pose, body.restPose);
            body.kinematic = in.kinematic != 0;
            out.bodies.push_back(std::move(body));
        }
        for (std::uint32_t j = 0; j < host.world_joint_count(world); ++j) {
            lagrange_joint_info in{};
            in.struct_size = sizeof in;
            if (host.world_joint(world, j, &in) == 0) {
                continue;
            }
            Joint joint;
            joint.name = in.name != nullptr ? in.name : "";
            joint.type = in.type;
            joint.parent = in.parent;
            joint.child = in.child;
            copy(in.axis, joint.axis);
            joint.lower = in.lower;
            joint.upper = in.upper;
            joint.effortLimit = in.effort_limit;
            joint.velocityLimit = in.velocity_limit;
            joint.armature = in.armature;
            joint.initialPosition = in.initial_position;
            joint.actuated = in.actuated != 0;
            out.joints.push_back(std::move(joint));
        }
        for (std::uint32_t c = 0; c < host.world_collider_count(world); ++c) {
            lagrange_collider_info in{};
            in.struct_size = sizeof in;
            if (host.world_collider(world, c, &in) == 0) {
                continue;
            }
            Collider collider;
            collider.body = in.body;
            collider.shape = in.shape;
            copy(in.size, collider.size);
            copy(in.pose, collider.pose);
            collider.friction = in.friction;
            for (std::uint32_t p = 0; p < in.point_count && in.points != nullptr; ++p) {
                collider.points.push_back(
                    {in.points[3 * p], in.points[3 * p + 1], in.points[3 * p + 2]});
            }
            for (std::uint32_t t = 0; t < in.triangle_count && in.triangles != nullptr; ++t) {
                collider.triangles.push_back(
                    {in.triangles[3 * t], in.triangles[3 * t + 1], in.triangles[3 * t + 2]});
            }
            collider.columns = in.columns;
            collider.rows = in.rows;
            copy(in.cell, collider.cell);
            if (in.heights != nullptr) {
                collider.heights.assign(
                    in.heights, in.heights + static_cast<std::size_t>(in.columns) * in.rows);
            }
            out.colliders.push_back(std::move(collider));
        }
        for (std::uint32_t s = 0; s < host.world_sensor_count(world); ++s) {
            lagrange_sensor_info in{};
            in.struct_size = sizeof in;
            if (host.world_sensor(world, s, &in) == 0) {
                continue;
            }
            Sensor sensor;
            sensor.name = in.name != nullptr ? in.name : "";
            sensor.kind = in.kind != nullptr ? in.kind : "";
            sensor.body = in.body;
            copy(in.pose_in_body, sensor.poseInBody);
            sensor.rate = in.rate;
            out.sensors.push_back(std::move(sensor));
        }
        lagrange_site_info site{};
        site.struct_size = sizeof site;
        if (host.world_site(world, &site) != 0) {
            copy(site.gravity, out.gravity);
            copy(site.region_origin, out.regionOrigin);
        }
        return out;
    }

private:
    template <std::size_t N>
    static void copy(const double (&from)[N], std::array<double, N>& to) {
        for (std::size_t i = 0; i < N; ++i) {
            to[i] = from[i];
        }
    }
};

}  // namespace lagrange::plugin_sdk
