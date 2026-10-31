// MJCF -> world tables and scene, from MuJoCo's compiled model (roadmap 0.2-G4). Reading the
// compiled mjModel rather than the XML gets defaults classes, includes, inertia from geoms and
// mesh processing exactly as MuJoCo itself does them.
#include "Description.hpp"
#include "lagrange/importers/Importers.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace lagrange::importers {

namespace {

using detail::CollisionDesc;
using detail::Description;
using detail::Geom;
using detail::GeomType;
using detail::JointDesc;
using detail::LinkDesc;
using detail::SensorDesc;
using detail::VisualDesc;

// MuJoCo draws geom groups 0, 1 and 2 by default; higher groups hold collision-only geoms.
constexpr int kVisibleGroups = 3;

struct ModelDeleter {
    void operator()(mjModel* m) const { mj_deleteModel(m); }
};

std::string nameOf(const mjModel& m, mjtObj type, int id) {
    const char* name = mj_id2name(&m, type, id);
    return name != nullptr ? name : std::string();
}

Pose poseAt(const mjtNum* position, const mjtNum* quaternion) {
    return {{position[0], position[1], position[2]},
            {quaternion[0], quaternion[1], quaternion[2], quaternion[3]}};
}

/// Rotation matrix (row-major) of a unit quaternion.
std::array<double, 9> matrix(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
            2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

/// Principal moments in a frame turned by q, as a full tensor in the body axes: R D Rᵀ.
std::array<double, 6> fromPrincipal(const mjtNum* moments, const std::array<double, 4>& q) {
    const std::array<double, 9> r = matrix(q);
    std::array<double, 9> out{};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int k = 0; k < 3; ++k) {
                out[a * 3 + b] += r[a * 3 + k] * moments[k] * r[b * 3 + k];
            }
        }
    }
    return {out[0], out[4], out[8], out[1], out[2], out[5]};
}

/// A compiled mesh as flat-shaded triangles (three vertices per face, the face's normal).
TriMesh triangles(const mjModel& m, int mesh) {
    TriMesh out;
    const int vertices = m.mesh_vertadr[mesh];
    const int first = m.mesh_faceadr[mesh];
    for (int f = first; f < first + m.mesh_facenum[mesh]; ++f) {
        std::array<std::array<float, 3>, 3> corner{};
        for (int c = 0; c < 3; ++c) {
            const float* v =
                m.mesh_vert + 3 * static_cast<std::ptrdiff_t>(vertices + m.mesh_face[3 * f + c]);
            corner[c] = {v[0], v[1], v[2]};
        }
        const std::array<float, 3> e1{corner[1][0] - corner[0][0], corner[1][1] - corner[0][1],
                                      corner[1][2] - corner[0][2]};
        const std::array<float, 3> e2{corner[2][0] - corner[0][0], corner[2][1] - corner[0][1],
                                      corner[2][2] - corner[0][2]};
        std::array<float, 3> n{e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                               e1[0] * e2[1] - e1[1] * e2[0]};
        const float length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (length > 0.0F) {
            n = {n[0] / length, n[1] / length, n[2] / length};
        }
        for (const std::array<float, 3>& p : corner) {
            out.positions.insert(out.positions.end(), p.begin(), p.end());
            out.normals.insert(out.normals.end(), n.begin(), n.end());
            out.indices.push_back(static_cast<std::uint32_t>(out.indices.size()));
        }
    }
    return out;
}

/// The geom's shape; false when it has none the description can hold.
bool geomOf(const mjModel& m, int g, Geom& geom, Model& model, std::map<int, std::uint32_t>& meshes,
            Description& d) {
    const mjtNum* size = m.geom_size + 3 * static_cast<std::ptrdiff_t>(g);
    geom.pose = poseAt(m.geom_pos + 3 * static_cast<std::ptrdiff_t>(g),
                       m.geom_quat + 4 * static_cast<std::ptrdiff_t>(g));
    geom.size = {size[0], size[1], size[2]};
    switch (m.geom_type[g]) {
        case mjGEOM_PLANE:
            geom.type = GeomType::Plane;
            return true;
        case mjGEOM_SPHERE:
            geom.type = GeomType::Sphere;
            return true;
        case mjGEOM_CAPSULE:
            geom.type = GeomType::Capsule;
            return true;
        case mjGEOM_ELLIPSOID:
            geom.type = GeomType::Ellipsoid;
            return true;
        case mjGEOM_CYLINDER:
            geom.type = GeomType::Cylinder;
            return true;
        case mjGEOM_BOX:
            geom.type = GeomType::Box;
            return true;
        case mjGEOM_MESH: {
            const int mesh = m.geom_dataid[g];
            if (mesh < 0) {
                return false;
            }
            auto [it, added] = meshes.emplace(mesh, static_cast<std::uint32_t>(d.meshes.size()));
            if (added) {
                d.meshes.push_back(triangles(m, mesh));
            }
            geom.type = GeomType::Mesh;
            geom.mesh = it->second;
            geom.size = {1.0, 1.0, 1.0};  // the compiled vertices are already scaled
            return true;
        }
        case mjGEOM_HFIELD: {
            const int h = m.geom_dataid[g];
            if (h < 0) {
                return false;
            }
            HeightField field;
            field.columns = static_cast<std::uint32_t>(m.hfield_ncol[h]);
            field.rows = static_cast<std::uint32_t>(m.hfield_nrow[h]);
            const mjtNum* extent = m.hfield_size + 4 * static_cast<std::ptrdiff_t>(h);
            field.cellX = 2.0 * extent[0] / std::max<int>(m.hfield_ncol[h] - 1, 1);
            field.cellY = 2.0 * extent[1] / std::max<int>(m.hfield_nrow[h] - 1, 1);
            const float* data = m.hfield_data + m.hfield_adr[h];
            field.heights.reserve(static_cast<std::size_t>(field.columns) * field.rows);
            for (std::uint32_t i = 0; i < field.columns * field.rows; ++i) {
                field.heights.push_back(data[i] * extent[2]);  // data is 0 to 1
            }
            geom.type = GeomType::HeightField;
            geom.mesh = static_cast<std::uint32_t>(d.heightFields.size());
            d.heightFields.push_back(std::move(field));
            return true;
        }
        default:
            detail::note(
                model, "geom '" + nameOf(m, mjOBJ_GEOM, g) + "' is an SDF, which is not supported");
            return false;
    }
}

}  // namespace

Expected<Model, std::string> importMjcf(const std::string& path, const ImportOptions& options) {
    std::array<char, 1024> error{};
    const std::unique_ptr<mjModel, ModelDeleter> compiled(
        mj_loadXML(path.c_str(), nullptr, error.data(), static_cast<int>(error.size())));
    if (!compiled) {
        return Unexpected{path + " is not a valid MJCF file: " + std::string(error.data())};
    }
    const mjModel& m = *compiled;

    Model model;
    model.sourcePath = path;
    model.world.gravityLocal = {m.opt.gravity[0], m.opt.gravity[1], m.opt.gravity[2]};
    Description d;
    d.name = m.names[0] != '\0' ? std::string(m.names)  // the <mujoco model="..."> attribute
                                : std::filesystem::path(path).stem().string();

    // Links: the world body's own geoms (scenery) as a static link, then one per body.
    const auto bodies = static_cast<int>(m.nbody);
    std::vector<Pose> world(bodies);
    std::vector<std::string> linkName(bodies);
    for (int b = 1; b < bodies; ++b) {
        world[b] = compose(world[m.body_parentid[b]],
                           poseAt(m.body_pos + 3 * static_cast<std::ptrdiff_t>(b),
                                  m.body_quat + 4 * static_cast<std::ptrdiff_t>(b)));
        linkName[b] = nameOf(m, mjOBJ_BODY, b);
        if (linkName[b].empty()) {
            linkName[b] = "body" + std::to_string(b);
        }
    }
    linkName[0] = "worldbody";
    std::vector<std::ptrdiff_t> linkOf(bodies, -1);
    const auto ensureLink = [&](int b) {
        if (linkOf[b] < 0) {
            LinkDesc link;
            link.name = linkName[b];
            link.pose = world[b];
            if (b > 0) {
                link.mass = m.body_mass[b];
                link.com = {m.body_ipos[3 * b], m.body_ipos[3 * b + 1], m.body_ipos[3 * b + 2]};
                link.inertia = fromPrincipal(m.body_inertia + 3 * static_cast<std::ptrdiff_t>(b),
                                             {m.body_iquat[4 * b], m.body_iquat[4 * b + 1],
                                              m.body_iquat[4 * b + 2], m.body_iquat[4 * b + 3]});
            }
            linkOf[b] = static_cast<std::ptrdiff_t>(d.links.size());
            d.links.push_back(std::move(link));
        }
        return linkOf[b];
    };
    for (int b = 1; b < bodies; ++b) {
        ensureLink(b);
    }

    // Joints, in order per body; a body without joints is welded to its parent.
    std::vector<bool> actuated(static_cast<std::size_t>(m.njnt), false);
    std::vector<double> effort(static_cast<std::size_t>(m.njnt), 0.0);
    for (int a = 0; a < static_cast<int>(m.nactuator); ++a) {
        if (m.actuator_trntype[a] != mjTRN_JOINT) {
            detail::note(model, "actuator '" + nameOf(m, mjOBJ_ACTUATOR, a) +
                                    "' does not drive a joint directly; it is not used");
            continue;
        }
        const int joint = m.actuator_trnid[2 * a];
        actuated[joint] = true;
        const double gear = std::abs(m.actuator_gear[6 * m.actuator_outadr[a]]);
        const int control = m.actuator_ctrladr[a];
        if (m.actuator_forcelimited[a] != 0) {
            effort[joint] = std::max(std::abs(m.actuator_forcerange[2 * a]),
                                     std::abs(m.actuator_forcerange[2 * a + 1]));
        } else if (m.actuator_ctrllimited[control] != 0 && m.actuator_biastype[a] == mjBIAS_NONE) {
            effort[joint] = gear * std::max(std::abs(m.actuator_ctrlrange[2 * control]),
                                            std::abs(m.actuator_ctrlrange[2 * control + 1]));
        }
        if (m.actuator_biastype[a] != mjBIAS_NONE) {
            detail::note(model, "actuator '" + nameOf(m, mjOBJ_ACTUATOR, a) +
                                    "' is a servo; its joint takes torque commands instead");
        }
    }
    for (int b = 1; b < bodies; ++b) {
        const int parent = m.body_parentid[b];
        const int count = m.body_jntnum[b];
        if (count == 0) {
            if (parent != 0) {
                JointDesc weld;
                weld.name = linkName[b] + "_weld";
                weld.parent = linkName[parent];
                weld.child = linkName[b];
                d.joints.push_back(std::move(weld));
            }
            continue;
        }
        for (int k = 0; k < count; ++k) {
            const int j = m.body_jntadr[b] + k;
            JointDesc joint;
            joint.name = nameOf(m, mjOBJ_JOINT, j);
            joint.parent = parent == 0 ? std::string() : linkName[parent];
            joint.child = linkName[b];
            joint.anchor.position = {m.jnt_pos[3 * j], m.jnt_pos[3 * j + 1], m.jnt_pos[3 * j + 2]};
            joint.axis = {m.jnt_axis[3 * j], m.jnt_axis[3 * j + 1], m.jnt_axis[3 * j + 2]};
            const bool limited = m.jnt_limited[j] != 0;
            switch (m.jnt_type[j]) {
                case mjJNT_FREE:
                    joint.type = JointType::Free;
                    joint.anchor = Pose{};
                    joint.axis = {0.0, 0.0, 1.0};
                    if (joint.name.empty()) {
                        joint.name = linkName[b] + "_floating";
                    }
                    break;
                case mjJNT_BALL:
                    joint.type = JointType::Spherical;
                    break;
                case mjJNT_SLIDE:
                    joint.type = JointType::Prismatic;
                    break;
                default:
                    joint.type = limited ? JointType::Revolute : JointType::Continuous;
                    break;
            }
            if (joint.name.empty()) {
                joint.name = linkName[b] + "_joint" + (k == 0 ? "" : std::to_string(k));
            }
            if (limited && joint.type != JointType::Spherical) {
                joint.lower = m.jnt_range[2 * j];
                joint.upper = m.jnt_range[2 * j + 1];
            }
            joint.actuated = actuated[j];
            joint.effort = effort[j];
            const int address = m.jnt_qposadr[j];
            if ((joint.type == JointType::Revolute || joint.type == JointType::Continuous ||
                 joint.type == JointType::Prismatic) &&
                m.qpos0[address] != 0.0) {
                detail::note(model, "joint '" + joint.name +
                                        "' has a reference position; its rest pose is at ref");
            }
            d.joints.push_back(std::move(joint));
        }
    }

    // Geoms: colliders and visuals on their body's link.
    std::map<int, std::uint32_t> meshes;
    for (int g = 0; g < static_cast<int>(m.ngeom); ++g) {
        const int b = m.geom_bodyid[g];
        Geom geom;
        if (!geomOf(m, g, geom, model, meshes, d)) {
            continue;
        }
        const bool collides = m.geom_contype[g] != 0 || m.geom_conaffinity[g] != 0;
        const int materialId = m.geom_matid[g];
        const float* rgba = materialId >= 0
                                ? m.mat_rgba + 4 * static_cast<std::ptrdiff_t>(materialId)
                                : m.geom_rgba + 4 * static_cast<std::ptrdiff_t>(g);
        const bool visible = m.geom_group[g] < kVisibleGroups && rgba[3] > 0.0F;
        if (!collides && !visible) {
            continue;
        }
        LinkDesc& link = d.links[ensureLink(b)];
        if (collides) {
            link.collisions.push_back(CollisionDesc{geom, m.geom_friction[3 * g]});
        }
        if (visible) {
            VisualDesc visual;
            visual.geom = geom;
            if (geom.type == GeomType::HeightField) {
                d.meshes.push_back(flatMesh(d.heightFields[geom.mesh].triangles()));
                visual.geom.type = GeomType::Mesh;
                visual.geom.mesh = static_cast<std::uint32_t>(d.meshes.size() - 1);
                visual.geom.size = {1.0, 1.0, 1.0};
            }
            visual.material = materialId >= 0 ? nameOf(m, mjOBJ_MATERIAL, materialId) : "";
            visual.rgba = {rgba[0], rgba[1], rgba[2], rgba[3]};
            link.visuals.push_back(visual);
        }
    }

    // A site with a gyro or an accelerometer is an IMU, sampled at every physics step.
    std::map<int, SensorDesc> imus;
    for (int s = 0; s < static_cast<int>(m.nsensor); ++s) {
        const int type = m.sensor_type[s];
        if ((type == mjSENS_GYRO || type == mjSENS_ACCELEROMETER) &&
            m.sensor_objtype[s] == mjOBJ_SITE) {
            const int site = m.sensor_objid[s];
            SensorDesc& imu = imus[site];
            imu.name = nameOf(m, mjOBJ_SITE, site);
            if (imu.name.empty()) {
                imu.name = "imu" + std::to_string(site);
            }
            imu.type = SensorType::Imu;
            imu.rateHz = 1.0 / m.opt.timestep;
            imu.pose = poseAt(m.site_pos + 3 * static_cast<std::ptrdiff_t>(site),
                              m.site_quat + 4 * static_cast<std::ptrdiff_t>(site));
            imu.settings[type == mjSENS_GYRO ? "gyro_noise" : "accel_noise"] = m.sensor_noise[s];
            imu.settings.try_emplace("gyro_noise", 0.0);
            imu.settings.try_emplace("accel_noise", 0.0);
        } else {
            detail::note(model, "sensor '" + nameOf(m, mjOBJ_SENSOR, s) +
                                    "' is of an MJCF type that is not simulated yet");
        }
    }
    for (auto& [site, imu] : imus) {
        d.links[ensureLink(m.site_bodyid[site])].sensors.push_back(std::move(imu));
    }

    const bool addGround = options.addGround && !detail::hasGroundPlane(d);
    if (addGround) {
        model.world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{},
                                options.groundFriction);
    }
    if (auto built = detail::build(d, model); !built) {
        return Unexpected{path + ": " + built.error()};
    }
    model.name = d.name;
    if (addGround) {
        addGroundPrim(model);
    }
    return model;
}

}  // namespace lagrange::importers
