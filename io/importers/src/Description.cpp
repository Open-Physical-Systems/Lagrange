#include "Description.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::importers::detail {

namespace {

// Moving bodies need positive mass and inertia in MuJoCo; links without one get this, as do the
// helper frames between the joints of a link with several.
constexpr double kPlaceholderMassKg = 1e-3;
constexpr double kPlaceholderInertia = 1e-6;  // kg·m²
constexpr double kGroundHalfSizeM = 25.0;     // an unbounded plane is drawn this large
constexpr double kPlaneThicknessM = 0.02;

/// The pose of `b` in the frame of `a` (both in the same parent frame).
Pose relative(const Pose& a, const Pose& b) {
    const std::array<double, 4> inverse = conjugate(a.orientation);
    return {rotate(inverse, {b.position[0] - a.position[0], b.position[1] - a.position[1],
                             b.position[2] - a.position[2]}),
            multiply(inverse, b.orientation)};
}

/// Rotation matrix (row-major) of a unit quaternion.
std::array<double, 9> matrix(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
            2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

/// An inertia tensor given in frame A, expressed in frame B where q turns B into A: Rᵀ I R.
std::array<double, 6> inFrame(const std::array<double, 6>& inertia,
                              const std::array<double, 4>& q) {
    const std::array<double, 9> i{inertia[0], inertia[3], inertia[4], inertia[3], inertia[1],
                                  inertia[5], inertia[4], inertia[5], inertia[2]};
    const std::array<double, 9> r = matrix(q);
    std::array<double, 9> ir{};
    std::array<double, 9> out{};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int k = 0; k < 3; ++k) {
                ir[a * 3 + b] += i[a * 3 + k] * r[k * 3 + b];
            }
        }
    }
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            for (int k = 0; k < 3; ++k) {
                out[a * 3 + b] += r[k * 3 + a] * ir[k * 3 + b];
            }
        }
    }
    return {out[0], out[4], out[8], out[1], out[2], out[5]};
}

std::string jointTypeName(JointType type) {
    switch (type) {
        case JointType::Revolute:
            return "revolute";
        case JointType::Continuous:
            return "continuous";
        case JointType::Prismatic:
            return "prismatic";
        case JointType::Spherical:
            return "spherical";
        case JointType::Free:
            return "floating";
        case JointType::Fixed:
            break;
    }
    return "fixed";
}

std::string shapeKey(const char* kind, const std::array<double, 3>& size) {
    return std::string(kind) + " " + std::to_string(size[0]) + " " + std::to_string(size[1]) + " " +
           std::to_string(size[2]);
}

/// Links that never move: no movable joint between them and the world.
std::vector<bool> staticLinks(const Description& d) {
    std::map<std::string, std::size_t, std::less<>> index;
    for (std::size_t l = 0; l < d.links.size(); ++l) {
        index.emplace(d.links[l].name, l);
    }
    std::vector<int> state(d.links.size(), 0);  // 0 unknown, 1 static, 2 moving, 3 visiting
    std::function<bool(std::size_t)> isStatic = [&](std::size_t l) -> bool {
        if (state[l] == 1 || state[l] == 2) {
            return state[l] == 1;
        }
        if (state[l] == 3) {
            return false;  // a cycle: build() reports it
        }
        state[l] = 3;
        bool result = true;
        for (const JointDesc& joint : d.joints) {
            if (joint.child != d.links[l].name) {
                continue;
            }
            if (joint.type != JointType::Fixed) {
                result = false;
            } else if (!joint.parent.empty()) {
                const auto parent = index.find(joint.parent);
                result = result && parent != index.end() && isStatic(parent->second);
            }
        }
        state[l] = result ? 1 : 2;
        return result;
    };
    std::vector<bool> out(d.links.size());
    for (std::size_t l = 0; l < d.links.size(); ++l) {
        out[l] = isStatic(l);
    }
    return out;
}

class Builder {
public:
    Builder(const Description& d, Model& m) : d_(d), m_(m) {}

    Expected<void, std::string> run() {
        for (std::size_t l = 0; l < d_.links.size(); ++l) {
            if (!linkIndex_.emplace(d_.links[l].name, l).second) {
                return Unexpected{"'" + d_.name + "' has two links named '" + d_.links[l].name +
                                  "'"};
            }
        }
        jointsOf_.resize(d_.links.size());
        for (std::size_t j = 0; j < d_.joints.size(); ++j) {
            const JointDesc& joint = d_.joints[j];
            const auto child = linkIndex_.find(joint.child);
            if (child == linkIndex_.end()) {
                return Unexpected{"joint '" + joint.name + "' has an unknown child link '" +
                                  joint.child + "'"};
            }
            if (!joint.parent.empty() && !linkIndex_.contains(joint.parent)) {
                return Unexpected{"joint '" + joint.name + "' has an unknown parent link '" +
                                  joint.parent + "'"};
            }
            std::vector<std::size_t>& of = jointsOf_[child->second];
            if (!of.empty() && d_.joints[of.front()].parent != joint.parent) {
                return Unexpected{"link '" + joint.child + "' is the child of two links"};
            }
            of.push_back(j);
        }
        for (std::size_t l = 0; l < d_.links.size(); ++l) {
            const std::vector<std::size_t>& of = jointsOf_[l];
            const bool welded = std::any_of(of.begin(), of.end(), [this](std::size_t j) {
                return d_.joints[j].type == JointType::Fixed;
            });
            const bool free = std::any_of(of.begin(), of.end(), [this](std::size_t j) {
                return d_.joints[j].type == JointType::Free;
            });
            if ((welded || free) && of.size() > 1) {
                return Unexpected{"link '" + d_.links[l].name +
                                  "' has a fixed or free joint among several joints"};
            }
            if (free && !d_.joints[of.front()].parent.empty()) {
                return Unexpected{"the free joint '" + d_.joints[of.front()].name +
                                  "' does not connect to the world"};
            }
        }
        static_ = staticLinks(d_);

        ScenePrim top;
        top.name = d_.name;
        top.typeName = "Xform";
        topPrim_ = addPrim(std::move(top));

        bodyOf_.assign(d_.links.size(), kWorldBody);
        bodyWorld_.resize(d_.links.size());
        primOf_.assign(d_.links.size(), kNoIndex);
        std::vector<bool> done(d_.links.size(), false);
        // Parents first: roots in file order, then each link's children in joint order.
        std::function<void(std::size_t)> visit = [&](std::size_t l) {
            done[l] = true;
            addLink(l);
            for (const JointDesc& joint : d_.joints) {
                if (joint.parent == d_.links[l].name) {
                    const std::size_t child = linkIndex_.at(joint.child);
                    if (!done[child]) {
                        visit(child);
                    }
                }
            }
        };
        for (std::size_t l = 0; l < d_.links.size(); ++l) {
            if (jointsOf_[l].empty() || d_.joints[jointsOf_[l].front()].parent.empty()) {
                visit(l);
            }
        }
        for (std::size_t l = 0; l < d_.links.size(); ++l) {
            if (!done[l]) {
                return Unexpected{"link '" + d_.links[l].name +
                                  "' is in a loop of joints, which a body tree cannot hold"};
            }
        }
        // A model that floats as one moves its top prim with its root body (the articulation).
        std::size_t roots = 0;
        std::size_t floating = d_.links.size();
        for (std::size_t l = 0; l < d_.links.size(); ++l) {
            if (jointsOf_[l].empty() || d_.joints[jointsOf_[l].front()].parent.empty()) {
                ++roots;
                if (!jointsOf_[l].empty() &&
                    d_.joints[jointsOf_[l].front()].type == JointType::Free) {
                    floating = l;
                }
            }
        }
        if (roots == 1 && floating < d_.links.size()) {
            m_.scene.prims[topPrim_].body = bodyOf_[floating].index;
        }
        return {};
    }

private:
    std::uint32_t addPrim(ScenePrim prim) {
        prim.path = (prim.parent != kNoIndex ? m_.scene.prims[prim.parent].path : std::string()) +
                    "/" + prim.name;
        m_.scene.prims.push_back(std::move(prim));
        return static_cast<std::uint32_t>(m_.scene.prims.size() - 1);
    }

    /// A body, joint or sensor name that is not taken yet in the model.
    std::string unique(const std::vector<std::string>& taken, const std::string& name) {
        std::string out = name;
        while (std::find(taken.begin(), taken.end(), out) != taken.end()) {
            out = d_.name + "_" + out;
        }
        if (out != name) {
            note(m_, "'" + name + "' of '" + d_.name + "' is called '" + out +
                         "', since the name was taken");
        }
        return out;
    }

    BodyId addBody(const std::string& name, BodyId parent, const Pose& parentWorld,
                   const Pose& world, double mass, const std::array<double, 3>& com,
                   const std::array<double, 6>& inertia) {
        const BodyId body = m_.world.addBody(unique(m_.world.bodies.names, name), parent,
                                             relative(parentWorld, world), mass, com, inertia);
        m_.bodyRestPose.push_back(world);
        return body;
    }

    void addJoint(const JointDesc& joint, BodyId body) {
        const bool actuated = joint.actuated && joint.type != JointType::Free;
        const std::string name = unique(m_.world.joints.names, joint.name);
        m_.world.addJoint(name, joint.type, body, joint.axis, actuated);
        if (joint.type == JointType::Revolute || joint.type == JointType::Prismatic) {
            m_.world.joints.lowerLimit.back() = joint.lower;
            m_.world.joints.upperLimit.back() = joint.upper;
        }
        m_.world.joints.effortLimit.back() = joint.effort;
        m_.world.joints.velocityLimit.back() = joint.velocity;
        m_.jointPrim.push_back(kNoIndex);  // set once the child's prim exists
        if (actuated) {
            m_.actuatedJoints.push_back(name);
        }
    }

    void addLink(std::size_t l) {
        const LinkDesc& link = d_.links[l];
        std::vector<const JointDesc*> joints;
        for (const std::size_t j : jointsOf_[l]) {
            if (d_.joints[j].type != JointType::Fixed) {
                joints.push_back(&d_.joints[j]);
            }
        }
        BodyId parent = kWorldBody;
        Pose parentWorld;
        std::uint32_t parentPrim = topPrim_;
        if (!jointsOf_[l].empty() && !d_.joints[jointsOf_[l].front()].parent.empty()) {
            const std::size_t p = linkIndex_.at(d_.joints[jointsOf_[l].front()].parent);
            parent = bodyOf_[p];
            parentWorld = bodyWorld_[p];
            parentPrim = primOf_[p];
        }

        // Helper frames for all but the last joint: each turns about its own anchor, in order.
        const std::size_t firstJointPrim = m_.jointPrim.size();
        for (std::size_t k = 0; k + 1 < joints.size(); ++k) {
            const JointDesc& joint = *joints[k];
            const Pose world = compose(link.pose, joint.anchor);
            const BodyId helper =
                addBody(joint.name + "_frame", parent, parentWorld, world, kPlaceholderMassKg, {},
                        {kPlaceholderInertia, kPlaceholderInertia, kPlaceholderInertia, 0, 0, 0});
            addJoint(joint, helper);
            ScenePrim prim;
            prim.name = m_.world.bodies.names[helper.index];
            prim.parent = parentPrim;
            prim.typeName = "Xform";
            prim.body = helper.index;
            prim.apiSchemas = {"PhysicsRigidBodyAPI", "joint:" + jointTypeName(joint.type)};
            parentPrim = addPrim(std::move(prim));
            m_.bodyPrim.push_back(parentPrim);
            m_.jointPrim.back() = parentPrim;
            parent = helper;
            parentWorld = world;
        }
        if (joints.size() > 1) {
            note(m_, "link '" + link.name + "' has " + std::to_string(joints.size()) +
                         " joints; a 1 g frame was added for each joint but the last");
        }

        // The body frame is at the anchor of the link's (last) joint.
        const bool moving = !joints.empty();
        const Pose frame =
            moving && joints.back()->type != JointType::Free ? joints.back()->anchor : Pose{};
        const Pose world = compose(link.pose, frame);
        double mass = link.mass;
        std::array<double, 3> com = relative(frame, Pose{link.com, {1.0, 0.0, 0.0, 0.0}}).position;
        std::array<double, 6> inertia = inFrame(link.inertia, frame.orientation);
        if (moving && mass <= 0.0) {
            note(m_, "link '" + link.name + "' moves but has no mass; it was given 1 g");
            mass = kPlaceholderMassKg;
            com = {};
            inertia = {kPlaceholderInertia, kPlaceholderInertia, kPlaceholderInertia, 0, 0, 0};
        }
        const BodyId body = addBody(link.name, parent, parentWorld, world, mass, com, inertia);
        if (moving) {
            addJoint(*joints.back(), body);
        }
        bodyOf_[l] = body;
        bodyWorld_[l] = world;

        bool collides = false;
        for (const CollisionDesc& collision : link.collisions) {
            collides = addCollider(link, collision, body, frame, static_[l]) || collides;
        }

        ScenePrim prim;
        prim.name = m_.world.bodies.names[body.index];
        prim.parent = parentPrim;
        prim.typeName = "Xform";
        prim.body = body.index;
        if (mass > 0.0) {
            prim.apiSchemas.emplace_back("PhysicsRigidBodyAPI");
        }
        if (collides) {
            prim.apiSchemas.emplace_back("PhysicsCollisionAPI");
        }
        if (moving) {
            prim.apiSchemas.push_back("joint:" + jointTypeName(joints.back()->type));
        }
        const std::uint32_t linkPrim = addPrim(std::move(prim));
        primOf_[l] = linkPrim;
        m_.bodyPrim.push_back(linkPrim);
        for (std::size_t j = firstJointPrim; j < m_.jointPrim.size(); ++j) {
            if (m_.jointPrim[j] == kNoIndex) {
                m_.jointPrim[j] = linkPrim;
            }
        }

        std::size_t index = 0;
        for (const VisualDesc& visual : link.visuals) {
            addVisual(link, visual, index++, body, linkPrim, frame);
        }
        for (const SensorDesc& sensor : link.sensors) {
            addSensor(sensor, body, linkPrim, frame);
        }
    }

    bool addCollider(const LinkDesc& link, const CollisionDesc& collision, BodyId body,
                     const Pose& frame, bool isStatic) {
        const Geom& geom = collision.geom;
        const Pose pose = relative(frame, geom.pose);
        switch (geom.type) {
            case GeomType::Box:
                m_.world.addCollider(body, ShapeType::Box, geom.size, pose, collision.friction);
                return true;
            case GeomType::Sphere:
                m_.world.addCollider(body, ShapeType::Sphere, geom.size, pose, collision.friction);
                return true;
            case GeomType::Capsule:
                m_.world.addCollider(body, ShapeType::Capsule, geom.size, pose, collision.friction);
                return true;
            case GeomType::Cylinder:
                m_.world.addCollider(body, ShapeType::Cylinder, geom.size, pose,
                                     collision.friction);
                return true;
            case GeomType::Plane: {
                if (!isStatic) {
                    note(m_, "a plane on the moving link '" + link.name +
                                 "' is skipped: planes are ground only");
                    return false;
                }
                // The ground: on the world, as the loaders' own plane.
                const Pose ground = compose(m_.bodyRestPose[body.index], pose);
                m_.world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, ground,
                                     collision.friction);
                return false;
            }
            case GeomType::Ellipsoid:
                note(m_, "ellipsoid colliders are not supported yet; the one on link '" +
                             link.name + "' is skipped");
                return false;
            case GeomType::Mesh:
                note(m_, "collision meshes are not used yet (link '" + link.name +
                             "'): convex decomposition comes with the collision bake tool");
                return false;
        }
        return false;
    }

    std::uint32_t mesh(const std::string& key, const std::function<TriMesh()>& make) {
        const auto it = meshes_.find(key);
        if (it != meshes_.end()) {
            return it->second;
        }
        m_.scene.meshes.push_back(make());
        const auto id = static_cast<std::uint32_t>(m_.scene.meshes.size() - 1);
        meshes_.emplace(key, id);
        return id;
    }

    std::uint32_t material(const std::string& name, const std::array<float, 4>& rgba) {
        for (std::uint32_t i = 0; i < m_.scene.materials.size(); ++i) {
            if (m_.scene.materials[i].rgba == rgba) {
                return i;
            }
        }
        m_.scene.materials.push_back({name.empty() ? "default" : name, rgba});
        return static_cast<std::uint32_t>(m_.scene.materials.size() - 1);
    }

    void addVisual(const LinkDesc& link, const VisualDesc& visual, std::size_t index, BodyId body,
                   std::uint32_t linkPrim, const Pose& frame) {
        const Geom& geom = visual.geom;
        const std::array<double, 3>& s = geom.size;
        Pose pose = relative(frame, geom.pose);
        std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
        std::uint32_t meshId = kNoIndex;
        switch (geom.type) {
            case GeomType::Box:
                meshId = mesh(shapeKey("box", s),
                              [&s] { return boxMesh(2 * s[0], 2 * s[1], 2 * s[2]); });
                break;
            case GeomType::Sphere:
                meshId = mesh(shapeKey("sphere", s), [&s] { return sphereMesh(s[0]); });
                break;
            case GeomType::Capsule:
                note(m_, "capsule visuals are drawn as cylinders");
                [[fallthrough]];
            case GeomType::Cylinder:
                meshId =
                    mesh(shapeKey("cylinder", s), [&s] { return cylinderMesh(s[0], 2 * s[1]); });
                break;
            case GeomType::Ellipsoid:
                meshId = mesh("unit sphere", [] { return sphereMesh(1.0); });
                scale = {static_cast<float>(s[0]), static_cast<float>(s[1]),
                         static_cast<float>(s[2])};
                break;
            case GeomType::Plane: {
                const double x = s[0] > 0.0 ? s[0] : kGroundHalfSizeM;
                const double y = s[1] > 0.0 ? s[1] : kGroundHalfSizeM;
                meshId = mesh(shapeKey("plane", {x, y, 0.0}),
                              [x, y] { return boxMesh(2 * x, 2 * y, kPlaneThicknessM); });
                pose = compose(pose, Pose{{0.0, 0.0, -kPlaneThicknessM / 2.0}, {1, 0, 0, 0}});
                break;
            }
            case GeomType::Mesh:
                if (geom.mesh == kNoIndex || geom.mesh >= d_.meshes.size()) {
                    note(m_,
                         "visual mesh files are not loaded yet; a magenta box is shown (link '" +
                             link.name + "')");
                    addPlaceholder(index, body, linkPrim, pose);
                    return;
                }
                meshId = mesh(d_.name + " mesh " + std::to_string(geom.mesh),
                              [this, &geom] { return d_.meshes[geom.mesh]; });
                scale = {static_cast<float>(s[0]), static_cast<float>(s[1]),
                         static_cast<float>(s[2])};
                break;
        }
        ScenePrim prim;
        prim.name = index == 0 ? "visual" : "visual_" + std::to_string(index);
        prim.parent = linkPrim;
        prim.typeName = "Mesh";
        prim.mesh = meshId;
        prim.material = material(visual.material, visual.rgba);
        prim.body = body.index;
        prim.offset = pose;
        prim.scale = scale;
        addPrim(std::move(prim));
    }

    void addPlaceholder(std::size_t index, BodyId body, std::uint32_t linkPrim, const Pose& pose) {
        constexpr std::array<float, 4> kPlaceholderColor{0.85F, 0.20F, 0.55F, 1.0F};
        ScenePrim prim;
        prim.name = index == 0 ? "visual" : "visual_" + std::to_string(index);
        prim.parent = linkPrim;
        prim.typeName = "Mesh";
        prim.mesh = mesh("placeholder", [] { return boxMesh(0.1, 0.1, 0.1); });
        prim.material = material("missing-mesh", kPlaceholderColor);
        prim.body = body.index;
        prim.offset = pose;
        addPrim(std::move(prim));
    }

    void addSensor(const SensorDesc& sensor, BodyId body, std::uint32_t linkPrim,
                   const Pose& frame) {
        const Pose mount = relative(frame, sensor.pose);
        const std::string name = unique(m_.world.sensors.names, sensor.name);
        m_.world.addSensor(name, sensor.type, body, mount, sensor.rateHz, sensor.settings);
        ScenePrim prim;
        prim.name = name;
        prim.parent = linkPrim;
        switch (sensor.type) {
            case SensorType::Lidar:
                prim.typeName = "Lidar";
                prim.apiSchemas = {"SensorAPI_1", "LidarAPI_1"};
                break;
            case SensorType::Camera:
                prim.typeName = "Camera";
                prim.apiSchemas = {"SensorAPI_1", "CameraSensorAPI_1"};
                break;
            case SensorType::Imu:
                prim.typeName = "Imu";
                prim.apiSchemas = {"SensorAPI_1", "ImuAPI_1"};
                break;
        }
        prim.body = body.index;
        prim.offset = mount;
        m_.sensorPrim.push_back(addPrim(std::move(prim)));
    }

    const Description& d_;
    Model& m_;
    std::map<std::string, std::size_t, std::less<>> linkIndex_;
    std::vector<std::vector<std::size_t>> jointsOf_;
    std::vector<bool> static_;
    std::vector<BodyId> bodyOf_;
    std::vector<Pose> bodyWorld_;
    std::vector<std::uint32_t> primOf_;
    std::map<std::string, std::uint32_t> meshes_;
    std::uint32_t topPrim_{kNoIndex};
};

}  // namespace

void note(Model& model, std::string text) {
    if (std::find(model.notes.begin(), model.notes.end(), text) == model.notes.end()) {
        model.notes.push_back(std::move(text));
    }
}

bool hasGroundPlane(const Description& description) {
    const std::vector<bool> isStatic = staticLinks(description);
    for (std::size_t l = 0; l < description.links.size(); ++l) {
        for (const CollisionDesc& collision : description.links[l].collisions) {
            if (collision.geom.type == GeomType::Plane && isStatic[l]) {
                return true;
            }
        }
    }
    return false;
}

Expected<void, std::string> build(const Description& description, Model& model) {
    return Builder(description, model).run();
}

}  // namespace lagrange::importers::detail
