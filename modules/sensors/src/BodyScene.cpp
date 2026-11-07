#include "lagrange/sensors/BodyScene.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <span>

namespace lagrange::sensors {

BodyScene::BodyScene(const World& world) : world_(world), rest_(restPoses(world)) {}

const services::SceneQuery& BodyScene::at(const SignalBus& bus) {
    const std::lock_guard<std::mutex> lock(mutex_);
    const World::Bodies& bodies = world_.bodies;
    if (!resolved_) {
        poses_.resize(bodies.names.size());
        for (std::size_t b = 0; b < bodies.names.size(); ++b) {
            poses_[b] = bus.find("body/" + bodies.names[b] + "/pose");
        }
        stamps_.assign(bodies.names.size(), SimTime{-1});
        resolved_ = true;
    }
    bool moved = !placed_;
    for (std::size_t b = 0; b < poses_.size(); ++b) {
        if (poses_[b].valid() && bus.stamp(poses_[b]) != stamps_[b]) {
            stamps_[b] = bus.stamp(poses_[b]);
            moved = true;
        }
    }
    if (!moved) {
        return caster_;
    }
    const World::Colliders& colliders = world_.colliders;
    shapes_.resize(colliders.body.size());
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        ShapeCaster::Shape& shape = shapes_[c];
        shape.type = colliders.shape[c];
        shape.size = colliders.size[c];
        shape.hull = colliders.shape[c] == ShapeType::ConvexHull
                         ? &world_.hulls.at(colliders.hull[c])
                         : nullptr;
        shape.field = colliders.shape[c] == ShapeType::HeightField
                          ? &world_.heightfields.at(colliders.hull[c])
                          : nullptr;
        const BodyId owner = colliders.body[c];
        if (owner == kWorldBody) {
            shape.pose = colliders.poseInBody[c];
            continue;
        }
        Pose body = rest_[owner.index];
        if (poses_[owner.index].valid()) {
            const std::span<const double> v = bus.readN(poses_[owner.index]);
            body = {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
        }
        shape.pose = compose(body, colliders.poseInBody[c]);
    }
    caster_.update(shapes_);
    placed_ = true;
    return caster_;
}

}  // namespace lagrange::sensors
