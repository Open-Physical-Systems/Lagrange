// lagrange_jolt: the Jolt rigid-body backend as a plugin through the C ABI (RFC 0008 §5), the
// proof that a whole physics engine fits through it. Run with
//
//     lagrange_server --plugin lagrange_jolt --physics jolt_abi
//
// It reads the world with the ABI 1.1 getters (lagrange/plugin_sdk/World.hpp), rebuilds the
// world tables from them, and runs the same engine code as the built-in Jolt backend, whose
// signals go through the ABI's declare and bus functions instead of the scheduler's.
#include "lagrange/module/IModule.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/plugin_sdk/World.hpp"
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange_plugin.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace {

using lagrange::BodyId;
using lagrange::Pose;
using lagrange::SignalId;
using lagrange::SignalSpec;
using lagrange::rigid_body::JoltBackend;

Pose poseOf(const std::array<double, 7>& p) {
    return {{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
}

BodyId bodyOf(std::uint32_t index) {
    return index == LAGRANGE_WORLD_BODY ? lagrange::kWorldBody : BodyId{index};
}

/// The world tables the backend needs, from what the host reports.
lagrange::World worldOf(const lagrange::plugin_sdk::World& in) {
    lagrange::World out;
    for (const auto& body : in.bodies) {
        out.addBody(body.name, bodyOf(body.parent), poseOf(body.poseInParent), body.mass, body.com,
                    body.inertia);
        out.bodies.robot.back() = body.robot;
        out.bodies.kinematic.back() = body.kinematic;
    }
    for (const auto& joint : in.joints) {
        out.addJoint(joint.name, static_cast<lagrange::JointType>(joint.type), bodyOf(joint.child),
                     joint.axis, joint.actuated);
        out.joints.lowerLimit.back() = joint.lower;
        out.joints.upperLimit.back() = joint.upper;
        out.joints.effortLimit.back() = joint.effortLimit;
        out.joints.velocityLimit.back() = joint.velocityLimit;
        out.joints.armature.back() = joint.armature;
        out.joints.initialPosition.back() = joint.initialPosition;
        out.joints.damping.back() = joint.damping;
        out.joints.friction.back() = joint.friction;
        out.joints.mimic.back() = joint.mimic;
        out.joints.mimicMultiplier.back() = joint.mimicMultiplier;
        out.joints.mimicOffset.back() = joint.mimicOffset;
    }
    for (const auto& collider : in.colliders) {
        const auto shape = static_cast<lagrange::ShapeType>(collider.shape);
        if (shape == lagrange::ShapeType::ConvexHull) {
            out.addHullCollider(bodyOf(collider.body),
                                lagrange::ConvexHull{collider.points, collider.triangles, {}},
                                poseOf(collider.pose), collider.friction);
        } else if (shape == lagrange::ShapeType::HeightField) {
            lagrange::HeightField field;
            field.columns = collider.columns;
            field.rows = collider.rows;
            field.cellX = collider.cell[0];
            field.cellY = collider.cell[1];
            field.heights = collider.heights;
            out.addHeightFieldCollider(std::move(field), poseOf(collider.pose), collider.friction);
        } else {
            out.addCollider(bodyOf(collider.body), shape, collider.size, poseOf(collider.pose),
                            collider.friction);
        }
    }
    out.gravityLocal = in.gravity;
    out.region.worldEnu = in.regionOrigin;
    return out;
}

/// The engine's signal access over the C ABI: declare_input/declare_output in declare(), the bus
/// functions in init() and step(). Ids are the host's, carried in SignalId::index.
class AbiIo final : public JoltBackend::SignalIo {
public:
    AbiIo(const lagrange_host_api& host, lagrange_declarer* declarer, lagrange_bus* bus)
        : host_(host), declarer_(declarer), bus_(bus) {}

    SignalId input(SignalSpec spec) override {
        return id(host_.declare_input(declarer_, &(converted_ = convert(spec))));
    }
    SignalId output(SignalSpec spec) override {
        return id(host_.declare_output(declarer_, &(converted_ = convert(spec))));
    }
    [[nodiscard]] double read(SignalId id) const override {
        return host_.bus_read(bus_, id.index, 0);
    }
    [[nodiscard]] std::span<const double> readN(SignalId id) const override {
        std::uint32_t count = 0;
        const double* values = host_.bus_read_n(bus_, id.index, 0, &count);
        return {values, count};
    }
    void write(SignalId id, double value) override { host_.bus_write(bus_, id.index, 0, value); }
    void writeN(SignalId id, std::span<const double> values) override {
        host_.bus_write_n(bus_, id.index, 0, values.data(),
                          static_cast<std::uint32_t>(values.size()));
    }

private:
    static SignalId id(std::uint32_t value) {
        SignalId out;
        if (value != LAGRANGE_INVALID_SIGNAL) {
            out.index = value;
        }
        return out;
    }
    lagrange_signal_spec convert(const SignalSpec& spec) {
        name_ = spec.name;
        frame_ = spec.frame;
        lagrange_signal_spec out{};
        out.struct_size = sizeof out;
        out.name = name_.c_str();
        out.type = static_cast<lagrange_signal_type>(spec.type);
        out.unit = {spec.unit.dim.length,      spec.unit.dim.mass,
                    spec.unit.dim.time,        spec.unit.dim.current,
                    spec.unit.dim.temperature, spec.unit.dim.amount,
                    spec.unit.dim.luminous,    spec.unit.dim.angle,
                    spec.unit.scale,           static_cast<std::uint16_t>(spec.unit.kind)};
        out.array_length = spec.arrayLength;
        out.frame = frame_.empty() ? nullptr : frame_.c_str();
        out.hold = static_cast<std::uint8_t>(spec.hold);
        out.optional = spec.optional ? 1 : 0;
        out.feedback = spec.feedback ? 1 : 0;
        out.default_value = spec.defaultValue;
        return out;
    }

    const lagrange_host_api& host_;
    lagrange_declarer* declarer_;
    lagrange_bus* bus_;
    std::string name_, frame_;
    lagrange_signal_spec converted_{};
};

struct Plugin {
    const lagrange_host_api* host{nullptr};
    lagrange::World world;
    JoltBackend backend;
    std::string error;
};

lagrange_status result(Plugin& self, const lagrange::Status& status) {
    if (status.ok()) {
        return LAGRANGE_OK;
    }
    self.error = status.message;
    return static_cast<lagrange_status>(status.code);
}

void* create(const lagrange_host_api* host) {
    auto* self = new (std::nothrow) Plugin;
    if (self != nullptr) {
        self->host = host;
    }
    return self;
}

void destroy(void* self) {
    delete static_cast<Plugin*>(self);
}

lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config*,
                        lagrange_declarer* declarer) {
    auto& self = *static_cast<Plugin*>(p);
    const auto read = lagrange::plugin_sdk::World::read(*self.host, world);
    if (!read) {
        self.error = "lagrange_jolt needs a host with plugin ABI 1.1";
        return LAGRANGE_UNSUPPORTED;
    }
    self.world = worldOf(*read);
    AbiIo io(*self.host, declarer, nullptr);
    return result(self,
                  self.backend.declare(lagrange::WorldView(self.world),
                                       lagrange::ModuleConfig(lagrange::ModuleConfigData{}), io));
}

lagrange_status init(void* p, const lagrange_world*, const lagrange_config* config,
                     lagrange_bus* bus) {
    auto& self = *static_cast<Plugin*>(p);
    AbiIo io(*self.host, nullptr, bus);
    // domains.rigid_body.substeps, as the built-in backend takes it (a whole count only).
    lagrange::ModuleConfigData data;
    if (double substeps = 0.0; self.host->config_get_real(config, "substeps", &substeps) == 1) {
        data.values["substeps"] = static_cast<std::int64_t>(std::llround(substeps));
    }
    return result(self, self.backend.init(lagrange::WorldView(self.world),
                                          lagrange::ModuleConfig(std::move(data)), io));
}

lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    auto& self = *static_cast<Plugin*>(p);
    AbiIo io(*self.host, nullptr, bus);
    lagrange::StepContext c;
    c.t = lagrange::SimTime{context->t_ns};
    c.dt = lagrange::Duration{context->dt_ns};
    c.stepIndex = context->step_index;
    c.seed = context->seed;
    return result(self, self.backend.step(c, io));
}

void shutdown(void* p) {
    static_cast<Plugin*>(p)->backend.shutdown();
}

lagrange_status saveState(const void* p, std::uint8_t* buffer, std::size_t capacity,
                          std::size_t* written) {
    const auto& self = *static_cast<const Plugin*>(p);
    lagrange::StateWriter writer;
    if (!self.backend.saveState(writer).ok()) {
        return LAGRANGE_INTERNAL;
    }
    const std::span<const std::byte> bytes = writer.bytes();
    *written = bytes.size();
    if (buffer == nullptr) {
        return LAGRANGE_OK;  // size query
    }
    if (capacity < bytes.size()) {
        return LAGRANGE_INTERNAL;
    }
    std::memcpy(buffer, bytes.data(), bytes.size());
    return LAGRANGE_OK;
}

lagrange_status restoreState(void* p, const std::uint8_t* buffer, std::size_t size) {
    auto& self = *static_cast<Plugin*>(p);
    lagrange::StateReader reader(
        std::span<const std::byte>(reinterpret_cast<const std::byte*>(buffer), size));
    return result(self, self.backend.restoreState(reader));
}

const char* lastError(const void* p) {
    const auto& self = *static_cast<const Plugin*>(p);
    return self.error.empty() ? nullptr : self.error.c_str();
}

void shiftOrigin(void* p, const double shift[3]) {
    static_cast<Plugin*>(p)->backend.shiftOrigin({shift[0], shift[1], shift[2]});
}

const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "rigid_body",
    "jolt_abi",
    1,
    0,
    0,          // a backend, not a substitute
    1,          // deterministic: Jolt cross-platform deterministic, one thread
    0,          // Tier 0
    1'000'000,  // 1 ms
    create,
    destroy,
    declare,
    init,
    step,
    shutdown,
    saveState,
    restoreState,
    lastError,
    shiftOrigin,
};

}  // namespace

extern "C" LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(
    std::uint32_t major, std::uint32_t, std::uint32_t index) {
    return major == LAGRANGE_PLUGIN_ABI_MAJOR && index == 0 ? &kModule : nullptr;
}
