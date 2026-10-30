// The host side of the plugin C ABI: dlopen, the host function table, and the IModule adapter.
#include "lagrange/plugin_abi/PluginLibrary.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"
#include "lagrange_plugin.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace lagrange {

namespace {

// The opaque C types are the host's own objects.
SignalDeclarer& declarer(lagrange_declarer* d) {
    return *reinterpret_cast<SignalDeclarer*>(d);
}
/// What a plugin's `lagrange_bus*` points to: the bus, and the module's services and world for
/// the scene queries (ABI 1.1). Made on the stack for each call into the plugin.
struct BusContext {
    SignalBus* bus;
    const PluginServices* services;
    const World* world;  // null until init()
};
SignalBus& bus(lagrange_bus* b) {
    return *reinterpret_cast<BusContext*>(b)->bus;
}
const SignalBus& bus(const lagrange_bus* b) {
    return *reinterpret_cast<const BusContext*>(b)->bus;
}
const BusContext& context(const lagrange_bus* b) {
    return *reinterpret_cast<const BusContext*>(b);
}
const WorldView& world(const lagrange_world* w) {
    return *reinterpret_cast<const WorldView*>(w);
}
const ModuleConfig& config(const lagrange_config* c) {
    return *reinterpret_cast<const ModuleConfig*>(c);
}
lagrange_declarer* opaque(SignalDeclarer& d) {
    return reinterpret_cast<lagrange_declarer*>(&d);
}
lagrange_bus* opaque(BusContext& b) {
    return reinterpret_cast<lagrange_bus*>(&b);
}
const lagrange_world* opaque(const WorldView& w) {
    return reinterpret_cast<const lagrange_world*>(&w);
}
const lagrange_config* opaque(const ModuleConfig& c) {
    return reinterpret_cast<const lagrange_config*>(&c);
}

SignalId id(std::uint32_t value) {
    SignalId out;
    out.index = value;
    return out;
}

/// A spec from the plugin, read only as far as the plugin's (possibly older) struct reaches.
SignalSpec toSpec(const lagrange_signal_spec* in) {
    lagrange_signal_spec c{};
    std::memcpy(&c, in, std::min<std::size_t>(in->struct_size, sizeof(c)));
    SignalSpec spec;
    spec.name = c.name != nullptr ? c.name : "";
    spec.type = static_cast<SignalType>(c.type);
    spec.unit.dim = {c.unit.length,      c.unit.mass,   c.unit.time,     c.unit.current,
                     c.unit.temperature, c.unit.amount, c.unit.luminous, c.unit.angle};
    spec.unit.scale = c.unit.scale;
    spec.unit.kind = static_cast<QuantityKind>(c.unit.kind);
    spec.arrayLength = c.array_length == 0 ? 1 : c.array_length;  // zero-filled: one value
    spec.frame = c.frame != nullptr ? c.frame : "";
    spec.hold = static_cast<Hold>(c.hold);
    spec.optional = c.optional != 0;
    spec.feedback = c.feedback != 0;
    spec.defaultValue = c.default_value;
    return spec;
}

std::uint32_t declareInput(lagrange_declarer* d, const lagrange_signal_spec* spec) {
    return spec != nullptr ? declarer(d).input(toSpec(spec)).index : LAGRANGE_INVALID_SIGNAL;
}
std::uint32_t declareOutput(lagrange_declarer* d, const lagrange_signal_spec* spec) {
    return spec != nullptr ? declarer(d).output(toSpec(spec)).index : LAGRANGE_INVALID_SIGNAL;
}
double busRead(const lagrange_bus* b, std::uint32_t signal, std::uint32_t env) {
    return bus(b).read(id(signal), env);
}
void busWrite(lagrange_bus* b, std::uint32_t signal, std::uint32_t env, double value) {
    bus(b).write(id(signal), value, env);
}
const double* busReadN(const lagrange_bus* b, std::uint32_t signal, std::uint32_t env,
                       std::uint32_t* count) {
    const std::span<const double> values = bus(b).readN(id(signal), env);
    if (count != nullptr) {
        *count = static_cast<std::uint32_t>(values.size());
    }
    return values.data();
}
void busWriteN(lagrange_bus* b, std::uint32_t signal, std::uint32_t env, const double* values,
               std::uint32_t count) {
    bus(b).writeN(id(signal), std::span<const double>(values, count), env);
}
std::uint32_t worldJointCount(const lagrange_world* w) {
    return static_cast<std::uint32_t>(world(w).joints().names.size());
}
const char* worldJointName(const lagrange_world* w, std::uint32_t joint) {
    const auto& names = world(w).joints().names;
    return joint < names.size() ? names[joint].c_str() : nullptr;
}
/// The `instance`-th application of `schema` in the world, or nullptr.
const SchemaInstance* schemaInstance(const lagrange_world* w, const char* schema,
                                     std::uint32_t instance) {
    if (schema == nullptr) {
        return nullptr;
    }
    std::uint32_t seen = 0;
    for (const SchemaInstance& s : world(w).raw().schemaInstances) {
        if (s.schema == schema && seen++ == instance) {
            return &s;
        }
    }
    return nullptr;
}
std::uint32_t worldSchemaInstanceCount(const lagrange_world* w, const char* schema) {
    if (schema == nullptr) {
        return 0;
    }
    std::uint32_t count = 0;
    for (const SchemaInstance& s : world(w).raw().schemaInstances) {
        count += s.schema == schema ? 1 : 0;
    }
    return count;
}
int worldSchemaReal(const lagrange_world* w, const char* schema, std::uint32_t instance,
                    const char* attr, double* out) {
    const SchemaInstance* s = schemaInstance(w, schema, instance);
    if (s == nullptr || attr == nullptr || out == nullptr) {
        return 0;
    }
    const std::optional<double> value = s->real(attr);
    if (!value) {
        return 0;
    }
    *out = *value;
    return 1;
}
const char* worldSchemaInstanceName(const lagrange_world* w, const char* schema,
                                    std::uint32_t instance) {
    const SchemaInstance* s = schemaInstance(w, schema, instance);
    if (s == nullptr) {
        return nullptr;
    }
    const auto& names = world(w).raw().prims.instanceNames;
    return s->prim.index < names.size() ? names[s->prim.index].c_str() : nullptr;
}
int configGetReal(const lagrange_config* c, const char* key, double* out) {
    if (key == nullptr || out == nullptr) {
        return 0;
    }
    if (const auto value = config(c).get<double>(key)) {  // integers too
        *out = *value;
        return 1;
    }
    return 0;
}
int configGetString(const lagrange_config* c, const char* key, const char** out) {
    if (key == nullptr || out == nullptr) {
        return 0;
    }
    const std::string* text = config(c).find<std::string>(key);
    if (text == nullptr) {
        return 0;
    }
    *out = text->c_str();
    return 1;
}
void log(int level, const char* message) {
    const auto clamped = static_cast<services::LogLevel>(level < 0 ? 0 : (level > 3 ? 3 : level));
    services::log(clamped, "plugin", message != nullptr ? message : "");
}

// ABI 1.1 (RFC 0008): the world's tables and the scene.

/// Fills the plugin's struct as far as both sides know it.
template <typename T>
int fill(const T& host, T* out) {
    if (out == nullptr || out->struct_size == 0) {
        return 0;
    }
    const std::size_t size = std::min<std::size_t>(out->struct_size, sizeof(T));
    const std::uint32_t theirs = out->struct_size;
    std::memcpy(out, &host, size);
    out->struct_size = theirs;
    return 1;
}
void putPose(const Pose& pose, double out[7]) {
    std::copy(pose.position.begin(), pose.position.end(), out);
    std::copy(pose.orientation.begin(), pose.orientation.end(), out + 3);
}
std::uint32_t bodyIndex(BodyId body) {
    return body == kWorldBody ? LAGRANGE_WORLD_BODY : body.index;
}
/// Rest poses of the world's bodies, computed once per world for the getters.
const std::vector<Pose>& restPosesOf(const World& w) {
    thread_local const World* cached = nullptr;
    thread_local std::size_t cachedCount = 0;
    thread_local std::vector<Pose> poses;
    if (cached != &w || cachedCount != w.bodies.names.size()) {
        poses = restPoses(w);
        cached = &w;
        cachedCount = w.bodies.names.size();
    }
    return poses;
}
std::uint32_t worldBodyCount(const lagrange_world* w) {
    return static_cast<std::uint32_t>(world(w).raw().bodies.names.size());
}
int worldBody(const lagrange_world* w, std::uint32_t b, lagrange_body_info* out) {
    const World& raw = world(w).raw();
    const World::Bodies& bodies = raw.bodies;
    if (b >= bodies.names.size()) {
        return 0;
    }
    lagrange_body_info info{};
    info.struct_size = sizeof info;
    info.name = bodies.names[b].c_str();
    info.parent = bodyIndex(bodies.parent[b]);
    info.robot = b < bodies.robot.size() ? bodies.robot[b] : 0;
    info.mass = bodies.mass[b];
    std::copy(bodies.comLocal[b].begin(), bodies.comLocal[b].end(), info.com);
    std::copy(bodies.inertiaLocal[b].begin(), bodies.inertiaLocal[b].end(), info.inertia);
    putPose(bodies.poseInParent[b], info.pose_in_parent);
    putPose(restPosesOf(raw)[b], info.rest_pose);
    info.kinematic = b < bodies.kinematic.size() && bodies.kinematic[b] ? 1 : 0;
    return fill(info, out);
}
int worldJoint(const lagrange_world* w, std::uint32_t j, lagrange_joint_info* out) {
    const World::Joints& joints = world(w).raw().joints;
    if (j >= joints.names.size()) {
        return 0;
    }
    lagrange_joint_info info{};
    info.struct_size = sizeof info;
    info.name = joints.names[j].c_str();
    info.type = static_cast<lagrange_joint_type>(joints.type[j]);
    info.parent = bodyIndex(joints.parent[j]);
    info.child = bodyIndex(joints.child[j]);
    std::copy(joints.axis[j].begin(), joints.axis[j].end(), info.axis);
    info.lower = joints.lowerLimit[j];
    info.upper = joints.upperLimit[j];
    info.effort_limit = joints.effortLimit[j];
    info.velocity_limit = joints.velocityLimit[j];
    info.armature = joints.armature[j];
    info.initial_position = j < joints.initialPosition.size() ? joints.initialPosition[j] : 0.0;
    info.actuated = joints.actuated[j] ? 1 : 0;
    return fill(info, out);
}
std::uint32_t worldColliderCount(const lagrange_world* w) {
    return static_cast<std::uint32_t>(world(w).raw().colliders.body.size());
}
int worldCollider(const lagrange_world* w, std::uint32_t c, lagrange_collider_info* out) {
    const World& raw = world(w).raw();
    const World::Colliders& colliders = raw.colliders;
    if (c >= colliders.body.size()) {
        return 0;
    }
    lagrange_collider_info info{};
    info.struct_size = sizeof info;
    info.body = bodyIndex(colliders.body[c]);
    info.shape = static_cast<lagrange_shape_type>(colliders.shape[c]);
    std::copy(colliders.size[c].begin(), colliders.size[c].end(), info.size);
    putPose(colliders.poseInBody[c], info.pose);
    info.friction = colliders.friction[c];
    if (colliders.shape[c] == ShapeType::ConvexHull && colliders.hull[c] < raw.hulls.size()) {
        const ConvexHull& hull = raw.hulls[colliders.hull[c]];
        info.point_count = static_cast<std::uint32_t>(hull.points.size());
        info.points = hull.points.empty() ? nullptr : hull.points.front().data();
        info.triangle_count = static_cast<std::uint32_t>(hull.triangles.size());
        info.triangles = hull.triangles.empty() ? nullptr : hull.triangles.front().data();
    }
    if (colliders.shape[c] == ShapeType::HeightField &&
        colliders.hull[c] < raw.heightfields.size()) {
        const HeightField& field = raw.heightfields[colliders.hull[c]];
        info.columns = field.columns;
        info.rows = field.rows;
        info.cell[0] = field.cellX;
        info.cell[1] = field.cellY;
        info.heights = field.heights.empty() ? nullptr : field.heights.data();
    }
    return fill(info, out);
}
std::uint32_t worldSensorCount(const lagrange_world* w) {
    return static_cast<std::uint32_t>(world(w).raw().sensors.names.size());
}
const char* sensorKind(const World::Sensors& sensors, std::size_t s) {
    static constexpr std::array<const char*, 7> kKinds{
        "imu", "lidar", "camera", "gnss", "magnetometer", "barometer", "radar"};
    const auto type = static_cast<std::size_t>(sensors.type[s]);
    return type < kKinds.size() ? kKinds.at(type) : "unknown";
}
int worldSensor(const lagrange_world* w, std::uint32_t s, lagrange_sensor_info* out) {
    const World::Sensors& sensors = world(w).raw().sensors;
    if (s >= sensors.names.size()) {
        return 0;
    }
    lagrange_sensor_info info{};
    info.struct_size = sizeof info;
    info.name = sensors.names[s].c_str();
    info.kind = sensorKind(sensors, s);
    info.body = bodyIndex(sensors.body[s]);
    putPose(sensors.poseInBody[s], info.pose_in_body);
    info.rate = sensors.rateHz[s];
    return fill(info, out);
}
int worldSensorParamReal(const lagrange_world* w, std::uint32_t s, const char* key, double* out) {
    const World::Sensors& sensors = world(w).raw().sensors;
    if (s >= sensors.names.size() || key == nullptr || out == nullptr) {
        return 0;
    }
    const auto it = sensors.settings[s].find(key);
    if (it == sensors.settings[s].end()) {
        return 0;
    }
    *out = it->second;
    return 1;
}
int worldSensorParamString(const lagrange_world* w, std::uint32_t s, const char* key,
                           const char** out) {
    (void)w;
    (void)s;
    (void)key;
    (void)out;
    return 0;  // text parameters come with custom sensors (RFC 0008 §3)
}
int worldSite(const lagrange_world* w, lagrange_site_info* out) {
    const World& raw = world(w).raw();
    lagrange_site_info info{};
    info.struct_size = sizeof info;
    std::copy(raw.gravityLocal.begin(), raw.gravityLocal.end(), info.gravity);
    std::copy(raw.region.worldEnu.begin(), raw.region.worldEnu.end(), info.region_origin);
    return fill(info, out);
}

/// The scene for this call, or null when the host gave none.
const services::SceneQuery* sceneOf(const lagrange_bus* b) {
    const BusContext& c = context(b);
    if (c.services == nullptr || !c.services->scene) {
        return nullptr;
    }
    return c.services->scene(*c.bus);
}
void putHit(const BusContext& c, const services::RayHit& hit, lagrange_ray_hit* out) {
    lagrange_ray_hit info{};
    info.struct_size = sizeof info;
    info.distance = hit.distance;
    info.collider = hit.hit() ? hit.shape : LAGRANGE_INVALID_SIGNAL;
    info.body = LAGRANGE_WORLD_BODY;
    if (hit.hit() && c.world != nullptr && hit.shape < c.world->colliders.body.size()) {
        info.body = bodyIndex(c.world->colliders.body[hit.shape]);
    }
    (void)fill(info, out);
}
int sceneRay(const lagrange_bus* b, const double origin[3], const double dir[3], double maxDistance,
             lagrange_ray_hit* out) {
    const services::SceneQuery* scene = origin != nullptr && dir != nullptr ? sceneOf(b) : nullptr;
    const services::RayHit hit =
        scene != nullptr
            ? scene->ray({origin[0], origin[1], origin[2]}, {dir[0], dir[1], dir[2]}, maxDistance)
            : services::RayHit{};
    putHit(context(b), hit, out);
    return hit.hit() ? 1 : 0;
}
std::uint32_t sceneRays(const lagrange_bus* b, std::uint32_t count, const double* origins,
                        const double* dirs, double maxDistance, lagrange_ray_hit* out) {
    if (origins == nullptr || dirs == nullptr || out == nullptr) {
        return 0;
    }
    const services::SceneQuery* scene = sceneOf(b);
    const std::size_t stride = std::max<std::size_t>(out[0].struct_size, 1);
    auto* bytes = reinterpret_cast<std::uint8_t*>(out);
    std::uint32_t hits = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        const double* o = origins + 3 * static_cast<std::size_t>(i);
        const double* d = dirs + 3 * static_cast<std::size_t>(i);
        const services::RayHit hit =
            scene != nullptr ? scene->ray({o[0], o[1], o[2]}, {d[0], d[1], d[2]}, maxDistance)
                             : services::RayHit{};
        auto* slot = reinterpret_cast<lagrange_ray_hit*>(bytes + i * stride);
        slot->struct_size = static_cast<std::uint32_t>(stride);
        putHit(context(b), hit, slot);
        hits += hit.hit() ? 1 : 0;
    }
    return hits;
}
std::uint32_t sceneOverlapSphere(const lagrange_bus* b, const double center[3], double radius,
                                 std::uint32_t* colliders, std::uint32_t capacity) {
    const services::SceneQuery* scene = center != nullptr ? sceneOf(b) : nullptr;
    if (scene == nullptr) {
        return 0;
    }
    const std::vector<std::uint32_t> found =
        scene->overlapSphere({center[0], center[1], center[2]}, radius);
    if (colliders != nullptr) {
        std::copy_n(found.begin(), std::min<std::size_t>(found.size(), capacity), colliders);
    }
    return static_cast<std::uint32_t>(found.size());
}

constexpr lagrange_host_api kHostApi{
    sizeof(lagrange_host_api),
    LAGRANGE_PLUGIN_ABI_MAJOR,
    LAGRANGE_PLUGIN_ABI_MINOR,
    declareInput,
    declareOutput,
    busRead,
    busWrite,
    busReadN,
    busWriteN,
    worldJointCount,
    worldJointName,
    worldSchemaInstanceCount,
    worldSchemaReal,
    worldSchemaInstanceName,
    configGetReal,
    configGetString,
    log,
    worldBodyCount,
    worldBody,
    worldJoint,
    worldColliderCount,
    worldCollider,
    worldSensorCount,
    worldSensor,
    worldSensorParamReal,
    worldSensorParamString,
    worldSite,
    sceneRay,
    sceneRays,
    sceneOverlapSphere,
};

}  // namespace

struct PluginLibrary::Handle {
    void* library{nullptr};
    std::string path;
    std::vector<const lagrange_module_api*> modules;

    Handle() = default;
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    Handle(Handle&&) = delete;
    Handle& operator=(Handle&&) = delete;
    ~Handle() {
        if (library != nullptr) {
            dlclose(library);
        }
    }
};

namespace {

/// One module of a plugin, as the scheduler sees it.
class PluginModule final : public IModule {
public:
    PluginModule(std::shared_ptr<PluginLibrary::Handle> library, const lagrange_module_api* api,
                 void* self, PluginServices services)
        : library_(std::move(library)), api_(api), self_(self), services_(std::move(services)) {}
    PluginModule(const PluginModule&) = delete;
    PluginModule& operator=(const PluginModule&) = delete;
    PluginModule(PluginModule&&) = delete;
    PluginModule& operator=(PluginModule&&) = delete;
    ~PluginModule() override { api_->destroy(self_); }

    [[nodiscard]] ModuleInfo info() const override {
        return {api_->domain,
                api_->name,
                api_->version_major,
                api_->version_minor,
                api_->is_substitute != 0,
                api_->deterministic != 0,
                static_cast<CapabilityTier>(api_->minimum_tier)};
    }
    [[nodiscard]] Duration preferredStep() const override {
        return Duration{api_->preferred_step_ns};
    }
    Status declare(const WorldView& world, const ModuleConfig& config,
                   SignalDeclarer& declarer) override {
        return status(api_->declare(self_, opaque(world), opaque(config), opaque(declarer)));
    }
    Status init(const WorldView& world, const ModuleConfig& config, SignalBus& bus) override {
        world_ = &world.raw();
        BusContext call{&bus, &services_, world_};
        return status(api_->init(self_, opaque(world), opaque(config), opaque(call)));
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        const lagrange_step_context c{sizeof(lagrange_step_context), context.t.ns, context.dt.ns,
                                      context.stepIndex, context.seed};
        BusContext call{&bus, &services_, world_};
        return status(api_->step(self_, &c, opaque(call)));
    }
    void shutdown() override {
        if (api_->shutdown != nullptr) {
            api_->shutdown(self_);
        }
    }
    Status saveState(StateWriter& writer) const override {
        if (api_->save_state == nullptr) {
            return IModule::saveState(writer);
        }
        std::size_t size = 0;
        if (const lagrange_status s = api_->save_state(self_, nullptr, 0, &size);
            s != LAGRANGE_OK) {
            return status(s);
        }
        std::vector<std::uint8_t> bytes(size);
        std::size_t written = 0;
        if (const lagrange_status s = api_->save_state(self_, bytes.data(), size, &written);
            s != LAGRANGE_OK) {
            return status(s);
        }
        writer.put<std::uint64_t>(written);
        writer.putBytes(bytes.data(), written);
        return {};
    }
    Status restoreState(StateReader& reader) override {
        if (api_->restore_state == nullptr) {
            return IModule::restoreState(reader);
        }
        std::uint64_t size = 0;
        if (!reader.get(size) || size > reader.remaining()) {
            return Status::error(StatusCode::Internal, "short plugin state");
        }
        std::vector<std::uint8_t> bytes(size);
        reader.getBytes(bytes.data(), bytes.size());
        return status(api_->restore_state(self_, bytes.data(), bytes.size()));
    }
    void shiftOrigin(const std::array<double, 3>& shift) override {
        // Appended in ABI 1.0: only when the plugin's table is long enough to have it.
        if (api_->struct_size >= offsetof(lagrange_module_api, shift_origin) + sizeof(void*) &&
            api_->shift_origin != nullptr) {
            api_->shift_origin(self_, shift.data());
        }
    }

private:
    [[nodiscard]] Status status(lagrange_status s) const {
        if (s == LAGRANGE_OK) {
            return {};
        }
        const char* reason = api_->last_error != nullptr ? api_->last_error(self_) : nullptr;
        std::string message = std::string(api_->domain) + "/" + api_->name + ": ";
        message += reason != nullptr ? reason : "failed";
        const auto code =
            s <= LAGRANGE_INTERNAL ? static_cast<StatusCode>(s) : StatusCode::Internal;
        return Status::error(code, message);
    }

    std::shared_ptr<PluginLibrary::Handle> library_;
    const lagrange_module_api* api_;
    void* self_;
    PluginServices services_;
    const World* world_{nullptr};
};

}  // namespace

Expected<PluginLibrary, std::string> PluginLibrary::open(const std::string& path) {
    auto handle = std::make_shared<Handle>();
    handle->path = path;
    // RTLD_LOCAL: two plugins may define the same symbols without clashing.
    handle->library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle->library == nullptr) {
        const char* reason = dlerror();
        return Unexpected{"cannot load plugin " + path + ": " + (reason != nullptr ? reason : "")};
    }
    using GetModule = const lagrange_module_api* (*)(std::uint32_t, std::uint32_t, std::uint32_t);
    void* symbol = dlsym(handle->library, "lagrange_plugin_get_module");
    if (symbol == nullptr) {
        return Unexpected{path + " is not a Lagrange plugin: no lagrange_plugin_get_module"};
    }
    const auto get = reinterpret_cast<GetModule>(symbol);
    for (std::uint32_t index = 0;; ++index) {
        const lagrange_module_api* api =
            get(LAGRANGE_PLUGIN_ABI_MAJOR, LAGRANGE_PLUGIN_ABI_MINOR, index);
        if (api == nullptr) {
            break;
        }
        if (api->struct_size < offsetof(lagrange_module_api, last_error) ||
            api->create == nullptr || api->destroy == nullptr || api->declare == nullptr ||
            api->init == nullptr || api->step == nullptr || api->domain == nullptr ||
            api->name == nullptr) {
            return Unexpected{path + ": module " + std::to_string(index) +
                              " is incomplete (a required function or name is missing)"};
        }
        handle->modules.push_back(api);
    }
    if (handle->modules.empty()) {
        return Unexpected{path + " offers no module for plugin ABI " +
                          std::to_string(LAGRANGE_PLUGIN_ABI_MAJOR) + ".x"};
    }
    PluginLibrary out;
    out.handle_ = std::move(handle);
    return out;
}

std::size_t PluginLibrary::size() const {
    return handle_->modules.size();
}

std::string PluginLibrary::label(std::size_t index) const {
    const lagrange_module_api* api = handle_->modules.at(index);
    return std::string(api->domain) + "/" + api->name;
}

std::unique_ptr<IModule> PluginLibrary::create(std::size_t index, PluginServices services) const {
    const lagrange_module_api* api = handle_->modules.at(index);
    void* self = api->create(&kHostApi);
    if (self == nullptr) {
        return nullptr;
    }
    return std::make_unique<PluginModule>(handle_, api, self, std::move(services));
}

}  // namespace lagrange
