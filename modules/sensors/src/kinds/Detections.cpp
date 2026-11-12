// detections (sensors.md §9): ground-truth annotations for machine-learning datasets, from a
// camera's instance image (sensor/<camera>/instance, the scene prim index per pixel) and the
// scene: per visible object its 2D box, visible pixels, occlusion and truncation, class, and 3D
// box in the camera's optical frame (KITTI's convention, Geiger et al. 2012); and a semantic
// segmentation image, the class id per pixel.
#include "Places.hpp"
#include "lagrange/world/FisheyeLens.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <tuple>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDefaultMaxObjects = 64.0;
constexpr double kDefaultMinPixels = 1.0;
constexpr std::size_t kFields = 16;  // per detection, see sensors.md §9 "detections"

/// The classes when the sensor gives none (parameter `semantic`): words of a prim's or body's
/// name, and the class id they give. 0 is unlabelled; 9 is a moving body nothing else names.
constexpr const char* kDefaultClasses =
    "ground,floor,terrain=1;road,lane,sidewalk,crossing,marking=2;"
    "building,wall,house,shed,roof=3;car,sedan,vehicle,truck,bus,van=4;"
    "human,person,pedestrian,walker,actor=5;tree,bush,hedge,plant,vegetation=6;"
    "pole,sign,light,signal=7;box,crate,pallet,rack,shelf,barrel,cone=8";
constexpr double kRobotClass = 9.0;

struct ClassRule {
    std::vector<std::string> words;
    double id{0.0};
};

/// "w1,w2=ID;w3=ID": empty when malformed (the caller refuses).
std::vector<ClassRule> parseClasses(std::string_view text, std::string& error) {
    std::vector<ClassRule> out;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t end = std::min(text.find(';', start), text.size());
        const std::string_view entry = text.substr(start, end - start);
        start = end + 1;
        if (entry.find_first_not_of(' ') == std::string_view::npos) {
            if (end == text.size()) {
                break;
            }
            continue;
        }
        const std::size_t eq = entry.find('=');
        if (eq == std::string_view::npos) {
            error = "'" + std::string(entry) + "' has no '=ID'";
            return {};
        }
        ClassRule rule;
        try {
            rule.id = std::stod(std::string(entry.substr(eq + 1)));
        } catch (const std::exception&) {
            error = "'" + std::string(entry) + "' has no number after '='";
            return {};
        }
        // Alternatives separated by ',' or '|'; each a whole word of a name (words()).
        std::string word;
        for (const char c : std::string(entry.substr(0, eq)) + ",") {
            if (c == ',' || c == '|') {
                if (!word.empty()) {
                    rule.words.push_back(lowerCase(word));
                }
                word.clear();
            } else if (c != ' ') {
                word.push_back(c);
            }
        }
        out.push_back(std::move(rule));
        if (end == text.size()) {
            break;
        }
    }
    return out;
}

/// The class of a name: the first rule with one of its words, else 0.
double classOf(const std::vector<ClassRule>& rules, std::string_view name) {
    const std::vector<std::string> parts = words(name);
    for (const ClassRule& rule : rules) {
        for (const std::string& w : rule.words) {
            if (std::ranges::find(parts, w) != parts.end()) {
                return rule.id;
            }
        }
    }
    return 0.0;
}

struct Part {  // one drawable of the camera scene
    std::uint32_t prim{0};
    std::uint32_t mesh{0};
    std::uint32_t body{kNoIndex};
    Pose offset;
    std::array<float, 3> scale{1, 1, 1};
    bool hasBox{false};
    Vec3 lo{}, hi{};  // its mesh's box in its own frame, scaled
};

struct Object {
    std::vector<std::uint32_t> parts;   // into Detections::parts_
    std::uint32_t frameBody{kNoIndex};  // the body whose frame its 3D box is in; kNoIndex: static
    Pose frameOffset;                   // the static part's pose (frameBody kNoIndex)
    double instance{0.0};               // the lowest scene prim index of its parts
    double cls{0.0};
};

class Detections final : public KindModule {
public:
    struct Camera {
        std::string name;
        std::uint32_t body{0};
        Pose mount;
        std::uint32_t width{0}, height{0};
        double fx{1.0}, cx{0.0}, cy{0.0}, near{0.05};
        double rateHz{30.0};
        bool pinhole{true};  // false: a fisheye (FisheyeLens), no pinhole projection
    };

    Detections(const World& world, std::uint32_t sensor, bool groundTruth, Camera camera,
               std::vector<Part> parts, std::vector<Object> objects,
               std::shared_ptr<gpu::RasterScene> raster)
        : KindModule(world, sensor, groundTruth)
        , camera_(std::move(camera))
        , parts_(std::move(parts))
        , objects_(std::move(objects))
        , raster_(std::move(raster)) {
        for (std::uint32_t o = 0; o < objects_.size(); ++o) {
            for (const std::uint32_t k : objects_[o].parts) {
                const std::uint32_t prim = parts_[k].prim;
                if (prim >= objectOfPrim_.size()) {
                    objectOfPrim_.resize(prim + 1, kNoIndex);
                }
                objectOfPrim_[prim] = o;
            }
        }
        // In step with the camera it reads: the camera's period in whole milliseconds.
        const auto ms =
            std::max<std::int64_t>(std::llround(1e3 / std::max(camera_.rateHz, 1e-3)), 1);
        step_ = Duration{ms * 1'000'000};
    }

    [[nodiscard]] ModuleInfo info() const override {
        ModuleInfo out = KindModule::info();
        out.deterministic = groundTruth_ || !raster_;  // the occlusion renders are a GPU path
        return out;
    }

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        maxObjects_ = static_cast<std::uint32_t>(std::clamp(
            kindSetting(world, sensor_, "max_objects", kDefaultMaxObjects), 1.0, 4096.0));
        minPixels_ = kindSetting(world, sensor_, "min_pixels", kDefaultMinPixels);
        // The occlusion renders draw through a pinhole: none for a fisheye camera.
        occlusion_ = kindSetting(world, sensor_, "occlusion", 1.0) != 0.0 && camera_.pinhole;
        instance_ = d.input(
            kindSpec("sensor/" + camera_.name + "/instance", SignalType::Image, Unit{}, "optical"));
        poses_.begin(world);
        poses_.watch(camera_.body, d);
        for (const Part& p : parts_) {
            poses_.watch(p.body, d);
        }
        SignalSpec out = kindSpec(base() + "/detections", SignalType::ScalarArray, units::one);
        out.arrayLength = static_cast<std::uint32_t>(maxObjects_ * kFields);
        detections_ = d.output(std::move(out));
        count_ = d.output(kindSpec(base() + "/count", SignalType::Int, units::one));
        semantic_ = d.output(kindSpec(base() + "/semantic", SignalType::Image, Unit{}, "optical"));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (raster_ && occlusion_ && !groundTruth_) {
            render_ = std::make_unique<gpu::RasterCamera>(raster_, camera_.width, camera_.height);
        }
        publish(bus, {}, nullptr);
        return {};
    }

    Status step(const StepContext&, SignalBus& bus) override {
        const std::shared_ptr<const Buffer> image = bus.buffer(bus.readBuffer(instance_));
        if (!image || image->encoding != "r32u" ||
            image->bytes.size() != std::size_t{image->width} * image->height * 4) {
            publish(bus, {}, nullptr);
            return {};
        }
        const std::uint32_t w = image->width;
        const std::uint32_t h = image->height;
        // Per object: visible pixels and their box.
        struct Seen {
            double pixels{0.0};
            std::uint32_t u0{~0U}, v0{~0U}, u1{0}, v1{0};
        };
        std::vector<Seen> seen(objects_.size());
        auto semantic = std::make_shared<Buffer>();
        semantic->width = w;
        semantic->height = h;
        semantic->encoding = "r32u";
        semantic->bytes.assign(image->bytes.size(), std::byte{0});
        const std::byte* in = image->bytes.data();
        std::byte* classes = semantic->bytes.data();
        for (std::uint32_t v = 0; v < h; ++v) {
            for (std::uint32_t u = 0; u < w; ++u) {
                const std::size_t i = std::size_t{v} * w + u;
                std::uint32_t id = 0;
                std::memcpy(&id, in + i * 4, 4);
                if (id >= objectOfPrim_.size() || objectOfPrim_[id] == kNoIndex) {
                    continue;
                }
                const std::uint32_t o = objectOfPrim_[id];
                Seen& s = seen[o];
                s.pixels += 1.0;
                s.u0 = std::min(s.u0, u);
                s.v0 = std::min(s.v0, v);
                s.u1 = std::max(s.u1, u);
                s.v1 = std::max(s.v1, v);
                const auto cls = static_cast<std::uint32_t>(std::max(objects_[o].cls, 0.0));
                std::memcpy(classes + i * 4, &cls, 4);
            }
        }
        std::vector<std::uint32_t> order;
        for (std::uint32_t o = 0; o < objects_.size(); ++o) {
            if (seen[o].pixels > 0.0 && seen[o].pixels >= minPixels_) {
                order.push_back(o);
            }
        }
        std::ranges::sort(order, [&](std::uint32_t a, std::uint32_t b) {
            return std::tie(seen[b].pixels, objects_[a].instance) <
                   std::tie(seen[a].pixels, objects_[b].instance);
        });
        if (order.size() > maxObjects_) {
            order.resize(maxObjects_);
        }
        const Pose sensor = compose(poses_.pose(bus, camera_.body), camera_.mount);
        const Pose optical{sensor.position, multiply(sensor.orientation, kSensorFromOptical)};
        std::vector<double> out;
        out.reserve(order.size() * kFields);
        for (const std::uint32_t o : order) {
            const Seen& s = seen[o];
            const Object& object = objects_[o];
            std::array<double, kFields> row{};
            row[0] = object.instance;
            row[1] = object.cls;
            row[2] = s.u0;
            row[3] = s.v0;
            row[4] = s.u1;
            row[5] = s.v1;
            row[6] = s.pixels;
            row[7] = kNaN;
            if (render_ && w == camera_.width && h == camera_.height) {
                const double alone = unoccludedPixels(bus, object, optical);
                row[7] = alone > 0.0 ? std::clamp(1.0 - s.pixels / alone, 0.0, 1.0) : kNaN;
            }
            box3d(bus, object, optical, w, h, row);
            out.insert(out.end(), row.begin(), row.end());
        }
        publish(bus, out, std::move(semantic));
        return {};
    }

private:
    /// The world pose of the frame the object's box is in.
    [[nodiscard]] Pose frameOf(const SignalBus& bus, const Object& object) const {
        return object.frameBody == kNoIndex ? object.frameOffset
                                            : poses_.pose(bus, object.frameBody);
    }
    [[nodiscard]] Pose partPose(const SignalBus& bus, const Part& p) const {
        return p.body == kNoIndex ? p.offset : compose(poses_.pose(bus, p.body), p.offset);
    }

    /// The 3D box (centre, size, yaw) and the truncation, into row[8..15].
    void box3d(const SignalBus& bus, const Object& object, const Pose& optical, std::uint32_t w,
               std::uint32_t h, std::array<double, kFields>& row) const {
        for (std::size_t k = 8; k < kFields; ++k) {
            row.at(k) = kNaN;
        }
        const Pose frame = frameOf(bus, object);
        Vec3 lo{1e300, 1e300, 1e300};
        Vec3 hi{-1e300, -1e300, -1e300};
        bool any = false;
        for (const std::uint32_t k : object.parts) {
            const Part& p = parts_[k];
            if (!p.hasBox) {
                continue;
            }
            const Pose in = relative(frame, partPose(bus, p));
            for (int c = 0; c < 8; ++c) {
                const Vec3 corner{(c & 1) != 0 ? p.hi[0] : p.lo[0],
                                  (c & 2) != 0 ? p.hi[1] : p.lo[1],
                                  (c & 4) != 0 ? p.hi[2] : p.lo[2]};
                const Vec3 q = add(in.position, rotate(in.orientation, corner));
                for (std::size_t i = 0; i < 3; ++i) {
                    lo.at(i) = std::min(lo.at(i), q.at(i));
                    hi.at(i) = std::max(hi.at(i), q.at(i));
                }
            }
            any = true;
        }
        if (!any) {
            return;
        }
        const Vec3 centreInFrame = scaled(add(lo, hi), 0.5);
        const Vec3 centre =
            toFrame(optical, add(frame.position, rotate(frame.orientation, centreInFrame)));
        row[9] = centre[0];
        row[10] = centre[1];
        row[11] = centre[2];
        row[12] = hi[0] - lo[0];
        row[13] = hi[1] - lo[1];
        row[14] = hi[2] - lo[2];
        // KITTI's rotation_y: the object's x axis about the optical y (down), 0 along the optical
        // x.
        const Vec3 heading =
            rotate(conjugate(optical.orientation), rotate(frame.orientation, {1.0, 0.0, 0.0}));
        row[15] = std::atan2(-heading[2], heading[0]);
        // Truncation: the share of the projected box outside the image (through a pinhole: a
        // fisheye's is not computed, NaN).
        if (!camera_.pinhole) {
            return;
        }
        double u0 = 1e300, v0 = 1e300, u1 = -1e300, v1 = -1e300;
        for (int c = 0; c < 8; ++c) {
            const Vec3 corner{(c & 1) != 0 ? hi[0] : lo[0], (c & 2) != 0 ? hi[1] : lo[1],
                              (c & 4) != 0 ? hi[2] : lo[2]};
            const Vec3 q = toFrame(optical, add(frame.position, rotate(frame.orientation, corner)));
            if (q[2] <= camera_.near) {
                return;  // partly behind the camera: truncation unknown (NaN)
            }
            const double u = camera_.fx * q[0] / q[2] + camera_.cx;
            const double v = camera_.fx * q[1] / q[2] + camera_.cy;
            u0 = std::min(u0, u);
            u1 = std::max(u1, u);
            v0 = std::min(v0, v);
            v1 = std::max(v1, v);
        }
        const double area = (u1 - u0) * (v1 - v0);
        const double iw = std::max(0.0, std::min(u1, w - 0.5) - std::max(u0, -0.5));
        const double ih = std::max(0.0, std::min(v1, h - 0.5) - std::max(v0, -0.5));
        row[8] = area > 0.0 ? std::clamp(1.0 - iw * ih / area, 0.0, 1.0) : 0.0;
    }

    /// The object's pixels when it is drawn alone: what it would cover with nothing in front.
    double unoccludedPixels(const SignalBus& bus, const Object& object, const Pose& optical) {
        instances_.clear();
        for (const std::uint32_t k : object.parts) {
            const Part& p = parts_[k];
            const Pose pose = partPose(bus, p);
            gpu::RasterInstance instance;
            instance.mesh = p.mesh;
            instance.position = pose.position;
            instance.rotation = pose.orientation;
            instance.scale = p.scale;
            instance.id = p.prim;
            instances_.push_back(instance);
        }
        gpu::PinholeCamera camera;
        camera.width = camera_.width;
        camera.height = camera_.height;
        camera.fx = camera_.fx;
        camera.fy = camera_.fx;
        camera.cx = camera_.cx;
        camera.cy = camera_.cy;
        camera.near = camera_.near;
        camera.position = optical.position;
        camera.orientation = optical.orientation;
        render_->render(instances_, camera, alone_);
        double count = 0.0;
        for (const std::uint32_t id : alone_.instance) {
            if (id != gpu::kNoInstance) {
                count += 1.0;
            }
        }
        return count;
    }

    void publish(SignalBus& bus, const std::vector<double>& rows,
                 std::shared_ptr<const Buffer> semantic) {
        std::vector<double> out(std::size_t{maxObjects_} * kFields, kNaN);
        std::copy(rows.begin(), rows.end(), out.begin());
        bus.writeN(detections_, out);
        bus.write(count_, static_cast<double>(rows.size() / kFields));
        if (!semantic) {
            auto empty = std::make_shared<Buffer>();
            empty->width = camera_.width;
            empty->height = camera_.height;
            empty->encoding = "r32u";
            semantic = std::move(empty);
        }
        bus.publishBuffer(semantic_, std::move(semantic));
    }

    Camera camera_;
    std::vector<Part> parts_;
    std::vector<Object> objects_;
    std::vector<std::uint32_t> objectOfPrim_;
    std::shared_ptr<gpu::RasterScene> raster_;
    std::unique_ptr<gpu::RasterCamera> render_;
    std::vector<gpu::RasterInstance> instances_;
    gpu::RasterImages alone_;
    std::uint32_t maxObjects_{64};
    double minPixels_{kDefaultMinPixels};
    bool occlusion_{true};
    BodyPoses poses_;
    SignalId instance_, detections_, count_, semantic_;
};

}  // namespace

std::unique_ptr<IModule> createDetections(const World& world, std::uint32_t sensor,
                                          bool groundTruth, const KindContext& context) {
    const std::string me = world.sensors.names[sensor];
    const std::string cameraName = ownSensorName(world, sensor, kindText(world, sensor, "camera"));
    std::uint32_t cam = kNoIndex;
    for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
        if (world.sensors.names[s] == cameraName && world.sensors.type[s] == SensorType::Camera) {
            cam = s;
        }
    }
    if (cam == kNoIndex) {
        return refuseKind(
            world, sensor,
            "detections " + me + ": parameter camera names no camera ('" + cameraName + "')");
    }
    std::string error;
    const std::vector<ClassRule> rules =
        parseClasses(kindText(world, sensor, "semantic", kDefaultClasses), error);
    if (!error.empty()) {
        return refuseKind(world, sensor, "detections " + me + ": semantic: " + error);
    }
    Detections::Camera camera;
    camera.name = cameraName;
    camera.body = world.sensors.body[cam].index;
    camera.mount = world.sensors.poseInBody[cam];
    camera.rateHz = world.sensors.rateHz[cam];
    const double width = kindSetting(world, cam, "width", 640.0);
    const double height = kindSetting(world, cam, "height", 480.0);
    const double fov = kindSetting(world, cam, "horizontal_fov", 1.047);
    camera.width = static_cast<std::uint32_t>(std::max(width, 1.0));
    camera.height = static_cast<std::uint32_t>(std::max(height, 1.0));
    camera.fx = width / 2.0 / std::tan(fov / 2.0);
    camera.cx = (width - 1.0) / 2.0;
    camera.cy = (height - 1.0) / 2.0;
    camera.near = kindSetting(world, cam, "near_clip", 0.05);
    camera.pinhole = !FisheyeLens::isFisheye(world.sensors.settings[cam]);

    // The parts: every drawable of the camera scene, with its mesh's box from the scene.
    std::vector<Part> parts;
    if (context.cameraScene) {
        for (const CameraScene::Drawable& drawable : context.cameraScene->drawables) {
            Part p;
            p.prim = drawable.prim;
            p.mesh = drawable.mesh;
            p.body = drawable.body;
            p.offset = drawable.offset;
            p.scale = drawable.scale;
            if (context.scene != nullptr && drawable.mesh < context.scene->meshes.size()) {
                const std::vector<float>& points = context.scene->meshes[drawable.mesh].positions;
                p.lo = {1e300, 1e300, 1e300};
                p.hi = {-1e300, -1e300, -1e300};
                for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
                    for (std::size_t k = 0; k < 3; ++k) {
                        const double v = static_cast<double>(points[i + k]) * drawable.scale.at(k);
                        p.lo.at(k) = std::min(p.lo.at(k), v);
                        p.hi.at(k) = std::max(p.hi.at(k), v);
                    }
                }
                p.hasBox = points.size() >= 3;
            }
            parts.push_back(p);
        }
    }
    // Objects: a robot (the root of a body tree), a walking human (one robot per person, RFC
    // 0009), or a static prim alone.
    const World::Bodies& bodies = world.bodies;
    std::vector<bool> human(world.robots.size(), false);
    for (const SchemaInstance& s : world.schemaInstances) {
        if (s.schema == "HumanAPI_1" && s.prim.index < world.prims.instanceNames.size()) {
            const std::string& n = world.prims.instanceNames[s.prim.index];
            for (std::size_t r = 0; r < world.robots.size(); ++r) {
                human[r] = human[r] || world.robots[r] == n;
            }
        }
    }
    const auto rootOf = [&](std::uint32_t b) {
        for (int guard = 0; guard < 4096 && bodies.parent[b] != kWorldBody; ++guard) {
            b = bodies.parent[b].index;
        }
        return b;
    };
    std::map<std::pair<int, std::uint32_t>, std::uint32_t> keyed;  // (kind, index) → object
    std::vector<Object> objects;
    const std::vector<std::string>* paths = nullptr;
    std::vector<std::string> scenePaths;
    if (context.scene != nullptr) {
        for (const ScenePrim& prim : context.scene->prims) {
            scenePaths.push_back(prim.path);
        }
        paths = &scenePaths;
    } else if (context.cameraScene) {
        for (const CameraScene::Drawable& drawable : context.cameraScene->drawables) {
            if (drawable.prim >= scenePaths.size()) {
                scenePaths.resize(drawable.prim + 1);
            }
            scenePaths[drawable.prim] = drawable.path;
        }
        paths = &scenePaths;
    }
    for (std::uint32_t k = 0; k < parts.size(); ++k) {
        const Part& p = parts[k];
        std::pair<int, std::uint32_t> key{2, p.prim};
        std::uint32_t frameBody = kNoIndex;
        if (p.body != kNoIndex && p.body < bodies.names.size()) {
            const std::uint32_t robot = p.body < bodies.robot.size() ? bodies.robot[p.body] : 0;
            if (robot < human.size() && human[robot]) {
                key = {1, robot};
            } else {
                key = {0, rootOf(p.body)};
            }
            frameBody = key.first == 0 ? key.second : p.body;
        }
        auto [it, fresh] = keyed.try_emplace(key, static_cast<std::uint32_t>(objects.size()));
        if (fresh) {
            Object o;
            o.frameBody = frameBody;
            o.frameOffset = p.offset;
            o.instance = p.prim;
            objects.push_back(o);
        }
        Object& o = objects[it->second];
        o.parts.push_back(k);
        o.instance = std::min(o.instance, static_cast<double>(p.prim));
    }
    // Classes: the body names and the parts' paths, else a moving body is a robot.
    const std::vector<bool> moving = movingBodies(world);
    for (Object& o : objects) {
        for (const std::uint32_t k : o.parts) {
            const Part& p = parts[k];
            if (o.cls == 0.0 && p.body != kNoIndex && p.body < bodies.names.size()) {
                o.cls = classOf(rules, bodies.names[rootOf(p.body)]);
                if (o.cls == 0.0) {
                    o.cls = classOf(rules, bodies.names[p.body]);
                }
            }
            if (o.cls == 0.0 && paths != nullptr && p.prim < paths->size()) {
                o.cls = classOf(rules, (*paths)[p.prim]);
            }
        }
        if (o.cls == 0.0 && o.frameBody < moving.size() && moving[o.frameBody]) {
            o.cls = kRobotClass;
        }
    }
    std::shared_ptr<gpu::RasterScene> raster =
        context.cameraScene ? context.cameraScene->raster : nullptr;
    return std::make_unique<Detections>(world, sensor, groundTruth, std::move(camera),
                                        std::move(parts), std::move(objects), std::move(raster));
}

}  // namespace lagrange::sensors::kinds
