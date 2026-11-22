#include "lagrange/vegetation/Vegetation.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <string>

namespace lagrange::vegetation {

namespace {

constexpr int kCells = 4;  // a side, per collider
/// m³, a tree's cut volume.
constexpr Unit kCubicMetres{{.length = 3}, 1.0, QuantityKind::Unspecified};

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

/// A shape's half extents in its frame and its volume (0 for planes and height fields).
std::pair<std::array<double, 3>, double> extentOf(ShapeType type, const std::array<double, 3>& s,
                                                  const ConvexHull* hull,
                                                  std::array<double, 3>& centre) {
    constexpr double kPi = std::numbers::pi;
    centre = {};
    switch (type) {
        case ShapeType::Sphere:
            return {{s[0], s[0], s[0]}, 4.0 / 3.0 * kPi * s[0] * s[0] * s[0]};
        case ShapeType::Capsule:
            return {{s[0], s[0], s[1] + s[0]}, kPi * s[0] * s[0] * (2.0 * s[1] + 4.0 / 3.0 * s[0])};
        case ShapeType::Cylinder:
            return {{s[0], s[0], s[1]}, kPi * s[0] * s[0] * 2.0 * s[1]};
        case ShapeType::Box:
            return {s, 8.0 * s[0] * s[1] * s[2]};
        case ShapeType::ConvexHull: {
            if (hull == nullptr || hull->points.empty()) {
                return {{}, 0.0};
            }
            std::array<double, 3> lo = hull->points.front();
            std::array<double, 3> hi = lo;
            for (const auto& p : hull->points) {
                for (std::size_t k = 0; k < 3; ++k) {
                    lo.at(k) = std::min(lo.at(k), p.at(k));
                    hi.at(k) = std::max(hi.at(k), p.at(k));
                }
            }
            double volume = 0.0;  // the divergence theorem over outward triangles
            for (const auto& t : hull->triangles) {
                const auto& a = hull->points.at(t[0]);
                const auto& b = hull->points.at(t[1]);
                const auto& c = hull->points.at(t[2]);
                volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
                           a[2] * (b[0] * c[1] - b[1] * c[0])) /
                          6.0;
            }
            std::array<double, 3> half{};
            for (std::size_t k = 0; k < 3; ++k) {
                centre.at(k) = (lo.at(k) + hi.at(k)) / 2.0;
                half.at(k) = (hi.at(k) - lo.at(k)) / 2.0;
            }
            return {half, std::abs(volume)};
        }
        case ShapeType::Plane:
        case ShapeType::HeightField:
            break;
    }
    return {{}, 0.0};
}

bool insideShape(ShapeType type, const std::array<double, 3>& s, const std::array<double, 3>& q,
                 const ConvexHull* hull) {
    switch (type) {
        case ShapeType::ConvexHull:
            return std::ranges::all_of(hull->faces, [&](const std::array<double, 4>& f) {
                return f[0] * q[0] + f[1] * q[1] + f[2] * q[2] <= f[3];
            });
        case ShapeType::Sphere:
            return std::hypot(q[0], q[1], q[2]) <= s[0];
        case ShapeType::Capsule:
            return std::hypot(q[0], q[1], q[2] - std::clamp(q[2], -s[1], s[1])) <= s[0];
        case ShapeType::Cylinder:
            return std::hypot(q[0], q[1]) <= s[0];
        default:
            return true;
    }
}

}  // namespace

std::vector<Canopy> canopiesOf(const World& world) {
    std::vector<Canopy> out;
    const World::Colliders& c = world.colliders;
    for (std::size_t i = 0; i < c.body.size(); ++i) {
        if (c.body[i] != kWorldBody || c.material[i] == kNoMaterial ||
            (c.shape[i] != ShapeType::Box && c.shape[i] != ShapeType::Plane)) {
            continue;
        }
        const SurfaceMaterial& m = world.surfaceMaterials.at(c.material[i]);
        if (m.canopyHeight > 0.0) {
            out.push_back({c.shape[i], c.size[i], c.poseInBody[i], m.canopyHeight, m.leafAreaIndex,
                           static_cast<std::uint32_t>(i)});
        }
    }
    return out;
}

namespace {

std::array<double, 3> inCanopyFrame(const Canopy& canopy, const std::array<double, 3>& point) {
    return rotate(conjugate(canopy.pose.orientation),
                  {point[0] - canopy.pose.position[0], point[1] - canopy.pose.position[1],
                   point[2] - canopy.pose.position[2]});
}

/// The soil's height in the canopy collider's frame: a box's top face, a plane's own.
double soilOf(const Canopy& canopy) {
    return canopy.type == ShapeType::Plane ? 0.0 : canopy.size[2];
}

bool overFace(const Canopy& canopy, const std::array<double, 3>& q) {
    return canopy.type == ShapeType::Plane ||
           (std::abs(q[0]) <= canopy.size[0] && std::abs(q[1]) <= canopy.size[1]);
}

}  // namespace

bool insideCanopy(const Canopy& canopy, const std::array<double, 3>& point, const CutMap* cuts) {
    const std::array<double, 3> q = inCanopyFrame(canopy, point);
    if (!overFace(canopy, q)) {
        return false;
    }
    double top = canopy.height;
    if (cuts != nullptr) {
        top = std::min(top, cuts->top(canopy.collider, q[0], q[1]));
    }
    const double over = q[2] - soilOf(canopy);
    return over >= 0.0 && over <= top;
}

double CutMap::top(std::uint32_t collider, double x, double y) const {
    const auto it = index_.find(canopyCutKey(collider, canopyCutIndex(x), canopyCutIndex(y)));
    if (it == index_.end()) {
        return std::numeric_limits<double>::infinity();
    }
    return values_[kCanopyCutHeader + (4 * static_cast<std::size_t>(it->second)) + 3];
}

double CutMap::sweep(const Canopy& canopy, const std::array<double, 3>& centre,
                     const std::array<double, 3>& normal, double radius) {
    if (!(radius > 0.0) || !(canopy.height > 0.0)) {
        return 0.0;
    }
    const double soil = soilOf(canopy);
    // The plants reach at most twice the preset height (the lidars' shaped canopy stands up to
    // (1 + spread)(1 + jitter) of it, under 1.7 for every form): a disc above that cuts nothing.
    const double reach = 2.0 * canopy.height;
    const std::array<double, 3> c = inCanopyFrame(canopy, centre);
    if (c[2] - radius > soil + reach) {
        return 0.0;
    }
    if (canopy.type == ShapeType::Box &&
        (std::abs(c[0]) > canopy.size[0] + radius || std::abs(c[1]) > canopy.size[1] + radius)) {
        return 0.0;
    }
    const std::array<double, 3> n = rotate(conjugate(canopy.pose.orientation), normal);
    // Two unit vectors across the disc.
    const std::array<double, 3> pick = std::abs(n[0]) < 0.9 ? std::array<double, 3>{1.0, 0.0, 0.0}
                                                            : std::array<double, 3>{0, 1, 0};
    std::array<double, 3> e1 = cross(n, pick);
    const double l1 = std::hypot(e1[0], e1[1], e1[2]);
    if (!(l1 > 0.0)) {
        return 0.0;
    }
    e1 = {e1[0] / l1, e1[1] / l1, e1[2] / l1};
    const std::array<double, 3> e2 = cross(n, e1);
    // Points on rings half a cell apart, so every cell under the disc is met.
    const double spacing = 0.5 * kCanopyCutCell;
    const int rings = std::max(1, static_cast<int>(std::ceil(radius / spacing)));
    const double cellArea = kCanopyCutCell * kCanopyCutCell;
    double area = 0.0;
    bool changed = false;
    for (int k = 0; k < rings; ++k) {
        const double r = std::min(radius, (k + 0.5) * spacing);
        const int around =
            std::max(1, static_cast<int>(std::ceil(2.0 * std::numbers::pi * r / spacing)));
        for (int a = 0; a < around; ++a) {
            const double angle = 2.0 * std::numbers::pi * (a + 0.5) / around;
            const double ca = r * std::cos(angle);
            const double sa = r * std::sin(angle);
            const std::array<double, 3> q{c[0] + ca * e1[0] + sa * e2[0],
                                          c[1] + ca * e1[1] + sa * e2[1],
                                          c[2] + ca * e1[2] + sa * e2[2]};
            const double over = q[2] - soil;
            if (over > reach || !overFace(canopy, q)) {
                continue;
            }
            const double cut = std::max(over, 0.0);
            const std::int32_t cellX = canopyCutIndex(q[0]);
            const std::int32_t cellY = canopyCutIndex(q[1]);
            const std::uint64_t key = canopyCutKey(canopy.collider, cellX, cellY);
            const auto it = index_.find(key);
            const double old =
                it == index_.end()
                    ? std::numeric_limits<double>::infinity()
                    : values_[kCanopyCutHeader + (4 * static_cast<std::size_t>(it->second)) + 3];
            if (!(cut < old - kMinCutPiece)) {
                continue;
            }
            // The stems were standing over the cut only where the plants reached above it.
            if (cut < std::min(old, canopy.height)) {
                area += cellArea;
            }
            std::size_t slot = 0;
            if (it == index_.end()) {
                if (full()) {
                    continue;
                }
                slot = index_.size();
                index_.emplace(key, static_cast<std::uint32_t>(slot));
                const std::size_t at = kCanopyCutHeader + (4 * slot);
                values_[at] = canopy.collider;
                values_[at + 1] = cellX;
                values_[at + 2] = cellY;
                values_[1] = static_cast<double>(index_.size());
            } else {
                slot = it->second;
            }
            values_[kCanopyCutHeader + (4 * slot) + 3] = cut;
            changed = true;
        }
    }
    if (changed) {
        values_[0] += 1.0;
    }
    return area;
}

bool CutMap::assign(std::span<const double> values) {
    if (values.size() != kCanopyCutLength) {
        return false;
    }
    values_.assign(values.begin(), values.end());
    index_.clear();
    const auto count = static_cast<std::size_t>(
        std::clamp(values_[1], 0.0, static_cast<double>(kCanopyCutCapacity)));
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t at = kCanopyCutHeader + (4 * k);
        index_.emplace(canopyCutKey(static_cast<std::uint32_t>(values_[at]),
                                    static_cast<std::int32_t>(values_[at + 1]),
                                    static_cast<std::int32_t>(values_[at + 2])),
                       static_cast<std::uint32_t>(k));
    }
    return true;
}

double frontalAreaDensity(double height, double leafAreaIndex) {
    if (!(height > 0.0) || !(leafAreaIndex > 0.0)) {
        return 0.0;
    }
    return kLeafProjection * leafAreaIndex / height;
}

std::array<double, 3> stemDrag(double density, double dragCoefficient, double frontalArea,
                               double volume, const std::array<double, 3>& velocity) {
    const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
    const double gain = 0.5 * density * dragCoefficient * frontalArea * volume * speed;
    return {-gain * velocity[0], -gain * velocity[1], -gain * velocity[2]};
}

double TreeCutMap::sweep(const TreeSpec& tree, std::uint32_t index,
                         const std::array<double, 3>& centre, const std::array<double, 3>& normal,
                         double radius, const std::array<double, 3>& sway,
                         std::vector<std::array<double, 3>>* cut) {
    if (!(radius > 0.0)) {
        return 0.0;
    }
    // Far from the crown: nothing to cut.
    const std::array<double, 3> c = inCrown(tree, centre, sway);
    const double reach = std::max(tree.crownRadius, tree.halfHeight()) + radius;
    if (std::hypot(c[0], c[1], c[2]) > reach) {
        return 0.0;
    }
    const std::array<double, 3> pick = std::abs(normal[0]) < 0.9
                                           ? std::array<double, 3>{1.0, 0.0, 0.0}
                                           : std::array<double, 3>{0.0, 1.0, 0.0};
    std::array<double, 3> e1 = cross(normal, pick);
    const double l1 = std::hypot(e1[0], e1[1], e1[2]);
    if (!(l1 > 0.0)) {
        return 0.0;
    }
    e1 = {e1[0] / l1, e1[1] / l1, e1[2] / l1};
    const std::array<double, 3> e2 = cross(normal, e1);
    // Points on rings half a cell apart, so every cell the disc passes through is met.
    const double spacing = 0.5 * kTreeCutCell;
    const int rings = std::max(1, static_cast<int>(std::ceil(radius / spacing)));
    constexpr double kCellVolume = kTreeCutCell * kTreeCutCell * kTreeCutCell;
    double volume = 0.0;
    for (int k = 0; k < rings; ++k) {
        const double r = std::min(radius, (k + 0.5) * spacing);
        const int around =
            std::max(1, static_cast<int>(std::ceil(2.0 * std::numbers::pi * r / spacing)));
        for (int a = 0; a < around && !full(); ++a) {
            const double angle = 2.0 * std::numbers::pi * (a + 0.5) / around;
            const double ca = r * std::cos(angle);
            const double sa = r * std::sin(angle);
            const std::array<double, 3> point{centre[0] + ca * e1[0] + sa * e2[0],
                                              centre[1] + ca * e1[1] + sa * e2[1],
                                              centre[2] + ca * e1[2] + sa * e2[2]};
            const std::array<double, 3> q = inCrown(tree, point, sway);
            if (!inEllipsoid(tree, q)) {
                continue;
            }
            const std::int32_t x = treeCutIndex(q[0]);
            const std::int32_t y = treeCutIndex(q[1]);
            const std::int32_t z = treeCutIndex(q[2]);
            if (!keys_.insert(treeCutKey(index, x, y, z)).second) {
                continue;  // already cut
            }
            const std::size_t at = kTreeCutHeader + (4 * (keys_.size() - 1));
            values_[at] = index;
            values_[at + 1] = x;
            values_[at + 2] = y;
            values_[at + 3] = z;
            volume += kCellVolume;
            if (cut != nullptr) {
                const std::array<double, 3> o = rotate(
                    tree.pose.orientation, {(x + 0.5) * kTreeCutCell, (y + 0.5) * kTreeCutCell,
                                            tree.centreHeight() + ((z + 0.5) * kTreeCutCell)});
                cut->push_back({tree.pose.position[0] + o[0] + sway[0],
                                tree.pose.position[1] + o[1] + sway[1],
                                tree.pose.position[2] + o[2] + sway[2]});
            }
        }
    }
    if (volume > 0.0) {
        values_[0] += 1.0;
        values_[1] = static_cast<double>(keys_.size());
        reader_.load(values_);
    }
    return volume;
}

bool TreeCutMap::assign(std::span<const double> values) {
    if (values.size() != kTreeCutLength) {
        return false;
    }
    values_.assign(values.begin(), values.end());
    keys_.clear();
    const auto count = static_cast<std::size_t>(
        std::clamp(values_[1], 0.0, static_cast<double>(kTreeCutCapacity)));
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t at = kTreeCutHeader + (4 * k);
        keys_.insert(treeCutKey(static_cast<std::uint32_t>(values_[at]),
                                static_cast<std::int32_t>(values_[at + 1]),
                                static_cast<std::int32_t>(values_[at + 2]),
                                static_cast<std::int32_t>(values_[at + 3])));
    }
    reader_ = TreeCuts{};
    reader_.load(values_);
    return true;
}

std::array<double, 2> crownForce(const TreeSpec& tree, double density,
                                 const std::array<double, 2>& air,
                                 const std::array<double, 2>& crownVelocity) {
    const std::array<double, 2> u{air[0] - crownVelocity[0], air[1] - crownVelocity[1]};
    const double speed = std::hypot(u[0], u[1]);
    const double gain = 0.5 * density * tree.dragCoefficient * tree.frontalArea() * speed;
    return {gain * u[0], gain * u[1]};
}

void stepSway(const TreeSpec& tree, Sway& sway, const std::array<double, 2>& force, double dt) {
    const double omega = 2.0 * std::numbers::pi * tree.naturalFrequency;
    for (std::size_t k = 0; k < 2; ++k) {
        const double acceleration = force.at(k) / tree.crownMass -
                                    2.0 * tree.dampingRatio * omega * sway.velocity.at(k) -
                                    omega * omega * sway.position.at(k);
        sway.velocity.at(k) += dt * acceleration;
        sway.position.at(k) += dt * sway.velocity.at(k);
    }
}

ModuleInfo Vegetation::info() const {
    return {"vegetation", "stem_drag", 1, 0};
}

Duration Vegetation::preferredStep() const {
    return Duration{1'000'000};
}

Status Vegetation::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    const World& raw = world.raw();
    canopies_ = canopiesOf(raw);
    std::string treeError;
    treeSpecs_ = treesOf(raw, treeError);
    if (!treeError.empty()) {
        return Status::error(StatusCode::InvalidConfig, treeError);
    }
    trees_.clear();
    for (const TreeSpec& t : treeSpecs_) {
        Tree tree;
        const std::string base = "tree/" + t.name + "/";
        SignalSpec air = spec(base + "downwash", SignalType::Vec3, units::m_per_s, "world_local");
        air.optional = true;
        air.feedback = true;
        air.defaultValue = 0.0;
        tree.downwash = d.input(air);
        tree.sway = d.output(spec(base + "sway", SignalType::Vec3, units::m, "world_local"));
        tree.force = d.output(spec(base + "air_force", SignalType::Vec3, units::N, "world_local"));
        tree.cutVolume = d.output(spec(base + "cut_volume", SignalType::Scalar, kCubicMetres));
        tree.leaves = d.output(spec(base + "leaves_shed", SignalType::Scalar, units::one));
        if (config.get<bool>("leaves").value_or(true)) {
            tree.flying = d.output(spec(base + "leaves_flying", SignalType::Scalar, units::one));
        }
        trees_.push_back(tree);
    }
    wind_ = SignalId{};
    if (!trees_.empty()) {
        SignalSpec wind =
            spec("env/wind/velocity", SignalType::Vec3, units::m_per_s, "world_local");
        wind.optional = true;
        wind.feedback = true;
        wind.defaultValue = 0.0;
        wind_ = d.input(wind);
    }
    leavesOut_ = SignalId{};
    if (!trees_.empty() && config.get<bool>("leaves").value_or(true)) {
        SignalSpec out =
            spec(std::string(kLeavesSignal), SignalType::ScalarArray, units::one, "world_local");
        out.arrayLength = kLeavesLength;
        leavesOut_ = d.output(out);
        landing_.build(raw);
    }
    bodies_.clear();
    density_ = std::max(0.0, config.get<double>("density").value_or(0.0));
    dragCoefficient_ =
        std::max(0.0, config.get<double>("drag_coefficient").value_or(kStemDragCoefficient));
    const std::vector<bool> moving = movingBodies(raw);
    for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
        if (!moving[b] || raw.bodies.kinematic[b]) {
            continue;
        }
        Body body;
        body.com = raw.bodies.comLocal[b];
        for (std::size_t c = 0; c < raw.colliders.body.size(); ++c) {
            if (raw.colliders.body[c] != BodyId{b}) {
                continue;
            }
            const ShapeType type = raw.colliders.shape[c];
            const std::array<double, 3>& size = raw.colliders.size[c];
            const ConvexHull* hull =
                type == ShapeType::ConvexHull ? &raw.hulls.at(raw.colliders.hull[c]) : nullptr;
            std::array<double, 3> centre{};
            const auto [half, volume] = extentOf(type, size, hull, centre);
            if (volume <= 0.0) {
                continue;
            }
            std::vector<Cell> cells;
            for (int i = 0; i < kCells; ++i) {
                for (int j = 0; j < kCells; ++j) {
                    for (int k = 0; k < kCells; ++k) {
                        const std::array<double, 3> q{
                            centre[0] + half[0] * (2.0 * (i + 0.5) / kCells - 1.0),
                            centre[1] + half[1] * (2.0 * (j + 0.5) / kCells - 1.0),
                            centre[2] + half[2] * (2.0 * (k + 0.5) / kCells - 1.0)};
                        if (insideShape(type, size, q, hull)) {
                            cells.push_back(
                                {compose(raw.colliders.poseInBody[c], Pose{q, {1.0, 0.0, 0.0, 0.0}})
                                     .position,
                                 0.0});
                        }
                    }
                }
            }
            for (Cell& cell : cells) {  // the shape's true volume, shared out
                cell.volume = volume / static_cast<double>(cells.size());
            }
            body.volume += volume;
            body.cells.insert(body.cells.end(), cells.begin(), cells.end());
        }
        if (body.cells.empty()) {
            continue;
        }
        const std::string base = "body/" + raw.bodies.names[b] + "/";
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        SignalSpec twist = spec(base + "twist", SignalType::Wrench, units::m_per_s, "world_local");
        pose.feedback = twist.feedback = true;
        body.pose = d.input(pose);
        body.twist = d.input(twist);
        body.wrench =
            d.output(spec(base + "wrench/vegetation", SignalType::Wrench, units::N, "world_local"));
        body.inside = d.output(spec(base + "in_vegetation", SignalType::Scalar, units::one));
        bodies_.push_back(std::move(body));
    }
    // Cutting: every rotor, while the run has canopies.
    rotors_.clear();
    cutOut_ = SignalId{};
    cutEnergy_ = std::max(0.0, config.get<double>("cut_energy").value_or(kCutEnergyPerArea));
    treeCutOut_ = SignalId{};
    if ((canopies_.empty() && treeSpecs_.empty()) || !config.get<bool>("cutting").value_or(true)) {
        return {};
    }
    const World::Joints& joints = world.joints();
    const World::Bodies& bodies = world.bodies();
    for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
        const auto j =
            static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) - joints.prim.begin());
        const auto radius = s->real("aero:rotor:radius");
        if (j >= joints.prim.size() || !radius || !(*radius > 0.0)) {
            continue;  // the rotor domain says what is wrong with it
        }
        Rotor r;
        r.radius = *radius;
        const std::array<double, 3>& a = joints.axis[j];
        const double norm = std::hypot(a[0], a[1], a[2]);
        if (!(norm > 0.0)) {
            continue;
        }
        r.axis = {a[0] / norm, a[1] / norm, a[2] / norm};
        const std::size_t child = joints.child[j].index;
        const std::array<double, 6>& in = bodies.inertiaLocal[child];  // xx yy zz xy xz yz
        const std::array<double, 3>& u = r.axis;
        r.inertia = in[0] * u[0] * u[0] + in[1] * u[1] * u[1] + in[2] * u[2] * u[2] +
                    2.0 * (in[3] * u[0] * u[1] + in[4] * u[0] * u[2] + in[5] * u[1] * u[2]);
        const std::string joint(world.signalInstanceName(s->prim));
        const std::string link(world.signalInstanceName(bodies.prim[child]));
        SignalSpec pose = spec("body/" + link + "/pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        r.pose = d.input(pose);
        SignalSpec velocity =
            spec("joint/" + joint + "/velocity", SignalType::Scalar, units::rad_per_s);
        velocity.feedback = true;
        r.velocity = d.input(velocity);
        r.wrench = d.output(
            spec("body/" + link + "/wrench/cutting", SignalType::Wrench, units::N, "world_local"));
        r.power = d.output(spec("rotor/" + joint + "/cutting_power", SignalType::Scalar, units::W));
        if (leavesOut_.valid()) {  // the wake the flying leaves fly in
            const std::string rotor = "rotor/" + joint + "/";
            SignalSpec induced =
                spec(rotor + "induced_velocity", SignalType::Scalar, units::m_per_s);
            SignalSpec thrustAxis =
                spec(rotor + "thrust_axis", SignalType::Vec3, units::one, "world_local");
            for (SignalSpec* input : {&induced, &thrustAxis}) {
                input->optional = true;
                input->feedback = true;
                input->defaultValue = 0.0;
            }
            r.induced = d.input(induced);
            r.thrustAxis = d.input(thrustAxis);
        }
        rotors_.push_back(r);
    }
    if (!rotors_.empty() && !canopies_.empty()) {
        SignalSpec out = spec(std::string(kCanopyCutSignal), SignalType::ScalarArray, units::m);
        out.arrayLength = kCanopyCutLength;
        cutOut_ = d.output(out);
    }
    if (!rotors_.empty() && !treeSpecs_.empty()) {
        SignalSpec out = spec(std::string(kTreeCutSignal), SignalType::ScalarArray, units::one);
        out.arrayLength = kTreeCutLength;
        treeCutOut_ = d.output(out);
    }
    return {};
}

Status Vegetation::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (const Body& body : bodies_) {
        bus.writeN(body.wrench, std::array<double, 6>{});
        bus.write(body.inside, 0.0);
    }
    for (const Rotor& r : rotors_) {
        bus.writeN(r.wrench, std::array<double, 6>{});
        bus.write(r.power, 0.0);
    }
    if (cutOut_.valid()) {
        bus.writeN(cutOut_, cuts_.values());
    }
    if (treeCutOut_.valid()) {
        bus.writeN(treeCutOut_, treeCuts_.values());
    }
    for (Tree& tree : trees_) {
        tree.state = Sway{};
        tree.cut = 0.0;
        bus.writeN(tree.sway, std::array<double, 3>{});
        bus.writeN(tree.force, std::array<double, 3>{});
        bus.write(tree.cutVolume, 0.0);
        bus.write(tree.leaves, 0.0);
        if (tree.flying.valid()) {
            bus.write(tree.flying, 0.0);
        }
    }
    leaves_.reset(trees_.size());
    leafVersion_ = 0.0;
    treeCells_.assign(trees_.size(), {});
    if (leavesOut_.valid()) {
        leaves_.write(leafValues_, leafVersion_);
        bus.writeN(leavesOut_, leafValues_);
    }
    return {};
}

Status Vegetation::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const double version = cuts_.values()[0];
    const double treeVersion = treeCuts_.values()[0];
    // The crowns' sway in the wind and the rotors' air.
    const std::span<const double> wind =
        wind_.valid() ? bus.readN(wind_) : std::span<const double>{};
    std::vector<std::array<double, 3>> sways(trees_.size());
    for (std::size_t k = 0; k < trees_.size(); ++k) {
        Tree& tree = trees_[k];
        const std::span<const double> w = bus.readN(tree.downwash);
        std::array<double, 2> air{};
        for (std::size_t c = 0; c < 2; ++c) {
            air.at(c) = (wind.size() == 3 ? wind[c] : 0.0) + (w.size() == 3 ? w[c] : 0.0);
        }
        const std::array<double, 2> force =
            crownForce(treeSpecs_[k], density_, air, tree.state.velocity);
        stepSway(treeSpecs_[k], tree.state, force, dt);
        sways[k] = {tree.state.position[0], tree.state.position[1], 0.0};
        bus.writeN(tree.sway, sways[k]);
        bus.writeN(tree.force, std::array<double, 3>{force[0], force[1], 0.0});
    }
    for (Rotor& r : rotors_) {
        const std::span<const double> p = bus.readN(r.pose);
        const double omega = bus.read(r.velocity);
        std::array<double, 6> wrench{};
        double power = 0.0;
        if (p.size() == 7 && std::abs(omega) * r.radius >= kMinCutTipSpeed) {
            const Pose hub{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            const std::array<double, 3> axis = rotate(hub.orientation, r.axis);
            double area = 0.0;
            for (const Canopy& canopy : canopies_) {
                area += cuts_.sweep(canopy, hub.position, axis, r.radius);
            }
            r.owed += area * cutEnergy_;
            for (std::size_t k = 0; k < trees_.size(); ++k) {
                cutScratch_.clear();
                const double cut =
                    treeCuts_.sweep(treeSpecs_[k], static_cast<std::uint32_t>(k), hub.position,
                                    axis, r.radius, sways[k], &cutScratch_);
                collectCut(k, hub.position, axis, omega, r.radius);
                trees_[k].cut += cut;
                r.owed += cut * treeSpecs_[k].cutEnergy;
            }
            // The cut is paid by the spinning propeller, no faster than a twentieth of its
            // kinetic energy per step, so a deep plunge slows it over steps instead of reversing
            // it in one (a propeller without inertia pays at once).
            double paid = r.owed;
            if (r.inertia > 0.0) {
                constexpr double kShare = 0.05;
                paid = std::min(paid, kShare * 0.5 * r.inertia * omega * omega);
            }
            r.owed -= paid;
            if (paid > 0.0 && dt > 0.0) {
                power = paid / dt;
                const double torque = -std::copysign(power / std::abs(omega), omega);
                for (std::size_t k = 0; k < 3; ++k) {
                    wrench.at(k + 3) = torque * axis.at(k);
                }
            }
        } else {
            r.owed = 0.0;  // stopped: what it did not cut, it never will
        }
        bus.writeN(r.wrench, wrench);
        bus.write(r.power, power);
    }
    if (cutOut_.valid()) {
        if (cuts_.values()[0] != version) {
            bus.writeN(cutOut_, cuts_.values());
        } else {
            bus.hold(cutOut_);
        }
    }
    if (treeCutOut_.valid()) {
        if (treeCuts_.values()[0] != treeVersion) {
            bus.writeN(treeCutOut_, treeCuts_.values());
        } else {
            bus.hold(treeCutOut_);
        }
    }
    stepLeaves(context, bus, sways, wind);
    for (std::size_t k = 0; k < trees_.size(); ++k) {
        const TreeSpec& t = treeSpecs_[k];
        bus.write(trees_[k].cutVolume, trees_[k].cut);
        bus.write(trees_[k].leaves, std::floor(trees_[k].cut * t.leafAreaDensity / t.leafArea));
    }
    for (const Body& body : bodies_) {
        const std::span<const double> p = bus.readN(body.pose);
        const std::span<const double> v = bus.readN(body.twist);
        std::array<double, 6> wrench{};
        double inside = 0.0;
        if (p.size() == 7 && v.size() == 6) {
            const Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            const std::array<double, 3> com = compose(pose, Pose{body.com, {1, 0, 0, 0}}).position;
            const std::array<double, 3> spin{v[3], v[4], v[5]};
            for (const Cell& cell : body.cells) {
                const std::array<double, 3> at =
                    compose(pose, Pose{cell.at, {1, 0, 0, 0}}).position;
                for (const Canopy& canopy : canopies_) {
                    if (!insideCanopy(canopy, at, &cuts_)) {
                        continue;
                    }
                    inside += cell.volume;
                    const std::array<double, 3> around =
                        cross(spin, {at[0] - p[0], at[1] - p[1], at[2] - p[2]});
                    const std::array<double, 3> force = stemDrag(
                        density_, dragCoefficient_,
                        frontalAreaDensity(canopy.height, canopy.leafAreaIndex), cell.volume,
                        {v[0] + around[0], v[1] + around[1], v[2] + around[2]});
                    const std::array<double, 3> moment =
                        cross({at[0] - com[0], at[1] - com[1], at[2] - com[2]}, force);
                    for (std::size_t k = 0; k < 3; ++k) {
                        wrench.at(k) += force.at(k);
                        wrench.at(k + 3) += moment.at(k);
                    }
                    break;  // one canopy per cell: overlapping canopies do not add up
                }
                for (std::size_t k = 0; k < treeSpecs_.size(); ++k) {
                    const TreeSpec& t = treeSpecs_[k];
                    if (!inLeaves(t, static_cast<std::uint32_t>(k), at, sways[k],
                                  &treeCuts_.cuts())) {
                        continue;
                    }
                    inside += cell.volume;
                    // Relative to the swaying crown.
                    const std::array<double, 3> around =
                        cross(spin, {at[0] - p[0], at[1] - p[1], at[2] - p[2]});
                    const std::array<double, 2>& cv = trees_[k].state.velocity;
                    const std::array<double, 3> force = stemDrag(
                        density_, dragCoefficient_, kLeafProjection * t.leafAreaDensity,
                        cell.volume,
                        {v[0] + around[0] - cv[0], v[1] + around[1] - cv[1], v[2] + around[2]});
                    const std::array<double, 3> moment =
                        cross({at[0] - com[0], at[1] - com[1], at[2] - com[2]}, force);
                    for (std::size_t c = 0; c < 3; ++c) {
                        wrench.at(c) += force.at(c);
                        wrench.at(c + 3) += moment.at(c);
                    }
                    break;
                }
            }
        }
        bus.writeN(body.wrench, wrench);
        bus.write(body.inside, body.volume > 0.0 ? inside / body.volume : 0.0);
    }
    return {};
}

void Vegetation::collectCut(std::size_t tree, const std::array<double, 3>& hub,
                            const std::array<double, 3>& axis, double omega, double radius) {
    if (!leavesOut_.valid() || tree >= treeCells_.size()) {
        return;
    }
    const std::array<double, 3> spin{omega * axis[0], omega * axis[1], omega * axis[2]};
    for (const std::array<double, 3>& c : cutScratch_) {
        // The blade's velocity at the cell's foot on the disc, its radius at most the blade's.
        std::array<double, 3> r{c[0] - hub[0], c[1] - hub[1], c[2] - hub[2]};
        const double along = r[0] * axis[0] + r[1] * axis[1] + r[2] * axis[2];
        r = {r[0] - along * axis[0], r[1] - along * axis[1], r[2] - along * axis[2]};
        const double length = std::hypot(r[0], r[1], r[2]);
        if (length > radius) {
            r = {r[0] * radius / length, r[1] * radius / length, r[2] * radius / length};
        }
        treeCells_[tree].push_back({c, cross(spin, r)});
    }
}

void Vegetation::stepLeaves(const StepContext& context, SignalBus& bus,
                            std::span<const std::array<double, 3>> sways,
                            std::span<const double> wind) {
    if (!leavesOut_.valid()) {
        return;
    }
    wakes_.clear();
    for (const Rotor& r : rotors_) {
        if (!r.induced.valid()) {
            continue;
        }
        const std::span<const double> p = bus.readN(r.pose);
        const std::span<const double> n = bus.readN(r.thrustAxis);
        if (p.size() == 7 && n.size() == 3) {
            wakes_.push_back(
                {{p[0], p[1], p[2]}, {n[0], n[1], n[2]}, r.radius, bus.read(r.induced)});
        }
    }
    const std::array<double, 3> w = wind.size() == 3
                                        ? std::array<double, 3>{wind[0], wind[1], wind[2]}
                                        : std::array<double, 3>{};
    const double dt = context.dt.seconds();
    std::uint32_t started = 0;
    for (std::size_t k = 0; k < trees_.size(); ++k) {
        const auto tree = static_cast<std::uint32_t>(k);
        started += leaves_.spawnFromCut(tree, treeSpecs_[k], treeCells_[k], wakes_, w, context.seed,
                                        context.stepIndex);
        treeCells_[k].clear();
        const std::span<const double> down = bus.readN(trees_[k].downwash);
        std::array<double, 3> air = w;
        if (down.size() == 3) {
            air = {air[0] + down[0], air[1] + down[1], air[2] + down[2]};
        }
        started += leaves_.strip(tree, treeSpecs_[k], trees_[k].cut, air, sways[k], wakes_, w, dt,
                                 context.seed, context.stepIndex);
    }
    const bool moved = leaves_.step(dt, w, wakes_, landing_);
    if (started > 0 || moved) {
        leafVersion_ += 1.0;
        leaves_.write(leafValues_, leafVersion_);
        bus.writeN(leavesOut_, leafValues_);
    } else {
        bus.hold(leavesOut_);
    }
    for (std::size_t k = 0; k < trees_.size(); ++k) {
        if (trees_[k].flying.valid()) {
            bus.write(trees_[k].flying,
                      static_cast<double>(leaves_.flying(static_cast<std::uint32_t>(k))));
        }
    }
}

Status Vegetation::saveState(StateWriter& writer) const {
    writer.putDoubles(cuts_.values());
    for (const Rotor& r : rotors_) {
        writer.put(r.owed);
    }
    writer.putDoubles(treeCuts_.values());
    for (const Tree& t : trees_) {
        for (const double v : {t.state.position[0], t.state.position[1], t.state.velocity[0],
                               t.state.velocity[1], t.cut}) {
            writer.put(v);
        }
    }
    const std::vector<double> leaves = leaves_.state();
    writer.put(leafVersion_);
    writer.put<std::uint64_t>(leaves.size());
    writer.putDoubles(leaves);
    return {};
}

Status Vegetation::restoreState(StateReader& reader) {
    std::vector<double> values(kCanopyCutLength, 0.0);
    if (!reader.getDoubles(values) || !cuts_.assign(values)) {
        return Status::error(StatusCode::Internal, "vegetation: the saved cut map is short");
    }
    for (Rotor& r : rotors_) {
        reader.get(r.owed);
    }
    std::vector<double> trees(kTreeCutLength, 0.0);
    if (!reader.getDoubles(trees) || !treeCuts_.assign(trees)) {
        return Status::error(StatusCode::Internal, "vegetation: the saved tree cuts are short");
    }
    for (Tree& t : trees_) {
        for (double* v : {&t.state.position[0], &t.state.position[1], &t.state.velocity[0],
                          &t.state.velocity[1], &t.cut}) {
            if (!reader.get(*v)) {
                return Status::error(StatusCode::Internal, "vegetation: short tree state");
            }
        }
    }
    std::uint64_t size = 0;
    if (!reader.get(leafVersion_) || !reader.get(size) || size > (1U << 26U)) {
        return Status::error(StatusCode::Internal, "vegetation: no saved leaves");
    }
    std::vector<double> leaves(size);
    if (!reader.getDoubles(leaves) || !leaves_.assign(leaves)) {
        return Status::error(StatusCode::Internal, "vegetation: the saved leaves are short");
    }
    treeCells_.assign(trees_.size(), {});
    return {};
}

}  // namespace lagrange::vegetation
