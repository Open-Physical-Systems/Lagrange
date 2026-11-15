// URDF writer and reader of the robot builder (RFC 0016 §6, §10; work package WP-B).
//
// The writer prints the document itself (numbers through std::to_chars, shortest round trip) and
// prints kept XML through tinyxml2's printer, so the same file read and written again gives the
// same bytes. Where the robot's and links' kept elements stood among the modelled ones is held by
// placement markers in keptXml (comments `<!--lagrange-builder:link-->`, see Urdf.hpp); a marker
// stands for "the next link (joint, sensor, visual, ...) in document order".
//
// What URDF has no place for (plan WP-B; the server's loader ignores unknown attributes, as
// urdfdom does): a link's density is the attribute `lagrange_density` (kg/m³) on <link>; which
// parts of an <inertial> are the user's overrides is `lagrange_overrides="mass com inertia"` on
// <inertial> (written only for a link with a density; the parts not listed were computed and are
// computed again on reading); a shape that counts for mass against the default (collision shapes
// do, visual-only ones do not) carries `lagrange_solid="true|false"`.

#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <map>
#include <numbers>
#include <set>
#include <sys/stat.h>
#include <tinyxml2.h>
#include <unistd.h>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

constexpr std::string_view kMarker = "lagrange-builder";
constexpr const char* kDensityAttribute = "lagrange_density";
/// Addendum A: how the link collides (Link::collision), ignored by the loaders.
constexpr const char* kCollisionAttribute = "lagrange_collision";
constexpr const char* kOverridesAttribute = "lagrange_overrides";
constexpr const char* kSolidAttribute = "lagrange_solid";
constexpr double kDefaultUpdateRate = 10.0;  // Hz, the server's reader's default (Robot.cpp)

// --- numbers --------------------------------------------------------------------------------

std::string num(double value) {
    if (value == 0.0) {
        return "0";  // never "-0"
    }
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return {buffer.data(), result.ptr};
}

std::string num(float value) {
    if (value == 0.0F) {
        return "0";
    }
    std::array<char, 64> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return {buffer.data(), result.ptr};
}

template <typename T, std::size_t N>
std::string joined(const std::array<T, N>& values) {
    std::string out;
    for (std::size_t i = 0; i < N; ++i) {
        out += (i == 0 ? "" : " ") + num(values[i]);
    }
    return out;
}

bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

std::string_view trim(std::string_view text) {
    while (!text.empty() && isSpace(text.front())) {
        text.remove_prefix(1);
    }
    while (!text.empty() && isSpace(text.back())) {
        text.remove_suffix(1);
    }
    return text;
}

/// Whitespace-separated numbers (locale independent); nullopt if any word is not a number.
std::optional<std::vector<double>> numbers(const char* text) {
    if (text == nullptr) {
        return std::nullopt;
    }
    std::vector<double> out;
    std::string_view rest(text);
    while (true) {
        while (!rest.empty() && isSpace(rest.front())) {
            rest.remove_prefix(1);
        }
        if (rest.empty()) {
            break;
        }
        if (rest.front() == '+') {
            rest.remove_prefix(1);
        }
        double value = 0.0;
        const auto result = std::from_chars(rest.data(), rest.data() + rest.size(), value);
        if (result.ec != std::errc{} ||
            (result.ptr != rest.data() + rest.size() && !isSpace(*result.ptr))) {
            return std::nullopt;
        }
        out.push_back(value);
        rest.remove_prefix(static_cast<std::size_t>(result.ptr - rest.data()));
    }
    return out;
}

std::optional<double> number(const char* text) {
    const auto values = numbers(text);
    if (!values || values->size() != 1) {
        return std::nullopt;
    }
    return values->front();
}

template <std::size_t N>
std::optional<std::array<double, N>> fixedNumbers(const char* text) {
    const auto values = numbers(text);
    if (!values || values->size() != N) {
        return std::nullopt;
    }
    std::array<double, N> out{};
    std::copy(values->begin(), values->end(), out.begin());
    return out;
}

std::string escape(std::string_view text) {
    std::string out;
    for (const char c : text) {
        switch (c) {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            default:
                out += c;
        }
    }
    return out;
}

// --- rotations (URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)) -----------------------------------

using Matrix = std::array<std::array<double, 3>, 3>;

Matrix rotation(const std::array<double, 3>& rpy) {
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

Matrix tensor(const std::array<double, 6>& i) {
    return {{{i[0], i[1], i[2]}, {i[1], i[3], i[4]}, {i[2], i[4], i[5]}}};
}

std::array<double, 6> packed(const Matrix& m) {
    return {m[0][0], m[0][1], m[0][2], m[1][1], m[1][2], m[2][2]};
}

/// R I Rᵀ.
Matrix rotated(const Matrix& r, const Matrix& inertia) {
    Matrix out{};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                for (int l = 0; l < 3; ++l) {
                    sum += r[a][k] * inertia[k][l] * r[b][l];
                }
            }
            out[a][b] = sum;
        }
    }
    return out;
}

bool isZero(const Pose& pose) {
    return pose.xyz == std::array<double, 3>{} && pose.rpy == std::array<double, 3>{};
}

/// What the written <inertial> says: the overrides as they are when all three are given (so
/// files of overrides never depend on the mass code), else linkMass() with the overrides applied.
std::optional<MassProperties> inertialOf(const Link& link) {
    const Inertial& in = link.inertial;
    if (in.mass && in.com && in.inertia) {
        return MassProperties{*in.mass, *in.com, *in.inertia};
    }
    return linkMass(link);
}

// --- kept XML -------------------------------------------------------------------------------

/// tinyxml2's printer with two spaces per level, starting at `depth`.
class Printer : public tinyxml2::XMLPrinter {
public:
    explicit Printer(int depth) : XMLPrinter(nullptr, false, depth) {}

protected:
    void PrintSpace(int depth) override {
        for (int i = 0; i < depth; ++i) {
            Write("  ");
        }
    }
};

std::string printed(const XMLNode& node, int depth) {
    Printer printer(depth);
    node.Accept(&printer);
    return std::string(trim(printer.CStr()));
}

std::string marker(std::string_view kind) {
    return "<!--" + std::string(kMarker) + ":" + std::string(kind) + "-->";
}

/// The marker's kind ("link", "visual", ...) if `node` is a placement marker, else "".
std::string markerKind(const XMLNode& node) {
    const tinyxml2::XMLComment* comment = node.ToComment();
    if (comment == nullptr || comment->Value() == nullptr) {
        return {};
    }
    const std::string_view value = comment->Value();
    if (!value.starts_with(kMarker) || value.size() <= kMarker.size() + 1 ||
        value[kMarker.size()] != ':') {
        return {};
    }
    return std::string(value.substr(kMarker.size() + 1));
}

std::string joinedKept(const std::vector<std::string>& pieces) {
    std::string out;
    for (const std::string& piece : pieces) {
        out += (out.empty() ? "" : "\n") + piece;
    }
    return out;
}

/// keptXml parsed again for writing. A fragment that does not parse (edited by hand into the
/// document) is written as a comment, so the file stays well-formed.
struct Kept {
    XMLDocument xml;
    std::vector<const XMLNode*> nodes;
    std::string broken;

    explicit Kept(const std::string& text) {
        if (trim(text).empty()) {
            return;
        }
        if (xml.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS) {
            broken = text;
            return;
        }
        for (const XMLNode* n = xml.FirstChild(); n != nullptr; n = n->NextSibling()) {
            nodes.push_back(n);
        }
    }
};

class Out {
public:
    void line(int depth, std::string_view text) {
        text_.append(static_cast<std::size_t>(depth) * 2, ' ');
        text_ += text;
        text_ += '\n';
    }
    void kept(int depth, const XMLNode& node) { line(depth, printed(node, depth)); }
    void brokenKept(int depth, const std::string& text) {
        std::string safe = text;
        for (std::size_t at = safe.find("--"); at != std::string::npos; at = safe.find("--")) {
            safe.replace(at, 2, "- -");
        }
        line(depth, "<!-- kept XML that does not parse: " + safe + " -->");
    }
    [[nodiscard]] std::string take() { return std::move(text_); }

private:
    std::string text_;
};

/// Walks kept nodes with placement markers: `emit(kind, all)` writes the next item of `kind`
/// (or, with all, every one left). A kind's last marker writes what is left of it; the last
/// marker of all writes what is left of every kind; with no marker, the items follow the kept
/// nodes.
template <typename Emit>
void walkKept(Out& out, int depth, const std::vector<const XMLNode*>& nodes,
              const std::vector<std::string>& kinds, Emit emit) {
    std::map<std::string, std::size_t, std::less<>> lastOf;
    std::optional<std::size_t> lastAny;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const std::string kind = markerKind(*nodes[i]);
        if (std::ranges::find(kinds, kind) != kinds.end()) {
            lastOf[kind] = i;
            lastAny = i;
        }
    }
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const std::string kind = markerKind(*nodes[i]);
        if (std::ranges::find(kinds, kind) != kinds.end()) {
            emit(kind, false);
            if (lastOf[kind] == i) {
                emit(kind, true);
            }
            if (lastAny == i) {
                for (const std::string& k : kinds) {
                    emit(k, true);
                }
            }
        } else if (kind.empty()) {
            out.kept(depth, *nodes[i]);
        }
        // A marker of another level (left by hand) is dropped.
    }
    if (!lastAny) {
        for (const std::string& k : kinds) {
            emit(k, true);
        }
    }
}

// --- writer ---------------------------------------------------------------------------------

std::string meshPath(const std::string& mesh, const WriteOptions& options) {
    if (mesh.find("://") != std::string::npos) {
        return mesh;  // package://, lagrange-asset://, file://: as written
    }
    std::filesystem::path path(mesh);
    const std::filesystem::path folder = options.savedAt.parent_path();
    std::error_code error;
    if (path.is_relative() && options.meshFolder) {
        // Where the document reads it: the file it came from (or the working directory).
        const std::filesystem::path from = options.meshFolder->empty()
                                               ? std::filesystem::current_path(error)
                                               : *options.meshFolder;
        const std::filesystem::path to =
            folder.empty() ? std::filesystem::current_path(error) : folder;
        if (!options.absoluteMeshes &&
            std::filesystem::absolute(from, error).lexically_normal() ==
                std::filesystem::absolute(to, error).lexically_normal()) {
            return mesh;  // the same folder: as written
        }
        path = std::filesystem::absolute(from / path, error).lexically_normal();
    }
    if (options.absoluteMeshes) {
        if (path.is_absolute()) {
            return path.string();
        }
        const std::filesystem::path base =
            folder.empty() ? std::filesystem::current_path(error) : folder;
        return std::filesystem::absolute(base / path, error).lexically_normal().string();
    }
    if (!options.savedAt.empty() && path.is_absolute()) {
        const std::filesystem::path base = std::filesystem::absolute(folder, error);
        const std::filesystem::path relative = path.lexically_relative(base);
        if (!relative.empty()) {
            return relative.string();
        }
    }
    return path.is_absolute() ? path.string() : mesh;
}

/// Colours by name: the kept robot-level <material> definitions, and the ones the writer adds.
class Colours {
public:
    explicit Colours(const Kept& kept) {
        for (const XMLNode* node : kept.nodes) {
            const XMLElement* e = node->ToElement();
            if (e == nullptr || std::string_view(e->Name()) != "material" ||
                e->Attribute("name") == nullptr) {
                continue;
            }
            taken_.insert(e->Attribute("name"));
            const XMLElement* colour = e->FirstChildElement("color");
            if (const auto rgba = fixedNumbers<4>(colour ? colour->Attribute("rgba") : nullptr)) {
                const std::array<float, 4> value{
                    static_cast<float>((*rgba)[0]), static_cast<float>((*rgba)[1]),
                    static_cast<float>((*rgba)[2]), static_cast<float>((*rgba)[3])};
                byColour_.emplace(value, e->Attribute("name"));  // the first of a colour
            }
        }
    }
    /// The name a colour is written under; a new one is added for writing at the top.
    std::string nameOf(const std::array<float, 4>& rgba) {
        if (auto it = byColour_.find(rgba); it != byColour_.end()) {
            return it->second;
        }
        std::string name;
        for (int k = 1;; ++k) {
            name = "builder_color_" + std::to_string(k);
            if (!taken_.contains(name)) {
                break;
            }
        }
        taken_.insert(name);
        byColour_.emplace(rgba, name);
        added_.emplace_back(name, rgba);
        return name;
    }
    [[nodiscard]] const std::vector<std::pair<std::string, std::array<float, 4>>>& added() const {
        return added_;
    }

private:
    std::map<std::array<float, 4>, std::string> byColour_;
    std::set<std::string, std::less<>> taken_;
    std::vector<std::pair<std::string, std::array<float, 4>>> added_;
};

void writeOrigin(Out& out, int depth, const Pose& pose) {
    out.line(depth, "<origin xyz=\"" + joined(pose.xyz) + "\" rpy=\"" + joined(pose.rpy) + "\"/>");
}

std::string geometry(const Shape& shape, const WriteOptions& options) {
    switch (shape.kind) {
        case Shape::Kind::Box:
            return "<box size=\"" + joined(shape.size) + "\"/>";
        case Shape::Kind::Cylinder:
            return "<cylinder radius=\"" + num(shape.size[0]) + "\" length=\"" +
                   num(shape.size[1]) + "\"/>";
        case Shape::Kind::Sphere:
            return "<sphere radius=\"" + num(shape.size[0]) + "\"/>";
        case Shape::Kind::Capsule:
            return "<capsule radius=\"" + num(shape.size[0]) + "\" length=\"" + num(shape.size[1]) +
                   "\"/>";
        case Shape::Kind::Mesh: {
            std::string text = "<mesh filename=\"" + escape(meshPath(shape.mesh, options)) + "\"";
            if (shape.scale != std::array<double, 3>{1, 1, 1}) {
                text += " scale=\"" + joined(shape.scale) + "\"";
            }
            return text + "/>";
        }
    }
    return {};
}

void writeShape(Out& out, const Shape& shape, bool visual, const WriteOptions& options,
                Colours& colours) {
    const char* tag = visual ? "visual" : "collision";
    std::string open = std::string("<") + tag;
    // The solid flag goes on the shape's first element when it is not the default (§4).
    const bool first = visual || !shape.visual;
    if (first && shape.solid != shape.collision) {
        open +=
            std::string(" ") + kSolidAttribute + "=\"" + (shape.solid ? "true" : "false") + "\"";
    }
    out.line(2, open + ">");
    if (!isZero(shape.origin)) {
        writeOrigin(out, 3, shape.origin);
    }
    out.line(3, "<geometry>" + geometry(shape, options) + "</geometry>");
    if (visual && shape.rgba != std::array<float, 4>{}) {
        out.line(3, "<material name=\"" + escape(colours.nameOf(shape.rgba)) + "\"/>");
    }
    out.line(2, std::string("</") + tag + ">");
}

void writeInertial(Out& out, const Link& link) {
    const auto mass = inertialOf(link);
    if (!mass) {
        return;
    }
    std::string open = "<inertial";
    if (link.inertial.density > 0.0) {
        std::string parts;
        const auto add = [&parts](bool on, const char* word) {
            if (on) {
                parts += (parts.empty() ? "" : " ") + std::string(word);
            }
        };
        add(link.inertial.mass.has_value(), "mass");
        add(link.inertial.com.has_value(), "com");
        add(link.inertial.inertia.has_value(), "inertia");
        open += std::string(" ") + kOverridesAttribute + "=\"" + parts + "\"";
    }
    out.line(2, open + ">");
    out.line(3, "<origin xyz=\"" + joined(mass->com) + "\"/>");
    out.line(3, "<mass value=\"" + num(mass->mass) + "\"/>");
    const auto& i = mass->inertia;
    out.line(3, "<inertia ixx=\"" + num(i[0]) + "\" ixy=\"" + num(i[1]) + "\" ixz=\"" + num(i[2]) +
                    "\" iyy=\"" + num(i[3]) + "\" iyz=\"" + num(i[4]) + "\" izz=\"" + num(i[5]) +
                    "\"/>");
    out.line(2, "</inertial>");
}

void writeLink(Out& out, const Link& link, const WriteOptions& options, Colours& colours) {
    std::string open = "<link name=\"" + escape(link.name) + "\"";
    if (link.inertial.density > 0.0) {
        open += std::string(" ") + kDensityAttribute + "=\"" + num(link.inertial.density) + "\"";
    }
    if (link.collision != CollisionChoice::Unset) {
        open += std::string(" ") + kCollisionAttribute + "=\"" +
                std::string(toString(link.collision)) + "\"";
    }
    out.line(1, open + ">");
    const Kept kept(link.keptXml);
    std::size_t visual = 0, collision = 0;
    bool inertial = false;
    const auto emit = [&](const std::string& kind, bool all) {
        if (kind == "inertial") {
            if (!inertial) {
                writeInertial(out, link);
                inertial = true;
            }
            return;
        }
        const bool isVisual = kind == "visual";
        std::size_t& at = isVisual ? visual : collision;
        while (at < link.shapes.size()) {
            const Shape& shape = link.shapes[at++];
            if (isVisual ? shape.visual : shape.collision) {
                writeShape(out, shape, isVisual, options, colours);
                if (!all) {
                    return;
                }
            }
        }
    };
    walkKept(out, 2, kept.nodes, {"inertial", "visual", "collision"}, emit);
    if (!kept.broken.empty()) {
        out.brokenKept(2, kept.broken);
    }
    out.line(1, "</link>");
}

void writeJoint(Out& out, const Joint& joint) {
    out.line(1, "<joint name=\"" + escape(joint.name) + "\" type=\"" +
                    std::string(toString(joint.type)) + "\">");
    out.line(2, "<parent link=\"" + escape(joint.parent) + "\"/>");
    out.line(2, "<child link=\"" + escape(joint.child) + "\"/>");
    writeOrigin(out, 2, joint.origin);
    std::array<double, 3> axis = joint.axis;
    const double length = std::hypot(axis[0], axis[1], axis[2]);
    if (length > 0.0 && std::abs(length - 1.0) > 1e-12) {
        for (double& a : axis) {
            a /= length;
        }
    }
    if (isMoving(joint.type) || axis != std::array<double, 3>{1, 0, 0}) {
        out.line(2, "<axis xyz=\"" + joined(axis) + "\"/>");
    }
    std::string limit;
    const auto add = [&limit](const char* key, const std::optional<double>& value) {
        if (value) {
            limit += std::string(" ") + key + "=\"" + num(*value) + "\"";
        }
    };
    add("lower", joint.lower);
    add("upper", joint.upper);
    add("effort", joint.effort);
    add("velocity", joint.velocity);
    if (!limit.empty()) {
        out.line(2, "<limit" + limit + "/>");
    }
    if (joint.damping != 0.0 || joint.friction != 0.0) {
        out.line(2, "<dynamics damping=\"" + num(joint.damping) + "\" friction=\"" +
                        num(joint.friction) + "\"/>");
    }
    if (joint.mimic) {
        const auto& [leader, multiplier, offset] = *joint.mimic;
        out.line(2, "<mimic joint=\"" + escape(leader) + "\" multiplier=\"" + num(multiplier) +
                        "\" offset=\"" + num(offset) + "\"/>");
    }
    const Kept kept(joint.keptXml);
    for (const XMLNode* node : kept.nodes) {
        if (markerKind(*node).empty()) {
            out.kept(2, *node);
        }
    }
    if (!kept.broken.empty()) {
        out.brokenKept(2, kept.broken);
    }
    out.line(1, "</joint>");
}

/// The built-in URDF sensor types the server reads itself (Robot.cpp addSensors); any other
/// type is a kind, whose parameters sit under a wrapper element named as the type.
bool builtinType(std::string_view type) {
    static const std::set<std::string_view> kTypes{
        "imu",   "ray",          "gpu_ray",     "lidar",   "gpu_lidar",     "camera",
        "depth", "depth_camera", "rgbd_camera", "thermal", "thermal_camera"};
    return kTypes.contains(type);
}

/// Elements the server never takes as a kind's parameters (Robot.cpp kNotParameters): written
/// directly under <sensor>.
bool notAParameter(std::string_view first) {
    static const std::set<std::string_view> kNot{"always_on", "visualize", "topic", "plugin",
                                                 "enable_metrics"};
    return kNot.contains(first);
}

std::vector<std::string> split(std::string_view path) {
    std::vector<std::string> out;
    while (true) {
        const auto slash = path.find('/');
        out.emplace_back(path.substr(0, slash));
        if (slash == std::string_view::npos) {
            break;
        }
        path.remove_prefix(slash + 1);
    }
    return out;
}

/// A parameter tree: children in the order first named (the map's sorted order).
struct ParamNode {
    std::string name;
    std::string text;
    std::vector<std::pair<std::string, std::string>> attributes;
    std::vector<ParamNode> children;

    ParamNode& child(const std::string& n) {
        for (ParamNode& c : children) {
            if (c.name == n) {
                return c;
            }
        }
        children.push_back({n, {}, {}, {}});
        return children.back();
    }
    void add(const std::vector<std::string>& path, std::size_t from, const std::string& value) {
        if (from + 1 == path.size() && path[from].starts_with('@')) {
            attributes.emplace_back(path[from].substr(1), value);
            return;
        }
        ParamNode& c = child(path[from]);
        if (from + 1 == path.size()) {
            c.text = value;
        } else {
            c.add(path, from + 1, value);
        }
    }
    void write(Out& out, int depth) const {
        std::string open = "<" + name;
        for (const auto& [key, value] : attributes) {
            open += " " + key + "=\"" + escape(value) + "\"";
        }
        if (children.empty()) {
            out.line(depth,
                     text.empty() ? open + "/>" : open + ">" + escape(text) + "</" + name + ">");
            return;
        }
        out.line(depth, open + ">");
        for (const ParamNode& c : children) {
            c.write(out, depth + 1);
        }
        out.line(depth, "</" + name + ">");
    }
};

void writeSensor(Out& out, const Sensor& sensor) {
    out.line(1, "<gazebo reference=\"" + escape(sensor.link) + "\">");
    out.line(2,
             "<sensor name=\"" + escape(sensor.name) + "\" type=\"" + escape(sensor.type) + "\">");
    out.line(3, "<pose>" + joined(sensor.origin.xyz) + " " + joined(sensor.origin.rpy) + "</pose>");
    out.line(3, "<update_rate>" + num(sensor.updateRate) + "</update_rate>");
    if (sensor.latency) {
        out.line(3, "<latency>" + num(*sensor.latency) + "</latency>");
    }
    if (sensor.jitter) {
        out.line(3, "<jitter>" + num(*sensor.jitter) + "</jitter>");
    }
    const bool kind = !builtinType(sensor.type);
    ParamNode top{"sensor", {}, {}, {}};
    ParamNode wrapper{sensor.type, {}, {}, {}};
    for (const auto& [key, value] : sensor.params) {
        std::vector<std::string> path = split(key);
        // "{axis}" (the catalog's IMU channels, plan R9) is written on x, y and z alike.
        std::vector<std::vector<std::string>> paths{path};
        for (std::size_t i = 0; i < path.size(); ++i) {
            if (path[i] == "{axis}") {
                paths.clear();
                for (const char* axis : {"x", "y", "z"}) {
                    path[i] = axis;
                    paths.push_back(path);
                }
                break;
            }
        }
        for (const auto& p : paths) {
            if (kind && !notAParameter(p.front())) {
                wrapper.add(p, 0, value);
            } else {
                top.add(p, 0, value);
            }
        }
    }
    for (const ParamNode& c : top.children) {
        c.write(out, 3);
    }
    if (!wrapper.children.empty() || !wrapper.attributes.empty()) {
        wrapper.write(out, 3);
    }
    out.line(2, "</sensor>");
    out.line(1, "</gazebo>");
}

// --- reader ---------------------------------------------------------------------------------

class Notes {
public:
    void add(const std::string& text) {
        if (std::ranges::find(lines_, text) == lines_.end()) {
            lines_.push_back(text);
        }
    }
    void count(const std::string& what, const std::string& item) { counted_[what].push_back(item); }
    std::vector<std::string> take() {
        for (const auto& [what, items] : counted_) {
            std::string list;
            for (std::size_t i = 0; i < items.size() && i < 4; ++i) {
                list += (i == 0 ? "" : ", ") + items[i];
            }
            if (items.size() > 4) {
                list += ", ... (" + std::to_string(items.size()) + " in all)";
            }
            lines_.push_back(what + ": " + list);
        }
        return std::move(lines_);
    }

private:
    std::vector<std::string> lines_;
    std::map<std::string, std::vector<std::string>, std::less<>> counted_;
};

Pose readOrigin(const XMLElement* parent) {
    Pose pose;
    const XMLElement* origin = parent != nullptr ? parent->FirstChildElement("origin") : nullptr;
    if (origin == nullptr) {
        return pose;
    }
    if (const auto xyz = fixedNumbers<3>(origin->Attribute("xyz"))) {
        pose.xyz = *xyz;
    }
    if (const auto rpy = fixedNumbers<3>(origin->Attribute("rpy"))) {
        pose.rpy = *rpy;
    }
    return pose;
}

/// The geometry of a <visual> or <collision>; nullopt when it is none this builder models.
std::optional<Shape> readGeometry(const XMLElement& element) {
    const XMLElement* geometry = element.FirstChildElement("geometry");
    const XMLElement* g = geometry != nullptr ? geometry->FirstChildElement() : nullptr;
    if (g == nullptr) {
        return std::nullopt;
    }
    Shape shape;
    shape.origin = readOrigin(&element);
    const std::string_view kind = g->Name();
    if (kind == "box") {
        const auto size = fixedNumbers<3>(g->Attribute("size"));
        if (!size) {
            return std::nullopt;
        }
        shape.kind = Shape::Kind::Box;
        shape.size = *size;
    } else if (kind == "cylinder") {
        const auto r = number(g->Attribute("radius"));
        const auto l = number(g->Attribute("length"));
        if (!r || !l) {
            return std::nullopt;
        }
        shape.kind = Shape::Kind::Cylinder;
        shape.size = {*r, *l, 0};
    } else if (kind == "sphere") {
        const auto r = number(g->Attribute("radius"));
        if (!r) {
            return std::nullopt;
        }
        shape.kind = Shape::Kind::Sphere;
        shape.size = {*r, 0, 0};
    } else if (kind == "capsule") {
        const auto r = number(g->Attribute("radius"));
        const auto l = number(g->Attribute("length"));
        if (!r || !l) {
            return std::nullopt;
        }
        shape.kind = Shape::Kind::Capsule;
        shape.size = {*r, *l, 0};
    } else if (kind == "mesh") {
        if (g->Attribute("filename") == nullptr) {
            return std::nullopt;
        }
        shape.kind = Shape::Kind::Mesh;
        shape.mesh = g->Attribute("filename");
        if (const auto scale = fixedNumbers<3>(g->Attribute("scale"))) {
            shape.scale = *scale;
        }
    } else {
        return std::nullopt;
    }
    return shape;
}

bool sameGeometry(const Shape& a, const Shape& b) {
    return a.kind == b.kind && a.size == b.size && a.mesh == b.mesh && a.scale == b.scale &&
           a.origin.xyz == b.origin.xyz && a.origin.rpy == b.origin.rpy;
}

using MaterialTable = std::map<std::string, std::array<float, 4>, std::less<>>;

std::optional<std::array<float, 4>> colourOf(const XMLElement* material) {
    const XMLElement* colour = material != nullptr ? material->FirstChildElement("color") : nullptr;
    const auto rgba = fixedNumbers<4>(colour != nullptr ? colour->Attribute("rgba") : nullptr);
    if (!rgba) {
        return std::nullopt;
    }
    return std::array<float, 4>{static_cast<float>((*rgba)[0]), static_cast<float>((*rgba)[1]),
                                static_cast<float>((*rgba)[2]), static_cast<float>((*rgba)[3])};
}

struct LinkReader {
    MaterialTable& materials;
    std::vector<std::string>& hoisted;  // local <material> definitions, made robot-level
    Notes& notes;

    Link read(const XMLElement& element) {
        Link link;
        link.name = element.Attribute("name") != nullptr ? element.Attribute("name") : "";
        if (const auto density = number(element.Attribute(kDensityAttribute))) {
            link.inertial.density = *density;
        }
        if (const char* choice = element.Attribute(kCollisionAttribute)) {
            if (const auto parsed = parseCollisionChoice(choice)) {
                link.collision = *parsed;
            } else {
                notes.count("an unknown lagrange_collision is dropped", link.name);
            }
        }
        std::vector<std::string> kept;
        std::optional<std::size_t> lastVisual, lastCollision;  // shape indices, for merging
        bool inertialSeen = false;
        for (const XMLNode* node = element.FirstChild(); node != nullptr;
             node = node->NextSibling()) {
            const XMLElement* child = node->ToElement();
            const std::string_view name = child != nullptr ? child->Name() : "";
            if (name == "inertial" && !inertialSeen) {
                readInertial(*child, link);
                inertialSeen = true;
                kept.push_back(marker("inertial"));
            } else if (name == "visual" || name == "collision") {
                const bool visual = name == "visual";
                std::optional<Shape> shape = readGeometry(*child);
                if (!shape) {
                    notes.count("kept as written (a geometry the builder does not edit)",
                                link.name);
                    kept.push_back(printed(*child, 0));
                    continue;
                }
                if (child->Attribute("name") != nullptr) {
                    notes.count("visual and collision names are not kept", link.name);
                }
                std::optional<bool> solid;
                bool flag = false;
                if (child->QueryBoolAttribute(kSolidAttribute, &flag) == tinyxml2::XML_SUCCESS) {
                    solid = flag;
                }
                if (visual) {
                    shape->rgba = colour(*child, link.name);
                }
                placeShape(link, *shape, visual, solid, lastVisual, lastCollision);
                kept.push_back(marker(visual ? "visual" : "collision"));
            } else if (node->ToText() != nullptr) {
                continue;  // stray text
            } else {
                kept.push_back(printed(*node, 0));
            }
        }
        link.keptXml = joinedKept(kept);
        return link;
    }

    std::array<float, 4> colour(const XMLElement& visual, const std::string& link) {
        const XMLElement* material = visual.FirstChildElement("material");
        if (material == nullptr) {
            return {};
        }
        if (material->FirstChildElement("texture") != nullptr) {
            notes.count("textures are not kept (the colour is)", link);
        }
        const char* name = material->Attribute("name");
        if (name != nullptr) {
            if (const auto it = materials.find(name); it != materials.end()) {
                return it->second;  // urdfdom: a defined name wins over a local colour
            }
        }
        const auto rgba = colourOf(material);
        if (!rgba) {
            notes.count("material without a colour or an unknown name", link);
            return {};
        }
        if (name != nullptr && *name != '\0') {
            materials.emplace(name, *rgba);
            XMLDocument scratch;
            XMLElement* definition = scratch.NewElement("material");
            definition->SetAttribute("name", name);
            XMLElement* colour = scratch.NewElement("color");
            colour->SetAttribute("rgba", joined(*rgba).c_str());
            definition->InsertEndChild(colour);
            scratch.InsertEndChild(definition);
            hoisted.push_back(printed(*definition, 0));
        }
        return *rgba;
    }

    /// Appends a shape, or merges it into an earlier one of the other kind with the same
    /// geometry and origin (keeping the visuals' and the collisions' orders as read).
    static void placeShape(Link& link, Shape shape, bool visual, std::optional<bool> solid,
                           std::optional<std::size_t>& lastVisual,
                           std::optional<std::size_t>& lastCollision) {
        std::optional<std::size_t>& mine = visual ? lastVisual : lastCollision;
        const std::size_t from = mine ? *mine + 1 : 0;
        for (std::size_t i = from; i < link.shapes.size(); ++i) {
            Shape& other = link.shapes[i];
            const bool otherKind =
                visual ? (other.collision && !other.visual) : (other.visual && !other.collision);
            if (otherKind && sameGeometry(other, shape) &&
                (!solid || *solid == other.solid || !visual)) {
                if (visual) {
                    other.visual = true;
                    other.rgba = shape.rgba;
                } else {
                    other.collision = true;
                    other.solid = solid.value_or(true);
                }
                mine = i;
                return;
            }
        }
        shape.visual = visual;
        shape.collision = !visual;
        shape.solid = solid.value_or(!visual);
        link.shapes.push_back(std::move(shape));
        mine = link.shapes.size() - 1;
    }

    void readInertial(const XMLElement& element, Link& link) {
        const Pose origin = readOrigin(&element);
        std::optional<double> mass;
        if (const XMLElement* m = element.FirstChildElement("mass")) {
            mass = number(m->Attribute("value"));
        }
        std::optional<std::array<double, 6>> inertia;
        if (const XMLElement* i = element.FirstChildElement("inertia")) {
            std::array<double, 6> v{};
            const std::array<const char*, 6> keys{"ixx", "ixy", "ixz", "iyy", "iyz", "izz"};
            for (std::size_t k = 0; k < keys.size(); ++k) {
                v[k] = number(i->Attribute(keys[k])).value_or(0.0);
            }
            if (origin.rpy != std::array<double, 3>{}) {
                v = packed(rotated(rotation(origin.rpy), tensor(v)));
                notes.count("<inertial> turned into the link's axes", link.name);
            }
            inertia = v;
        }
        // Which parts are overrides: the builder's own list, else all (RFC §10).
        bool all = true;
        std::set<std::string, std::less<>> listed;
        if (link.inertial.density > 0.0) {
            if (const char* list = element.Attribute(kOverridesAttribute)) {
                all = false;
                std::string_view rest(list);
                while (!(rest = trim(rest)).empty()) {
                    const auto end = std::min(rest.find(' '), rest.size());
                    listed.emplace(rest.substr(0, end));
                    rest.remove_prefix(end);
                }
            }
        }
        const auto wanted = [&](const char* part) { return all || listed.contains(part); };
        if (wanted("mass")) {
            link.inertial.mass = mass;
        }
        if (wanted("com")) {
            link.inertial.com = origin.xyz;
        }
        if (wanted("inertia")) {
            link.inertial.inertia = inertia;
        }
        if (all) {
            notes.count("<inertial> kept as an override until you compute it", link.name);
        }
    }
};

/// Every leaf under `element` by its path; attributes as "<path>/@name".
void readParams(const XMLElement& element, const std::string& prefix, Sensor& sensor,
                Notes& notes) {
    for (const tinyxml2::XMLAttribute* a = element.FirstAttribute(); a != nullptr; a = a->Next()) {
        const std::string path = (prefix.empty() ? "" : prefix + "/") + "@" + a->Name();
        sensor.params.insert_or_assign(path, a->Value());
    }
    for (const XMLNode* node = element.FirstChild(); node != nullptr; node = node->NextSibling()) {
        if (node->ToComment() != nullptr) {
            notes.count("comments inside a <sensor> are not kept", sensor.name);
            continue;
        }
        const XMLElement* child = node->ToElement();
        if (child == nullptr) {
            continue;
        }
        const std::string path = (prefix.empty() ? "" : prefix + "/") + child->Name();
        const bool leaf = child->FirstChildElement() == nullptr;
        if (leaf) {
            if (sensor.params.contains(path)) {
                notes.count("a repeated element of a sensor is kept once", sensor.name);
            }
            const char* text = child->GetText();
            sensor.params.insert_or_assign(path, text != nullptr ? std::string(trim(text)) : "");
            for (const tinyxml2::XMLAttribute* a = child->FirstAttribute(); a != nullptr;
                 a = a->Next()) {
                sensor.params.insert_or_assign(path + "/@" + a->Name(), a->Value());
            }
        } else {
            readParams(*child, path, sensor, notes);
        }
    }
}

/// x, y and z with the same parameters become the catalog's "{axis}" (plan R9).
void collapseAxes(Sensor& sensor) {
    std::map<std::string, std::string, std::less<>> collapsed;
    std::set<std::string, std::less<>> used;
    for (const auto& [path, value] : sensor.params) {
        const auto at = path.find("/x/");
        if (at == std::string::npos) {
            continue;
        }
        const std::string head = path.substr(0, at), tail = path.substr(at + 3);
        bool same = true;
        for (const char* axis : {"y", "z"}) {
            const auto it = sensor.params.find(head + "/" + axis + "/" + tail);
            same = same && it != sensor.params.end() && it->second == value;
        }
        if (!same) {
            continue;
        }
        // Collapse only a whole channel: every key under x has its twin under y and z, and
        // they have no others.
        const auto subtree = [&](const char* axis) {
            std::map<std::string, std::string, std::less<>> out;
            const std::string start = head + "/" + axis + "/";
            for (const auto& [p, v] : sensor.params) {
                if (p.starts_with(start)) {
                    out.emplace(p.substr(start.size()), v);
                }
            }
            return out;
        };
        const auto x = subtree("x");
        if (x != subtree("y") || x != subtree("z")) {
            continue;
        }
        collapsed.emplace(head + "/{axis}/" + tail, value);
        for (const char* axis : {"x", "y", "z"}) {
            used.insert(head + "/" + axis + "/" + tail);
        }
    }
    for (const std::string& path : used) {
        sensor.params.erase(path);
    }
    sensor.params.merge(collapsed);
}

std::optional<Sensor> readSensor(const XMLElement& element, const std::string& link, Notes& notes) {
    const char* name = element.Attribute("name");
    const char* type = element.Attribute("type");
    if (name == nullptr || type == nullptr) {
        return std::nullopt;
    }
    Sensor sensor;
    sensor.name = name;
    sensor.type = type;
    sensor.link = link;
    const bool kind = !builtinType(sensor.type);
    for (const XMLNode* node = element.FirstChild(); node != nullptr; node = node->NextSibling()) {
        if (node->ToComment() != nullptr) {
            notes.count("comments inside a <sensor> are not kept", sensor.name);
            continue;
        }
        const XMLElement* child = node->ToElement();
        if (child == nullptr) {
            continue;
        }
        const std::string_view tag = child->Name();
        if (tag == "pose") {
            if (const auto v = fixedNumbers<6>(child->GetText())) {
                sensor.origin.xyz = {(*v)[0], (*v)[1], (*v)[2]};
                sensor.origin.rpy = {(*v)[3], (*v)[4], (*v)[5]};
            }
        } else if (tag == "update_rate") {
            sensor.updateRate = number(child->GetText()).value_or(kDefaultUpdateRate);
        } else if (tag == "latency") {
            sensor.latency = number(child->GetText());
        } else if (tag == "jitter") {
            sensor.jitter = number(child->GetText());
        } else if (kind && tag == sensor.type) {
            readParams(*child, "", sensor, notes);  // the kind's wrapper: not in the path
        } else {
            const std::string path(tag);
            if (child->FirstChildElement() == nullptr) {
                const char* text = child->GetText();
                sensor.params.insert_or_assign(path,
                                               text != nullptr ? std::string(trim(text)) : "");
                for (const tinyxml2::XMLAttribute* a = child->FirstAttribute(); a != nullptr;
                     a = a->Next()) {
                    sensor.params.insert_or_assign(path + "/@" + a->Name(), a->Value());
                }
            } else {
                readParams(*child, path, sensor, notes);
            }
        }
    }
    if (sensor.type == "imu") {
        collapseAxes(sensor);
    }
    return sensor;
}

Joint readJoint(const XMLElement& element, Joint::Type type) {
    Joint joint;
    joint.type = type;
    joint.name = element.Attribute("name") != nullptr ? element.Attribute("name") : "";
    std::vector<std::string> kept;
    for (const XMLNode* node = element.FirstChild(); node != nullptr; node = node->NextSibling()) {
        const XMLElement* child = node->ToElement();
        const std::string_view tag = child != nullptr ? child->Name() : "";
        if (tag == "parent" || tag == "child") {
            const char* link = child->Attribute("link");
            (tag == "parent" ? joint.parent : joint.child) = link != nullptr ? link : "";
        } else if (tag == "origin") {
            joint.origin = readOrigin(&element);
        } else if (tag == "axis") {
            if (const auto xyz = fixedNumbers<3>(child->Attribute("xyz"))) {
                joint.axis = *xyz;
            }
        } else if (tag == "limit") {
            joint.lower = number(child->Attribute("lower"));
            joint.upper = number(child->Attribute("upper"));
            joint.effort = number(child->Attribute("effort"));
            joint.velocity = number(child->Attribute("velocity"));
        } else if (tag == "dynamics") {
            joint.damping = number(child->Attribute("damping")).value_or(0.0);
            joint.friction = number(child->Attribute("friction")).value_or(0.0);
        } else if (tag == "mimic" && child->Attribute("joint") != nullptr) {
            joint.mimic = std::tuple<std::string, double, double>{
                child->Attribute("joint"), number(child->Attribute("multiplier")).value_or(1.0),
                number(child->Attribute("offset")).value_or(0.0)};
        } else if (node->ToText() == nullptr) {
            kept.push_back(printed(*node, 0));
        }
    }
    joint.keptXml = joinedKept(kept);
    return joint;
}

bool looksLikeXacro(std::string_view text) {
    return text.find("xmlns:xacro") != std::string_view::npos ||
           text.find("<xacro:") != std::string_view::npos;
}

}  // namespace

std::string keptForDisplay(std::string_view keptXml) {
    const std::string text(keptXml);
    XMLDocument xml;
    if (xml.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS) {
        return text;
    }
    std::vector<std::string> pieces;
    for (const XMLNode* n = xml.FirstChild(); n != nullptr; n = n->NextSibling()) {
        if (markerKind(*n).empty()) {
            pieces.push_back(printed(*n, 0));
        }
    }
    return joinedKept(pieces);
}

std::string writeUrdf(const Document& whole, const WriteOptions& options) {
    // GNSS, magnetometer, barometer and radar go to the overlay file (writeOverlay), never here:
    // the server reads no such <sensor> from URDF (RFC 0016 phase 3).
    Document filtered;
    const bool overlaySensors =
        std::ranges::any_of(whole.sensors, [](const Sensor& s) { return isOverlaySensor(s.type); });
    if (overlaySensors) {
        filtered = whole;
        std::erase_if(filtered.sensors, [](const Sensor& s) { return isOverlaySensor(s.type); });
    }
    const Document& document = overlaySensors ? filtered : whole;
    Out out;
    out.line(0, "<?xml version=\"1.0\"?>");
    const Kept kept(document.keptXml);
    // Before the robot marker: what stood before <robot> (comments).
    std::vector<const XMLNode*> body = kept.nodes;
    if (const auto at = std::ranges::find_if(
            kept.nodes, [](const XMLNode* n) { return markerKind(*n) == "robot"; });
        at != kept.nodes.end()) {
        for (auto it = kept.nodes.begin(); it != at; ++it) {
            out.kept(0, **it);
        }
        body.assign(at + 1, kept.nodes.end());
    }
    // A capsule is URDF 1.1: urdfdom ignores it in a 1.0 file (Addendum A), so the version is
    // written exactly when one is used.
    const bool capsule = std::ranges::any_of(document.links, [](const Link& link) {
        return std::ranges::any_of(
            link.shapes, [](const Shape& shape) { return shape.kind == Shape::Kind::Capsule; });
    });
    out.line(0, "<robot name=\"" + escape(document.robot) + "\"" +
                    (capsule ? " version=\"1.1\"" : "") + ">");
    Colours colours(kept);
    // The body is written into its own buffer first: colours the shapes name for the first
    // time are defined at the top, before it.
    Out inner;
    std::size_t link = 0, joint = 0, sensor = 0;
    const auto emit = [&](const std::string& kind, bool all) {
        do {
            if (kind == "link" && link < document.links.size()) {
                writeLink(inner, document.links[link++], options, colours);
            } else if (kind == "joint" && joint < document.joints.size()) {
                writeJoint(inner, document.joints[joint++]);
            } else if (kind == "sensor" && sensor < document.sensors.size()) {
                writeSensor(inner, document.sensors[sensor++]);
            } else {
                return;
            }
        } while (all);
    };
    walkKept(inner, 1, body, {"link", "joint", "sensor"}, emit);
    if (!kept.broken.empty()) {
        inner.brokenKept(1, kept.broken);
    }
    for (const auto& [name, rgba] : colours.added()) {
        XMLDocument scratch;
        XMLElement* definition = scratch.NewElement("material");
        definition->SetAttribute("name", name.c_str());
        XMLElement* colour = scratch.NewElement("color");
        colour->SetAttribute("rgba", joined(rgba).c_str());
        definition->InsertEndChild(colour);
        scratch.InsertEndChild(definition);
        out.kept(1, *definition);
    }
    std::string text = out.take() + inner.take();
    text += "</robot>\n";
    return text;
}

ReadResult readUrdf(std::string_view text) {
    ReadResult result;
    if (looksLikeXacro(text)) {
        result.error = "this is xacro: expand it first (xacro FILE.xacro > FILE.urdf)";
        return result;
    }
    XMLDocument xml;
    const std::string copy(text);
    if (xml.Parse(copy.c_str(), copy.size()) != tinyxml2::XML_SUCCESS) {
        result.errorLine = xml.ErrorLineNum();
        result.error = std::string("not well-formed XML at line ") +
                       std::to_string(xml.ErrorLineNum()) + " (" + xml.ErrorName() + ")";
        return result;
    }
    Notes notes;
    std::vector<std::string> prolog;
    const XMLElement* robot = nullptr;
    for (const XMLNode* node = xml.FirstChild(); node != nullptr; node = node->NextSibling()) {
        if (const XMLElement* e = node->ToElement()) {
            if (std::string_view(e->Name()) == "robot" && robot == nullptr) {
                robot = e;
                continue;
            }
            result.errorLine = e->GetLineNum();
            result.error = "line " + std::to_string(e->GetLineNum()) + ": <" + e->Name() +
                           "> where a URDF has only <robot>";
            return result;
        }
        if (node->ToComment() != nullptr) {
            if (robot == nullptr) {
                prolog.push_back(printed(*node, 0));
            } else {
                notes.add("comments after </robot> are not kept");
            }
        }
    }
    if (robot == nullptr) {
        result.error = "no <robot> element";
        return result;
    }
    Document document;
    if (const char* name = robot->Attribute("name"); name != nullptr) {
        document.robot = name;
    } else {
        notes.add("the <robot> has no name");
    }
    for (const tinyxml2::XMLAttribute* a = robot->FirstAttribute(); a != nullptr; a = a->Next()) {
        // The version is written again from what the document uses (1.1 for a capsule).
        if (std::string_view(a->Name()) != "name" && std::string_view(a->Name()) != "version") {
            notes.add(std::string("attribute ") + a->Name() + " of <robot> is not kept");
        }
    }
    MaterialTable materials;
    for (const XMLElement* m = robot->FirstChildElement("material"); m != nullptr;
         m = m->NextSiblingElement("material")) {
        if (const char* name = m->Attribute("name"); name != nullptr) {
            if (const auto rgba = colourOf(m)) {
                materials.emplace(name, *rgba);
            }
        }
    }
    std::vector<std::string> hoisted;
    LinkReader links{materials, hoisted, notes};
    std::vector<std::string> body;
    for (const XMLNode* node = robot->FirstChild(); node != nullptr; node = node->NextSibling()) {
        const XMLElement* e = node->ToElement();
        const std::string_view tag = e != nullptr ? e->Name() : "";
        if (tag == "link") {
            document.links.push_back(links.read(*e));
            body.push_back(marker("link"));
        } else if (tag == "joint") {
            const char* type = e->Attribute("type");
            const auto parsed = parseJointType(type != nullptr ? type : "");
            if (!parsed) {
                notes.add(std::string("joint '") +
                          (e->Attribute("name") ? e->Attribute("name") : "") + "' of type '" +
                          (type ? type : "") + "' is kept as written (not edited)");
                body.push_back(printed(*node, 0));
                continue;
            }
            document.joints.push_back(readJoint(*e, *parsed));
            body.push_back(marker("joint"));
        } else if (tag == "gazebo" && e->Attribute("reference") != nullptr &&
                   e->FirstChildElement("sensor") != nullptr) {
            // Sensors are taken out; the block's other children stay as written.
            const std::string reference = e->Attribute("reference");
            XMLDocument scratch;
            XMLNode* rest = e->DeepClone(&scratch);
            std::vector<XMLElement*> sensors;
            for (XMLElement* s = rest->FirstChildElement("sensor"); s != nullptr;
                 s = s->NextSiblingElement("sensor")) {
                sensors.push_back(s);
            }
            int taken = 0;
            for (XMLElement* s : sensors) {
                if (auto sensor = readSensor(*s, reference, notes)) {
                    document.sensors.push_back(std::move(*sensor));
                    rest->DeleteChild(s);
                    ++taken;
                }
            }
            if (rest->FirstChild() != nullptr) {
                body.push_back(printed(*rest, 0));
            }
            for (int k = 0; k < taken; ++k) {
                body.push_back(marker("sensor"));
            }
        } else if (node->ToText() != nullptr) {
            continue;
        } else {
            body.push_back(printed(*node, 0));
        }
    }
    // Local <material> definitions become robot-level ones: after the file's own definitions
    // that come before the first link, else just before it.
    const auto firstLink = std::ranges::find(body, marker("link"));
    auto at = firstLink;
    for (auto it = body.begin(); it != firstLink; ++it) {
        if (it->starts_with("<material")) {
            at = std::next(it);
        }
    }
    body.insert(at, hoisted.begin(), hoisted.end());
    if (!prolog.empty()) {
        prolog.push_back(marker("robot"));
        body.insert(body.begin(), prolog.begin(), prolog.end());
    }
    document.keptXml = joinedKept(body);
    result.document = std::move(document);
    result.notes = notes.take();
    return result;
}

std::string saveAtomically(const std::filesystem::path& requested, std::string_view text) {
    if (requested.extension() == ".xacro") {
        return "cannot save " + requested.string() + ": a .xacro is never written (save as .urdf)";
    }
    // A symlink is written through to its target (a --symlink-install share/ file points at the
    // package's source: that is where the edit belongs), never replaced by a file of its own.
    std::filesystem::path path = requested;
    std::error_code error;
    if (std::filesystem::is_symlink(requested, error)) {
        path = std::filesystem::weakly_canonical(requested, error);
        if (error || path.empty()) {
            return "cannot save " + requested.string() + ": the link cannot be followed";
        }
        if (path.extension() == ".xacro") {
            return "cannot save " + requested.string() + ": it links to a .xacro (" +
                   path.string() + "), which is never written";
        }
    }
    // An existing file keeps its permissions; a read-only one is refused, not replaced.
    mode_t mode = 0644;
    if (struct stat existing{}; ::stat(path.c_str(), &existing) == 0) {
        if (!S_ISREG(existing.st_mode)) {
            return "cannot save " + requested.string() + ": it is not a regular file";
        }
        if (::access(path.c_str(), W_OK) != 0) {
            return "cannot save " + requested.string() +
                   ": the file is read-only (use Save as… for a copy)";
        }
        mode = existing.st_mode & 07777;
    }
    const std::filesystem::path temporary =
        path.parent_path() / ("." + path.filename().string() + ".tmp" + std::to_string(getpid()));
    const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
        return "cannot save " + path.string() + ": cannot create a file in its folder";
    }
    (void)::fchmod(fd, mode);  // the old file's mode (0644 for a new one), not the umask's
    std::size_t written = 0;
    while (written < text.size()) {
        const ssize_t n = ::write(fd, text.data() + written, text.size() - written);
        if (n <= 0) {
            ::close(fd);
            std::filesystem::remove(temporary);
            return "cannot save " + path.string() + ": the write failed";
        }
        written += static_cast<std::size_t>(n);
    }
    if (::fsync(fd) != 0 || ::close(fd) != 0) {
        std::filesystem::remove(temporary);
        return "cannot save " + path.string() + ": the data did not reach the disk";
    }
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary);
        return "cannot save " + path.string() + ": " + error.message();
    }
    return {};
}

std::string safeFileName(std::string_view name) {
    constexpr std::size_t kLongest = 100;  // well under NAME_MAX (255) with any suffix
    std::string out;
    for (const char c : name.substr(0, kLongest)) {
        const bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        out += plain ? c : '_';
    }
    // No hidden file, and never "." or "..".
    while (!out.empty() && out.front() == '.') {
        out.front() = '_';
    }
    return out.empty() ? std::string("robot") : out;
}

}  // namespace lagrange::viewer::builder
