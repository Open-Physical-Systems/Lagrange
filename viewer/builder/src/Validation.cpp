// The builder's own, live validation (RFC 0016 §8.1). One rule per item of §8.1, each with a
// message that says what is wrong and what to do. The server's loader stays the judge (§8.2).
#include "TreeCheck.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <sstream>
#include <system_error>

namespace lagrange::viewer::builder {

namespace {

std::string inQuotes(std::string_view name) {
    return "'" + std::string(name) + "'";
}

std::string number(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

class Findings {
public:
    void add(Level level, std::string_view item, std::string message) {
        problems_.push_back(Problem{level, std::string(item), std::move(message)});
    }
    void error(std::string_view item, std::string message) {
        add(Level::Error, item, std::move(message));
    }
    void warning(std::string_view item, std::string message) {
        add(Level::Warning, item, std::move(message));
    }
    void hint(std::string_view item, std::string message) {
        add(Level::Hint, item, std::move(message));
    }
    void append(const std::vector<Problem>& more) {
        problems_.insert(problems_.end(), more.begin(), more.end());
    }
    std::vector<Problem> take() {
        // Errors, then warnings, then hints; document order within a level.
        std::ranges::stable_sort(problems_, {}, &Problem::level);
        return std::move(problems_);
    }

private:
    std::vector<Problem> problems_;
};

// --- names ---------------------------------------------------------------------------------

template <typename Items>
void checkNames(Findings& findings, const Items& items, std::string_view what) {
    std::map<std::string_view, int> seen;
    for (const auto& item : items) {
        if (!validName(item.name)) {
            findings.error(item.name,
                           std::string(what) + " name " + inQuotes(item.name) +
                               " is not valid: use letters, digits and '_', not starting "
                               "with a digit (the server's rule)");
        }
        if (++seen[item.name] == 2) {
            findings.error(item.name, "two " + std::string(what) + "s are named " +
                                          inQuotes(item.name) + "; rename one");
        }
    }
}

// --- joints --------------------------------------------------------------------------------

void checkJoint(Findings& findings, const Document& document, const Joint& joint) {
    const std::string name = inQuotes(joint.name);
    const bool limited =
        joint.type == Joint::Type::Revolute || joint.type == Joint::Type::Prismatic;
    if (limited) {
        // URDF requires <limit> for these; an empty field is an error, never a made-up default.
        const std::string_view type = toString(joint.type);
        const auto required = [&](const std::optional<double>& value, std::string_view field) {
            if (!value) {
                findings.error(joint.name, "joint " + name + " is " + std::string(type) +
                                               " and has no " + std::string(field) +
                                               " limit; URDF requires one, type it in");
            }
        };
        required(joint.lower, "lower");
        required(joint.upper, "upper");
        required(joint.effort, "effort");
        required(joint.velocity, "velocity");
        if (joint.lower && joint.upper && !(*joint.lower < *joint.upper)) {
            findings.error(joint.name, "joint " + name + ": the lower limit " +
                                           number(*joint.lower) + " is not below the upper limit " +
                                           number(*joint.upper) + "; make lower < upper");
        }
    }
    if (joint.effort && !(*joint.effort > 0.0)) {
        findings.error(joint.name, "joint " + name + ": effort " + number(*joint.effort) +
                                       " must be greater than 0");
    }
    if (joint.velocity && !(*joint.velocity > 0.0)) {
        findings.error(joint.name, "joint " + name + ": velocity " + number(*joint.velocity) +
                                       " must be greater than 0");
    }
    if (isMoving(joint.type)) {
        const auto& a = joint.axis;
        if (!(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] > 0.0)) {
            findings.error(joint.name,
                           "joint " + name + " has a zero axis; give it a direction (1 0 0, ...)");
        }
    }
    if (joint.damping < 0.0 || joint.friction < 0.0) {
        findings.error(joint.name, "joint " + name + ": damping and friction cannot be negative");
    }
    if (joint.mimic) {
        const std::string& leader = std::get<0>(*joint.mimic);
        const Joint* found = findJoint(document, leader);
        if (found == nullptr) {
            findings.error(joint.name, "joint " + name + " mimics " + inQuotes(leader) +
                                           ", which does not exist; pick a joint to follow");
        } else if (found == &joint || leader == joint.name) {
            findings.error(joint.name, "joint " + name + " mimics itself; pick another joint");
        } else if (!isMoving(found->type)) {
            findings.error(joint.name, "joint " + name + " mimics " + inQuotes(leader) +
                                           ", a fixed joint that never moves; pick a moving one");
        }
        if (!isMoving(joint.type)) {
            findings.warning(joint.name, "joint " + name +
                                             " is fixed, so its mimic does nothing; remove it "
                                             "or make the joint move");
        }
    }
}

// --- links: shapes and mass ----------------------------------------------------------------

/// The joint whose child the link is, or nullptr.
const Joint* parentJointOf(const Document& document, std::string_view link) {
    const auto it = std::ranges::find_if(
        document.joints, [link](const Joint& joint) { return joint.child == link; });
    return it == document.joints.end() ? nullptr : &*it;
}

void checkShapes(Findings& findings, const Link& link) {
    for (std::size_t i = 0; i < link.shapes.size(); ++i) {
        const Shape& shape = link.shapes[i];
        const std::string where = "link " + inQuotes(link.name) + ", shape " +
                                  std::to_string(i + 1) + " (" + std::string(toString(shape.kind)) +
                                  ")";
        bool bad = false;
        switch (shape.kind) {
            case Shape::Kind::Box:
                bad = !(shape.size[0] > 0.0 && shape.size[1] > 0.0 && shape.size[2] > 0.0);
                break;
            case Shape::Kind::Cylinder:
                bad = !(shape.size[0] > 0.0 && shape.size[1] > 0.0);
                break;
            case Shape::Kind::Sphere:
                bad = !(shape.size[0] > 0.0);
                break;
            case Shape::Kind::Mesh:
                bad = shape.scale[0] == 0.0 || shape.scale[1] == 0.0 || shape.scale[2] == 0.0;
                break;
        }
        if (bad) {
            findings.error(link.name,
                           where + " has a size or scale of zero or less; give it a size");
        }
    }
}

/// The root link of a robot fixed to the ground, by the URDF convention.
constexpr std::string_view kWorldLink = "world";

void checkMass(Findings& findings, const Document& document, const Link& link) {
    const std::string name = inQuotes(link.name);
    if (link.inertial.mass && !(*link.inertial.mass > 0.0)) {
        findings.error(link.name, "link " + name + ": the mass " + number(*link.inertial.mass) +
                                      " kg must be greater than 0");
        return;
    }
    if (link.inertial.density < 0.0) {
        findings.error(link.name, "link " + name + ": the density cannot be negative");
        return;
    }
    const Joint* parent = parentJointOf(document, link.name);
    if (parent == nullptr && link.name == kWorldLink && link.shapes.empty() &&
        !link.inertial.mass) {
        // The URDF convention for a robot bolted to the ground: a massless, shapeless root
        // named "world", the fixed base's parent. Nothing to simulate, nothing missing.
        return;
    }
    const std::optional<MassProperties> mass = linkMass(link);
    if (!mass || !(mass->mass > 0.0)) {
        std::string message = "link " + name + " has no mass: ";
        message += std::ranges::any_of(link.shapes, &Shape::solid)
                       ? "give it a density (a material) or a mass"
                       : "no shape is solid; mark one solid or give it a mass and inertia";
        if (parent != nullptr && parent->type == Joint::Type::Fixed) {
            // A massless link on a fixed joint (a sensor frame) merges into its parent.
            findings.warning(link.name, message);
        } else {
            findings.error(link.name, message);
        }
        return;
    }
    switch (checkInertia(mass->inertia)) {
        case InertiaCheck::Ok:
            break;
        case InertiaCheck::NotPositiveDefinite:
            findings.error(link.name, "link " + name +
                                          ": the inertia is not positive definite (a principal "
                                          "moment is zero or negative); compute it from the "
                                          "shapes or correct it");
            break;
        case InertiaCheck::TriangleInequality:
            findings.error(link.name, "link " + name +
                                          ": the inertia breaks the triangle inequality (two "
                                          "principal moments add up to less than the third), "
                                          "which no real body has; compute it from the shapes");
            break;
    }
}

/// A sphere caster on a fixed joint slides instead of rolling (RFC §8.1's hint).
void hintCaster(Findings& findings, const Document& document, const Link& link) {
    const Joint* parent = parentJointOf(document, link.name);
    if (parent == nullptr || parent->type != Joint::Type::Fixed) {
        return;
    }
    bool collides = false;
    bool allSpheres = true;
    for (const Shape& shape : link.shapes) {
        if (shape.collision) {
            collides = true;
            allSpheres = allSpheres && shape.kind == Shape::Kind::Sphere;
        }
    }
    if (collides && allSpheres) {
        findings.hint(link.name, "link " + inQuotes(link.name) +
                                     " is a sphere on a fixed joint: "
                                     "as a caster it slides on the ground rather than rolls; "
                                     "that is often fine, but give it low friction if it drags");
    }
}

// --- meshes (R12) ----------------------------------------------------------------------------

std::string environment(const char* name) {
    const char* value = std::getenv(name);  // NOLINT(concurrency-mt-unsafe): read-only use
    return value == nullptr ? std::string{} : std::string{value};
}

bool isPackage(const std::filesystem::path& directory, const std::string& package) {
    std::ifstream file(directory / "package.xml");
    if (!file) {
        return false;
    }
    const std::string text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    return text.find("<name>" + package + "</name>") != std::string::npos;
}

/// A ROS package's share folder: the ament index on $AMENT_PREFIX_PATH, else the nearest folder
/// at or above `near` that is the package or holds it (as io/meshes' resolveMeshUri does).
std::optional<std::filesystem::path> packageShare(const std::string& package,
                                                  const std::filesystem::path& near) {
    std::error_code error;
    std::istringstream prefixes(environment("AMENT_PREFIX_PATH"));
    for (std::string prefix; std::getline(prefixes, prefix, ':');) {
        if (prefix.empty()) {
            continue;
        }
        const std::filesystem::path root(prefix);
        if (std::filesystem::exists(
                root / "share" / "ament_index" / "resource_index" / "packages" / package, error)) {
            return root / "share" / package;
        }
    }
    if (near.empty()) {
        return std::nullopt;
    }
    std::filesystem::path at = std::filesystem::absolute(near, error).lexically_normal();
    while (!at.empty()) {
        if (isPackage(at, package)) {
            return at;
        }
        if (isPackage(at / package, package)) {
            return at / package;
        }
        if (at == at.parent_path()) {
            break;
        }
        at = at.parent_path();
    }
    return std::nullopt;
}

void checkMesh(Findings& findings, const Link& link, const Shape& shape,
               const std::filesystem::path& meshBase) {
    constexpr std::string_view kAsset = "lagrange-asset://";
    constexpr std::string_view kPackage = "package://";
    constexpr std::string_view kFile = "file://";
    const std::string& uri = shape.mesh;
    const std::string where = "link " + inQuotes(link.name) + ": mesh " + inQuotes(uri);
    if (uri.empty()) {
        findings.error(link.name, "link " + inQuotes(link.name) +
                                      " has a mesh shape with no file; pick a file");
        return;
    }
    std::filesystem::path path;
    if (uri.starts_with(kAsset)) {
        // $LAGRANGE_ASSET_DIR, by default $XDG_DATA_HOME/lagrange/assets
        // (~/.local/share/lagrange/assets), as the server resolves it.
        if (const std::string dir = environment("LAGRANGE_ASSET_DIR"); !dir.empty()) {
            path = dir;
        } else if (const std::string data = environment("XDG_DATA_HOME"); !data.empty()) {
            path = std::filesystem::path(data) / "lagrange" / "assets";
        } else if (const std::string home = environment("HOME"); !home.empty()) {
            path = std::filesystem::path(home) / ".local" / "share" / "lagrange" / "assets";
        }
        path /= uri.substr(kAsset.size());
        std::error_code error;
        if (!std::filesystem::is_regular_file(path, error)) {
            findings.error(link.name, where + " is not fetched (" + path.string() +
                                          "); run `ros2 run lagrange_assets fetch_assets`");
        }
        return;
    }
    if (uri.starts_with(kPackage)) {
        const std::string rest = uri.substr(kPackage.size());
        const std::size_t slash = rest.find('/');
        const std::string package = rest.substr(0, slash);
        const std::optional<std::filesystem::path> share = packageShare(package, meshBase);
        if (!share) {
            findings.error(link.name, where + ": package " + inQuotes(package) +
                                          " is not installed nor found above the robot's file");
            return;
        }
        path = *share;
        if (slash != std::string::npos) {
            path /= rest.substr(slash + 1);
        }
    } else if (uri.starts_with(kFile)) {
        path = uri.substr(kFile.size());
    } else {
        path = uri;
        if (path.is_relative()) {
            if (meshBase.empty()) {
                return;  // not saved yet: a relative mesh cannot be checked
            }
            path = meshBase / path;
        }
    }
    std::error_code error;
    if (!std::filesystem::is_regular_file(path.lexically_normal(), error)) {
        findings.error(link.name, where + " is not found (" + path.lexically_normal().string() +
                                      "); pick the file again");
    }
}

// --- sensors -------------------------------------------------------------------------------

std::optional<double> parseNumber(std::string_view text, bool integer) {
    while (!text.empty() && text.front() == ' ') {
        text.remove_prefix(1);
    }
    while (!text.empty() && text.back() == ' ') {
        text.remove_suffix(1);
    }
    if (text.empty()) {
        return std::nullopt;
    }
    if (integer) {
        long long value = 0;
        const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
        if (error != std::errc{} || end != text.data() + text.size()) {
            return std::nullopt;
        }
        return static_cast<double>(value);
    }
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

void checkSensor(Findings& findings, const Document& document, const Sensor& sensor,
                 const SensorCatalog* catalog) {
    const std::string name = inQuotes(sensor.name);
    if (findLink(document, sensor.link) == nullptr) {
        findings.error(sensor.name, "sensor " + name + " is on link " + inQuotes(sensor.link) +
                                        ", which does not exist; pick a link");
    }
    if (!(sensor.updateRate > 0.0)) {
        findings.error(sensor.name, "sensor " + name + ": the update rate " +
                                        number(sensor.updateRate) + " Hz must be greater than 0");
    }
    if ((sensor.latency && *sensor.latency < 0.0) || (sensor.jitter && *sensor.jitter < 0.0)) {
        findings.error(sensor.name, "sensor " + name + ": latency and jitter cannot be negative");
    }
    if (catalog == nullptr) {
        return;
    }
    const CatalogEntry* entry = catalog->find(sensor.type);
    if (entry == nullptr) {
        findings.warning(sensor.name, "sensor " + name + " is of type " + inQuotes(sensor.type) +
                                          ", which the sensor catalog does not know; its "
                                          "parameters are not checked (a plugin's kind?)");
        return;
    }
    findings.append(checkSensorParams(sensor, *entry));
}

}  // namespace

std::vector<Problem> checkSensorParams(const Sensor& sensor, const CatalogEntry& entry) {
    Findings findings;
    const std::string name = inQuotes(sensor.name);
    for (const auto& [path, text] : sensor.params) {
        const auto param = std::ranges::find_if(
            entry.params, [&](const CatalogParam& p) { return p.path == path; });
        const std::string where = "sensor " + name + ": " + path;
        if (param == entry.params.end()) {
            findings.warning(sensor.name, where + " is not a parameter of " + inQuotes(entry.name) +
                                              " in the catalog; the server may ignore it");
            continue;
        }
        const std::string unit = param->unit.empty() || param->unit == "1" ? "" : " " + param->unit;
        switch (param->type) {
            case CatalogParam::Type::Number:
            case CatalogParam::Type::Int: {
                const bool integer = param->type == CatalogParam::Type::Int;
                const std::optional<double> value = parseNumber(text, integer);
                if (!value) {
                    findings.error(sensor.name, where + " = " + inQuotes(text) + " is not " +
                                                    (integer ? "a whole number" : "a number"));
                    break;
                }
                if (param->min &&
                    (*value < *param->min || (param->minExclusive && *value == *param->min))) {
                    findings.error(sensor.name,
                                   where + " = " + text + unit +
                                       (param->minExclusive ? " must be more than "
                                                            : " is below the catalog's minimum ") +
                                       number(*param->min) + unit);
                }
                if (param->max &&
                    (*value > *param->max || (param->maxExclusive && *value == *param->max))) {
                    findings.error(sensor.name,
                                   where + " = " + text + unit +
                                       (param->maxExclusive ? " must be less than "
                                                            : " is above the catalog's maximum ") +
                                       number(*param->max) + unit);
                }
                break;
            }
            case CatalogParam::Type::Bool:
                if (text != "true" && text != "false" && text != "1" && text != "0") {
                    findings.error(sensor.name,
                                   where + " = " + inQuotes(text) + " is not true or false");
                }
                break;
            case CatalogParam::Type::Enum:
                if (std::ranges::find(param->options, text) == param->options.end()) {
                    std::string options;
                    for (const std::string& option : param->options) {
                        options += (options.empty() ? "" : ", ") + option;
                    }
                    findings.error(sensor.name,
                                   where + " = " + inQuotes(text) + " is not one of " + options);
                }
                break;
            case CatalogParam::Type::Text:
            case CatalogParam::Type::List:
                break;
        }
    }
    return findings.take();
}

std::vector<Problem> validate(const Document& document, const SensorCatalog* catalog,
                              const std::filesystem::path& meshBase) {
    Findings findings;
    if (!validName(document.robot)) {
        findings.error("", "robot name " + inQuotes(document.robot) +
                               " is not valid: use letters, digits and '_', not starting "
                               "with a digit (the server's rule)");
    }
    if (document.links.empty()) {
        findings.error("", "the robot has no link; add one");
    }
    checkNames(findings, document.links, "link");
    checkNames(findings, document.joints, "joint");
    checkNames(findings, document.sensors, "sensor");
    for (const detail::TreeFinding& finding : detail::analyseTree(document).findings) {
        findings.error(finding.item, finding.message);
    }
    for (const Link& link : document.links) {
        checkShapes(findings, link);
        checkMass(findings, document, link);
        for (const Shape& shape : link.shapes) {
            if (shape.kind == Shape::Kind::Mesh) {
                checkMesh(findings, link, shape, meshBase);
            }
        }
        hintCaster(findings, document, link);
    }
    for (const Joint& joint : document.joints) {
        checkJoint(findings, document, joint);
    }
    for (const Sensor& sensor : document.sensors) {
        checkSensor(findings, document, sensor, catalog);
    }
    return findings.take();
}

bool validName(std::string_view name) {
    const auto letter = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    };
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    return !name.empty() && letter(name.front()) &&
           std::ranges::all_of(name, [&](char c) { return letter(c) || digit(c); });
}

std::string_view toString(Level level) {
    switch (level) {
        case Level::Error:
            return "error";
        case Level::Warning:
            return "warning";
        case Level::Hint:
            return "hint";
    }
    return "error";
}

}  // namespace lagrange::viewer::builder
