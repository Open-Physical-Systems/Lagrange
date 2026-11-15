// Diff, schema and coupling for the Domains workspace's secondary tabs (ProfileExtras.hpp).
#include "lagrange/viewer/ProfileExtras.hpp"

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <map>
#include <set>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace lagrange::viewer {

// --- Diff ----------------------------------------------------------------------------------

namespace {

std::vector<std::string_view> splitLines(std::string_view text) {
    std::vector<std::string_view> lines;
    while (!text.empty()) {
        const std::size_t end = text.find('\n');
        lines.push_back(text.substr(0, end));
        if (end == std::string_view::npos) {
            break;
        }
        text.remove_prefix(end + 1);
    }
    return lines;
}

}  // namespace

std::vector<DiffLine> diffLines(std::string_view before, std::string_view after) {
    const std::vector<std::string_view> a = splitLines(before);
    const std::vector<std::string_view> b = splitLines(after);
    const std::size_t n = a.size();
    const std::size_t m = b.size();
    // lcs[i][j]: the longest common subsequence of a[i..] and b[j..]. Profiles are a few
    // dozen lines, so the table is small.
    std::vector<std::vector<std::uint32_t>> lcs(n + 1, std::vector<std::uint32_t>(m + 1, 0));
    for (std::size_t i = n; i-- > 0;) {
        for (std::size_t j = m; j-- > 0;) {
            lcs[i][j] =
                a[i] == b[j] ? lcs[i + 1][j + 1] + 1 : std::max(lcs[i + 1][j], lcs[i][j + 1]);
        }
    }
    std::vector<DiffLine> out;
    out.reserve(n + m);
    std::size_t i = 0;
    std::size_t j = 0;
    while (i < n || j < m) {
        if (i < n && j < m && a[i] == b[j]) {
            out.push_back({DiffLine::Kind::Same, static_cast<int>(i + 1), static_cast<int>(j + 1),
                           std::string(a[i])});
            ++i;
            ++j;
        } else if (i < n && (j == m || lcs[i + 1][j] >= lcs[i][j + 1])) {
            out.push_back({DiffLine::Kind::Removed, static_cast<int>(i + 1), 0, std::string(a[i])});
            ++i;
        } else {
            out.push_back({DiffLine::Kind::Added, 0, static_cast<int>(j + 1), std::string(b[j])});
            ++j;
        }
    }
    return out;
}

std::size_t diffHunks(const std::vector<DiffLine>& diff) {
    std::size_t hunks = 0;
    bool inHunk = false;
    for (const DiffLine& line : diff) {
        const bool changed = line.kind != DiffLine::Kind::Same;
        hunks += changed && !inHunk ? 1U : 0U;
        inHunk = changed;
    }
    return hunks;
}

// --- Schema --------------------------------------------------------------------------------

namespace {

std::vector<std::filesystem::path> prefixes() {
    std::vector<std::filesystem::path> out;
    const char* path = std::getenv("AMENT_PREFIX_PATH");
    if (path == nullptr) {
        return out;
    }
    std::string_view rest(path);
    while (!rest.empty()) {
        const std::size_t colon = rest.find(':');
        const std::string_view prefix = rest.substr(0, colon);
        if (!prefix.empty()) {
            out.emplace_back(prefix);
        }
        rest = colon == std::string_view::npos ? std::string_view{} : rest.substr(colon + 1);
    }
    return out;
}

std::string scalar(const YAML::Node& node) {
    if (node.IsScalar()) {
        return node.Scalar();
    }
    if (node.IsSequence()) {
        std::string text = "[";
        for (std::size_t i = 0; i < node.size(); ++i) {
            text += (i == 0 ? "" : ", ") + scalar(node[i]);
        }
        return text + "]";
    }
    if (node.IsMap()) {
        return "{…}";
    }
    return "null";
}

class SchemaReader {
public:
    explicit SchemaReader(YAML::Node root) : root_(std::move(root)) {}

    /// The definition a "$ref": "#/$defs/name" points at; the node itself otherwise.
    [[nodiscard]] YAML::Node resolve(const YAML::Node& node) const {
        if (node.IsMap() && node["$ref"]) {
            const std::string ref = node["$ref"].Scalar();
            constexpr std::string_view kPrefix = "#/$defs/";
            if (ref.starts_with(kPrefix)) {
                const YAML::Node def = root_["$defs"][ref.substr(kPrefix.size())];
                if (def) {
                    return def;
                }
            }
        }
        return node;
    }

    /// "number", "one of: a | b", "array of string", ... for one key's schema.
    [[nodiscard]] std::string type(const YAML::Node& raw) const {
        const YAML::Node node = resolve(raw);
        if (!node.IsMap()) {
            return "";
        }
        if (node["const"]) {
            return "= " + scalar(node["const"]);
        }
        if (node["enum"]) {
            std::string text = "one of: ";
            for (std::size_t i = 0; i < node["enum"].size(); ++i) {
                text += (i == 0 ? "" : " | ") + scalar(node["enum"][i]);
            }
            return text;
        }
        for (const char* combinator : {"oneOf", "anyOf"}) {
            if (node[combinator] && node[combinator].IsSequence()) {
                std::string text;
                for (const YAML::Node& alternative : node[combinator]) {
                    std::string part = type(alternative);
                    const YAML::Node resolved = resolve(alternative);
                    if (resolved.IsMap() && resolved["pattern"]) {
                        part += " (a plugin's name)";
                    }
                    if (resolved.IsMap() && resolved["properties"]) {
                        part = "{";
                        bool first = true;
                        for (const auto& property : resolved["properties"]) {
                            part += (first ? "" : ", ") + property.first.Scalar();
                            first = false;
                        }
                        part += "}";
                    }
                    text += (text.empty() ? "" : "  or  ") + part;
                }
                return text;
            }
        }
        const YAML::Node kind = node["type"];
        if (!kind) {
            return node["properties"] ? "object" : "";
        }
        std::string text = scalar(kind);
        if (text == "array" && node["items"]) {
            const YAML::Node items = resolve(node["items"]);
            const std::string inner = type(items);
            if (!inner.empty()) {
                text += " of " + inner;
            }
        }
        return text;
    }

    [[nodiscard]] static std::string constraints(const YAML::Node& node) {
        if (!node.IsMap()) {
            return "";
        }
        std::string text;
        const auto add = [&](const char* key, const char* symbol) {
            if (node[key]) {
                text += (text.empty() ? "" : ", ") + std::string(symbol) + " " + scalar(node[key]);
            }
        };
        add("exclusiveMinimum", ">");
        add("minimum", "≥");
        add("exclusiveMaximum", "<");
        add("maximum", "≤");
        add("minItems", "items ≥");
        add("maxItems", "items ≤");
        if (node["minLength"] && scalar(node["minLength"]) == "1") {
            text += (text.empty() ? "" : ", ") + std::string("not empty");
        }
        return text;
    }

    /// Appends a key and, for an object, its own keys one level deeper.
    void field(std::vector<SchemaField>& out, const std::string& key, const YAML::Node& raw,
               int depth, bool required) const {
        const YAML::Node node = resolve(raw);
        SchemaField item;
        item.key = key;
        item.depth = depth;
        item.required = required;
        item.type = type(raw);
        item.constraints = constraints(node);
        if (node.IsMap()) {
            if (node["default"]) {
                item.defaultValue = scalar(node["default"]);
            }
            // The key's own description first; a shared definition's otherwise.
            const YAML::Node description =
                raw.IsMap() && raw["description"] ? raw["description"] : node["description"];
            if (description) {
                item.description = description.Scalar();
            }
            if (node["examples"]) {
                item.description +=
                    (item.description.empty() ? "e.g. " : "  e.g. ") + scalar(node["examples"]);
            }
        }
        out.push_back(std::move(item));
        if (!node.IsMap()) {
            return;
        }
        YAML::Node object = node;
        if (node["type"] && scalar(node["type"]) == "array" && node["items"]) {
            object = resolve(node["items"]);  // an array of objects: its items' keys
        }
        if (object.IsMap() && object["properties"]) {
            properties(out, object, depth + 1);
        }
    }

    void properties(std::vector<SchemaField>& out, const YAML::Node& object, int depth,
                    std::string_view skip = {}) const {
        const std::set<std::string> required = requiredOf(object);
        for (const auto& property : object["properties"]) {
            const std::string key = property.first.Scalar();
            if (key != skip) {
                field(out, key, property.second, depth, required.contains(key));
            }
        }
    }

    [[nodiscard]] static std::set<std::string> requiredOf(const YAML::Node& object) {
        std::set<std::string> out;
        if (object["required"] && object["required"].IsSequence()) {
            for (const YAML::Node& key : object["required"]) {
                out.insert(key.Scalar());
            }
        }
        return out;
    }

    /// A domain's definition: the keys of every definition it is made of (allOf), then its own.
    void domain(SchemaSection& section, const YAML::Node& raw) const {
        const YAML::Node node = resolve(raw);
        if (!node.IsMap()) {
            return;
        }
        std::set<std::string> seen;
        const auto take = [&](const YAML::Node& part) {
            const YAML::Node def = resolve(part);
            if (!def.IsMap()) {
                return;
            }
            if (def["properties"]) {
                std::vector<SchemaField> fields;
                properties(fields, def, 0);
                for (SchemaField& item : fields) {
                    if (item.depth != 0 || seen.insert(item.key).second) {
                        section.fields.push_back(std::move(item));
                    }
                }
            } else if (def["description"]) {
                section.note += (section.note.empty() ? "" : " ") + def["description"].Scalar();
            }
        };
        if (node["allOf"] && node["allOf"].IsSequence()) {
            for (const YAML::Node& part : node["allOf"]) {
                take(part);
            }
        }
        if (node["properties"]) {
            take(node);
        }
    }

private:
    YAML::Node root_;
};

}  // namespace

const SchemaSection* ProfileSchemaDoc::domain(std::string_view name) const {
    for (const SchemaSection& section : domains) {
        if (section.name == name) {
            return &section;
        }
    }
    return nullptr;
}

std::filesystem::path findProfileSchema() {
    std::error_code error;
    for (const char* share :
         {"share/lagrange_viewer/profiles/schema.json", "share/lagrange_profiles/schema.json"}) {
        for (const std::filesystem::path& prefix : prefixes()) {
            if (std::filesystem::is_regular_file(prefix / share, error)) {
                return prefix / share;
            }
        }
    }
    return {};
}

ProfileSchemaDoc loadProfileSchema(const std::filesystem::path& path) {
    ProfileSchemaDoc doc;
    doc.path = path;
    if (path.empty()) {
        doc.error = "No profile schema in this install (share/lagrange_profiles/schema.json).";
        return doc;
    }
    try {
        // JSON is read as YAML: the schema uses no JSON that YAML 1.2 reads differently.
        const YAML::Node root = YAML::LoadFile(path.string());
        if (!root.IsMap() || !root["properties"]) {
            doc.error = "Not a profile schema: no \"properties\".";
            return doc;
        }
        const SchemaReader reader(root);
        if (root["title"]) {
            doc.title = root["title"].Scalar();
        }
        if (root["$schema"]) {
            doc.dialect = root["$schema"].Scalar();
        }
        const YAML::Node version = root["properties"]["schema_version"];
        if (version && version["const"]) {
            doc.version = version["const"].as<std::uint32_t>(0U);
        }
        doc.top.name = "profile";
        reader.properties(doc.top.fields, root, 0, "domains");
        const YAML::Node domains = root["properties"]["domains"];
        if (domains && domains["properties"]) {
            for (const auto& property : domains["properties"]) {
                SchemaSection section;
                section.name = property.first.Scalar();
                reader.domain(section, property.second);
                doc.domains.push_back(std::move(section));
            }
            for (const std::string& name : SchemaReader::requiredOf(domains)) {
                doc.requiredDomains.push_back(name);
            }
        }
    } catch (const std::exception& error) {
        doc.error = std::string("Could not read the schema: ") + error.what();
    }
    return doc;
}

// --- Coupling ------------------------------------------------------------------------------

std::vector<Coupling> couplings(const WiringInfo& wiring) {
    const auto node = [&](const std::string& module) -> const WiringInfo::Node* {
        for (const WiringInfo::Node& item : wiring.nodes) {
            if (item.module == module) {
                return &item;
            }
        }
        return nullptr;
    };
    std::vector<Coupling> out;
    std::map<std::pair<std::string, std::string>, std::size_t> index;
    for (const WiringInfo::Wire& wire : wiring.wires) {
        const auto key = std::make_pair(wire.from, wire.to);
        auto it = index.find(key);
        if (it == index.end()) {
            Coupling pair;
            pair.from = wire.from;
            pair.to = wire.to;
            if (const WiringInfo::Node* writer = node(wire.from)) {
                pair.fromDomain = writer->domain;
                pair.fromDtNs = writer->dtNs;
                pair.fromSubstitute = writer->substitute;
            }
            if (const WiringInfo::Node* reader = node(wire.to)) {
                pair.toDomain = reader->domain;
                pair.toDtNs = reader->dtNs;
            }
            if (pair.fromDtNs > 0 && pair.toDtNs > 0 && pair.fromDtNs > pair.toDtNs) {
                pair.holdSteps = pair.fromDtNs / pair.toDtNs;
            }
            it = index.emplace(key, out.size()).first;
            out.push_back(std::move(pair));
        }
        Coupling& pair = out[it->second];
        ++pair.wires;
        pair.unitConflicts += wire.unitOk ? 0U : 1U;
        pair.stiff = pair.stiff || wire.stiff;
        std::string symbol = wire.symbol.empty() ? wire.signal : wire.symbol;
        if (!wire.sourceUnit.empty()) {
            symbol += " · " + wire.sourceUnit;
        }
        if (std::ranges::find(pair.symbols, symbol) == pair.symbols.end()) {
            pair.symbols.push_back(std::move(symbol));
        }
    }
    for (Coupling& pair : out) {
        pair.twoWay = index.contains(std::make_pair(pair.to, pair.from));
    }
    return out;
}

}  // namespace lagrange::viewer
