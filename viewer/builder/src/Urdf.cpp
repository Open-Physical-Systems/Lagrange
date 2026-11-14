// Stub (RFC 0016 WP0): WP-B replaces this file with the tinyxml2 writer and reader of RFC 0016
// §6 and the atomic save of §10.
#include "lagrange/viewer/builder/Urdf.hpp"

namespace lagrange::viewer::builder {

namespace {
constexpr const char* kNotImplemented =
    "URDF reading and writing are not implemented yet "
    "(RFC 0016 WP-B)";
}  // namespace

std::string writeUrdf(const Document& document, const WriteOptions& /*options*/) {
    // A well-formed placeholder, so a caller that saves it gets a file that says what it is.
    return "<?xml version=\"1.0\"?>\n<!-- " + std::string(kNotImplemented) +
           " -->\n<robot name=\"" + document.robot + "\"/>\n";
}

ReadResult readUrdf(std::string_view /*text*/) {
    ReadResult result;
    result.error = kNotImplemented;
    return result;
}

std::string saveAtomically(const std::filesystem::path& path, std::string_view /*text*/) {
    return "cannot save " + path.string() + ": " + kNotImplemented;
}

}  // namespace lagrange::viewer::builder
