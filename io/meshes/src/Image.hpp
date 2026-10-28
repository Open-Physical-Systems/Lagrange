#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace lagrange::meshes::detail {

/// An 8-bit RGB image, rows top to bottom.
struct Image {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> rgb;
};

/// Decodes a PNG or JPEG file (told apart by its signature), or nothing.
[[nodiscard]] std::optional<Image> readImage(const std::string& path);

/// Decodes a PNG or JPEG held in memory (a texture embedded in a glTF or FBX file), or nothing.
[[nodiscard]] std::optional<Image> decodeImage(std::span<const std::uint8_t> bytes);

}  // namespace lagrange::meshes::detail
