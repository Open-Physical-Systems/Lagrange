#include "Image.hpp"

#include <algorithm>
#include <csetjmp>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <jpeglib.h>
#include <png.h>

namespace lagrange::meshes::detail {

namespace {

constexpr std::uint8_t kPngSignature[] = {0x89, 'P', 'N', 'G'};
constexpr std::uint8_t kJpegSignature[] = {0xFF, 0xD8};

std::optional<Image> decodePng(std::span<const std::uint8_t> bytes) {
    png_image png{};
    png.version = PNG_IMAGE_VERSION;
    if (png_image_begin_read_from_memory(&png, bytes.data(), bytes.size()) == 0) {
        return std::nullopt;
    }
    png.format = PNG_FORMAT_RGB;
    Image image;
    image.width = static_cast<int>(png.width);
    image.height = static_cast<int>(png.height);
    image.rgb.resize(PNG_IMAGE_SIZE(png));
    if (png_image_finish_read(&png, nullptr, image.rgb.data(), 0, nullptr) == 0) {
        png_image_free(&png);
        return std::nullopt;
    }
    return image;
}

struct JpegError {
    jpeg_error_mgr base{};
    std::jmp_buf jump{};
};

[[noreturn]] void onJpegError(j_common_ptr info) {
    std::longjmp(reinterpret_cast<JpegError*>(info->err)->jump, 1);  // NOLINT
}

// libjpeg reports errors by longjmp; nothing with a destructor may live across setjmp here,
// so the output buffer is owned by the caller.
bool decodeJpegInto(std::span<const std::uint8_t> bytes, Image& image) {
    jpeg_decompress_struct info{};
    JpegError error;
    info.err = jpeg_std_error(&error.base);
    error.base.error_exit = onJpegError;
    if (setjmp(error.jump) != 0) {  // NOLINT(cert-err52-cpp): libjpeg's error model
        jpeg_destroy_decompress(&info);
        return false;
    }
    jpeg_create_decompress(&info);
    jpeg_mem_src(&info, bytes.data(), static_cast<unsigned long>(bytes.size()));
    jpeg_read_header(&info, TRUE);
    info.out_color_space = JCS_RGB;
    jpeg_start_decompress(&info);
    image.width = static_cast<int>(info.output_width);
    image.height = static_cast<int>(info.output_height);
    image.rgb.resize(static_cast<std::size_t>(image.width) * image.height * 3);
    while (info.output_scanline < info.output_height) {
        JSAMPROW row = image.rgb.data() + static_cast<std::size_t>(info.output_scanline) *
                                              static_cast<std::size_t>(image.width) * 3;
        jpeg_read_scanlines(&info, &row, 1);
    }
    jpeg_finish_decompress(&info);
    jpeg_destroy_decompress(&info);
    return true;
}

bool startsWith(std::span<const std::uint8_t> bytes, std::span<const std::uint8_t> prefix) {
    return bytes.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), bytes.begin());
}

}  // namespace

std::optional<Image> decodeImage(std::span<const std::uint8_t> bytes) {
    if (startsWith(bytes, kPngSignature)) {
        return decodePng(bytes);
    }
    if (startsWith(bytes, kJpegSignature)) {
        Image image;
        if (decodeJpegInto(bytes, image)) {
            return image;
        }
    }
    return std::nullopt;
}

std::optional<Image> readImage(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return std::nullopt;
    }
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());
    return decodeImage(bytes);
}

}  // namespace lagrange::meshes::detail
