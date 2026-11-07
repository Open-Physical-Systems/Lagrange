#include "SurfaceTextures.hpp"

#include <array>
#include <csetjmp>
#include <cstddef>
#include <cstdio>
#include <jpeglib.h>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>

namespace lagrange::gpu {

namespace {

// In the order of realistic.slang's layers.
constexpr std::array<std::string_view, 8> kKinds{"brick",    "asphalt", "pavement", "plaster",
                                                 "concrete", "lawn",    "bark",     "roof"};

struct JpegError {
    jpeg_error_mgr base{};
    std::jmp_buf jump{};
};

[[noreturn]] void onJpegError(j_common_ptr info) {
    std::longjmp(reinterpret_cast<JpegError*>(info->err)->jump, 1);  // NOLINT
}

// libjpeg reports errors by longjmp; `pixels` is sized before setjmp and nothing with a destructor
// starts between it and the end.
bool decode(std::FILE* file, std::vector<std::uint8_t>& pixels) {
    jpeg_decompress_struct info{};
    JpegError error;
    info.err = jpeg_std_error(&error.base);
    error.base.error_exit = onJpegError;
    if (setjmp(error.jump) != 0) {  // NOLINT
        jpeg_destroy_decompress(&info);
        return false;
    }
    jpeg_create_decompress(&info);
    jpeg_stdio_src(&info, file);
    jpeg_read_header(&info, TRUE);
    info.out_color_space = JCS_EXT_RGBA;
    jpeg_start_decompress(&info);
    if (info.output_width != SurfaceTextures::kSize ||
        info.output_height != SurfaceTextures::kSize || info.output_components != 4) {
        jpeg_destroy_decompress(&info);
        return false;
    }
    while (info.output_scanline < info.output_height) {
        JSAMPROW row =
            pixels.data() + (std::size_t{info.output_scanline} * SurfaceTextures::kSize * 4);
        jpeg_read_scanlines(&info, &row, 1);
    }
    jpeg_finish_decompress(&info);
    jpeg_destroy_decompress(&info);
    return true;
}

bool read(const std::filesystem::path& path, std::vector<std::uint8_t>& pixels) {
    const std::unique_ptr<std::FILE, int (*)(std::FILE*)> file(std::fopen(path.c_str(), "rb"),
                                                               &std::fclose);
    if (!file) {
        return false;
    }
    pixels.resize(std::size_t{SurfaceTextures::kSize} * SurfaceTextures::kSize * 4);
    return decode(file.get(), pixels);
}

}  // namespace

SurfaceTextures loadSurfaceTextures(const std::filesystem::path& directory) {
    SurfaceTextures out;
    for (const std::string_view kind : kKinds) {
        std::vector<std::uint8_t> albedo;
        std::vector<std::uint8_t> normal;
        if (!read(directory / (std::string(kind) + "_albedo.jpg"), albedo) ||
            !read(directory / (std::string(kind) + "_normal.jpg"), normal)) {
            return {};
        }
        out.albedo.push_back(std::move(albedo));
        out.normal.push_back(std::move(normal));
    }
    out.complete = true;
    return out;
}

std::filesystem::path surfaceTextureDirectory() {
    std::error_code ec;
    const std::filesystem::path installed = LAGRANGE_GPU_TEXTURES;
    return std::filesystem::is_directory(installed, ec) ? installed : LAGRANGE_GPU_SOURCE_TEXTURES;
}

}  // namespace lagrange::gpu
