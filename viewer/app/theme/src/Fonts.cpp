#include "lagrange/viewer/Fonts.hpp"

#include <array>
#include <filesystem>

namespace lagrange::theme {

namespace {

Fonts gFonts;  // UI thread only, like everything else that touches ImGui

}  // namespace

const Fonts& loadFonts(const std::string& directory) {
    struct Face {
        const char* file;
        ImFont* Fonts::* slot;
        float size;
    };
    static constexpr std::array<Face, 5> kFaces{{
        {"IBMPlexSans-Regular.ttf", &Fonts::sans, kBodySize},  // first: the default font
        {"IBMPlexSans-Medium.ttf", &Fonts::sansMedium, kLabelSize},
        {"IBMPlexSans-SemiBold.ttf", &Fonts::sansSemiBold, kBodySize},
        {"IBMPlexMono-Regular.ttf", &Fonts::mono, kMonoSize},
        {"IBMPlexMono-Medium.ttf", &Fonts::monoMedium, kMonoSize},
    }};

    ImFontAtlas* atlas = ImGui::GetIO().Fonts;
    const std::filesystem::path base(directory);
    bool complete = !directory.empty();
    for (const Face& face : kFaces) {
        std::error_code ec;
        complete = complete && std::filesystem::is_regular_file(base / face.file, ec);
    }

    gFonts = Fonts{};
    if (complete) {
        for (const Face& face : kFaces) {
            gFonts.*face.slot = atlas->AddFontFromFileTTF((base / face.file).c_str(), face.size);
            complete = complete && gFonts.*face.slot != nullptr;
        }
    }
    if (!complete) {
        ImFont* fallback = atlas->AddFontDefault();
        for (const Face& face : kFaces) {
            gFonts.*face.slot = fallback;
        }
    }
    gFonts.bundled = complete;
    ImGui::GetIO().FontDefault = gFonts.sans;
    ImGui::GetStyle().FontSizeBase = kBodySize;
    return gFonts;
}

const Fonts& fonts() {
    return gFonts;
}

}  // namespace lagrange::theme
