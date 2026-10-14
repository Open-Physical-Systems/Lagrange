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
        const char* fallback;  // merged in for glyphs `file` lacks; nullptr for none
    };
    // IBM Plex Mono has no Greek letters; IBM Plex Sans does. Symbols such as tau and omega
    // appear in mono text (signal symbols, units), so the mono faces fall back to Sans.
    static constexpr std::array<Face, 5> kFaces{{
        {"IBMPlexSans-Regular.ttf", &Fonts::sans, kBodySize, nullptr},  // first: the default
        {"IBMPlexSans-Medium.ttf", &Fonts::sansMedium, kLabelSize, nullptr},
        {"IBMPlexSans-SemiBold.ttf", &Fonts::sansSemiBold, kBodySize, nullptr},
        {"IBMPlexMono-Regular.ttf", &Fonts::mono, kMonoSize, "IBMPlexSans-Regular.ttf"},
        {"IBMPlexMono-Medium.ttf", &Fonts::monoMedium, kMonoSize, "IBMPlexSans-Medium.ttf"},
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
            if (face.fallback != nullptr && gFonts.*face.slot != nullptr) {
                ImFontConfig merge;
                merge.MergeMode = true;
                atlas->AddFontFromFileTTF((base / face.fallback).c_str(), face.size, &merge);
            }
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
