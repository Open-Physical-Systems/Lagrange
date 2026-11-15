#include "lagrange/viewer/Fonts.hpp"

#include <array>
#include <filesystem>

namespace lagrange::theme {

namespace {

Fonts gFonts;  // UI thread only, like everything else that touches ImGui

/// The faces' configuration: the design's sizes are CSS font sizes (the em), so each glyph is
/// scaled up from Dear ImGui's line height to the em (kPlexLineHeightEm), while the font size
/// that Dear ImGui lays out with stays the token's size. Oversampled twice across, crisp at the
/// fractional pixel positions that the larger glyphs take.
ImFontConfig faceConfig(bool merge, float scale = kPlexLineHeightEm) {
    ImFontConfig config;
    config.MergeMode = merge;
    config.ExtraSizeScale = scale;
    config.OversampleH = 2;
    config.OversampleV = 1;
    return config;
}

}  // namespace

const Fonts& loadFonts(const std::string& directory) {
    struct Face {
        const char* file;
        ImFont* Fonts::* slot;
        float size;
        const char* fallback;  // merged in for glyphs `file` lacks; nullptr for none
        float scale;           // the glyphs' scale over Dear ImGui's line (faceConfig)
    };
    // The code face: pushed at kCodeLine, its em is kCodeSize (Fonts::code).
    constexpr float kCodeScale = kPlexLineHeightEm * kCodeSize / kCodeLine;
    // IBM Plex Mono has no Greek letters; IBM Plex Sans does. Symbols such as tau and omega
    // appear in mono text (signal symbols, units), so the mono faces fall back to Sans.
    static constexpr std::array<Face, 6> kFaces{{
        // first: the default
        {"IBMPlexSans-Regular.ttf", &Fonts::sans, kBodySize, nullptr, kPlexLineHeightEm},
        {"IBMPlexSans-Medium.ttf", &Fonts::sansMedium, kLabelSize, nullptr, kPlexLineHeightEm},
        {"IBMPlexSans-SemiBold.ttf", &Fonts::sansSemiBold, kBodySize, nullptr, kPlexLineHeightEm},
        {"IBMPlexMono-Regular.ttf", &Fonts::mono, kMonoSize, "IBMPlexSans-Regular.ttf",
         kPlexLineHeightEm},
        {"IBMPlexMono-Medium.ttf", &Fonts::monoMedium, kMonoSize, "IBMPlexSans-Medium.ttf",
         kPlexLineHeightEm},
        {"IBMPlexMono-Regular.ttf", &Fonts::code, kCodeLine, "IBMPlexSans-Regular.ttf", kCodeScale},
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
            const ImFontConfig own = faceConfig(false, face.scale);
            gFonts.*face.slot =
                atlas->AddFontFromFileTTF((base / face.file).c_str(), face.size, &own);
            if (face.fallback != nullptr && gFonts.*face.slot != nullptr) {
                const ImFontConfig merge = faceConfig(true, face.scale);
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
