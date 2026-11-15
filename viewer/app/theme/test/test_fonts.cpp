// The design's font sizes are CSS font sizes: loadFonts() draws a token's size as the em of IBM
// Plex, as reference.html does, while Dear ImGui lays out with the same size.
#include "lagrange/viewer/Fonts.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <imgui.h>

namespace lagrange::theme {
namespace {

const std::string kFontDir = LAGRANGE_FONT_DIR;

std::vector<unsigned char> readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::uint32_t be32(const std::vector<unsigned char>& d, std::size_t at) {
    return (std::uint32_t{d.at(at)} << 24U) | (std::uint32_t{d.at(at + 1)} << 16U) |
           (std::uint32_t{d.at(at + 2)} << 8U) | std::uint32_t{d.at(at + 3)};
}

int be16s(const std::vector<unsigned char>& d, std::size_t at) {
    return static_cast<std::int16_t>((std::uint16_t{d.at(at)} << 8U) | d.at(at + 1));
}

/// The offset of a TrueType table, from the file's table directory.
std::size_t table(const std::vector<unsigned char>& d, const char* tag) {
    const std::size_t count = (std::size_t{d.at(4)} << 8U) | d.at(5);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t entry = 12 + (16 * i);
        if (std::string(d.begin() + entry, d.begin() + entry + 4) == tag) {
            return be32(d, entry + 8);
        }
    }
    ADD_FAILURE() << "no " << tag << " table";
    return 0;
}

TEST(Fonts, LineHeightInEmsIsTheFilesOwn) {
    for (const char* file :
         {"IBMPlexSans-Regular.ttf", "IBMPlexSans-Medium.ttf", "IBMPlexSans-SemiBold.ttf",
          "IBMPlexMono-Regular.ttf", "IBMPlexMono-Medium.ttf"}) {
        const std::vector<unsigned char> d = readFile(kFontDir + "/" + file);
        ASSERT_GT(d.size(), 64U) << file;
        const double unitsPerEm = be16s(d, table(d, "head") + 18);
        const std::size_t hhea = table(d, "hhea");
        const double line = be16s(d, hhea + 4) - be16s(d, hhea + 6);  // ascender - descender
        EXPECT_FLOAT_EQ(static_cast<float>(line / unitsPerEm), kPlexLineHeightEm) << file;
    }
}

class FontsInImGui : public ::testing::Test {
protected:
    void SetUp() override {
        ImGui::CreateContext();
        // As the viewer's renderer: the atlas grows on demand, no texture upload needed here.
        ImGui::GetIO().BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
        fonts_ = &loadFonts(kFontDir);
    }
    void TearDown() override { ImGui::DestroyContext(); }
    const Fonts* fonts_{nullptr};
};

TEST_F(FontsInImGui, ATokensSizeIsTheEmAsInCss) {
    ASSERT_TRUE(fonts_->bundled);
    // IBM Plex Mono is 600 units wide per glyph in 1000: 12 px CSS mono is 7.2 px a character.
    ImFontBaked* mono = fonts_->mono->GetFontBaked(kMonoSize);
    EXPECT_NEAR(mono->GetCharAdvance('0'), 0.6F * kMonoSize, 0.01F);
    // Cap height 698 units: an H of 13 px CSS text stands about 9 px tall.
    ImFontBaked* sans = fonts_->sans->GetFontBaked(kBodySize);
    const ImFontGlyph* h = sans->FindGlyph('H');
    ASSERT_NE(h, nullptr);
    EXPECT_NEAR(h->Y1 - h->Y0, 0.698F * kBodySize, 1.0F);
    // The baseline sits where CSS puts it in a line box of the font size (1025 / 1300 of 1.3 em
    // from the top, less half the leading): within a pixel, so text centres in its controls.
    const float cssBaseline = (1.025F - (kPlexLineHeightEm - 1.0F) / 2.0F) * kBodySize;
    EXPECT_NEAR(sans->Ascent, cssBaseline, 1.0F);
    // The size Dear ImGui lays out with stays the token's: a 13 px line, 22 px controls.
    ImGui::GetIO().DisplaySize = ImVec2(800, 600);
    ImGui::NewFrame();
    EXPECT_FLOAT_EQ(ImGui::GetFontSize(), kBodySize);
    EXPECT_FLOAT_EQ(ImGui::GetTextLineHeight(), kBodySize);
    ImGui::EndFrame();
}

}  // namespace
}  // namespace lagrange::theme
