// The design's font sizes are CSS font sizes: loadFonts() draws a token's size as the em of IBM
// Plex, as reference.html does, while Dear ImGui lays out with the same size.
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <imgui_internal.h>  // ImTextCharFromUtf8
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <imgui.h>

#include <lagrange/signals/Unit.hpp>

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

TEST_F(FontsInImGui, TheCodeFaceHasCssLinesOfItsOwn) {
    ASSERT_TRUE(fonts_->bundled);
    // Pushed at kCodeLine (19 px lines, as InputTextMultiline spaces them), its glyphs are 12 px
    // CSS mono: 7.2 px a character, the baseline where CSS centres a 12 px line in 19 px.
    ImFontBaked* code = fonts_->code->GetFontBaked(kCodeLine);
    EXPECT_NEAR(code->GetCharAdvance('0'), 0.6F * kCodeSize, 0.01F);
    const float cssBaseline =
        (kCodeLine - kPlexLineHeightEm * kCodeSize) / 2.0F + 1.025F * kCodeSize;
    EXPECT_NEAR(code->Ascent, cssBaseline, 1.0F);
}

// Wrapped text keeps the design's CSS line heights (widgets::Paragraph), where Dear ImGui would
// space its lines one font size apart.
TEST_F(FontsInImGui, ParagraphsHaveTheDesignsLineHeight) {
    using viewer::widgets::ParagraphLineHeight;
    EXPECT_FLOAT_EQ(ParagraphLineHeight(kBodySize), 18.0F);     // body 13 / 18
    EXPECT_FLOAT_EQ(ParagraphLineHeight(kLabelSize), 18.0F);    // .sm keeps the body's
    EXPECT_FLOAT_EQ(ParagraphLineHeight(kCaptionSize), 14.0F);  // .xs 11 / 14

    ImGui::GetIO().DisplaySize = ImVec2(800, 600);
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(400, 400));
    ImGui::Begin("paragraph");
    const char* text = "one two three four five six seven eight nine ten eleven twelve";
    ImFont* sans = fonts_->sans;
    // Wide enough for about four words a line: three lines or more, each 18 px.
    const float wrap = sans->CalcTextSizeA(kBodySize, FLT_MAX, 0.0F, "one two three four").x;
    const float height = viewer::widgets::ParagraphHeight(sans, kBodySize, text, wrap);
    const float lines = height / 18.0F;
    EXPECT_GE(lines, 3.0F);
    EXPECT_FLOAT_EQ(lines, std::round(lines));
    // Dear ImGui's own wrapping gives the same lines at 13 px each.
    const float imgui = sans->CalcTextSizeA(kBodySize, FLT_MAX, wrap, text).y;
    EXPECT_FLOAT_EQ(imgui / kBodySize, lines);
    // As an item: the same height, and a SameLine after it on its first line.
    ImGui::PushFont(sans, kBodySize);
    const float top = ImGui::GetCursorScreenPos().y;
    viewer::widgets::Paragraph(text, 0.0F, wrap);
    EXPECT_FLOAT_EQ(ImGui::GetItemRectSize().y, height);
    ImGui::SameLine();
    EXPECT_FLOAT_EQ(ImGui::GetCursorScreenPos().y, top);
    // Lines that would run below a limit are left out, and it says so.
    ImDrawList* draw = ImGui::GetWindowDrawList();
    EXPECT_FALSE(viewer::widgets::DrawParagraph(draw, sans, kBodySize, ImVec2(0, 0), 0xFFFFFFFF,
                                                text, wrap, 0.0F, height));
    EXPECT_TRUE(viewer::widgets::DrawParagraph(draw, sans, kBodySize, ImVec2(0, 0), 0xFFFFFFFF,
                                               text, wrap, 0.0F, height - 1.0F));
    ImGui::PopFont();
    ImGui::End();
    ImGui::EndFrame();
}

/// Every character of every unit text the core can write (lagrange::symbol, label, to_string:
/// superscript digits and minus, the middle dot, Omega) and the symbols the panels write
/// themselves is drawn by every face: none falls back to Dear ImGui's missing-glyph box, as the
/// superscript minus of "kg·s⁻¹" did before the LagrangeUnitGlyphs fonts.
TEST_F(FontsInImGui, EveryUnitsTextHasItsGlyphs) {
    ASSERT_TRUE(fonts_->bundled);
    std::vector<std::string> texts;
    const auto add = [&texts](const Unit& unit) {
        texts.push_back(symbol(unit));
        texts.push_back(label(unit));
        texts.push_back(to_string(unit));
    };
    // Each base dimension at every exponent a schema writes ("s-1", "m2"; two digits at most).
    for (int exponent = -19; exponent <= 19; ++exponent) {
        const auto e = static_cast<std::int8_t>(exponent);
        for (Dimension d :
             {Dimension{.length = e}, Dimension{.mass = e}, Dimension{.time = e},
              Dimension{.current = e}, Dimension{.temperature = e}, Dimension{.amount = e},
              Dimension{.luminous = e}, Dimension{.angle = e}}) {
            add(Unit{d, 1.0, QuantityKind::Unspecified});
        }
    }
    // The units the fire, actuators and lights domains name, and every quantity kind's name.
    for (const char* text : {"kg/s", "kg.m-2.s-1", "kg/m2/s", "W/m2", "ohm", "N.m", "kg.m2", "lm/W",
                             "1/m", "m3/s", "Pa.s"}) {
        if (const UnitParse parsed = parseUnit(text); parsed.unit) {
            add(*parsed.unit);
        }
    }
    for (int kind = 0; kind <= static_cast<int>(QuantityKind::LuminousEfficacy); ++kind) {
        add(Unit{{.length = -2, .mass = 1, .time = -1}, 1.0, static_cast<QuantityKind>(kind)});
    }
    // Written by the panels: degrees, micro, plus-minus, Greek symbols, arrows, relations.
    texts.emplace_back("° ± ² ³ µ · × ÷ Δ Ω σ τ – — … ↑ → ↓ ↔ − ∞ ≠ ≤ ≥ §");
    ASSERT_NE(std::ranges::find(texts, "kg·m⁻²·s⁻¹"), texts.end());

    for (ImFont* font : {fonts_->sans, fonts_->sansMedium, fonts_->sansSemiBold, fonts_->mono,
                         fonts_->monoMedium, fonts_->code}) {
        for (const std::string& text : texts) {
            const char* at = text.c_str();
            const char* const end = at + text.size();
            while (at < end) {
                unsigned int c = 0;
                at += ImTextCharFromUtf8(&c, at, end);
                EXPECT_TRUE(font->IsGlyphInFont(static_cast<ImWchar>(c)))
                    << "U+" << std::hex << c << " of \"" << text << "\" in "
                    << font->GetDebugName();
            }
        }
    }
}

// The superscript minus sits on the superscript figures' middle, a Mono cell wide in Mono.
TEST_F(FontsInImGui, TheSuperscriptMinusSitsWithTheFigures) {
    ASSERT_TRUE(fonts_->bundled);
    for (ImFont* font : {fonts_->sans, fonts_->mono}) {
        const float size = font == fonts_->mono ? kMonoSize : kBodySize;
        ImFontBaked* baked = font->GetFontBaked(size);
        const ImFontGlyph* minus = baked->FindGlyphNoFallback(0x207B);
        const ImFontGlyph* one = baked->FindGlyphNoFallback(0x00B9);
        ASSERT_NE(minus, nullptr) << font->GetDebugName();
        ASSERT_NE(one, nullptr);
        EXPECT_NEAR((minus->Y0 + minus->Y1) / 2.0F, (one->Y0 + one->Y1) / 2.0F, 1.0F);
        EXPECT_GT(minus->X1 - minus->X0, 0.15F * size);
        if (font == fonts_->mono) {
            EXPECT_NEAR(minus->AdvanceX, 0.6F * kMonoSize, 0.01F);
        }
    }
}

}  // namespace
}  // namespace lagrange::theme
