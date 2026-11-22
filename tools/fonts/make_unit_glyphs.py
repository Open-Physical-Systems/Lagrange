#!/usr/bin/env python3
"""Writes the viewer's unit-glyph fonts: the superscript minus (U+207B) that IBM Plex lacks.

Unit symbols such as "kg·s⁻¹" (core/signals/src/Unit.cpp, `symbol`) need U+207B SUPERSCRIPT
MINUS, which no IBM Plex Sans or Mono face has. These small fonts hold that one glyph, drawn
here as a plain bar (no outline is taken from Plex), sized to sit with Plex's superscript
figures: centred on them (329 to 698 font units in every Plex face), 0.75 times the face's
hyphen thick, as wide as a superscript figure in Sans and a whole cell in Mono. Their vertical
metrics are Plex's (1000 units per em, hhea 1025 / -275), so Dear ImGui places a merged glyph on
the same baseline (viewer/app/theme/src/Fonts.cpp merges each into its face).

    python3 tools/fonts/make_unit_glyphs.py viewer/assets/fonts

Needs fontTools (python3-fonttools).
"""

from __future__ import annotations

import sys
from pathlib import Path

from fontTools.fontBuilder import FontBuilder
from fontTools.pens.ttGlyphPen import TTGlyphPen

UNITS_PER_EM = 1000
ASCENT = 1025  # IBM Plex hhea ascender, every face
DESCENT = -275  # IBM Plex hhea descender, every face
SUPERSCRIPT_BOTTOM = 329  # IBM Plex superscript figures' extent (font units), every face
SUPERSCRIPT_TOP = 698
SUPERSCRIPT_SCALE = 0.75  # the bar's thickness over the face's hyphen

# (file, family style, advance, bar left, bar right, the face's hyphen thickness in units)
# Hyphen thicknesses measured from the bundled Plex TTFs (U+002D bounds): Sans Regular 86,
# Sans Medium 111, Mono Regular 84, Mono Medium 109.
FACES = [
    ("LagrangeUnitGlyphs-SansRegular.ttf", "Sans Regular", 346, 48, 298, 86),
    ("LagrangeUnitGlyphs-SansMedium.ttf", "Sans Medium", 346, 46, 300, 111),
    ("LagrangeUnitGlyphs-MonoRegular.ttf", "Mono Regular", 600, 170, 430, 84),
    ("LagrangeUnitGlyphs-MonoMedium.ttf", "Mono Medium", 600, 168, 432, 109),
]


def bar(left: int, right: int, bottom: int, top: int):
    pen = TTGlyphPen(None)
    pen.moveTo((left, bottom))
    pen.lineTo((left, top))
    pen.lineTo((right, top))
    pen.lineTo((right, bottom))
    pen.closePath()
    return pen.glyph()


def build(path: Path, style: str, advance: int, left: int, right: int, hyphen: int) -> None:
    thickness = round(hyphen * SUPERSCRIPT_SCALE)
    centre = (SUPERSCRIPT_BOTTOM + SUPERSCRIPT_TOP) / 2
    bottom = round(centre - thickness / 2)
    top = bottom + thickness
    fb = FontBuilder(UNITS_PER_EM, isTTF=True)
    fb.setupGlyphOrder([".notdef", "uni207B"])
    fb.setupCharacterMap({0x207B: "uni207B"})
    empty = TTGlyphPen(None).glyph()
    fb.setupGlyf({".notdef": empty, "uni207B": bar(left, right, bottom, top)})
    fb.setupHorizontalMetrics({".notdef": (advance, 0), "uni207B": (advance, left)})
    fb.setupHorizontalHeader(ascent=ASCENT, descent=DESCENT)
    family = "Lagrange Unit Glyphs"
    fb.setupNameTable(
        {
            "familyName": family,
            "styleName": style,
            "copyright": "Open Physical Systems and the Lagrange authors",
            "licenseDescription": "Apache-2.0",
        }
    )
    fb.setupOS2(
        sTypoAscender=780,
        sTypoDescender=-220,
        sTypoLineGap=300,
        usWinAscent=ASCENT,
        usWinDescent=-DESCENT,
    )
    fb.setupPost()
    fb.save(str(path))


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("viewer/assets/fonts")
    for name, style, advance, left, right, hyphen in FACES:
        build(out / name, style, advance, left, right, hyphen)
        print(out / name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
