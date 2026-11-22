#!/usr/bin/env python3
"""Writes the orchard: rows of fruit trees with a forest edge, for agricultural drone runs.

Every prim is made here, so the orchard needs no outside assets. Run it from this directory:

    python3 make_orchard.py

Eight rows of fifteen fruit trees, 4 m apart in the row and 4 m between rows (rows along x),
mown grass alleys between them, tall grass in a few patches, a landing pad west of the rows and a
forest edge of taller trees to the north and east. Each tree is a TreeAPI_1 prim (ground.md
"Trees") with a trunk collider up to its crown base; the crowns are not colliders.

The species presets below are typical values, not measurements of one orchard. Where a number has
no source we have read, it says TODO(verify) (docs/physics/ground.md "Trees", "Numbers and
sources").
"""

import math
import random
from pathlib import Path

random.seed(20261122)

SPACING = 4.0  # m between trees in a row
ROW_GAP = 4.0  # m between rows
ROWS = 8
PER_ROW = 15
PAD = (-36.0, 0.0)  # the landing pad's middle, west of the rows
PAD_HEIGHT = 0.1  # m

# Species presets. height, crownRadius, crownBase, trunkDiameter in m; leafAreaDensity in m²/m³;
# naturalFrequency in Hz (first sway mode); dampingRatio; branchCount.
#
# Fruit trees: trained, pruned trees of a high-density orchard. Leaf area density: the brief's
# 1-3 m²/m³ for apple crowns, TODO(verify) against a measured profile (e.g. Sinoquet et al.'s 3D
# digitised apple trees). First sway frequencies of 1-2 Hz for trees of 3-5 m follow the trend
# Moore and Maguire 2004 (Trees 18, 195) collect, f falling with height (f ∝ DBH/H²), TODO(verify)
# against free-swing measurements of fruit trees (trunk-shaker studies report the shaking
# frequencies that detach fruit, 4-6 Hz for apple canopies, not the first mode). Damping ratios of
# 0.1-0.2 are within Moore and Maguire's collection, TODO(verify) per species.
#
# Forest edge: open-grown broadleaves and conifers of 14-20 m, 0.25-0.4 Hz (Moore and Maguire
# 2004: measured first frequencies of 0.2-0.5 Hz for trees of that size); leaf area densities
# TODO(verify).
SPECIES = {
    "apple": dict(
        height=3.5,
        crownRadius=1.1,
        crownBase=0.8,
        trunkDiameter=0.12,
        leafAreaDensity=2.0,
        naturalFrequency=1.6,
        dampingRatio=0.15,
        branchCount=16,
        leafArea=0.0025,  # m², TODO(verify)
    ),
    "pear": dict(
        height=4.5,
        crownRadius=1.2,
        crownBase=1.0,
        trunkDiameter=0.15,
        leafAreaDensity=1.6,
        naturalFrequency=1.3,
        dampingRatio=0.15,
        branchCount=14,
        leafArea=0.003,
    ),
    "cherry": dict(
        height=5.0,
        crownRadius=1.6,
        crownBase=1.2,
        trunkDiameter=0.2,
        leafAreaDensity=1.3,
        naturalFrequency=1.0,
        dampingRatio=0.12,
        branchCount=12,
        leafArea=0.004,
    ),
    "oak": dict(
        height=15.0,
        crownRadius=4.0,
        crownBase=4.5,
        trunkDiameter=0.6,
        leafAreaDensity=1.0,
        naturalFrequency=0.35,
        dampingRatio=0.1,
        branchCount=24,
        leafArea=0.004,
    ),
    "poplar": dict(
        height=19.0,
        crownRadius=2.6,
        crownBase=5.0,
        trunkDiameter=0.5,
        leafAreaDensity=0.8,
        naturalFrequency=0.3,
        dampingRatio=0.08,
        branchCount=18,
        leafArea=0.003,
    ),
    "spruce": dict(
        height=17.0,
        crownRadius=2.4,
        crownBase=2.5,
        trunkDiameter=0.45,
        leafAreaDensity=1.5,
        naturalFrequency=0.3,
        dampingRatio=0.08,
        branchCount=30,
        leafArea=0.0005,  # m², a shoot's needles counted as one leaf, TODO(verify)
    ),
}
ROW_SPECIES = ["apple"] * 4 + ["pear"] * 2 + ["cherry"] * 2

LOOKS = {
    "lawn": ((0.22, 0.38, 0.14), "grass"),
    "tall_grass": ((0.36, 0.42, 0.18), "tall_grass"),
    "soil": ((0.36, 0.29, 0.2), None),
    "pad": ((0.95, 0.55, 0.15), None),
    "mark": ((0.96, 0.96, 0.94), None),
}


def f(v):
    return f"{v:.3f}"


def material(name, colour, preset):
    api = (
        '\n        (\n            prepend apiSchemas = ["SensorMaterialAPI_1"]\n        )'
        if preset
        else ""
    )
    sens = f'\n            uniform token sensmat:preset = "{preset}"' if preset else ""
    r, g, b = colour
    return f"""        def Material "{name}"{api}
        {{{sens}
            token outputs:surface.connect = </orchard/looks/{name}/surface.outputs:surface>
            def Shader "surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({r}, {g}, {b})
                float inputs:metallic = 0.0
                float inputs:roughness = 0.9
                token outputs:surface
            }}
        }}
"""


def box(name, centre, size, look, collide=True):
    """A box drawn and (unless collide is False) collided, bound to a look."""
    colour = LOOKS[look][0]
    out = f"""    def Xform "{name}"
    {{
        double3 xformOp:translate = ({f(centre[0])}, {f(centre[1])}, {f(centre[2])})
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "visual" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {{
            double size = 1
            color3f[] primvars:displayColor = [({colour[0]}, {colour[1]}, {colour[2]})]
            rel material:binding = </orchard/looks/{look}>
            float3 xformOp:scale = ({f(size[0])}, {f(size[1])}, {f(size[2])})
            uniform token[] xformOpOrder = ["xformOp:scale"]
        }}
"""
    if collide:
        out += f"""        def Cube "collision" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            double size = 1
            uniform token purpose = "guide"
            rel material:binding = </orchard/looks/{look}>
            float3 xformOp:scale = ({f(size[0])}, {f(size[1])}, {f(size[2])})
            uniform token[] xformOpOrder = ["xformOp:scale"]
        }}
"""
    return out + "    }\n"


def tree(name, x, y, species, scale=1.0):
    """A TreeAPI_1 prim at the trunk's foot and the trunk's collider up to the crown base."""
    s = dict(SPECIES[species])
    for key in ("height", "crownRadius", "crownBase", "trunkDiameter"):
        s[key] *= scale
    # Taller trees of a species sway slower: f ∝ 1/H at a fixed slenderness (Moore and Maguire).
    s["naturalFrequency"] /= scale
    yaw = random.uniform(0.0, 2.0 * math.pi)
    qw, qz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    trunk_h = s["crownBase"] + 0.3 * (s["height"] - s["crownBase"])  # into the crown
    return f"""    def Xform "{name}" (
        prepend apiSchemas = ["TreeAPI_1"]
        doc = "{species}"
    )
    {{
        double tree:height = {f(s["height"])}
        double tree:crownRadius = {f(s["crownRadius"])}
        double tree:crownBase = {f(s["crownBase"])}
        double tree:trunkDiameter = {f(s["trunkDiameter"])}
        double tree:leafAreaDensity = {s["leafAreaDensity"]}
        double tree:naturalFrequency = {f(s["naturalFrequency"])}
        double tree:dampingRatio = {s["dampingRatio"]}
        double tree:leafArea = {s["leafArea"]}
        int tree:branchCount = {s["branchCount"]}
        quatd xformOp:orient = ({qw:.6f}, 0, 0, {qz:.6f})
        double3 xformOp:translate = ({f(x)}, {f(y)}, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
    }}

    def Xform "{name}_trunk"
    {{
        double3 xformOp:translate = ({f(x)}, {f(y)}, {f(trunk_h / 2.0)})
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cylinder "collision" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            uniform token axis = "Z"
            double height = {f(trunk_h)}
            uniform token purpose = "guide"
            double radius = {f(s["trunkDiameter"] / 2.0)}
        }}
    }}
"""


def main():
    parts = []
    parts.append('    def Scope "looks"\n    {\n')
    for name, (colour, preset) in LOOKS.items():
        parts.append(material(name, colour, preset))
    parts.append("    }\n")

    x0 = -(PER_ROW - 1) * SPACING / 2.0
    y0 = -(ROWS - 1) * ROW_GAP / 2.0
    length = (PER_ROW - 1) * SPACING + 2 * SPACING
    # Mown alleys between the rows and outside the outer ones (a 1 m bare strip under each row).
    for a in range(ROWS + 1):
        y = y0 + (a - 0.5) * ROW_GAP
        parts.append(box(f"alley_{a}", (0.0, y, 0.01), (length, ROW_GAP - 1.0, 0.02), "lawn"))
    for r in range(ROWS):
        y = y0 + r * ROW_GAP
        parts.append(box(f"strip_{r}", (0.0, y, 0.005), (length, 1.0, 0.01), "soil", False))
    trees = 0
    for r in range(ROWS):
        y = y0 + r * ROW_GAP
        species = ROW_SPECIES[r]
        for k in range(PER_ROW):
            x = x0 + k * SPACING + random.uniform(-0.15, 0.15)
            parts.append(tree(f"row{r + 1}_{k + 1:02d}", x, y, species, random.uniform(0.9, 1.1)))
            trees += 1
    # The forest edge: an irregular belt north of the rows and along their east end.
    edge = []
    x = x0 - 6.0
    while x < -x0 + 14.0:
        edge.append((x + random.uniform(-1.0, 1.0), -y0 + 9.0 + random.uniform(-1.5, 1.5)))
        x += random.uniform(5.0, 7.0)
    y = -y0 + 2.0
    while y > y0 - 8.0:
        edge.append((-x0 + 12.0 + random.uniform(-1.5, 1.5), y + random.uniform(-1.0, 1.0)))
        y -= random.uniform(5.0, 7.0)
    for n, (ex, ey) in enumerate(edge):
        species = random.choice(["oak", "poplar", "spruce"])
        parts.append(tree(f"edge_{n + 1:02d}", ex, ey, species, random.uniform(0.85, 1.15)))
        trees += 1
    # Tall grass in a few patches along the forest edge and the headland.
    for n, (cx, cy, sx, sy) in enumerate(
        [
            (x0 - 8.0, -y0 + 5.0, 8.0, 5.0),
            (-x0 + 7.0, y0 - 3.0, 5.0, 8.0),
            (4.0, y0 - 5.0, 12.0, 3.0),
        ]
    ):
        parts.append(box(f"tall_grass_{n + 1}", (cx, cy, 0.01), (sx, sy, 0.02), "tall_grass"))
    # The landing pad: a 2 m square platform 10 cm high, a bar across it as in the open field.
    # (A pad of a few millimetres let the quadcopter's legs sink into it and hold it there.)
    parts.append(
        box("landing_pad", (PAD[0], PAD[1], PAD_HEIGHT / 2.0), (2.0, 2.0, PAD_HEIGHT), "pad")
    )
    parts.append(
        box(
            "landing_pad_mark",
            (PAD[0], PAD[1], PAD_HEIGHT + 0.001),
            (1.2, 0.2, 0.002),
            "mark",
            False,
        )
    )

    doc = (
        f"A sample orchard for agricultural drone runs: {ROWS} rows of {PER_ROW} fruit trees "
        f"(apple, pear, cherry), a forest edge, {trees} TreeAPI_1 trees in all, mown alleys, tall "
        "grass, a landing pad. Written by make_orchard.py; no real place."
    )
    text = f"""#usda 1.0
(
    defaultPrim = "orchard"
    doc = "{doc}"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "orchard"
{{
{"".join(parts)}}}
"""
    out = Path(__file__).with_name("orchard.usda")
    out.write_text(text)
    print(f"{out.name}: {trees} trees, {len(text) / 1024:.0f} kB")


if __name__ == "__main__":
    main()
