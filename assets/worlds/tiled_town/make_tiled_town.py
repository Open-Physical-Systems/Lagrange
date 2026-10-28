#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Open Physical Systems
# SPDX-License-Identifier: Apache-2.0
"""Writes the tiled town: a streamed world for the huge-worlds samples (huge-worlds.md §2).

A 2048 m square town in two levels: 4 coarse tiles of 1024 m (level 0, visual only: one flat
block per building quarter) and 64 tiles of 256 m (level 1, with colliders: the buildings as
boxes). Each tile is a WorldTileAPI_1 prim in tiled_town.usda whose payload is a file in
tiles/, authored relative to the tile's corner. The buildings come from a fixed pseudo-random
sequence per tile, so the output is the same on every run.

    python3 make_tiled_town.py [OUT_DIR]   # default: this script's directory
"""

import sys
from pathlib import Path

WORLD_M = 2048.0
FINE_M = 256.0
COARSE_M = 1024.0
STREET_M = 24.0  # the free band along each tile's west and south edges
BLOCKS = 3  # buildings per row and column of a fine tile


def lcg(seed: int):
    """A small linear congruential generator (Numerical Recipes' constants), as floats in [0, 1)."""
    state = seed & 0xFFFFFFFF
    while True:
        state = (1664525 * state + 1013904223) & 0xFFFFFFFF
        yield state / 2**32


def buildings(x: int, y: int) -> list[tuple[float, float, float, float, float]]:
    """(centre x, centre y, width, depth, height) in the fine tile's frame, metres."""
    rand = lcg(x * 7919 + y * 104729 + 1)
    out = []
    cell = (FINE_M - STREET_M) / BLOCKS
    for i in range(BLOCKS):
        for j in range(BLOCKS):
            if next(rand) < 0.25:
                continue  # a square or a car park
            width = cell * (0.45 + 0.4 * next(rand))
            depth = cell * (0.45 + 0.4 * next(rand))
            height = 6.0 + 54.0 * next(rand) ** 2
            cx = STREET_M + (i + 0.5) * cell
            cy = STREET_M + (j + 0.5) * cell
            out.append((cx, cy, width, depth, height))
    return out


def cube(name: str, centre, size, colour, collide: bool) -> str:
    """A box: an Xform at its centre with a drawn Cube, and with `collide` a collision Cube
    (purpose guide: not drawn), as the importers write them."""
    rgb = "(" + ", ".join(f"{c:.2f}" for c in colour) + ")"
    scale = f"({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f})"
    out = (
        f'    def Xform "{name}"\n    {{\n'
        f"        double3 xformOp:translate = ({centre[0]:.2f}, {centre[1]:.2f}, {centre[2]:.2f})\n"
        f'        uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f'        def Cube "visual"\n        {{\n'
        f"            double size = 1\n"
        f"            color3f[] primvars:displayColor = [{rgb}]\n"
        f"            float3 xformOp:scale = {scale}\n"
        f'            uniform token[] xformOpOrder = ["xformOp:scale"]\n'
        f"        }}\n"
    )
    if collide:
        out += (
            f'        def Cube "collision" (\n'
            f'            prepend apiSchemas = ["PhysicsCollisionAPI"]\n'
            f"        )\n        {{\n"
            f"            double size = 1\n"
            f'            uniform token purpose = "guide"\n'
            f"            float3 xformOp:scale = {scale}\n"
            f'            uniform token[] xformOpOrder = ["xformOp:scale"]\n'
            f"        }}\n"
        )
    return out + "    }\n"


HEADER = '#usda 1.0\n(\n    defaultPrim = "tile"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'


def fine_tile(x: int, y: int) -> str:
    body = ""
    for k, (cx, cy, w, d, h) in enumerate(buildings(x, y)):
        shade = 0.55 + 0.1 * ((x + y + k) % 3)
        body += cube(
            f"building_{k}", (cx, cy, h / 2), (w, d, h), (shade, shade, shade + 0.03), True
        )
    return HEADER + f'def Xform "tile"\n{{\n{body}}}\n'


def coarse_tile(x: int, y: int) -> str:
    body = ""
    per = int(COARSE_M / FINE_M)
    for i in range(per):
        for j in range(per):
            fx, fy = x * per + i, y * per + j
            heights = [h for (_, _, _, _, h) in buildings(fx, fy)]
            mean = sum(heights) / len(heights) if heights else 0.0
            if mean <= 0.0:
                continue
            centre = (
                i * FINE_M + FINE_M / 2 + STREET_M / 2,
                j * FINE_M + FINE_M / 2 + STREET_M / 2,
                mean / 2,
            )
            body += cube(
                f"quarter_{i}_{j}",
                centre,
                (FINE_M - STREET_M, FINE_M - STREET_M, mean),
                (0.6, 0.6, 0.63),
                False,
            )
    return HEADER + f'def Xform "tile"\n{{\n{body}}}\n'


def tile_prim(
    level: int, x: int, y: int, size: float, error: float, collide: bool, top: float
) -> str:
    x0, y0 = x * size, y * size
    return (
        f'    def Xform "L{level}_{x}_{y}" (\n'
        f'        prepend apiSchemas = ["WorldTileAPI_1"]\n'
        f"        prepend payload = @./tiles/L{level}_{x}_{y}.usda@\n"
        f"    )\n    {{\n"
        f"        int world:tile:level = {level}\n"
        f"        int2 world:tile:index = ({x}, {y})\n"
        f"        double3 world:tile:boundsMin = ({x0:.1f}, {y0:.1f}, 0)\n"
        f"        double3 world:tile:boundsMax = ({x0 + size:.1f}, {y0 + size:.1f}, {top:.1f})\n"
        f"        double world:tile:geometricError = {error}\n"
        f"        bool world:tile:hasCollision = {'true' if collide else 'false'}\n"
        f"        double3 xformOp:translate = ({x0:.1f}, {y0:.1f}, 0)\n"
        f'        uniform token[] xformOpOrder = ["xformOp:translate"]\n'
        f"    }}\n"
    )


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
    (out / "tiles").mkdir(parents=True, exist_ok=True)
    prims = ""
    for x in range(int(WORLD_M / COARSE_M)):
        for y in range(int(WORLD_M / COARSE_M)):
            (out / "tiles" / f"L0_{x}_{y}.usda").write_text(coarse_tile(x, y))
            prims += tile_prim(0, x, y, COARSE_M, 8.0, False, 60.0)
    for x in range(int(WORLD_M / FINE_M)):
        for y in range(int(WORLD_M / FINE_M)):
            (out / "tiles" / f"L1_{x}_{y}.usda").write_text(fine_tile(x, y))
            prims += tile_prim(1, x, y, FINE_M, 0.5, True, 60.0)
    doc = (
        "The tiled town (huge-worlds.md section 2), written by make_tiled_town.py:\n"
        "    a 2048 m square town streamed in tiles. 4 coarse tiles of 1024 m (visual only)\n"
        "    and 64 tiles of 256 m with the buildings as colliders; the server loads them\n"
        "    around the robot as it drives. The ground is the plane the loader adds."
    )
    root = (
        "#usda 1.0\n(\n"
        '    defaultPrim = "tiled_town"\n'
        f'    doc = """{doc}"""\n'
        "    kilogramsPerUnit = 1\n    metersPerUnit = 1\n"
        '    upAxis = "Z"\n)\n\n'
        f'def Xform "tiled_town"\n{{\n{prims}}}\n'
    )
    (out / "tiled_town.usda").write_text(root)
    print(f"wrote {out / 'tiled_town.usda'} and {4 + 64} tiles")


if __name__ == "__main__":
    main()
