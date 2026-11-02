#!/usr/bin/env python3
"""Writes the Ahmed body: make_ahmed_body.py SLANT_DEGREES OUT_STEM (OUT_STEM.obj, OUT_STEM.urdf).

The reference car shape of Ahmed, Ramm and Faltin (SAE 840300, 1984): length 1.044 m, width
0.389 m, height 0.288 m, nose radius 0.1 m on all four front edges, slant length 0.222 m. No stilts. Run it 50 mm over the
road: lagrange tunnel OUT_STEM.urdf --speed 40 --ground --clearance 0.05
"""

import math
import sys
from pathlib import Path

LENGTH, WIDTH, HEIGHT, RADIUS, SLANT = 1.044, 0.389, 0.288, 0.1, 0.222
ARC_POINTS = 8


def sections(slant_degrees: float) -> list[tuple[float, float, float, float]]:
    """(x, half width, bottom z, top z) along the wind, nose at x = 0, bottom at z = 0.

    The nose is rounded on all four edges: each section is the rectangle moved in by the fillet.
    """
    slant = math.radians(slant_degrees)
    out = []
    for k in range(ARC_POINTS + 1):
        angle = math.pi / 2 * k / ARC_POINTS
        x = RADIUS * (1 - math.cos(angle))
        inset = RADIUS * (1 - math.sin(angle))
        out.append((x, WIDTH / 2 - inset, inset, HEIGHT - inset))
    start = LENGTH - SLANT * math.cos(slant)
    return [
        *out,
        (start, WIDTH / 2, 0.0, HEIGHT),
        (LENGTH, WIDTH / 2, 0.0, HEIGHT - SLANT * math.sin(slant)),
    ]


def main() -> None:
    slant, stem = float(sys.argv[1]), Path(sys.argv[2])
    rings = sections(slant)
    lines = []
    for x, half, bottom, top in rings:
        for y, z in ((-half, bottom), (half, bottom), (half, top), (-half, top)):
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
    for r in range(len(rings) - 1):
        for k in range(4):
            a, b = 4 * r + k + 1, 4 * r + (k + 1) % 4 + 1
            lines += [f"f {a} {a + 4} {b + 4}", f"f {a} {b + 4} {b}"]
    last = 4 * (len(rings) - 1)
    lines += ["f 1 2 3", "f 1 3 4", f"f {last + 1} {last + 3} {last + 2}"]
    lines.append(f"f {last + 1} {last + 4} {last + 3}")
    stem.with_suffix(".obj").write_text("\n".join(lines) + "\n")
    mesh = f'<geometry><mesh filename="{stem.with_suffix(".obj").name}"/></geometry>'
    stem.with_suffix(".urdf").write_text(
        f'<robot name="ahmed_body"><link name="body"><inertial><mass value="10"/>'
        f'<inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>'
        f"<visual>{mesh}</visual><collision>{mesh}</collision></link></robot>\n"
    )


if __name__ == "__main__":
    main()
