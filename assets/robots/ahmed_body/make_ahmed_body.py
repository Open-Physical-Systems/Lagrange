#!/usr/bin/env python3
"""Writes the Ahmed body: make_ahmed_body.py SLANT_DEGREES OUT_STEM (OUT_STEM.obj, OUT_STEM.urdf).

The reference car shape of Ahmed, Ramm and Faltin (SAE 840300, 1984): length 1.044 m, width
0.389 m, height 0.288 m, nose radius 0.1 m, slant length 0.222 m. No stilts. Run it 50 mm over the
road: lagrange tunnel OUT_STEM.urdf --speed 40 --ground --clearance 0.05
"""

import math
import sys
from pathlib import Path

LENGTH, WIDTH, HEIGHT, RADIUS, SLANT = 1.044, 0.389, 0.288, 0.1, 0.222
ARC_POINTS = 8


def profile(slant_degrees: float) -> list[tuple[float, float]]:
    """(x, z) round the side view, wind along +x, nose at x = 0, bottom at z = 0."""
    slant = math.radians(slant_degrees)
    arc = [math.pi / 2 * k / ARC_POINTS for k in range(ARC_POINTS + 1)]
    top = [(RADIUS * (1 - math.cos(a)), HEIGHT - RADIUS + RADIUS * math.sin(a)) for a in arc]
    bottom = [(RADIUS * (1 - math.cos(a)), RADIUS - RADIUS * math.sin(a)) for a in reversed(arc)]
    tail = [
        (LENGTH - SLANT * math.cos(slant), HEIGHT),
        (LENGTH, HEIGHT - SLANT * math.sin(slant)),
        (LENGTH, 0.0),
    ]
    return [*top, *tail, *bottom]


def main() -> None:
    slant, stem = float(sys.argv[1]), Path(sys.argv[2])
    side = profile(slant)
    n = len(side)
    lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for y in (-WIDTH / 2, WIDTH / 2) for x, z in side]
    for i in range(n):
        j = (i + 1) % n
        lines += [f"f {i + 1} {j + 1} {n + j + 1}", f"f {i + 1} {n + j + 1} {n + i + 1}"]
    for i in range(1, n - 1):
        lines += [f"f 1 {i + 2} {i + 1}", f"f {n + 1} {n + i + 1} {n + i + 2}"]
    stem.with_suffix(".obj").write_text("\n".join(lines) + "\n")
    mesh = f'<geometry><mesh filename="{stem.with_suffix(".obj").name}"/></geometry>'
    stem.with_suffix(".urdf").write_text(
        f'<robot name="ahmed_body"><link name="body"><inertial><mass value="10"/>'
        f'<inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>'
        f"<visual>{mesh}</visual><collision>{mesh}</collision></link></robot>\n"
    )


if __name__ == "__main__":
    main()
