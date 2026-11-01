"""Splits a URDF's concave collision meshes into convex parts, as a new URDF beside it.

    lagrange bake robot.urdf [-o robot_baked.urdf] [--threshold 0.05] [--seed 0]

A collision mesh collides as its convex hull, which fills in what is hollow. This writes each
concave one as several `<collision>` elements, one per convex part (CoACD), with the parts as OBJ
files in a folder next to the new URDF. The original files are not touched.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_prefix

from lagrange_collision_bake.bake import Settings, _decompose


def read_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    points: list[list[float]] = []
    faces: list[list[int]] = []
    for line in path.read_text().splitlines():
        words = line.split()
        if words and words[0] == "v":
            points.append([float(x) for x in words[1:4]])
        elif words and words[0] == "f":
            faces.append([int(x.split("/")[0]) - 1 for x in words[1:4]])
    return np.asarray(points, dtype=np.float64), np.asarray(faces, dtype=np.int32)


def write_obj(path: Path, points: np.ndarray, faces: np.ndarray) -> None:
    lines = [f"v {p[0]:.9g} {p[1]:.9g} {p[2]:.9g}" for p in points]
    lines += [f"f {f[0] + 1} {f[1] + 1} {f[2] + 1}" for f in faces]
    path.write_text("\n".join(lines) + "\n")


def to_obj(uri: str, base: Path, scale: str, out: Path) -> None:
    tool = Path(get_package_prefix("lagrange_server")) / "lib/lagrange_server/lagrange_mesh_to_obj"
    subprocess.run(
        [str(tool), uri, str(base), scale.replace(" ", ","), str(out)],
        check=True,
        capture_output=True,
        text=True,
    )


def bake_urdf(source: Path, target: Path, settings: Settings) -> list[tuple[str, int]]:
    """Writes the new URDF and its parts. Returns each mesh's link and how many parts it became."""
    tree = ElementTree.parse(source)
    root = tree.getroot()
    folder = target.with_suffix("")
    folder = folder.parent / f"{folder.name}_collision"
    folder.mkdir(parents=True, exist_ok=True)
    report = []
    with tempfile.TemporaryDirectory() as work:
        for link in root.findall("link"):
            for k, collision in enumerate(link.findall("collision")):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    continue
                scratch = Path(work) / "mesh.obj"
                to_obj(mesh.get("filename", ""), source.parent, mesh.get("scale", "1 1 1"), scratch)
                points, faces = read_obj(scratch)
                parts = _decompose(points, faces, settings)
                name = link.get("name", "link")
                report.append((name, len(parts)))
                index = list(link).index(collision)
                link.remove(collision)
                for p, (part_points, part_faces) in enumerate(parts):
                    file = folder / f"{name}_{k}_{p:03d}.obj"
                    write_obj(file, np.asarray(part_points), np.asarray(part_faces))
                    part = ElementTree.Element("collision")
                    origin = collision.find("origin")
                    if origin is not None:
                        part.append(origin)
                    geometry = ElementTree.SubElement(part, "geometry")
                    ElementTree.SubElement(
                        geometry, "mesh", {"filename": f"{folder.name}/{file.name}"}
                    )
                    link.insert(index + p, part)
    # The new file sits beside the old one, so what the old one names relative to it still holds.
    tree.write(target, encoding="unicode", xml_declaration=False)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="lagrange bake", description="Splits a URDF's concave collision meshes."
    )
    parser.add_argument("urdf", type=Path)
    parser.add_argument("-o", "--output", type=Path, help="default: NAME_baked.urdf beside it")
    parser.add_argument("--threshold", type=float, default=Settings.threshold)
    parser.add_argument("--seed", type=int, default=Settings.seed)
    args = parser.parse_args(argv)
    if not args.urdf.exists():
        print(f"lagrange bake: no file {args.urdf}", file=sys.stderr)
        return 2
    target = args.output or args.urdf.with_name(f"{args.urdf.stem}_baked.urdf")
    settings = Settings(threshold=args.threshold, seed=args.seed)
    report = bake_urdf(args.urdf, target, settings)
    for link, parts in report:
        print(f"{link}: {parts} convex part{'s' if parts != 1 else ''}")
    if not report:
        print("no collision meshes")
    print(f"wrote {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
