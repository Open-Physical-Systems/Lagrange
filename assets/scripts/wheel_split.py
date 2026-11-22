# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E501  (the Collada template's lines are as long as Collada's)
"""Split a car's one-piece Collada mesh into a body and its wheels, by geometry.

Some upstream car meshes (Autoware's lexus.dae) are one geometry that includes the four wheels,
so a robot drawing it on base_link shows wheels that neither spin nor steer. fetch_assets runs
this on such a file (`derive: wheel_split` in external_assets.yaml): every triangle whose
centroid lies inside a wheel's cylinder and whose three vertices lie inside a slightly wider one
(a coarse tyre's long tread triangles poke out of it) goes to that wheel, re-centred on the
wheel centre; the rest is the body. The parts keep the source's texture coordinates, material,
texture and axes: each part is drawn with the same visual rotation as the source, the body at
the source's visual origin and each wheel at its own link's origin.

The cylinders are given in the robot's base frame: the wheel centres, a radius (for the
centroid) and a `vertex_radius` (for the corners), and the slab along y from `inboard` metres
towards the car's centre line to `outboard` metres away from it.
`frame` is the source mesh's visual origin in that base frame (URDF xyz and rpy). The output is
deterministic: the same source and parameters give the same bytes.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

STAMP = "wheel_split.stamp"


@dataclass
class Mesh:
    """One triangle geometry: positions and texture coordinates, indexed per corner."""

    head: str  # the file up to <library_geometries>
    tail: str  # the file from </library_geometries>
    positions: np.ndarray  # (n, 3) float
    position_text: list[str]  # the source's own number text, so the body is copied exactly
    uvs: np.ndarray  # (m, 2) float
    uv_text: list[str]
    triangles: np.ndarray  # (t, 3, 2) int: per corner, position index and uv index
    material: str


def _array(text: str, name: str) -> list[str]:
    match = re.search(rf'<float_array id="[^"]*-{name}-array"[^>]*>([^<]*)<', text)
    if match is None:
        raise ValueError(f"no {name} array")
    return match.group(1).split()


def read(path: Path) -> Mesh:
    """Reads a Collada file with one triangle geometry (POSITION and TEXCOORD inputs)."""
    text = path.read_text()
    if text.count("<geometry ") != 1 or text.count("<triangles ") != 1:
        raise ValueError(f"{path}: expected one geometry with one <triangles>")
    start = text.index("<library_geometries>")
    end = text.index("</library_geometries>")
    position_text = _array(text, "positions")
    uv_text = _array(text, "map")
    tris = re.search(r'<triangles count="(\d+)" material="([^"]*)">(.*?)</triangles>', text, re.S)
    if tris is None:
        raise ValueError(f"{path}: no <triangles>")
    offsets = re.findall(r'<input offset="(\d)" semantic="(\w+)"', tris.group(3))
    if sorted(offsets) != [("0", "VERTEX"), ("1", "TEXCOORD")]:
        raise ValueError(f"{path}: expected VERTEX at offset 0 and TEXCOORD at offset 1")
    p = re.search(r"<p>([^<]*)</p>", tris.group(3))
    if p is None:
        raise ValueError(f"{path}: no <p>")
    triangles = np.array(p.group(1).split(), dtype=np.int64).reshape(-1, 3, 2)
    if len(triangles) != int(tris.group(1)):
        raise ValueError(f"{path}: <triangles> count differs from its indices")
    return Mesh(
        head=text[:start],
        tail=text[end:],
        positions=np.array(position_text, dtype=float).reshape(-1, 3),
        position_text=position_text,
        uvs=np.array(uv_text, dtype=float).reshape(-1, 2),
        uv_text=uv_text,
        triangles=triangles,
        material=tris.group(2),
    )


def rotation(rpy: list[float]) -> np.ndarray:
    """URDF's fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def assign(mesh: Mesh, spec: dict[str, Any]) -> tuple[np.ndarray, list[np.ndarray]]:
    """Each triangle's part: -1 the body, else the wheel's index in spec['wheels']."""
    turn = rotation(spec["frame"]["rpy"])
    corners = mesh.positions[mesh.triangles[:, :, 0]] @ turn.T + np.array(spec["frame"]["xyz"])
    part = np.full(len(mesh.triangles), -1)
    centres_in_source = []
    for i, centre in enumerate(spec["wheels"].values()):
        offset = corners - np.array(centre, dtype=float)
        outward = offset[..., 1] * (1.0 if centre[1] >= 0 else -1.0)
        centroid = offset.mean(axis=1)
        inside = (
            (np.hypot(offset[..., 0], offset[..., 2]) < spec["vertex_radius"])
            & (outward < spec["outboard"])
            & (outward > -spec["inboard"])
        ).all(axis=1) & (np.hypot(centroid[:, 0], centroid[:, 2]) < spec["radius"])
        part[inside & (part < 0)] = i
        centres_in_source.append(turn.T @ (np.array(centre, dtype=float) - spec["frame"]["xyz"]))
    return part, centres_in_source


def _number(v: float) -> str:
    text = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def write(
    mesh: Mesh, triangles: np.ndarray, path: Path, shift: np.ndarray | None, note: str
) -> None:
    """Writes the triangles as their own Collada file, with only the vertices they use."""
    used_p, p_index = np.unique(triangles[:, :, 0], return_inverse=True)
    used_t, t_index = np.unique(triangles[:, :, 1], return_inverse=True)
    if shift is None:
        positions = [" ".join(mesh.position_text[3 * i : 3 * i + 3]) for i in used_p]
    else:
        positions = [" ".join(_number(v) for v in mesh.positions[i] - shift) for i in used_p]
    uvs = [" ".join(mesh.uv_text[2 * i : 2 * i + 2]) for i in used_t]
    corners = np.stack([p_index.reshape(-1, 3), t_index.reshape(-1, 3)], axis=2).reshape(-1)
    name = path.stem
    geometry = f"""<library_geometries>
        <geometry id="{name}-lib" name="{name}">
            <mesh>
                <source id="{name}-lib-positions" name="position">
                    <float_array id="{name}-lib-positions-array" count="{3 * len(positions)}">{" ".join(positions)}</float_array>
                    <technique_common>
                        <accessor count="{len(positions)}" source="#{name}-lib-positions-array" stride="3">
                            <param name="X" type="float"/>
                            <param name="Y" type="float"/>
                            <param name="Z" type="float"/>
                        </accessor>
                    </technique_common>
                </source>
                <source id="{name}-lib-map" name="map">
                    <float_array id="{name}-lib-map-array" count="{2 * len(uvs)}">{" ".join(uvs)}</float_array>
                    <technique_common>
                        <accessor count="{len(uvs)}" source="#{name}-lib-map-array" stride="2">
                            <param name="U" type="float"/>
                            <param name="V" type="float"/>
                        </accessor>
                    </technique_common>
                </source>
                <vertices id="{name}-lib-vertices">
                    <input semantic="POSITION" source="#{name}-lib-positions"/>
                </vertices>
                <triangles count="{len(triangles)}" material="{mesh.material}">
                    <input offset="0" semantic="VERTEX" source="#{name}-lib-vertices"/>
                    <input offset="1" semantic="TEXCOORD" source="#{name}-lib-map"/>
                    <p>{" ".join(map(str, corners.tolist()))}</p>
                </triangles>
            </mesh>
        </geometry>
    """
    head = re.sub(r"(<\?xml[^>]*\?>\n)", r"\1<!-- " + note + " -->\n", mesh.head, count=1)
    tail = re.sub(r'url="#[^"]*-lib"', f'url="#{name}-lib"', mesh.tail, count=1)
    part = path.with_name(path.name + ".part")
    part.write_text(head + geometry + tail)
    part.replace(path)


def stamp_of(source: Path, spec: dict[str, Any]) -> str:
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    return json.dumps({"source_sha256": digest, "spec": spec}, sort_keys=True) + "\n"


def outputs(spec: dict[str, Any]) -> list[str]:
    return [spec["body"], *spec["wheels"]]


def derive(directory: Path, spec: dict[str, Any], force: bool = False) -> bool:
    """Splits directory/spec['source'] when its parts are missing or stale; True when written."""
    source = directory / spec["source"]
    stamp = stamp_of(source, spec)
    stamp_file = directory / STAMP
    if (
        not force
        and stamp_file.is_file()
        and stamp_file.read_text() == stamp
        and all((directory / name).is_file() for name in outputs(spec))
    ):
        return False
    mesh = read(source)
    part, centres = assign(mesh, spec)
    note = (
        f"Derived from {spec['source']} by Lagrange's fetch_assets (wheel_split);"
        " the source's licence applies"
    )
    write(mesh, mesh.triangles[part < 0], directory / spec["body"], None, note + ": the body")
    for i, name in enumerate(spec["wheels"]):
        write(
            mesh,
            mesh.triangles[part == i],
            directory / name,
            centres[i],
            note + f": a wheel, re-centred on {spec['wheels'][name]}",
        )
    stamp_file.write_text(stamp)
    return True
