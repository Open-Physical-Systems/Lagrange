"""Triangle meshes: primitives and the import-only file formats (STL, OBJ, a subset of DAE).

Every loader returns a TriMesh with one normal per vertex. Meshes are never loaded at
simulation time; this module is used when a robot description is imported.
"""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


class MeshError(Exception):
    """The file is not a mesh this importer can read."""


@dataclass
class TriMesh:
    positions: np.ndarray  # float32, (n, 3)
    normals: np.ndarray  # float32, (n, 3)
    indices: np.ndarray  # uint32, (3 * triangles,)

    @property
    def triangle_count(self) -> int:
        return len(self.indices) // 3


def from_triangles(triangles: np.ndarray) -> TriMesh:
    """A mesh from an (n, 3, 3) array of triangle corners, with flat (per-face) normals."""
    triangles = np.asarray(triangles, dtype=np.float64).reshape(-1, 3, 3)
    if not np.all(np.isfinite(triangles)):
        raise MeshError("mesh contains non-finite coordinates")
    edges_a = triangles[:, 1] - triangles[:, 0]
    edges_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_a, edges_b)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    normals[lengths[:, 0] == 0] = (0.0, 0.0, 1.0)  # degenerate triangles: any unit normal
    return TriMesh(
        positions=triangles.reshape(-1, 3).astype("<f4"),
        normals=np.repeat(normals, 3, axis=0).astype("<f4"),
        indices=np.arange(len(triangles) * 3, dtype="<u4"),
    )


# --- primitives ----------------------------------------------------------------------------


def box(size_x: float, size_y: float, size_z: float) -> TriMesh:
    half = np.array([size_x, size_y, size_z]) / 2.0
    triangles = []
    for axis in range(3):
        u, v = (axis + 1) % 3, (axis + 2) % 3
        for sign in (-1.0, 1.0):
            corners = []
            for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                corner = np.zeros(3)
                corner[axis] = sign * half[axis]
                corner[u] = du * half[u]
                corner[v] = dv * half[v]
                corners.append(corner)
            if sign < 0:
                corners.reverse()
            triangles += [
                [corners[0], corners[1], corners[2]],
                [corners[0], corners[2], corners[3]],
            ]
    return from_triangles(np.array(triangles))


def cylinder(radius: float, length: float, segments: int = 32) -> TriMesh:
    """A cylinder along Z, centred at the origin (the URDF convention)."""
    angles = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
    top, bottom = length / 2.0, -length / 2.0
    positions, normals, indices = [], [], []
    for i in range(segments):  # smooth side wall
        x, y = ring[i]
        outward = [x / radius, y / radius, 0.0]
        positions += [[x, y, bottom], [x, y, top]]
        normals += [outward, outward]
        a, b = 2 * i, 2 * ((i + 1) % segments)
        indices += [a, b, a + 1, a + 1, b, b + 1]
    for z, direction in ((top, 1.0), (bottom, -1.0)):  # flat caps
        centre = len(positions)
        positions.append([0.0, 0.0, z])
        normals.append([0.0, 0.0, direction])
        for x, y in ring:
            positions.append([x, y, z])
            normals.append([0.0, 0.0, direction])
        for i in range(segments):
            j = (i + 1) % segments
            pair = (centre + 1 + i, centre + 1 + j)
            indices += [centre, *(pair if direction > 0 else pair[::-1])]
    return TriMesh(np.array(positions, "<f4"), np.array(normals, "<f4"), np.array(indices, "<u4"))


def sphere(radius: float, segments: int = 32, rings: int = 16) -> TriMesh:
    positions, normals, indices = [], [], []
    for r in range(rings + 1):
        polar = np.pi * r / rings
        for s in range(segments + 1):
            azimuth = 2.0 * np.pi * s / segments
            n = [np.sin(polar) * np.cos(azimuth), np.sin(polar) * np.sin(azimuth), np.cos(polar)]
            normals.append(n)
            positions.append([radius * c for c in n])
    for r in range(rings):
        for s in range(segments):
            a = r * (segments + 1) + s
            b = a + segments + 1
            indices += [a, b, a + 1, a + 1, b, b + 1]
    return TriMesh(np.array(positions, "<f4"), np.array(normals, "<f4"), np.array(indices, "<u4"))


# --- files ---------------------------------------------------------------------------------

MAX_TRIANGLES = 2_000_000  # refuse absurd files instead of exhausting memory


def load_stl(data: bytes) -> TriMesh:
    # A binary STL is exactly 84 + 50 * n bytes. Checking that is more reliable than looking
    # for the word "solid", which some binary files also start with.
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:
            if count > MAX_TRIANGLES:
                raise MeshError(f"STL has {count} triangles; the limit is {MAX_TRIANGLES}")
            records = np.frombuffer(
                data,
                dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                count=count,
                offset=84,
            )
            return from_triangles(records["v"])
    try:
        text = data.decode("ascii", errors="strict")
    except UnicodeDecodeError as error:
        raise MeshError("not a binary STL of consistent size and not ASCII") from error
    vertices = [
        line.split()[1:4] for line in text.splitlines() if line.strip().startswith("vertex")
    ]
    if not vertices or len(vertices) % 3 != 0:
        raise MeshError("ASCII STL has no triangles or an incomplete one")
    try:
        return from_triangles(np.array(vertices, dtype=np.float64).reshape(-1, 3, 3))
    except ValueError as error:
        raise MeshError(f"ASCII STL has a malformed vertex: {error}") from error


def load_obj(data: bytes) -> TriMesh:
    vertices: list[list[float]] = []
    triangles: list[list[list[float]]] = []
    for line in data.decode("utf-8", errors="replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        try:
            if parts[0] == "v":
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif parts[0] == "f":
                corners = []
                for token in parts[1:]:
                    index = int(token.split("/")[0])
                    corners.append(vertices[index - 1 if index > 0 else index])
                for k in range(1, len(corners) - 1):  # fan triangulation of polygons
                    triangles.append([corners[0], corners[k], corners[k + 1]])
        except (ValueError, IndexError) as error:
            raise MeshError(f"malformed OBJ line: {line.strip()!r}") from error
        if len(triangles) > MAX_TRIANGLES:
            raise MeshError(f"OBJ has more than {MAX_TRIANGLES} triangles")
    if not triangles:
        raise MeshError("OBJ has no faces")
    return from_triangles(np.array(triangles))


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(element: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in element if _local(child.tag) == name]


def _first(element: ET.Element, name: str) -> ET.Element | None:
    found = _children(element, name)
    return found[0] if found else None


def _node_matrix(node: ET.Element) -> np.ndarray:
    """The local transform of a COLLADA <node>: its transform elements multiplied in order."""
    matrix = np.eye(4)
    for child in node:
        kind = _local(child.tag)
        values = [float(v) for v in (child.text or "").split()]
        if kind == "matrix" and len(values) == 16:
            matrix = matrix @ np.array(values).reshape(4, 4)
        elif kind == "translate" and len(values) == 3:
            step = np.eye(4)
            step[:3, 3] = values
            matrix = matrix @ step
        elif kind == "scale" and len(values) == 3:
            matrix = matrix @ np.diag([*values, 1.0])
        elif kind == "rotate" and len(values) == 4:
            axis = np.array(values[:3])
            norm = np.linalg.norm(axis)
            if norm > 0:
                x, y, z = axis / norm
                angle = np.radians(values[3])
                c, s = np.cos(angle), np.sin(angle)
                step = np.eye(4)
                step[:3, :3] = [
                    [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                    [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                    [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
                ]
                matrix = matrix @ step
    return matrix


def load_dae(data: bytes) -> TriMesh:
    """Geometry of a COLLADA file: triangles and polygons, placed by the scene's node transforms.

    Read: <triangles>, <polylist>, <polygons>; node matrices; the unit and up axis of the asset.
    Not read: materials, textures, skinning, animation.
    """
    try:
        root = ET.fromstring(data)
    except ET.ParseError as error:
        raise MeshError(f"not valid COLLADA XML: {error}") from error

    unit, up_axis = 1.0, "Y_UP"
    asset = _first(root, "asset")
    if asset is not None:
        unit_element = _first(asset, "unit")
        if unit_element is not None:
            unit = float(unit_element.get("meter", "1"))
        axis_element = _first(asset, "up_axis")
        if axis_element is not None and axis_element.text:
            up_axis = axis_element.text.strip()

    geometries: dict[str, np.ndarray] = {}
    for library in _children(root, "library_geometries"):
        for geometry in _children(library, "geometry"):
            mesh = _first(geometry, "mesh")
            if mesh is None:
                continue
            sources = {}
            for source in _children(mesh, "source"):
                array = _first(source, "float_array")
                if array is not None and array.text:
                    sources["#" + source.get("id", "")] = np.array(
                        array.text.split(), dtype=np.float64
                    )
            vertex_sources = {}
            for vertices in _children(mesh, "vertices"):
                for item in _children(vertices, "input"):
                    if item.get("semantic") == "POSITION":
                        vertex_sources["#" + vertices.get("id", "")] = item.get("source", "")
            triangles: list[np.ndarray] = []
            for kind in ("triangles", "polylist", "polygons"):
                for primitive in _children(mesh, kind):
                    inputs = _children(primitive, "input")
                    stride = max((int(i.get("offset", "0")) for i in inputs), default=0) + 1
                    vertex_input = next((i for i in inputs if i.get("semantic") == "VERTEX"), None)
                    if vertex_input is None:
                        continue
                    position_id = vertex_sources.get(vertex_input.get("source", ""))
                    if position_id not in sources:
                        continue
                    points = sources[position_id].reshape(-1, 3)
                    offset = int(vertex_input.get("offset", "0"))
                    polygons: list[np.ndarray] = []
                    if kind == "polygons":
                        for p in _children(primitive, "p"):
                            polygons.append(
                                np.array((p.text or "").split(), dtype=np.int64).reshape(
                                    -1, stride
                                )[:, offset]
                            )
                    else:
                        p = _first(primitive, "p")
                        if p is None or not p.text:
                            continue
                        indices = np.array(p.text.split(), dtype=np.int64).reshape(-1, stride)[
                            :, offset
                        ]
                        if kind == "triangles":
                            polygons = list(indices[: len(indices) // 3 * 3].reshape(-1, 3))
                        else:
                            counts = _first(primitive, "vcount")
                            start = 0
                            for n in (
                                (int(c) for c in (counts.text or "").split())
                                if counts is not None
                                else ()
                            ):
                                polygons.append(indices[start : start + n])
                                start += n
                    for polygon in polygons:
                        if (
                            len(polygon) < 3
                            or polygon.max(initial=0) >= len(points)
                            or polygon.min(initial=0) < 0
                        ):
                            continue
                        for k in range(1, len(polygon) - 1):
                            triangles.append(points[[polygon[0], polygon[k], polygon[k + 1]]])
                    if len(triangles) > MAX_TRIANGLES:
                        raise MeshError(f"COLLADA file has more than {MAX_TRIANGLES} triangles")
            if triangles:
                geometries["#" + geometry.get("id", "")] = np.array(triangles)

    if not geometries:
        raise MeshError("COLLADA file has no triangle geometry")

    placed: list[np.ndarray] = []

    def visit(node: ET.Element, parent: np.ndarray) -> None:
        matrix = parent @ _node_matrix(node)
        for instance in _children(node, "instance_geometry"):
            triangles = geometries.get(instance.get("url", ""))
            if triangles is not None:
                flat = triangles.reshape(-1, 3)
                placed.append((flat @ matrix[:3, :3].T + matrix[:3, 3]).reshape(-1, 3, 3))
        for child in _children(node, "node"):
            visit(child, matrix)

    for library in _children(root, "library_visual_scenes"):
        for scene in _children(library, "visual_scene"):
            for node in _children(scene, "node"):
                visit(node, np.eye(4))
    if not placed:  # no scene graph: take the geometries as they are
        placed = list(geometries.values())

    triangles = np.concatenate(placed) * unit
    if up_axis == "Y_UP":  # to Z up: (x, y, z) -> (x, -z, y)
        triangles = np.stack([triangles[..., 0], -triangles[..., 2], triangles[..., 1]], axis=-1)
    elif up_axis == "X_UP":  # (x, y, z) -> (-y, x, z)
        triangles = np.stack([-triangles[..., 1], triangles[..., 0], triangles[..., 2]], axis=-1)
    return from_triangles(triangles)


LOADERS = {".stl": load_stl, ".obj": load_obj, ".dae": load_dae}


def load_mesh_file(path: Path) -> TriMesh:
    loader = LOADERS.get(path.suffix.lower())
    if loader is None:
        raise MeshError(f"unsupported mesh format '{path.suffix}' (supported: STL, OBJ, DAE)")
    try:
        data = path.read_bytes()
    except OSError as error:
        raise MeshError(f"cannot read {path}: {error.strerror}") from error
    return loader(data)
