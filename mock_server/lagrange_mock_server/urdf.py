"""URDF (and xacro) to a viewer scene.

This is the v0.1 path for "open a robot and look at it": the mock server reads the URDF and
builds protocol scene messages directly. In v0.2 the real importer converts URDF to USD and
this module is retired (docs/plan/open-questions.md, Q4).

What is read: links, joints and their origins (the robot is posed with every joint at zero),
visual geometry (box, cylinder, sphere, STL/OBJ/DAE meshes with scale), and material colours.
What is not: collision geometry, inertial data, textures, `<gazebo>` and `<ros2_control>`
elements. Everything skipped or substituted is listed in the import report.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from lagrange.record.v1 import record_pb2
from lagrange.viewer.v1 import scene_pb2

from . import meshes
from .recording import FORMAT_VERSION, Recording

NO_PARENT = 0xFFFFFFFF
DEFAULT_COLOR = (0.70, 0.70, 0.72, 1.0)
PLACEHOLDER_COLOR = (0.85, 0.20, 0.55, 1.0)  # magenta: "this mesh could not be loaded"
MAX_SCENE_BYTES = 56 * 1024 * 1024  # stay under the 64 MiB frame limit of the protocol
XACRO_TIMEOUT_SECONDS = 30


class UrdfError(Exception):
    """The file cannot be imported at all. The message is meant for the user."""


@dataclass
class ImportReport:
    """What the importer could not use. Shown to the user after an import."""

    notes: list[str] = field(default_factory=list)

    def add(self, note: str) -> None:
        if note not in self.notes:
            self.notes.append(note)


def _floats(text: str | None, count: int, default: tuple[float, ...], what: str) -> np.ndarray:
    if text is None:
        return np.array(default, dtype=np.float64)
    try:
        values = np.array(text.split(), dtype=np.float64)
    except ValueError as error:
        raise UrdfError(f"{what}: '{text}' is not a list of numbers") from error
    if len(values) != count or not np.all(np.isfinite(values)):
        raise UrdfError(f"{what}: expected {count} finite numbers, got '{text}'")
    return values


def _origin(element: ET.Element | None, what: str) -> np.ndarray:
    """4x4 transform of an <origin xyz rpy> (fixed-axis roll, pitch, yaw: R = Rz * Ry * Rx)."""
    matrix = np.eye(4)
    if element is None:
        return matrix
    xyz = _floats(element.get("xyz"), 3, (0.0, 0.0, 0.0), f"{what} origin xyz")
    roll, pitch, yaw = _floats(element.get("rpy"), 3, (0.0, 0.0, 0.0), f"{what} origin rpy")
    cr, sr, cp, sp, cy, sy = (
        np.cos(roll),
        np.sin(roll),
        np.cos(pitch),
        np.sin(pitch),
        np.cos(yaw),
        np.sin(yaw),
    )
    matrix[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    matrix[:3, 3] = xyz
    return matrix


def _quaternion(rotation: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) of a rotation matrix."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def read_description(path: Path, xacro_args: tuple[str, ...] = ()) -> str:
    """The URDF text of `path`, running xacro first for .xacro files."""
    if not path.is_file():
        raise UrdfError(f"{path}: no such file")
    if path.suffix == ".xacro" or path.name.endswith(".urdf.xacro"):
        xacro = shutil.which("xacro")
        if xacro is None:
            raise UrdfError("this is a xacro file, but the 'xacro' program is not installed")
        try:
            result = subprocess.run(
                [xacro, str(path), *xacro_args],
                capture_output=True,
                text=True,
                timeout=XACRO_TIMEOUT_SECONDS,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise UrdfError(f"xacro did not finish within {XACRO_TIMEOUT_SECONDS} s") from error
        if result.returncode != 0:
            last = result.stderr.strip().splitlines()[-1] if result.stderr.strip() else "no message"
            raise UrdfError(f"xacro failed: {last}")
        return result.stdout
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise UrdfError(f"cannot read {path}: {error}") from error


def _expand_if_xacro(path: Path, text: str, report: ImportReport) -> str:
    """Some files named .urdf still contain xacro macros. Expand them when possible."""
    if "xmlns:xacro" not in text or path.suffix == ".xacro":
        return text
    xacro = shutil.which("xacro")
    if xacro is None:
        report.add("the file uses xacro macros but 'xacro' is not installed; they are not expanded")
        return text
    try:
        result = subprocess.run(
            [xacro, str(path)],
            capture_output=True,
            text=True,
            timeout=XACRO_TIMEOUT_SECONDS,
            check=False,
        )
    except subprocess.TimeoutExpired:
        result = None
    if result is None or result.returncode != 0 or not result.stdout.strip():
        report.add(
            "the file uses xacro macros that could not be expanded; it is read as plain URDF"
        )
        return text
    return result.stdout


def resolve_mesh_path(uri: str, urdf_dir: Path) -> Path | None:
    """Local path of a mesh URI: package://pkg/..., file://..., or a plain path."""
    if uri.startswith("package://"):
        package, _, relative = uri[len("package://") :].partition("/")
        # 1. An installed ROS package.
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            candidate = Path(prefix) / "share" / package / relative
            if prefix and candidate.is_file():
                return candidate
        # 2. A source checkout: a directory named like the package, at or above the URDF.
        for ancestor in [urdf_dir, *urdf_dir.parents]:
            if ancestor.name == package and (ancestor / relative).is_file():
                return ancestor / relative
            if (ancestor / package / relative).is_file():
                return ancestor / package / relative
        return None
    candidate = Path(uri[len("file://") :]) if uri.startswith("file://") else Path(uri)
    if not candidate.is_absolute():
        candidate = urdf_dir / candidate
    return candidate if candidate.is_file() else None


class _SceneBuilder:
    def __init__(self, stage_path: str) -> None:
        self.snapshot = scene_pb2.SceneSnapshot(sequence=1, stage_path=stage_path)
        self.transforms = bytearray()
        self._mesh_ids: dict[object, int] = {}
        self._material_ids: dict[tuple[float, ...], int] = {}
        self.payload_bytes = 0

    def prim(
        self,
        name: str,
        parent: int,
        type_name: str,
        matrix: np.ndarray,
        scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
        mesh: int = NO_PARENT,
        material: int = NO_PARENT,
    ) -> int:
        prim_id = len(self.snapshot.prims)
        parent_path = self.snapshot.prims[parent].path if parent != NO_PARENT else ""
        self.snapshot.prims.add(
            id=prim_id,
            parent=parent,
            name=name,
            path=f"{parent_path}/{name}",
            type_name=type_name,
            mesh=mesh,
            material=material,
            visible=True,
        )
        if parent != NO_PARENT:
            self.snapshot.prims[parent].child_count += 1
        self.transforms += matrix[:3, 3].astype("<f8").tobytes()
        self.transforms += _quaternion(matrix[:3, :3]).astype("<f4").tobytes()
        self.transforms += np.array(scale, dtype="<f4").tobytes()
        return prim_id

    def mesh(self, key: object, build) -> int:
        """Id of the mesh for `key`, building it on first use so that shared meshes are sent once."""
        if key not in self._mesh_ids:
            geometry: meshes.TriMesh = build()
            mesh = self.snapshot.meshes.add(
                id=len(self.snapshot.meshes),
                vertex_count=len(geometry.positions),
                index_count=len(geometry.indices),
            )
            for ref, array in (
                (mesh.positions, geometry.positions),
                (mesh.normals, geometry.normals),
                (mesh.indices, geometry.indices),
            ):
                data = np.ascontiguousarray(array).tobytes()
                ref.inline = data
                ref.size = len(data)
                self.payload_bytes += len(data)
            if len(geometry.positions):
                mesh.aabb.extend(
                    [
                        *geometry.positions.min(axis=0).tolist(),
                        *geometry.positions.max(axis=0).tolist(),
                    ]
                )
            self._mesh_ids[key] = mesh.id
        return self._mesh_ids[key]

    def material(self, name: str, rgba: tuple[float, ...]) -> int:
        key = tuple(round(float(c), 4) for c in rgba)
        if key not in self._material_ids:
            material = self.snapshot.materials.add(
                id=len(self.snapshot.materials), name=name, base_color=list(key), roughness=0.6
            )
            self._material_ids[key] = material.id
        return self._material_ids[key]


def import_urdf(
    path: Path | str, xacro_args: tuple[str, ...] = ()
) -> tuple[Recording, ImportReport]:
    """Build a static scene from a URDF or xacro file."""
    path = Path(path).expanduser().resolve()
    report = ImportReport()
    text = _expand_if_xacro(path, read_description(path, xacro_args), report)
    try:
        robot = ET.fromstring(text)
    except ET.ParseError as error:
        raise UrdfError(f"{path.name} is not valid XML: {error}") from error
    if robot.tag != "robot":
        raise UrdfError(f"{path.name}: the top element is <{robot.tag}>, not <robot>; not a URDF")
    robot_name = robot.get("name") or path.stem

    # Materials defined once at the top and referred to by name.
    named_colors: dict[str, tuple[float, ...]] = {}

    def color_of(material: ET.Element | None, what: str) -> tuple[str, tuple[float, ...]]:
        if material is None:
            return "default", DEFAULT_COLOR
        name = material.get("name", "")
        color = material.find("color")
        if color is not None:
            rgba = tuple(_floats(color.get("rgba"), 4, DEFAULT_COLOR, f"{what} colour"))
            if name:
                named_colors[name] = rgba
            return name or "unnamed", rgba
        if material.find("texture") is not None:
            report.add("textures are not loaded; textured materials are shown in grey")
        if name in named_colors:
            return name, named_colors[name]
        if name:
            report.add(f"material '{name}' is used but never given a colour")
        return name or "default", DEFAULT_COLOR

    for material in robot.findall("material"):
        color_of(material, f"material '{material.get('name', '')}'")

    links = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name:
            raise UrdfError("a <link> has no name")
        if name in links:
            raise UrdfError(f"two links are named '{name}'")
        links[name] = link
    if not links:
        raise UrdfError(f"{path.name} has no links")

    children: dict[str, list[tuple[str, str, np.ndarray, str]]] = {name: [] for name in links}
    has_parent: set[str] = set()
    for joint in robot.findall("joint"):
        joint_name = joint.get("name", "?")
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise UrdfError(f"joint '{joint_name}' needs both <parent> and <child>")
        parent_link, child_link = parent.get("link", ""), child.get("link", "")
        for link_name in (parent_link, child_link):
            if link_name not in links:
                raise UrdfError(
                    f"joint '{joint_name}' refers to link '{link_name}', which does not exist"
                )
        if child_link in has_parent:
            raise UrdfError(f"link '{child_link}' is the child of more than one joint")
        has_parent.add(child_link)
        children[parent_link].append(
            (
                joint_name,
                child_link,
                _origin(joint.find("origin"), f"joint '{joint_name}'"),
                joint.get("type", "fixed"),
            )
        )
    roots = [name for name in links if name not in has_parent]
    if not roots:
        raise UrdfError("the joints form a loop: no link is without a parent")
    if len(roots) > 1:
        report.add(
            f"{len(roots)} links have no parent ({', '.join(roots[:4])}); each is placed at the origin"
        )

    for skipped in ("gazebo", "transmission", "ros2_control"):
        if robot.find(skipped) is not None:
            report.add(f"<{skipped}> elements are not used by the viewer")

    builder = _SceneBuilder(str(path))
    robot_prim = builder.prim(robot_name, NO_PARENT, "Xform", np.eye(4))
    movable = 0

    def add_link(
        name: str, parent_prim: int, world: np.ndarray, joint_type: str, seen: frozenset[str]
    ) -> None:
        nonlocal movable
        if name in seen:
            raise UrdfError(f"the joints form a loop through link '{name}'")
        link_prim = builder.prim(name, parent_prim, "Xform", world)
        if joint_type not in ("fixed", "root"):
            builder.snapshot.prims[link_prim].api_schemas.append(f"joint:{joint_type}")
            movable += 1
        for index, visual in enumerate(links[name].findall("visual")):
            what = f"link '{name}' visual {index}"
            geometry = visual.find("geometry")
            shape = next(iter(geometry), None) if geometry is not None else None
            if shape is None:
                report.add(f"{what} has no geometry")
                continue
            material_name, rgba = color_of(visual.find("material"), what)
            scale = (1.0, 1.0, 1.0)
            if shape.tag == "box":
                size = _floats(shape.get("size"), 3, (1.0, 1.0, 1.0), f"{what} box size")
                mesh = builder.mesh(("box", *size), lambda s=size: meshes.box(*s))
            elif shape.tag == "cylinder":
                radius = float(_floats(shape.get("radius"), 1, (0.5,), f"{what} radius")[0])
                length = float(_floats(shape.get("length"), 1, (1.0,), f"{what} length")[0])
                mesh = builder.mesh(
                    ("cylinder", radius, length), lambda r=radius, h=length: meshes.cylinder(r, h)
                )
            elif shape.tag == "sphere":
                radius = float(_floats(shape.get("radius"), 1, (0.5,), f"{what} radius")[0])
                mesh = builder.mesh(("sphere", radius), lambda r=radius: meshes.sphere(r))
            elif shape.tag == "mesh":
                uri = shape.get("filename", "")
                scale = tuple(_floats(shape.get("scale"), 3, (1.0, 1.0, 1.0), f"{what} mesh scale"))
                file = resolve_mesh_path(uri, path.parent)
                try:
                    if file is None:
                        raise meshes.MeshError("file not found")
                    mesh = builder.mesh(
                        ("file", str(file)), lambda f=file: meshes.load_mesh_file(f)
                    )
                except meshes.MeshError as error:
                    report.add(
                        f"mesh '{uri}' could not be loaded ({error}); a magenta box is shown instead"
                    )
                    mesh = builder.mesh(("placeholder",), lambda: meshes.box(0.1, 0.1, 0.1))
                    material_name, rgba, scale = "missing-mesh", PLACEHOLDER_COLOR, (1.0, 1.0, 1.0)
            else:
                report.add(f"{what}: unknown geometry <{shape.tag}>")
                continue
            if builder.payload_bytes > MAX_SCENE_BYTES:
                raise UrdfError(
                    f"the robot's meshes are larger than {MAX_SCENE_BYTES // (1024 * 1024)} MiB in total, "
                    "more than the viewer protocol carries in one message; simplify the meshes"
                )
            visual_world = world @ _origin(visual.find("origin"), what)
            builder.prim(
                f"visual_{index}" if index else "visual",
                link_prim,
                "Mesh",
                visual_world,
                scale=scale,
                mesh=mesh,
                material=builder.material(material_name, rgba),
            )
        for _joint_name, child, origin, child_type in children[name]:
            add_link(child, link_prim, world @ origin, child_type, seen | {name})

    for root in roots:
        add_link(root, robot_prim, np.eye(4), "root", frozenset())
    if movable:
        report.add(f"{movable} movable joints are shown at their zero position")

    builder.snapshot.transforms.inline = bytes(builder.transforms)
    builder.snapshot.transforms.size = len(builder.transforms)
    meta = record_pb2.RunMeta(
        format_version=FORMAT_VERSION,
        stage_path=str(path),
        generator="lagrange_mock_server.urdf",
        synthetic=False,
        description=f"Static scene imported from {path.name}",
    )
    recording = Recording(meta=meta, snapshot=builder.snapshot, signals=[])
    recording.validate()
    return recording, report
