"""The robot builder's URDF against the server's loader (RFC 0016 test plan 3, WP-B).

- Read then write is semantically the same (elements, attributes, numbers) for every URDF under
  assets/robots, the templates included.
- Every golden file and template loads with no notes, and `lagrange_import F.urdf OUT.usda`
  gives each body the file's mass, centre of mass and inertia, and each joint its type, axis
  and limits (read from the .usda text).

lagrange_import runs the server's own loader (server::loadModel, the one lagrange_server uses).
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
ASSETS = ROOT / "assets" / "robots"
GOLDEN = HERE / "builder" / "golden"
TEMPLATES = ASSETS / "templates"

TOOL = os.environ.get("LAGRANGE_BUILDER_URDF_TOOL", "")

# The built-in URDF sensor types (server/src/Robot.cpp addSensors); others are kinds.
BUILTIN_TYPES = {
    "imu",
    "ray",
    "gpu_ray",
    "lidar",
    "gpu_lidar",
    "camera",
    "depth",
    "depth_camera",
    "rgbd_camera",
    "thermal",
    "thermal_camera",
}
NOT_PARAMETERS = {"always_on", "visualize", "topic", "plugin", "enable_metrics"}


def _urdfs() -> list[Path]:
    return sorted(ASSETS.rglob("*.urdf"))


def _parse(path: Path) -> ET.Element:
    # Comments are left out of the comparison (the gtests check they are kept); some samples
    # have "--" inside comments, which a strict parser refuses.
    text = re.sub(r"<!--.*?-->", "", path.read_text(), flags=re.DOTALL)
    return ET.fromstring(text)


def _value(text: str | None) -> Any:
    words = (text or "").split()
    try:
        numbers = [float(w) for w in words]
    except ValueError:
        return " ".join(words)
    return numbers


def _element(e: ET.Element) -> Any:
    """An element as nested data: tag, attributes (numbers parsed), text, children in order."""
    return (
        e.tag,
        tuple(sorted((k, _value(v)) for k, v in e.attrib.items())),
        _value(e.text),
        tuple(_element(c) for c in e),
    )


def _close(a: Any, b: Any, rel: float = 1e-9) -> bool:
    if isinstance(a, float) and isinstance(b, float):
        return math.isclose(a, b, rel_tol=rel, abs_tol=1e-12)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_close(x, y, rel) for x, y in zip(a, b, strict=True))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], rel) for k in a)
    return bool(a == b)


def _diff(a: Any, b: Any, where: str = "") -> str:
    if isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys():
            return f"{where}: keys {sorted(a.keys() ^ b.keys())}"
        for k in a:
            if not _close(a[k], b[k]):
                return _diff(a[k], b[k], f"{where}/{k}")
    return f"{where}: {a!r} != {b!r}"


def _rotation(rpy: list[float]) -> list[list[float]]:
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _origin(e: ET.Element | None) -> tuple[list[float], list[float]]:
    o = e.find("origin") if e is not None else None
    if o is None:
        return [0.0] * 3, [0.0] * 3
    return _value(o.get("xyz", "0 0 0")), _value(o.get("rpy", "0 0 0"))


def _inertial(link: ET.Element) -> dict[str, Any] | None:
    inertial = link.find("inertial")
    if inertial is None:
        return None
    xyz, rpy = _origin(inertial)
    mass = inertial.find("mass")
    i = inertial.find("inertia")
    keys = ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]
    v = [float(i.get(k, "0")) if i is not None else 0.0 for k in keys]
    m = [[v[0], v[1], v[2]], [v[1], v[3], v[4]], [v[2], v[4], v[5]]]
    r = _rotation(rpy)
    tensor = [
        [sum(r[a][k] * m[k][n] * r[b][n] for k in range(3) for n in range(3)) for b in range(3)]
        for a in range(3)
    ]
    return {
        "mass": float(mass.get("value", "0")) if mass is not None else None,
        "com": xyz,
        "inertia": [
            tensor[0][0],
            tensor[0][1],
            tensor[0][2],
            tensor[1][1],
            tensor[1][2],
            tensor[2][2],
        ],
    }


def _shape(e: ET.Element, materials: dict[str, list[float]]) -> dict[str, Any]:
    xyz, rpy = _origin(e)
    g = e.find("geometry")
    geometry = _element(g[0]) if g is not None and len(g) else None
    if geometry is not None and geometry[0] == "mesh":
        attrs = dict(geometry[1])
        attrs.setdefault("scale", [1.0, 1.0, 1.0])
        geometry = ("mesh", tuple(sorted(attrs.items())))
    out: dict[str, Any] = {"xyz": xyz, "rpy": rpy, "geometry": geometry}
    m = e.find("material")
    if e.tag == "visual" and m is not None:
        name = m.get("name")
        colour = m.find("color")
        if name in materials:
            out["rgba"] = materials[name]
        elif colour is not None:
            out["rgba"] = _value(colour.get("rgba"))
            if name:
                materials[name] = out["rgba"]
    return out


def _sensor(reference: str, s: ET.Element) -> dict[str, Any]:
    kind = s.get("type", "") not in BUILTIN_TYPES
    params: dict[str, Any] = {}

    def walk(e: ET.Element, prefix: str) -> None:
        for k, v in e.attrib.items():
            if prefix:
                params[f"{prefix}/@{k}"] = _value(v)
        for c in e:
            path = f"{prefix}/{c.tag}" if prefix else c.tag
            if not prefix and kind and c.tag == s.get("type"):
                walk(c, "")
                continue
            if len(c) == 0:
                params[path] = _value(c.text)
                for k, v in c.attrib.items():
                    params[f"{path}/@{k}"] = _value(v)
            else:
                walk(c, path)

    walk(s, "")
    pose = params.pop("pose", [0.0] * 6)
    rate = params.pop("update_rate", [10.0])
    return {
        "link": reference,
        "type": s.get("type"),
        "pose": pose,
        "rate": rate,
        "latency": params.pop("latency", None),
        "jitter": params.pop("jitter", None),
        "params": params,
    }


def canonical(path: Path) -> dict[str, Any]:
    """What a URDF says, in a form where equal meaning compares equal."""
    robot = _parse(path)
    materials: dict[str, list[float]] = {}
    for m in robot.findall("material"):
        colour = m.find("color")
        if colour is not None and m.get("name"):
            materials.setdefault(m.get("name", ""), _value(colour.get("rgba")))
    links: dict[str, Any] = {}
    joints: dict[str, Any] = {}
    sensors: dict[str, Any] = {}
    gazebo: list[Any] = []
    other: list[Any] = []
    for e in robot:
        if e.tag == "link":
            kept = [
                _element(c)
                for c in e
                if c.tag not in ("inertial", "visual", "collision")
                or (
                    c.tag in ("visual", "collision")
                    and (g := c.find("geometry")) is not None
                    and len(g)
                    and g[0].tag not in ("box", "cylinder", "sphere", "mesh")
                )
            ]
            shapes = {
                tag: [
                    _shape(c, materials)
                    for c in e.findall(tag)
                    if (g := c.find("geometry")) is not None
                    and len(g)
                    and g[0].tag in ("box", "cylinder", "sphere", "mesh")
                ]
                for tag in ("visual", "collision")
            }
            links[e.get("name", "")] = {"inertial": _inertial(e), **shapes, "kept": kept}
        elif e.tag == "joint" and e.get("type") in ("fixed", "revolute", "continuous", "prismatic"):
            axis = e.find("axis")
            a = _value(axis.get("xyz")) if axis is not None else [1.0, 0.0, 0.0]
            n = math.sqrt(sum(x * x for x in a)) or 1.0
            limit = e.find("limit")
            dynamics = e.find("dynamics")
            mimic = e.find("mimic")
            xyz, rpy = _origin(e)
            joints[e.get("name", "")] = {
                "type": e.get("type"),
                "parent": e.find("parent").get("link"),  # type: ignore[union-attr]
                "child": e.find("child").get("link"),  # type: ignore[union-attr]
                "xyz": xyz,
                "rpy": rpy,
                "axis": [x / n for x in a],
                "limit": {k: _value(v) for k, v in limit.attrib.items()}
                if limit is not None
                else {},
                "dynamics": [
                    float(dynamics.get("damping", "0")) if dynamics is not None else 0.0,
                    float(dynamics.get("friction", "0")) if dynamics is not None else 0.0,
                ],
                "mimic": (
                    [
                        mimic.get("joint"),
                        float(mimic.get("multiplier", "1")),
                        float(mimic.get("offset", "0")),
                    ]
                    if mimic is not None
                    else None
                ),
                "kept": [
                    _element(c)
                    for c in e
                    if c.tag
                    not in ("parent", "child", "origin", "axis", "limit", "dynamics", "mimic")
                ],
            }
        elif e.tag == "gazebo" and e.get("reference") is not None:
            for c in e:
                if c.tag == "sensor" and c.get("name") and c.get("type"):
                    sensors[c.get("name", "")] = _sensor(e.get("reference", ""), c)
                else:
                    gazebo.append((e.get("reference"), _element(c)))
        elif e.tag != "material":
            other.append(_element(e))
    return {
        "robot": robot.get("name"),
        "links": links,
        "joints": joints,
        "sensors": sensors,
        "gazebo": sorted(gazebo, key=repr),
        "other": other,
    }


def _tool(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([TOOL, *args], capture_output=True, text=True, timeout=60, check=False)


@pytest.mark.skipif(not TOOL, reason="LAGRANGE_BUILDER_URDF_TOOL not set (run through colcon test)")
@pytest.mark.parametrize("path", _urdfs(), ids=lambda p: str(p.relative_to(ASSETS)))
def test_read_then_write_is_the_same(path: Path, tmp_path: Path) -> None:
    out = tmp_path / path.name
    run = _tool("roundtrip", str(path), str(out))
    assert run.returncode == 0, run.stderr
    a, b = canonical(path), canonical(out)
    assert _close(a, b), _diff(a, b)


def test_every_sample_and_template_is_compared() -> None:
    names = {p.relative_to(ASSETS).as_posix() for p in _urdfs()}
    assert len(names) >= 14, names
    for template in ("diff_drive_base", "arm_3dof", "pan_tilt"):
        assert f"templates/{template}.urdf" in names


def _import_tool() -> str | None:
    try:
        from ament_index_python.packages import get_package_prefix

        found = (
            Path(get_package_prefix("lagrange_server"))
            / "lib"
            / "lagrange_server"
            / "lagrange_import"
        )
        return str(found) if found.exists() else None
    except Exception:
        return shutil.which("lagrange_import")


LOADED = sorted(GOLDEN.glob("*.urdf")) + sorted(TEMPLATES.glob("*.urdf"))


def _quat_rotate(q: list[float], v: list[float]) -> list[float]:
    w, x, y, z = q
    r = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]
    return [sum(r[a][k] * v[k] for k in range(3)) for a in range(3)]


def _tuple(text: str) -> list[float]:
    return [float(x) for x in text.strip("() ").split(",")]


def _usd_bodies(usda: str) -> dict[str, dict[str, Any]]:
    bodies = {}
    for m in re.finditer(
        r'def Xform "(\w+)" \(\s*prepend apiSchemas = \[[^\]]*"PhysicsMassAPI"[^\]]*\]\s*\)'
        r"\s*\{(.*?)\n\s*(?:def |\})",
        usda,
        flags=re.DOTALL,
    ):
        body: dict[str, Any] = {}
        for key, value in re.findall(
            r"physics:(mass|centerOfMass|diagonalInertia|principalAxes) = (.+)", m.group(2)
        ):
            body[key] = float(value) if key == "mass" else _tuple(value)
        bodies[m.group(1)] = body
    return bodies


def _usd_joints(usda: str) -> list[dict[str, Any]]:
    joints = []
    for m in re.finditer(
        r'def (Physics\w+Joint) "(\w+)"\s*(?:\([^)]*\)\s*)?\{(.*?)\n\s*\}', usda, flags=re.DOTALL
    ):
        block = m.group(3)
        body1 = re.search(r"physics:body1 = <([^>]*)>", block)
        rot1 = re.search(r"physics:localRot1 = (\([^)]*\))", block)
        lower = re.search(r"physics:lowerLimit = (\S+)", block)
        upper = re.search(r"physics:upperLimit = (\S+)", block)
        mimic = re.search(r'joint:mimic = "(\w+)"', block)
        damping = re.search(r"joint:damping = (\S+)", block)
        joints.append(
            {
                "type": m.group(1),
                "child": body1.group(1).rsplit("/", 1)[-1] if body1 else None,
                "rot1": _tuple(rot1.group(1)) if rot1 else [1.0, 0.0, 0.0, 0.0],
                "lower": float(lower.group(1)) if lower else None,
                "upper": float(upper.group(1)) if upper else None,
                "mimic": mimic.group(1) if mimic else None,
                "damping": float(damping.group(1)) if damping else 0.0,
            }
        )
    return joints


@pytest.mark.parametrize("path", LOADED, ids=lambda p: p.name)
def test_loads_with_no_notes_and_imports_the_same_bodies_and_joints(
    path: Path, tmp_path: Path
) -> None:
    tool = _import_tool()
    if tool is None:
        pytest.skip("lagrange_import is not installed")
    out = tmp_path / (path.stem + ".usda")
    run = subprocess.run(
        [tool, str(path), str(out)], capture_output=True, text=True, timeout=120, check=False
    )
    assert run.returncode == 0, run.stderr
    notes = [line for line in run.stderr.splitlines() if line.startswith("note:")]
    assert notes == [], notes
    usda = out.read_text()
    doc = canonical(path)
    bodies = _usd_bodies(usda)
    for name, link in doc["links"].items():
        if link["inertial"] is None:
            continue  # "world": not a body
        assert name in bodies, f"{name} is not a body in {sorted(bodies)}"
        body = bodies[name]
        expected = link["inertial"]
        assert math.isclose(body["mass"], expected["mass"], rel_tol=1e-6), name
        assert _close(body["centerOfMass"], expected["com"], rel=1e-5), name
        # I = R diag(d) Rᵀ, R from the principal axes quaternion (w, x, y, z).
        d = body["diagonalInertia"]
        q = body["principalAxes"]
        columns = [_quat_rotate(q, e) for e in ([1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0])]
        tensor = [
            [sum(columns[k][a] * d[k] * columns[k][b] for k in range(3)) for b in range(3)]
            for a in range(3)
        ]
        got = [tensor[0][0], tensor[0][1], tensor[0][2], tensor[1][1], tensor[1][2], tensor[2][2]]
        scale = max(abs(x) for x in expected["inertia"]) or 1.0
        assert all(
            abs(g - e) <= 1e-5 * scale for g, e in zip(got, expected["inertia"], strict=True)
        ), (
            name,
            got,
            expected,
        )
    usd_joints = {j["child"]: j for j in _usd_joints(usda)}
    kinds = {
        "fixed": "PhysicsFixedJoint",
        "revolute": "PhysicsRevoluteJoint",
        "continuous": "PhysicsRevoluteJoint",
        "prismatic": "PhysicsPrismaticJoint",
    }
    for name, joint in doc["joints"].items():
        usd = usd_joints.get(joint["child"])
        assert usd is not None, f"joint {name}: no USD joint to {joint['child']}"
        assert usd["type"] == kinds[joint["type"]], name
        if joint["type"] == "fixed":
            continue
        assert usd["mimic"] == (joint["mimic"][0] if joint["mimic"] else None), name
        assert math.isclose(usd["damping"], joint["dynamics"][0]), name
        axis = _quat_rotate(usd["rot1"], [1.0, 0.0, 0.0])  # the USD joint's X in the child
        assert _close(axis, joint["axis"], rel=1e-5) or all(
            abs(a - b) < 1e-6 for a, b in zip(axis, joint["axis"], strict=True)
        ), (
            name,
            axis,
            joint["axis"],
        )
        if joint["type"] in ("revolute", "prismatic"):
            scale = 180.0 / math.pi if joint["type"] == "revolute" else 1.0
            lower, upper = joint["limit"]["lower"][0], joint["limit"]["upper"][0]
            assert usd["lower"] is not None and math.isclose(
                usd["lower"], lower * scale, rel_tol=1e-5, abs_tol=1e-5
            )
            assert usd["upper"] is not None and math.isclose(
                usd["upper"], upper * scale, rel_tol=1e-5, abs_tol=1e-5
            )
        else:
            assert usd["lower"] is None, name
