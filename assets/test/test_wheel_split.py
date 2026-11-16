# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: E501  (the Collada template's lines are as long as Collada's)
"""wheel_split cuts a one-piece car mesh into a body and wheels re-centred on their centres."""

import os
import re
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ASSETS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ASSETS / "scripts"))

import wheel_split  # noqa: E402

SPEC = yaml.safe_load((ASSETS / "external_assets.yaml").read_text())["assets"]["lexus_rx450h"][
    "derive"
]["wheel_split"]
TYRE_RADIUS = 0.383  # m, the URDF's (Autoware's vehicle_info)


def to_base(spec: dict, points: np.ndarray) -> np.ndarray:
    return points @ wheel_split.rotation(spec["frame"]["rpy"]).T + spec["frame"]["xyz"]


def to_source(spec: dict, points: np.ndarray) -> np.ndarray:
    return (points - spec["frame"]["xyz"]) @ wheel_split.rotation(spec["frame"]["rpy"])


def synthetic_dae(path: Path, spec: dict) -> dict:
    """A box body, a 24-sided tyre at each wheel and one triangle that straddles a tyre's edge."""
    triangles: list[np.ndarray] = []  # base frame
    corners = np.array([[x, y, z] for x in (-2, 2) for y in (-0.5, 0.5) for z in (0.0, 1.0)])
    for a, b, c in [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1)]:
        triangles.append(corners[[a, b, c]])
    expected = {"body": 7}
    for name, centre in spec["wheels"].items():
        c = np.array(centre, dtype=float)
        side = 1.0 if c[1] >= 0 else -1.0
        n = 24
        for k in range(n):
            a0, a1 = 2 * np.pi * k / n, 2 * np.pi * (k + 1) / n
            rim = [np.array([0.38 * np.cos(a), 0.0, 0.38 * np.sin(a)]) for a in (a0, a1)]
            inner, outer = np.array([0, -0.11 * side, 0]), np.array([0, 0.11 * side, 0])
            triangles.append(c + np.array([rim[0] + inner, rim[1] + inner, rim[1] + outer]))
            triangles.append(c + np.array([rim[0] + inner, rim[1] + outer, rim[0] + outer]))
            triangles.append(c + np.array([outer, rim[0] + outer, rim[1] + outer]))  # hub face
        expected[name] = 3 * n
    c = np.array(next(iter(spec["wheels"].values())), dtype=float)
    triangles.append(c + np.array([[0, 0, 0.2], [0.1, 0, 0.2], [0, 0, 0.8]]))  # straddles: body
    points = to_source(spec, np.concatenate(triangles))
    numbers = " ".join(f"{v:.6f}" for v in points.ravel())
    uvs = " ".join(f"{(i % 7) / 7:.4f} {(i % 5) / 5:.4f}" for i in range(len(points)))
    p = " ".join(f"{i} {i}" for i in range(len(points)))
    path.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
    <asset><up_axis>Y_UP</up_axis></asset>
    <library_images><image id="texture0"><init_from>car.jpg</init_from></image></library_images>
    <library_geometries>
        <geometry id="shape0-lib" name="shape0"><mesh>
            <source id="shape0-lib-positions"><float_array id="shape0-lib-positions-array" count="{3 * len(points)}">{numbers}</float_array></source>
            <source id="shape0-lib-map"><float_array id="shape0-lib-map-array" count="{2 * len(points)}">{uvs}</float_array></source>
            <vertices id="shape0-lib-vertices"><input semantic="POSITION" source="#shape0-lib-positions"/></vertices>
            <triangles count="{len(points) // 3}" material="material0">
                <input offset="0" semantic="VERTEX" source="#shape0-lib-vertices"/>
                <input offset="1" semantic="TEXCOORD" source="#shape0-lib-map"/>
                <p>{p}</p>
            </triangles>
        </mesh></geometry>
    </library_geometries>
    <library_visual_scenes><visual_scene id="s"><node id="node"><instance_geometry url="#shape0-lib"/></node></visual_scene></library_visual_scenes>
</COLLADA>
""")
    return expected


def check_parts(directory: Path, spec: dict, width: tuple[float, float]) -> dict:
    """Triangle counts per part; checks each wheel's box and that the body leaves the wheels."""
    counts = {}
    body = wheel_split.read(directory / spec["body"])
    assert "lexus.jpg" in body.head or "car.jpg" in body.head  # the texture is kept
    counts["body"] = len(body.triangles)
    body_corners = to_base(spec, body.positions[body.triangles[:, :, 0]])
    for name, centre in spec["wheels"].items():
        wheel = wheel_split.read(directory / name)
        counts[name] = len(wheel.triangles)
        assert len(wheel.uvs) > 0
        # The part in its link's frame (the URDF's visual rotation, no offset).
        points = wheel.positions @ wheel_split.rotation(spec["frame"]["rpy"]).T
        lo, hi = points.min(axis=0), points.max(axis=0)
        # Upright, the tyre's diameter; along x a coarse tread triangle's corners may reach past
        # the tyre (up to the split's vertex_radius).
        assert hi[2] - lo[2] == pytest.approx(2 * TYRE_RADIUS, abs=0.03), (name, lo, hi)
        assert 2 * TYRE_RADIUS - 0.03 < hi[0] - lo[0] < 2 * spec["vertex_radius"], (name, lo, hi)
        assert abs((hi[0] + lo[0]) / 2) < 0.03 and abs((hi[2] + lo[2]) / 2) < 0.03, (name, lo, hi)
        print(name, "box", lo.round(3), hi.round(3))
        assert width[0] <= hi[1] - lo[1] <= width[1], (name, lo, hi)
        # No body triangle lies inside the tyre's own cylinder.
        offset = body_corners - np.array(centre, dtype=float)
        outward = offset[..., 1] * (1.0 if centre[1] >= 0 else -1.0)
        inside = (
            (np.hypot(offset[..., 0], offset[..., 2]) < TYRE_RADIUS)
            & (outward < spec["outboard"])
            & (outward > -spec["inboard"])
        ).all(axis=1)
        assert not inside.any(), name
    return counts


def test_split_of_a_synthetic_car(tmp_path: Path) -> None:
    spec = {**SPEC, "source": "car.dae"}
    expected = synthetic_dae(tmp_path / "car.dae", spec)
    assert wheel_split.derive(tmp_path, spec)
    assert check_parts(tmp_path, spec, (0.21, 0.23)) == expected
    first = {n: (tmp_path / n).read_bytes() for n in wheel_split.outputs(spec)}
    assert not wheel_split.derive(tmp_path, spec)  # up to date: nothing written
    (tmp_path / spec["body"]).unlink()  # an older copy without the parts
    assert wheel_split.derive(tmp_path, spec)
    assert {n: (tmp_path / n).read_bytes() for n in wheel_split.outputs(spec)} == first
    # Every triangle is in exactly one part.
    total = sum(expected.values())
    assert sum(len(wheel_split.read(tmp_path / n).triangles) for n in first) == total
    assert re.search(r'url="#lexus_body-lib"', (tmp_path / spec["body"]).read_text())


def fetched_lexus() -> Path:
    root = os.environ.get("LAGRANGE_ASSET_DIR") or str(
        Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share")
        / "lagrange"
        / "assets"
    )
    return Path(root) / "lexus_rx450h" / "lexus.dae"


@pytest.mark.skipif(not fetched_lexus().is_file(), reason="lexus_rx450h is not fetched")
def test_split_of_the_lexus(tmp_path: Path) -> None:
    source = fetched_lexus()
    (tmp_path / "lexus.dae").symlink_to(source)
    assert wheel_split.derive(tmp_path, SPEC)
    # The mesh's tyres run inboard of the 0.235 m tread (its coarse tyre is a 0.3-0.4 m drum).
    counts = check_parts(tmp_path, SPEC, (0.2, 0.43))
    print(counts)
    assert sum(counts.values()) == len(wheel_split.read(source).triangles)
    for name in SPEC["wheels"]:
        assert counts[name] > 500, (name, counts)
