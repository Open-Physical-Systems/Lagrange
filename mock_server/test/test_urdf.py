import asyncio
import struct
import tempfile
from pathlib import Path

import numpy as np
import pytest
from lagrange_mock_server import meshes
from lagrange_mock_server.client import ProtocolClient
from lagrange_mock_server.endpoint import Endpoint
from lagrange_mock_server.replay import ReplayEngine
from lagrange_mock_server.sample import generate_sample
from lagrange_mock_server.server import MockServer
from lagrange_mock_server.urdf import UrdfError, import_urdf, resolve_mesh_path

ARM = (
    Path(__file__).resolve().parents[2]
    / "assets"
    / "robots"
    / "primitives_arm"
    / "primitives_arm.urdf"
)


def prims_by_path(snapshot):
    return {prim.path: prim for prim in snapshot.prims}


def position(snapshot, prim):
    return np.frombuffer(snapshot.transforms.inline, dtype="<f8", count=3, offset=52 * prim.id)


def rotation(snapshot, prim):
    return np.frombuffer(snapshot.transforms.inline, dtype="<f4", count=4, offset=52 * prim.id + 24)


def scale(snapshot, prim):
    return np.frombuffer(snapshot.transforms.inline, dtype="<f4", count=3, offset=52 * prim.id + 40)


# --- the sample robot ------------------------------------------------------------------------


def test_sample_arm_tree_and_forward_kinematics():
    recording, report = import_urdf(ARM)
    snapshot = recording.snapshot
    prims = prims_by_path(snapshot)
    assert snapshot.prims[0].path == "/primitives_arm"
    assert "/primitives_arm/base/shoulder/forearm/tool/visual" in prims
    assert prims["/primitives_arm/base/shoulder"].parent == prims["/primitives_arm/base"].id
    assert prims["/primitives_arm/base"].child_count == 2  # its visual and the shoulder

    # Joint origins chain: base -> shoulder (0.10, 0, 0.10) -> forearm (+0.30 in z, yaw 90°).
    assert np.allclose(position(snapshot, prims["/primitives_arm/base/shoulder"]), [0.10, 0, 0.10])
    forearm = prims["/primitives_arm/base/shoulder/forearm"]
    assert np.allclose(position(snapshot, forearm), [0.10, 0, 0.40])
    assert np.allclose(rotation(snapshot, forearm), [np.sqrt(0.5), 0, 0, np.sqrt(0.5)], atol=1e-6)
    # The tool is 0.34 m along the forearm's X axis, which now points along world Y.
    tool = prims["/primitives_arm/base/shoulder/forearm/tool"]
    assert np.allclose(position(snapshot, tool), [0.10, 0.34, 0.40], atol=1e-9)
    # A visual's own origin is applied on top of its link.
    assert np.allclose(position(snapshot, prims["/primitives_arm/base/visual"]), [0, 0, 0.05])
    assert np.allclose(
        position(snapshot, prims["/primitives_arm/base/shoulder/forearm/visual_1"]),
        [0.10, 0.30, 0.40],
        atol=1e-9,
    )

    assert "joint:revolute" in prims["/primitives_arm/base/shoulder"].api_schemas
    assert "joint:continuous" in forearm.api_schemas
    assert not tool.api_schemas  # fixed joint
    assert report.notes == ["2 movable joints are shown at their zero position"]


def test_sample_arm_geometry_materials_and_mesh_scale():
    recording, _ = import_urdf(ARM)
    snapshot = recording.snapshot
    prims = prims_by_path(snapshot)
    assert len(snapshot.meshes) == 5  # base box, cylinder, forearm box, sphere, tool mesh
    colours = {m.name: tuple(round(c, 2) for c in m.base_color) for m in snapshot.materials}
    assert colours == {"steel": (0.55, 0.57, 0.60, 1.0), "orange": (0.90, 0.45, 0.10, 1.0)}

    tool_visual = prims["/primitives_arm/base/shoulder/forearm/tool/visual"]
    assert np.allclose(scale(snapshot, tool_visual), [0.001, 0.001, 0.001])  # millimetres
    tool_mesh = snapshot.meshes[tool_visual.mesh]
    assert tool_mesh.index_count == 12 and list(tool_mesh.aabb)[3] == 60.0
    for mesh in snapshot.meshes:
        normals = np.frombuffer(mesh.normals.inline, dtype="<f4").reshape(-1, 3)
        assert np.allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-5)
        indices = np.frombuffer(mesh.indices.inline, dtype="<u4")
        assert indices.max() < mesh.vertex_count
    assert len(snapshot.transforms.inline) == 52 * len(snapshot.prims)
    assert recording.duration_ns == 0 and not recording.signals


# --- problems in the file ----------------------------------------------------------------------


def write(tmp_path, body, name="robot.urdf"):
    path = tmp_path / name
    path.write_text(f'<?xml version="1.0"?>\n{body}', encoding="utf-8")
    return path


@pytest.mark.parametrize(
    ("body", "message"),
    [
        ("<robot name='r'>", "not valid XML"),
        ("<model name='r'/>", "not <robot>"),
        ("<robot name='r'/>", "no links"),
        ("<robot name='r'><link name='a'/><link name='a'/></robot>", "two links are named 'a'"),
        (
            "<robot name='r'><link name='a'/><joint name='j' type='fixed'><parent link='a'/>"
            "<child link='ghost'/></joint></robot>",
            "link 'ghost', which does not exist",
        ),
        (
            "<robot name='r'><link name='a'/><link name='b'/>"
            "<joint name='j1' type='fixed'><parent link='a'/><child link='b'/></joint>"
            "<joint name='j2' type='fixed'><parent link='b'/><child link='a'/></joint></robot>",
            "loop",
        ),
        (
            "<robot name='r'><link name='a'><visual><origin xyz='1 2'/><geometry><box size='1 1 1'/>"
            "</geometry></visual></link></robot>",
            "expected 3 finite numbers",
        ),
        (
            "<robot name='r'><link name='a'><visual><geometry><sphere radius='nan'/></geometry>"
            "</visual></link></robot>",
            "finite",
        ),
    ],
)
def test_malformed_files_give_a_clear_error(tmp_path, body, message):
    with pytest.raises(UrdfError, match=message):
        import_urdf(write(tmp_path, body))


def test_missing_file_and_missing_mesh(tmp_path):
    with pytest.raises(UrdfError, match="no such file"):
        import_urdf(tmp_path / "nothing.urdf")
    path = write(
        tmp_path,
        "<robot name='r'><link name='a'><visual><geometry>"
        "<mesh filename='package://nowhere/part.stl'/></geometry></visual></link></robot>",
    )
    recording, report = import_urdf(path)
    assert any("part.stl" in note and "magenta" in note for note in report.notes)
    assert recording.snapshot.materials[0].name == "missing-mesh"  # still loads, visibly wrong


def test_unused_elements_are_reported_not_fatal(tmp_path):
    path = write(
        tmp_path,
        "<robot name='r'><link name='a'><visual><geometry><box size='1 1 1'/></geometry>"
        "<material name='undefined'/></visual></link><link name='b'/>"
        "<gazebo/><ros2_control name='x' type='system'/></robot>",
    )
    _, report = import_urdf(path)
    text = " | ".join(report.notes)
    assert "'undefined' is used but never given a colour" in text
    assert "<gazebo>" in text and "<ros2_control>" in text
    assert "2 links have no parent" in text


def test_package_uris_resolve_in_a_source_checkout(tmp_path):
    package = tmp_path / "ws" / "my_robot_description"
    (package / "meshes").mkdir(parents=True)
    (package / "urdf").mkdir()
    (package / "meshes" / "part.stl").write_bytes(b"x")
    found = resolve_mesh_path("package://my_robot_description/meshes/part.stl", package / "urdf")
    assert found == package / "meshes" / "part.stl"
    assert resolve_mesh_path("package://other/meshes/part.stl", package / "urdf") is None
    assert resolve_mesh_path("../meshes/part.stl", package / "urdf").samefile(
        package / "meshes" / "part.stl"
    )
    assert (
        resolve_mesh_path(f"file://{package}/meshes/part.stl", tmp_path)
        == package / "meshes" / "part.stl"
    )


# --- mesh formats ----------------------------------------------------------------------------

TRIANGLE = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]


def test_binary_and_ascii_stl_agree():
    binary = b"solid looks like ascii but is binary".ljust(80, b" ") + struct.pack("<I", 1)
    binary += struct.pack("<12fH", 0, 0, 1, *np.array(TRIANGLE, dtype=float).ravel(), 0)
    ascii_text = b"solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n"
    for data in (binary, ascii_text):
        mesh = meshes.load_stl(data)
        assert mesh.triangle_count == 1
        assert np.allclose(mesh.positions, TRIANGLE)
        assert np.allclose(mesh.normals, [[0, 0, 1]] * 3)


def test_obj_polygons_are_triangulated_and_negative_indices_work():
    mesh = meshes.load_obj(
        b"v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1 4//1\nf -4 -3 -2\n"
    )
    assert mesh.triangle_count == 3
    assert np.allclose(mesh.normals[:, 2], 1.0)


def test_collada_units_up_axis_and_node_transform():
    dae = b"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="0.01"/><up_axis>Y_UP</up_axis></asset>
  <library_geometries><geometry id="g"><mesh>
    <source id="p"><float_array id="pa" count="12">0 0 0  100 0 0  100 100 0  0 100 0</float_array></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <polylist count="1"><input semantic="VERTEX" source="#v" offset="0"/><vcount>4</vcount><p>0 1 2 3</p></polylist>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node>
    <translate>0 0 200</translate><instance_geometry url="#g"/>
  </node></visual_scene></library_visual_scenes>
</COLLADA>"""
    mesh = meshes.load_dae(dae)
    assert mesh.triangle_count == 2
    # Centimetres to metres, moved 2 m along the file's Z, then Y-up to Z-up: (x, y, z) -> (x, -z, y).
    assert np.allclose(mesh.positions.min(axis=0), [0, -2, 0])
    assert np.allclose(mesh.positions.max(axis=0), [1, -2, 1])


@pytest.mark.parametrize(
    "loader,data",
    [
        (meshes.load_stl, b"not an stl at all \xff\xfe"),
        (meshes.load_stl, b"solid empty\nendsolid empty\n"),
        (meshes.load_obj, b"v 0 0 0\nf 1 2 3\n"),
        (meshes.load_obj, b"# nothing here\n"),
        (meshes.load_dae, b"<COLLADA><broken"),
        (meshes.load_dae, b"<COLLADA/>"),
    ],
)
def test_bad_mesh_files_raise_mesh_error(loader, data):
    with pytest.raises(meshes.MeshError):
        loader(data)


def test_primitives_have_the_right_size_and_outward_normals():
    for mesh, extent in (
        (meshes.box(2, 4, 6), [1, 2, 3]),
        (meshes.cylinder(0.5, 2), [0.5, 0.5, 1]),
        (meshes.sphere(1.5), [1.5, 1.5, 1.5]),
    ):
        assert np.allclose(np.abs(mesh.positions).max(axis=0), extent, atol=1e-6)
        # A convex shape centred at the origin: every normal points away from the centre.
        assert np.all(np.einsum("ij,ij->i", mesh.positions, mesh.normals) > -1e-6)


# --- through the server ----------------------------------------------------------------------


def test_server_imports_a_robot_on_request_and_keeps_the_old_scene_on_failure():
    async def scenario(socket_path):
        server = MockServer(
            ReplayEngine(generate_sample(duration_s=0.5)), Endpoint("unix", socket_path)
        )
        bound = await server.start()
        try:
            client = await ProtocolClient.connect(bound)
            hello = await client.hello()

            bad = await client.command(import_robot={"path": "/no/such/robot.urdf"})
            failed = None
            while failed is None:
                envelope = await client.receive()
                if (
                    envelope.WhichOneof("body") == "command_result"
                    and envelope.command_result.request_id == bad
                ):
                    failed = envelope.command_result
            prims_after_failure = len(server.engine.recording.snapshot.prims)

            good = await client.command(import_robot={"path": str(ARM)})
            seen = {}
            while "command_result" not in seen:
                envelope = await client.receive()
                body = envelope.WhichOneof("body")
                if body == "command_result" and envelope.command_result.request_id != good:
                    continue
                seen[body] = envelope
            await client.close()
            return hello, failed, prims_after_failure, seen
        finally:
            await server.stop()

    with tempfile.TemporaryDirectory(prefix="lgr-") as directory:
        hello, failed, prims_after_failure, seen = asyncio.run(scenario(f"{directory}/s.sock"))
    assert hello.server_hello.capabilities.can_import
    assert not failed.ok and "no such file" in failed.error
    assert prims_after_failure == 12  # the sample scene is still being served
    assert seen["command_result"].command_result.ok
    assert seen["command_result"].command_result.imported_stage_path == str(ARM)
    assert seen["catalog"].catalog.generation == 2 and not seen["catalog"].catalog.signals
    assert seen["snapshot"].snapshot.prims[0].name == "primitives_arm"
    assert seen["snapshot"].snapshot.stage_path == str(ARM)
    assert "movable joints" in seen["log"].log.text
