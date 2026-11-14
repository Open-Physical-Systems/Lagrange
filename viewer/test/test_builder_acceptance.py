"""The robot builder's acceptance (RFC 0016 WP-G, docs/plan/builder-phase1-plan.md §2.2), headless.

1. The script of §2.2: open builder/empty.urdf, set the Add material, Box, its joint to revolute
   with limits, a rangefinder from the catalog with range/max 20, undo and redo, Test, wait for
   the test server, check the Test layout, Back (Esc), and --builder-save: byte for byte
   golden/builder_input.urdf.
2. A mesh part (an L-shaped prism, concave) with each of the four collision choices of
   Addendum A, saved, reopened and saved again with the same bytes.
3. The cost rule (R7): connected to a server, the Sensors workspace streams; with the Builder in
   front nothing but the scene comes (the client's counters).

No --connect except in 3, and the test server runs with --no-ros. Run on the software Vulkan
device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
EMPTY = HERE / "builder" / "empty.urdf"
GOLDEN = HERE / "builder" / "golden" / "builder_input.urdf"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC), with
# builder/empty.urdf open (base_link only).
ADD_MATERIAL = "120,905"  # Add: the material dropdown (custom: half the row)
ADD_MATERIAL_PLA = "160,932"  # its "PLA" row
BOX_TILE = "40,835"
MESH_TILE = "225,835"
MESH_PATH = "400,920"  # Add → Mesh…: the path field
ADD_MESH_LINK = "280,971"
CHILD_JOINT_TEXT = "115,221"  # "fixed · box" on the new link's row: selects its joint
CHILD_LINK_ROW = "80,221"
REVOLUTE = "1425,272"  # Properties: the joint type's Segmented
LOWER_LIMIT = "1470,517"
CATALOG_TAB = "860,763"
CATALOG_FILTER = "950,793"
FIRST_ROW_ADD = "1205,845"  # Add on the first catalog row under its group, while hovered
RANGE_MAX = "1470,546"  # Properties: a rangefinder's range/max
TEST_BUTTON = "1430,48"  # "Test · empty world", left of its chevron
# Properties of the mesh link: the four collision choices, then each choice's own controls.
COLLISION = {"mesh": "1415,648", "split": "1415,708", "shapes": "1415,768", "drawn": "1415,828"}
FIT_TO_MESH = "1363,906"  # under Shapes from templates (box is the default kind)
DRAWN_ADD_BOX = "1287,880"  # under Drawn by hand
PROPERTIES_MIDDLE = "1415,800"
SPLIT_BUTTON = "1383,603"  # after scrolling Properties down by 10

# §2.2, to the redo; the frames are those the script needs on llvmpipe.
BUILD_STEPS = [
    f"3:click:{ADD_MATERIAL}",
    f"6:click:{ADD_MATERIAL_PLA}",
    f"9:click:{BOX_TILE}",
    f"14:click:{CHILD_JOINT_TEXT}",
    f"18:click:{REVOLUTE}",
    f"22:click:{LOWER_LIMIT}",
    "24:text:-90",
    "26:key:tab",
    "28:text:90",
    "30:key:tab",
    "32:text:2",
    "34:key:tab",
    "36:text:3",
    "38:key:enter",
    f"42:click:{CHILD_LINK_ROW}",
    f"45:click:{CATALOG_TAB}",
    f"48:click:{CATALOG_FILTER}",
    "50:text:rangefinder",
    f"54:click:{FIRST_ROW_ADD}",
    f"60:click:{RANGE_MAX}",
    "62:key:ctrl+a",
    "64:text:20",
    "66:key:enter",
    "70:key:ctrl+z",
    "74:key:ctrl+shift+z",
]
TEST_STEPS = [f"80:click:{TEST_BUTTON}", "82:wait:connected"]
BACK_STEPS = ["100:key:escape", "102:wait:build", "103:wait:idle"]


def program(package: str, name: str) -> str:
    try:
        path = Path(get_package_prefix(package)) / "lib" / package / name
    except PackageNotFoundError:
        pytest.skip(f"needs {package} installed")
    if not path.exists():
        pytest.skip(f"needs {name}")
    return str(path)


def run(tmp_path: Path, *extra: str, frames: int) -> tuple[dict, int, str]:
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "LAGRANGE_BUILDER_ROS_DOMAIN_ID": "off",
    }
    done = subprocess.run(
        [
            program("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--frames",
            str(frames),
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=180,
        env=environment,
        check=False,
    )
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return (json.loads(lines[-1]) if lines else {}), done.returncode, done.stderr


def builder(tmp_path: Path, opened: Path, script: list[str], frames: int, *extra: str):
    return run(
        tmp_path,
        "--workspace",
        "builder",
        "--builder-open",
        str(opened),
        "--input",
        ";".join(script),
        *extra,
        frames=frames,
    )


def test_the_script_builds_the_robot_step_by_step(tmp_path):
    report, code, err = builder(tmp_path, EMPTY, BUILD_STEPS, 80)
    assert code == 0, err
    assert report["validation_errors"] == 0
    b = report["builder"]
    assert b["robot"] == "empty"
    assert (b["links"], b["joints"], b["sensors"]) == (2, 1, 1)
    assert b["joint_types"] == {"box_link_joint": "revolute"}
    assert b["selected"] == "rangefinder"
    assert b["errors"] == 0
    # Box, the type, four limits, the sensor and its range: one step each; the undo and the
    # redo leave the stack where it was.
    assert b["history"]["labels"][1:] == [
        "Add link box_link",
        "Set box_link_joint type",
        "Set box_link_joint lower",
        "Set box_link_joint upper",
        "Set box_link_joint effort",
        "Set box_link_joint velocity",
        "Add sensor rangefinder on box_link",
        "Set range/max 12 → 20 m",
    ]
    assert b["history"]["position"] == 8
    assert b["edits"] == 8


def test_test_shows_the_test_layout(tmp_path):
    program("lagrange_server", "lagrange_server")
    shot = tmp_path / "testing.png"
    report, code, err = builder(
        tmp_path, EMPTY, BUILD_STEPS + TEST_STEPS, 110, "--screenshot", str(shot)
    )
    assert code == 0, err
    b = report["builder"]
    assert b["mode"] == "testing", b["loader"]
    assert report["connected"] is True
    # The test server's scene: base_link and box_link as prims, moving in time.
    assert report["snapshots"] >= 1
    assert report["prims"] >= 2
    assert report["sim_time_ns"] > 0
    # SPEC state 4: Drive in front at the bottom right; the catalog behind it.
    assert "Drive" in report["windows"], report["windows"]
    # SPEC state 4: the centre top is the normal server-fed Viewport (the same dock slot).
    assert "Viewport" in report["windows"], report["windows"]
    assert not any(w.startswith("Sensor catalog") for w in report["windows"])
    # R7: nothing but the scene and the moving joints' positions stream from the test server (the
    # Joints panel reads them live, RFC 0016 §7.3); no sensor frames.
    assert set(b["live_joints"]) == {"box_link_joint"}
    assert report["sensor_frames"] == 0
    assert shot.stat().st_size > 0


def test_back_and_save_give_the_golden_file(tmp_path):
    program("lagrange_server", "lagrange_server")
    out = tmp_path / "out.urdf"
    report, code, err = builder(
        tmp_path,
        EMPTY,
        BUILD_STEPS + TEST_STEPS + BACK_STEPS,
        110,
        "--builder-save",
        str(out),
    )
    assert code == 0, err
    b = report["builder"]
    assert b["mode"] == "build"
    assert report["connected"] is False
    assert any(w.startswith("Sensor catalog") for w in report["windows"]), report["windows"]
    assert b["edits"] == 0  # saved
    assert out.read_text() == GOLDEN.read_text()


# --- a mesh part and the four collision choices (Addendum A) ------------------------------


def write_ell(path: Path) -> None:
    """An L-shaped prism, 0.2 by 0.2 by 0.05 m with a 0.08 m arm: closed, outward, concave."""
    outline = [(0, 0), (0.2, 0), (0.2, 0.08), (0.08, 0.08), (0.08, 0.2), (0, 0.2)]
    n = len(outline)
    vertices = [(x, y, 0.0) for x, y in outline] + [(x, y, 0.05) for x, y in outline]
    fan = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5)]
    faces = [(a, c, b) for a, b, c in fan] + [(a + n, b + n, c + n) for a, b, c in fan]
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, j + n), (i, j + n, i + n)]
    lines = [f"v {x:g} {y:g} {z:g}" for x, y, z in vertices]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


def can_split() -> bool:
    try:
        prefix = Path(get_package_prefix("lagrange_collision_bake"))
    except PackageNotFoundError:
        return False
    if not (prefix / "lib" / "lagrange_collision_bake" / "bake_urdf").exists():
        return False
    return importlib.util.find_spec("coacd") is not None


CHOICE_STEPS = {
    "mesh": [f"23:click:{COLLISION['mesh']}"],
    "split": [
        f"23:click:{COLLISION['split']}",
        f"26:move:{PROPERTIES_MIDDLE}",
        "27:wheel:-10",
        f"30:click:{SPLIT_BUTTON}",
        "34:wait:idle",
    ],
    "shapes": [f"23:click:{COLLISION['shapes']}", f"27:click:{FIT_TO_MESH}"],
    "drawn": [f"23:click:{COLLISION['drawn']}", f"27:click:{DRAWN_ADD_BOX}"],
}


def link_element(text: str, name: str) -> str:
    found = re.search(rf'<link name="{name}"[^>]*>.*?</link>', text, flags=re.DOTALL)
    assert found, text
    return found.group(0)


@pytest.mark.parametrize("choice", ["mesh", "split", "shapes", "drawn"])
def test_a_mesh_part_with_each_collision_choice_saves_and_reopens(tmp_path, choice):
    if choice == "split" and not can_split():
        pytest.skip("needs lagrange_collision_bake and coacd (pip install coacd)")
    folder = tmp_path / "robot"
    folder.mkdir()
    robot = folder / "robot.urdf"
    shutil.copy(EMPTY, robot)
    mesh = folder / "ell.obj"
    write_ell(mesh)
    script = [
        f"3:click:{ADD_MATERIAL}",
        f"6:click:{ADD_MATERIAL_PLA}",
        f"9:click:{MESH_TILE}",
        f"12:click:{MESH_PATH}",
        f"14:text:{mesh}",
        "16:key:enter",
        f"19:click:{ADD_MESH_LINK}",
        *CHOICE_STEPS[choice],
    ]
    out = folder / "saved.urdf"
    report, code, err = builder(tmp_path, robot, script, 40, "--builder-save", str(out))
    assert code == 0, err
    b = report["builder"]
    assert b["links"] == 2 and b["errors"] == 0, b
    text = out.read_text()
    ell = link_element(text, "ell")
    assert f'lagrange_collision="{choice}"' in ell, ell
    assert '<mesh filename="ell.obj"/>' in ell  # beside the file: written relative
    collisions = re.findall(r"<collision.*?</collision>", ell, flags=re.DOTALL)
    if choice == "mesh":
        assert len(collisions) == 1 and "ell.obj" in collisions[0], ell
    elif choice == "split":
        parts = [c for c in collisions if "empty_collision/ell_0_" in c]
        assert len(parts) >= 2, ell  # an L is two convex parts at least
        for part in parts:
            name = re.search(r'filename="([^"]+)"', part).group(1)
            assert (folder / name).stat().st_size > 0
    else:
        # Fit to mesh, or one box drawn by hand: a collision box, the mesh only drawn.
        assert len(collisions) == 1 and "<box" in collisions[0], ell
        assert "ell.obj" not in collisions[0]

    # Reopened, it is the same choice and saves to the same bytes.
    again = folder / "again.urdf"
    report, code, err = builder(tmp_path, out, ["1:wait:idle"], 4, "--builder-save", str(again))
    assert code == 0, err
    assert report["builder"]["errors"] == 0
    assert again.read_text() == text


# --- the cost rule: quiet while the Builder is in front (R7) -------------------------------


def test_the_builder_in_front_stops_sensor_and_signal_streams():
    server_program = program("lagrange_server", "lagrange_server")
    with tempfile.TemporaryDirectory(prefix="lgr-") as directory:  # short socket path
        socket = Path(directory) / "q.sock"
        server = subprocess.Popen(
            [server_program, "--no-ros", "--listen", f"unix:{socket}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            for _ in range(100):
                if socket.exists():
                    break
                time.sleep(0.1)
            # The Sensors workspace subscribes the lidar and the IMU scopes; Ctrl B brings the
            # Builder in front at frame 150, and the run goes on for some seconds after it.
            report, code, err = run(
                Path(directory),
                "--connect",
                f"unix:{socket}",
                "--workspace",
                "sensors",
                "--input",
                "150:key:ctrl+b",
                frames=400,
            )
        finally:
            server.terminate()
            server.wait(10)
    assert code == 0, err
    assert report["workspace"] == "Builder"
    assert report["sensor_frames"] > 0  # streamed before
    assert report["signal_blocks"] > 0
    # Nothing came later than a second after the Builder was in front.
    assert report["streamed_while_quiet"] == 0
