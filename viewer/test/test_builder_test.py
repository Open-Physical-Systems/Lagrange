"""The Builder's Test, files and headless flags (RFC 0016 WP-F), headless.

Opens rover_mk1 with --builder-open, presses Test, waits for the test server's scene
(``FRAME:wait:connected``), and goes Back (Esc); saves with --builder-save; opens a template,
a xacro file, exports USD and checks loads, each a child process polled by the viewer. The test
server runs with --no-ros (LAGRANGE_BUILDER_ROS_DOMAIN_ID=off), so nothing reaches a ROS
domain. Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROVER = HERE / "builder" / "rover_mk1.urdf"
TEMPLATES = HERE.parent.parent / "assets" / "robots" / "templates"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
TEST_BUTTON = "1430,48"  # "Test · empty world", left of its chevron
SAVE_BUTTON = "1560,48"
EXPORT_USD = "1495,170"  # in the Save menu
CHECK_LOADS = "1555,84"  # on the Robot strip
TEMPLATES_TAB = "77,771"
ARM_TEMPLATE = "130,873"


def program(package: str, name: str) -> str:
    try:
        path = Path(get_package_prefix(package)) / "lib" / package / name
    except PackageNotFoundError:
        pytest.skip(f"needs {package} installed")
    if not path.exists():
        pytest.skip(f"needs {name}")
    return str(path)


def run(tmp_path: Path, *extra: str, frames: int = 6, env: dict | None = None):
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "LAGRANGE_BUILDER_ROS_DOMAIN_ID": "off",
        "LAGRANGE_BUILDER_TEMPLATES": str(TEMPLATES),
        **(env or {}),
    }
    process = subprocess.Popen(
        [
            program("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--workspace",
            "builder",
            "--frames",
            str(frames),
            *extra,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
    )
    out, err = process.communicate(timeout=150)
    lines = [line for line in out.splitlines() if line.startswith("{")]
    report = json.loads(lines[-1]) if lines else {}
    return report, process.returncode, err, process.pid


def no_test_server_of(pid: int) -> bool:
    found = subprocess.run(
        ["pgrep", "-f", f"builder-{pid}.sock"], capture_output=True, text=True, check=False
    )
    return found.stdout.strip() == ""


def test_open_rover_test_sees_its_scene(tmp_path):
    program("lagrange_server", "lagrange_server")
    report, code, err, pid = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:click:{TEST_BUTTON};6:wait:connected",
        frames=30,
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["robot"] == "rover_mk1"
    assert builder["mode"] == "testing", builder["loader"]
    assert report["connected"] is True
    # The test server's scene: its links as prims, from a snapshot, moving in time.
    assert report["snapshots"] >= 1
    assert report["prims"] >= 7
    assert report["sim_time_ns"] > 0
    # R7: nothing but the scene and the joints' positions stream while the Builder is in front
    # (the Joints panel reads them live, RFC 0016 §7.3): one readout per moving joint.
    assert set(builder["live_joints"]) == {
        "left_wheel_joint",
        "right_wheel_joint",
        "pan_joint",
        "tilt_joint",
    }
    # Closing the viewer stopped its test server and removed its runtime folder.
    assert no_test_server_of(pid)
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", "/nonexistent")) / "lagrange/builder"
    assert not (runtime / str(pid)).exists()


# Test ▾ → A sample world → warehouse_small (the menu opens to the left at the window's edge).
TEST_CHEVRON = "1515,48"
SAMPLE_WORLD = "1480,134"
WAREHOUSE_SMALL = "1310,355"  # 17 px a row; below banked_ring (#417), warehouse_fire (#441), reed_channel (#450)


def test_test_runs_in_a_sample_world_from_the_menu(tmp_path):
    program("lagrange_server", "lagrange_server")
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:click:{TEST_CHEVRON};7:move:{SAMPLE_WORLD};11:move:{WAREHOUSE_SMALL};"
        f"13:click:{WAREHOUSE_SMALL};18:click:{TEST_BUTTON};20:wait:connected",
        frames=40,
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["test_target"].endswith("worlds/warehouse_small/warehouse_small.usda")
    assert builder["mode"] == "testing", builder["loader"]
    # The warehouse's walls and racks are prims of the scene beside the robot's 7 links.
    assert report["prims"] >= 20


# Test ▾ → Start at… (below A world file) and its x, y and heading fields.
START_AT = "1480,177"
START_X = "1330,207"
START_Y = "1330,233"
START_HEADING = "1330,285"


def warehouse_small() -> Path:
    try:
        share = Path(get_package_prefix("lagrange_assets")) / "share" / "lagrange_assets"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_assets installed")
    return share / "worlds" / "warehouse_small" / "warehouse_small.usda"


def test_test_starts_the_robot_where_start_at_says(tmp_path):
    """Start at… typed in the Test menu: the test server gets --spawn and the robot's base
    starts there, facing the heading; the choice is remembered for that world."""
    program("lagrange_server", "lagrange_server")
    world = warehouse_small()
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"2:world:{world};3:click:{TEST_CHEVRON};6:move:{START_AT};9:move:1400,177;"
        f"10:move:{START_X};11:click:{START_X};13:text:2;14:key:enter;"
        f"16:click:{START_Y};18:text:-1.5;19:key:enter;"
        f"21:click:{START_HEADING};23:text:90;24:key:enter;"
        f"27:click:700,600;30:click:{TEST_BUTTON};32:wait:connected",
        frames=50,
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["mode"] == "testing", builder["loader"]
    assert builder["start_at"] == "2, -1.5, 0 · 90°"
    assert builder["test_spawn"] == "2,-1.5,0,1.5707963267948966"  # --spawn X,Y,Z,YAW (rad)
    x, y, z, heading = builder["test_root"]
    assert abs(x - 2.0) < 0.05 and abs(y + 1.5) < 0.05, builder["test_root"]
    assert 0.0 < z < 0.5  # on the floor (its lowest point at z = 0), not under it
    assert abs(heading - 90.0) < 3.0
    remembered = (tmp_path / "cfg" / "lagrange" / "builder_spawns.tsv").read_text()
    assert f"2\t-1.5\t0\t90\t{world}" in remembered

    # Remembered: the next viewer starts the robot there again without asking.
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"2:world:{world};5:click:{TEST_BUTTON};7:wait:connected",
        frames=20,
    )
    assert code == 0, err
    assert report["builder"]["test_spawn"] == "2,-1.5,0,1.5707963267948966"
    x, y, _, _ = report["builder"]["test_root"]
    assert abs(x - 2.0) < 0.05 and abs(y + 1.5) < 0.05


def test_a_world_without_a_start_starts_at_its_origin(tmp_path):
    """No --spawn at all for a world nobody chose a start in (the server's own default)."""
    program("lagrange_server", "lagrange_server")
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"2:world:{warehouse_small()};5:click:{TEST_BUTTON};7:wait:connected",
        frames=20,
    )
    assert code == 0, err
    assert report["builder"]["test_spawn"] == ""
    x, y, _, _ = report["builder"]["test_root"]
    assert abs(x) < 0.05 and abs(y) < 0.05


def test_a_world_file_is_a_test_target(tmp_path):
    """FRAME:world:FILE does what Test ▾ → A world file → Browse… does."""
    world = tmp_path / "flat.urdf"
    world.write_text(
        '<robot name="flat"><link name="ground"><collision><geometry>'
        '<box size="4 4 0.1"/></geometry></collision></link></robot>'
    )
    report, code, err, _ = run(
        tmp_path, "--builder-open", str(ROVER), "--input", f"3:world:{world}", frames=6
    )
    assert code == 0, err
    assert report["builder"]["test_target"] == str(world)


def test_back_returns_to_build_and_stops_the_server(tmp_path):
    program("lagrange_server", "lagrange_server")
    report, code, err, pid = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:click:{TEST_BUTTON};6:wait:connected;20:key:escape;22:wait:build;23:wait:idle",
        frames=30,
    )
    assert code == 0, err
    assert report["builder"]["mode"] == "build"
    # The viewer was not connected before the Test (no --connect): Back leaves it so.
    assert report["connected"] is False
    assert no_test_server_of(pid)


def test_back_returns_to_build_when_the_builder_was_opened_from_the_launcher(tmp_path):
    # 12-builder SPEC "Modes": Back stops the test and returns to Build with the document as
    # it was, whatever opened the Builder; with no server before the Test, it stays offline.
    program("lagrange_server", "lagrange_server")
    report, code, err, pid = run(
        tmp_path,
        "--launcher",
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:key:ctrl+b;8:click:{TEST_BUTTON};12:wait:connected;26:key:escape;28:wait:build;"
        "29:wait:idle",
        frames=40,
    )
    assert code == 0, err
    assert report["workspace"] == "Builder"
    assert report["launcher"] is False
    assert report["builder"]["mode"] == "build"
    assert report["builder"]["robot"] == "rover_mk1"
    assert report["connected"] is False
    assert no_test_server_of(pid)


def test_a_refused_robot_puts_the_server_log_in_problems(tmp_path):
    server = program("lagrange_server", "lagrange_server")
    # A lagrange_server that is given a world that does not exist: it exits before it is ready.
    prefix = tmp_path / "prefix"
    (prefix / "lib" / "lagrange_server").mkdir(parents=True)
    wrapper = prefix / "lib" / "lagrange_server" / "lagrange_server"
    wrapper.write_text(f'#!/bin/sh\nexec {server} "$@" --world {tmp_path}/missing_world.usda\n')
    wrapper.chmod(0o755)
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:click:{TEST_BUTTON};6:wait:build",
        frames=12,
        env={"AMENT_PREFIX_PATH": f"{prefix}:{os.environ.get('AMENT_PREFIX_PATH', '')}"},
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["mode"] == "build"
    assert any("stopped before it was ready" in line for line in builder["loader"])
    assert any("missing_world" in line for line in builder["loader"]), builder["loader"]


def test_save_writes_the_document_and_reopens_the_same(tmp_path):
    out = tmp_path / "saved" / "rover_copy.urdf"
    out.parent.mkdir()
    report, code, err, _ = run(tmp_path, "--builder-open", str(ROVER), "--builder-save", str(out))
    assert code == 0, err
    first = report["builder"]
    assert out.exists()
    again, code, err, _ = run(tmp_path, "--builder-open", str(out), "--builder-save", str(out))
    assert code == 0, err
    assert {k: again["builder"][k] for k in ("links", "joints", "sensors", "joint_types")} == {
        k: first[k] for k in ("links", "joints", "sensors", "joint_types")
    }
    assert again["builder"]["path"] == str(out)
    # Saving what was read back gives the same bytes (WP-B's writer is deterministic).
    text = out.read_text()
    run(tmp_path, "--builder-open", str(out), "--builder-save", str(out))
    assert out.read_text() == text
    # A .xacro is never written (RFC §10): the save fails with exit code 5.
    _, code, err, _ = run(
        tmp_path, "--builder-open", str(ROVER), "--builder-save", str(tmp_path / "r.xacro")
    )
    assert code == 5
    assert "xacro" in err


def test_open_reports_an_unreadable_file(tmp_path):
    bad = tmp_path / "broken.urdf"
    bad.write_text("<robot name='x'><link name='a'>")
    report, code, err, _ = run(tmp_path, "--builder-open", str(bad))
    assert code == 0, err
    builder = report["builder"]
    assert builder["robot"] == "robot"  # the new document stays
    assert any("broken.urdf" in line for line in builder["files"]), builder["files"]


def test_templates_tab_opens_a_template_as_a_new_document(tmp_path):
    report, code, err, _ = run(
        tmp_path, "--input", f"3:click:{TEMPLATES_TAB};8:click:{ARM_TEMPLATE}", frames=14
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["robot"] == "arm_3dof"
    assert builder["path"] == ""  # Save asks for a file; the template stays as it is


def test_xacro_opens_by_expansion(tmp_path):
    if shutil.which("xacro") is None:
        pytest.skip("needs xacro")
    source = tmp_path / "two.urdf.xacro"
    source.write_text(
        textwrap.dedent(
            """\
            <?xml version="1.0"?>
            <robot name="two" xmlns:xacro="http://www.ros.org/wiki/xacro">
              <xacro:property name="size" value="0.2"/>
              <xacro:macro name="cube" params="name">
                <link name="${name}">
                  <visual><geometry><box size="${size} ${size} ${size}"/></geometry></visual>
                </link>
              </xacro:macro>
              <xacro:cube name="a"/>
              <xacro:cube name="b"/>
              <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
            </robot>
            """
        )
    )
    # Named .xacro: the extension is what makes the builder expand it.
    xacro = tmp_path / "two.xacro"
    source.rename(xacro)
    report, code, err, _ = run(
        tmp_path, "--builder-open", str(xacro), "--input", "1:wait:idle", frames=6
    )
    assert code == 0, err
    builder = report["builder"]
    assert builder["robot"] == "two"
    assert builder["links"] == 2
    assert builder["path"] == ""  # saved as .urdf, never over the .xacro
    assert any("Expanded two.xacro" in line for line in builder["files"]), builder["files"]


def test_export_usd_and_check_loads_run_as_child_processes(tmp_path):
    program("lagrange_server", "lagrange_import")
    program("lagrange_server", "lagrange_check")
    robot = tmp_path / "rover_mk1.urdf"
    shutil.copy(ROVER, robot)
    report, code, err, _ = run(
        tmp_path,
        "--builder-open",
        str(robot),
        "--input",
        f"3:click:{SAVE_BUTTON};6:click:{EXPORT_USD};9:wait:idle;12:click:{CHECK_LOADS};15:wait:idle",
        frames=20,
    )
    assert code == 0, err
    builder = report["builder"]
    usda = tmp_path / "rover_mk1.usda"
    assert usda.exists() and usda.stat().st_size > 0
    assert any(f"Exported USD to {usda}" in line for line in builder["files"]), builder["files"]
    # -1: the check never ran (the button was not there, QA2 F2) or did not end by itself.
    assert builder["check_exit"] == 0, builder["files"]


def test_autosave_writes_to_the_state_folder(tmp_path):
    # An edit (Box in Add), then 31 s of ImGui time (frames at 1/60 s): the autosave is written.
    report, code, err, _ = run(tmp_path, "--input", "3:click:40,862", frames=60 * 32)
    assert code == 0, err
    assert report["builder"]["links"] == 2
    # One per document file and viewer (code review L6): robot-new.<pid>.autosave.urdf for a
    # robot with no file yet.
    folder = tmp_path / "state" / "lagrange" / "builder"
    autosaves = list(folder.glob("robot-new.*.autosave.urdf"))
    assert len(autosaves) == 1, list(folder.iterdir())
    assert "<robot" in autosaves[0].read_text()
    assert folder.stat().st_mode & 0o777 == 0o700
