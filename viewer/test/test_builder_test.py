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
TEMPLATES_TAB = "77,798"
ARM_TEMPLATE = "130,900"


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
    # R7: nothing but the scene streams while the Builder is in front.
    assert report["signal_blocks"] == 0
    # Closing the viewer stopped its test server and removed its runtime folder.
    assert no_test_server_of(pid)
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", "/nonexistent")) / "lagrange/builder"
    assert not (runtime / str(pid)).exists()


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
    assert builder["check_exit"] == 0


def test_autosave_writes_to_the_state_folder(tmp_path):
    # An edit (Box in Add), then 31 s of ImGui time (frames at 1/60 s): the autosave is written.
    report, code, err, _ = run(tmp_path, "--input", "3:click:40,862", frames=60 * 32)
    assert code == 0, err
    assert report["builder"]["links"] == 2
    autosave = tmp_path / "state" / "lagrange" / "builder" / "robot.autosave.urdf"
    assert autosave.exists()
    assert "<robot" in autosave.read_text()
