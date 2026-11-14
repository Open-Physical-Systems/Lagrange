"""The robot builder's files and edits, headless (QA pass of 2026-11-15, docs/guide/builder.md).

- Save as… into another folder keeps relative meshes: the saved copy loads in lagrange_import
  (the server's loader) with every mesh read, not as magenta boxes (F1).
- A failed --builder-open writes nothing with --builder-save and exits non-zero (F15).
- Save writes through a symbolic link and refuses a read-only file (F19).
- The JSON report stays JSON for a NaN or infinite size (F3).
- Deleting the root of everything is refused with a message (F7).
- A typed joint value stops at the joint's limits (F11).
- Renaming a joint renames its <ros2_control> references (F20).
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
PRIMITIVES_ARM = ROOT / "assets" / "robots" / "primitives_arm" / "primitives_arm.urdf"
ARM6 = ROOT / "assets" / "robots" / "arm6" / "arm6.urdf"
ROVER = HERE / "builder" / "rover_mk1.urdf"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
BOX_TILE = "40,835"
BASE_LINK_ROW = "70,199"
DELETE_TOOL = "1572,115"  # Properties' Delete in its tab bar
PAN_VALUE = "650,880"  # Joints: pan_joint's value field (rover_mk1)
NAME_FIELD = "1470,185"  # Properties: the selection's name


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(tmp_path: Path, *extra: str, frames: int = 8) -> tuple[dict, subprocess.CompletedProcess]:
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
    }
    done = subprocess.run(
        [
            viewer(),
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
        capture_output=True,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return (json.loads(lines[-1]) if lines else {}), done


def import_tool() -> str:
    try:
        found = Path(get_package_prefix("lagrange_server")) / "lib/lagrange_server/lagrange_import"
        if found.exists():
            return str(found)
    except PackageNotFoundError:
        pass
    found_on_path = shutil.which("lagrange_import")
    if not found_on_path:
        pytest.skip("lagrange_import is not installed")
    return found_on_path


def test_save_as_elsewhere_keeps_relative_meshes(tmp_path: Path) -> None:
    # A copy of a robot with a relative mesh, opened from its folder and saved into another.
    source = tmp_path / "original"
    shutil.copytree(PRIMITIVES_ARM.parent, source)
    saved = tmp_path / "mine" / "deeper" / "arm.urdf"
    saved.parent.mkdir(parents=True)
    _, done = run(
        tmp_path, "--builder-open", str(source / PRIMITIVES_ARM.name), "--builder-save", str(saved)
    )
    assert done.returncode == 0, done.stderr
    text = saved.read_text()
    assert 'filename="../../original/tool.stl"' in text, text
    # The server's loader reads the copy with every mesh.
    imported = subprocess.run(
        [import_tool(), str(saved), str(tmp_path / "arm.usda")],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert imported.returncode == 0, imported.stderr
    log = imported.stdout + imported.stderr
    assert "cannot read mesh" not in log, log
    assert "magenta" not in log, log
    # Saved again in its own folder: as written.
    _, again = run(tmp_path, "--builder-open", str(saved), "--builder-save", str(saved))
    assert again.returncode == 0, again.stderr
    assert saved.read_text() == text


def test_a_failed_open_saves_nothing(tmp_path: Path) -> None:
    broken = tmp_path / "broken.urdf"
    broken.write_text("<robot name='x'><link name='a'>")
    target = tmp_path / "out.urdf"
    _, done = run(tmp_path, "--builder-open", str(broken), "--builder-save", str(target))
    assert done.returncode != 0
    assert "not saved" in done.stderr, done.stderr
    assert not target.exists()
    # --builder-save over the file that did not open leaves it as it was.
    _, again = run(tmp_path, "--builder-open", str(broken), "--builder-save", str(broken))
    assert again.returncode != 0
    assert broken.read_text() == "<robot name='x'><link name='a'>"


def test_save_writes_through_a_link_and_refuses_a_read_only_file(tmp_path: Path) -> None:
    real = tmp_path / "src" / "robot.urdf"
    real.parent.mkdir()
    shutil.copy(ROVER, real)
    real.chmod(0o640)
    link = tmp_path / "robot.urdf"
    link.symlink_to(real)
    _, done = run(
        tmp_path,
        "--builder-open",
        str(link),
        "--builder-save",
        str(link),
        "--input",
        f"3:click:{BOX_TILE}",
    )
    assert done.returncode == 0, done.stderr
    assert link.is_symlink()
    assert "box_link" in real.read_text()
    assert real.stat().st_mode & 0o777 == 0o640

    read_only = tmp_path / "ro.urdf"
    shutil.copy(ROVER, read_only)
    read_only.chmod(0o444)
    if os.access(read_only, os.W_OK):
        pytest.skip("running as a user who may write a read-only file")
    before = read_only.read_text()
    _, refused = run(
        tmp_path,
        "--builder-open",
        str(read_only),
        "--builder-save",
        str(read_only),
        "--input",
        f"3:click:{BOX_TILE}",
    )
    assert refused.returncode != 0
    assert "read-only" in refused.stderr, refused.stderr
    assert read_only.read_text() == before
    assert read_only.stat().st_mode & 0o777 == 0o444


def test_the_report_stays_json_for_non_finite_sizes(tmp_path: Path) -> None:
    sizes = tmp_path / "sizes.urdf"
    sizes.write_text(
        "<robot name='sizes'><link name='a'><visual><geometry>"
        "<cylinder radius='nan' length='inf'/></geometry></visual></link></robot>"
    )
    report, done = run(tmp_path, "--builder-open", str(sizes))
    assert done.returncode == 0, done.stderr
    shape = report["builder"]["geometry"]["links"]["a"][0]
    assert shape["size"][:2] == [None, None]


def test_the_root_of_everything_is_not_deleted(tmp_path: Path) -> None:
    saved = tmp_path / "root.urdf"
    report, done = run(
        tmp_path,
        "--input",
        f"3:click:{BOX_TILE};8:click:{BASE_LINK_ROW};12:click:{DELETE_TOOL}",
        "--builder-save",
        str(saved),
        frames=20,
    )
    assert done.returncode == 0, done.stderr
    builder = report["builder"]
    assert builder["links"] == 2, builder
    assert any("is the root" in message for message in builder["files"]), builder["files"]
    assert "base_link" in saved.read_text()


def test_a_typed_joint_value_stops_at_the_limits(tmp_path: Path) -> None:
    report, done = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:click:{PAN_VALUE};5:key:ctrl+a;7:text:500;9:key:enter",
        frames=16,
    )
    assert done.returncode == 0, done.stderr
    pan = report["builder"]["preview"].get("pan_joint")
    assert pan is not None, report["builder"]["preview"]
    assert pan == pytest.approx(2.96706, abs=1e-4)  # 170 degrees, its upper limit


def test_renaming_a_joint_renames_its_ros2_control_references(tmp_path: Path) -> None:
    arm = tmp_path / "arm6.urdf"
    shutil.copy(ARM6, arm)
    saved = tmp_path / "renamed.urdf"
    report, done = run(
        tmp_path,
        "--builder-open",
        str(arm),
        "--builder-save",
        str(saved),
        "--input",
        f"3:select:elbow_joint;6:click:{NAME_FIELD};8:key:ctrl+a;10:text:elbow;12:key:enter",
        frames=20,
    )
    assert done.returncode == 0, done.stderr
    text = saved.read_text()
    assert '<joint name="elbow" type="revolute">' in text
    assert 'name="elbow_joint"' not in text, "a <ros2_control> joint kept the old name"
    assert report["builder"]["robot"] == "arm6"
