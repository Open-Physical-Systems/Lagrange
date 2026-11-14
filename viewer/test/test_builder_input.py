"""The Builder workspace, headless, without a server (RFC 0016).

WP0: the workspace opens with --workspace builder and no --connect, and the launcher opens it
with Ctrl B. WP-G fills in the scripted steps of docs/plan/builder-phase1-plan.md §2.2.
"""

from __future__ import annotations

import json
import os
import subprocess

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
BOX_TILE = "40,862"
CYLINDER_TILE = "101,862"
BOX_LINK_TYPE_TEXT = "115,221"  # "fixed · box" on the box link's row: selects its joint
REVOLUTE = "1425,272"  # Properties: the joint type's Segmented
LOWER_LIMIT = "1470,517"
OTHER_KIND_FIELD = "1005,897"
OTHER_KIND_ADD = "1218,897"
DELETE_TOOL = "1572,115"  # Properties' Delete in its tab bar


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(tmp_path, *extra: str, frames: int = 6) -> tuple[dict, subprocess.CompletedProcess]:
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}
    done = subprocess.run(
        [
            viewer(),
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
        timeout=120,
        env=environment,
        check=False,
    )
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return (json.loads(lines[-1]) if lines else {}), done


def test_builder_opens_without_a_server(tmp_path):
    shot = tmp_path / "builder.png"
    report, done = run(tmp_path, "--workspace", "builder", "--screenshot", str(shot))
    assert done.returncode == 0, done.stderr
    assert report.get("connected") is False
    assert report.get("workspace") == "Builder"
    assert shot.stat().st_size > 0


def test_ctrl_b_on_the_launcher_opens_the_builder(tmp_path):
    shot = tmp_path / "launcher_builder.png"
    report, done = run(tmp_path, "--launcher", "--input", "2:key:ctrl+b", "--screenshot", str(shot))
    assert done.returncode == 0, done.stderr
    assert report.get("workspace") == "Builder"
    assert report.get("launcher") is False
    assert report.get("connected") is False
    assert shot.stat().st_size > 0


def run_input(tmp_path, script: str, frames: int) -> dict:
    report, done = run(tmp_path, "--workspace", "builder", "--input", script, frames=frames)
    assert done.returncode == 0, done.stderr
    assert report.get("validation_errors") == 0
    return report["builder"]


def test_a_new_document_is_one_box_link(tmp_path):
    builder = run_input(tmp_path, "2:key:tab", 6)
    assert builder["links"] == 1
    assert builder["joints"] == 0
    assert builder["edits"] == 0


def test_add_tiles_make_links_and_the_joint_takes_type_and_limits(tmp_path):
    script = ";".join(
        [
            f"3:click:{BOX_TILE}",
            f"8:click:{CYLINDER_TILE}",
            f"12:click:{BOX_LINK_TYPE_TEXT}",
            f"16:click:{REVOLUTE}",
            f"20:click:{LOWER_LIMIT}",
            "22:text:-90",
            "24:key:tab",
            "26:text:90",
            "28:key:tab",
            "30:text:2",
            "32:key:tab",
            "34:text:3",
            "36:key:enter",
        ]
    )
    builder = run_input(tmp_path, script, 40)
    assert builder["links"] == 3
    assert builder["joint_types"] == {
        "box_link_joint": "revolute",
        "cylinder_link_joint": "fixed",
    }
    assert builder["selected"] == "box_link_joint"
    # Two links, the type, and four limits: one step each.
    assert builder["edits"] == 7


def test_other_kind_adds_a_sensor_on_the_selected_link(tmp_path):
    script = ";".join(
        [
            f"3:click:{BOX_TILE}",
            f"8:click:{OTHER_KIND_FIELD}",
            "10:text:rangefinder",
            "12:key:enter",
            f"16:click:{OTHER_KIND_ADD}",
        ]
    )
    builder = run_input(tmp_path, script, 24)
    assert builder["sensors"] == 1
    assert builder["selected"] == "rangefinder"


def test_delete_removes_the_selection(tmp_path):
    script = ";".join([f"3:click:{BOX_TILE}", f"8:click:{DELETE_TOOL}"])
    builder = run_input(tmp_path, script, 14)
    assert builder["links"] == 1
    assert builder["joints"] == 0
    assert builder["edits"] == 2
