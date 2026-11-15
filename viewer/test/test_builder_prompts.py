"""The robot builder's questions and its files outside the document, headless (code review of
2026-11-15).

- M2: Open, a dropped robot or a template over unsaved edits asks Save / Discard / Cancel;
  Cancel keeps the edits and the undo history, Discard opens the new one, Save writes the file
  first. Without edits nothing is asked.
- M3: while a xacro expansion runs, Test does not start (it would replace the document under
  the test); the expanded robot opens when it is done. Two expansions do not share files.
- L6: an autosave a viewer that did not close left of this file, newer than the file, is
  offered back (Restore); an older one is removed without a word.
- L7: Save as… adds ".urdf" before the overwrite question: an existing foo.urdf chosen as "foo"
  is not replaced without asking.
- QA4 M1: closing the viewer over unsaved edits asks Save / Discard / Cancel too (`N:close` does
  what the window's close button does).

`--input N:open:FILE`, `N:saveas:FILE` and `N:template:NAME` do what a dropped file, the Save
as… dialog and a template card do. Run on the software Vulkan device (llvmpipe) unless
LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
ROVER = HERE / "builder" / "rover_mk1.urdf"
TEMPLATES = ROOT / "assets" / "robots" / "templates"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
BOX_TILE = "40,835"
TEST_BUTTON = "1430,48"  # "Test · empty world", left of its chevron
# The unsaved-changes sheet over rover_mk1 (1 edit) asking about "primitives_arm.urdf".
UNSAVED_DISCARD = "927,484"
UNSAVED_SAVE = "983,484"
# The same sheet when the viewer is closed (QA4 M1).
CLOSE_DISCARD = UNSAVED_DISCARD
CLOSE_SAVE = UNSAVED_SAVE
# The restore sheet over rover_mk1.urdf.
RESTORE = "975,490"
DISCARD_AUTOSAVE = "911,490"
# The overwrite sheet for "target.urdf".
REPLACE = "975,490"

ARM = "<robot name='arm'><link name='base'/></robot>\n"


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(
    tmp_path: Path, *extra: str, frames: int = 8, env: dict | None = None
) -> tuple[dict, subprocess.CompletedProcess]:
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "LAGRANGE_BUILDER_ROS_DOMAIN_ID": "off",
        "LAGRANGE_BUILDER_TEMPLATES": str(TEMPLATES),
        **(env or {}),
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
        timeout=180,
        env=environment,
        check=False,
    )
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return (json.loads(lines[-1])["builder"] if lines else {}), done


def rover_copy(tmp_path: Path) -> Path:
    robot = tmp_path / "rover_mk1.urdf"
    robot.write_text(ROVER.read_text())
    return robot


def arm(tmp_path: Path) -> Path:
    other = tmp_path / "primitives_arm.urdf"
    other.write_text(ARM)
    return other


def edited_then_open(tmp_path: Path, *after: str, frames: int = 18):
    """rover_mk1 with one edit (a Box link), then the arm asked for, then `after`."""
    script = [f"3:click:{BOX_TILE}", f"8:open:{arm(tmp_path)}", *after]
    return run(
        tmp_path,
        "--builder-open",
        str(rover_copy(tmp_path)),
        "--input",
        ";".join(script),
        frames=frames,
    )


# --- M2 ------------------------------------------------------------------------------------------


def test_open_over_unsaved_edits_asks_first(tmp_path):
    builder, done = edited_then_open(tmp_path, frames=14)
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == "unsaved"
    assert builder["prompt_title"] == "Unsaved changes"
    assert builder["robot"] == "rover_mk1"  # nothing replaced yet
    assert builder["edits"] == 1


def test_cancel_keeps_the_edits_and_the_history(tmp_path):
    builder, done = edited_then_open(tmp_path, "14:key:escape")
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == ""
    assert builder["robot"] == "rover_mk1"
    assert builder["links"] == 8 and builder["edits"] == 1
    assert len(builder["history"]["labels"]) == 2, builder["history"]


def test_discard_opens_the_new_robot(tmp_path):
    builder, done = edited_then_open(tmp_path, f"14:click:{UNSAVED_DISCARD}")
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == ""
    assert builder["robot"] == "arm"
    assert builder["edits"] == 0
    assert "box_link" not in (tmp_path / "rover_mk1.urdf").read_text()  # not saved


def test_save_writes_the_file_then_opens_the_new_robot(tmp_path):
    builder, done = edited_then_open(tmp_path, f"14:click:{UNSAVED_SAVE}")
    assert done.returncode == 0, done.stderr
    assert builder["robot"] == "arm"
    assert "box_link" in (tmp_path / "rover_mk1.urdf").read_text()


def test_without_edits_nothing_is_asked(tmp_path):
    builder, done = run(
        tmp_path,
        "--builder-open",
        str(rover_copy(tmp_path)),
        "--input",
        f"3:open:{arm(tmp_path)}",
        frames=8,
    )
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == ""
    assert builder["robot"] == "arm"


def test_a_template_over_unsaved_edits_asks_too(tmp_path):
    builder, done = run(
        tmp_path,
        "--builder-open",
        str(rover_copy(tmp_path)),
        "--input",
        f"3:click:{BOX_TILE};8:template:arm_3dof",
        frames=14,
    )
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == "unsaved"
    assert builder["robot"] == "rover_mk1"
    # A new robot (no file): Save as… needs the windowed viewer's dialog; Cancel still works.
    fresh, done = run(
        tmp_path, "--input", f"3:click:{BOX_TILE};8:template:arm_3dof;14:key:escape", frames=18
    )
    assert done.returncode == 0, done.stderr
    assert fresh["prompt"] == "" and fresh["links"] == 2 and fresh["edits"] == 1


# --- QA4 M1: closing the viewer -------------------------------------------------------------------


def edited_then_close(tmp_path: Path, *after: str, frames: int = 18):
    script = [f"3:click:{BOX_TILE}", "8:close", *after]
    return run(
        tmp_path,
        "--builder-open",
        str(rover_copy(tmp_path)),
        "--input",
        ";".join(script),
        frames=frames,
    )


def test_closing_over_unsaved_edits_asks_first(tmp_path):
    builder, done = edited_then_close(tmp_path, frames=14)
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == "unsaved"
    assert "closed at frame" not in done.stderr


def test_closing_cancel_keeps_the_viewer_open(tmp_path):
    builder, done = edited_then_close(tmp_path, "14:key:escape")
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == "" and builder["edits"] == 1
    assert "closed at frame" not in done.stderr


def test_closing_discard_closes_without_saving(tmp_path):
    _, done = edited_then_close(tmp_path, f"14:click:{CLOSE_DISCARD}", frames=30)
    assert done.returncode == 0, done.stderr
    assert "closed at frame" in done.stderr
    assert "box_link" not in (tmp_path / "rover_mk1.urdf").read_text()


def test_closing_save_writes_the_file_then_closes(tmp_path):
    _, done = edited_then_close(tmp_path, f"14:click:{CLOSE_SAVE}", frames=30)
    assert done.returncode == 0, done.stderr
    assert "closed at frame" in done.stderr
    assert "box_link" in (tmp_path / "rover_mk1.urdf").read_text()


def test_closing_without_edits_asks_nothing(tmp_path):
    _, done = run(tmp_path, "--builder-open", str(rover_copy(tmp_path)), "--input", "3:close")
    assert done.returncode == 0, done.stderr
    assert "closed at frame" in done.stderr


# --- M3 ------------------------------------------------------------------------------------------


def slow_xacro(tmp_path: Path) -> dict:
    """A stand-in for xacro on PATH: three seconds, then the input copied to the output."""
    folder = tmp_path / "bin"
    folder.mkdir(exist_ok=True)
    program = folder / "xacro"
    program.write_text('#!/bin/sh\nsleep 3\ncp "$1" "$3"\n')
    program.chmod(0o755)
    return {"PATH": f"{folder}:{os.environ.get('PATH', '')}"}


def test_test_waits_for_a_xacro_expansion(tmp_path):
    source = tmp_path / "late.xacro"
    source.write_text("<robot name='late'><link name='base'/></robot>\n")
    builder, done = run(
        tmp_path,
        "--builder-open",
        str(rover_copy(tmp_path)),
        "--input",
        f"3:open:{source};6:click:{TEST_BUTTON};10:wait:idle",
        frames=16,
        env=slow_xacro(tmp_path),
    )
    assert done.returncode == 0, done.stderr
    # The Test pressed while it expanded did not start, and the expanded robot opened.
    assert builder["mode"] == "build", builder["loader"]
    assert builder["robot"] == "late", builder["files"]
    # Its output and log are its own (xacro-1-...), not <stem>.expanded.urdf shared by two.
    assert not any("could not" in line for line in builder["files"]), builder["files"]


# --- L6 ------------------------------------------------------------------------------------------


def file_key(text: str) -> str:
    """Files.hpp fileKey: FNV-1a 64 of the absolute path, 16 hex digits."""
    value = 14695981039346656037
    for byte in text.encode():
        value ^= byte
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def gone_pid() -> int:
    process = subprocess.Popen(["true"])
    process.wait()
    return process.pid


def left_autosave(tmp_path: Path, robot: Path, age_s: float) -> Path:
    folder = tmp_path / "state" / "lagrange" / "builder"
    folder.mkdir(parents=True, exist_ok=True)
    autosave = folder / f"rover_mk1-{file_key(str(robot))}.{gone_pid()}.autosave.urdf"
    autosave.write_text(
        robot.read_text().replace(
            "</robot>",
            '<link name="restored_link"/><joint name="restored_joint" type="fixed">'
            '<parent link="base_link"/><child link="restored_link"/></joint></robot>',
        )
    )
    when = time.time() - age_s
    os.utime(autosave, (when, when))
    return autosave


def test_a_newer_autosave_is_offered_back_and_restored(tmp_path):
    robot = rover_copy(tmp_path)
    os.utime(robot, (time.time() - 3600, time.time() - 3600))
    autosave = left_autosave(tmp_path, robot, 0)
    asked, done = run(tmp_path, "--builder-open", str(robot), frames=6)
    assert done.returncode == 0, done.stderr
    assert asked["prompt"] == "restore" and asked["links"] == 7
    assert autosave.exists()  # "Not now" (the run ended): kept for the next time
    restored, done = run(
        tmp_path, "--builder-open", str(robot), "--input", f"5:click:{RESTORE}", frames=10
    )
    assert done.returncode == 0, done.stderr
    assert restored["prompt"] == "" and restored["links"] == 8 and restored["edits"] == 1
    assert not autosave.exists()


def test_discard_removes_the_autosave(tmp_path):
    robot = rover_copy(tmp_path)
    os.utime(robot, (time.time() - 3600, time.time() - 3600))
    autosave = left_autosave(tmp_path, robot, 0)
    builder, done = run(
        tmp_path, "--builder-open", str(robot), "--input", f"5:click:{DISCARD_AUTOSAVE}", frames=10
    )
    assert done.returncode == 0, done.stderr
    assert builder["links"] == 7 and builder["edits"] == 0
    assert not autosave.exists()


def test_an_autosave_older_than_the_file_is_removed_not_offered(tmp_path):
    robot = rover_copy(tmp_path)
    autosave = left_autosave(tmp_path, robot, 3600)
    builder, done = run(tmp_path, "--builder-open", str(robot), frames=6)
    assert done.returncode == 0, done.stderr
    assert builder["prompt"] == "" and builder["links"] == 7
    assert not autosave.exists()


# --- L7 ------------------------------------------------------------------------------------------


def test_save_as_asks_before_replacing_the_added_urdf_name(tmp_path):
    target = tmp_path / "target.urdf"
    target.write_text("<robot name='old'><link name='a'/></robot>\n")
    asked, done = run(
        tmp_path, "--builder-open", str(ROVER), "--input", f"3:saveas:{tmp_path / 'target'}"
    )
    assert done.returncode == 0, done.stderr
    assert asked["prompt"] == "overwrite" and asked["prompt_title"] == "Replace target.urdf?"
    assert "old" in target.read_text()
    cancelled, done = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:saveas:{tmp_path / 'target'};6:key:escape",
        frames=10,
    )
    assert done.returncode == 0, done.stderr
    assert cancelled["prompt"] == "" and "old" in target.read_text()
    replaced, done = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        f"3:saveas:{tmp_path / 'target'};6:click:{REPLACE}",
        frames=12,
    )
    assert done.returncode == 0, done.stderr
    assert replaced["prompt"] == "" and "rover_mk1" in target.read_text()
    assert replaced["path"] == str(target)
    # A name that is not there yet: saved as name.urdf without a question.
    fresh, done = run(
        tmp_path, "--builder-open", str(ROVER), "--input", f"3:saveas:{tmp_path / 'fresh'}"
    )
    assert done.returncode == 0, done.stderr
    assert fresh["prompt"] == ""
    assert "rover_mk1" in (tmp_path / "fresh.urdf").read_text()
    assert fresh["path"] == str(tmp_path / "fresh.urdf")
