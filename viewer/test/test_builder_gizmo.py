"""The Builder view's gizmos, headless, without a server (RFC 0016; 12-builder SPEC "Builder view").

Each test presses what a user presses (--input) on a new document (base_link, a 0.3 x 0.2 x 0.1 m
box) and reads the document back from the headless report's "geometry". Every drag must be one
undo step, its values snapped (5 mm, 5 deg), and Esc must put the document back.
"""

from __future__ import annotations

import json
import math
import os
import subprocess

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

# Where things are at --size 1600x1000 in the Builder's default layout with the view's first
# camera (BuilderView: kStartYaw, kStartPitch, fitted to the new document's box).
BOX = "760,450"  # the base box's top face
IN_VIEW = "1000,650"  # an empty spot of the view, to hover it before a tool key
X_ARROW = ("850,488", ["870,500", "890,511", "902,518"])  # base box selected, Move tool
PLUS_X_FACE = ("829,476", ["849,487", "869,499"])  # base box selected, Select tool
BOX_TILE = "40,862"  # Add: a box link on a fixed joint, selected
Z_RING = ("671,374", ["680,392", "700,405", "725,412"])  # box_link's joint origin, Rotate
AXIS_TIP = ("760,290", ["790,340", "815,380", "830,405"])  # box_link's joint, Joint axis tool
STEP = 0.005  # m
ANGLE_STEP = math.radians(5.0)


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(tmp_path, script: str, frames: int) -> dict:
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}
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
            "--input",
            script,
        ],
        capture_output=True,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    assert lines, done.stdout
    return json.loads(lines[-1])["builder"]


def drag(start_frame: int, handle: tuple[str, list[str]], release: bool = True) -> list[str]:
    """Move onto the handle, press, move through the points two frames apart, let go."""
    at, path = handle
    events = [f"{start_frame}:move:{at}", f"{start_frame + 2}:down"]
    frame = start_frame + 4
    for point in path:
        events.append(f"{frame}:move:{point}")
        frame += 2
    if release:
        events.append(f"{frame}:up")
    return events


def on_step(value: float, step: float) -> bool:
    return abs(value / step - round(value / step)) < 1e-6


def base_box(builder: dict) -> dict:
    return builder["geometry"]["links"]["base_link"][0]


def test_dragging_a_move_arrow_moves_the_shape_by_snapped_steps_in_one_undo_step(tmp_path):
    script = [f"3:click:{BOX}", f"7:move:{IN_VIEW}", "8:key:w", *drag(12, X_ARROW)]
    builder = run(tmp_path, ";".join(script), 30)
    box = base_box(builder)
    assert box["xyz"][0] > 0.03, box
    assert on_step(box["xyz"][0], STEP)
    assert box["xyz"][1:] == [0, 0]
    assert box["size"] == [0.3, 0.2, 0.1]
    assert builder["edits"] == 1


def test_undo_after_a_drag_puts_everything_back(tmp_path):
    script = [f"3:click:{BOX}", f"7:move:{IN_VIEW}", "8:key:w", *drag(12, X_ARROW), "28:key:ctrl+z"]
    builder = run(tmp_path, ";".join(script), 34)
    assert base_box(builder)["xyz"] == [0, 0, 0]
    assert builder["edits"] == 0


def test_escape_cancels_a_drag(tmp_path):
    script = [f"3:click:{BOX}", f"7:move:{IN_VIEW}", "8:key:w"]
    script += [*drag(12, X_ARROW, release=False), "21:key:escape", "25:up"]
    builder = run(tmp_path, ";".join(script), 30)
    assert base_box(builder)["xyz"] == [0, 0, 0]
    assert builder["edits"] == 0


def test_a_face_handle_sizes_the_box_and_the_opposite_face_stays(tmp_path):
    script = [f"3:click:{BOX}", *drag(10, PLUS_X_FACE)]
    builder = run(tmp_path, ";".join(script), 26)
    box = base_box(builder)
    assert box["size"][0] > 0.33, box
    assert on_step(box["size"][0], STEP)
    assert box["size"][1:] == [0.2, 0.1]
    assert box["xyz"][0] - box["size"][0] / 2 == pytest.approx(-0.15, abs=1e-9)
    assert builder["edits"] == 1


def test_a_rotate_ring_turns_the_joint_origin_by_snapped_degrees(tmp_path):
    script = [f"3:click:{BOX_TILE}", f"7:move:{IN_VIEW}", "8:key:e", *drag(10, Z_RING)]
    builder = run(tmp_path, ";".join(script), 26)
    joint = builder["geometry"]["joints"]["box_link_joint"]
    assert joint["rpy"][:2] == [0, 0]
    assert abs(joint["rpy"][2]) >= ANGLE_STEP
    assert on_step(joint["rpy"][2], ANGLE_STEP)
    assert joint["xyz"] == [0, 0, 0.1]
    assert builder["edits"] == 2  # the link, then the turn


def test_the_joint_axis_tool_snaps_the_axis_to_a_principal_axis(tmp_path):
    script = [f"3:click:{BOX_TILE}", f"7:move:{IN_VIEW}", "8:key:a", *drag(10, AXIS_TIP)]
    builder = run(tmp_path, ";".join(script), 26)
    assert builder["geometry"]["joints"]["box_link_joint"]["axis"] == [1, 0, 0]
    assert builder["edits"] == 2


def test_measure_changes_nothing(tmp_path):
    script = [f"3:move:{IN_VIEW}", "4:key:m", "6:click:690,400", "10:click:840,445"]
    builder = run(tmp_path, ";".join(script), 16)
    assert builder["edits"] == 0
    assert base_box(builder)["xyz"] == [0, 0, 0]
