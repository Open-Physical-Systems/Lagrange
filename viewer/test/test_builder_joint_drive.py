"""The Builder's joint sliders drive a tested robot (RFC 0016 §7.3), headless.

Opens rover_mk1, presses Test (its own lagrange_server, --joint-control, --no-ros), and drags
pan_joint's slider in the Joints panel: the viewer sends CommandJoint, and the run's
joint/pan_joint/position, which the panel reads live while the Builder is quiet, goes to the
commanded value. Into a connected run where the robot is passive, the run refuses and the panel
says so. Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROVER = HERE / "builder" / "rover_mk1.urdf"
ASSETS = HERE.parent.parent / "assets"

# Where things are at --size 1600x1000 in the Builder's Test layout (12-builder SPEC).
TEST_BUTTON = "1430,48"  # "Test · empty world", left of its chevron
TEST_CHEVRON = "1515,48"
INTO_THE_RUN = "1500,169"
TEST_INTO_RUN = "1440,48"
PAN_KNOB = (562, 879)  # pan_joint's slider at zero, in the Joints panel
# Dragged from the knob to here, then let go: about +105° on its -170 … 170° track.
DRAG = (
    f"40:move:{PAN_KNOB[0]},{PAN_KNOB[1]};41:down;43:move:570,{PAN_KNOB[1]};"
    f"45:move:580,{PAN_KNOB[1]};47:move:590,{PAN_KNOB[1]};50:up"
)


def program(package: str, name: str) -> str:
    try:
        path = Path(get_package_prefix(package)) / "lib" / package / name
    except PackageNotFoundError:
        pytest.skip(f"needs {package} installed")
    if not path.exists():
        pytest.skip(f"needs {name}")
    return str(path)


def run(tmp_path: Path, script: str, frames: int, *extra: str) -> dict:
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
            "--workspace",
            "builder",
            "--builder-open",
            str(ROVER),
            "--frames",
            str(frames),
            "--input",
            script,
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=150,
        check=False,
        env=environment,
    )
    assert done.returncode == 0, done.stderr
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return json.loads(lines[-1])


def test_dragging_a_slider_turns_the_joint_in_the_test_run(tmp_path):
    program("lagrange_server", "lagrange_server")
    report = run(tmp_path, f"3:click:{TEST_BUTTON};6:wait:connected;{DRAG}", 200)
    builder = report["builder"]
    assert builder["mode"] == "testing", builder["loader"]
    assert builder["joint_drive_error"] == ""
    # Only the dragged joint was commanded, to where the knob was let go.
    assert list(builder["joint_targets"]) == ["pan_joint"]
    target = builder["joint_targets"]["pan_joint"]
    assert 1.0 < target < 2.9, target
    # The run's position, read live, went there (joint control's position loop).
    live = builder["live_joints"]
    assert set(live) == {"left_wheel_joint", "right_wheel_joint", "pan_joint", "tilt_joint"}
    assert live["pan_joint"] == pytest.approx(target, abs=0.05)
    assert abs(live["tilt_joint"]) < 0.1, "the tilt joint was not told to move"


@pytest.fixture
def warehouse(tmp_path):
    """The sample AMR in the warehouse with joint control, on a socket of its own."""
    socket = f"/tmp/lagrange_joint_drive_{os.getpid()}.sock"
    with (tmp_path / "server.log").open("w") as out:
        server = subprocess.Popen(
            [
                program("lagrange_server", "lagrange_server"),
                "--robot",
                str(ASSETS / "robots" / "amr_diffdrive" / "amr_diffdrive.urdf"),
                "--world",
                str(ASSETS / "worlds" / "warehouse_small" / "warehouse_small.urdf"),
                "--no-ros",
                "--joint-control",
                "--listen",
                f"unix:{socket}",
            ],
            stdout=out,
            stderr=subprocess.STDOUT,
        )
    for _ in range(200):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    yield f"unix:{socket}"
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=20)
    Path(socket).unlink(missing_ok=True)


def test_a_passive_robot_in_the_connected_run_is_not_driven_and_says_why(tmp_path, warehouse):
    report = run(
        tmp_path,
        f"4:click:{TEST_CHEVRON};8:click:{INTO_THE_RUN};12:click:{TEST_INTO_RUN};"
        f"14:wait:connected;{DRAG}",
        120,
        "--connect",
        warehouse,
    )
    builder = report["builder"]
    assert builder["mode"] == "testing", builder["loader"]
    assert builder["test_passive"] is True, "the run already has the AMR"
    # Its joints are read by their names in the run (prefixed), not the AMR's wheels.
    assert "rover_mk1_pan_joint takes no commands" in builder["joint_drive_error"]
    assert "pan_joint" in builder["live_joints"]
