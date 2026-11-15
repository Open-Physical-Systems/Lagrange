"""The ROS 2 workspace's Services, Actions, Params and TF tree tabs (docs/design/ui/09-ros2)
against the real lagrange_server with its ROS 2 bridge, and a small user stack beside it: an
action server and client, and robot_state_publisher on the sample AMR.

The viewer never joins the ROS domain (invariant 14): what it shows comes from the server's
RosGraph report and its scene. Runs on llvmpipe unless LAGRANGE_TEST_GPU names another device.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from test_headless import read_png

VIEWER = os.environ.get("LAGRANGE_VIEWER", "lagrange_viewer")
GPU = os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe")
HERE = Path(__file__).resolve().parent

# Where things are at --size 1600x1000: the first row of a left-dock table, the base_link row
# of the TF tree, and the TF tree's header line (its pills).
FIRST_ACTION_ROW = "60,198"
BASE_LINK_ROW = "560,185"
TF_HEADER = (450, 130, 1250, 150)
# Pill fills: theme c::warn at 14 % and c::error at 15 % over the panel's background.
WARN = (42, 42, 38)  # the "not on /tf" pill
ERROR = (45, 28, 40)  # the "stale" pill


def local_tool(package: str, name: str) -> str:
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        candidate = Path(prefix) / f"lib/{package}/{name}"
        if prefix and candidate.is_file():
            return str(candidate)
    pytest.skip(f"{package} is not installed")


def asset(relative: str) -> Path:
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        candidate = Path(prefix) / "share/lagrange_assets" / relative
        if prefix and candidate.is_file():
            return candidate
    pytest.skip("lagrange_assets is not installed")


def stop(process: subprocess.Popen) -> None:
    """SIGTERM first (a killed ROS process leaves shared-memory debris), then wait."""
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(20)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(5)


@pytest.fixture(scope="module")
def ros_run():
    """lagrange_server with ROS on the sample AMR, in a ROS domain of its own."""
    for module in ("rclpy", "control_msgs.action", "example_interfaces.action"):
        pytest.importorskip(module)
    local_tool("robot_state_publisher", "robot_state_publisher")
    server_tool = local_tool("lagrange_server", "lagrange_server")
    directory = Path(tempfile.mkdtemp(prefix="lgr-ros-"))
    environment = dict(os.environ)
    # A domain away from the default, so a run on the developer's machine is not disturbed.
    environment.setdefault("ROS_DOMAIN_ID", str(60 + os.getpid() % 30))
    socket = directory / "s.sock"
    server = subprocess.Popen(
        [server_tool, "--listen", f"unix:{socket}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=environment,
    )
    for _ in range(150):
        if socket.exists():
            break
        time.sleep(0.1)
    assert socket.exists(), "lagrange_server did not listen"
    processes: list[subprocess.Popen] = []

    def start_stack() -> None:
        """The user's side: an action server and client, robot_state_publisher."""
        processes.append(
            subprocess.Popen(
                [sys.executable, str(HERE / "ros2_stack.py")],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=environment,
            )
        )
        urdf = asset("robots/amr_diffdrive/amr_diffdrive.urdf").read_text()
        parameters = directory / "rsp.yaml"
        parameters.write_text(
            json.dumps(
                {
                    "robot_state_publisher": {
                        "ros__parameters": {"robot_description": urdf, "use_sim_time": True}
                    }
                }
            )
        )
        processes.append(
            subprocess.Popen(
                [
                    local_tool("robot_state_publisher", "robot_state_publisher"),
                    "--ros-args",
                    "--params-file",
                    str(parameters),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=environment,
            )
        )
        time.sleep(4.0)  # discovery

    def view(name: str, panel: str, *extra: str) -> tuple[dict, list]:
        image = directory / name
        done = subprocess.run(
            [
                VIEWER,
                "--headless",
                "--gpu",
                GPU,
                "--validation",
                "--connect",
                f"unix:{socket}",
                "--size",
                "1600x1000",
                "--frames",
                "150",
                "--workspace",
                "ros 2",
                "--panel",
                panel,
                "--screenshot",
                str(image),
                *extra,
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
            env={**environment, "HOME": str(directory), "XDG_CONFIG_HOME": str(directory / "c")},
        )
        assert done.returncode == 0, done.stderr
        stats = json.loads(done.stdout.strip().splitlines()[-1])
        assert stats["validation_errors"] == 0, done.stderr
        _width, _height, rows = read_png(image)
        return stats, rows

    yield start_stack, view
    for process in processes:
        stop(process)
    stop(server)
    shutil.rmtree(directory, ignore_errors=True)


def near(rows, box, colour, tolerance=3) -> int:
    x0, y0, x1, y1 = box
    return sum(
        1
        for row in rows[y0:y1]
        for i in range(x0 * 4, x1 * 4, 4)
        if all(abs(row[i + k] - colour[k]) <= tolerance for k in range(3))
    )


def test_ros2_tabs(ros_run) -> None:
    start_stack, view = ros_run
    # Alone, the bridge publishes odom -> base_link and the sensors' mounts; nobody turns the
    # joints into transforms, which the TF tree says with a warning pill.
    stats, rows = view("tf_alone.png", "TF tree")
    assert "TF tree" in stats["windows"]
    assert stats["ros_topics"] >= 5
    assert near(rows, TF_HEADER, WARN) > 300, "the TF tree does not warn of unpublished links"
    assert near(rows, TF_HEADER, ERROR) == 0, "a running bridge's odom frame is stale"

    for tab in ("Services", "Params", "Actions"):
        stats, _ = view(f"{tab}.png", tab)
        assert tab in stats["windows"], stats["windows"]

    start_stack()
    # With robot_state_publisher every frame is published: no warning, nothing stale.
    stats, rows = view("tf_stack.png", "TF tree", "--input", f"100:click:{BASE_LINK_ROW}")
    assert stats["ros_nodes"] >= 3
    assert near(rows, TF_HEADER, WARN) == 0, "links stay unpublished beside robot_state_publisher"
    assert near(rows, TF_HEADER, ERROR) == 0
    # The selected frame row has the accent bar (2 px, x about 455).
    accent = near(rows, (452, 172, 462, 198), (0xE8, 0x77, 0x2E), 30)
    assert accent > 10, "base_link was not selected"
    # The actions of the stack, picked from their hidden topics: the first row selects.
    stats, rows = view("actions.png", "Actions", "--input", f"100:click:{FIRST_ACTION_ROW}")
    assert "Actions" in stats["windows"]
    # A selected row is drawn with the selection colour across the table; the section header of
    # the chosen action appears under it with its caret (more light pixels in the lower part).
    lower = sum(1 for row in rows[820:940] for i in range(0, 440 * 4, 4) if row[i] > 150)
    assert lower > 200, "the chosen action's details are missing"
