"""The Builder's Test into the connected run (RFC 0016 §7.2), headless, against a real server.

A lagrange_server runs the sample AMR in the sample warehouse (--no-ros). The viewer connects
to it, opens rover_mk1 in the Builder, picks *Into the connected run* in the Test menu and
presses Test: the robot's URDF goes to the server as ImportRobot.content, the server names it
by its <robot name> and spawns it beside the AMR, passive, since the run already has a robot.
Back (Esc) sends RemoveRobot and the run goes on without it; so does closing the viewer while
testing (QA F12). Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names
another.
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

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
TEST_CHEVRON = "1515,48"  # the ▾ of "Test · empty world"
INTO_THE_RUN = "1500,169"  # "Into the connected run" in its menu
TEST_BUTTON = "1440,48"  # "Test · connected run", left of its chevron


def program(package: str, name: str) -> str:
    try:
        path = Path(get_package_prefix(package)) / "lib" / package / name
    except PackageNotFoundError:
        pytest.skip(f"needs {package} installed")
    if not path.exists():
        pytest.skip(f"needs {name}")
    return str(path)


@pytest.fixture
def warehouse(tmp_path):
    """The sample AMR in the warehouse on a socket of its own; yields (endpoint, log path)."""
    socket = f"/tmp/lagrange_into_run_{os.getpid()}.sock"
    log = tmp_path / "server.log"
    with log.open("w") as out:
        server = subprocess.Popen(
            [
                program("lagrange_server", "lagrange_server"),
                "--robot",
                str(ASSETS / "robots" / "amr_diffdrive" / "amr_diffdrive.urdf"),
                "--world",
                str(ASSETS / "worlds" / "warehouse_small" / "warehouse_small.urdf"),
                "--no-ros",
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
    yield f"unix:{socket}", log
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=20)
    Path(socket).unlink(missing_ok=True)


def run(tmp_path: Path, endpoint: str, script: str, frames: int, *extra: str) -> dict:
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
            "--connect",
            endpoint,
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


def test_the_robot_goes_into_the_run_passive_and_back_removes_it(tmp_path, warehouse):
    endpoint, log = warehouse
    report = run(
        tmp_path,
        endpoint,
        f"4:click:{TEST_CHEVRON};8:click:{INTO_THE_RUN};12:click:{TEST_BUTTON};14:wait:connected",
        40,
    )
    builder = report["builder"]
    assert builder["test_into_run"] is True
    assert builder["mode"] == "testing", builder["loader"]
    assert builder["test_robot"] == "rover_mk1"
    assert builder["test_passive"] is True, "the run already has the AMR"
    assert "spawned rover_mk1 from" in log.read_text()
    # The viewer stayed on the user's run: no test server of its own was started.
    assert report["connected"] is True

    # The viewer quit while testing: the robot left the run with it (QA F12), as with Back.
    text = log.read_text()
    for _ in range(50):
        if "deleted rover_mk1\n" in text:
            break
        time.sleep(0.1)
        text = log.read_text()
    assert "deleted rover_mk1\n" in text, "its viewer quit: the test robot leaves the run"

    # Again, then Back: the name is free again, and Back removes it from the run.
    report = run(
        tmp_path,
        endpoint,
        f"4:click:{TEST_CHEVRON};8:click:{INTO_THE_RUN};12:click:{TEST_BUTTON};14:wait:connected;"
        "24:key:escape;26:wait:build",
        60,
    )
    assert report["builder"]["mode"] == "build"
    assert report["builder"]["test_robot"] == ""
    text = log.read_text()
    assert text.count("spawned rover_mk1 from") == 2, text
    for _ in range(50):
        if text.count("deleted rover_mk1\n") == 2:
            break
        time.sleep(0.1)
        text = log.read_text()
    assert text.count("deleted rover_mk1\n") == 2, "Back removed the second one"
    assert "rover_mk1_2" not in text, "no test robot was left behind to take the name"
