"""The sedan's front radar drawn by the viewer: its detections reach the 3D view (headless)."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture(params=["sedan.usda", "sedan_4d_radar.usda"])
def sedan_in_the_city(request, tmp_path):
    """The server with the sample sedan in the city block: its RadarAPI_1 front radar, and in
    the second the 4D imaging radar (a custom sensor of kind imaging_radar) beside it."""
    try:
        assets = Path(get_package_share_directory("lagrange_assets"))
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server and lagrange_assets")
    socket = f"/tmp/lagrange_radar_{os.getpid()}.sock"
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            "--robot",
            str(assets / "robots/sedan" / request.param),
            "--world",
            str(assets / "worlds/city_block/city_block.usda"),
            "--no-ros",
            "--listen",
            f"unix:{socket}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(300):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    yield socket, tmp_path, request.param
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=30)
    Path(socket).unlink(missing_ok=True)


def run_viewer(socket: str, home: Path, *extra: str) -> dict:
    done = subprocess.run(
        [
            tool("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--connect",
            f"unix:{socket}",
            "--size",
            "1280x800",
            "--frames",
            "120",
            "--wait-ms",
            "30000",
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=240,
        check=False,
        env={**os.environ, "HOME": str(home), "XDG_CONFIG_HOME": str(home / "cfg")},
    )
    assert done.returncode == 0, done.stderr
    return json.loads(done.stdout.strip().splitlines()[-1])


def test_the_radar_detections_are_drawn(sedan_in_the_city) -> None:
    socket, home, robot = sedan_in_the_city
    report = run_viewer(socket, home, "--shading", "lit", "--overlays", "radar")
    assert report["connected"]
    assert report["sensor_frames"] > 0
    # The radars see the city's buildings and traffic: the front radar at most 64 per scan
    # (kRadarMaxDetections), the imaging radar at most its max_detections (512).
    most = 64 if robot == "sedan.usda" else 64 + 512
    assert 0 < report["radar_detections"] <= most
