"""The World workspace's Regions and Geodesy tabs (docs/design/ui/10-world), headless against
a real lagrange_server: the tiled town streamed around the AMR, with and without a geo origin.

Regions: the server runs its world as one region, which the tab lists; picking it marks it on
the map in the accent. Geodesy: with a geo origin the tab turns the origin, the floating origin,
the agents and the map's cursor into latitude and longitude (the math is checked against the
core's coords in test_geodesy); without one it says how to give the stage a place.
"""

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
from test_performance_tabs import read_png

GPU = os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe")
ACCENT = (0xE8, 0x77, 0x2E)
REGION_ROW = "150,150"  # the first row of the left dock at --size 1600x1000
MAP_POINT = "500,300"  # a point on the map, away from its legend
RIGHT_DOCK = (1285, 130, 1590, 900)
MAP_AREA = (320, 130, 1270, 960)


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture(scope="module", params=["plain", "geo"])
def server(request, tmp_path_factory):
    try:
        assets = get_package_share_directory("lagrange_assets")
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_world_{os.getpid()}_{request.param}.sock"
    geo = ["--geo-origin", "47.397742,8.545594,488"] if request.param == "geo" else []
    process = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            "--world",
            f"{assets}/worlds/tiled_town/tiled_town.usda",
            "--spawn",
            "1036,1036",
            *geo,
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
    yield request.param, socket, tmp_path_factory.mktemp(request.param)
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(15)
    except subprocess.TimeoutExpired:
        process.kill()
    Path(socket).unlink(missing_ok=True)


def view(socket: str, tmp_path: Path, panel: str, inputs: str) -> tuple[dict, list]:
    image = tmp_path / f"world_{panel}.png"
    done = subprocess.run(
        [
            tool("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            GPU,
            "--validation",
            "--connect",
            f"unix:{socket}",
            "--size",
            "1600x1000",
            "--frames",
            "120",
            "--wait-ms",
            "60000",
            "--workspace",
            "world",
            "--panel",
            panel,
            "--input",
            inputs,
            "--screenshot",
            str(image),
        ],
        capture_output=True,
        text=True,
        timeout=240,
        check=False,
        env={**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")},
    )
    assert done.returncode == 0, done.stderr
    stats = json.loads(done.stdout.strip().splitlines()[-1])
    assert stats["validation_errors"] == 0, done.stderr
    _width, _height, rows = read_png(image)
    return stats, rows


def count(rows, box, colour=None, tolerance=30) -> int:
    """Pixels in the box near the colour, or bright ones (any channel over 150) without one."""
    x0, y0, x1, y1 = box
    total = 0
    for row in rows[y0:y1]:
        for i in range(x0 * 4, x1 * 4, 4):
            if colour is None:
                total += max(row[i], row[i + 1], row[i + 2]) > 150
            else:
                total += all(abs(row[i + k] - colour[k]) <= tolerance for k in range(3))
    return total


def test_regions_tab_picks_the_servers_region(server) -> None:
    _kind, socket, tmp_path = server
    stats, before = view(socket, tmp_path, "Regions", f"200:move:{MAP_POINT}")
    assert "Regions" in stats["windows"], stats["windows"]
    assert stats["world_tiles"] > 0
    stats, after = view(socket, tmp_path, "Regions", f"40:click:{REGION_ROW};80:move:{MAP_POINT}")
    # The picked row has the accent bar on its left edge ...
    assert count(after, (8, 126, 16, 178), ACCENT) > 40, "the region row was not picked"
    # ... and the map frames the region and outlines it in the accent.
    assert count(after, MAP_AREA, ACCENT) > count(before, MAP_AREA, ACCENT) + 500


def test_geodesy_tab_reads_out_the_cursor(server) -> None:
    kind, socket, tmp_path = server
    stats, rows = view(socket, tmp_path, "Geodesy", f"80:move:{MAP_POINT}")
    assert "Geodesy" in stats["windows"], stats["windows"]
    bright = count(rows, RIGHT_DOCK)
    # Georeferenced, the tab has the geo origin's nine rows and the cursor's and the agents'
    # latitude and longitude; without a geo origin a caption stands in for them.
    if kind == "geo":
        assert bright > 2200, bright
    else:
        assert 600 < bright < 2000, bright
