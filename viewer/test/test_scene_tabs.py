"""The Scene workspace's extra tabs against a real server (headless, docs/design/ui/02-scene):
the sensor views beside the viewport, the Layers tab beside the tree, and the History tab
beside the Inspector after an edit applied from the Inspector."""

from __future__ import annotations

import json
import os
import signal
import struct
import subprocess
import time
import zlib
from pathlib import Path

import pytest
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)

SIZE = "1600x1000"
MOTOR = "/amr_diffdrive/base_link/left_drive_motor"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture
def amr(tmp_path):
    """The sample AMR with its lidars and a thermal camera in the small warehouse."""
    try:
        assets = Path(get_package_share_directory("lagrange_assets"))
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server and lagrange_assets")
    socket = f"/tmp/lagrange_scene_tabs_{os.getpid()}.sock"
    log = tmp_path / "server.log"
    with log.open("w") as out:
        server = subprocess.Popen(
            [
                tool("lagrange_server", "lagrange_server"),
                "--robot",
                str(assets / "robots/amr_diffdrive/amr_diffdrive_thermal.usda"),
                "--world",
                str(assets / "worlds/warehouse_small/warehouse_small.usda"),
                "--ambient-temperature",
                "293.15",
                "--no-ros",
                "--listen",
                f"unix:{socket}",
            ],
            stdout=out,
            stderr=subprocess.STDOUT,
        )
    for _ in range(300):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    yield socket, tmp_path, log
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=30)
    Path(socket).unlink(missing_ok=True)


def run_viewer(socket: str, home: Path, *extra: str, frames: int = 90) -> dict:
    done = subprocess.run(
        [
            tool("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--connect",
            f"unix:{socket}",
            "--size",
            SIZE,
            "--frames",
            str(frames),
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


def read_png(path: Path):
    """Decode an 8-bit RGBA PNG without filtering (what the viewer writes)."""
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset, width, height, idat = 8, 0, 0, b""
    while offset < len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        kind = data[offset + 4 : offset + 8]
        body = data[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            width, height, depth, colour = struct.unpack(">IIBB", body[:10])
            assert (depth, colour) == (8, 6)
        elif kind == b"IDAT":
            idat += body
        offset += 12 + length
    raw = zlib.decompress(idat)
    stride = width * 4 + 1
    return [raw[y * stride + 1 : (y + 1) * stride] for y in range(height)]


def count(rows, box, test) -> int:
    x0, y0, x1, y1 = box
    found = 0
    for y in range(y0, y1):
        row = rows[y]
        for x in range(x0, x1):
            found += 1 if test(row[4 * x], row[4 * x + 1], row[4 * x + 2]) else 0
    return found


def accent(r, g, b) -> bool:
    return r > 180 and 80 < g < 160 and b < 90  # the accent #E8772E


def test_the_scene_workspace_streams_no_sensor_until_a_sensor_tab_is_opened(amr) -> None:
    socket, home, _ = amr
    report = run_viewer(socket, home)
    assert report["connected"]
    if report.get("shading") != 0:  # Shading::Realistic
        # A software device starts in Lit with the sensor overlays on: the viewport itself
        # draws the lidar's points, so frames come whatever the tabs do.
        pytest.skip("the viewport's sensor overlay streams on a software device")
    assert report["sensor_frames"] == 0  # the tabs exist, behind the viewport


@pytest.mark.parametrize("panel", ["###SceneLidar", "###SceneCamera"])
def test_a_sensor_tab_in_front_shows_its_sensor(amr, panel) -> None:
    socket, home, _ = amr
    report = run_viewer(socket, home, "--panel", panel, frames=150)
    assert report["connected"]
    assert report["sensor_frames"] > 0


def test_the_layers_tab_draws(amr) -> None:
    socket, home, _ = amr
    shot = home / "layers.png"
    report = run_viewer(socket, home, "--panel", "Layers", "--screenshot", str(shot))
    assert report["connected"]
    rows = read_png(shot)
    # The session layer row is selected at first: its accent bar and "edit target".
    assert count(rows, (8, 160, 280, 190), accent) > 20


def test_an_inspector_edit_appears_in_the_history_tab(amr) -> None:
    socket, home, log = amr
    shot = home / "history.png"
    # Select the motor, type a new current limit, Apply, then open the History tab.
    script = ";".join(
        [
            "40:click:1460,694",  # the Inspector's maxCurrent field (below gearBackEfficiency)
            "42:key:ctrl+a",
            "44:text:10",
            "46:key:enter",
            "60:click:1557,957",  # Apply
            "100:click:1347,115",  # the History tab
        ]
    )
    report = run_viewer(
        socket,
        home,
        "--select",
        MOTOR,
        "--input",
        script,
        "--screenshot",
        str(shot),
        frames=150,
    )
    assert report["connected"]
    assert f"{MOTOR} elec:maxCurrent 8 -> 10 A" in log.read_text()
    rows = read_png(shot)
    # One change in the list: its accent mark at the row's left.
    assert count(rows, (1266, 168, 1290, 190), accent) > 10
