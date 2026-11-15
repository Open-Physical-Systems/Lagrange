"""The Performance workspace's six KPI cards and its Threads, GPU passes and Memory tabs
(docs/design/ui/11-performance), headless against a real lagrange_server: the sample AMR and
the sedan in the city block."""

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

GPU = os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe")


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


def read_png(path: Path):
    """Decode an 8-bit RGBA PNG without filtering (what the viewer writes)."""
    import struct
    import zlib

    data = path.read_bytes()
    offset, width, height, idat = 8, 0, 0, b""
    while offset < len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        kind = data[offset + 4 : offset + 8]
        body = data[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            width, height = struct.unpack(">II", body[:8])
        elif kind == b"IDAT":
            idat += body
        offset += 12 + length
    raw = zlib.decompress(idat)
    stride = width * 4 + 1
    return width, height, [raw[y * stride + 1 : (y + 1) * stride] for y in range(height)]


@pytest.fixture(scope="module", params=["amr", "sedan"])
def server(request, tmp_path_factory):
    """A real server: the sample AMR (its default run) or the sedan in the city block."""
    try:
        assets = get_package_share_directory("lagrange_assets")
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_perf_{os.getpid()}_{request.param}.sock"
    scene = (
        []
        if request.param == "amr"
        else [
            "--robot",
            f"{assets}/robots/sedan/sedan.usda",
            "--world",
            f"{assets}/worlds/city_block/city_block.usda",
        ]
    )
    process = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            *scene,
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
    yield socket, tmp_path_factory.mktemp(request.param)
    process.send_signal(signal.SIGTERM)
    try:
        process.wait(15)
    except subprocess.TimeoutExpired:
        process.kill()
    Path(socket).unlink(missing_ok=True)


def run_performance(socket: str, tmp_path: Path, panel: str | None) -> tuple[dict, Path]:
    image = tmp_path / f"performance_{(panel or 'default').strip('#').replace(' ', '_')}.png"
    extra = ["--panel", panel] if panel else []
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
            "200",  # long enough for two of the once-a-second statistics
            "--wait-ms",
            "60000",
            "--workspace",
            "performance",
            "--screenshot",
            str(image),
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=240,
        check=False,
        env={**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")},
    )
    assert done.returncode == 0, done.stderr[-2000:]
    report = json.loads(done.stdout.strip().splitlines()[-1])
    assert report["validation_errors"] == 0, done.stderr[-2000:]
    assert report["server_stats"] is True
    return report, image


def test_the_kpi_row_has_six_cards_and_the_default_tabs(server) -> None:
    socket, tmp_path = server
    report, image = run_performance(socket, tmp_path, None)
    windows = report["windows"]
    # Scheduler and Modules are in front by default; the other tabs are behind them.
    assert "Overview" in windows and "Scheduler · last 10 ms" in windows
    assert "Modules" in windows and "Threads" not in windows
    width, _height, rows = read_png(image)
    # Six cards: below their detail line (y = 182) the row is runs of the cards' fill, one per
    # card, each wider than 100 px (a sixth of 1600 px less the gaps).
    row = rows[182]
    line = [tuple(row[i : i + 3]) for i in range(0, width * 4, 4)]
    fill = line[30]
    runs, length = [], 0
    for pixel in line:
        if pixel == fill:
            length += 1
        else:
            if length > 100:
                runs.append(length)
            length = 0
    if length > 100:
        runs.append(length)
    assert len(runs) == 6, f"cards along the KPI row: {runs}"


def test_threads_tab_lists_the_viewers_threads(server) -> None:
    socket, tmp_path = server
    # --panel takes the tab's visible title as well as its ImGui id (###PerformanceThreads).
    report, _image = run_performance(socket, tmp_path, "Threads")
    assert "Threads" in report["windows"]
    assert "Scheduler · last 10 ms" not in report["windows"]


def test_gpu_passes_tab_times_the_viewers_own_passes(server) -> None:
    socket, tmp_path = server
    report, _image = run_performance(socket, tmp_path, "GPU passes")
    assert "GPU passes" in report["windows"] and "Modules" not in report["windows"]
    gpu = report["viewer_gpu"]
    if not gpu["timed"]:
        pytest.skip(f"{GPU} reports no timestamps")
    passes = gpu["passes"]
    assert set(passes) == {"sensor images upload", "3D view", "panels (ImGui)"}
    assert passes["panels (ImGui)"]["frames"] == gpu["window"] > 0
    assert passes["panels (ImGui)"]["mean_ms"] > 0.0
    assert 0 < passes["3D view"]["frames"] <= gpu["window"]
    for timing in passes.values():
        assert 0.0 <= timing["mean_ms"] <= timing["max_ms"] < 1000.0


def test_memory_tab_shows_the_server_and_the_viewer(server) -> None:
    socket, tmp_path = server
    report, _image = run_performance(socket, tmp_path, "###PerformanceMemory")
    assert "Memory" in report["windows"] and "Modules" not in report["windows"]
    gpu = report["viewer_gpu"]
    if gpu["vram_known"]:
        assert gpu["vram_used"] > 0
