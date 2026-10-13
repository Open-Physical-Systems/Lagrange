"""The viewer binary, headless, against the Python mock server.

This is the first test in which the C++ client and the Python server talk over a real socket.
It runs on the software Vulkan device (llvmpipe), which every CI runner has. Set
LAGRANGE_TEST_GPU to run it on another device, e.g. LAGRANGE_TEST_GPU=NVIDIA.
"""

import asyncio
import json
import os
import struct
import subprocess
import tempfile
import threading
import zlib
from pathlib import Path

import pytest
from lagrange_mock_server.endpoint import Endpoint
from lagrange_mock_server.replay import ReplayEngine
from lagrange_mock_server.sample import generate_sample
from lagrange_mock_server.server import MockServer

VIEWER = os.environ.get("LAGRANGE_VIEWER", "lagrange_viewer")
GPU = os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe")


class ServerThread:
    """Runs a MockServer on its own event loop in a background thread."""

    def __init__(self, socket_path: str) -> None:
        self.endpoint = Endpoint("unix", socket_path)
        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._stop: asyncio.Event | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self) -> None:
        engine = ReplayEngine(generate_sample(duration_s=2.0, seed=1))
        engine.play()
        server = MockServer(engine, self.endpoint)
        await server.start()
        self._stop = asyncio.Event()
        self._ready.set()
        await self._stop.wait()
        await server.stop()

    def __enter__(self):
        self._thread.start()
        assert self._ready.wait(10), "mock server did not start"
        return self

    def __exit__(self, *_):
        self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(10)


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
    rows = [raw[y * stride + 1 : (y + 1) * stride] for y in range(height)]
    assert all(raw[y * stride] == 0 for y in range(height))
    return width, height, rows


def run_viewer(*args, timeout=120):
    return subprocess.run(
        [VIEWER, "--headless", "--gpu", GPU, "--validation", *args],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


@pytest.fixture
def socket_dir():
    # Unix socket paths are limited to about 100 characters, so not pytest's tmp_path.
    with tempfile.TemporaryDirectory(prefix="lgr-") as directory:
        yield Path(directory)


def test_viewer_renders_the_scene_from_the_mock_server(socket_dir):
    image = socket_dir / "frame.png"
    with ServerThread(str(socket_dir / "s.sock")) as server:
        result = run_viewer(
            "--connect",
            f"unix:{server.endpoint.address}",
            "--frames",
            "40",
            "--size",
            "1400x1000",
            "--screenshot",
            str(image),
        )
    assert result.returncode == 0, result.stderr
    stats = json.loads(result.stdout.strip().splitlines()[-1])
    assert stats["connected"] is True
    assert stats["frames"] == 40
    assert stats["snapshots"] >= 1
    assert stats["deltas"] >= 5  # the scene kept moving while we rendered
    assert stats["prims"] == 11 and stats["meshes"] == 4
    assert stats["draw_calls"] == 5  # base, two wheels, caster, ground
    assert stats["sim_time_ns"] > 0
    assert stats["validation_errors"] == 0, result.stderr

    width, height, rows = read_png(image)
    assert (width, height) == (1400, 1000)
    pixels = [row[i : i + 4] for row in rows for i in range(0, len(row), 4)]
    assert len(set(pixels)) > 50, "the image is nearly uniform: nothing was drawn"
    # The robot body is orange (base colour 0.85, 0.45, 0.10): clearly red > green > blue.
    orange = sum(1 for r, g, b, _a in pixels if r > 130 and r > g + 40 and g > b + 20)
    # The viewport shares the window with the other panels, so the robot is small.
    assert orange > 150, f"robot body not visible ({orange} orange pixels)"
    assert all(p[3] == 255 for p in pixels[:: len(pixels) // 500]), "image is not opaque"


SCOPE_SIGNALS = [
    "motor/left_drive_motor/current_q",  # 10 kHz
    "motor/right_drive_motor/current_q",
    "bus/dc_48v/voltage",
    "battery/battery/current",
    "joint/left_wheel_joint/velocity",  # 1 kHz
    "thermal/left_drive_motor/temperature",  # 10 Hz
]


def test_scope_receives_every_sample_of_high_rate_signals(socket_dir):
    image = socket_dir / "scope.png"
    with ServerThread(str(socket_dir / "s.sock")) as server:
        result = run_viewer(
            "--connect",
            f"unix:{server.endpoint.address}",
            "--frames",
            "120",
            "--size",
            "1280x900",
            "--scope",
            ",".join(SCOPE_SIGNALS),
            "--screenshot",
            str(image),
        )
    assert result.returncode == 0, result.stderr
    stats = json.loads(result.stdout.strip().splitlines()[-1])
    assert stats["validation_errors"] == 0, result.stderr
    assert stats["invalid_runs"] == 0
    assert stats["dropped_samples"] == 0, "the scope lost samples"
    assert stats["signal_blocks"] > 10
    # 120 frames take about a second of replay: four 10 kHz signals alone give tens of
    # thousands of samples. Far fewer would mean the subscription did not work.
    assert stats["samples"] > 15000

    _width, _height, rows = read_png(image)
    pixels = [row[i : i + 4] for row in rows for i in range(0, len(row), 4)]
    # The scope draws traces in the plot colour map; its first colour is a clear blue.
    lower_half = pixels[len(pixels) // 2 :]
    blue = sum(1 for r, g, b, _a in lower_half if b > 150 and b > r + 50 and b > g + 20)
    assert blue > 200, f"no scope trace visible ({blue} blue pixels)"


def test_unknown_scope_signal_is_ignored(socket_dir):
    with ServerThread(str(socket_dir / "s.sock")) as server:
        result = run_viewer(
            "--connect",
            f"unix:{server.endpoint.address}",
            "--frames",
            "20",
            "--scope",
            "no/such/signal,bus/dc_48v/voltage",
        )
    assert result.returncode == 0, result.stderr
    stats = json.loads(result.stdout.strip().splitlines()[-1])
    assert stats["samples"] > 0 and stats["invalid_runs"] == 0


def test_viewer_reports_a_missing_server_and_exits(socket_dir):
    result = run_viewer(
        "--connect", f"unix:{socket_dir}/nobody.sock", "--frames", "5", "--wait-ms", "600"
    )
    assert result.returncode == 4
    assert "no scene" in result.stderr


def test_viewer_rejects_bad_arguments():
    assert run_viewer("--connect", "http://example").returncode == 2
    assert run_viewer("--size", "10x10").returncode == 2
    assert run_viewer("--no-such-option").returncode == 2


def test_unknown_gpu_is_an_error_with_a_message():
    result = subprocess.run(
        [VIEWER, "--headless", "--gpu", "no-such-gpu", "--frames", "1"],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 3
    assert "no-such-gpu" in result.stderr
