"""Replay workspace (docs/design/ui/08-replay): Compare, Sim vs real and Validation against
real recordings.

Two runs of the sample AMR are recorded by lagrange_server (MuJoCo, then Jolt: the project has
no hardware logs, so the "real" log is a second simulated run), the first is replayed by
lagrange_server --replay, and the headless viewer compares it with the second.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

VIEWER = os.environ.get("LAGRANGE_VIEWER", "lagrange_viewer")
GPU = os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe")
RECORD_SECONDS = 4.0


def server_program() -> str:
    try:
        path = Path(get_package_prefix("lagrange_server")) / "lib/lagrange_server/lagrange_server"
    except PackageNotFoundError:
        pytest.skip("lagrange_server is not installed")
    return str(path)


def wait_for(path: Path, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while not path.exists():
        assert time.monotonic() < deadline, f"{path} did not appear"
        time.sleep(0.1)


def record(directory: Path, physics: str) -> Path:
    out = directory / f"amr_{physics}.mcap"
    sock = directory / f"{physics}.sock"
    server = subprocess.Popen(
        [
            server_program(),
            "--no-ros",
            "--lidar",
            "cpu",
            "--physics",
            physics,
            "--listen",
            f"unix:{sock}",
            "--record",
            str(out),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        wait_for(sock, 30)
        time.sleep(RECORD_SECONDS)
    finally:
        server.send_signal(signal.SIGTERM)
        server.wait(30)
    assert out.stat().st_size > 0
    return out


@pytest.fixture(scope="module")
def runs():
    # Unix socket paths are limited to about 100 characters, so not pytest's tmp_path.
    with tempfile.TemporaryDirectory(prefix="lgr-cmp-") as name:
        directory = Path(name)
        yield directory, record(directory, "mujoco"), record(directory, "jolt")


def view(directory: Path, replayed: Path, *args: str):
    sock = directory / "replay.sock"
    if sock.exists():
        sock.unlink()
    server = subprocess.Popen(
        [server_program(), "--replay", str(replayed), "--listen", f"unix:{sock}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        wait_for(sock, 30)
        result = subprocess.run(
            [
                VIEWER,
                "--headless",
                "--gpu",
                GPU,
                "--connect",
                f"unix:{sock}",
                "--frames",
                "40",
                "--size",
                "1600x1000",
                "--workspace",
                "replay",
                *args,
            ],
            capture_output=True,
            text=True,
            timeout=240,
            check=False,
        )
    finally:
        server.send_signal(signal.SIGTERM)
        server.wait(30)
    assert result.returncode == 0, result.stderr
    stats = json.loads(result.stdout.strip().splitlines()[-1])
    assert stats["validation_errors"] == 0, result.stderr
    return stats


def test_without_a_second_log_the_panels_offer_one(runs):
    directory, mujoco, _jolt = runs
    stats = view(directory, mujoco)
    assert stats["replay_compare"] == {"state": "none"}
    assert "Sim vs real" in stats["windows"]
    assert "Validation" in stats["windows"]


def test_a_second_engine_compared_signal_by_signal(runs):
    directory, mujoco, jolt = runs
    image = directory / "compare.png"
    stats = view(directory, mujoco, "--compare", str(jolt), "--screenshot", str(image))
    compare = stats["replay_compare"]
    assert compare["state"] == "ready", compare
    assert compare["suite"] == "shared signals"
    # Joint, motor, battery and bus signals change in both runs.
    assert compare["total"] >= 20
    assert 0 <= compare["passed"] <= compare["total"]
    assert compare["selected"]
    assert image.stat().st_size > 0


def test_the_same_run_against_itself_passes_with_no_lag(runs):
    directory, mujoco, _jolt = runs
    stats = view(directory, mujoco, "--compare", str(mujoco))
    compare = stats["replay_compare"]
    assert compare["state"] == "ready", compare
    assert compare["passed"] == compare["total"] > 0
    assert compare["nrmse"] == 0
    assert compare["lag_s"] == 0


def test_a_suite_file_picks_the_signals_and_limits(runs):
    directory, mujoco, jolt = runs
    suite = directory / "amr_drive.yaml"
    suite.write_text(
        "suite: amr_drive\n"
        "method: {alignment: cross-correlation, resample_hz: 50}\n"
        "signals:\n"
        "  - {sim: motor/left_drive_motor/current_q, limit: 0.05}\n"
        "  - {sim: motor/right_drive_motor/current_q, limit: 0.05}\n"
        "  - {sim: bus/dc_48v/voltage, limit: 0.02}\n"
        "  - {sim: body/base_link/twist, component: 0, limit: 0.05}\n"
    )
    stats = view(directory, mujoco, "--compare", str(jolt), "--suite", str(suite))
    compare = stats["replay_compare"]
    assert compare["state"] == "ready", compare
    assert compare["suite"] == "amr_drive"
    assert compare["total"] == 4


def test_a_missing_log_is_an_error_not_a_crash(runs):
    directory, mujoco, _jolt = runs
    stats = view(directory, mujoco, "--compare", str(directory / "missing.mcap"))
    assert stats["replay_compare"]["state"] == "error"
    assert "no such file" in stats["replay_compare"]["error"]
